"""옆선 켜짐 확인(side_rise_only, FFT1-163 이어서): 옆선 노드는 그 옆 IR 이 감지 창 시작 뒤 꺼짐 → 켜짐일 때만.

근거(실로봇 d660 0.08 m/s, 2026-11-09 21:54 N02→N14): 회전 직후 오른쪽으로 밀려 왼쪽 IR 이 8.0 cm 부터 본선
가장자리를 밟음(raw 650~1300) → 창(14.8 cm)이 열리자 15.1 cm 오감지 → 3 cm 이른 회전 → line_not_found.
진짜 옆선은 18.3 cm 에서 꺼짐 → 켜짐(raw 1005 → 238). 2D 시뮬은 실로봇 검증을 대신하지 않는다.
"""

import csv
import math
import os

import pytest
from sim import Sim

from roport_line_nav.executor import ADVANCE, ARRIVED, ERROR, FOLLOW, NavConfig, RouteExecutor
from roport_line_nav.graph import Graph
from roport_pinky_line.line_sensor import LineReading

HERE = os.path.dirname(__file__)
LAYOUT = os.path.join(HERE, '..', '..', 'roport_gz_world', 'layout', 'roport_airport.yaml')
FIXTURE = os.path.join(HERE, 'data', 'd660_20261109_215338_N02-N14.csv')
ROUTE = ['N01', 'N18', 'N10', 'N02', 'N14', 'N16']     # 실로그와 같은 경로(P4 → P5)
LEG = 3                                               # N02 → N14(왼쪽 옆선 N16, T자 아님)


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def _reading(on, error=0.0):
    n = tuple(1.0 if o else 0.0 for o in on)
    return LineReading(raw=(0, 0, 0), n=n, on_line=tuple(on), error=error, lost=not any(on),
                       node=all(on))


def _at_leg(graph, cfg, pose, k=LEG, s0=3.0):
    """실행부를 경로의 k 번 구간 시작(회전 직후, 센서 = 노드 + s0)으로 맞춤."""
    ex = RouteExecutor(graph, cfg)
    ex.start(ROUTE)
    ex._steps.clear()
    ex._pre = None
    ex._update_odom(pose)
    ex.index = k
    ex._begin_leg(s0)
    return ex


# ---------------------------------------------------------------- 실로그 재생(개루프)
def _replay_fixture(graph, cfg):
    """기록된 IR 판정(on0..2, error)·odom 을 그대로 넣음. 감지하면 (누적 거리, 왼쪽 IR run 시작) 반환."""
    with open(FIXTURE) as f:
        rows = list(csv.DictReader(f))

    def pose(r):
        return (float(r['odom_x_cm']), float(r['odom_y_cm']), math.radians(float(r['odom_yaw_deg'])))

    ex = _at_leg(graph, cfg, pose(rows[0]))
    cum, last = 0.0, pose(rows[0])
    for r in rows:
        p = pose(r)
        cum += math.hypot(p[0] - last[0], p[1] - last[1])
        last = p
        run = dict(ex._side_run[0])
        ex.step(_reading(tuple(r[k] == '1' for k in ('on0', 'on1', 'on2')), float(r['error'])),
                p, float(r['t']))
        if ex.state != FOLLOW:
            return round(cum, 1), (run['start'] if run['on'] else cum), ex
    return None, None, ex


def test_real_log_n02_n14_false_detection_fixed(graph):
    """실로그 개루프 재생: 예전 규칙은 기록과 같은 15.4 cm 오감지, 새 규칙은 18.5 cm(진짜 옆선 run 18.3 cm)."""
    d_old, run_old, ex_old = _replay_fixture(graph, NavConfig(side_rise_only=False))
    assert d_old == pytest.approx(15.4, abs=0.2) and run_old == pytest.approx(8.0, abs=0.2)
    d_new, run_new, ex_new = _replay_fixture(graph, NavConfig())
    win = ex_new._min_ok if ex_new.state == FOLLOW else 14.8
    assert d_new == pytest.approx(18.5, abs=0.2), d_new
    assert run_new == pytest.approx(18.3, abs=0.2) and run_new >= win
    ign = [e['detail'] for e in ex_new.events if e['event'] == 'side_ignored']
    assert ign and 'left on since 8.0 cm' in ign[0]
    assert ex_new.state == ADVANCE                                   # 평소처럼 회전 전 전진


# ---------------------------------------------------------------- 가짜 입력
LEFT = (True, True, False)
CENTER = (False, True, False)


def _drive(ex, pose, pattern_at, until_cm, step=0.3, t0=0.0):
    """odom 이 +x 로 step 씩, pattern_at(d) 패턴. 감지하거나 until_cm 넘으면 멈춤 → 감지 거리 또는 None."""
    x, y, yaw = pose
    t = t0
    d = 0.0
    while d <= until_cm:
        ex.step(_reading(pattern_at(d), -0.5 if pattern_at(d)[0] else 0.0), (x, y, yaw), t)
        if ex.state != FOLLOW:
            return d
        x, d, t = x + step, d + step, t + 0.05
    return None


def _ex(graph, **cfg):
    n02 = graph.nodes['N02']
    return _at_leg(graph, NavConfig(**cfg), (n02.x, n02.y, 0.0)), (n02.x, n02.y, 0.0)


def test_edge_run_then_true_side_line(graph):
    """가장자리 run(8~16 cm) → 꺼짐 → 진짜 옆선(18.3 cm~): 새 규칙은 18.3 cm 대, 예전은 창 시작 14.8 cm."""
    def pat(d):
        return LEFT if 8.0 <= d < 16.0 or d >= 18.3 else CENTER
    ex, pose = _ex(graph, side_rise_only=False)
    win = ex._min_ok
    assert _drive(ex, pose, pat, 30) == pytest.approx(win + 0.3, abs=0.35)      # 창 시작에서 오감지
    ex, pose = _ex(graph)
    d = _drive(ex, pose, pat, 30)
    assert d is not None and 18.3 <= d <= 19.0


def test_edge_run_merging_into_side_line_is_not_a_node(graph):
    """가장자리 run 이 꺼지지 않고 진짜 옆선으로 이어지면(11-07 16:57·17:55 실로그 모양) 노드로 보지 않음
    → 예상 거리(max_ok)를 넘어 node_missed 정지(오감지·추측 회전 대신 안전 정지)."""
    ex, pose = _ex(graph)
    d = _drive(ex, pose, lambda d: LEFT if 11.0 <= d < 21.0 else CENTER, 40)
    assert d is not None and ex.state == ERROR and 'node_missed' in ex.reason
    assert not [e for e in ex.events if e['event'] == 'node']


def test_flicker_shorter_than_off_samples_keeps_run(graph):
    """side_rise_off_samples(2) 보다 짧은 꺼짐(1 샘플)은 같은 run → 무시. 2 샘플 꺼지면 새 run → 감지."""
    ex, pose = _ex(graph)
    one = _drive(ex, pose, lambda d: CENTER if 16.4 < d < 16.7 else (LEFT if d >= 8.0 else CENTER), 25)
    assert one is None or ex.state == ERROR                     # 1 샘플 꺼짐: 감지 없음
    ex, pose = _ex(graph)
    two = _drive(ex, pose, lambda d: CENTER if 16.4 < d < 17.0 else (LEFT if d >= 8.0 else CENTER), 25)
    assert two is not None and two >= 17.0 and ex.state == ADVANCE


def test_run_starting_inside_window_detects_immediately(graph):
    """창 안에서 처음 켜진 옆선은 예전과 같은 거리에서 감지(node_confirm 2 샘플)."""
    for rise in (True, False):
        ex, pose = _ex(graph, side_rise_only=rise)
        assert _drive(ex, pose, lambda d: LEFT if d >= 18.0 else CENTER, 30) == pytest.approx(18.3, abs=0.35)


def test_tee_unchanged_by_default(graph):
    """T자(tee_both)는 기본으로 켜짐 확인을 쓰지 않음: 한쪽 가장자리를 타도 반대쪽이 함께 켜지는 가로선에서 감지."""
    ex = RouteExecutor(graph)
    ex.start(['N16', 'N14', 'N15', 'N17'])
    assert ex.legs[0].sides == {'left', 'right'}
    n16 = graph.nodes['N16']
    ex._update_odom((n16.x, n16.y, -math.pi / 2))
    x, y, t, d = n16.x, n16.y, 0.0, 0.0
    hit = None
    while d < 30:
        on = (True, True, True) if d >= 23.0 else ((True, True, False) if d >= 10.0 else CENTER)
        ex.step(_reading(on), (x, y, -math.pi / 2), t)
        if ex.state != FOLLOW:
            hit = d
            break
        y, d, t = y - 0.3, d + 0.3, t + 0.05
    assert hit is not None and 23.0 <= hit <= 23.7


def test_side_rise_config():
    assert NavConfig().side_rise_only and NavConfig().side_rise_off_samples == 2
    assert not NavConfig().side_rise_tee
    for bad in (0, 11):
        with pytest.raises(ValueError):
            NavConfig(side_rise_off_samples=bad)


# ---------------------------------------------------------------- 2D 시뮬: 결함 주입으로 가장자리 타기 재현
# 2D 시뮬의 PD 는 실로봇보다 훨씬 빨리 선에 맞춰져서(README) 본선 옆 띠를 붙여도 가장자리를 계속 타지 않는다.
# 그래서 물리·조향(오차)은 그대로 두고 실행부가 받는 왼쪽 IR 판정만 8~16 cm 동안 켜짐으로 덮어쓴다
# (실로그에서 실행부가 받은 입력과 같은 모양: 가운데 + 왼쪽 켜짐이 창 시작 전부터 이어짐).
SPEEDS = (0.03, 0.06, 0.08)
DELAYS = (0.0, 0.15, 0.2)
EDGE_ROUTE = ['N10', 'N02', 'N14', 'N16']        # N10 교차선 출발 → N02 왼쪽 회전 → N14 왼쪽 옆선 → P5


def _run_edge(graph, speed, delay, rise, inject=True):
    from dataclasses import replace
    sim = Sim(graph, cmd_delay_s=delay)
    sim.place('N10', 'N', 0.0)
    ex = RouteExecutor(graph, NavConfig(base_speed=speed, max_linear=max(0.06, speed),
                                        side_rise_only=rise))
    ex.start(EDGE_ROUTE, start_cross=True, start_facing='N')
    real = sim.sensor.update

    def update(raw):
        r = real(raw)
        d = ex._odo - ex._ref_odo
        if inject and ex.index == 1 and ex.state == FOLLOW and 8.0 <= d < 16.0 and r.on_line[1]:
            on = (True, r.on_line[1], r.on_line[2])
            r = replace(r, on_line=on, lost=False, node=all(on))
        return r

    sim.sensor.update = update
    det = None
    while ex.active and sim.t < 300:
        before = (ex.index, ex.state, ex._odo - ex._ref_odo)
        sim.tick(ex)
        if before[0] == 1 and before[1] == FOLLOW and ex.state != FOLLOW and det is None:
            det = before[2]
    return ex, det


@pytest.mark.parametrize('delay', DELAYS)
@pytest.mark.parametrize('speed', SPEEDS)
def test_sim_edge_riding_no_false_detection(graph, speed, delay):
    """왼쪽 IR 이 창 전(8 cm)부터 켜져 있으면 예전 규칙은 창 시작(14.8 cm)에서 오감지 → 회전에서 선 못 찾음
    (실로봇 11-09 21:54 와 같은 결과). 새 규칙은 진짜 옆선에서 감지하고 P5 까지 완주(주입 없을 때와 같은 위치)."""
    old, old_det = _run_edge(graph, speed, delay, rise=False)
    assert old.state == ERROR and 'line_not_found' in old.reason and old_det < 15.5, (old.reason, old_det)
    new, new_det = _run_edge(graph, speed, delay, rise=True)
    assert new.state == ARRIVED, new.as_dict()
    assert any(e['event'] == 'side_ignored' and 'N14 left' in e['detail'] for e in new.events)
    ref, ref_det = _run_edge(graph, speed, delay, rise=True, inject=False)
    assert ref.state == ARRIVED and new_det == pytest.approx(ref_det, abs=0.01) and new_det > 19.0
