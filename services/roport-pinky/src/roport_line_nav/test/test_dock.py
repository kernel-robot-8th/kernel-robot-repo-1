"""정밀 도킹 값(FFT1-112)과 교차선 출발(start_cross). 2D 시뮬(test/sim.py)로만 확인한다(실로봇 미검증).

목표 자세 정의는 dock_zone(Core.predictedDock·plan)과 같다:
  예상 정지 자세 = 노드 중심에서 접근 반대 방향으로 (half_line + ir_ahead − 밀림), 방향 = 접근 + 90°
  목표 = 예상 + a·접근 방향 + b·(접근 + r) 방향, 방향 = 접근 + r
밀림(구동 지연·감지 확인 샘플로 가로선을 더 지나간 거리)은 같은 조건의 기본값(0, 90, 0) 주행에서 잰다.
"""

import math
import os

import pytest
from sim import Sim

from roport_line_nav.dock import (DEFAULT, DockTable, DockValue, load_file, load_nodes,
                                  save_file)
from roport_line_nav.executor import (ADVANCE, ARRIVED, ERROR, LOADING, NavConfig,
                                      RouteExecutor, STOPPED)
from roport_line_nav.graph import Graph, GraphError, plan_route

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')
TO_N01 = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']                      # P5 → P4
TO_N11 = ['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11']        # P5 → P7
DOCKS = {'N11': (2.0, 96.0, 2.0), 'N01': (-2.0, 84.0, -3.0)}             # 작업 지시 예시 값
DOCKS_REQ = {'N01': (3.7, 90.4, 2.6), 'N11': (-4.6, 88.8, 2.8)}         # 2026-11-09 실측 요청값
DOCK_SETS = {'example': DOCKS, 'request': DOCKS_REQ}
SPEEDS = (0.03, 0.06, 0.08)
DELAYS = (0.0, 0.15, 0.2)
POS_TOL_CM = 1.0
HEAD_TOL_DEG = 3.0


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def _cfg(speed):
    return NavConfig(base_speed=speed, max_linear=max(0.06, speed))


def _done(ex):
    return not ex.active


def _drive_all(sim, ex, max_s=900.0, load=True):
    """끝까지 달리며 이벤트를 모두 모은다(ex.events 는 최근 64개만)."""
    events, seen = [], ex.event_seq
    events.extend(list(ex.events)[len(ex.events) - seen:] if seen else [])
    while ex.active and sim.t < max_s:
        sim.tick(ex)
        if load and ex.state == LOADING:
            ex.load_done()
        new = ex.event_seq - seen
        if new:
            events.extend(list(ex.events)[-new:])
            seen = ex.event_seq
    return events


def _is_zero(cmd):
    return cmd is not None and cmd.linear == 0.0 and cmd.angular == 0.0


def _ang(d):
    return (d + 180.0) % 360.0 - 180.0


def _run_to_loading(graph, route, speed, delay, dock=None, settle_ticks=30):
    """route 를 달려 LOADING 이 settle_ticks(1.5 s) 이어진 뒤의 실제 축 자세. 그 뒤 load_done 으로 끝까지."""
    sim = Sim(graph, cmd_delay_s=delay)
    sim.place(route[0], 'S', 0.0)
    ex = RouteExecutor(graph, _cfg(speed))
    for node, v in (dock or {}).items():
        ex.set_dock(node, *v)
    ex.start(route)
    got = {}

    def on_loading(s):
        got['n'] = got.get('n', 0) + 1
        if got['n'] == settle_ticks:
            got['pose'] = (s.x, s.y, s.yaw)
            got['est'] = ex.as_dict()['pose_est']
            return True
        return False

    sim.run(ex, _done, on_loading=on_loading)
    return sim, ex, got


def _approach(graph, node):
    n = graph.nodes[node]
    prev = graph.nodes[n.links[0]]          # 상차 노드는 이웃 1개
    return n, math.atan2(n.y - prev.y, n.x - prev.x)


def _target(graph, node, over, a, r, b, cfg):
    """dock_zone Core.predictedDock + plan 과 같은 정의의 목표 축 자세 (x, y, heading rad)."""
    n, th = _approach(graph, node)
    back = cfg.half_line_cm + cfg.ir_ahead_cm - over
    px, py = n.x - math.cos(th) * back, n.y - math.sin(th) * back
    h = th + math.radians(r)
    return px + a * math.cos(th) + b * math.cos(h), py + a * math.sin(th) + b * math.sin(h), h


# ---------------------------------------------------------------- 기본값 = 지금 동작
def test_default_dock_is_identical(graph):
    """파일에 없음({0, 90, 0})과 명시한 {0, 90, 0}이 같은 궤적·이벤트(회귀 없음)."""
    runs = []
    for explicit in (False, True):
        sim = Sim(graph, cmd_delay_s=0.2)
        sim.place('N16', 'S', 0.0)
        ex = RouteExecutor(graph)
        if explicit:
            ex.set_dock('N01', 0, 90, 0)
            ex.events.clear()
        ex.start(TO_N01 + ['N18', 'N07', 'N23', 'N19', 'N11'])
        poses = []
        while ex.active and sim.t < 900:
            sim.tick(ex)
            poses.append((round(sim.x, 6), round(sim.y, 6), round(sim.yaw, 6)))
            if ex.state == LOADING:
                ex.load_done()
        runs.append((ex.state, poses, [(e['event'], e['detail']) for e in ex.events]))
        assert ex.as_dict()['dock'] == {}
    assert runs[0] == runs[1]
    assert runs[0][0] == ARRIVED
    assert not [e for e in runs[0][2] if e[0] == 'advance' and "'a'" in e[1]]


# ---------------------------------------------------------------- 목표 자세 ±1 cm·±3°
@pytest.mark.parametrize('delay', DELAYS)
@pytest.mark.parametrize('speed', SPEEDS)
@pytest.mark.parametrize('node, route', [('N11', TO_N11), ('N01', TO_N01)])
@pytest.mark.parametrize('dock_set', sorted(DOCK_SETS))
def test_dock_pose_at_loading(graph, dock_set, node, route, speed, delay):
    a, r, b = DOCK_SETS[dock_set][node]
    _, ex0, base = _run_to_loading(graph, route, speed, delay)
    assert ex0.state == ARRIVED and 'pose' in base
    n, th = _approach(graph, node)
    cfg = ex0.cfg
    x0, y0, _ = base['pose']
    over = (x0 - n.x) * math.cos(th) + (y0 - n.y) * math.sin(th) + cfg.half_line_cm + cfg.ir_ahead_cm
    assert 0.0 <= over <= 1.5, over                       # 감지 확인 샘플 + 구동 지연만큼

    _, ex, got = _run_to_loading(graph, route, speed, delay, {node: (a, r, b)})
    assert ex.state == ARRIVED, ex.as_dict()
    tx, ty, th_t = _target(graph, node, over, a, r, b, cfg)
    x, y, yaw = got['pose']
    err_cm = math.hypot(x - tx, y - ty)
    err_deg = _ang(math.degrees(yaw - th_t))
    assert err_cm <= POS_TOL_CM, (err_cm, err_deg)
    assert abs(err_deg) <= HEAD_TOL_DEG, (err_cm, err_deg)
    # 화면 추정(pose_est)은 밀림을 모를 뿐 같은 목표(밀림 0)를 가리킴
    ex_t = _target(graph, node, 0.0, a, r, b, cfg)
    est = got['est']
    assert math.hypot(est['x_cm'] - ex_t[0], est['y_cm'] - ex_t[1]) <= 0.15
    assert abs(_ang(est['heading_deg'] - math.degrees(ex_t[2]))) <= 0.1
    details = [e['detail'] for e in ex.events if e['event'] == 'advance']
    for phase in ('a', 'b', '-b', '-a'):                 # 상차 뒤 −b·시계 r°·−a 로 되돌아옴
        assert [d for d in details if d.endswith(f"'{phase}')")], (phase, details)


def test_dock_moves_respect_speed_and_sign(graph):
    """도킹 이동 명령: |선속도| ≤ approach_speed, a < 0·b < 0 은 후진, 회전 상한 안."""
    sim = Sim(graph, cmd_delay_s=0.15)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, _cfg(0.08))
    ex.set_dock('N01', -2.0, 84.0, -3.0)
    ex.start(TO_N01)
    seen = {}
    while ex.active and sim.t < 400:
        sim.tick(ex)
        if ex.state == ADVANCE and ex._prim.get('signed'):
            seen.setdefault(ex._prim['phase'], []).append(sim.cmd[0])
        if ex.state == LOADING:
            ex.load_done()
    assert ex.state == ARRIVED
    assert set(seen) == {'a', 'b', '-b', '-a'}
    for phase, lin in seen.items():
        assert all(abs(v) <= ex.approach_speed + 1e-9 for v in lin), phase
    assert min(seen['a']) < 0 and min(seen['b']) < 0 and max(seen['-b']) > 0 \
        and max(seen['-a']) > 0


# ---------------------------------------------------------------- 상차 뒤 선 다시 찾기·경유
@pytest.mark.parametrize('dock_set', sorted(DOCK_SETS))
@pytest.mark.parametrize('speed, delay', [(0.03, 0.0), (0.06, 0.2), (0.08, 0.15)])
@pytest.mark.parametrize('route', [
    TO_N01 + ['N18', 'N10', 'N02', 'N14', 'N16'],                          # P5 → P4 → P5
    TO_N11 + ['N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16'],            # P5 → P7 → P5
    TO_N01 + ['N18', 'N07', 'N23', 'N19', 'N11'],                          # P5 → P4 → P7
], ids=['P5-P4-P5', 'P5-P7-P5', 'P5-P4-P7'])
def test_via_after_dock(graph, route, speed, delay, dock_set):
    docks = DOCK_SETS[dock_set]
    sim = Sim(graph, cmd_delay_s=delay)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, _cfg(speed))
    for node, v in docks.items():
        ex.set_dock(node, *v)
    ex.start(route)
    events = _drive_all(sim, ex)
    assert ex.state == ARRIVED, ex.as_dict()
    kinds = [e['event'] for e in events]
    assert 'via' in kinds and kinds.count('load_done') == (2 if route[-1] == 'N11' else 1)
    # a ≠ 0: 상차 뒤 시계 r° → −a → 반시계 선 찾기(load_done 다음 line_found) = 90 + turn_min ~ 90 + turn_max
    via = 'N01' if route[:len(TO_N01)] == TO_N01 else 'N11'
    i = kinds.index('load_done')
    back = float(events[i + kinds[i:].index('line_found')]['detail'].rstrip('°'))
    c = ex.cfg
    assert 90 + c.turn_min_deg <= back <= 90 + c.turn_max_deg, back
    assert 'rotate' in kinds[i:] and "(-1.0" in [e['detail'] for e in events[i:]
                                                if e['event'] == 'rotate'][0]
    assert [e['detail'] for e in events if e['event'] == 'via'] == [via]
    if route[-1] == 'N16':
        assert 'reverse' in kinds
        n16 = graph.nodes['N16']
        assert math.hypot(sim.x - n16.x, sim.y - n16.y) < 8.0


def test_next_leg_s0_after_return(graph):
    """상차 뒤 −a 로 기본 정지 자세에 돌아오므로 다음 구간 s0 = 2·ir_ahead + half_line(a 와 무관)."""
    ex = RouteExecutor(graph)
    leg = plan_route(graph, TO_N11)[-1]
    c = ex.cfg
    for a in (2.0, -4.6, 0.0):
        ex.set_dock('N11', a, 96.0, 2.0)
        assert ex._end_of_leg(leg)['s0'] == pytest.approx(2 * c.ir_ahead_cm + c.half_line_cm)


def test_zero_advance_keeps_previous_return(graph):
    """a = 0 이면 상차 뒤 되돌림 없이 반시계 (180 − r) ± 30 으로 진입선 찾기(1차 구현 그대로)."""
    sim = Sim(graph, cmd_delay_s=0.15)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.set_dock('N11', 0.0, 100.0, 3.0)
    ex.start(TO_N11)
    events = _drive_all(sim, ex)
    assert ex.state == ARRIVED, ex.as_dict()
    kinds = [e['event'] for e in events]
    i = kinds.index('load_done')
    after = [e['detail'] for e in events[i:] if e['event'] in ('rotate', 'advance')]
    assert after[0].endswith("'-b')") and after[1].startswith('(1.0, 50.0, 110.0')
    assert not [d for d in after if d.endswith("'-a')") or d.endswith("'unr')")]


# 1차 구현(되돌림 없음)에서 a +5: 24회 중 5회, +6: 22회 line_not_found 로 정지하던 조합
LARGE_A_RUNS = [(0.03, 0.0), (0.03, 0.2), (0.06, 0.15), (0.08, 0.2)]


@pytest.mark.parametrize('a', [5.0, 6.0])
def test_large_advance_returns_then_finds_line(graph, a):
    """a = +5·+6: 상차 뒤 시계 r° → −a 로 기본 정지 자세에 돌아와 진입선을 찾고 P5 로 복귀(노드 2 × r 3 × 4)."""
    fails = []
    for node, route in (('N11', TO_N11 + ['N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16']),
                        ('N01', TO_N01 + ['N18', 'N10', 'N02', 'N14', 'N16'])):
        for r in (60.0, 90.0, 120.0):
            for speed, delay in LARGE_A_RUNS:
                sim = Sim(graph, cmd_delay_s=delay)
                sim.place('N16', 'S', 0.0)
                ex = RouteExecutor(graph, _cfg(speed))
                ex.set_dock(node, a, r, 0.0)
                ex.start(route)
                sim.run(ex, _done, max_s=900, on_loading=lambda s: True)
                if ex.state != ARRIVED:
                    fails.append((node, r, speed, delay, ex.reason))
    assert not fails, fails


@pytest.mark.parametrize('delay', DELAYS)
def test_extreme_dock_values_round_trip(graph, delay):
    """범위 끝 값(a ±6 × r 60·120 × b ±6) × 상차 노드 4개: 상차 뒤 진입선을 찾아 P5 로 복귀."""
    back = ['N05', 'N15', 'N14', 'N16']
    routes = {
        'N01': TO_N01 + ['N18', 'N10', 'N02', 'N14', 'N16'],
        'N11': TO_N11 + ['N19', 'N23', 'N07'] + back,
        'N21': TO_N11[:6] + ['N21', 'N23', 'N07'] + back,
        'N22': TO_N11[:7] + ['N24', 'N22', 'N24', 'N19', 'N23', 'N07'] + back,
    }
    fails = []
    for node, route in routes.items():
        for a in (-6.0, 6.0):
            for r in (60.0, 120.0):
                for b in (-6.0, 6.0):
                    sim = Sim(graph, cmd_delay_s=delay)
                    sim.place('N16', 'S', 0.0)
                    ex = RouteExecutor(graph, _cfg(0.06))
                    ex.set_dock(node, a, r, b)
                    ex.start(route)
                    sim.run(ex, _done, max_s=900, on_loading=lambda s: True)
                    n16 = graph.nodes['N16']
                    if ex.state != ARRIVED or math.hypot(sim.x - n16.x, sim.y - n16.y) >= 8.0:
                        fails.append((node, a, r, b, ex.state, ex.reason))
    assert not fails, fails


# ---------------------------------------------------------------- 정지 경로
def _to_dock_move(graph, phase='a'):
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.set_dock('N11', 3.0, 90.0, 3.0)
    ex.start(TO_N11)
    assert sim.run(ex, lambda e: e.state == ADVANCE and e._prim.get('phase') == phase)
    return sim, ex


def test_stop_during_dock_move(graph):
    sim, ex = _to_dock_move(graph)
    for _ in range(6):
        sim.tick(ex)
    assert ex.state == ADVANCE
    ex.stop()
    reading = sim.sensor.update(sim.raw())
    assert _is_zero(ex.step(reading, sim.pose(), sim.t))
    assert ex.state == STOPPED and ex.step(reading, sim.pose(), sim.t) is None


def test_dock_move_not_settling_fails(graph):
    """0 명령 뒤에도 odom 이 계속 움직이면(밀림·엔코더 이상) 도킹 이동을 시작하지 않고 오류 정지."""
    sim, ex = _to_dock_move(graph)
    reading = sim.sensor.update(sim.raw())
    x, y, yaw = sim.pose()
    t = sim.t
    cmd = None
    for i in range(60):
        cmd = ex.step(reading, (x, y - 0.3 * (i + 1), yaw), t + 0.05 * (i + 1))
        if not ex.active:
            break
    assert ex.state == ERROR and 'settle' in ex.reason and _is_zero(cmd)


def test_settle_counts_only_fresh_odom(graph):
    """멈춤 확인: 첫 샘플은 기준만, odom stamp 가 같은(갱신 안 된) 샘플은 세지 않고 새 샘플 2번 연속으로만 판정."""
    sim, ex = _to_dock_move(graph)
    p = ex._prim
    assert not p['settled'] and p['still'] == 0          # 들어온 샘플(같은 pose)로는 세지 않음
    reading = sim.sensor.update(sim.raw())
    x, y, yaw = sim.pose()
    t = sim.t

    def feed(pose, stamp, k):
        return ex.step(reading, pose, t + 0.05 * k, pose_stamp=stamp)

    feed((x, y, yaw), 1, 1)                              # 새 샘플, 멈춤 1
    assert p['still'] == 1
    for k in range(2, 8):                                # odom 이 안 바뀐 샘플(같은 stamp)은 세지 않음
        assert _is_zero(feed((x, y, yaw), 1, k))
    assert p['still'] == 1 and not p['settled']
    feed((x, y - 0.3, yaw), 2, 8)                        # 아직 밀리는 중 → 0 부터
    assert p['still'] == 0
    feed((x, y - 0.3, yaw), 3, 9)
    feed((x, y - 0.3, yaw), 3, 10)                       # 같은 stamp
    assert p['still'] == 1 and not p['settled']
    feed((x, y - 0.3, yaw), 4, 11)
    assert p['settled'] and p['origin'] == (x, y - 0.3)


def test_settle_with_stale_odom_times_out(graph):
    """odom 이 0 명령 뒤 한 번도 새로 안 오면 멈춤으로 보지 않고 1.5 s 뒤 오류 정지."""
    sim, ex = _to_dock_move(graph)
    reading = sim.sensor.update(sim.raw())
    pose, t = sim.pose(), sim.t
    cmd = None
    for k in range(1, 60):
        cmd = ex.step(reading, pose, t + 0.05 * k, pose_stamp=7)
        if not ex.active:
            break
    assert ex.state == ERROR and 'settle' in ex.reason and _is_zero(cmd)


def test_dock_move_no_progress_times_out(graph):
    """명령해도 움직이지 않으면(바퀴 걸림) 시간 초과로 정지."""
    sim, ex = _to_dock_move(graph, 'b')
    reading = sim.sensor.update(sim.raw())
    pose, t = sim.pose(), sim.t
    for i in range(400):
        ex.step(reading, pose, t + 0.05 * i)
        if not ex.active:
            break
    assert ex.state == ERROR and 'timeout' in ex.reason


def test_dock_move_wrong_way_fails(graph):
    """명령 반대쪽으로 밀리면(목표 + 2 cm) 정지."""
    sim, ex = _to_dock_move(graph)
    reading = sim.sensor.update(sim.raw())
    x, y, yaw = sim.pose()
    t = sim.t
    for i in range(4):                     # 멈춤 확인(stamp 없음 = 샘플마다 새 pose)
        ex.step(reading, (x, y, yaw), t + 0.05 * i)
    assert ex._prim['settled']
    for i in range(40):                    # 접근 방향(남, −y) 반대로 밀림
        ex.step(reading, (x, y + 0.3 * i, yaw), t + 0.2 + 0.05 * i)
        if not ex.active:
            break
    assert ex.state == ERROR and 'overrun' in ex.reason


# ---------------------------------------------------------------- 값 검사·주행 중 거부
@pytest.mark.parametrize('bad', [
    (6.1, 90, 0), (-6.5, 90, 0), (0, 59.9, 0), (0, 120.5, 0), (0, 90, 6.01), (0, 90, -7),
    (float('nan'), 90, 0), (0, float('inf'), 0), (True, 90, 0), ('1', 90, 0), (None, 90, 0)])
def test_dock_value_range(graph, bad):
    ex = RouteExecutor(graph)
    ex.set_dock('N11', 1.0, 95.0, 1.0)
    with pytest.raises(ValueError):
        ex.set_dock('N11', *bad)
    assert ex.dock.get('N11') == DockValue(1.0, 95.0, 1.0)


def test_dock_limits_and_nodes(graph):
    assert load_nodes(graph) == {'N01', 'N11', 'N21', 'N22'}
    DockValue(6, 60, -6)
    DockValue(-6, 120, 6)
    ex = RouteExecutor(graph)
    for node in ('N16', 'N07', 'N12', 'X1'):        # 대기 칸·분기·handoff·없는 노드
        with pytest.raises(ValueError, match='not a load node'):
            ex.set_dock(node, 0, 90, 0)
    ex.set_dock('N21', 0, 90, 0)                    # 기본값이면 표에 남기지 않음
    assert ex.as_dict()['dock'] == {}
    assert DEFAULT.is_default


def test_dock_rejected_while_driving(graph):
    ex = RouteExecutor(graph)
    ex.start(TO_N11)
    with pytest.raises(ValueError, match='while driving'):
        ex.set_dock('N11', 1, 90, 0)
    with pytest.raises(ValueError, match='while driving'):
        ex.reset_dock('N11')
    with pytest.raises(ValueError, match='while driving'):
        ex.replace_dock(DockTable(graph))
    ex.stop()
    ex.set_dock('N11', 1, 90, 0)
    assert ex.as_dict()['dock'] == {'N11': {'advance_cm': 1.0, 'rotate_deg': 90.0, 'move_cm': 0.0}}


# ---------------------------------------------------------------- 파일
def test_dock_file_round_trip(graph, tmp_path):
    path = tmp_path / 'sub' / 'roport_dock.yaml'
    table = DockTable(graph)
    table.set('N11', 2, 96, 2)
    table.set('N01', -2.5, 84, -3)
    save_file(str(path), table)
    assert sorted(os.listdir(path.parent)) == ['roport_dock.yaml']     # 임시 파일 안 남음
    loaded, warning = load_file(str(path), graph)
    assert warning is None and loaded.as_dict() == table.as_dict()
    import yaml
    data = yaml.safe_load(path.read_text())
    assert data == {'version': 1, 'nodes': {
        'N01': {'advance_cm': -2.5, 'rotate_deg': 84.0, 'move_cm': -3.0},
        'N11': {'advance_cm': 2.0, 'rotate_deg': 96.0, 'move_cm': 2.0}}}


@pytest.mark.parametrize('text', [
    'not: [valid', 'version: 2\nnodes: {}\n', '- 1\n- 2\n', 'version: 1\nnodes: [1]\n',
    'version: 1\nnodes:\n  N11: {advance_cm: 9, rotate_deg: 90, move_cm: 0}\n',
    'version: 1\nnodes:\n  N16: {advance_cm: 0, rotate_deg: 90, move_cm: 0}\n',
    'version: 1\nnodes:\n  N11: {advance_cm: 1, rotate_deg: 90, move_cm: 0, extra: 1}\n',
    'version: 1\nnodes:\n  N11: {advance_cm: .nan, rotate_deg: 90, move_cm: 0}\n',
    'version: 1\nnodes:\n  N11: {advance_cm: 1}\n  N01: {rotate_deg: 200}\n',
])
def test_broken_dock_file_uses_defaults(graph, tmp_path, text):
    path = tmp_path / 'dock.yaml'
    path.write_text(text)
    table, warning = load_file(str(path), graph)
    assert warning and 'using defaults' in warning
    assert table.as_dict() == {}                   # 일부만 쓰지 않음


def test_missing_dock_file_and_partial_values(graph, tmp_path):
    table, warning = load_file(str(tmp_path / 'none.yaml'), graph)
    assert table.as_dict() == {} and 'not found' in warning
    path = tmp_path / 'dock.yaml'
    path.write_text('version: 1\nnodes:\n  N11: {rotate_deg: 95}\n')     # 빠진 값 = 기본
    table, warning = load_file(str(path), graph)
    assert warning is None and table.get('N11') == DockValue(0.0, 95.0, 0.0)


# ---------------------------------------------------------------- 교차선 출발
CROSS_ROUTES = [(['N07', 'N18', 'N01'], 'right'), (['N07', 'N23', 'N19', 'N11'], 'left')]


@pytest.mark.parametrize('speed, delay', [(0.03, 0.0), (0.03, 0.2), (0.06, 0.15), (0.08, 0.2)])
@pytest.mark.parametrize('docked', [None, 'example', 'request'])
@pytest.mark.parametrize('route, turn', CROSS_ROUTES, ids=['N07-N01', 'N07-N11'])
def test_start_cross_from_n07_south(graph, route, turn, docked, speed, delay):
    """N07 남쪽을 보고 IR 3개가 동서 선 위(축 = N07.y + 3) → 축을 노드로 → 90° 선 찾기 → 완주."""
    sim = Sim(graph, cmd_delay_s=delay)
    sim.place('N07', 'S', 0.0)
    n07 = graph.nodes['N07']
    assert sim.y == pytest.approx(n07.y + 3.0) and sim.x == pytest.approx(n07.x)
    assert sim.sensor.update(sim.raw()).on_line[1]
    ex = RouteExecutor(graph, _cfg(speed))
    for node, v in (DOCK_SETS[docked] if docked else {}).items():
        ex.set_dock(node, *v)
    ex.start(route, start_cross=True, start_facing='S')
    assert ex.state == ADVANCE
    assert ex.as_dict()['pose_est']['y_cm'] == pytest.approx(n07.y + 3.0, abs=0.1)
    first = None
    while ex.active and sim.t < 400:
        sim.tick(ex)
        if first is None and ex.state == 'follow':
            first = (ex.as_dict()['window_cm'], ex._expected, ex._leg_s0)
        if ex.state == LOADING:
            ex.load_done()
    assert ex.state == ARRIVED, ex.as_dict()
    c = ex.cfg
    window, expected, s0 = first
    leg = ex.legs[0]
    assert s0 == c.ir_ahead_cm
    assert expected == pytest.approx(leg.length - c.half_line_cm - c.ir_ahead_cm)
    assert window[0] == pytest.approx(max(c.gate_min * expected - c.gate_slack_cm,
                                          expected - c.gate_early_cm), abs=0.05)   # gate_early 적용
    found = [float(e['detail'].rstrip('°')) for e in ex.events if e['event'] == 'line_found']
    assert c.turn_min_deg <= found[0] <= c.turn_max_deg
    rot = [e for e in ex.events if e['event'] == 'rotate'][0]
    assert rot['detail'].startswith('(1.0' if turn == 'left' else '(-1.0')


def test_start_cross_straight_and_back(graph):
    """첫 구간이 보는 방향과 같으면 회전 없이, 반대면 반 바퀴(back_min~back_max) 뒤 출발."""
    for route, rotate in ((['N07', 'N23', 'N19', 'N11'], False),     # 동쪽 = 보는 방향
                          (['N07', 'N18', 'N01'], True)):            # 서쪽 = 반대
        sim = Sim(graph)
        sim.place('N07', 'E', 0.0)          # 축 = N07 서쪽 3 cm, IR 3개가 남북 선(N05 쪽) 위
        ex = RouteExecutor(graph)
        ex.start(route, start_cross=True, start_facing='E')
        events = _drive_all(sim, ex)
        assert ex.state == ARRIVED, ex.as_dict()
        kinds = [e['event'] for e in events]
        first_leg = kinds.index('leg')
        assert ('rotate' in kinds[:first_leg]) == rotate
        if rotate:
            found = float(events[kinds.index('line_found')]['detail'].rstrip('°'))
            assert found >= ex.cfg.back_min_deg


@pytest.mark.parametrize('kw, match', [
    ({'start_cross': True, 'start_facing': 'S'}, 'bar'),                       # route[0] = N16 가로선
    ({'start_cross': True, 'start_facing': 'X'}, 'start_facing'),
    ({'start_cross': True}, 'start_facing'),
    ({'start_facing': 'S'}, 'only for start_cross'),
    ({'start_cross': True, 'start_facing': 'S', 'start_offset_cm': 3.0}, 'start_offset'),
])
def test_start_cross_rejects(graph, kw, match):
    ex = RouteExecutor(graph)
    route = TO_N01 if kw.get('start_facing') == 'S' and 'start_offset_cm' not in kw \
        and kw.get('start_cross') else ['N07', 'N18', 'N01']
    with pytest.raises(GraphError, match=match):
        ex.start(route, **kw)
    assert ex.state == 'idle'


def test_start_cross_needs_cross_line():
    g = Graph.from_dict({'nodes': {'items': [
        {'id': 'A', 'at': [0, 0], 'links': ['B'], 'bar': 'y'},
        {'id': 'B', 'at': [50, 0], 'links': ['A', 'C']},
        {'id': 'C', 'at': [100, 0], 'links': ['B'], 'bar': 'y'}]}})
    ex = RouteExecutor(g)
    with pytest.raises(GraphError, match='no line across'):
        ex.start(['B', 'C'], start_cross=True, start_facing='E')
