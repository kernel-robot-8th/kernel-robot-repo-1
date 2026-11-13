import math
import os

import pytest
from sim import Sim

from roport_line_nav.executor import (ARRIVED, ERROR, HANDOFF_STATE, LOADING, NavConfig,
                                      RouteExecutor, STOPPED)
from roport_line_nav.graph import Graph, GraphError

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def test_confirmed_n12_start_rejects_early_noise_but_accepts_expected_branch(graph):
    from roport_pinky_line.line_sensor import LineReading
    runner = RouteExecutor(graph)
    runner.start(['N12', 'N08', 'N09', 'H1'], start_offset_cm=0., confirmed_start=True)
    runner._odo = runner._ref_odo+25.
    side = LineReading((300,)*3, (1., 1., 0.), (True, True, False), -.5, False, False)
    for _ in range(2):
        runner._follow(side)
    assert not any(e['event'] == 'node' for e in runner.events)
    runner._odo = runner._ref_odo+35.
    # _arrive dispatch needs an established odom heading.
    runner._pose, runner._hold_yaw = (0., 0., math.pi), math.pi
    for _ in range(2):
        runner._follow(side)
    assert any(e['event'] == 'node' and e['detail'].startswith('N08') for e in runner.events)


@pytest.mark.parametrize('offset', [None, -1., math.nan, math.inf, 1000.])
def test_confirmed_start_rejects_unmeasured_offsets(graph, offset):
    runner = RouteExecutor(graph)
    with pytest.raises(ValueError):
        runner.start(['N12', 'N08'], start_offset_cm=offset, confirmed_start=True)
    assert runner.state == 'idle'


def _axle_to(sim, node_id):
    n = sim.graph.nodes[node_id]
    return math.hypot(sim.x - n.x, sim.y - n.y)


def _heading_deg(sim):
    return math.degrees(math.atan2(math.sin(sim.yaw), math.cos(sim.yaw)))


def _done(ex):
    return not ex.active


def run_route(graph, route, start_heading, start_offset, skip=(), loading=None, sim_kw=None,
              **cfg):
    sim = Sim(graph, skip=skip, **(sim_kw or {}))
    sim.place(route[0], start_heading, start_offset)
    ex = RouteExecutor(graph, NavConfig(**cfg))
    ex.start(route)
    sim.run(ex, _done, on_loading=loading or (lambda s: True))
    return sim, ex


def test_p5_to_warehouse_load(graph):
    """물류 대기(P5, 칸에서 바깥을 보고 섬) → 창고 상차(P4): 회전 4번 + 상차 90° + 대기 + 90°."""
    seen = []

    def loading(sim):
        seen.append(_heading_deg(sim))
        return True

    sim, ex = run_route(graph, ['N16', 'N14', 'N02', 'N10', 'N18', 'N01'], 'S', 0.0,
                        loading=loading)
    assert ex.state == ARRIVED, ex.as_dict()
    # N18 → N01 은 남(−y)쪽. 반시계 90° 뒤 동(0°)을 보고 상차 대기
    assert seen and abs(seen[0] - 0.0) < 8, seen
    # 상차 뒤 반시계로 진입선을 찾아 북(+90°)을 봄 = N18 쪽으로 다시 출발할 수 있음
    assert abs(_heading_deg(sim) - 90.0) < 15, _heading_deg(sim)
    assert _axle_to(sim, 'N01') < 6.0
    kinds = [e['event'] for e in ex.events]
    assert kinds.count('line_found') == 5   # 회전 4번 + 상차 두 번째 회전


def test_load_waits_until_done(graph):
    sim = Sim(graph)
    sim.place('N01', 'N', 7.0)
    ex = RouteExecutor(graph)
    ex.start(['N01', 'N18', 'N07', 'N23', 'N19', 'N11'])
    assert sim.run(ex, lambda e: e.state == LOADING)
    heading = _heading_deg(sim)
    for _ in range(100):                     # 5초 동안 상차 완료가 없으면 그대로
        sim.tick(ex)
    assert ex.state == LOADING and sim.cmd == (0.0, 0.0)
    assert abs(_heading_deg(sim) - heading) < 1.0
    assert ex.load_done()
    assert sim.run(ex, _done)
    assert ex.state == ARRIVED
    assert abs(_heading_deg(sim) - 90.0) < 15     # N11 에서 N19 쪽(북)을 봄


@pytest.mark.parametrize('route, stop', [
    (['N16', 'N14', 'N02', 'N10', 'N18', 'N07', 'N23', 'N21'], 'N21'),
    (['N11', 'N19', 'N24', 'N22'], 'N22'),
])
def test_delivery_side_stops_load_like_n11(graph, route, stop):
    """02 인도장 새 정차 노드(N21·N22, arrival: load): N11처럼 반시계 90° 뒤 상차 대기."""
    seen = []

    def loading(sim):
        seen.append(_heading_deg(sim))
        return True

    heading, offset = ('S', 0.0) if route[0] == 'N16' else ('N', 7.0)
    sim, ex = run_route(graph, route, heading, offset, loading=loading)
    assert ex.state == ARRIVED, ex.as_dict()
    assert seen and abs(seen[0] - 0.0) < 8, seen       # 남쪽으로 들어와 반시계 90° = 동쪽
    assert abs(_heading_deg(sim) - 90.0) < 15          # 상차 뒤 간선(북) 쪽을 봄
    assert _axle_to(sim, stop) < 6.0


def test_load_to_p6_park(graph):
    """인도장(P7) → 물류 대기(P6): 180° 회전 후 후진해 가로선에서 정지."""
    sim, ex = run_route(graph, ['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N17'], 'N', 7.0)
    assert ex.state == ARRIVED, ex.as_dict()
    assert abs(_heading_deg(sim) - (-90.0)) < 15    # 칸에서 바깥(남)을 봄
    assert sim.sensor.update(sim.raw()).node        # IR 3개가 가로선 위
    n17 = graph.nodes['N17']
    assert sim.y > n17.y                            # 바퀴 축이 가로선보다 벽 쪽(후진함)
    assert any(e['event'] == 'reverse' for e in ex.events)


def test_smartcart_p1_to_delivery(graph):
    """스마트카트 대기(P1) → 인도장(P7). 스마트카트도 같은 패키지를 쓴다."""
    sim, ex = run_route(graph, ['H1', 'N09', 'N24', 'N19', 'N11'], 'N', 0.0)
    assert ex.state == ARRIVED, ex.as_dict()
    assert _axle_to(sim, 'N11') < 6.0


def test_delivery_to_nav2_handoff(graph):
    sim, ex = run_route(graph, ['N11', 'N19', 'N24', 'N09', 'N08', 'N12'], 'N', 7.0)
    assert ex.state == HANDOFF_STATE, ex.as_dict()
    assert _axle_to(sim, 'N12') < 6.0
    assert sim.cmd == (0.0, 0.0)


def test_stop_command_zero_then_silent(graph):
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    for _ in range(40):
        sim.tick(ex)
    assert sim.cmd[0] > 0
    ex.stop()
    reading = sim.sensor.update(sim.raw())
    assert ex.step(reading, sim.pose(), sim.t).linear == 0.0
    assert ex.step(reading, sim.pose(), sim.t) is None
    assert ex.state == STOPPED


def test_missing_branch_tape_stops(graph):
    """N18 의 P4 분기 테이프가 없으면 노드를 놓치고 정지(node_missed)."""
    sim, ex = run_route(graph, ['N10', 'N18', 'N01'], 'E', 4.0, skip={('N01', 'N18')})
    assert ex.state == ERROR and 'node_missed' in ex.reason, ex.reason
    assert sim.cmd == (0.0, 0.0)


def test_no_odom_stops(graph):
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    sim.tick(ex)
    reading = sim.sensor.update(sim.raw())
    cmd = ex.step(reading, None, sim.t)
    assert ex.state == ERROR and ex.reason == 'no_odom' and cmd.linear == 0.0


def test_start_rejects_bad_route_and_busy(graph):
    ex = RouteExecutor(graph)
    with pytest.raises(GraphError):
        ex.start(['N16', 'N02'])
    assert ex.state == 'idle'
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    with pytest.raises(GraphError, match='busy'):
        ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])


def test_config_limits():
    with pytest.raises(ValueError):
        NavConfig(max_linear=0.5)
    with pytest.raises(ValueError):
        NavConfig(gate_min=1.2)
    with pytest.raises(ValueError):
        NavConfig(turn_max_deg=80)
    with pytest.raises(ValueError):
        NavConfig(back_min_deg=185)


def _shifted(graph, shifts):
    """실제 테이프가 지도와 다르게 붙은 경우: 노드 id → (dx, dy) cm."""
    from dataclasses import replace
    nodes = {k: replace(n, x=n.x + shifts.get(k, (0, 0))[0], y=n.y + shifts.get(k, (0, 0))[1])
             for k, n in graph.nodes.items()}
    return replace(graph, nodes=nodes)


@pytest.mark.parametrize('odom_scale, yaw_scale', [(1.0, 1.0), (0.93, 1.06), (1.07, 0.94)])
def test_robust_to_odom_error_and_tape_offset(graph, odom_scale, yaw_scale):
    """엔코더 ±7%, 테이프 위치 지도와 최대 4 cm 차이(11-03 재측정에서 본 정도)에서도 완주."""
    # N01 가로선(상차)이 지도보다 3 cm 짧게, N18 분기가 x 로 2 cm 어긋나게 붙은 경우
    tape = _shifted(graph, {'N01': (2.0, 3.0), 'N18': (2.0, 0.0)})
    sim, ex = run_route(graph, ['N16', 'N14', 'N02', 'N10', 'N18', 'N01'], 'S', 0.0,
                        sim_kw={'tape_graph': tape, 'odom_scale': odom_scale,
                                'odom_yaw_scale': yaw_scale})
    assert ex.state == ARRIVED, ex.as_dict()
    assert sim.sensor.update(sim.raw()).on_line[1]      # 끝나고 진입선 위


def test_pose_estimate_follows_route(graph):
    """화면용 위치 추정: 구간 진행률이 0→1 로 늘고, 끝나면 마지막 노드에 있음."""
    sim = Sim(graph)
    sim.place('N14', 'N', 0.0)
    ex = RouteExecutor(graph)
    assert ex.as_dict()['pose_est'] is None
    ex.start(['N14', 'N16'])
    seen = []
    while ex.active and sim.t < 120:
        sim.tick(ex)
        d = ex.as_dict()
        if d['state'] == 'follow':
            seen.append(d['leg_progress'])
    assert seen and seen == sorted(seen) and seen[-1] > 0.8
    d = ex.as_dict()
    assert d['state'] == ARRIVED and d['leg_progress'] == 1.0
    n16 = graph.nodes['N16']
    assert d['pose_est']['heading_deg'] == pytest.approx(-90.0)   # 칸에서 바깥(남)을 봄
    assert d['pose_est']['x_cm'] == pytest.approx(n16.x, abs=0.1)
    assert d['pose_est']['y_cm'] == pytest.approx(n16.y, abs=0.1)


def _chain(graph, place, routes, **sim_kw):
    """한 로봇이 도착한 자리에서 다음 경로를 이어서 출발(테스트 관제 「현재 위치」)."""
    sim = Sim(graph, **sim_kw)
    sim.place(*place)
    ex = RouteExecutor(graph)
    states = []
    for r in routes:
        ex.start(r)
        sim.run(ex, _done, max_s=sim.t + 600, on_loading=lambda s: True)
        states.append((ex.state, ex.reason))
    return sim, ex, states


def test_back_from_handoff_turns_first(graph):
    """N12 handoff(선 끝을 보고 멈춤) 뒤 되돌아가는 경로: 먼저 반 바퀴 돌아 진입선을 찾는다."""
    _, ex, states = _chain(graph, ('N11', 'N', 0.0), [
        ['N11', 'N19', 'N24', 'N09', 'N08', 'N12'],
        ['N12', 'N08', 'N09', 'N24', 'N19', 'N23', 'N07', 'N05', 'N15', 'N17']])
    assert states == [(HANDOFF_STATE, 'route done'), (ARRIVED, 'route done')]
    pre = [e for e in ex.events if e['event'] == 'pre_rotate']
    assert pre and pre[-1]['detail'] == 'E → W'


def test_continue_from_park_after_offset_arrival(graph):
    """옆으로 치우쳐 대기 칸에 들어온 뒤 다시 출발(2026-11-04 가짜 로봇 오류 재현 경로)."""
    _, ex, states = _chain(graph, ('H1', 'N', 0.0), [
        ['H1', 'N09', 'N24', 'N19', 'N23', 'N07', 'N18', 'N01'],
        ['N01', 'N18', 'N07', 'N23', 'N19', 'N24', 'N09', 'N08', 'H2'],
        ['H2', 'N08', 'N09', 'N24', 'N19', 'N23', 'N07', 'N05', 'N15', 'N17']])
    assert [s for s, _ in states] == [ARRIVED, ARRIVED, ARRIVED]
    found = [float(e['detail'].rstrip('°')) for e in ex.events if e['event'] == 'line_found']
    assert min(f for f in found if f > 150) >= 172      # 180° 회전은 172° 전에 멈추지 않음


def test_stop_forgets_end_pose(graph):
    """정지하면 끝난 방향을 잊는다(사람이 옮겼을 수 있음) → 출발 전 회전 없음."""
    sim = Sim(graph)
    sim.place('N11', 'N', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N11', 'N19', 'N24', 'N09', 'N08', 'N12'])
    sim.run(ex, _done, on_loading=lambda s: True)
    assert ex.state == HANDOFF_STATE
    ex.start(['N12', 'N08', 'H2'])
    assert ex.state == 'rotate'                # 기억: 선 끝(E)을 봄 → 먼저 반 바퀴
    ex.stop()                                  # 주행 중 정지 → 기억 없음
    ex.start(['N12', 'N08', 'H2'])
    assert ex.state == 'follow'
    assert ex.as_dict()['window_cm'][0] > 0    # 가로선 노드 출발 = 센서 0 cm 로 보고 거리 창

    sim = Sim(graph)
    sim.place('N11', 'N', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N11', 'N19', 'N24', 'N09', 'N08', 'N12'])
    sim.run(ex, _done, on_loading=lambda s: True)
    ex.stop()                                  # 끝난 뒤 정지: 상태는 그대로, 기억만 지움
    assert ex.state == HANDOFF_STATE
    ex.start(['N12', 'N08', 'H2'])
    assert ex.state == 'follow'


def test_chained_random_routes(graph):
    """정차 노드 사이 무작위 20경로를 이어서(엔코더 오차 포함) 모두 완주."""
    import random
    goals = sorted(n for n in graph.nodes if graph.nodes[n].bar)
    rnd = random.Random(3)
    sim = Sim(graph, odom_scale=1.07, odom_yaw_scale=0.95)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    cur = 'N16'
    for _ in range(20):
        goal = rnd.choice([g for g in goals if g != cur])
        route = _shortest(graph, cur, goal)
        ex.start(route)
        sim.run(ex, _done, max_s=sim.t + 600, on_loading=lambda s: True)
        assert ex.state in (ARRIVED, HANDOFF_STATE), (route, ex.reason)
        cur = goal


def _shortest(graph, start, goal):
    import heapq
    dist, prev, q = {start: 0.0}, {}, [(0.0, start)]
    while q:
        d, n = heapq.heappop(q)
        if n == goal:
            break
        if d > dist[n] or (n != start and graph.nodes[n].bar):
            continue        # 가로선 노드는 지나갈 수 없음
        for m in graph.nodes[n].links:
            nd = d + graph.length(n, m)
            if nd < dist.get(m, math.inf):
                dist[m], prev[m] = nd, n
                heapq.heappush(q, (nd, m))
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]


VIA_ROUTE = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11']


def test_via_load_stop_in_one_route(graph):
    """P5 → P4 상차(경유) → P7 상차를 한 번에: 중간 상차에서 기다렸다 이어 감."""
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(VIA_ROUTE)
    loads = []

    def on_loading(s):
        loads.append(ex.legs[ex.index].to)
        return True
    sim.run(ex, _done, on_loading=on_loading)
    assert ex.state == ARRIVED and ex.route == VIA_ROUTE
    assert loads[0] == 'N01' and loads[-1] == 'N11'
    assert [e['detail'] for e in ex.events if e['event'] == 'via'] == ['N01']
    assert _axle_to(sim, 'N11') < 8.0


def test_via_waits_for_load_done(graph):
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(VIA_ROUTE)
    sim.run(ex, lambda e: e.state == LOADING)
    assert ex.legs[ex.index].to == 'N01'
    sim.run(ex, _done, max_s=sim.t + 10)          # 상차 완료 전에는 그대로
    assert ex.state == LOADING
    assert ex.load_done()
    sim.run(ex, _done, max_s=sim.t + 900, on_loading=lambda s: True)
    assert ex.state == ARRIVED


def test_via_through_handoff_turns_back(graph):
    """N12 를 경유하면 반 바퀴 돌아 되돌아감(N11 → N12 → H2)."""
    sim = Sim(graph)
    sim.place('N11', 'N', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N11', 'N19', 'N24', 'N09', 'N08', 'N12', 'N08', 'H2'])
    sim.run(ex, _done, on_loading=lambda s: True)
    assert ex.state == ARRIVED
    assert [e['event'] for e in ex.events if e['event'] in ('via', 'pre_rotate')] == \
        ['via', 'pre_rotate']


def test_via_rejects_bad_segments(graph):
    ex = RouteExecutor(graph)
    with pytest.raises(GraphError):
        ex.start(['N16', 'N14', 'N02', 'N14', 'N16'])     # 중간 U턴(가로선 아님)
    assert ex.state == 'idle'


def test_set_speed(graph):
    ex = RouteExecutor(graph)
    assert ex.set_speed(0.05, 0.5) == (0.05, 0.5)
    assert ex.as_dict()['speed']['base_speed'] == 0.05
    for bad in ({'base_speed': 0.07}, {'base_speed': 0.01}, {'rotate_speed': 0.7},
                {'rotate_speed': 0.1}, {'base_speed': float('nan')}):
        with pytest.raises(ValueError):
            ex.set_speed(**bad)
    assert ex.cfg.base_speed == 0.05 and ex.cfg.rotate_speed == 0.5


def test_faster_full_route(graph):
    """상한(0.06 m/s, 0.6 rad/s)으로 P5 → P4 → P7 경유 경로 완주."""
    sim = Sim(graph, odom_scale=1.07, odom_yaw_scale=0.95)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.set_speed(0.06, 0.6)
    ex.start(VIA_ROUTE)
    sim.run(ex, _done, on_loading=lambda s: True)
    assert ex.state == ARRIVED


# ---------------------------------------------------------------- 노드 앞 감속(2026-11-06)
# 실로봇 d660 0.06 m/s: 노드 회전이 107~117° 에서 끝나고(0.03 m/s 는 70~93°) 다음 구간을 약 20° 비스듬히
# 출발 → N14→N02(24 cm)에서 왼쪽 센서가 직선 테이프 가장자리를 밟아 10.9 cm(창 시작 10.86 cm)에서
# 모서리로 오판 → 회전 120° 안에 선 없음(line_not_found). 0.03 m/s 는 18.4 cm 에서 맞게 감지.

def _reading(on, error=0.0):
    from roport_pinky_line.line_sensor import LineReading
    n = tuple(1.0 if o else 0.0 for o in on)
    return LineReading(raw=(0, 0, 0), n=n, on_line=tuple(on), error=error, lost=not any(on),
                       node=all(on))


CENTER_ONLY = (False, True, False)
LEFT_SIDE = (True, True, False)


def _drive_synthetic(ex, pose, heading, n, reading, step_cm=0.3, t0=0.0, dt=0.05):
    """odom 이 heading 쪽으로 step_cm 씩 가는 가짜 입력 n 샘플. → (명령 목록, 마지막 자세, 시각)."""
    from sim import DIRV
    dx, dy = DIRV[heading]
    x, y, yaw = pose
    cmds, t = [], t0
    for _ in range(n):
        cmds.append((ex.as_dict()['traveled_cm'], ex.state, ex.step(reading, (x, y, yaw), t)))
        x, y, t = x + dx * step_cm, y + dy * step_cm, t + dt
    return cmds, (x, y, yaw), t


def test_turn_leg_keeps_min_window(graph):
    """노드에서 90° 돈 직후 구간: 바퀴 축 = 노드 중심, 센서 = ir_ahead 앞으로 보고 최소 거리 창을 둔다."""
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    assert sim.run(ex, lambda e: e.index == 1 and e.state == 'follow')
    c = ex.cfg
    leg = ex.legs[1]
    expected = leg.length - c.half_line_cm - c.ir_ahead_cm
    lo, hi = ex.as_dict()['window_cm']
    # 창 시작 = max(비율 창, 예상 − gate_early_cm). 짧은 구간(N14→N02)은 예상 − 5 cm 쪽
    assert lo == pytest.approx(max(c.gate_min * expected - c.gate_slack_cm,
                                   expected - c.gate_early_cm), abs=0.05)
    assert lo == pytest.approx(expected - 5.0, abs=0.05) and lo > 14
    assert hi == pytest.approx(c.gate_max * expected + c.gate_slack_cm, abs=0.05)


def test_early_side_pattern_before_window_ignored(graph):
    """회전 직후 구간에서 창 시작 전에 옆선 패턴(on=110)이 와도 노드로 보지 않고, 창 안에서는 감지."""
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    assert sim.run(ex, lambda e: e.index == 1 and e.state == 'follow')
    lo = ex.as_dict()['window_cm'][0]
    pose, t = sim.pose(), sim.t
    n_before = int((lo - 0.5) / 0.3)          # 창 시작 0.5 cm 전까지 계속 옆선 패턴
    _, pose, t = _drive_synthetic(ex, pose, 'W', n_before, _reading(LEFT_SIDE), t0=t)
    assert ex.state == 'follow' and ex.index == 1
    assert not [e for e in ex.events if e['event'] == 'node' and 'N02' in e['detail']]
    _, pose, t = _drive_synthetic(ex, pose, 'W', 6, _reading(LEFT_SIDE), t0=t)
    assert ex.state in ('advance', 'rotate'), ex.as_dict()       # 창 안: 노드 N02


def test_approach_slows_before_node_and_at_node(graph):
    """긴 구간(N02→N10 88.9 cm)을 0.06 m/s 로: 예상 노드 위치 approach_lead(5 cm) 전부터 0.03,
    노드 뒤 회전 전 전진도 0.03(2026-11-07 사용자: 노드 3~5 cm 전 감속)."""
    ex = RouteExecutor(graph)
    ex.set_speed(0.06, 0.6)
    ex.start(['N02', 'N10', 'N18', 'N01'], start_offset_cm=3.0)
    lo, hi = ex.as_dict()['window_cm']
    exp = ex._expected
    assert lo < exp - 5.0                                      # 예전 기준(창 시작)보다 늦게 감속
    c = ex.cfg
    n02 = graph.nodes['N02']
    pose = (n02.x, n02.y + 3.0 - c.ir_ahead_cm, -math.pi / 2)
    cmds, pose, t = _drive_synthetic(ex, pose, 'S', int(exp / 0.3) - 2, _reading(CENTER_ONLY))
    fast = [cmd.linear for d, st, cmd in cmds if st == 'follow' and cmd and d < exp - 5.0 - 0.3]
    slow = [cmd.linear for d, st, cmd in cmds if st == 'follow' and cmd and d >= exp - 5.0 + 0.3]
    assert fast and slow and min(fast) == pytest.approx(0.06) and max(slow) == pytest.approx(0.03)
    assert max(d for d, st, cmd in cmds if st == 'follow' and cmd and cmd.linear > 0.05) >= lo  # 창 안도 빠르게
    cmds, pose, t = _drive_synthetic(ex, pose, 'S', 3, _reading(LEFT_SIDE), t0=t)
    assert ex.state == 'advance'
    cmds, _, _ = _drive_synthetic(ex, pose, 'S', 3, _reading(LEFT_SIDE), t0=t)
    assert all(cmd.linear == pytest.approx(0.03) for _, st, cmd in cmds if st == 'advance')
    for cmd in (cmd for *_, cmd in cmds if cmd is not None):
        assert abs(cmd.linear) <= c.max_linear and abs(cmd.angular) <= c.max_angular


def test_straight_pass_keeps_full_speed(graph):
    """직진 통과 노드(N07→N23→N19 의 N23)는 다가갈 때·지날 때 감속하지 않음(2026-11-07 사용자)."""
    ex = RouteExecutor(graph)
    ex.set_speed(0.06, 0.6)
    ex.start(['N07', 'N23', 'N19', 'N11'], start_offset_cm=3.0)
    assert ex.legs[0].action == 'straight'
    exp = ex._expected
    n07 = graph.nodes['N07']
    pose = (n07.x + 3.0 - ex.cfg.ir_ahead_cm, n07.y, 0.0)
    cmds, pose, t = _drive_synthetic(ex, pose, 'E', int(exp / 0.3), _reading(CENTER_ONLY))
    assert all(cmd.linear == pytest.approx(0.06) for _, st, cmd in cmds if st == 'follow' and cmd and _ > 0.5)
    cmds, pose, t = _drive_synthetic(ex, pose, 'E', 3, _reading(RIGHT_SIDE), t0=t)   # N23: 오른쪽 갈래
    assert ex.state == 'pass'
    cmds, pose, t = _drive_synthetic(ex, pose, 'E', 4, _reading(CENTER_ONLY), t0=t)
    assert all(cmd.linear == pytest.approx(0.06) for _, st, cmd in cmds if st == 'pass' and cmd)
    slow = RouteExecutor(graph, NavConfig(pass_full_speed=False))
    slow.set_speed(0.06, 0.6)
    slow.start(['N07', 'N23', 'N19', 'N11'], start_offset_cm=3.0)
    cmds, *_ = _drive_synthetic(slow, (n07.x, n07.y, 0.0), 'E', int(exp / 0.3), _reading(CENTER_ONLY))
    assert min(cmd.linear for _, st, cmd in cmds if st == 'follow' and cmd and _ > 0.5) == pytest.approx(0.03)


def test_approach_whole_leg_when_start_unknown(graph):
    """출발 위치를 모르는 구간(창 최소 0)은 처음부터 approach_speed."""
    ex = RouteExecutor(graph)
    ex.set_speed(0.06, 0.6)
    ex.start(['N02', 'N10', 'N18', 'N01'])
    assert ex.as_dict()['window_cm'][0] == 0.0
    n02 = graph.nodes['N02']
    cmds, _, _ = _drive_synthetic(ex, (n02.x, n02.y, -math.pi / 2), 'S', 40,
                                  _reading(CENTER_ONLY))
    lin = [cmd.linear for _, st, cmd in cmds if st == 'follow' and cmd and cmd.linear > 0]
    assert lin and max(lin) == pytest.approx(0.03)


def test_approach_speed_config_and_set_speed(graph):
    ex = RouteExecutor(graph)
    d = ex.as_dict()['speed']
    assert d['approach_speed'] == 0.03 and d['approach_lead_cm'] == 5.0 and d['pass_full_speed']
    ex.set_speed(0.06)                                   # 직선 속도만 바꿔도 감속 값은 그대로
    assert ex.as_dict()['speed']['approach_speed'] == 0.03
    ex.set_speed(0.02)                                   # base 가 더 느리면 base(상한·시험 속도 제한 존중)
    assert ex.approach_speed == 0.02 and ex.as_dict()['speed']['approach_speed'] == 0.02
    assert RouteExecutor(graph, NavConfig(base_speed=0.02, approach_speed=0.05)).approach_speed \
        == 0.02
    for bad in ({'approach_speed': 0.01}, {'approach_speed': 0.2},
                {'approach_speed': float('nan')}, {'approach_lead_cm': -1.0},
                {'approach_lead_cm': float('inf')}):
        with pytest.raises(ValueError):
            NavConfig(**bad)


def test_sim_cmd_delay():
    """시뮬레이터 구동 지연: 명령이 cmd_delay_s 뒤에 바퀴에 걸림(기본 0 = 바로)."""
    from roport_line_nav.executor import Command

    class Once:
        state = 'follow'

        def __init__(self):
            self.n = 0

        def step(self, reading, pose, now):
            self.n += 1
            return Command(0.05, 0.0) if self.n == 1 else None

    g = Graph.load(LAYOUT)
    for delay, ticks_still in ((0.0, 0), (0.2, 4)):
        sim = Sim(g, cmd_delay_s=delay)
        sim.place('N02', 'S', 0.0)
        ex = Once()
        moved = []
        for _ in range(8):
            sim.tick(ex)
            moved.append(sim.cmd[0] > 0)
        assert moved.index(True) == ticks_still, (delay, moved)
    with pytest.raises(ValueError):
        Sim(g, cmd_delay_s=-0.1)


DELAY_S = 0.2   # 실로봇 0.06 m/s 기록: 전진 → 회전 전환 뒤 0.9 cm(약 0.15 s) 더 감. 그보다 넉넉히
N16_N01 = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']


@pytest.mark.parametrize('approach, ok', [(0.06, False), (0.03, True)])
def test_fast_with_drive_delay_needs_approach(graph, approach, ok):
    """0.06 m/s + 구동 지연 0.2 s: 감속이 없으면(approach = base) 노드를 지나쳐 회전 중 선을 못 찾아 정지,
    기본 감속(0.03)이면 완주하고 90° 회전이 110° 안에서 끝난다."""
    sim = Sim(graph, cmd_delay_s=DELAY_S)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, NavConfig(approach_speed=approach))
    ex.set_speed(0.06, 0.4)
    ex.start(N16_N01)
    sim.run(ex, _done, on_loading=lambda s: True)
    if not ok:
        assert ex.state == ERROR and 'line_not_found' in ex.reason, ex.as_dict()
        return
    assert ex.state == ARRIVED, ex.as_dict()
    found = [float(e['detail'].rstrip('°')) for e in ex.events if e['event'] == 'line_found']
    assert len(found) == 5 and max(found[:4]) <= 110, found


FAST_ROUTES = [
    (N16_N01, 'S', 0.0, ARRIVED),
    (['N01', 'N18', 'N07', 'N23', 'N19', 'N11'], 'N', 7.0, ARRIVED),
    (['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16'], 'N', 7.0, ARRIVED),
    (['H1', 'N09', 'N24', 'N19', 'N11'], 'N', 0.0, ARRIVED),             # 스마트카트
    (['N11', 'N19', 'N24', 'N09', 'N08', 'N12'], 'N', 7.0, HANDOFF_STATE),
    (VIA_ROUTE, 'S', 0.0, ARRIVED),
]


@pytest.mark.parametrize('odom', [(1.0, 1.0), (1.07, 0.95)])
@pytest.mark.parametrize('route, heading, offset, final', FAST_ROUTES,
                         ids=[f'{r[0][0]}-{r[0][-1]}' + ('-via' if len(r[0]) > 8 else '')
                              for r in FAST_ROUTES])
def test_fast_routes_with_drive_delay(graph, route, heading, offset, final, odom):
    """상한 0.06 m/s·0.6 rad/s + 구동 지연 0.2 s(+엔코더 오차)로 주요 경로 완주."""
    sim, ex = run_route(graph, route, heading, offset,
                        sim_kw={'cmd_delay_s': DELAY_S, 'odom_scale': odom[0],
                                'odom_yaw_scale': odom[1]},
                        base_speed=0.06, rotate_speed=0.6)
    assert ex.state == final, ex.as_dict()
    assert ex.cfg.base_speed == 0.06 and ex.approach_speed == 0.03


def test_fast_routes_with_drive_delay_speed_cmd(graph):
    """주행 중 속도 명령(set_speed, 콘솔 슬라이더)으로 0.06 m/s 를 줘도 감속은 유지: P7 → P6 후진 정차."""
    sim = Sim(graph, cmd_delay_s=DELAY_S)
    sim.place('N11', 'N', 7.0)
    ex = RouteExecutor(graph)
    ex.set_speed(0.06, 0.6)
    ex.start(['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N17'])
    sim.run(ex, _done, on_loading=lambda s: True)
    assert ex.state == ARRIVED, ex.as_dict()


def test_approach_default_starts_when_line_cap_is_below_it():
    """trial launch 가 max_linear = 직선 속도(예 0.02)로 낮춰도 기본 approach 0.03 으로 시작, 실제는 0.02."""
    cfg = NavConfig(base_speed=0.02, max_linear=0.02)
    ex = RouteExecutor(Graph.load(LAYOUT), cfg)
    assert ex.approach_speed == 0.02
    with pytest.raises(ValueError, match='approach_speed'):
        NavConfig(approach_speed=0.2)


def test_round_trip_p5_p4_p5(graph):
    """테스트 콘솔 예시 「P5 → P4(P7) 상차 → P5 왕복」: 경유 상차 뒤 P5 칸으로 후진 정차."""
    p7 = ['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11',
          'N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16']
    for route, speed in ((['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N10', 'N02', 'N14', 'N16'], 0.03),
                         (['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N10', 'N02', 'N14', 'N16'], 0.06),
                         (p7, 0.03), (p7, 0.06)):
        sim, ex = run_route(graph, route, 'S', 0.0, base_speed=speed)
        assert ex.state == ARRIVED, (speed, ex.as_dict())
        kinds = [e['event'] for e in ex.events]
        assert 'via' in kinds and kinds.count('loading') == 1 and 'reverse' in kinds
        assert _axle_to(sim, 'N16') < 8.0


# ---------------------------------------------------------------- T자 양쪽 확인(2026-11-07)
# 실로봇 d660: P5→N14(줄기 쪽에서 T자)에서 왼쪽 센서만 0.3 cm 테이프 가장자리 → N14 오감지 →
# 6 cm 앞에서 회전 → line_not_found after 120°. 실로그 16건의 진짜 T자 감지는 모두 양쪽이 켜짐.
RIGHT_SIDE = (False, True, True)
BOTH = (True, True, True)


def _to_n14_window(graph, **cfg):
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, NavConfig(**cfg))
    ex.start(['N16', 'N14', 'N15', 'N17'])
    assert ex.legs[0].sides == {'left', 'right'}             # 줄기 쪽에서 T자
    lo, hi = ex.as_dict()['window_cm']
    pose = sim.pose()
    _, pose, t = _drive_synthetic(ex, pose, 'S', int(lo / 0.3) + 1, _reading(CENTER_ONLY))
    return ex, pose, t


def test_tee_needs_both_sides(graph):
    ex, pose, t = _to_n14_window(graph)
    _, pose, t = _drive_synthetic(ex, pose, 'S', 8, _reading(LEFT_SIDE), t0=t)   # 왼쪽만 2.4 cm
    assert ex.state == 'follow', ex.as_dict()
    _, pose, t = _drive_synthetic(ex, pose, 'S', 3, _reading(BOTH), t0=t)        # 양쪽
    assert ex.state == 'advance' and any(
        e['event'] == 'node' and 'N14' in e['detail'] for e in ex.events)


def test_tee_sides_a_little_apart_still_detected(graph):
    """비스듬히 들어가 왼쪽이 먼저(0.9 cm 앞) 보여도 창(1.5 cm) 안이면 감지."""
    ex, pose, t = _to_n14_window(graph)
    _, pose, t = _drive_synthetic(ex, pose, 'S', 3, _reading(LEFT_SIDE), t0=t)
    _, pose, t = _drive_synthetic(ex, pose, 'S', 3, _reading(RIGHT_SIDE), t0=t)
    assert ex.state == 'advance', ex.as_dict()


def test_tee_far_apart_rejected(graph):
    ex, pose, t = _to_n14_window(graph)
    _, pose, t = _drive_synthetic(ex, pose, 'S', 2, _reading(LEFT_SIDE), t0=t)
    _, pose, t = _drive_synthetic(ex, pose, 'S', 8, _reading(CENTER_ONLY), t0=t)  # 2.4 cm 뒤
    _, pose, t = _drive_synthetic(ex, pose, 'S', 3, _reading(RIGHT_SIDE), t0=t)
    assert ex.state == 'follow', ex.as_dict()


def test_tee_check_can_be_disabled(graph):
    ex, pose, t = _to_n14_window(graph, tee_both=False)
    _, pose, t = _drive_synthetic(ex, pose, 'S', 3, _reading(LEFT_SIDE), t0=t)   # 예전: 한쪽만으로 감지
    assert ex.state == 'advance'
    with pytest.raises(ValueError):
        NavConfig(tee_window_cm=0.0)


def test_single_side_branch_unchanged(graph):
    """한쪽 갈래(모서리 N02)는 그대로 한쪽만 보면 감지."""
    ex = RouteExecutor(graph)
    ex.start(['N02', 'N10', 'N18', 'N01'], start_offset_cm=3.0)
    assert ex.legs[0].sides == {'left'}


# ---------------------------------------------------------------- 속도 맞춤 kp(2026-11-07 시험판)

def test_kp_scales_only_above_ref_speed(graph):
    ex = RouteExecutor(graph, NavConfig(base_speed=0.08, max_linear=0.08))
    assert ex.kp_for(0.03) == ex.kp_for(0.06) == 1.0          # 실로봇 완주 속도는 그대로
    assert ex.kp_for(0.07) == pytest.approx(7 / 6) and ex.kp_for(0.08) == pytest.approx(4 / 3)
    off = RouteExecutor(graph, NavConfig(base_speed=0.08, max_linear=0.08, kp_speed_scale=False))
    assert off.kp_for(0.08) == 1.0
    sp = ex.as_dict()['speed']
    assert sp['kp'] == 1.0 and sp['kp_at_base'] == pytest.approx(1.333, abs=1e-3) and sp['kp_speed_scale']
    for bad in ({'gain_ref_speed': 0.0}, {'gain_ref_speed': 0.2}, {'gain_ref_speed': float('nan')},
                {'kp': 0.0}, {'kd': -0.1}, {'kp': float('inf')}):
        with pytest.raises(ValueError):
            NavConfig(**bad)


@pytest.mark.parametrize('speed, kp_eff', [(0.06, 1.0), (0.08, 4 / 3)])
def test_follow_uses_speed_kp(graph, speed, kp_eff):
    """직선 추종 명령의 회전 = -kp(속도)·오차(kd 0). 노드 앞 감속(0.03) 구간은 kp 그대로."""
    sim = Sim(graph, cmd_delay_s=0.15)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, NavConfig(base_speed=speed, max_linear=max(speed, 0.06), kd=0.0))
    ex.start(N16_N01)
    seen = {}
    step = ex.step

    def spy(reading, pose, now):
        cmd = step(reading, pose, now)
        if ex.state == 'follow' and cmd is not None and not reading.lost \
                and abs(reading.error) > 0.02 and abs(cmd.angular) < ex.cfg.max_angular \
                and cmd.linear > ex.cfg.min_speed:
            lin = cmd.linear / (1.0 - ex.cfg.slow_gain * abs(reading.error))
            seen.setdefault(round(lin, 3), set()).add(round(-cmd.angular / reading.error, 3))
        return cmd

    ex.step = spy
    sim.run(ex, _done, on_loading=lambda s: True)
    assert ex.state == ARRIVED, ex.as_dict()
    assert seen.get(round(speed, 3)) == {round(kp_eff, 3)}, seen
    if speed > 0.06:
        assert seen.get(0.03) == {1.0}, seen                       # 노드 앞 감속 구간


@pytest.mark.parametrize('delay', [0.15, DELAY_S])
@pytest.mark.parametrize('route, heading, offset, final', FAST_ROUTES,
                         ids=[f'{r[0][0]}-{r[0][-1]}' + ('-via' if len(r[0]) > 8 else '')
                              for r in FAST_ROUTES])
def test_routes_at_007_with_speed_kp(graph, route, heading, offset, final, delay):
    """0.07 m/s(kp 1.17) + 구동 지연으로 주요 경로 완주(시뮬. 실로봇 미검증)."""
    sim, ex = run_route(graph, route, heading, offset, sim_kw={'cmd_delay_s': delay},
                        base_speed=0.07, max_linear=0.07)
    assert ex.state == final, ex.as_dict()
    assert ex.kp_for(ex.cfg.base_speed) == pytest.approx(7 / 6)


# ---------------------------------------------------------------- 감지 창 시작 = 예상 − 5 cm(2026-11-07 사용자 1안)

@pytest.mark.parametrize('early_cm, detected', [(5.0, False), (0.0, True)])
def test_corner_early_side_inside_ratio_window_rejected(graph, early_cm, detected):
    """실로봇 d660 N14→N02(0.06·0.08): 진짜 모서리 18.3~18.5 cm, 왼쪽 센서 오감지 10.9~11.2 cm(비율 창
    시작 바로 뒤) → 회전 실패. gate_early_cm 5 면 비율 창 시작 + 0.6 cm 의 옆선은 무시하고 예상 − 1.4 cm
    (실로그 18.4 / 19.8 과 같은 차이)에서 감지. 0(예전 비율 창만)이면 이른 옆선을 노드로 오판."""
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, NavConfig(gate_early_cm=early_cm))
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    assert sim.run(ex, lambda e: e.index == 1 and e.state == 'follow')
    c = ex.cfg
    expected = ex._expected
    false_cm = c.gate_min * expected - c.gate_slack_cm + 0.6
    assert false_cm < expected - 5.0
    pose, t = sim.pose(), sim.t
    _, pose, t = _drive_synthetic(ex, pose, 'W', int(false_cm / 0.3) + 1, _reading(LEFT_SIDE), t0=t)
    early = [e for e in ex.events if e['event'] == 'node' and 'N02' in e['detail']]
    assert bool(early) == detected, (false_cm, ex.as_dict()['window_cm'])
    if not detected:
        assert ex.state == 'follow'
        n = int((expected - 1.4 - false_cm) / 0.3) + 2
        _, pose, t = _drive_synthetic(ex, pose, 'W', n, _reading(LEFT_SIDE), t0=t)
        assert ex.state in ('advance', 'rotate'), ex.as_dict()


def test_early_gate_not_on_placed_start(graph):
    """사람이 놓은 첫 구간(출발 위치 가정)은 비율 창만: N11 칸 안쪽에 놓고 출발해도 N19 를 놓치지 않음."""
    ex = RouteExecutor(graph)
    ex.start(['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16'])
    c = ex.cfg
    lo, _ = ex.as_dict()['window_cm']
    leg = ex.legs[0]
    expected = leg.length - c.half_line_cm          # 가로선 노드 출발 가정 s0 = 0
    assert lo == pytest.approx(max(0.0, c.gate_min * expected - c.gate_slack_cm), abs=0.05)
    assert lo < expected - c.gate_early_cm
    sim, ex = run_route(graph, ['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16'], 'N', 7.0,
                        sim_kw={'odom_scale': 0.95})
    assert ex.state == ARRIVED, ex.as_dict()


def test_gate_early_validation():
    for bad in (-1.0, 51.0, float('nan')):
        with pytest.raises(ValueError, match='gate_early_cm'):
            NavConfig(gate_early_cm=bad)
    assert NavConfig().gate_early_cm == 5.0
