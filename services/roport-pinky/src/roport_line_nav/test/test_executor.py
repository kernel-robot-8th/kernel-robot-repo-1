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
