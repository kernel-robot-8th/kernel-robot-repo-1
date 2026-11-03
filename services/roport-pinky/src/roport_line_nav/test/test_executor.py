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
    ex.start(['N01', 'N18', 'N07', 'N19', 'N11'])
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


def test_load_to_p6_park(graph):
    """인도장(P7) → 물류 대기(P6): 180° 회전 후 후진해 가로선에서 정지."""
    sim, ex = run_route(graph, ['N11', 'N19', 'N07', 'N05', 'N15', 'N17'], 'N', 7.0)
    assert ex.state == ARRIVED, ex.as_dict()
    assert abs(_heading_deg(sim) - (-90.0)) < 15    # 칸에서 바깥(남)을 봄
    assert sim.sensor.update(sim.raw()).node        # IR 3개가 가로선 위
    n17 = graph.nodes['N17']
    assert sim.y > n17.y                            # 바퀴 축이 가로선보다 벽 쪽(후진함)
    assert any(e['event'] == 'reverse' for e in ex.events)


def test_smartcart_p1_to_delivery(graph):
    """스마트카트 대기(P1) → 인도장(P7). 스마트카트도 같은 패키지를 쓴다."""
    sim, ex = run_route(graph, ['H1', 'N09', 'N19', 'N11'], 'N', 0.0)
    assert ex.state == ARRIVED, ex.as_dict()
    assert _axle_to(sim, 'N11') < 6.0


def test_delivery_to_nav2_handoff(graph):
    sim, ex = run_route(graph, ['N11', 'N19', 'N09', 'N08', 'N12'], 'N', 7.0)
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
