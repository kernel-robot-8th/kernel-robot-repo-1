"""일시정지·이어 가기·라인 후진(FFT1-172). 2D 시뮬(test/sim.py)과 합성 입력으로만 확인한다(실로봇 미검증).

예시 경로 = N07(교차선 출발, 동쪽) → N23(직진 통과) → N19(우회전) → N11(상차, P7).
후진 목적 노드 N23 = 옆선 N21(동쪽을 보면 오른쪽). 시작 상황:
  ga       N23→N19 직선 추종 25 cm 에서 pause → back N23 (가)
  ga_det   N19 감지 직후 회전 전 전진(ADVANCE) 중 pause → back N23 (가, 감지 직후)
  ga_short N23 통과 직후 N23→N19 1 cm 에서 pause → back N23 (가′, odom 거리만)
  na       N19 에서 우회전을 마치고 N19→N11 0.5 cm 에서 pause → back N23 (나)
  da       N19→N11 15 cm 에서 pause → back N23 (다)
  dap      N11 상차 대기(LOADING) → back N23 (다′)
"""

import math
import os

import pytest
from sim import Sim

from roport_line_nav.executor import (ADVANCE, ARRIVED, ERROR, FOLLOW, LOADING, PAUSED,
                                      REVERSE, ROTATE, STOPPED, NavConfig, RouteExecutor)
from roport_line_nav.graph import Graph, GraphError

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')
ROUTE = ['N07', 'N23', 'N19', 'N11']
AGAIN = ['N23', 'N19', 'N11']
CASES = ('ga', 'ga_det', 'ga_short', 'na', 'da', 'dap')
# 계약 행렬: 속도 0.02·0.03 × 구동 지연 0·0.15·0.2 s × 엔코더 0.95·1.0·1.05 × 시작 상황 6 = 108회
COMBOS = tuple((d, o) for d in (0.0, 0.15, 0.2) for o in (0.95, 1.0, 1.05))
IR_TOL_CM = 1.2 + 0.6      # 센서가 노드 테이프 위(반폭 1.2) + 지나침
LAT_TOL_CM = 1.0
YAW_TOL_DEG = 3.0


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def _sensor(sim):
    return sim.x + sim.ir_ahead * math.cos(sim.yaw), sim.y + sim.ir_ahead * math.sin(sim.yaw)


def _deg(a):
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


def _start(graph, speed=0.02, delay=0.0, odom=1.0, cls=RouteExecutor, **cfg):
    sim = Sim(graph, cmd_delay_s=delay, odom_scale=odom)
    sim.place('N07', 'E', 0.0)
    ex = cls(graph, NavConfig(reverse_follow_speed=speed, **cfg))
    ex.start(ROUTE, start_cross=True, start_facing='E')
    return sim, ex


def _at(idx, cm, state=FOLLOW):
    return lambda e: e.state == state and e.index == idx and e.as_dict()['traveled_cm'] >= cm


WHEN = {'ga': _at(1, 25.0), 'ga_det': lambda e: e.state == ADVANCE and e.index == 1,
        'ga_short': _at(1, 1.0), 'na': _at(2, 0.5), 'da': _at(2, 15.0),
        'dap': lambda e: e.state == LOADING}


def _pause_settle(sim, ex):
    ex.pause()
    assert ex.state == PAUSED
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5.0), 'odom did not settle'


def _to_case(graph, case, **kw):
    sim, ex = _start(graph, **kw)
    assert sim.run(ex, lambda e: WHEN[case](e) or not e.active, max_s=200.0)
    assert ex.active, ex.as_dict()
    if case == 'dap':
        assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5.0)
    else:
        _pause_settle(sim, ex)
    return sim, ex


def _drive(sim, ex, max_s=400.0, watch=None):
    cmds = []
    while ex.active and sim.t < max_s:
        sim.tick(ex)
        cmds.append(sim.cmd)
        if watch:
            watch(sim, ex)
    return cmds


def _end_pose(graph, sim, node, heading_deg=0.0):
    n = graph.nodes[node]
    sx, _ = _sensor(sim)
    return sx - n.x, sim.y - n.y, _deg(sim.yaw - math.radians(heading_deg))


def forward_arrival_pose(graph, speed, delay, odom):
    """같은 조건에서 N07 → N23 을 전진으로 와서 N23 을 감지한 순간의 자세(비교 기준)."""
    sim, ex = _start(graph, speed, delay, odom)
    while ex.active:
        sim.tick(ex)
        if ex.events and ex.events[-1]['event'] in ('node', 'pass'):
            return _end_pose(graph, sim, 'N23')
    raise AssertionError('N23 not detected')


def run_case(graph, case, speed=0.02, delay=0.0, odom=1.0):
    """→ (후진 결과 상태, N23 끝 자세(센서 x − 노드, 축 y − 노드, 방향°), 최대 옆 오차, 이어 가기 결과, 이벤트)."""
    sim, ex = _to_case(graph, case, speed=speed, delay=delay, odom=odom)
    ex.back('N23')
    lat = [0.0]
    n23, n19 = graph.nodes['N23'], graph.nodes['N19']

    def watch(s, e):     # N23–N19 가로 테이프를 후진하는 동안 축의 옆 오차
        if e.state == REVERSE and n23.x - 5 < s.x < n19.x + 5 and abs(math.sin(s.yaw)) < 0.5:
            lat[0] = max(lat[0], abs(s.y - n23.y))
    _drive(sim, ex, watch=watch)
    result = (ex.state, ex.reason, list(ex.route))
    pose = _end_pose(graph, sim, 'N23')
    events = [e['event'] for e in ex.events]
    again = None
    if ex.state == ARRIVED:
        ex.start(AGAIN)
        sim.run(ex, lambda e: not e.active, max_s=sim.t + 300.0, on_loading=lambda s: True)
        again = (ex.state, ex.reason, _sensor(sim), sim.x, sim.y)
    return result, pose, lat[0], again, events


# ---------------------------------------------------------------- 제어 법칙(부호·안정성)
def _linear_run(omega, v_mps, y0=1.0, th0=math.radians(3.0), d=3.0, dist=40.0, dt=0.05,
                delay_s=0.15, w_max=0.6):
    """선 좌표 단순 운동학: ẏ = v·sin θ, θ̇ = ω, IR 오차 e = y + d·sin θ (cm, + = 축이 선 왼쪽).

    omega(e, θ) → rad/s. 구동 지연 delay_s. → 최대 |y|, 끝 |y|, 끝 |θ|(°).
    """
    y, th, s = y0, th0, 0.0
    v = v_mps * 100.0
    queue = [0.0] * int(round(delay_s / dt))
    peak = abs(y)
    while abs(s) < dist and abs(y) < 20.0:
        e = y + d * math.sin(th)
        queue.append(max(-w_max, min(w_max, omega(e, th))))
        w = queue.pop(0)
        y += v * math.sin(th) * dt
        th += w * dt
        s += v * dt
        peak = max(peak, abs(y))
    return peak, abs(y), abs(math.degrees(th))


@pytest.mark.parametrize('speed', (0.01, 0.02, 0.03))
@pytest.mark.parametrize('y0, th0', [(1.0, 3.0), (-1.0, 6.0), (0.5, -5.0)])
def test_state_feedback_sign_converges(graph, speed, y0, th0):
    """실행부의 ω = +k_y·ŷ − k_θ·θ̂ (ŷ = e − d·sin θ̂) 를 후진(v < 0)에 넣으면 수렴한다.

    특성식 s² + k_θ·s + k_y·|v| (k_y = ωₙ²/|v|, k_θ = 2ζωₙ) → 계수가 모두 양수.
    """
    ex = RouteExecutor(graph, NavConfig(reverse_follow_speed=speed))
    c = ex.cfg
    k_th = 2 * c.reverse_zeta * c.reverse_wn
    k_y = c.reverse_wn ** 2 / (speed * 100.0)
    assert k_th > 0 and k_y * speed * 100.0 == pytest.approx(c.reverse_wn ** 2)

    def omega(e, th):
        return ex._reverse_omega(None, e - c.ir_ahead_cm * math.sin(th), th, speed)
    peak, end_y, end_th = _linear_run(omega, -speed, y0, math.radians(th0))
    assert end_y < 0.1 and end_th < 0.5, (peak, end_y, end_th)
    assert peak < abs(y0) + 1.0


@pytest.mark.parametrize('sign', (1.0, -1.0), ids=('pd_flipped', 'pd_as_forward'))
def test_forward_pd_reversed_diverges_linear(graph, sign):
    """전진 PD(ω = −(kp·e + kd·ė), e 는 IR 간격으로 나눈 값)를 그대로·부호만 바꿔 후진하면 발산한다.

    영점 s = −v/d 가 후진에서 우반면(+|v|/d) → 비례 이득 하나로는 k·d > 0 과 k·v > 0 을 함께 만족 못 함.
    """
    c = NavConfig()
    prev = [None]

    def omega(e, th):
        en = e / c.ir_spacing_cm
        de = 0.0 if prev[0] is None else (en - prev[0]) / 0.05
        prev[0] = en
        return sign * (c.kp * en + c.kd * de)
    peak, end_y, _ = _linear_run(omega, -0.02, 0.5, math.radians(1.0), dist=50.0)
    assert peak > 3.0, peak       # 테이프 반폭(1.2) + IR 간격(2) 밖 = 선 놓침


class _FlippedPD(RouteExecutor):
    """회귀 시험용: 후진에 전진 PD 를 부호만 바꿔 씀(ω = +(kp·e + kd·ė))."""

    def _reverse_omega(self, reading, y_hat, theta, speed):
        e = reading.error
        prev, self._pe = getattr(self, '_pe', None), e
        de = 0.0 if prev is None else (e - prev) / 0.05
        return self.kp_for(speed) * e + self.cfg.kd * de


@pytest.mark.parametrize('dy, dyaw', [(0.0, 0.0), (0.6, 4.0), (-0.8, -5.0)])
def test_flipped_pd_diverges_in_sim_state_feedback_arrives(graph, dy, dyaw):
    """같은 시작(멈춘 뒤 옆 dy cm·방향 dyaw° 흐트러짐)에서 상태 피드백은 N23 에 도착, 부호만 바꾼 PD 는 발산 → 정지."""
    out = {}
    for name, cls in (('state', RouteExecutor), ('pd', _FlippedPD)):
        sim, ex = _start(graph, 0.02, 0.15, cls=cls)
        assert sim.run(ex, _at(1, 40.0))
        _pause_settle(sim, ex)
        sim.y += dy
        sim.yaw += math.radians(dyaw)
        ox, oy, oyaw = sim.odom
        sim.odom = (ox, oy, oyaw + math.radians(dyaw))
        ex.back('N23')
        _drive(sim, ex)
        out[name] = (ex.state, ex.reason)
    assert out['state'][0] == ARRIVED, out
    assert out['pd'][0] == ERROR and out['pd'][1].startswith(('reverse_heading', 'reverse_lost')), out


# ---------------------------------------------------------------- 2D 시뮬 행렬
@pytest.mark.parametrize('delay, odom', COMBOS)
@pytest.mark.parametrize('speed', (0.02, 0.03))
@pytest.mark.parametrize('case', CASES)
def test_back_to_n23_matrix(graph, case, speed, delay, odom):
    """모든 시작 상황에서 N23 에 전진 도착 자세로 서고, 이어서 N23 → N19 → N11 완주."""
    result, pose, lat, again, events = run_case(graph, case, speed, delay, odom)
    assert result[0] == ARRIVED and result[2] == ['N07', 'N23'], (result, events)
    assert 'reverse_arrived' in events and 'load_done' not in events
    ir_dx, axle_dy, yaw = pose
    ref = forward_arrival_pose(graph, speed, delay, odom)
    assert abs(ir_dx) <= IR_TOL_CM and abs(ref[0]) <= IR_TOL_CM, (pose, ref)
    assert abs(axle_dy) <= LAT_TOL_CM and abs(yaw) <= YAW_TOL_DEG, (pose, ref)
    assert lat <= LAT_TOL_CM, lat
    assert again is not None and again[0] == ARRIVED, again
    n11 = graph.nodes['N11']
    assert math.hypot(again[3] - n11.x, again[4] - n11.y) < 6.0, again


def test_back_through_two_nodes_to_n07(graph):
    """(다) 일반화: N19→N11 주행 중 → N19 판정·축 맞춤·회전 되돌리기 → N23 은 직진이라 지나쳐 N07 까지."""
    sim, ex = _to_case(graph, 'da', speed=0.03, delay=0.15)
    ex.back('N07')
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.route == ['N07'], ex.as_dict()
    kinds = [e['event'] for e in ex.events]
    assert kinds.count('reverse_node') == 2          # N19, N07 (N23 은 멈추지 않고 지남)
    ir_dx, axle_dy, yaw = _end_pose(graph, sim, 'N07')
    assert abs(ir_dx) <= IR_TOL_CM and abs(axle_dy) <= LAT_TOL_CM and abs(yaw) <= YAW_TOL_DEG
    assert ex.as_dict()['leg'] is None               # 출발 노드까지 되감음
    ex.start(ROUTE)                                  # 기억한 끝 자세(동쪽, 센서 = 반폭)로 다시 출발
    sim.run(ex, lambda e: not e.active, max_s=sim.t + 300.0, on_loading=lambda s: True)
    assert ex.state == ARRIVED, ex.as_dict()


def test_back_from_corner_turns_to_line_direction(graph):
    """모서리(N02, 직진 선 없음)를 되감으면 선 찾기 대신 기억한 선 방향(odom)까지 돌려 진입 구간을 후진한다."""
    sim = Sim(graph, cmd_delay_s=0.15)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    assert sim.run(ex, lambda e: e.index == 2 and e.state == FOLLOW
                   and e.as_dict()['traveled_cm'] >= 20.0)
    _pause_settle(sim, ex)
    ex.back('N14')
    rot = []
    _drive(sim, ex, watch=lambda s, e: rot.append(e._prim.get('acquire'))
           if e.state == ROTATE else None)
    assert ex.state == ARRIVED and ex.route == ['N16', 'N14'], ex.as_dict()
    assert rot and not any(rot)                       # 고정 각 되돌림(선 찾기 아님)
    leg = graph.heading('N14', 'N02')
    n14 = graph.nodes['N14']
    sx, sy = _sensor(sim)
    along = (sx - n14.x) if leg in 'EW' else (sy - n14.y)
    assert abs(along) <= IR_TOL_CM, along


# ---------------------------------------------------------------- 명령·거절·실패
def _reading(on, error=0.0):
    from roport_pinky_line.line_sensor import LineReading
    n = tuple(1.0 if o else 0.0 for o in on)
    return LineReading(raw=(0, 0, 0), n=n, on_line=tuple(on), error=error, lost=not any(on),
                       node=all(on))


def test_pause_resume_keeps_leg_and_completes(graph):
    sim, ex = _start(graph, delay=0.15)
    assert sim.run(ex, _at(1, 20.0))
    before = ex.as_dict()
    ex.pause()
    cmd = ex.step(sim.sensor.update(sim.raw()), sim.pose(), sim.t)
    assert ex.state == PAUSED and (cmd.linear, cmd.angular) == (0.0, 0.0)
    assert ex.as_dict()['pause'] == {'from': FOLLOW, 'settled': False}
    for _ in range(200):                     # 10 s 멈춤(시간 제한 없음)
        sim.tick(ex)
    st = ex.as_dict()
    assert st['state'] == PAUSED and st['pause']['settled'] is True
    assert st['window_cm'] is None and st['leg'] == before['leg']
    assert any(e['event'] == 'paused_settled' for e in ex.events)
    assert ex.pause() == 'already paused'
    ex.resume()
    assert ex.state == FOLLOW and ex.as_dict()['window_cm'] == before['window_cm']
    assert ex.as_dict()['traveled_cm'] >= before['traveled_cm']
    assert sim.run(ex, lambda e: e.state == LOADING)
    ex.load_done()
    sim.run(ex, lambda e: not e.active)
    assert ex.state == ARRIVED, ex.as_dict()


def test_pause_during_rotate_then_resume(graph):
    sim, ex = _start(graph)
    assert sim.run(ex, lambda e: e.state == ROTATE and e.as_dict()['rotated_deg'] > 30)
    ex.pause()
    for _ in range(40):
        sim.tick(ex)
    with pytest.raises(ValueError, match='rotating'):
        ex.back('N23')
    ex.resume()
    assert ex.state == ROTATE
    assert sim.run(ex, lambda e: e.state == LOADING)


def test_pause_resume_shifts_timers(graph):
    """멈춘 시간은 도킹 이동 시간 초과 등에 세지 않는다."""
    ex = RouteExecutor(graph)
    ex.start(['N14', 'N16'])
    ex.step(_reading((False, True, False)), (0.0, 0.0, 0.0), 0.0)
    ex.pause()
    ex.step(_reading((False, True, False)), (0.0, 0.0, 0.0), 60.0)
    ex.resume()
    assert ex._lost_since is None
    ex.step(_reading((False, False, False)), (0.0, 0.0, 0.0), 60.05)
    assert ex.state == FOLLOW


def test_pause_resume_rejections(graph):
    ex = RouteExecutor(graph)
    with pytest.raises(ValueError):
        ex.pause()
    with pytest.raises(ValueError):
        ex.resume()
    with pytest.raises(ValueError):
        ex.back('N16')
    ex.start(['N14', 'N16'])
    with pytest.raises(ValueError, match='not paused'):
        ex.resume()
    with pytest.raises(ValueError, match='needs paused'):
        ex.back('N14')


def test_back_rejections(graph):
    sim, ex = _start(graph)
    assert sim.run(ex, _at(2, 10.0))
    ex.pause()
    sim.tick(ex)
    with pytest.raises(ValueError, match='still moving'):   # 정지 확인 전
        ex.back('N23')
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5)
    with pytest.raises(ValueError, match='not a node passed'):
        ex.back('N11')                    # 앞쪽 노드
    with pytest.raises(ValueError, match='not a node passed'):
        ex.back('N24')                    # 경로 밖
    assert ex.state == PAUSED and sim.cmd == (0.0, 0.0)
    ex.back('N19')
    with pytest.raises(ValueError, match='already'):
        ex.back('N23')


def test_back_rejected_across_via_and_after_load(graph):
    route = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11']
    sim = Sim(graph)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(route)
    assert sim.run(ex, lambda e: e.index == 7 and e.state == FOLLOW
                   and e.as_dict()['traveled_cm'] > 20, on_loading=lambda s: True)
    _pause_settle(sim, ex)
    with pytest.raises(ValueError, match='not a node passed'):
        ex.back('N10')                    # 경유 상차(N01)를 거꾸로 지나지 않음
    ex.stop()
    assert ex.state == STOPPED
    sim2 = Sim(graph)
    sim2.place('N07', 'E', 0.0)
    ex2 = RouteExecutor(graph)
    ex2.start(ROUTE, start_cross=True, start_facing='E')
    sim2.run(ex2, lambda e: not e.active, on_loading=lambda s: True)
    assert ex2.state == ARRIVED
    for _ in range(20):
        sim2.tick(ex2)
    with pytest.raises(ValueError, match='faces away'):
        ex2.back('N19')                   # 상차 뒤 되돌아 선 자세


def test_back_paused_in_gap_and_dock_move_rejected(graph):
    sim = Sim(graph, gaps=[('N23', 'N19', 20.0, 4.0)])
    sim.place('N07', 'E', 0.0)
    ex = RouteExecutor(graph)
    ex.start(ROUTE, start_cross=True, start_facing='E')
    assert sim.run(ex, lambda e: e._gap is not None)
    _pause_settle(sim, ex)
    with pytest.raises(ValueError, match='gap'):
        ex.back('N23')
    ex.resume()
    assert sim.run(ex, lambda e: e.state == LOADING)

    from roport_line_nav.dock import DockTable
    dock = DockTable(graph)
    dock.set('N11', 3.0, 90.0, 0.0)
    sim, ex = _start(graph)
    ex.dock = dock
    assert sim.run(ex, lambda e: e.state == ADVANCE and e._prim.get('signed')
                   and e._prim.get('settled'))
    _pause_settle(sim, ex)
    with pytest.raises(ValueError, match='dock move'):
        ex.back('N23')


def test_back_from_loading_with_dock_values(graph):
    """(다′) 정밀 도킹 값이 있으면 −b → 시계 r° → −a 로 가로선 정지 자세에 돌아온 뒤 되감는다."""
    from roport_line_nav.dock import DockTable
    dock = DockTable(graph)
    dock.set('N11', -2.0, 96.0, 2.5)
    sim, ex = _start(graph, delay=0.15)
    ex.dock = dock
    assert sim.run(ex, lambda e: e.state == LOADING)
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5)
    ex.back('N23')
    phases = []
    _drive(sim, ex, watch=lambda s, e: phases.append(e._dock_phase) if e._dock_phase else None)
    assert ex.state == ARRIVED, ex.as_dict()
    assert [p for i, p in enumerate(phases) if i == 0 or phases[i - 1] != p] == ['-b', 'unr', '-a']
    ir_dx, axle_dy, yaw = _end_pose(graph, sim, 'N23')
    assert abs(ir_dx) <= IR_TOL_CM and abs(axle_dy) <= LAT_TOL_CM and abs(yaw) <= YAW_TOL_DEG
    assert not ex._loaded and all(e['event'] != 'load_done' for e in ex.events)


def test_stop_while_paused_and_during_reverse(graph):
    sim, ex = _to_case(graph, 'ga')
    ex.stop()
    assert ex.state == STOPPED
    assert ex.step(sim.sensor.update(sim.raw()), sim.pose(), sim.t) is not None   # 0 한 번
    assert ex.step(sim.sensor.update(sim.raw()), sim.pose(), sim.t) is None
    sim, ex = _to_case(graph, 'ga')
    ex.back('N23')
    assert sim.run(ex, lambda e: e.state == REVERSE and e.as_dict()['traveled_cm'] > 5)
    ex.pause()                            # 후진 중 pause → resume 도 됨
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5)
    ex.resume()
    assert ex.state == REVERSE
    ex.stop()
    assert ex.state == STOPPED and ex._end is None


def test_reverse_lost_stops(graph):
    """후진 중 테이프 끊김(GAP·SEARCH 하지 않음) → reverse_lost, 0 속도 한 번."""
    sim = Sim(graph, gaps=[('N23', 'N19', 15.0, 5.0)])
    sim.place('N07', 'E', 0.0)
    ex = RouteExecutor(graph, NavConfig(gap_max_cm=8.0))
    ex.start(ROUTE, start_cross=True, start_facing='E')
    # 끊긴 곳은 전진 때 GAP 으로 건넘. 그 뒤 pause → back 에서 같은 끊김을 후진으로 만남
    assert sim.run(ex, _at(1, 30.0))
    _pause_settle(sim, ex)
    ex.back('N23')
    cmds = _drive(sim, ex)
    assert ex.state == ERROR and ex.reason.startswith('reverse_lost'), ex.reason
    assert cmds[-1] == (0.0, 0.0)
    assert 'reverse_lost' in [e['event'] for e in ex.events]


def test_reverse_node_missed_stops(graph):
    """N23 옆선(N21 쪽 테이프)이 없으면 최대 거리에서 reverse_node_missed."""
    from sim import tape_rects
    sim, ex = _to_case(graph, 'ga')
    sim.rects = tape_rects(graph, skip=(('N21', 'N23'),))    # 전진 뒤 N23 옆선을 떼어 냄
    ex.back('N23')
    _drive(sim, ex)
    assert ex.state == ERROR and ex.reason.startswith('reverse_node_missed'), ex.reason
    max_ok = float(ex.reason.split('within ')[1].split(' cm')[0])
    assert max_ok == pytest.approx(1.3 * (25.0 + 3.8 - 1.2) + 3.0, abs=1.0)


def test_reverse_heading_stops(graph):
    """odom 방향이 선 방향에서 15° 넘게 벌어지면 reverse_heading."""
    ex = RouteExecutor(graph)
    sim, ex = _to_case(graph, 'ga')
    ex.back('N23')
    sim.tick(ex)
    ox, oy, oyaw = sim.odom
    sim.odom = (ox, oy, oyaw + math.radians(20.0))     # odom 만 틀어짐(실제는 선 위)
    sim.tick(ex)
    assert ex.state == ERROR and ex.reason.startswith('reverse_heading'), ex.reason


def test_reverse_speed_command_and_limits(graph):
    ex = RouteExecutor(graph)
    assert ex.cfg.reverse_follow_speed == 0.02 and ex.cfg.reverse_speed == 0.02
    ex.set_speed(reverse_speed=0.03)
    assert ex.cfg.reverse_follow_speed == 0.03 and ex.cfg.reverse_speed == 0.02   # 주차 후진은 그대로
    assert ex.events[-1]['detail'].endswith('reverse 0.030 m/s')
    for bad in (0.005, 0.031, float('nan')):
        with pytest.raises(ValueError):
            ex.set_speed(reverse_speed=bad)
    assert ex.cfg.reverse_follow_speed == 0.03
    sp = ex.as_dict()['speed']
    assert (sp['reverse_follow_speed'], sp['reverse_follow_min'], sp['reverse_follow_max']) == \
        (0.03, 0.01, 0.03)
    ex.set_speed(base_speed=0.04)
    assert ex.events[-1]['detail'] == '0.040 m/s, 0.40 rad/s'      # 기존 문구 그대로
    for bad in ({'reverse_follow_speed': 0.04}, {'reverse_follow_speed': 0.005},
                {'reverse_wn': 0.0}, {'reverse_zeta': 2.0}, {'reverse_lost_cm': 0.0},
                {'reverse_max_heading_deg': 45.0}, {'reverse_short_cm': -1.0}):
        with pytest.raises(ValueError):
            NavConfig(**bad)


def test_reverse_speed_and_angular_capped(graph):
    """라인 후진 명령은 −reverse_follow_speed(실행 중 바꾼 값)·|ω| ≤ max_angular."""
    sim, ex = _to_case(graph, 'da', speed=0.02)       # 지연 0: sim.cmd = 실행부 명령
    ex.set_speed(reverse_speed=0.03)
    ex.back('N23')
    lin, ang = [], []

    def watch(s, e):
        if e.state == REVERSE and e._prim.get('mode') == 'line':
            lin.append(s.cmd[0])
            ang.append(s.cmd[1])
    _drive(sim, ex, watch=watch)
    assert ex.state == ARRIVED
    assert lin and all(v == pytest.approx(-0.03) for v in lin)
    assert all(abs(w) <= ex.cfg.max_angular + 1e-9 for w in ang)


def test_state_report_shape(graph):
    """테스트 콘솔이 쓰는 상태 JSON: pause·reverse 필드."""
    sim, ex = _to_case(graph, 'ga')
    st = ex.as_dict()
    assert st['state'] == 'paused' and st['pause']['settled'] is True and st['reverse'] is None
    ex.back('N23')
    assert sim.run(ex, lambda e: e.state == REVERSE and e.as_dict()['traveled_cm'] > 6)
    rv = ex.as_dict()['reverse']
    assert set(rv) == {'to', 'phase', 'from', 'leg', 'expected_cm', 'done_cm', 'y_cm',
                       'theta_deg', 'window_cm'}
    assert rv['to'] == 'N23' and rv['phase'] == 'reverse' and rv['leg'] == ['N23', 'N19']
    assert rv['from'] == 'follow' and rv['done_cm'] > 6 and rv['expected_cm'] > 20
    pe = ex.as_dict()['pose_est']
    assert graph.nodes['N23'].x < pe['x_cm'] < graph.nodes['N19'].x and pe['heading_deg'] == 0.0
    _drive(sim, ex)
    st = ex.as_dict()
    assert st['state'] == 'arrived' and st['route'] == ['N07', 'N23']
    assert st['leg']['to'] == 'N23' and st['reverse']['phase'] == 'done'
    kinds = [e['event'] for e in ex.events]
    for k in ('pause', 'paused_settled', 'back', 'reverse', 'reverse_node', 'reverse_arrived'):
        assert k in kinds, kinds
    assert 'node' not in kinds[kinds.index('back'):]     # 관제 trial_driver 는 node 이벤트로 진행을 셈


def test_no_pause_route_unchanged(graph):
    """pause·back 을 쓰지 않으면 기존 경로 주행과 같다(이벤트 종류에 새 이벤트 없음)."""
    sim, ex = _start(graph, delay=0.15)
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    assert ex.state == ARRIVED
    kinds = {e['event'] for e in ex.events}
    assert not kinds & {'pause', 'resume', 'back', 'reverse', 'reverse_node', 'reverse_short',
                        'paused_settled', 'reverse_arrived'}
    assert ex.as_dict()['reverse'] is None and ex.as_dict()['pause'] is None


# ---------------------------------------------------------------- 검토 반영(07_pinky-reviewer)
def _stop_graph():
    """N12 를 Nav2 전환 없는 가로선 정차(stop) 노드로 바꾼 시험용 그래프(지금 지도에는 stop 도착 노드가 없음)."""
    import yaml
    with open(LAYOUT, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    for it in data['nodes']['items']:
        if it['id'] == 'N12':
            it['nav2_switch'] = False
            it['arrival'] = 'stop'
    return Graph.from_dict(data)


def _to_end(graph, route):
    sim = Sim(graph)
    sim.place(route[0], graph.heading(route[0], route[1]), 0.0)
    ex = RouteExecutor(graph)
    ex.start(route)
    sim.run(ex, lambda e: not e.active, max_s=300.0)
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5.0)
    return sim, ex


def test_back_after_handoff_rejected(graph):
    """[중]1: N12 handoff 뒤는 Nav2 소유 → back 거절, 후보 없음."""
    sim, ex = _to_end(graph, ['N09', 'N08', 'N12'])
    assert ex.state == 'handoff'
    assert ex.back_targets() == []
    with pytest.raises(ValueError, match='Nav2'):
        ex.back('N08')


def test_back_after_stop_arrival_rules():
    """[중]1: 가로선 정차(stop)로 끝난 뒤는 허용. 끝난 뒤 stop(기억 지움)·odom 2 cm 넘게 이동·5° 넘게 회전이면 거절."""
    g = _stop_graph()
    sim, ex = _to_end(g, ['N09', 'N08', 'N12'])
    assert ex.state == ARRIVED and ex.legs[-1].action == 'stop'
    assert ex.back_targets() == ['N08', 'N09']
    ex.back('N08')
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.route == ['N09', 'N08'], ex.as_dict()

    sim, ex = _to_end(g, ['N09', 'N08', 'N12'])
    ex.stop()                                   # 끝난 뒤 stop = 위치를 잊음(사람이 옮길 수 있음)
    with pytest.raises(ValueError, match='forgotten'):
        ex.back('N08')
    for moved in ((3.0, 0.0), (0.0, 6.0)):      # odom 상 3 cm 이동 / 6° 회전(Nav2·손)
        sim, ex = _to_end(g, ['N09', 'N08', 'N12'])
        ox, oy, oyaw = sim.odom
        sim.odom = (ox + moved[0], oy, oyaw + math.radians(moved[1]))
        sim.x += moved[0]
        sim.yaw += math.radians(moved[1])
        assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5.0)
        assert ex.back_targets() == []
        with pytest.raises(ValueError, match='since arrival'):
            ex.back('N08')


@pytest.mark.parametrize('delay, odom', COMBOS)
@pytest.mark.parametrize('speed', (0.02, 0.03))
def test_turn_start_after_back_centers_axle(graph, speed, delay, odom):
    """[중]2: 후진 끝(N23, 축이 중심 약 2 cm 뒤)에서 바로 옆 분기 N21 로 회전 출발 → 회전 전 축 맞춤 → 완주."""
    sim, ex = _to_case(graph, 'ga', speed=speed, delay=delay, odom=odom)
    ex.back('N23')
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex._end.get('center') is not None
    ex.set_speed(base_speed=max(0.03, speed))
    ex.start(['N23', 'N21'])
    kinds = []
    while ex.active and sim.t < 600:
        sim.tick(ex)
        if ex.state == LOADING:
            ex.load_done()
    kinds = [e['event'] for e in ex.events]
    assert ex.state == ARRIVED, ex.reason
    assert 'pre_center' in kinds and 'line_found' in kinds
    n21 = graph.nodes['N21']
    assert math.hypot(sim.x - n21.x, sim.y - n21.y) < 6.0


def test_turn_start_without_back_unchanged(graph):
    """전진으로 끝난 기억(center 없음)에서의 회전 출발은 예전과 같음(pre_center 없음)."""
    sim, ex = _to_end(graph, ['N09', 'N08', 'N12'])
    ex.start(['N12', 'N08', 'N09', 'H1'])
    assert 'pre_center' not in [e['event'] for e in ex.events]


@pytest.mark.parametrize('delay, odom', COMBOS)
@pytest.mark.parametrize('speed', (0.02, 0.03))
def test_back_through_branch_tee_reacquires(graph, speed, delay, odom):
    """[중]3: 분기 N21 에서 올라와 T자 N23 에서 돈 뒤 되감기(직진 선 없음) → 센서가 테이프 밖에서 시작해도 다시 잡음."""
    sim = Sim(graph, cmd_delay_s=delay, odom_scale=odom)
    sim.place('N21', 'N', 0.0)
    ex = RouteExecutor(graph, NavConfig(reverse_follow_speed=speed))
    ex.start(['N21', 'N23', 'N19', 'N11'])
    assert sim.run(ex, _at(1, 20.0), max_s=300.0)
    _pause_settle(sim, ex)
    ex.back('N21')
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.route == ['N21'], ex.reason
    n21 = graph.nodes['N21']
    sx, sy = _sensor(sim)
    assert abs(sy - n21.y) <= IR_TOL_CM and abs(sim.x - n21.x) <= LAT_TOL_CM


@pytest.mark.parametrize('delay, odom', ((0.0, 1.0), (0.15, 0.95), (0.2, 1.05)))
def test_back_to_via_load_node(graph, delay, odom):
    """[중]3·[하]6: 경유 상차 N01 을 지나 N18(T자)에서 돈 뒤 back N01 → 도착, 줄인 경로 끝 구간은 stop 표시."""
    from roport_line_nav.status_indicator import classify
    route = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11']
    sim = Sim(graph, cmd_delay_s=delay, odom_scale=odom)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(route)
    assert sim.run(ex, lambda e: e.index == 7 and e.state == FOLLOW
                   and e.as_dict()['traveled_cm'] > 20, on_loading=lambda s: True)
    _pause_settle(sim, ex)
    assert ex.back_targets() == ['N07', 'N18', 'N01']
    ex.back('N01')
    _drive(sim, ex)
    st = ex.as_dict()
    assert st['state'] == ARRIVED and st['route'][-1] == 'N01', ex.reason
    assert st['leg']['action'] == 'stop' and classify(st).name == 'READY'
    assert sim.sensor.update(sim.raw()).node or sim.sensor.update(sim.raw()).on_line[1]


def test_map_offset_reset_per_route(graph):
    """[하]5: 지도 방향 차이(odom − 지도)는 경로마다 새로."""
    sim, ex = _start(graph)
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    assert ex._map_off is not None
    ex.start(['N11', 'N19', 'N23', 'N07', 'N18', 'N01'])
    assert ex._map_off is None


def test_dock_display_counts_last_undo_move(graph):
    """[하]8: (다′) 도킹 되돌림의 마지막 −a 도 표시용 누적값에 더함(되돌린 뒤 ≈ 0)."""
    from roport_line_nav.dock import DockTable
    dock = DockTable(graph)
    dock.set('N11', -3.0, 90.0, 2.0)
    sim, ex = _start(graph)
    ex.dock = dock
    assert sim.run(ex, lambda e: e.state == LOADING)
    a, b = ex._dock_disp
    assert abs(a + 3.0) < 0.5 and abs(b - 2.0) < 0.5
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5)
    ex.back('N19')
    assert sim.run(ex, lambda e: e.state == REVERSE)
    a, b = ex._dock_disp
    assert abs(a) < 0.5 and abs(b) < 0.5, ex._dock_disp


def test_back_targets_report(graph):
    """[하]7: line_nav/state 의 back_targets = 지금 back 으로 받을 노드(최근 순), 아니면 []."""
    sim, ex = _start(graph)
    assert sim.run(ex, _at(1, 25.0))
    assert ex.as_dict()['back_targets'] == []          # 주행 중
    ex.pause()
    sim.tick(ex)
    assert ex.as_dict()['back_targets'] == []          # 정지 확인 전
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5)
    assert ex.as_dict()['back_targets'] == ['N23', 'N07']
    assert sim.run(ex, lambda e: e.state == PAUSED)
    ex.resume()
    assert sim.run(ex, lambda e: e.state == LOADING)
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5)
    assert ex.as_dict()['back_targets'] == ['N19', 'N23', 'N07']
    ex.back('N23')
    assert ex.as_dict()['back_targets'] == []          # 후진 중
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.as_dict()['back_targets'] == []   # 이미 후진함
    ex.start(AGAIN)
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    for _ in range(20):
        sim.tick(ex)
    assert ex.as_dict()['back_targets'] == []          # 상차 뒤 되돌아 선 끝


@pytest.mark.parametrize('dx, dyaw', [(20.0, 0.0), (3.0, 0.0), (0.0, 8.0)])
def test_turn_start_after_back_rejected_if_moved(graph, dx, dyaw):
    """재검토 R1: 후진 도착 뒤 stop 없이 odom 상 옮겨졌으면(20 cm·3 cm·8°) 회전 출발 거절, 움직이지 않음, 기억 버림."""
    sim, ex = _to_case(graph, 'ga')
    ex.back('N23')
    _drive(sim, ex)
    assert ex.state == ARRIVED
    sim.x += dx
    sim.yaw += math.radians(dyaw)
    ox, oy, oyaw = sim.odom
    sim.odom = (ox + dx, oy, oyaw + math.radians(dyaw))
    for _ in range(10):
        sim.tick(ex)
    with pytest.raises(GraphError, match='since reverse arrival'):
        ex.start(['N23', 'N21'])
    assert ex.state == ARRIVED and ex.route == ['N07', 'N23'] and ex._end is None
    assert ex.step(sim.sensor.update(sim.raw()), sim.pose(), sim.t) is None   # 명령 없음
    ex.start(['N23', 'N21'])           # 기억이 없으면 예전 회전 출발(보정 없음)
    assert 'pre_center' not in [e['event'] for e in ex.events]


def test_pre_center_move_limit(graph):
    """재검토 R1: 노드 중심까지 이동이 ir_ahead − 반폭 + 1 cm 를 넘으면 계획 단계에서 거절(실행 중에도 같은 상한)."""
    sim, ex = _to_case(graph, 'ga')
    ex.back('N23')
    _drive(sim, ex)
    cx, cy = ex._end['center']
    ex._end["center"] = (cx - 6.0, cy)        # 기억한 중심이 6 cm 뒤라고 가정(이동 약 −4 cm)
    with pytest.raises(GraphError, match='exceeds'):
        ex.start(['N23', 'N21'])
    assert ex.state == ARRIVED and ex._end is None


# ---------------------------------------------------------------- 실로봇 후속(08 A): 이어 출발 때 지나온 길 잇기
# 실로봇 시험 1(2026-11-12, d660): P5(N16) → N01 출발, N18→N01 10.3 cm 에서 pause → back N18(남쪽 보기).
# 그 뒤 「현재」(N18) → N01 로 다시 출발하면 직전 경로의 지나온 길(N16→N14→N02→N10→N18)을 앞에 이어 붙여
# back 이 N10 등까지 되감을 수 있어야 한다(되감기 규칙·판정 기준은 그대로).
P5_N01 = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']


def _test1(graph, speed=0.02, delay=0.0, odom=1.0):
    """시험 1 재현: N18→N01 10.3 cm 에서 pause → back N18 → ARRIVED(N18, 남쪽)."""
    sim = Sim(graph, cmd_delay_s=delay, odom_scale=odom)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, NavConfig(reverse_follow_speed=speed))
    ex.start(P5_N01)
    assert sim.run(ex, _at(4, 10.3), max_s=400.0), ex.as_dict()
    _pause_settle(sim, ex)
    ex.back('N18')
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.route == P5_N01[:5], ex.as_dict()
    return sim, ex


def _pose_on(graph, sim, node, heading):
    """(센서 − 노드 진행 방향 cm, 축 옆 cm, 방향 °) — heading 방향 기준."""
    n = graph.nodes[node]
    h = math.radians({'E': 0.0, 'N': 90.0, 'W': 180.0, 'S': -90.0}[heading])
    sx, sy = _sensor(sim)
    along = (sx - n.x) * math.cos(h) + (sy - n.y) * math.sin(h)
    lat = -(sim.x - n.x) * math.sin(h) + (sim.y - n.y) * math.cos(h)
    return along, lat, _deg(sim.yaw - h)


def _back_n10_and_again(graph, sim, ex):
    ex.back('N10')
    _drive(sim, ex)
    out = (ex.state, ex.reason, list(ex.as_dict()['route']), _pose_on(graph, sim, 'N10', 'E'))
    again = None
    if ex.state == ARRIVED:
        ex.start(['N10', 'N18', 'N01'])
        sim.run(ex, lambda e: not e.active, max_s=sim.t + 400.0, on_loading=lambda s: True)
        n01 = graph.nodes['N01']
        again = (ex.state, ex.reason, math.hypot(sim.x - n01.x, sim.y - n01.y))
    return out, again


@pytest.mark.parametrize('delay, odom', COMBOS)
@pytest.mark.parametrize('speed', (0.02, 0.03))
def test_chain_start_back_to_previous_route(graph, speed, delay, odom):
    """시험 1 → 「현재」 N18 → N01 출발 → N18→N01 10 cm pause → back N10 = 한 경로에서 바로 back N10 한 (다) 와 같은 끝 자세,
    이어서 N10 → N18 → N01 완주. 보고하는 route·index 는 back 전까지 명령한 경로 그대로(관제 trial_driver 비교 유지)."""
    # 기준 (다): 같은 조건 한 경로 안에서 N18→N01 10.3 cm pause → back N10
    sim, ex = Sim(graph, cmd_delay_s=delay, odom_scale=odom), None
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph, NavConfig(reverse_follow_speed=speed))
    ex.start(P5_N01)
    assert sim.run(ex, _at(4, 10.3), max_s=400.0)
    _pause_settle(sim, ex)
    ref, ref_again = _back_n10_and_again(graph, sim, ex)
    assert ref[0] == ARRIVED, ref

    sim, ex = _test1(graph, speed, delay, odom)
    st = ex.as_dict()
    assert st['end'] == {'node': 'N18', 'facing': 'S', 'moved': False,
                         'passed': ['N16', 'N14', 'N02', 'N10']}
    ex.start(['N18', 'N01'])
    st = ex.as_dict()
    assert st['route'] == ['N18', 'N01'] and st['index'] == 0 and st['end'] is None
    assert st['route_event_seq0'] <= ex.event_seq
    assert 'chain' in [e['event'] for e in ex.events]
    assert sim.run(ex, lambda e: e.state == FOLLOW and e.as_dict()['traveled_cm'] >= 10.0,
                   max_s=sim.t + 200.0)
    _pause_settle(sim, ex)
    st = ex.as_dict()
    assert st['back_targets'] == ['N18', 'N10', 'N02', 'N14', 'N16']
    assert st['route'] == ['N18', 'N01'] and st['index'] == 0 and st['leg']['from'] == 'N18'
    out, again = _back_n10_and_again(graph, sim, ex)
    assert out[0] == ARRIVED and out[2] == ['N16', 'N14', 'N02', 'N10'], out
    along, lat, yaw = out[3]
    assert abs(along) <= IR_TOL_CM and abs(lat) <= LAT_TOL_CM and abs(yaw) <= YAW_TOL_DEG, out
    r_along, r_lat, r_yaw = ref[3]
    assert abs(along - r_along) <= 0.5 and abs(lat - r_lat) <= 0.5 and abs(yaw - r_yaw) <= 1.0, \
        (out, ref)
    assert again is not None and again[0] == ARRIVED and again[2] < 6.0, again
    assert ref_again[0] == ARRIVED


@pytest.mark.parametrize('how', ('stop', 'moved_3cm', 'turned_6deg', 'odom_lost'))
def test_no_chain_when_end_memory_invalid(graph, how):
    """끝 기억이 무효(끝난 뒤 stop·odom 2 cm/5° 넘게 이동·odom 끊김)면 이어 붙이지 않음 → 후보는 이번 경로만, back N10 거절."""
    sim, ex = _test1(graph, delay=0.15)
    if how == 'stop':
        ex.stop()
        assert ex.as_dict()['end'] is None
    elif how == 'odom_lost':
        ex.step(sim.sensor.update(sim.raw()), None, sim.t)
        st = ex.as_dict()['end']
        assert st['moved'] is None and st['passed'] == []
    else:
        dx, dyaw = (3.0, 0.0) if how == 'moved_3cm' else (0.0, 6.0)
        sim.y += dx                       # 남쪽을 보고 선 채 옆(북쪽)으로 3 cm / 제자리 6°
        sim.yaw += math.radians(dyaw)
        ox, oy, oyaw = sim.odom
        sim.odom = (ox, oy + dx, oyaw + math.radians(dyaw))
        for _ in range(10):
            sim.tick(ex)
        st = ex.as_dict()['end']
        assert st['moved'] is True and st['passed'] == [], st
        if how == 'moved_3cm':       # 시험에서 다시 선 위로(축 옆 오차 없이 이어 출발 확인)
            sim.y -= dx
            sim.odom = (ox, oy + dx, oyaw)
            for _ in range(10):
                sim.tick(ex)
    ex.start(['N18', 'N01'])
    assert 'chain' not in [e['event'] for e in ex.events]
    assert ex._base == 0 and ex.route == ['N18', 'N01']
    if how == 'odom_lost':
        return                            # odom 끊김: 첫 샘플에서 no_odom 정지(기존 동작)
    assert sim.run(ex, lambda e: e.state == FOLLOW and e.as_dict()['traveled_cm'] >= 10.0,
                   max_s=sim.t + 200.0)
    _pause_settle(sim, ex)
    assert ex.as_dict()['back_targets'] == ['N18']
    with pytest.raises(ValueError, match='not a node passed'):
        ex.back('N10')
    assert ex.state == PAUSED and sim.cmd == (0.0, 0.0)


def test_chain_does_not_cross_via_stop(graph):
    """경유 상차(N01)를 지난 경로에서 back N07 → 「현재」 N07 → N11 출발 → 이어 붙여도 후보는 N01 까지, N10 은 거절."""
    route = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11']
    sim = Sim(graph, cmd_delay_s=0.15)
    sim.place('N16', 'S', 0.0)
    ex = RouteExecutor(graph)
    ex.start(route)
    assert sim.run(ex, _at(7, 20.0), max_s=600.0, on_loading=lambda s: True)
    _pause_settle(sim, ex)
    ex.back('N07')
    _drive(sim, ex)
    assert ex.state == ARRIVED, ex.reason
    assert ex.as_dict()['end']['passed'] == ['N01', 'N18']
    ex.start(['N07', 'N23', 'N19', 'N11'])
    assert ex.as_dict()['route'] == ['N07', 'N23', 'N19', 'N11']
    assert sim.run(ex, lambda e: e.state == FOLLOW and e.legs[e.index].frm == 'N07'
                   and e.as_dict()['traveled_cm'] >= 15.0, max_s=sim.t + 300.0)
    _pause_settle(sim, ex)
    assert ex.as_dict()['back_targets'] == ['N07', 'N18', 'N01']
    with pytest.raises(ValueError, match='not a node passed'):
        ex.back('N10')
    ex.back('N18')                       # 이음 노드 N07 은 직진 → 지나쳐 N18(경유 뒤 첫 노드)까지
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.as_dict()['route'][-1] == 'N18', ex.reason


def test_chain_turn_start_and_uturn(graph):
    """이음 노드 회전: N18(남쪽 보기) → N07 출발은 직전 진입(동쪽)과 같은 방향 = 직진 이음 → back N10 이 N18 을 지나쳐 도착.
    N18 → N10(서쪽) 출발은 직전 진입 방향의 U턴이라 이어 붙이지 않음."""
    sim, ex = _test1(graph, delay=0.15)
    ex.start(['N18', 'N10', 'N02', 'N14', 'N16'])
    assert ex._base == 0 and 'chain' not in [e['event'] for e in ex.events]
    ex.stop()

    sim, ex = _test1(graph, delay=0.15)
    ex.start(['N18', 'N07', 'N23', 'N19', 'N11'])
    kinds = [e['event'] for e in ex.events]
    assert 'chain' in kinds and 'pre_center' in kinds
    assert ex.legs[ex._base - 1].action == 'straight'
    assert sim.run(ex, lambda e: e.state == FOLLOW and e.legs[e.index].frm == 'N18'
                   and e.as_dict()['traveled_cm'] >= 15.0, max_s=sim.t + 300.0)
    _pause_settle(sim, ex)
    assert ex.as_dict()['back_targets'] == ['N18', 'N10', 'N02', 'N14', 'N16']
    ex.back('N10')
    _drive(sim, ex)
    assert ex.state == ARRIVED and ex.route == ['N16', 'N14', 'N02', 'N10'], ex.reason
    kinds = [e['event'] for e in ex.events]
    last_back = len(kinds) - 1 - kinds[::-1].index('back')
    assert kinds[last_back:].count('reverse_node') == 1     # N18 은 멈추지 않고 지남
    along, lat, yaw = _pose_on(graph, sim, 'N10', 'E')
    assert abs(along) <= IR_TOL_CM and abs(lat) <= LAT_TOL_CM and abs(yaw) <= YAW_TOL_DEG


def test_chain_not_for_cross_or_offset_start(graph):
    """교차선 출발·출발 오프셋 지정 = 사람이 놓은 출발로 봄 → 이어 붙이지 않음."""
    sim, ex = _test1(graph)
    ex.start(['N18', 'N01'], start_offset_cm=2.0)
    assert ex._base == 0
    ex.stop()
    sim, ex = _test1(graph)
    ex.start(['N18', 'N07', 'N23', 'N19', 'N11'], start_cross=True, start_facing='S')
    assert ex._base == 0 and 'chain' not in [e['event'] for e in ex.events]


def test_end_report_after_forward_arrivals(graph):
    """end 보고: 주행 중 null, 전진으로 끝난 상차·handoff 는 node·facing 만(가로선 정차 노드라 passed 없음)."""
    sim, ex = _start(graph)
    assert ex.as_dict()['end'] is None
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    for _ in range(10):
        sim.tick(ex)
    assert ex.as_dict()['end'] == {'node': 'N11', 'facing': 'N', 'moved': False, 'passed': []}   # 상차 뒤 되돌아 섬
    ex.stop()
    assert ex.as_dict()['end'] is None
    sim, ex = _to_end(graph, ['N09', 'N08', 'N12'])
    e = ex.as_dict()['end']
    assert e['node'] == 'N12' and e['passed'] == []


@pytest.mark.parametrize('how', ('odom_3cm', 'odom_lost'))
def test_moved_end_memory_dropped_on_straight_start(graph, how):
    """결정 1(2026-11-12): 끝난 뒤 odom 이 2 cm 넘게 바뀌었거나 odom 을 모르면 같은 노드 직진 출발이라도 끝 기억을 버리고
    손으로 놓은 출발과 같게(거절 안 함): s0 모름 → 첫 구간 창 최소 0·gate_early 없음, pre_center·이어 붙이기 없음."""
    sim, ex = _test1(graph, delay=0.15)
    if how == 'odom_3cm':                 # 로봇은 그대로, odom 만 3 cm 바뀜(미끄러짐·조종 흉내)
        ox, oy, oyaw = sim.odom
        sim.odom = (ox, oy + 3.0, oyaw)
        for _ in range(10):
            sim.tick(ex)
        assert ex.as_dict()['end']['moved'] is True
        ex.start(['N18', 'N01'])
    else:
        ex.step(sim.sensor.update(sim.raw()), None, sim.t)
        assert ex.as_dict()['end']['moved'] is None
        ex.start(['N18', 'N01'])          # 출발 순간 odom 모름 → 기억 버림(odom 은 다음 샘플부터 다시 들어옴)
    kinds = [e['event'] for e in ex.events]
    assert 'end_dropped' in kinds and 'chain' not in kinds and 'pre_center' not in kinds
    assert ex._end is None and ex._base == 0
    assert ex._leg_guess and ex._expected is None and ex._min_ok == 0.0   # 손으로 놓은 출발과 같은 창
    sim.run(ex, lambda e: not e.active, max_s=sim.t + 300.0, on_loading=lambda s: True)
    assert ex.state == ARRIVED, ex.reason


def test_unmoved_end_memory_kept_on_straight_start(graph):
    """대조: 끝난 자세 그대로면 끝 기억(s0 = 반폭, gate_early)을 쓰고 end_dropped 없음."""
    sim, ex = _test1(graph, delay=0.15)
    ex.start(['N18', 'N01'])
    assert 'end_dropped' not in [e['event'] for e in ex.events]
    assert not ex._leg_guess and ex._expected is not None and ex._min_ok > 0.0


# ---------------------------------------------------------------- 실로봇 정지 확인 잡음(2026-11-12)
def _jitter_hold(ex, sim, seconds, yaw_tick=0.0018, x_tick=0.017, every=4, drift_cm_s=0.0):
    """일시정지 뒤 실로봇처럼 odom 이 가끔 1틱씩 튀는 입력(20 Hz)을 넣는다. → settled·back_targets 기록."""
    x, y, yaw = sim.pose()
    t = sim.t
    seen = []
    for n in range(int(seconds * 20)):
        t += 0.05
        j = 1 if (n // every) % 2 else 0          # every 샘플마다 1틱 올라갔다 내려옴
        p = (x + j * x_tick + drift_cm_s * (n * 0.05), y, yaw + j * yaw_tick)
        ex.step(_reading((False, True, False)), p, t, pose_stamp=t)
        seen.append((ex.settled, tuple(ex.back_targets())))
    return seen


def test_settled_survives_encoder_tick_jitter(graph):
    """실로봇: 서 있어도 엔코더 1틱(yaw 0.0018 rad = 메시지 간 0.036 rad/s)이 튀면 예전엔 정지 확인이 풀려
    back 후보가 비었다. 창(0.3 s) 안 자세 변화로 보면 정지 유지·후보 유지."""
    sim, ex = _to_case(graph, 'da')
    seen = _jitter_hold(ex, sim, 3.0)
    assert all(s for s, _ in seen[10:]), [s for s, _ in seen]
    assert all(c and c[0] == 'N19' and 'N23' in c for _, c in seen[10:])
    assert ex.as_dict()['back_why'] is None


def test_settled_false_when_really_creeping(graph):
    """창 안에서 실제로 0.5 cm/s 로 밀리면 정지로 보지 않고, back_why 가 이유를 보여 준다."""
    sim, ex = _to_case(graph, 'da')
    seen = _jitter_hold(ex, sim, 2.0, drift_cm_s=0.5)
    assert not any(s for s, _ in seen[10:])
    why = ex.as_dict()['back_why']
    assert why and 'still moving' in why
    assert ex.as_dict()['back_targets'] == []
