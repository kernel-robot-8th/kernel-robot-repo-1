"""장애물 정지·대기(FFT1-172 범위 변경, 09 계약. 양보 후진은 2026-11-15 삭제). 2D 시뮬(test/sim.py 라이다)과 합성 입력으로만 확인한다.

실로봇 미검증. 시연용 소프트웨어 정지이며 안전 인증 장치가 아니다.
시뮬 라이다: 세계 좌표 원형 물체 + 바깥 벽(444.5 × 221 cm) → 광선 거리(scan frame, 라이다 yaw = π 장착) →
노드와 같은 obstacle.edge_distances(TF) → executor.set_scan.

예시 경로 = N07(교차선 출발, 동쪽) → N23(직진 통과) → N19(우회전) → N11(상차). 물체는 N23→N19 선(y = 73.2) 위에 둔다.
"""

import math
import os

import pytest
from sim import Sim

from roport_line_nav.executor import (ARRIVED, FOLLOW, PASS, PAUSED, STOPPED, NavConfig,
                                      RouteExecutor)
from roport_line_nav.graph import Graph
from roport_line_nav.obstacle import edge_distances, planar_tf, tf_from_quaternion

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')
ROUTE = ['N07', 'N23', 'N19', 'N11']
R_OBJ = 1.5            # 시험 물체 반지름 cm
# 판정 논리 시험은 값을 직접 준다(기본값은 2026-11-15 실로봇 1차로 stop 5·blind 3·상차 노드 면제로 바뀜 —
# test_config_defaults_and_limits·test_exempt_* 가 기본값을 본다). 25/15 는 2D 시뮬 시나리오(물체 거리)에 맞춘 값
LOGIC_CFG = {'stop_cm': 25.0, 'blind_near_cm': 15.0, 'obstacle_exempt_nodes': ()}
OBS_EVENTS = ('obstacle_stop', 'obstacle_clear', 'obstacle_warn')


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def _y(graph):
    return graph.nodes['N23'].y


class Run:
    """시뮬 + 실행부 + 이벤트·명령 기록. obs = 지금 물체 목록(바꾸면 다음 scan 부터 반영)."""

    def __init__(self, graph, mode='stop', obs=(), delay=0.0, odom=1.0, lidar=True, **cfg):
        self.graph = graph
        self.sim = Sim(graph, cmd_delay_s=delay, odom_scale=odom)
        self.sim.place('N07', 'E', 0.0)
        self.obs = list(obs)
        if lidar:
            self.sim.enable_lidar(lambda t: self.obs)
        self.ex = RouteExecutor(graph, NavConfig(obstacle_mode=mode, **{**LOGIC_CFG, **cfg}))
        self.ex.start(ROUTE, start_cross=True, start_facing='E')
        self.events = []
        self.cmds = []
        self.nodes = []        # (노드 이벤트 detail, 센서 x, 센서 y)
        self.poses = []
        self._seen = self.ex.event_seq
        self.move = None       # 매 tick 전에 부를 함수(sim, run)

    def tick(self):
        if self.move is not None:
            self.move(self.sim, self)
        self.sim.tick(self.ex)
        self.cmds.append(self.sim.cmd)
        self.poses.append((round(self.sim.x, 6), round(self.sim.y, 6), round(self.sim.yaw, 6)))
        if self.ex.state == 'loading':
            self.ex.load_done()
        new = self.ex.event_seq - self._seen
        if new:
            for e in list(self.ex.events)[-new:]:
                self.events.append(e)
                if e['event'] == 'node':
                    self.nodes.append((e['detail'],) + self.sensor())
            self._seen = self.ex.event_seq

    def until(self, cond, max_s=60.0):
        end = self.sim.t + max_s
        while self.sim.t < end:
            self.tick()
            if cond(self.ex):
                return True
        return False

    def finish(self, max_s=400.0):
        self.until(lambda e: not e.active, max_s)
        return self.ex.state

    def sensor(self):
        s = self.sim
        return s.x + s.ir_ahead * math.cos(s.yaw), s.y + s.ir_ahead * math.sin(s.yaw)

    def kinds(self, only=OBS_EVENTS):
        return [e['event'] for e in self.events if e['event'] in only]

    def front_gap(self):
        """실제 몸체 앞 끝(축 + 4.2) ↔ 첫 물체 표면 거리(cm, 동쪽 진행 기준)."""
        x = self.sim.x + 4.2 * math.cos(self.sim.yaw)
        return self.obs[0][0] - R_OBJ - x


def _held(e):
    return e.state == PAUSED and e.as_dict()['pause_reason'] == 'obstacle'


def _baseline(graph, **kw):
    r = Run(graph, mode='off', lidar=False, **kw)
    assert r.finish() == ARRIVED
    return r


# ---------------------------------------------------------------- 계산(TF·부채꼴)
def _scan_with(points_base_cm, mount=(-1.7, 0.0, math.pi), step_deg=1.0):
    """base_link 점(cm) → 라이다 scan(ranges m, scan frame). 각 점은 가장 가까운 광선 하나에만."""
    n = int(round(360 / step_deg))
    inc = math.radians(step_deg)
    ranges = [math.inf] * n
    mx, my, myaw = mount
    for px, py in points_base_cm:
        dx, dy = px - mx, py - my
        r = math.hypot(dx, dy) / 100.0
        a = math.atan2(dy, dx) - myaw
        a = (a + math.pi) % (2 * math.pi) - math.pi
        i = int(round((a + math.pi) / inc)) % n
        ranges[i] = r
    return ranges, -math.pi, inc


def test_edge_distances_use_tf_not_raw_angles():
    """라이다 yaw = π: 로봇 앞 30 cm 점은 scan frame 각도 π. TF 를 쓰면 front = 30 − 4.2, 원시 각도면 뒤로 보임."""
    ranges, amin, inc = _scan_with([(30.0, 0.0), (-20.0, 0.0)])
    tf = planar_tf(-0.017, 0.0, math.pi)
    front, rear = edge_distances(ranges, amin, inc, 0.15, 12.0, tf, 20.0, 4.2, 7.5)
    assert front == pytest.approx(25.8, abs=0.05) and rear == pytest.approx(12.5, abs=0.05)
    raw = edge_distances(ranges, amin, inc, 0.15, 12.0, planar_tf(0, 0, 0), 20.0, 4.2, 7.5)
    assert raw[0] == pytest.approx(18.3 - 4.2, abs=0.05)    # 뒤 점이 앞으로 — 원시 각도는 틀림
    q = tf_from_quaternion(-0.017, 0.0, 0.0, 0.0, 1.0, 0.0)  # yaw π 쿼터니언(z = 1, w = 0)
    assert all(a == pytest.approx(b, abs=1e-12) for a, b in zip(q, tf))


def test_edge_distances_sector_and_validity():
    tf = planar_tf(-0.017, 0.0, math.pi)
    side = [(40 * math.cos(math.radians(a)), 40 * math.sin(math.radians(a))) for a in (19.0, 21.0)]
    ranges, amin, inc = _scan_with(side)
    front, rear = edge_distances(ranges, amin, inc, 0.15, 12.0, tf, 20.0, 4.2, 7.5)
    assert front == pytest.approx(40 * math.cos(math.radians(19.0)) - 4.2, abs=0.6) and rear is None
    ranges, amin, inc = _scan_with([(5.0, 0.0), (30.0, 0.0)])    # 5 cm 점 = range_min(15) 안 → 무효
    ranges[0] = float('nan')
    front, _ = edge_distances(ranges, amin, inc, 0.15, 12.0, tf, 20.0, 4.2, 7.5)
    assert front is None                                          # 같은 광선이라 둘 다 무효(가림)
    assert edge_distances([math.inf] * 360, -math.pi, inc, 0.15, 12.0, tf, 20.0, 4.2, 7.5) == \
        (None, None)


@pytest.mark.parametrize('deg', (179.0, -179.0, 161.0, -161.0))
def test_rear_sector_wraps_at_pi(deg):
    """뒤 부채꼴은 ±π 둘레(θ ≈ ±179°)를 감아 본다. ±161° 는 안, ±159° 는 밖."""
    tf = planar_tf(-0.017, 0.0, math.pi)
    a = math.radians(deg)
    ranges, amin, inc = _scan_with([(20.0 * math.cos(a), 20.0 * math.sin(a))])
    front, rear = edge_distances(ranges, amin, inc, 0.15, 12.0, tf, 20.0, 4.2, 7.5)
    assert front is None and rear == pytest.approx(-20.0 * math.cos(a) - 7.5, abs=0.4)
    a = math.radians(math.copysign(159.0, deg))
    ranges, amin, inc = _scan_with([(20.0 * math.cos(a), 20.0 * math.sin(a))])
    assert edge_distances(ranges, amin, inc, 0.15, 12.0, tf, 20.0, 4.2, 7.5) == (None, None)


def test_sim_scan_matches_mount(graph):
    """시뮬 라이다(yaw π 장착)가 동쪽을 보는 로봇 앞·뒤 물체를 몸체 끝 거리로 준다."""
    sim = Sim(graph)
    sim.place('N23', 'E', 0.0)
    y = _y(graph)
    sim.enable_lidar([(sim.x + 4.2 + 20.0 + R_OBJ, y, R_OBJ), (sim.x - 7.5 - 12.0 - R_OBJ, y, R_OBJ)])
    front, rear = sim.edges()
    assert front == pytest.approx(20.0, abs=0.3) and rear == pytest.approx(12.0, abs=0.3)


def test_config_defaults_and_limits():
    c = NavConfig()
    assert c.obstacle_mode == 'off' and not hasattr(c, 'gap_max_cm')
    assert (c.stop_cm, c.clear_margin_cm, c.resume_delay_s, c.blind_near_cm, c.sector_deg,
            c.front_edge_cm, c.rear_edge_cm, c.scan_max_age_s, c.warn_after_s) == \
        (5.0, 5.0, 3.0, 3.0, 20.0, 4.2, 7.5, 0.5, 30.0)          # 2026-11-15 실로봇 1차 결정
    assert c.obstacle_exempt_nodes == ('N01', 'N11', 'N21', 'N22')
    for bad in ({'obstacle_mode': 'on'}, {'stop_cm': 0.5}, {'stop_cm': float('nan')},
                {'blind_near_cm': 30.0}, {'sector_deg': 0.0}, {'scan_max_age_s': 0.0},
                {'warn_after_s': 0.0}, {'blind_near_cm': -1.0}):
        with pytest.raises(ValueError):
            NavConfig(**bad)


# ---------------------------------------------------------------- off·monitor = 동작 그대로
def test_off_mode_ignores_scan(graph):
    """기본 off: 라이다·물체가 있어도 궤적이 라이다 없는 주행과 같다(2D 시뮬·가상 관제 시험 영향 없음)."""
    base = _baseline(graph, delay=0.15)
    r = Run(graph, mode='off', obs=[(200.0, _y(graph), R_OBJ)], delay=0.15)
    assert r.finish() == ARRIVED
    assert r.poses == base.poses and r.kinds() == []
    ob = r.ex.as_dict()['obstacle']
    assert ob['mode'] == 'off' and ob['phase'] == 'clear' and ob['why'] is None


def test_monitor_mode_reports_without_changing_commands(graph):
    base = _baseline(graph, delay=0.15)
    r = Run(graph, mode='monitor', obs=[(200.0, _y(graph), R_OBJ)], delay=0.15)
    whys = []
    r.move = lambda s, run: whys.append(run.ex.as_dict()['obstacle']['why'])
    assert r.finish() == ARRIVED
    assert r.cmds == base.cmds and r.poses == base.poses and r.kinds() == []
    assert any(w and w.startswith('monitor: would stop (front') for w in whys)
    st = r.ex.as_dict()
    assert st['obstacle']['front_cm'] is not None and st['pause_reason'] is None


# ---------------------------------------------------------------- 정지·재개
@pytest.mark.parametrize('delay, odom', [(0.0, 1.0), (0.2, 1.05), (0.15, 0.95)])
def test_stop_resume_keeps_node_counting(graph, delay, odom):
    base = _baseline(graph, delay=delay, odom=odom)
    r = Run(graph, obs=[(215.0, _y(graph), R_OBJ)], delay=delay, odom=odom)
    assert r.until(_held), r.ex.as_dict()
    st = r.ex.as_dict()
    assert st['obstacle']['phase'] == 'blocked' and st['pause']['from'] == FOLLOW
    assert st['leg']['from'] == 'N23' and st['index'] == 1
    d_hold = st['traveled_cm']
    assert r.until(lambda e: e.settled, 5.0)
    assert r.front_gap() >= 20.0, r.front_gap()                  # 25 cm 에서 멈추기 시작(지연 밀림 포함)
    for _ in range(100):                                          # 5 s 그대로 막힘
        r.tick()
    assert r.ex.state == PAUSED and set(r.cmds[-100:]) == {(0.0, 0.0)}
    assert r.ex.as_dict()['traveled_cm'] == pytest.approx(d_hold, abs=1.0)
    t_clear = r.sim.t
    r.obs.clear()                                                 # 물체 치움 → 벽만 보임
    assert r.until(lambda e: e.state == FOLLOW, 6.0)
    waited = r.sim.t - t_clear
    assert 3.0 <= waited <= 3.0 + 0.2, waited
    assert r.finish() == ARRIVED, r.ex.reason
    assert r.kinds() == ['obstacle_stop', 'obstacle_clear']
    assert [n[0] for n in r.nodes] == [n[0] for n in base.nodes]
    for got, ref in zip(r.nodes, base.nodes):                     # 노드 감지 위치(센서)가 그대로
        assert math.hypot(got[1] - ref[1], got[2] - ref[2]) < 1.0, (got, ref)


def test_resume_wait_restarts_when_object_returns(graph):
    r = Run(graph, obs=[(215.0, _y(graph), R_OBJ)])
    assert r.until(_held)
    keep = list(r.obs)
    r.obs.clear()
    assert r.until(lambda e: e.as_dict()['obstacle']['phase'] == 'resume_wait', 1.0)
    r.until(lambda e: False, 1.5)
    r.obs.extend(keep)                                            # 3 s 전에 다시 들어옴 → 타이머 처음부터
    assert r.until(lambda e: e.as_dict()['obstacle']['phase'] == 'blocked', 1.0)
    r.until(lambda e: False, 4.0)
    assert r.ex.state == PAUSED
    r.obs.clear()
    assert r.finish() == ARRIVED


def test_stop_while_passing_straight_node(graph):
    """PASS(직진 노드 통과) 중에도 정지 → 재개 뒤 통과 거리·다음 노드 그대로. PASS 중 정지는 양보 후진 안 함."""
    base = _baseline(graph)
    r = Run(graph, obs=[(190.0, _y(graph), R_OBJ)])
    assert r.until(_held)
    assert r.ex.as_dict()['pause']['from'] == PASS
    r.until(lambda e: False, 2.0)
    r.obs.clear()
    assert r.finish() == ARRIVED, r.ex.reason
    assert [n[0] for n in r.nodes] == [n[0] for n in base.nodes]
    for got, ref in zip(r.nodes, base.nodes):
        assert math.hypot(got[1] - ref[1], got[2] - ref[2]) < 1.0, (got, ref)


def test_front_without_points_is_not_clear(graph):
    """막힌 뒤 앞 부채꼴에 점이 없으면(range_min 안에 붙은 물체 등) 비었다고 보지 않아 재개 안 함."""
    r = Run(graph, obs=[(215.0, _y(graph), R_OBJ)])
    assert r.until(_held)
    r.sim.lidar['room'] = None                                    # 벽도 안 보임 → 앞 점 없음
    r.obs.clear()
    r.until(lambda e: False, 6.0)
    st = r.ex.as_dict()
    assert st['state'] == PAUSED and st['obstacle']['front_cm'] is None
    assert 'no valid points' in st['obstacle']['why']


# ---------------------------------------------------------------- scan 끊김 fail-safe
def test_scan_stale_fail_safe(graph):
    r = Run(graph)
    assert r.until(lambda e: e.state == FOLLOW and e.index == 1
                   and e.as_dict()['traveled_cm'] > 10.0)
    t0 = r.sim.t
    r.sim.lidar['on'] = False
    assert r.until(lambda e: e.state == PAUSED, 2.0)
    st = r.ex.as_dict()
    assert st['pause_reason'] == 'scan_stale' and st['obstacle']['scan_age_s'] >= 0.5
    assert r.sim.t - t0 <= 0.5 + 0.1 + 0.1                        # 마지막 scan + 0.5 s 안(+ tick)
    for _ in range(40):
        r.tick()
    assert r.cmds[-1] == (0.0, 0.0)
    r.sim.lidar['on'] = True
    r.sim.lidar['next'] = 0.0
    assert r.until(lambda e: e.state == FOLLOW, 4.0)
    assert r.finish() == ARRIVED


def test_no_scan_at_all_holds_at_first_follow(graph):
    r = Run(graph, lidar=False)
    assert r.until(lambda e: e.state == PAUSED, 5.0)
    assert r.ex.as_dict()['pause_reason'] == 'scan_stale'
    assert r.ex.as_dict()['obstacle']['scan_age_s'] is None
    x0 = r.sim.x
    for _ in range(100):
        r.tick()
    assert abs(r.sim.x - x0) < 0.5 and r.ex.state == PAUSED


# ---------------------------------------------------------------- 다가오는 물체: 멈춰 기다림(양보 후진 없음)
def _approach(speed_cm_s, after_s=None, total_cm=None):
    """물체 obs[0] 을 서쪽(로봇 쪽)으로 움직이는 move 함수. after_s 뒤부터, total_cm 만큼(없으면 계속)."""
    st = {'t0': None, 'moved': 0.0}

    def move(sim, run):
        if not run.obs:
            return
        if st['t0'] is None:
            st['t0'] = sim.t
        if after_s is not None and sim.t - st['t0'] < after_s:
            return
        step = speed_cm_s * sim.dt
        if total_cm is not None:
            step = min(step, total_cm - st['moved'])
        if step <= 0:
            return
        st['moved'] += step
        x, y, r = run.obs[0]
        run.obs[0] = (x - step, y, r)
    return move


def _take_away(r, speed_cm_s=30.0, seconds=1.0):
    """물체를 로봇에서 멀어지게(동쪽, scan 한 번에 3 cm) 옮긴 뒤 치움. 한 번에 사라지게 하면 사각 의심 규칙이 걸림."""
    r.move = None
    end = r.sim.t + seconds
    while r.sim.t < end:
        x, y, rad = r.obs[0]
        r.obs[0] = (x + speed_cm_s * r.sim.dt, y, rad)
        r.tick()
    r.obs.clear()


def _held_settled(graph, x_obj=215.0, **kw):
    r = Run(graph, obs=[(x_obj, _y(graph), R_OBJ)], **kw)
    assert r.until(_held)
    assert r.until(lambda e: e.settled, 5.0)
    r.until(lambda e: False, 0.6)
    return r


@pytest.mark.parametrize('delay, odom', [(0.0, 1.0), (0.2, 1.05), (0.15, 0.95)])
def test_approaching_object_robot_waits_then_resumes(graph, delay, odom):
    """멈춘 뒤 물체가 5 cm/s 로 다가와도(15 cm 뒤 멈춤) 로봇은 후진하지 않고 0 그대로 기다린다(2026-11-15 결정).
    물체가 물러나면 3 s 뒤 재개, 노드 순서·감지 위치 그대로."""
    base = _baseline(graph, delay=delay, odom=odom)
    r = _held_settled(graph, delay=delay, odom=odom)
    x_hold, n = r.sim.x, len(r.cmds)
    r.move = _approach(5.0, total_cm=15.0)
    r.until(lambda e: False, 5.0)
    st = r.ex.as_dict()
    assert st['state'] == PAUSED and st['pause_reason'] == 'obstacle'
    assert st['obstacle']['phase'] == 'blocked' and st['obstacle']['front_cm'] < 15.0
    assert set(r.cmds[n:]) == {(0.0, 0.0)} and abs(r.sim.x - x_hold) < 0.05
    assert 'reverse' not in st and 'yielded_cm' not in st['obstacle']
    _take_away(r)
    assert r.finish() == ARRIVED, r.ex.reason
    assert r.kinds() == ['obstacle_stop', 'obstacle_clear']
    assert [n_[0] for n_ in r.nodes] == [n_[0] for n_ in base.nodes]
    for got, ref in zip(r.nodes, base.nodes):
        assert math.hypot(got[1] - ref[1], got[2] - ref[2]) < 1.0, (got, ref)


def test_rear_reported_only(graph):
    """rear_cm 은 감시용(state·CSV)으로만 계산. 뒤에 물체가 붙어도 동작은 같다."""
    r = _held_settled(graph)
    r.obs.append((r.sim.x - 7.5 - 12.0 - R_OBJ, _y(graph), R_OBJ))
    r.until(lambda e: False, 1.0)
    assert r.ex.as_dict()['obstacle']['rear_cm'] == pytest.approx(12.0, abs=0.5)
    r.obs.pop(0)                                                    # 앞 물체만 치움(앞 거리 24.8 → 벽)
    assert r.until(lambda e: e.state == FOLLOW, 4.0)


# ---------------------------------------------------------------- 수동 일시정지·stop 우선
def test_resume_rejected_while_blocked_and_manual_pause_wins(graph):
    r = _held_settled(graph)
    with pytest.raises(ValueError, match='obstacle stop'):
        r.ex.resume()
    assert r.ex.pause().startswith('paused (manual')
    st = r.ex.as_dict()
    assert st['pause_reason'] == 'obstacle' and st['pause']['manual'] is True
    with pytest.raises(ValueError, match='manual pause stays'):
        r.ex.resume()
    r.obs.clear()
    r.until(lambda e: False, 5.0)
    st = r.ex.as_dict()
    assert st['state'] == PAUSED and st['pause_reason'] == 'manual'
    assert st['obstacle']['phase'] == 'clear'
    clear = [e for e in r.events if e['event'] == 'obstacle_clear'][-1]['detail']
    assert 'manual pause remains' in clear
    r.ex.resume()
    assert r.finish() == ARRIVED


def test_manual_pause_first_blocks_no_obstacle_stop_then_resume_holds(graph):
    """수동 일시정지 중에는 정지 판정 안 함(거리만). resume 하면 바로 장애물 정지 — 앞으로 움직이지 않음."""
    r = Run(graph)
    assert r.until(lambda e: e.state == FOLLOW and e.index == 1
                   and e.as_dict()['traveled_cm'] > 15.0)
    r.ex.pause()
    r.until(lambda e: e.settled, 5.0)
    x = r.sim.x
    r.obs.append((x + 4.2 + 18.0 + R_OBJ, _y(graph), R_OBJ))        # 앞 18 cm
    r.until(lambda e: False, 1.0)
    st = r.ex.as_dict()
    assert 'obstacle_stop' not in r.kinds() and st['pause_reason'] == 'manual'
    assert st['obstacle']['phase'] == 'clear' and st['obstacle']['front_cm'] < 25.0
    r.ex.resume()
    r.tick()
    assert r.ex.as_dict()['pause_reason'] == 'obstacle' and r.sim.cmd == (0.0, 0.0)
    r.until(lambda e: False, 1.0)
    assert abs(r.sim.x - x) < 0.2


@pytest.mark.parametrize('when', ('blocked', 'resume_wait', 'manual'))
def test_stop_always_wins(graph, when):
    r = _held_settled(graph)
    if when == 'manual':
        r.ex.pause()
    elif when == 'resume_wait':
        r.obs.clear()
        assert r.until(lambda e: e.as_dict()['obstacle']['phase'] == 'resume_wait', 1.0)
    r.ex.stop()
    assert r.ex.state == STOPPED
    reading = r.sim.sensor.update(r.sim.raw())
    cmd = r.ex.step(reading, r.sim.pose(), r.sim.t)
    assert (cmd.linear, cmd.angular) == (0.0, 0.0)
    assert r.ex.step(reading, r.sim.pose(), r.sim.t + 0.05) is None
    st = r.ex.as_dict()
    assert st['pause_reason'] is None and st['obstacle']['phase'] == 'clear' and st['pause'] is None


# ---------------------------------------------------------------- 오래 막힘 경고·상태 모양
def test_warn_after_30s_once(graph):
    r = _held_settled(graph)
    r.until(lambda e: False, 25.0)
    assert r.ex.as_dict()['obstacle']['warn'] is False
    r.until(lambda e: False, 10.0)
    ob = r.ex.as_dict()['obstacle']
    assert ob['warn'] is True and ob['blocked_s'] >= 30.0
    assert r.kinds().count('obstacle_warn') == 1
    r.obs.clear()
    assert r.until(lambda e: e.state == FOLLOW, 5.0)
    assert r.ex.as_dict()['obstacle']['warn'] is False


def test_state_shape(graph):
    r = _held_settled(graph)
    st = r.ex.as_dict()
    assert set(st['obstacle']) == {'mode', 'front_cm', 'rear_cm', 'scan_age_s', 'phase',
                                   'blocked_s', 'warn', 'why'}
    assert st['obstacle']['mode'] == 'stop' and st['pause_reason'] == 'obstacle'
    assert set(st['pause']) == {'from', 'settled', 'manual'} and st['pause']['manual'] is False
    for gone in ('back_targets', 'back_why', 'reverse'):
        assert gone not in st
    assert not {'reverse_follow_speed', 'reverse_follow_min', 'reverse_follow_max'} & set(st['speed'])


@pytest.mark.parametrize('speed', (8.0, 15.0))
def test_object_vanishing_into_blind_zone_needs_manual_resume(graph, speed):
    """빠른 작은 물체가 range_min(시뮬 5 cm = 앞 끝 −0.9 cm) 안으로 들어가면 광선이 무효가 되고 뒤 벽이 보여 앞이 빈 것처럼 나온다.
    유효 scan 사이에 blind_near_cm 미만 → 재개 거리 이상으로 뛰면 자동 재개하지 않고 수동 재개만(사각 의심)."""
    r = _held_settled(graph)
    r.move = _approach(speed)
    r.until(lambda e: False, 6.0)
    r.move = None
    r.obs.clear()
    r.until(lambda e: False, 5.0)
    st = r.ex.as_dict()
    assert st['state'] == PAUSED and st['pause_reason'] == 'manual', st['obstacle']
    assert 'range_min' in st['obstacle']['why']
    r.ex.resume()
    assert r.finish() == ARRIVED


# ---------------------------------------------------------------- R1(09 검토) 보완
def _inject(r, schedule):
    """시뮬 라이다를 끄고 schedule [(초, front, rear), ...] 순서로 set_scan 을 직접 넣는 move 함수(10 Hz)."""
    r.sim.lidar['on'] = False
    st = {'t0': None, 'next': 0.0}

    def move(sim, run):
        if st['t0'] is None:
            st['t0'] = sim.t
        if sim.t + 1e-9 < st['next']:
            return
        st['next'] = sim.t + 0.1
        el, acc = sim.t - st['t0'], 0.0
        for dur, f, rear in schedule:
            acc += dur
            if el < acc:
                run.ex.set_scan(f, rear, sim.t)
                return
        run.ex.set_scan(schedule[-1][1], schedule[-1][2], sim.t)
    return move


def test_vanish_through_none_scan_needs_manual_resume(graph):
    """지적 1: 가까움(10) → 점 없음(None) → 멀리(200). 사이 None scan 을 건너뛰고 마지막 유효 front 로 사각 의심."""
    r = _held_settled(graph)
    r.move = _inject(r, [(0.5, 10.0, None), (0.5, None, None), (6.0, 200.0, None)])
    r.until(lambda e: False, 6.5)
    st = r.ex.as_dict()
    assert st['state'] == PAUSED and st['pause_reason'] == 'manual', st['obstacle']
    assert 'range_min' in st['obstacle']['why']
    assert st['obstacle']['phase'] == 'clear'      # 장애물 정지는 풀렸고 수동만 남음 → resume 받음
    r.move = None
    r.obs.clear()                                  # 사람이 확인하고 물체를 치움
    r.sim.lidar['on'] = True
    r.sim.lidar['next'] = 0.0
    r.ex.resume()
    assert r.finish() == ARRIVED, r.ex.reason


def test_gradual_retreat_still_auto_resumes(graph):
    """대조: 10 → 13 → … → 40 처럼 유효 scan 마다 조금씩 멀어지면 사각 의심 아님 → 3 s 뒤 자동 재개."""
    r = _held_settled(graph)
    sched = [(0.5, 10.0, None)] + [(0.1, 10.0 + 3.0 * k, None) for k in range(1, 11)] + \
        [(5.0, 40.0, None)]
    r.move = _inject(r, sched)
    assert r.until(lambda e: e.state == FOLLOW, 6.0), r.ex.as_dict()['obstacle']


def test_front_no_points_while_driving_stops(graph):
    """지적 2(사용자 결정 안전 쪽): 감시 단계에서 앞 점 없음이 scan_max_age_s 넘게 이어지면 정지
    (pause_reason scan_stale, why front_no_points). 유효 front ≥ 30 이 3 s 이어져야 재개."""
    r = Run(graph)
    r.sim.lidar['room'] = None                # 벽도 물체도 없음 → 앞·뒤 점 없음
    assert r.until(lambda e: e.state == PAUSED, 10.0)
    st = r.ex.as_dict()
    assert st['pause_reason'] == 'scan_stale' and st['obstacle']['why'] == 'front_no_points'
    assert st['obstacle']['front_cm'] is None and st['pause']['from'] == FOLLOW
    assert st['traveled_cm'] < 2.0                       # 추종 시작 직후(교차선 출발 전진 동안 이미 0.5 s 넘음)
    x = r.sim.x
    r.until(lambda e: False, 3.0)
    assert r.ex.state == PAUSED and abs(r.sim.x - x) < 0.5
    with pytest.raises(ValueError, match='obstacle stop'):
        r.ex.resume()
    t_back = r.sim.t
    r.sim.lidar['room'] = (0.0, 0.0, 444.5, 221.0)       # 벽이 다시 보임
    assert r.until(lambda e: e.state == FOLLOW, 5.0)
    assert 3.0 <= r.sim.t - t_back <= 3.3
    assert r.finish() == ARRIVED, r.ex.reason


def test_front_no_points_monitor_only_reports(graph):
    base = _baseline(graph)
    r = Run(graph, mode='monitor')
    r.sim.lidar['room'] = None
    whys = []
    r.move = lambda s, run: whys.append(run.ex.as_dict()['obstacle']['why'])
    assert r.finish() == ARRIVED and r.poses == base.poses
    assert any(w and w.startswith('monitor: would stop (front no points') for w in whys)


# ---------------------------------------------------------------- main 통합(hold_at_goal·끝 기억)
def test_obstacle_stop_then_hold_at_goal_keeps_centered_end(graph):
    """main 의 hold_at_goal(교차점 중심 정차)과 함께: 마지막 구간 장애물 정지 → 3 s 뒤 재개 → N19 에 축 중심 정차,
    끝 기억 centered·pose 보존 → 회전 출발이 그 기억을 씀. odom 이 3 cm 바뀌면 끝 기억을 버림(end_dropped)."""
    y = _y(graph)
    sim = Sim(graph)
    sim.place('N07', 'E', 0.0)
    obs = [(225.0, y, R_OBJ)]
    sim.enable_lidar(lambda t: obs)
    ex = RouteExecutor(graph, NavConfig(obstacle_mode='stop', **LOGIC_CFG))
    ex.start(['N07', 'N23', 'N19'], start_cross=True, start_facing='E', hold_at_goal=True)
    while ex.active and ex.state != PAUSED and sim.t < 200:
        sim.tick(ex)
    assert ex.state == PAUSED and ex.as_dict()['leg']['to'] == 'N19'
    obs.clear()
    while ex.active and sim.t < 400:
        sim.tick(ex)
    assert ex.state == ARRIVED, ex.reason
    n19 = graph.nodes['N19']
    assert abs(sim.x - n19.x) < 1.5 and abs(sim.y - n19.y) < 1.0     # 축이 N19 중심
    for _ in range(10):
        sim.tick(ex)
    end = ex._end
    assert end['centered'] is True and end['pose'] is not None
    assert ex.as_dict()['end'] == {'node': 'N19', 'facing': 'E', 'moved': False, 'passed': []}
    ex.start(['N19', 'N11'])                                       # 우회전 출발 = 중심 기억 사용
    assert 'end_dropped' not in [e['event'] for e in ex.events] and ex._pre is not None
    ex.stop()
    ex2 = RouteExecutor(graph, NavConfig(obstacle_mode='stop', **LOGIC_CFG))
    ex2._end = dict(end)
    ox, oy, oyaw = sim.odom
    ex2.step(sim.sensor.update(sim.raw()), (ox, oy + 3.0, oyaw), sim.t + 1.0)
    ex2.start(['N19', 'N11'])
    assert 'end_dropped' in [e['event'] for e in ex2.events]


# ---------------------------------------------------------------- 실로봇 1차(2026-11-15): 기본값·상차 노드 면제
def _approach_n11(graph, cfg, gap_cm=4.0):
    """N19 우회전 뒤 N19→N11(남쪽) 구간 가운데에서 몸체 앞 gap_cm 에 물체를 둔다. → (실행부, 이벤트·명령 기록)."""
    r = Run(graph, **cfg)
    assert r.until(lambda e: e.state == FOLLOW and e.index == 2 and e.as_dict()['traveled_cm'] > 8.0, 200)
    sx = r.sim.x
    front_y = r.sim.y - 4.2                                          # 남쪽을 봄: 앞 끝 y
    r.obs.append((sx, front_y - gap_cm - R_OBJ, R_OBJ))
    return r


def test_exempt_load_leg_does_not_stop_at_4cm(graph):
    """기본 면제(N01·N11·N21·N22): N19→N11 구간에서 앞 4 cm 물체여도 정지 판정 안 함(거리는 보고, why exempt)."""
    r = _approach_n11(graph, {'stop_cm': 5.0, 'blind_near_cm': 3.0,
                              'obstacle_exempt_nodes': ('N01', 'N11', 'N21', 'N22')})
    for _ in range(4):                                              # 다음 scan(10 Hz)까지
        r.tick()
    st = r.ex.as_dict()
    assert st['state'] == FOLLOW and st['pause_reason'] is None
    assert st['obstacle']['front_cm'] < 5.0 and st['obstacle']['why'] == 'exempt leg N19→N11'
    r.obs.clear()
    r.sim.lidar['on'] = False                                       # scan 끊김도 면제 구간에서는 판정 안 함
    for _ in range(30):
        r.tick()
    assert r.ex.state != PAUSED and 'obstacle_stop' not in r.kinds()


def test_same_obstacle_on_normal_leg_stops(graph):
    """같은 4 cm 물체라도 면제 노드가 없으면(빈 목록) 정지(stop 5 cm)."""
    r = _approach_n11(graph, {'stop_cm': 5.0, 'blind_near_cm': 3.0, 'obstacle_exempt_nodes': ()})
    assert r.until(_held, 1.0)
    st = r.ex.as_dict()
    assert st['leg']['to'] == 'N11' and st['obstacle']['why'] is None


def test_default_stop_5cm_on_straight_leg(graph):
    """기본값(stop 5): N23→N19 에서 24.8 cm 물체는 지나가고, 4 cm 앞에서 멈춘다(면제 아닌 구간)."""
    r = Run(graph, obs=[(195.0, _y(graph), R_OBJ)], **{k: v for k, v in NavConfig().__dict__.items()
                                                       if k in LOGIC_CFG})
    assert r.until(_held, 120)
    assert 0.0 < r.front_gap() <= 5.0, r.front_gap()
    assert r.ex.as_dict()['leg']['to'] == 'N19'


def test_monitor_does_not_report_would_stop_on_exempt_leg(graph):
    r = _approach_n11(graph, {'mode': 'monitor', 'stop_cm': 5.0, 'blind_near_cm': 3.0,
                              'obstacle_exempt_nodes': ('N11',)})
    for _ in range(4):
        r.tick()
    assert r.ex.as_dict()['obstacle']['why'] == 'exempt leg N19→N11'


def test_exempt_also_from_node_and_loading(graph):
    """구간 from 이 면제 노드여도 면제. LOADING(상차 대기)·도킹 이동은 원래 감시 단계가 아니라 몸체 앞 물체로 멈추지 않음."""
    from roport_line_nav.graph import Leg
    ex = RouteExecutor(graph, NavConfig(obstacle_mode='stop'))
    ex.legs, ex.index = [Leg('N11', 'N19', 'N', 36.1, 'straight', 'side', frozenset())], 0
    assert ex._exempt_leg() == 'N11→N19'
    r = Run(graph, stop_cm=5.0, blind_near_cm=3.0, obstacle_exempt_nodes=('N11',))
    r.ex.load_done = lambda: False                                   # 상차 대기에 머묾
    assert r.until(lambda e: e.state == 'loading', 200)
    r.obs.append((r.sim.x + 4.2 + 2.0 + R_OBJ, r.sim.y, R_OBJ))   # 바라보는 쪽(동쪽) 앞 2 cm
    for _ in range(20):
        r.tick()
    st = r.ex.as_dict()
    assert st['state'] == 'loading' and st['pause_reason'] is None
    assert st['obstacle']['why'] == 'exempt leg N19→N11'


@pytest.mark.parametrize('text, out', [('N01,N11,N21,N22', ('N01', 'N11', 'N21', 'N22')),
                                       (' N11 , N01 ', ('N11', 'N01')), ('', ())])
def test_exempt_list_parsing(text, out):
    from roport_line_nav.executor import parse_node_list
    assert parse_node_list(text) == out
    assert NavConfig(obstacle_exempt_nodes=text).obstacle_exempt_nodes == out


@pytest.mark.parametrize('bad', ['n11', 'N11;rm', 'N11,,N01', 'N 11', 'N11,'])
def test_exempt_list_rejects(bad):
    from roport_line_nav.executor import parse_node_list
    with pytest.raises(ValueError):
        parse_node_list(bad)
    with pytest.raises(ValueError):
        NavConfig(obstacle_exempt_nodes=bad)
