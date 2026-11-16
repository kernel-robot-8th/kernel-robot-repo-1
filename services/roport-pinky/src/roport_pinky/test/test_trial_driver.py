"""TrialLogic(관제 trial-v1 드라이버 판정부)을 RouteExecutor + 2D 시뮬레이터로 확인(ROS·MQTT 없음)."""

import hashlib
import math
import os
import sys

import pytest

SRC = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path[:0] = [os.path.join(SRC, 'roport_line_nav', 'test')]

from roport_line_nav.executor import RouteExecutor  # noqa: E402
from roport_line_nav.graph import Graph  # noqa: E402
from roport_pinky.trial_driver import TrialLogic, error_code  # noqa: E402
from sim import Sim  # noqa: E402

LAYOUT = os.path.join(SRC, 'roport_gz_world', 'layout', 'roport_airport.yaml')
TO_P7 = ['N16', 'N14', 'N02', 'N10', 'N18', 'N07', 'N23', 'N19', 'N11']
SOUTH = -math.pi / 2 + math.radians(0.10)
MAP = os.path.join(SRC, '..', 'maps', 'airport_map')
# 관제 설정 생성기와 같은 계산(yaml + pgm 의 sha256 앞 16자). 지도를 다시 만들면 값이 바뀐다.
with open(MAP + '.yaml', 'rb') as _y, open(MAP + '.pgm', 'rb') as _p:
    PROFILE = {'map_id': 'airport_map',
               'map_version': hashlib.sha256(_y.read() + _p.read()).hexdigest()[:16]}


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Rig:
    """line_nav 노드 자리에 RouteExecutor + Sim 을 둔다. 시간은 시뮬레이터 시간."""

    def __init__(self, start='N16', heading='S'):
        self.graph = Graph.load(LAYOUT)
        self.sim = Sim(self.graph)
        self.sim.place(start, heading, 0.0)
        self.ex = RouteExecutor(self.graph)
        self.clock = Clock()
        self.last = None
        self.sent = []
        self.zeros = 0
        self.logic = TrialLogic(self.graph, PROFILE, send=self.send, zero=self.zero,
                                start_node=start, now=self.clock)
        self.results = []
        self.feed()

    def send(self, body):
        self.sent.append(body['cmd'])
        try:
            {'speed': lambda: self.ex.set_speed(body.get('base_speed')),
             'start': lambda: self.ex.start(body['route'],
                                            hold_at_goal=body.get('hold_at_goal', False)),
             'load_done': self.ex.load_done, 'stop': self.ex.stop}[body['cmd']]()
            self.last = {'cmd': body['cmd'], 'ok': True, 'detail': ''}
        except Exception as e:
            self.last = {'cmd': body['cmd'], 'ok': False, 'detail': str(e)}

    def zero(self):
        self.zeros += 1

    def feed(self):
        st = self.ex.as_dict()
        st.update(graph_version=self.graph.version, last_command=self.last)
        self.logic.on_state(st)
        self.logic.on_odom(*self.sim.cmd)

    def run(self, seconds=600, until=None):
        end = self.sim.t + seconds
        while self.sim.t < end:
            self.sim.tick(self.ex)
            self.clock.t = 1000.0 + self.sim.t
            self.feed()
            self.results += self.logic.drain_results()
            if until is not None and until(self):
                return True
        return False


def route_cmd(route, heading=SOUTH, maneuver='none', speed=0.1, kind='route.requested'):
    return {'type': kind, 'command_id': 'c-' + route[-1], 'execution_id': 'e-' + route[-1],
            'payload': {'graph_version': 'line-v14-20261106', 'start_node': route[0],
                        'start_heading_rad': heading, 'start_maneuver': maneuver,
                        'route': route, 'speed_limit_mps': speed,
                        'goal': {'node_id': route[-1], 'location_id': None,
                                 'on_arrival': 'hold'}}}


def finished(rig):
    return any(p in ('completed', 'failed') for _, p, _ in rig.results)


def test_observe_at_start_bay():
    rig = Rig()
    s = rig.logic.observe()
    assert s['node_id'] == 'N16' and s['stopped'] and not s['line_enabled']
    assert s['mode'] == 'line' and s['drive_owner'] == 'none' and s['fault'] is None
    assert abs(s['heading_rad'] - SOUTH) < 1e-9            # 칸 바깥(N14 쪽) = 남쪽 + 0.1°
    assert s['graph_version'] == 'line-v14-20261106'
    assert s['map_version'] == PROFILE['map_version']


def test_p5_to_p7_progress_then_complete_while_loading():
    rig = Rig()
    cmd = route_cmd(TO_P7)
    rig.logic.validate_action(cmd)
    rig.logic.dispatch(cmd)
    assert rig.sent == ['speed', 'start']
    assert rig.ex.cfg.base_speed == 0.03                    # min(현재 0.03, 한계 0.1)
    assert rig.run(until=finished)
    assert 120 < rig.sim.t < 300  # 0.03 m/s: the former 120 s trial deadline was too short
    phases = [(p, d.get('seq')) for _, p, d in rig.results]
    assert phases == [('progress', i) for i in range(1, 8)] + [('completed', 8)]
    assert rig.ex.state == 'loading'                        # N11: 반시계 90° 뒤 상차 대기
    s = rig.logic.observe()
    assert s['node_id'] == 'N11' and s['stopped'] and not s['line_enabled']
    assert abs(s['heading_rad'] - math.radians(0.10)) < 0.1   # 남쪽 + 90° = 동쪽
    _, _, done = rig.results[-1]
    assert done == {'node_id': 'N11', 'seq': 8, 'total': 9,
                    'prev_node': 'N19', 'next_node': None}


def test_queued_progress_keeps_each_detected_node():
    rig = Rig()
    rig.logic.dispatch(route_cmd(TO_P7))
    while rig.logic.seq < 2:
        rig.sim.tick(rig.ex)
        rig.clock.t = 1000.0 + rig.sim.t
        rig.feed()
        assert rig.sim.t < 120
    events = rig.logic.drain_results()
    assert rig.logic.observe()['node_id'] == 'N02'
    assert [(d['seq'], d['node_id']) for _, phase, d in events if phase == 'progress'] == [
        (1, 'N14'), (2, 'N02')]


def test_next_route_from_loading_sends_load_done_first():
    rig = Rig()
    rig.logic.dispatch(route_cmd(TO_P7))
    rig.run(until=finished)
    rig.results.clear()
    rig.sent.clear()
    nxt = route_cmd(['N11', 'N19', 'N24', 'N09', 'N08', 'N12'], heading=math.radians(0.10))
    with pytest.raises(ValueError, match='loading_heading_mismatch'):
        rig.logic.validate_action(route_cmd(nxt['payload']['route'], heading=math.pi / 2))
    rig.logic.validate_action(nxt)
    rig.logic.dispatch(nxt)
    assert rig.sent == ['load_done']
    assert rig.run(until=finished)
    assert rig.sent == ['load_done', 'speed', 'start']
    assert [p for _, p, _ in rig.results][-1] == 'completed'
    assert rig.ex.state == 'handoff' and rig.logic.observe()['node_id'] == 'N12'


@pytest.mark.parametrize('start, heading, first, continuation', [
    ('N16', 'S', ['N16', 'N14', 'N02', 'N10', 'N18'],
     ['N18', 'N07', 'N23', 'N19', 'N11']),
    ('N01', 'N', ['N01', 'N18'], ['N18', 'N10', 'N02', 'N14', 'N16']),
    ('N01', 'N', ['N01', 'N18'], ['N18', 'N07', 'N23', 'N19', 'N11']),
])
def test_shared_driver_junction_hold_and_resume(start, heading, first, continuation):
    rig = Rig(start, heading)
    cmd = route_cmd(first, heading=rig.logic.observe()['heading_rad'])
    rig.logic.validate_action(cmd)
    rig.logic.dispatch(cmd)
    assert rig.run(until=finished)
    assert rig.results[-1][1] == 'completed', rig.results
    observed = rig.logic.observe()
    assert observed['node_id'] == first[-1] and observed['stopped']
    assert observed['drive_owner'] == 'none' and not observed['line_enabled']
    assert rig.ex.state == 'arrived'
    nxt = route_cmd(continuation, heading=observed['heading_rad'])
    with pytest.raises(ValueError, match='junction_heading_mismatch'):
        rig.logic.validate_action(route_cmd(continuation, heading=observed['heading_rad'] + 1.0))
    rig.logic.validate_action(nxt)
    rig.results.clear()
    rig.logic.dispatch(nxt)
    assert rig.run(until=finished)
    assert rig.results[-1][1] == 'completed', rig.results
    assert rig.logic.observe()['node_id'] == continuation[-1]


def test_failure_reports_line_nav_reason_and_forgets_node():
    rig = Rig()
    rig.logic.dispatch(route_cmd(TO_P7))
    rig.run(seconds=5)
    rig.ex.stop('stop command')
    rig.run(seconds=1, until=finished)
    assert rig.results[-1][1:] == ('failed', {'error_code': 'stop'})
    assert rig.logic.observe()['node_id'] is None


def test_safe_stop_stops_line_nav_and_zeroes():
    rig = Rig()
    rig.logic.dispatch(route_cmd(TO_P7))
    rig.run(seconds=3)
    rig.logic.safe_stop('control_link_lost')
    assert rig.sent[-1] == 'stop' and rig.zeros == 1 and rig.ex.state == 'stopped'
    rig.run(seconds=1)
    assert rig.logic.drain_results() == []                  # 정지 뒤 결과 보고 안 함


@pytest.mark.parametrize('cmd, code', [
    (route_cmd(TO_P7, kind='navigate.requested'), 'unsupported_operation'),
    (route_cmd(TO_P7, maneuver='turn_back'), 'unsupported_maneuver'),
    (route_cmd(['N16', 'N02', 'N11']), 'invalid_route'),
    (route_cmd(TO_P7, heading=0.0), 'start_heading_not_first_leg'),
    (route_cmd(TO_P7, speed=0.01), 'speed_limit_too_low'),
])
def test_validate_rejects(cmd, code):
    rig = Rig()
    with pytest.raises(ValueError, match=code):
        rig.logic.validate_action(cmd)


def test_validate_rejects_busy_and_stale():
    rig = Rig()
    rig.logic.dispatch(route_cmd(TO_P7))
    rig.run(seconds=2)
    with pytest.raises(ValueError, match='line_nav_busy'):
        rig.logic.validate_action(route_cmd(TO_P7))
    rig.clock.t += 5                                         # line_nav 상태가 끊김
    with pytest.raises(ValueError, match='line_nav_unavailable'):
        rig.logic.validate_action(route_cmd(TO_P7))
    s = rig.logic.observe()
    assert s['fault'] == 'line_nav_unavailable' and s['mode'] == 'unknown'
    assert not s['stopped']                                  # odom 도 오래됨


def test_unknown_start_node_rejected_by_constructor():
    with pytest.raises(ValueError):
        TrialLogic(Graph.load(LAYOUT), PROFILE, send=print, zero=print, start_node='N99')


@pytest.mark.parametrize('reason, code', [
    ('line_lost on N02→N10', 'line_lost'), ('node_missed: N14 not found', 'node_missed'),
    ('line_not_found after 121°', 'line_not_found'), ('', 'line_nav_failed'),
])
def test_error_code(reason, code):
    assert error_code(reason) == code


@pytest.mark.parametrize('result_code', ['SUCCEEDED', 'nav2_recovery_timeout'])
def test_result_reporting_without_optional_navigation_diagnostic(result_code):
    import threading
    import time
    from types import SimpleNamespace
    from roport_pinky.trial_driver import TrialDriver
    driver = object.__new__(TrialDriver)
    driver.lock = threading.RLock()
    pose = {'x': 1., 'y': 2., 'yaw': 0.}
    cmd = {'type': 'nav2_goal.requested', 'payload': {
        'goal_id': 'G2', 'goal_version': 1, 'pose': pose,
        'position_tolerance_m': .03, 'yaw_tolerance_rad': .1}}
    driver.nav_command = cmd
    driver.logic = SimpleNamespace(active=None, drain_results=lambda: [])
    driver.nav = SimpleNamespace(navigation=None, result=(result_code, time.monotonic()-4),
                                 snapshot=lambda: {'pose': pose})
    driver.observe = lambda: {'fault': None, 'stopped': True}
    leases, stopped = [], []
    driver._lease = leases.append
    driver.safe_stop = stopped.append
    result = driver.drain_results()
    assert leases == ['none']
    assert driver.nav_command is None
    assert result[0][1] == ('completed' if result_code == 'SUCCEEDED' else 'failed')
    assert stopped == ([] if result_code == 'SUCCEEDED' else [result_code])
