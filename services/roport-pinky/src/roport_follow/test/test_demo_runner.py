"""시험용 실행기(demo_runner): 콘솔 명령 → TrialDriver hook, 콘솔 끊김 정지, 결과 표시."""
import math

import pytest

from roport_follow.demo_runner import DemoRunner


class Graph:
    version = 'line-v15-20261116'
    nodes = {'N08': None, 'N12': None, 'N09': None}

    def heading(self, a, b):
        return {('N08', 'N12'): 'E', ('N08', 'N09'): 'W'}[(a, b)]


class Driver:
    def __init__(self):
        self.calls, self.results, self.reject = [], [], None

    def validate_action(self, cmd):
        self.calls.append(('validate', cmd['type']))
        if self.reject:
            raise ValueError(self.reject)

    def dispatch(self, cmd):
        self.calls.append(('dispatch', cmd['type'], cmd['payload']))
        self.last = cmd

    def safe_stop(self, reason):
        self.calls.append(('safe_stop', reason))

    def heartbeat(self):
        self.calls.append(('heartbeat',))

    def drain_results(self):
        out, self.results = self.results, []
        return out

    def observe(self):
        return {'mode': 'line', 'node_id': 'N08', 'stopped': True, 'fault': None}


PROFILE = {'map_id': 'airport_map', 'map_version': 'abc123', 'frame_yaw_rad': math.radians(0.1)}


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


def make():
    d, c = Driver(), Clock()
    return d, c, DemoRunner(d, PROFILE, Graph(), console_timeout_s=1.0, now=c)


def test_route_command_shape_and_heading():
    d, c, r = make()
    r.handle({'op': 'route', 'route': ['N08', 'N12'], 'speed_limit_mps': 0.06})
    kind, typ, p = d.calls[-1]
    assert (kind, typ) == ('dispatch', 'route.requested')
    assert p['start_node'] == 'N08' and p['start_maneuver'] == 'none' and p['graph_version'] == Graph.version
    assert p['start_heading_rad'] == pytest.approx(math.radians(0.1))
    assert r.state()['active']['type'] == 'route.requested'


def test_switch_and_follow_carry_map_identity():
    d, c, r = make()
    r.handle({'op': 'switch_nav2'})
    assert d.calls[-1][2] == {'to_mode': 'nav2', 'map_id': 'airport_map', 'map_version': 'abc123'}
    r.active = None
    r.handle({'op': 'follow', 'card_id': 35, 'standoff_m': 0.5, 'max_duration_s': 120})
    p = d.calls[-1][2]
    assert d.calls[-1][1] == 'follow.requested' and p['card_id'] == 35 and p['max_duration_s'] == 120.0
    assert p['map_version'] == 'abc123'
    assert 'lost_after_s' not in p                      # 안 주면 드라이버 기본(1 s, SR-22)
    r.active = None
    r.handle({'op': 'follow', 'card_id': 35, 'lost_after_s': 4})
    assert d.calls[-1][2]['lost_after_s'] == 4.0


def test_rejections_are_reported_not_dispatched():
    d, c, r = make()
    r.handle({'op': 'route', 'route': ['N08', 'N99']})
    d.reject = 'not_in_nav2_mode'
    r.handle({'op': 'follow', 'card_id': 35})
    r.handle({'op': 'dance'})
    assert not any(x[0] == 'dispatch' for x in d.calls)
    reasons = [e['reason'] for e in r.state()['events']]
    assert reasons == ['invalid_route', 'not_in_nav2_mode', 'unknown_op']


def test_heartbeat_only_while_console_alive_and_link_loss_stops():
    d, c, r = make()
    r.tick()
    assert ('heartbeat',) not in d.calls                 # 콘솔 신호 전에는 구동권 안 줌
    r.handle({'op': 'follow', 'card_id': 35})
    r.tick()
    assert d.calls[-1] == ('heartbeat',)
    c.t += 1.5                                           # 노트북 끊김
    r.tick()
    assert ('safe_stop', 'control_link_lost') in d.calls and r.active is None
    n = len(d.calls)
    r.tick()
    assert len(d.calls) == n                             # 한 번만 멈춤, heartbeat 도 안 함
    r.handle({'op': 'ping'})
    r.tick()
    assert d.calls[-1] == ('heartbeat',)


def test_operator_stop_and_results_clear_active():
    d, c, r = make()
    r.handle({'op': 'follow', 'card_id': 35})
    cmd = d.last
    d.results.append((cmd, 'failed', {'error_code': 'follow_lost'}))
    r.tick()
    assert r.active is None and r.state()['events'][-1] == {**r.state()['events'][-1], 'kind': 'failed',
                                                           'error_code': 'follow_lost'}
    r.handle({'op': 'route', 'route': ['N08', 'N12']})
    r.handle({'op': 'stop'})
    assert ('safe_stop', 'operator_stop') in d.calls and r.active is None


def test_start_mode_nav2_skips_line_route_for_tests_only():
    """11-21: 로봇을 N12 에 손으로 놓고 Nav2 모드로 바로 시작(①·② 건너뜀). line 이면 아무것도 안 바꿈."""
    import threading
    from types import SimpleNamespace
    from roport_follow.demo_runner import apply_start_mode
    d = SimpleNamespace(lock=threading.RLock(), mode='line', logic=SimpleNamespace(node='N12'))
    apply_start_mode(d, 'line')
    assert d.mode == 'line' and d.logic.node == 'N12'
    apply_start_mode(d, 'nav2')
    assert d.mode == 'nav2' and d.logic.node is None
    import pytest
    with pytest.raises(ValueError):
        apply_start_mode(d, 'dock')


def test_navigate_command_shape():
    d, c, r = make()
    r.handle({'op': 'navigate', 'goal_id': 'demo_G3', 'x': 0.11, 'y': -0.02, 'yaw': 3.5})
    kind, typ, p = d.calls[-1]
    assert (kind, typ) == ('dispatch', 'navigate.requested')
    assert p['goal_id'] == 'demo_G3' and p['frame_id'] == 'map' and p['map_version'] == 'abc123'
    assert p['pose']['x'] == 0.11 and p['pose']['yaw'] == pytest.approx(3.5 - 2 * math.pi)
    assert p['position_tolerance_m'] > 0 and p['yaw_tolerance_rad'] > 0
    r.active = None
    r.handle({'op': 'navigate', 'goal_id': 'demo_G3', 'x': float('nan'), 'y': 0, 'yaw': 0})
    assert r.state()['events'][-1]['kind'] == 'rejected' and r.active is None


def test_route_from_loading_uses_observed_heading():
    """상차 대기(N21 에서 반시계 90°)에서 출발: 경로 방향이 아니라 지금 자세를 출발 방향으로 보낸다."""
    from types import SimpleNamespace
    d, c, r = make()
    d.logic = SimpleNamespace(st={'state': 'loading', 'route': ['N09', 'N08']})
    d.observe = lambda: {'mode': 'line', 'node_id': 'N08', 'stopped': True, 'fault': None, 'heading_rad': 1.234}
    r.handle({'op': 'route', 'route': ['N08', 'N12']})
    assert d.calls[-1][2]['start_heading_rad'] == pytest.approx(1.234)
    r.active = None
    d.logic.st = {'state': 'loading', 'route': ['N08', 'N09']}          # 다른 노드에서 상차 중이면 경로 방향
    r.handle({'op': 'route', 'route': ['N08', 'N12']})
    assert d.calls[-1][2]['start_heading_rad'] == pytest.approx(math.radians(0.1))


def test_lift_nav_recovery_cap_keeps_explicit_arguments():
    from types import SimpleNamespace
    from roport_follow.demo_runner import lift_nav_recovery_cap
    seen = []
    d = SimpleNamespace(nav=SimpleNamespace(start=lambda payload, *, behavior_tree='', stamp=None, max_recoveries=2:
                                            seen.append((payload, behavior_tree, max_recoveries))))
    lift_nav_recovery_cap(d)
    d.nav.start({'goal_id': 'g'})
    d.nav.start({'goal_id': 'f'}, behavior_tree='follow.xml', max_recoveries=None)
    assert seen == [({'goal_id': 'g'}, '', None), ({'goal_id': 'f'}, 'follow.xml', None)]
