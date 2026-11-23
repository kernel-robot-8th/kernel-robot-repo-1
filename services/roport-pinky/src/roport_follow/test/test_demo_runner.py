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

    def set_boarding_alert(self, on):
        self.calls.append(('boarding_alert', on))

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
    assert seen[0][0] == {'goal_id': 'g'} and seen[0][1].endswith('navigate_patient.xml') and seen[0][2] is None
    assert seen[1] == ({'goal_id': 'f'}, 'follow.xml', None)             # 추종은 자기 트리 그대로
    import xml.etree.ElementTree as ET
    tree = ET.parse(seen[0][1]).getroot()
    names = [n.tag for n in tree.iter()]
    assert 'ClearEntireCostmap' not in names and 'Spin' not in names and 'BackUp' not in names
    assert names.count('Wait') == 1 and 'FollowPath' in names


def test_alert_only_toggles_boarding_and_never_drives():
    d, c, r = make()
    r.handle({'op': 'ping'})
    r.handle({'op': 'alert', 'on': True})
    assert r.state()['boarding'] is True and r.events[-1] == {**r.events[-1], 'kind': 'alert', 'on': True}
    r.handle({'op': 'alert', 'on': 'yes'})
    assert r.events[-1]['reason'] == 'invalid_alert' and r.state()['boarding'] is True
    r.handle({'op': 'alert', 'on': False})
    assert r.state()['boarding'] is False
    assert not any(x[0] in ('dispatch', 'safe_stop', 'validate') for x in d.calls)
    # 발행은 드라이버 hook 하나로: 잘못된 값은 hook 까지 가지 않는다
    assert [x for x in d.calls if x[0] == 'boarding_alert'] == [('boarding_alert', True), ('boarding_alert', False)]


def test_alert_turns_off_when_console_link_is_lost_after_stopping_drive():
    d, c, r = make()
    r.handle({'op': 'route', 'route': ['N08', 'N12']})
    r.handle({'op': 'alert', 'on': True})
    c.t += 1.5
    r.tick()
    assert ('safe_stop', 'control_link_lost') in d.calls and r.state()['boarding'] is False
    assert [e['kind'] for e in r.events][-2:] == ['stopped', 'alert']
    assert d.calls[-1] == ('boarding_alert', False)                 # 드라이버 알림도 끔


def test_fine_align_returns_the_commanded_pose_inside_the_envelope_only():
    from roport_follow.demo_runner import FineAlign
    f = FineAlign(('demo_RET_N12',))
    assert f.enabled('demo_RET_N12') and not f.enabled('demo_G3')
    goal = {'goal_id': 'demo_RET_N12', 'pose': {'x': 2.2642, 'y': -1.2839, 'yaw': math.pi}}
    f.reset(goal)
    f.begin(10.0)
    assert f.target({'x': 2.4642, 'y': -1.2839, 'yaw': 3.1}, 11.0) == {'x': 2.2642, 'y': -1.2839, 'yaw': math.pi}
    with pytest.raises(ValueError, match='outside_envelope'):          # 40 cm: 정밀 접근 범위 밖
        f.target({'x': 2.6642, 'y': -1.2839, 'yaw': math.pi}, 11.0)
    with pytest.raises(ValueError, match='outside_envelope'):          # 방향 30° 넘게 틀어짐
        f.target({'x': 2.4, 'y': -1.2839, 'yaw': math.pi - 0.5}, 11.0)
    with pytest.raises(ValueError, match='timeout'):
        f.target({'x': 2.3, 'y': -1.2839, 'yaw': math.pi}, 36.0)
    f.reset()
    assert f.active is False and f.command is None


def test_install_fine_align_leaves_configured_gate_docking_alone():
    from types import SimpleNamespace
    from roport_follow.demo_runner import install_fine_align
    logs = []
    d = SimpleNamespace(nav=SimpleNamespace(docking=object(), dock_action=None), node=None)
    assert install_fine_align(d, log=logs.append) is False and logs and d.nav.dock_action is None


def test_fine_align_is_the_driver_class():
    from roport_follow.demo_runner import FineAlign
    from roport_pinky.fine_align import FineAlign as DriverFineAlign
    assert FineAlign is DriverFineAlign             # 중복 정의 없음


def test_install_fine_align_adds_demo_goal_to_driver_fine_align():
    """드라이버가 ROPORT_FINE_ALIGN_GOALS 로 FineAlign 을 켰으면 그 목표에 demo_RET_N12 를 더하고 슬롯은 그대로 둔다."""
    import threading
    from types import SimpleNamespace
    from roport_follow.demo_runner import FINE_ALIGN_GOALS, install_fine_align
    from roport_pinky.fine_align import FineAlign
    existing, action = FineAlign(('N12_RETURN',)), object()
    logs = []
    d = SimpleNamespace(nav=SimpleNamespace(docking=existing, dock_action=action), node=None,
                        lock=threading.RLock())
    assert install_fine_align(d, log=logs.append) is True and logs
    assert d.nav.docking is existing and d.nav.dock_action is action
    assert existing.goals == frozenset(('N12_RETURN',) + FINE_ALIGN_GOALS)
    assert existing.enabled('N12_RETURN') and existing.enabled('demo_RET_N12')


def test_follow_turn_first_turns_in_place_then_follows_after_settle():
    from types import SimpleNamespace
    from roport_follow.demo_runner import TURN_GOAL_ID, TURN_SETTLE_S
    d, c, r = make()
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': {'x': 2.6, 'y': -0.02, 'yaw': 0.0}})
    r.handle({'op': 'follow', 'card_id': 35, 'lost_after_s': 4, 'turn_first': True})
    kind, typ, p = d.calls[-1]
    assert (kind, typ) == ('dispatch', 'navigate.requested') and p['goal_id'] == TURN_GOAL_ID
    assert (p['pose']['x'], p['pose']['y']) == (2.6, -0.02) and abs(p['pose']['yaw']) == pytest.approx(math.pi)
    turn = d.last
    d.results.append((turn, 'completed', {}))
    r.tick()                                            # 돌기 끝 → 바로 보내지 않고 카메라 영상을 기다림
    dispatched = [x for x in d.calls if x[0] == 'dispatch']
    assert dispatched[-1][1] == 'navigate.requested' and r.state()['pending'] == 'follow_after_turn'
    c.t += TURN_SETTLE_S / 2
    r.handle({'op': 'ping'})                            # 콘솔은 기다리는 동안 살아 있음
    c.t += TURN_SETTLE_S / 2
    r.tick()
    kind, typ, p = [x for x in d.calls if x[0] == 'dispatch'][-1]
    assert (kind, typ) == ('dispatch', 'follow.requested') and p['card_id'] == 35 and p['lost_after_s'] == 4.0
    assert r.state()['active']['type'] == 'follow.requested'


def test_follow_turn_first_is_dropped_on_failed_turn_stop_or_unknown_pose():
    from types import SimpleNamespace
    d, c, r = make()
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': None})
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})
    assert r.events[-1]['reason'] == 'turn_pose_unavailable' and not any(x[0] == 'dispatch' for x in d.calls)
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 1.0}})
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})
    d.results.append((d.last, 'failed', {'error_code': 'nav2_recovery_timeout'}))
    r.tick()
    c.t += 5
    r.tick()
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})
    r.handle({'op': 'stop'})                            # 도는 중 정지: 돈 뒤 추종도 버림
    d.results.append((d.last, 'completed', {}))
    c.t += 5
    r.tick()
    r.tick()
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')


def test_follow_after_turn_is_dropped_when_console_dies_while_waiting_or_new_command_arrives():
    from types import SimpleNamespace
    from roport_follow.demo_runner import TURN_SETTLE_S
    d, c, r = make()
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}})
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})
    d.results.append((d.last, 'completed', {}))
    r.tick()                                            # 돌기 끝(active 없음, 추종 대기)
    c.t += 1.2                                          # 콘솔 1 s 넘게 조용 → 끊김
    r.tick()
    assert r.events[-1]['reason'] == 'control_link_lost' and r.state()['pending'] is None
    c.t += TURN_SETTLE_S
    r.handle({'op': 'ping'})
    r.tick()
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})
    d.results.append((d.last, 'completed', {}))
    r.tick()
    d.reject = 'mode_mismatch'
    r.handle({'op': 'switch_line'})                     # 거절돼도 기다리던 추종은 버림
    d.reject = None
    c.t += TURN_SETTLE_S
    r.handle({'op': 'ping'})
    r.tick()
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')


def test_navigate_position_tolerance_default_and_explicit_range():
    d, c, r = make()
    r.handle({'op': 'navigate', 'goal_id': 'g', 'x': 1.0, 'y': 2.0, 'yaw': 0.0})
    assert d.calls[-1][2]['position_tolerance_m'] == 0.12
    r.active = None
    r.handle({'op': 'navigate', 'goal_id': 'ARR', 'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'position_tolerance_m': 0.3})
    assert d.calls[-1][2]['position_tolerance_m'] == 0.3
    r.active = None
    r.handle({'op': 'navigate', 'goal_id': 'g', 'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'position_tolerance_m': 0.5})
    assert r.events[-1]['reason'] == 'invalid_position_tolerance'


def test_alert_messages_do_not_keep_the_drive_link_alive():
    """주행 콘솔 ping 이 끊긴 뒤 표시 전용 alert(정상·잘못된 것)만 계속 와도 연결 유실 정지가 동작한다."""
    d, c, r = make()
    r.handle({'op': 'route', 'route': ['N08', 'N12']})
    for _ in range(8):                                  # 0.2 s 마다 alert 만 옴, ping 없음
        c.t += 0.2
        r.handle({'op': 'alert', 'on': True})
        r.handle({'op': 'alert', 'on': 'bad'})
        r.tick()
    assert ('safe_stop', 'control_link_lost') in d.calls and r.state()['active'] is None
    assert r.console_alive() is False
    beats = [i for i, x in enumerate(d.calls) if x == ('heartbeat',)]
    stop = d.calls.index(('safe_stop', 'control_link_lost'))
    assert not beats or beats[-1] < stop                # 정지 뒤에는 구동권 갱신도 없음


def test_expired_link_is_handled_before_a_reconnect_ping_is_recorded():
    """돈 뒤 추종 대기 중 신호가 만료되고, 다음 tick 전에 다시 ping 이 와도 기다리던 추종은 나가지 않는다."""
    from types import SimpleNamespace
    from roport_follow.demo_runner import TURN_SETTLE_S
    d, c, r = make()
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}})
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})
    d.results.append((d.last, 'completed', {}))
    r.tick()
    assert r.state()['pending'] == 'follow_after_turn'
    c.t += 1.2                                          # 만료(1 s). tick 이 돌기 전에
    r.handle({'op': 'ping'})                            # 재연결 ping 이 먼저 도착
    assert r.state()['pending'] is None and r.events[-1]['reason'] == 'control_link_lost'
    c.t += TURN_SETTLE_S
    r.handle({'op': 'ping'})
    r.tick()
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')
    # 진행 중 주행도 같다: 만료 뒤 ping 이 먼저 와도 멈춘다
    r.handle({'op': 'route', 'route': ['N08', 'N12']})
    c.t += 1.2
    r.handle({'op': 'ping'})
    assert ('safe_stop', 'control_link_lost') in d.calls and r.state()['active'] is None


def test_fine_align_is_opt_in_so_default_n12_return_is_unchanged(monkeypatch):
    """--fine-align 없이는 드라이버 Nav2 어댑터에 아무것도 끼우지 않는다(GateDock 없는 기본 Nav2 설정에서 기존 복귀 유지)."""
    import sys
    import types
    from types import SimpleNamespace
    import roport_follow.demo_runner as dr

    made = {}

    class FakeDriver:
        def __init__(self, *a, **k):
            self.nav = SimpleNamespace(docking=None, dock_action=None, start=lambda *a, **k: None)
            self.node = SimpleNamespace(create_subscription=lambda *a, **k: None,
                                        create_publisher=lambda *a, **k: SimpleNamespace(publish=lambda m: None))
            self.lock, self.mode, self.logic = __import__('threading').Lock(), 'line', SimpleNamespace(node='N12')
            made['driver'] = self

        def heartbeat(self):
            raise KeyboardInterrupt                     # 주 루프 첫 바퀴에서 끝냄

        def drain_results(self):
            return []

        def safe_stop(self, reason):
            pass

        def close(self):
            pass

    calls = []
    monkeypatch.setitem(sys.modules, 'std_msgs', types.ModuleType('std_msgs'))
    msg = types.ModuleType('std_msgs.msg')
    msg.String = msg.Bool = lambda data=None: SimpleNamespace(data=data)
    monkeypatch.setitem(sys.modules, 'std_msgs.msg', msg)
    graph = types.ModuleType('roport_line_nav.graph')
    graph.Graph = SimpleNamespace(load=lambda path: Graph())
    monkeypatch.setitem(sys.modules, 'roport_line_nav', types.ModuleType('roport_line_nav'))
    monkeypatch.setitem(sys.modules, 'roport_line_nav.graph', graph)
    drv = types.ModuleType('roport_pinky.trial_driver')
    drv.TrialDriver = FakeDriver
    monkeypatch.setitem(sys.modules, 'roport_pinky', types.ModuleType('roport_pinky'))
    monkeypatch.setitem(sys.modules, 'roport_pinky.trial_driver', drv)
    monkeypatch.setattr(dr, 'make_profile', lambda m, l: dict(PROFILE))
    monkeypatch.setattr(dr, 'install_fine_align', lambda driver, log=print: calls.append(driver) or True)
    monkeypatch.setattr(dr.DemoRunner, 'console_alive', lambda self: True)
    base = ['--map', 'm.yaml', '--layout', 'l.yaml', '--start-node', 'N12']
    dr.main(base)
    assert calls == [] and made['driver'].nav.docking is None
    dr.main(base + ['--fine-align'])
    assert calls == [made['driver']]


def test_reconnect_ping_after_expiry_also_turns_the_boarding_alert_off():
    """탑승 알림을 켠 채 연결이 만료되고 다음 tick 전에 ping 이 와도 주행 정지·추종 취소·알림 해제가 모두 된다."""
    from types import SimpleNamespace
    d, c, r = make()
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}})
    r.handle({'op': 'ping'})
    r.handle({'op': 'alert', 'on': True})
    r.handle({'op': 'follow', 'card_id': 35, 'turn_first': True})     # 돌기 진행 중(active)
    assert r.state()['boarding'] is True and r.state()['active']['type'] == 'navigate.requested'
    c.t += 1.2                                          # 만료. tick 이 돌기 전에
    r.handle({'op': 'ping'})                            # 재연결 ping 이 먼저 도착
    st = r.state()
    assert ('safe_stop', 'control_link_lost') in d.calls             # 주행 정지
    assert st['active'] is None and st['pending'] is None            # 돌기 뒤 추종도 취소
    assert st['boarding'] is False                                   # 탑승 알림 해제
    assert ('boarding_alert', False) in d.calls                      # 드라이버 hook 으로 끔
    kinds = [(e['kind'], e.get('reason')) for e in r.events]
    assert ('alert', 'control_link_lost') in kinds and ('stopped', 'control_link_lost') in kinds
    r.tick()                                            # 다음 tick: 다시 살아 있음, 알림은 꺼진 채
    assert r.state()['boarding'] is False and r.console_alive() is True
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')


def test_turn_op_turns_in_place_and_waits():
    """turn: 지금 AMCL 자리에서 반대 방향 Nav2 목표(demo_TURN)만 보내고 추종은 보내지 않는다(ARR 도착 뒤 대기)."""
    from types import SimpleNamespace
    from roport_follow.demo_runner import TURN_GOAL_ID
    d, c, r = make()
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': {'x': 2.6, 'y': -0.02, 'yaw': 0.0}})
    r.handle({'op': 'turn'})
    kind, typ, p = d.calls[-1]
    assert (kind, typ) == ('dispatch', 'navigate.requested') and p['goal_id'] == TURN_GOAL_ID
    assert (p['pose']['x'], p['pose']['y']) == (2.6, -0.02) and abs(p['pose']['yaw']) == pytest.approx(math.pi)
    d.results.append((d.last, 'completed', {}))
    c.t += 5
    r.handle({'op': 'ping'})
    r.tick()
    assert r.state()['active'] is None and r.state()['pending'] is None
    assert all(x[1] != 'follow.requested' for x in d.calls if x[0] == 'dispatch')
    d.nav = SimpleNamespace(snapshot=lambda: {'pose': None})
    r.handle({'op': 'turn'})
    assert r.events[-1]['reason'] == 'turn_pose_unavailable'
