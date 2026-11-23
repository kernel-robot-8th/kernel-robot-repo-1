"""마커 없는 정밀 접근(FineAlign)·드라이버 설치 조건·탑승 알림 hook(ROS 노드 없음)."""
import math
import sys
import threading
import time
import types
from types import SimpleNamespace

import pytest

from roport_pinky.fine_align import FineAlign, goals_from_env
from roport_pinky.trial_driver import TrialDriver
from roport_pinky.trial_nav2 import Nav2Bridge


@pytest.mark.parametrize('raw, goals', [
    (None, ()), ('', ()), (' , ,', ()), ('N12_RETURN', ('N12_RETURN',)),
    (' N12_RETURN , ,G3 ', ('N12_RETURN', 'G3')),
])
def test_goals_from_env_strips_and_drops_empty(raw, goals):
    env = {} if raw is None else {'ROPORT_FINE_ALIGN_GOALS': raw}
    assert goals_from_env(env) == goals


def test_fine_align_returns_the_commanded_pose_inside_the_envelope_only():
    f = FineAlign(('N12_RETURN',))
    assert f.enabled('N12_RETURN') and not f.enabled('G3')
    with pytest.raises(ValueError, match='fine_align_no_goal'):
        f.target({'x': 0., 'y': 0., 'yaw': 0.}, 0.)
    goal = {'goal_id': 'N12_RETURN', 'pose': {'x': 2.2642, 'y': -1.2839, 'yaw': math.pi}}
    f.reset(goal)
    f.begin(10.0)
    assert f.target({'x': 2.4642, 'y': -1.2839, 'yaw': 3.1}, 11.0) == {'x': 2.2642, 'y': -1.2839, 'yaw': math.pi}
    with pytest.raises(ValueError, match='fine_align_outside_envelope'):     # 40 cm
        f.target({'x': 2.6642, 'y': -1.2839, 'yaw': math.pi}, 11.0)
    with pytest.raises(ValueError, match='fine_align_outside_envelope'):     # 0.5 rad
        f.target({'x': 2.4, 'y': -1.2839, 'yaw': math.pi - 0.5}, 11.0)
    with pytest.raises(ValueError, match='fine_align_timeout'):
        f.target({'x': 2.3, 'y': -1.2839, 'yaw': math.pi}, 36.0)
    assert (f.MAX_OFFSET_M, f.MAX_YAW_RAD, f.TIMEOUT_S) == (0.30, 0.35, 25.0)
    f.add_goals(('demo_RET_N12',))
    assert f.goals == {'N12_RETURN', 'demo_RET_N12'}


# ---- Nav2Bridge: 환경 변수로 docking 슬롯 채우기

class FakeGate:
    def __init__(self, node, tf, path):
        self.path = path


def install(monkeypatch, gate=None, goals=None):
    for key, value in (('ROPORT_GATE_DOCK_CONFIG', gate), ('ROPORT_FINE_ALIGN_GOALS', goals)):
        if value is None:
            monkeypatch.delenv(key, raising=False)
        else:
            monkeypatch.setenv(key, value)
    import roport_pinky.gate_docking as gate_docking
    monkeypatch.setattr(gate_docking, 'GateDocking', FakeGate)
    bridge = object.__new__(Nav2Bridge)
    bridge.tf, bridge.docking, bridge.dock_action, bridge.dock_phase = None, None, None, None
    warnings, actions = [], []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append))
    bridge._install_docking(node, lambda: actions.append('follow_path') or 'client')
    return bridge, warnings, actions


@pytest.mark.parametrize('goals', [None, '', ' , '])
def test_no_env_keeps_docking_off(monkeypatch, goals):
    bridge, warnings, actions = install(monkeypatch, goals=goals)
    assert bridge.docking is None and bridge.dock_action is None and actions == [] and warnings == []


def test_fine_align_goals_install_fine_align(monkeypatch):
    bridge, warnings, actions = install(monkeypatch, goals=' N12_RETURN, ')
    assert isinstance(bridge.docking, FineAlign) and bridge.docking.goals == {'N12_RETURN'}
    assert bridge.dock_action == 'client' and actions == ['follow_path'] and warnings == []


def test_gate_docking_wins_and_warns_once(monkeypatch):
    bridge, warnings, actions = install(monkeypatch, gate='/unused/gate.yaml', goals='N12_RETURN')
    assert isinstance(bridge.docking, FakeGate) and bridge.docking.path == '/unused/gate.yaml'
    assert actions == ['follow_path'] and len(warnings) == 1
    bridge, warnings, _ = install(monkeypatch, gate='/unused/gate.yaml')
    assert isinstance(bridge.docking, FakeGate) and warnings == []


# ---- TrialDriver.set_boarding_alert

class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def boarding_driver(monkeypatch):
    """heartbeat·safe_stop·_guard_fault 는 실제 코드를 쓰고 ROS 출력만 가짜로 둔다. 시간은 가짜 시계."""
    import roport_pinky.trial_driver as trial_driver
    clock = Clock()
    monkeypatch.setattr(trial_driver, 'time', SimpleNamespace(monotonic=clock, time=time.time))
    driver = object.__new__(TrialDriver)
    driver.lock = threading.RLock()
    driver.boarding = False
    driver.boarding_runner_at = driver.runner_seen = None
    driver.halted = driver.guard = driver.guard_at = None
    driver.nav_command = driver.follow = driver.failed_navigation = None
    driver.nav = SimpleNamespace(cancel=lambda: None, result=None)
    driver.logic = SimpleNamespace(safe_stop=lambda reason: None, active=None)
    driver._lease = lambda owner: None
    sent = []
    driver.boarding_pub = SimpleNamespace(publish=sent.append)
    driver._Bool = lambda data: data
    driver.heartbeat()                                  # runner 가 살아 있음(guard 없음 = 구동 불가여도)
    return driver, sent, clock


def run(driver, clock, seconds, *, beats=True):
    """0.1 s 마다 runner heartbeat(beats), 0.5 s 마다 알림 tick — demo_runner·드라이버 타이머와 같은 주기."""
    for step in range(int(round(seconds * 10))):
        clock.t += 0.1
        if beats:
            driver.heartbeat()
        if step % 5 == 4:
            driver._boarding_tick()


def test_boarding_alert_publishes_while_on_and_false_once_when_off(monkeypatch):
    driver, sent, clock = boarding_driver(monkeypatch)
    driver._boarding_tick()
    assert sent == []                                   # 끈 상태: 아무것도 안 냄
    driver.set_boarding_alert(True)
    assert sent == [True]                               # 켤 때 바로 한 번
    run(driver, clock, 1.0)
    assert sent == [True, True, True]                   # 0.5 s 마다
    driver.set_boarding_alert(False)
    assert sent == [True, True, True, False] and driver.boarding is False
    run(driver, clock, 2.0)
    assert sent == [True, True, True, False]            # 끈 뒤 멈춤


@pytest.mark.parametrize('value', ['yes', 1, None, 0.0])
def test_boarding_alert_rejects_non_bool(monkeypatch, value):
    driver, sent, _ = boarding_driver(monkeypatch)
    with pytest.raises(ValueError, match='invalid_alert'):
        driver.set_boarding_alert(value)
    assert sent == [] and driver.boarding is False


def test_boarding_alert_stays_on_after_safe_stop_halts_the_drive(monkeypatch):
    """(a) 주행 중 운영자 정지로 halted 가 남아도 runner heartbeat 가 오는 동안 알림은 계속 나간다."""
    driver, sent, clock = boarding_driver(monkeypatch)
    driver.set_boarding_alert(True)
    driver.nav_command = {'type': 'navigate.requested'}
    driver.safe_stop('operator_stop')
    assert driver.halted == 'operator_stop' and driver._guard_fault() == 'operator_stop'
    run(driver, clock, 3.0)
    assert driver.boarding is True and sent == [True] * 7 and False not in sent
    # (b) halted 인 채로 끄고 다시 켜도 된다
    driver.set_boarding_alert(False)
    driver.set_boarding_alert(True)
    run(driver, clock, 1.0)
    assert sent[7:] == [False, True, True, True] and driver.boarding is True


def test_boarding_alert_turns_off_when_runner_heartbeat_is_lost(monkeypatch):
    """(c) heartbeat 가 1 s 넘게 없으면 halted 여부와 상관없이 끈다. 다시 켜도 heartbeat 가 없으면 바로 끈다."""
    driver, sent, clock = boarding_driver(monkeypatch)
    driver.set_boarding_alert(True)
    clock.t += 0.9                                      # 마지막 heartbeat 0.9 s 뒤: 유지
    driver._boarding_tick()
    assert driver.boarding is True and sent == [True, True]
    clock.t += 0.2                                      # 1.1 s: 끔
    driver._boarding_tick()
    assert sent == [True, True, False] and driver.boarding is False
    driver._boarding_tick()
    assert sent == [True, True, False]
    driver.set_boarding_alert(True)
    assert sent[-1] is False and driver.boarding is False
    driver.heartbeat()                                  # runner 가 돌아오면 다시 켤 수 있다
    driver.set_boarding_alert(True)
    assert sent[-1] is True and driver.boarding is True


def test_boarding_alert_survives_guard_fault_while_runner_beats(monkeypatch):
    """(d) trial_guard 상태가 끊겨(drive_guard_unavailable) 구동권 heartbeat 가 막혀도 runner 가 부르면 유지."""
    driver, sent, clock = boarding_driver(monkeypatch)
    driver.set_boarding_alert(True)
    assert driver._guard_fault() == 'drive_guard_unavailable'
    run(driver, clock, 3.0)
    assert driver.runner_seen is None                   # 구동권용 생존 시각은 그대로(갱신 안 됨)
    assert driver.boarding is True and sent == [True] * 7


def make_closable(driver, monkeypatch, order):
    fake_rclpy = types.ModuleType('rclpy')
    fake_rclpy.ok = lambda: False
    monkeypatch.setitem(sys.modules, 'rclpy', fake_rclpy)
    driver._ex = SimpleNamespace(shutdown=lambda timeout_sec: order.append('shutdown'))
    driver.thread = SimpleNamespace(join=lambda timeout: order.append('join'))
    driver.node = SimpleNamespace(destroy_node=lambda: order.append('destroy'))


def test_close_turns_boarding_alert_off(monkeypatch):
    driver, sent, _ = boarding_driver(monkeypatch)
    driver.set_boarding_alert(True)
    order = []
    make_closable(driver, monkeypatch, order)
    driver.close()
    assert sent == [True, False] and driver.boarding is False
    assert order == ['shutdown', 'join', 'destroy']


def test_close_still_cleans_up_when_publish_fails_after_context_shutdown(monkeypatch):
    driver, sent, _ = boarding_driver(monkeypatch)
    driver.set_boarding_alert(True)

    def dead(msg):
        raise RuntimeError('context is not valid')
    driver.boarding_pub = SimpleNamespace(publish=dead)
    order = []
    make_closable(driver, monkeypatch, order)
    driver.close()
    assert driver.boarding is False and order == ['shutdown', 'join', 'destroy']
