"""LineNavNode 를 실제 rclpy 로 띄워 명령·정지·감시를 확인한다(로봇·센서 없이).

안전: 별도 ROS_DOMAIN_ID(context) + 테스트 전용 namespace 를 써서
같은 네트워크/기기의 실제 로봇 /cmd_vel 에는 발행되지 않게 한다.
"""

import importlib.util
import json
import math
from pathlib import Path
import threading
import time

import pytest

_NEEDED = ('rclpy', 'geometry_msgs', 'std_msgs', 'nav_msgs', 'sensor_msgs', 'roport_pinky_line')
_MISSING = [m for m in _NEEDED if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(_MISSING), reason=f'missing: {_MISSING}')

if not _MISSING:
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String, UInt16MultiArray

    from roport_line_nav.line_nav_node import parse_command

TEST_DOMAIN_ID = 197  # 라인 추종 테스트 198, 콘솔 199 와 겹치지 않게
NS = '/roport_line_nav_pytest'
ROOT = Path(__file__).resolve().parents[2]
LAYOUT = ROOT / 'roport_gz_world' / 'layout' / 'roport_airport.yaml'
CONFIG = ROOT / 'roport_pinky_line' / 'config' / 'robots.yaml'
CENTER = [2308, 388, 2233]     # pinky_d660: 가운데만 테이프
ROUTE = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']


class Harness:
    def __init__(self, log_dir, extra=(), dock_file=None):
        from roport_line_nav.line_nav_node import LineNavNode
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=TEST_DOMAIN_ID)
        # 홈 ~/roport_dock.yaml 을 읽거나 쓰지 않게 테스트 임시 경로
        self.dock_file = str(dock_file or Path(log_dir).parent / 'roport_dock.yaml')
        self.node = LineNavNode(
            context=self.ctx, namespace=NS,
            parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('layout_file', value=str(LAYOUT)),
                Parameter('log_dir', value=str(log_dir)),     # 홈 ~/roport_logs 에 쓰지 않게
                Parameter('dock_file', value=self.dock_file),
                *extra,
            ])
        self.probe = rclpy.create_node('probe', context=self.ctx, namespace=NS)
        self.cmds = []
        self.states = []
        self.probe.create_subscription(
            Twist, 'cmd_vel', lambda m: self.cmds.append((m.linear.x, m.angular.z)), 10)
        self.probe.create_subscription(
            String, 'line_nav/state', lambda m: self.states.append(json.loads(m.data)), 10)
        self.ir_pub = self.probe.create_publisher(UInt16MultiArray, 'ir_sensor/range', 10)
        self.odom_pub = self.probe.create_publisher(Odometry, 'odom', 10)
        self.cmd_pub = self.probe.create_publisher(String, 'line_nav/command', 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        self.ex.add_node(self.probe)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        self.wait(lambda: (self.node.cmd_pub.get_subscription_count() > 0
                           and self.cmd_pub.get_subscription_count() > 0
                           and self.ir_pub.get_subscription_count() > 0
                           and self.odom_pub.get_subscription_count() > 0))

    @staticmethod
    def wait(cond, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not cond() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert cond()

    def feed(self, count=4, ir=CENTER, odom=True):
        for _ in range(count):
            if odom:
                self.odom_pub.publish(Odometry())
            self.ir_pub.publish(UInt16MultiArray(data=ir))
            time.sleep(0.05)

    def command(self, data):
        self.cmd_pub.publish(String(data=json.dumps(data)))

    def close(self):
        self.ex.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def ros(tmp_path):
    h = Harness(tmp_path / 'logs')
    yield h
    h.close()


def test_topics_are_isolated(ros):
    assert ros.node.cmd_pub.topic_name == f'{NS}/cmd_vel'


def test_idle_never_publishes(ros):
    ros.feed(6)
    assert ros.cmds == []
    ros.wait(lambda: ros.states and ros.states[-1]['state'] == 'idle')
    assert ros.states[-1]['graph_version'].startswith('line-v14')


def test_start_moves_then_stop_publishes_zero(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(4)
    assert any(v > 0 for v, _ in ros.cmds)
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')
    ros.wait(lambda: ros.cmds and ros.cmds[-1] == (0.0, 0.0))
    n = len(ros.cmds)
    ros.feed(4)
    assert len(ros.cmds) == n          # 정지 뒤에는 발행하지 않음(다른 주행 소스를 가리지 않게)


def test_bad_command_rejected_and_reported(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ['N16', 'N02']})
    ros.wait(lambda: ros.states and ros.states[-1]['last_command']
             and ros.states[-1]['last_command']['ok'] is False)
    assert 'not linked' in ros.states[-1]['last_command']['detail']
    assert ros.node.ex.state == 'idle' and ros.cmds == []


def test_junction_goal_requires_explicit_hold_command(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ['N16', 'N14']})
    ros.wait(lambda: ros.states and ros.states[-1]['last_command']
             and ros.states[-1]['last_command']['ok'] is False)
    assert ros.node.ex.state == 'idle'
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ['N16', 'N14'], 'hold_at_goal': True})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    assert ros.node.ex.legs[-1].action == 'stop'
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')


def test_ir_timeout_stops(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(2)
    ros.wait(lambda: ros.node.ex.state == 'error', timeout=2.0)   # IR 이 끊기면 0.3 s
    assert ros.node.ex.reason == 'no_ir'
    ros.wait(lambda: ros.cmds[-1] == (0.0, 0.0))


def test_no_odom_stops(ros):
    ros.feed(3, odom=False)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'error')
    assert ros.node.ex.reason == 'no_odom'


@pytest.mark.parametrize('text', [
    'not json', '[]', '{"cmd": 1}', '{"cmd": "fly"}',
    '{"cmd": "start", "route": "N16"}',
    '{"cmd": "start", "route": ["N16", "n14; rm -rf"]}',
    '{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": 99}',
    '{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": true}',
    '{"cmd": "speed"}', '{"cmd": "speed", "base_speed": "fast"}',
    '{"cmd": "speed", "base_speed": NaN}', '{"cmd": "speed", "rotate_speed": true}',
    '{"cmd": "dock"}', '{"cmd": "dock", "node": "n11"}',
    '{"cmd": "dock", "node": "N11", "advance_cm": 1, "rotate_deg": 90}',
    '{"cmd": "dock", "node": "N11", "advance_cm": "1", "rotate_deg": 90, "move_cm": 0}',
    '{"cmd": "dock", "node": "N11", "advance_cm": true, "rotate_deg": 90, "move_cm": 0}',
    '{"cmd": "dock", "node": "N11", "advance_cm": NaN, "rotate_deg": 90, "move_cm": 0}',
    '{"cmd": "dock", "node": "N11", "reset": "yes"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": 1, "start_facing": "S"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": true}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": true, "start_facing": "south"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_facing": "S"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": true, "start_facing": "S",'
    ' "start_offset_cm": 3}',
    '{"cmd": "back"}', '{"cmd": "back", "to": "N23"}',          # 수동 후진은 없앰(2026-11-15)
    '{"cmd": "speed", "reverse_speed": "slow"}', '{"cmd": "speed", "reverse_speed": 0.03}',
])
def test_parse_command_rejects(text):
    with pytest.raises(ValueError):
        parse_command(text)


def test_parse_command_ok():
    assert parse_command('{"cmd": "stop"}') == ('stop', {})
    cmd, args = parse_command('{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": 7}')
    assert cmd == 'start' and args == {'route': ['N16', 'N14'], 'start_offset_cm': 7.0,
                                       'start_cross': False, 'start_facing': None,
                                       'confirmed_start': False, 'hold_at_goal': False}
    _, args = parse_command('{"cmd":"start","route":["N01","N18"],"hold_at_goal":true}')
    assert args['hold_at_goal'] is True
    with pytest.raises(ValueError, match='hold_at_goal'):
        parse_command('{"cmd":"start","route":["N01","N18"],"hold_at_goal":1}')
    _, args = parse_command('{"cmd":"start","route":["N12","N08"],"start_offset_cm":0,"confirmed_start":true}')
    assert args['confirmed_start'] is True
    with pytest.raises(ValueError, match='confirmed_start'):
        parse_command('{"cmd":"start","route":["N16","N14"],"start_offset_cm":0,"confirmed_start":true}')
    cmd, args = parse_command('{"cmd": "start", "route": ["N07", "N18", "N01"], '
                              '"start_cross": true, "start_facing": "S"}')
    assert args['start_cross'] is True and args['start_facing'] == 'S'
    assert parse_command('{"cmd": "dock", "node": "N11", "advance_cm": 1.5, "rotate_deg": 96, '
                         '"move_cm": -2}') == ('dock', {'node': 'N11', 'reset': False,
                                                        'advance_cm': 1.5, 'rotate_deg': 96.0,
                                                        'move_cm': -2.0})
    assert parse_command('{"cmd": "dock", "node": "N11", "reset": true}') == \
        ('dock', {'node': 'N11', 'reset': True})
    assert parse_command('{"cmd": "pause"}') == ('pause', {})
    assert parse_command('{"cmd": "resume"}') == ('resume', {})
    assert parse_command('{"cmd": "speed", "base_speed": 0.03, "reverse_speed": 0.03}') == \
        ('speed', {'base_speed': 0.03, '_ignored': 'reverse_speed'})   # 양보 후진 삭제: 무시하고 알림


def test_speed_command(ros):
    """직선·회전 속도 바꾸기(상한 안만), 상태에 보고."""
    ros.feed(3)
    ros.command({'cmd': 'speed', 'base_speed': 0.05, 'rotate_speed': 0.5})
    ros.wait(lambda: ros.states and ros.states[-1]['speed']['base_speed'] == 0.05)
    assert ros.states[-1]['speed']['rotate_speed'] == 0.5
    # 노드 앞 감속은 별도 파라미터(approach_speed·approach_lead_cm): 직선 속도를 바꿔도 그대로
    assert ros.states[-1]['speed']['approach_speed'] == 0.03
    assert ros.states[-1]['speed']['approach_lead_cm'] == 5.0
    ros.command({'cmd': 'speed', 'base_speed': 0.2})
    ros.wait(lambda: ros.states and ros.states[-1]['last_command']['cmd'] == 'speed'
             and not ros.states[-1]['last_command']['ok'])
    assert ros.node.ex.cfg.base_speed == 0.05
    assert ros.cmds == []                       # 속도 바꾸기만으로는 움직이지 않음


def test_approach_parameters(tmp_path):
    """approach_speed·approach_lead_cm 파라미터: 범위 안이면 적용, 밖이면 노드가 뜨지 않음."""
    h = Harness(tmp_path / 'logs', extra=[Parameter('base_speed', value=0.05),
                                          Parameter('approach_speed', value=0.02),
                                          Parameter('approach_lead_cm', value=8.0)])
    try:
        assert h.node.ex.cfg.approach_lead_cm == 8.0
        assert h.node.ex.approach_speed == 0.02 and h.node.ex.cfg.base_speed == 0.05
    finally:
        h.close()
    from roport_line_nav.line_nav_node import LineNavNode
    ctx = Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    try:
        with pytest.raises(ValueError, match='approach_speed'):
            LineNavNode(context=ctx, namespace=NS, parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('layout_file', value=str(LAYOUT)),
                Parameter('log_dir', value=str(tmp_path / 'logs2')),
                Parameter('dock_file', value=str(tmp_path / 'dock2.yaml')),
                Parameter('approach_speed', value=0.2)])     # 하드 상한 0.15 초과
    finally:
        rclpy.shutdown(context=ctx)


# ---------------------------------------------------------------- 정밀 도킹(FFT1-112)
def _last(ros, cmd):
    ros.wait(lambda: ros.states and (ros.states[-1]['last_command'] or {}).get('cmd') == cmd)
    return ros.states[-1]['last_command']


def test_dock_command_saves_and_reloads(tmp_path):
    import yaml
    h = Harness(tmp_path / 'logs')
    try:
        assert h.node.ex.dock.as_dict() == {}            # 파일 없음 = 기본
        h.feed(2)
        h.command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1.5, 'rotate_deg': 96,
                   'move_cm': 2})
        last = _last(h, 'dock')
        assert last['ok'], last
        h.wait(lambda: h.states[-1]['dock'].get('N11'))
        assert h.states[-1]['dock'] == {
            'N11': {'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': 2.0}}
        h.command({'cmd': 'dock', 'node': 'N01', 'advance_cm': -2, 'rotate_deg': 84,
                   'move_cm': -3})
        h.wait(lambda: 'N01' in h.states[-1]['dock'])
        data = yaml.safe_load(Path(h.dock_file).read_text())
        assert data['version'] == 1 and set(data['nodes']) == {'N01', 'N11'}
        h.command({'cmd': 'dock', 'node': 'N01', 'reset': True})
        h.wait(lambda: 'N01' not in h.states[-1]['dock'])
        assert h.cmds == []                              # 값 바꾸기만으로는 움직이지 않음
    finally:
        h.close()
    h = Harness(tmp_path / 'logs')                       # 다시 켜면 파일에서 읽음
    try:
        assert h.node.ex.dock.as_dict() == {
            'N11': {'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': 2.0}}
    finally:
        h.close()


def test_dock_command_rejected(ros):
    ros.feed(3)
    for bad in ({'node': 'N11', 'advance_cm': 7, 'rotate_deg': 90, 'move_cm': 0},
                {'node': 'N11', 'advance_cm': 0, 'rotate_deg': 130, 'move_cm': 0},
                {'node': 'N16', 'advance_cm': 0, 'rotate_deg': 90, 'move_cm': 0}):
        n = len(ros.states)
        ros.command({'cmd': 'dock', **bad})
        ros.wait(lambda: len(ros.states) > n and ros.states[-1]['last_command']['cmd'] == 'dock')
        assert ros.states[-1]['last_command']['ok'] is False
    assert not Path(ros.dock_file).exists() and ros.node.ex.dock.as_dict() == {}
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1, 'rotate_deg': 90, 'move_cm': 0})
    ros.wait(lambda: ros.states[-1]['last_command']['cmd'] == 'dock')
    last = ros.states[-1]['last_command']
    assert last['ok'] is False and 'while driving' in last['detail']
    assert ros.node.ex.state == 'follow' and not Path(ros.dock_file).exists()
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')


def test_broken_dock_file_uses_defaults(tmp_path):
    bad = tmp_path / 'roport_dock.yaml'
    bad.write_text('version: 1\nnodes:\n  N11: {advance_cm: 99, rotate_deg: 90, move_cm: 0}\n')
    h = Harness(tmp_path / 'logs', dock_file=bad)
    try:
        assert h.node.ex.dock.as_dict() == {}
        h.feed(2)
        h.command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1, 'rotate_deg': 90, 'move_cm': 0})
        assert _last(h, 'dock')['ok']                    # 깨진 파일은 새 값으로 원자적 교체
        assert 'advance_cm: 1.0' in bad.read_text()
    finally:
        h.close()


def test_start_cross_command(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ['N07', 'N18', 'N01'], 'start_cross': True,
                 'start_facing': 'S'})
    ros.wait(lambda: ros.node.ex.state == 'advance')
    ros.feed(2)
    assert any(v > 0 for v, _ in ros.cmds)
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')
    ros.command({'cmd': 'start', 'route': ROUTE, 'start_cross': True, 'start_facing': 'S'})
    last = _last(ros, 'start')
    assert last['ok'] is False and 'bar' in last['detail']


# ---------------------------------------------------------------- 옆선 켜짐 확인 파라미터(라인 유실 복구는 없앰)
def test_side_rise_parameters_and_no_recovery(tmp_path):
    h = Harness(tmp_path / 'logs', extra=[Parameter('side_rise_only', value=False),
                                          Parameter('side_rise_off_samples', value=3)])
    try:
        c = h.node.ex.cfg
        assert (c.side_rise_only, c.side_rise_off_samples, c.side_rise_tee) == (False, 3, False)
        assert not hasattr(c, 'gap_max_cm') and not h.node.has_parameter('gap_max_cm')
        h.feed(2)
        h.wait(lambda: h.states and 'obstacle' in h.states[-1])
        assert 'recovery' not in h.states[-1]
    finally:
        h.close()


def test_pause_resume_commands(ros):
    """pause → 0 속도·paused(정지 확인), resume → 주행. back 은 알 수 없는 명령으로 거절."""
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(2)
    ros.command({'cmd': 'back', 'to': 'N16'})
    ros.wait(lambda: ros.states and (ros.states[-1]['last_command'] or {}).get('cmd') == '?')
    last = ros.states[-1]['last_command']
    assert last['ok'] is False and 'unknown cmd' in last['detail'] and ros.node.ex.state == 'follow'
    ros.command({'cmd': 'pause'})
    ros.wait(lambda: ros.node.ex.state == 'paused')
    ros.feed(10)                                        # odom 이 그대로 → 0.3 s 뒤 정지 확인
    ros.wait(lambda: ros.states and (ros.states[-1].get('pause') or {}).get('settled'))
    assert ros.cmds[-1] == (0.0, 0.0) and ros.states[-1]['pause_reason'] == 'manual'
    assert 'back_targets' not in ros.states[-1] and ros.states[-1]['obstacle']['mode'] == 'off'
    ros.command({'cmd': 'resume'})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(2)
    assert ros.cmds[-1][0] > 0
    ros.command({'cmd': 'speed', 'base_speed': 0.03, 'reverse_speed': 0.03})
    last = _last(ros, 'speed')
    assert last['ok'] and 'reverse_speed ignored' in last['detail']
    assert 'reverse' not in ros.states[-1] and 'reverse_follow_speed' not in ros.states[-1]['speed']
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')


# ---------------------------------------------------------------- 장애물 정지(FFT1-172)
SCAN_FRAME = 'pytest_rplidar_link'


def _laser(points_base_cm, frame=SCAN_FRAME, mount=(-0.017, 0.0, math.pi), step_deg=1.0):
    """base_link 점(cm) → 라이다(yaw π 장착) LaserScan. 각 점은 가장 가까운 광선 하나."""
    from sensor_msgs.msg import LaserScan
    n = int(round(360 / step_deg))
    inc = math.radians(step_deg)
    ranges = [math.inf] * n
    mx, my, myaw = mount
    for px, py in points_base_cm:
        dx, dy = px / 100.0 - mx, py / 100.0 - my
        a = math.atan2(dy, dx) - myaw
        a = (a + math.pi) % (2 * math.pi) - math.pi
        ranges[int(round((a + math.pi) / inc)) % n] = math.hypot(dx, dy)
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.angle_min, msg.angle_increment = -math.pi, inc
    msg.angle_max = -math.pi + inc * (n - 1)
    msg.range_min, msg.range_max = 0.15, 12.0
    msg.ranges = ranges
    return msg


def _static_tf(probe, frame=SCAN_FRAME):
    """URDF 와 같은 장착: base_link → rplidar_link (x −0.017, z 0.097, yaw π)."""
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    b = StaticTransformBroadcaster(probe)
    t = TransformStamped()
    t.header.stamp = probe.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = frame
    t.transform.translation.x = -0.017
    t.transform.translation.z = 0.097
    t.transform.rotation.z = 1.0          # yaw π
    t.transform.rotation.w = 0.0
    b.sendTransform(t)
    return b


@pytest.mark.skipif(importlib.util.find_spec('tf2_ros') is None, reason='tf2_ros missing')
def test_scan_tf_rotated_front_rear(tmp_path):
    """라이다 yaw = π: TF 로 돌린 앞 30 cm·뒤 20 cm 점 → front 25.8·rear 12.5(원시 각도면 앞뒤가 바뀜). monitor 는 동작 그대로."""
    from sensor_msgs.msg import LaserScan
    h = Harness(tmp_path / 'logs', extra=[Parameter('obstacle_mode', value='monitor')])
    try:
        keep = _static_tf(h.probe)
        pub = h.probe.create_publisher(LaserScan, 'scan', 10)
        h.wait(lambda: pub.get_subscription_count() > 0)

        def ob():
            return (h.states[-1].get('obstacle') or {}) if h.states else {}
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and ob().get('front_cm') is None:
            pub.publish(_laser([(30.0, 0.0), (-20.0, 0.0)]))
            h.feed(2)
        assert ob()['mode'] == 'monitor', ob()
        assert ob()['front_cm'] == pytest.approx(25.8, abs=0.1)
        assert ob()['rear_cm'] == pytest.approx(12.5, abs=0.1)
        assert h.cmds == [] and keep is not None
    finally:
        h.close()


def test_stop_mode_without_scan_holds(tmp_path):
    """stop 모드에서 scan 이 없으면 첫 추종 샘플에서 scan_stale 정지(0 속도), 경로는 끝내지 않음."""
    h = Harness(tmp_path / 'logs', extra=[Parameter('obstacle_mode', value='stop')])
    try:
        h.feed(3)
        h.command({'cmd': 'start', 'route': ROUTE})
        h.wait(lambda: h.node.ex.state == 'paused')
        h.feed(3)
        st = h.states[-1]
        assert st['pause_reason'] == 'scan_stale' and st['obstacle']['phase'] == 'blocked'
        assert h.cmds and h.cmds[-1] == (0.0, 0.0) and all(v <= 0.0 for v, _ in h.cmds)
        h.command({'cmd': 'resume'})
        last = _last(h, 'resume')
        assert last['ok'] is False and 'obstacle stop' in last['detail']
        h.command({'cmd': 'stop'})
        h.wait(lambda: h.node.ex.state == 'stopped')
    finally:
        h.close()


def test_obstacle_exempt_nodes_parameter(tmp_path):
    """obstacle_exempt_nodes: 기본 N01,N11,N21,N22, 문자열로 바꿀 수 있고, 노드 ID 가 아니면 노드가 뜨지 않음."""
    h = Harness(tmp_path / 'logs', extra=[Parameter('obstacle_exempt_nodes', value='N11, N22')])
    try:
        assert h.node.ex.cfg.obstacle_exempt_nodes == ('N11', 'N22')
        assert h.node.ex.cfg.stop_cm == 5.0 and h.node.ex.cfg.blind_near_cm == 3.0
    finally:
        h.close()
    from roport_line_nav.line_nav_node import LineNavNode
    ctx = Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    try:
        with pytest.raises(ValueError, match='obstacle_exempt_nodes'):
            LineNavNode(context=ctx, namespace=NS, parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('layout_file', value=str(LAYOUT)),
                Parameter('log_dir', value=str(tmp_path / 'logs2')),
                Parameter('dock_file', value=str(tmp_path / 'dock2.yaml')),
                Parameter('obstacle_exempt_nodes', value='N11;rm')])
    finally:
        rclpy.shutdown(context=ctx)
