"""LineNavNode 를 실제 rclpy 로 띄워 명령·정지·감시를 확인한다(로봇·센서 없이).

안전: 별도 ROS_DOMAIN_ID(context) + 테스트 전용 namespace 를 써서
같은 네트워크/기기의 실제 로봇 /cmd_vel 에는 발행되지 않게 한다.
"""

import importlib.util
import json
from pathlib import Path
import threading
import time

import pytest

_NEEDED = ('rclpy', 'geometry_msgs', 'std_msgs', 'nav_msgs', 'roport_pinky_line')
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
    def __init__(self, log_dir):
        from roport_line_nav.line_nav_node import LineNavNode
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=TEST_DOMAIN_ID)
        self.node = LineNavNode(
            context=self.ctx, namespace=NS,
            parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('layout_file', value=str(LAYOUT)),
                Parameter('log_dir', value=str(log_dir)),     # 홈 ~/roport_logs 에 쓰지 않게
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
])
def test_parse_command_rejects(text):
    with pytest.raises(ValueError):
        parse_command(text)


def test_parse_command_ok():
    assert parse_command('{"cmd": "stop"}') == ('stop', {})
    cmd, args = parse_command('{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": 7}')
    assert cmd == 'start' and args == {'route': ['N16', 'N14'], 'start_offset_cm': 7.0}


def test_speed_command(ros):
    """직선·회전 속도 바꾸기(상한 안만), 상태에 보고."""
    ros.feed(3)
    ros.command({'cmd': 'speed', 'base_speed': 0.05, 'rotate_speed': 0.5})
    ros.wait(lambda: ros.states and ros.states[-1]['speed']['base_speed'] == 0.05)
    assert ros.states[-1]['speed']['rotate_speed'] == 0.5
    ros.command({'cmd': 'speed', 'base_speed': 0.2})
    ros.wait(lambda: ros.states and ros.states[-1]['last_command']['cmd'] == 'speed'
             and not ros.states[-1]['last_command']['ok'])
    assert ros.node.ex.cfg.base_speed == 0.05
    assert ros.cmds == []                       # 속도 바꾸기만으로는 움직이지 않음
