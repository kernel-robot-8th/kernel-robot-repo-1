"""LineFollowerNode 를 실제 rclpy 로 띄워 cmd_vel 발행 조건을 확인한다(로봇·센서 없이).

안전: 별도 ROS_DOMAIN_ID(context) + 테스트 전용 namespace 를 써서
같은 네트워크/기기의 실제 로봇 /cmd_vel 에는 발행되지 않게 한다.
"""

import importlib.util
import json
from pathlib import Path
import threading
import time

import pytest

_NEEDED = ('rclpy', 'geometry_msgs', 'std_msgs')
_MISSING = [m for m in _NEEDED if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(_MISSING), reason=f'missing: {_MISSING}')

if not _MISSING:
    from geometry_msgs.msg import Twist
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool, String, UInt16MultiArray

TEST_DOMAIN_ID = 198  # 강의실 로봇이 쓰지 않을 높은 번호(콘솔 테스트 199 와 겹치지 않게)
NS = '/roport_line_pytest'
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'robots.yaml'
CENTER = [2308, 388, 2233]     # pinky_d660: 가운데만 테이프
RIGHT = [2308, 900, 281]       # 라인이 오른쪽
CARPET = [2308, 2376, 2233]


class Harness:
    def __init__(self, dry_run=False):
        from roport_pinky_line.line_follower_node import LineFollowerNode
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=TEST_DOMAIN_ID)
        self.node = LineFollowerNode(
            context=self.ctx, namespace=NS,
            parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('dry_run', value=dry_run),
            ])
        self.probe = rclpy.create_node('probe', context=self.ctx, namespace=NS)
        self.cmds = []
        self.states = []
        self.probe.create_subscription(
            Twist, 'cmd_vel', lambda m: self.cmds.append((m.linear.x, m.angular.z)), 10)
        self.probe.create_subscription(
            String, 'line_follower/state', lambda m: self.states.append(json.loads(m.data)), 10)
        self.ir_pub = self.probe.create_publisher(UInt16MultiArray, 'ir_sensor/range', 10)
        self.enable_pub = self.probe.create_publisher(Bool, 'line_follower/enable', 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        self.ex.add_node(self.probe)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        self.wait(lambda: (self.node.cmd_pub.get_subscription_count() > 0
                           and self.enable_pub.get_subscription_count() > 0
                           and self.ir_pub.get_subscription_count() > 0))

    @staticmethod
    def wait(cond, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not cond() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert cond()

    def ir(self, values, count=4):
        for _ in range(count):
            self.ir_pub.publish(UInt16MultiArray(data=values))
            time.sleep(0.05)

    def enable(self, on):
        self.enable_pub.publish(Bool(data=on))
        self.wait(lambda: self.node.follower.enabled == on)

    def close(self):
        self.ex.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def ros():
    h = Harness()
    yield h
    h.close()


def test_topics_are_isolated(ros):
    assert ros.node.cmd_pub.topic_name == f'{NS}/cmd_vel'


def test_disabled_never_publishes_cmd_vel(ros):
    ros.ir(CENTER)
    ros.wait(lambda: len(ros.states) >= 3)
    time.sleep(0.4)  # IR 끊김 감시도 지나가게
    assert ros.cmds == []
    assert ros.states[-1]['state'] in ('idle',)


def test_follow_then_turn_right_then_stop_on_disable(ros):
    ros.ir(CENTER, count=2)
    ros.enable(True)
    ros.ir(CENTER)
    ros.wait(lambda: ros.cmds)
    assert ros.cmds[-1][0] > 0 and ros.cmds[-1][1] == pytest.approx(0.0)
    ros.ir(RIGHT)
    ros.wait(lambda: ros.cmds[-1][1] < 0)
    ros.enable(False)
    ros.wait(lambda: ros.cmds[-1] == (0.0, 0.0))
    n = len(ros.cmds)
    ros.ir(RIGHT)
    time.sleep(0.2)
    assert len(ros.cmds) == n   # 끈 뒤에는 발행하지 않음


def test_line_lost_and_ir_timeout_publish_zero(ros):
    ros.enable(True)
    ros.ir(CARPET)
    ros.wait(lambda: ros.states and ros.states[-1]['state'] == 'line_lost')
    assert ros.cmds[-1] == (0.0, 0.0)
    ros.ir(CENTER)
    ros.wait(lambda: ros.cmds[-1][0] > 0)
    time.sleep(0.6)  # ir_timeout_sec 0.3
    ros.wait(lambda: ros.states[-1]['state'] == 'no_ir')
    assert ros.cmds[-1] == (0.0, 0.0)


def test_bad_ir_messages_do_not_feed_watchdog(ros):
    ros.enable(True)
    ros.ir(CENTER)
    ros.wait(lambda: ros.cmds and ros.cmds[-1][0] > 0)
    ros.ir([100, 200], count=12)   # 20 Hz 로 0.6초 동안 잘못된 메시지만
    # 보내는 도중(마지막 메시지 뒤 0.3초가 지나기 전)에 이미 멈춰 있어야 한다
    assert ros.states[-1]['state'] == 'no_ir'
    assert ros.cmds[-1] == (0.0, 0.0)


def test_read_failure_zeros_never_start(ros):
    ros.ir([0, 0, 0], count=2)
    ros.enable(True)
    ros.ir([0, 0, 0], count=20)    # 1초
    assert all(linear == 0.0 for linear, _ in ros.cmds)
    assert ros.states[-1]['state'] == 'no_ir'


def test_stop_on_exit_publishes_zero_when_enabled(ros):
    ros.enable(True)
    ros.ir(CENTER)
    ros.wait(lambda: ros.cmds and ros.cmds[-1][0] > 0)
    ros.node.publish_stop_on_exit()
    ros.wait(lambda: ros.cmds[-1] == (0.0, 0.0))


def test_dry_run_never_publishes():
    h = Harness(dry_run=True)
    try:
        h.enable(True)
        h.ir(RIGHT)
        h.wait(lambda: h.states and h.states[-1]['cmd'] and h.states[-1]['cmd']['angular'] < 0)
        h.node.publish_stop_on_exit()
        time.sleep(0.2)
        assert h.cmds == []
    finally:
        h.close()
