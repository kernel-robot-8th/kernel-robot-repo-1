"""ConsoleNode 를 실제 rclpy 로 띄워 /cmd_vel 발행을 확인 (로봇·Nav2·SLAM 없이).

안전: 별도 ROS_DOMAIN_ID(context) + 테스트 전용 namespace 를 써서
같은 네트워크/기기의 실제 로봇 /cmd_vel 에는 절대 발행되지 않게 한다.
"""

import importlib.util
import threading
import time

import pytest

_NEEDED = ('rclpy', 'nav2_msgs', 'slam_toolbox', 'flask')
_MISSING = [m for m in _NEEDED if importlib.util.find_spec(m) is None]
# 모듈 단위 importorskip 은 이 환경에서 디렉터리 전체 수집을 멈추게 해서 skipif 로 처리
pytestmark = pytest.mark.skipif(bool(_MISSING), reason=f'missing: {_MISSING}')

if not _MISSING:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

from pinky_test_console.teleop import MAX_ANGULAR, MAX_LINEAR  # noqa: E402

TEST_DOMAIN_ID = 199  # 강의실 로봇이 쓰지 않을 높은 번호 (ROS 2 허용 범위 0-232)
NS = '/pinky_console_pytest'


@pytest.fixture
def ros(tmp_path):
    from pinky_test_console.web_server import ConsoleNode
    ctx = Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    node = ConsoleNode(
        context=ctx, namespace=NS,
        parameter_overrides=[rclpy.parameter.Parameter('map_dir', value=str(tmp_path))])
    probe = rclpy.create_node('probe', context=ctx, namespace=NS)
    received = []
    probe.create_subscription(
        Twist, 'cmd_vel',
        lambda m: received.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    # 디스커버리 대기
    deadline = time.monotonic() + 5.0
    while node.cmd_vel_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    assert node.cmd_vel_pub.get_subscription_count() > 0
    yield node, received
    ex.shutdown(timeout_sec=1.0)
    node.destroy_node()
    probe.destroy_node()
    rclpy.shutdown(context=ctx)


def test_topic_is_isolated(ros):
    node, _ = ros
    assert node.cmd_vel_pub.topic_name == NS + '/cmd_vel'


def test_teleop_10hz_clamped_then_deadman(ros):
    node, received = ros
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:   # 브라우저처럼 10 Hz 로 과속 입력
        node.teleop(1.0, -5.0)
        last_input = time.monotonic()
        time.sleep(0.1)
    time.sleep(1.0)

    driving = [r for r in received if r[1] != 0.0 or r[2] != 0.0]
    zeros = [r for r in received if r[1] == 0.0 and r[2] == 0.0]
    assert driving, 'no cmd_vel received'
    assert all(r[1] == pytest.approx(MAX_LINEAR) and r[2] == pytest.approx(-MAX_ANGULAR)
               for r in driving)
    # 10 Hz (1 s 동안 대략 10개; CI 부하 여유)
    assert 7 <= len(driving) <= 13, len(driving)
    # 마지막 입력 후 0.3 s deadman → 0.3~0.45 s 안에 0 수신, 그 뒤 비영 명령 없음
    first_zero = min(r[0] for r in zeros if r[0] > last_input)
    assert 0.25 <= first_zero - last_input <= 0.45, first_zero - last_input
    assert all(r[0] < first_zero for r in driving)
    # 정지 후 조용해짐 (Nav2 를 가리지 않음)
    assert max(r[0] for r in received) - first_zero < 0.5


def test_idle_publishes_nothing(ros):
    _, received = ros
    time.sleep(0.5)
    assert received == []


def test_teleop_stop_publishes_zero_immediately(ros):
    node, received = ros
    node.teleop(0.05, 0.0)
    time.sleep(0.15)
    received.clear()
    t = time.monotonic()
    node.teleop_stop()
    time.sleep(0.1)
    assert received and received[0][1:] == (0.0, 0.0)
    assert received[0][0] - t < 0.1


def test_emergency_stop_without_nav2(ros):
    node, received = ros
    node.teleop(0.05, 0.3)
    t = time.monotonic()
    result = node.emergency_stop()          # Nav2 없음 → cancel 은 n/a
    assert result == {'nav_cancelled': False}
    time.sleep(0.1)
    zeros = [r for r in received if r[0] >= t and r[1:] == (0.0, 0.0)]
    assert zeros and zeros[0][0] - t < 0.1
    assert not node.teleop_guard.active


def test_publish_stop_on_exit(ros):
    node, received = ros
    node.publish_stop_on_exit()
    time.sleep(0.1)
    assert received and received[-1][1:] == (0.0, 0.0)


def test_status_and_state(ros):
    node, _ = ros
    s = node.get_status()
    assert s['memory']['total_kb'] > 0 and s['console_rss_kb'] > 0
    assert s['battery_percent'] is None and s['battery_voltage'] is None
    st = node.get_state_snapshot({'map': -1}, {'map'})
    assert st['map'] is None and st['navigating'] is False


def test_save_map_without_slam_reports_failure(ros):
    node, _ = ros
    r = node.slam_save_map('room')
    assert r['success'] is False and 'not available' in r['msg']
