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

LAYOUT = __import__('os').path.join(
    __import__('os').path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
    'roport_airport.yaml')
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


def test_route_mode_commands_and_estop(tmp_path):
    """mode:=route: 화면 명령이 line_nav/command 로 나가고, 정지 버튼은 경로 주행도 멈춘다."""
    import json
    from std_msgs.msg import String
    from pinky_test_console.web_server import ConsoleNode
    ctx = Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    node = ConsoleNode(context=ctx, namespace=NS, parameter_overrides=[
        rclpy.parameter.Parameter('map_dir', value=str(tmp_path)),
        rclpy.parameter.Parameter('mode', value='route'),
        rclpy.parameter.Parameter('layout_yaml', value=LAYOUT)])
    probe = rclpy.create_node('probe', context=ctx, namespace=NS)
    got = []
    probe.create_subscription(String, 'line_nav/command', lambda m: got.append(json.loads(m.data)),
                              10)
    state_pub = probe.create_publisher(String, 'line_nav/state', 10)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        deadline = time.monotonic() + 5.0
        while node.route_cmd_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert node.route_snapshot()['stale']
        node.route_command({'cmd': 'start', 'route': ['N14', 'N16']})
        node.emergency_stop()
        deadline = time.monotonic() + 3.0
        while len(got) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert got == [{'cmd': 'start', 'route': ['N14', 'N16']}, {'cmd': 'stop'}]
        for _ in range(5):
            state_pub.publish(String(data=json.dumps({'state': 'follow'})))
            time.sleep(0.05)
        snap = node.route_snapshot()
        assert snap['state'] == {'state': 'follow'} and not snap['stale']
        assert node.route_graph()['nodes']
    finally:
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown(context=ctx)


def test_monitor_mode_has_no_cmd_vel_publisher_and_refuses_drive(tmp_path, monkeypatch):
    from pinky_test_console.web_server import ConsoleNode
    from pinky_test_console.web_app import create_app, default_web_dir

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    context = Context()
    rclpy.init(context=context, domain_id=TEST_DOMAIN_ID)
    namespace = NS + '/monitor_only'
    node = ConsoleNode(context=context, namespace=namespace, parameter_overrides=[
        rclpy.parameter.Parameter('map_dir', value=str(tmp_path)),
        rclpy.parameter.Parameter('teleop_enabled', value=False)])
    probe = rclpy.create_node('monitor_probe', context=context, namespace=namespace)
    received = []
    probe.create_subscription(Twist, 'cmd_vel', lambda msg: received.append(msg), 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(probe)
    try:
        assert node.cmd_vel_pub is None
        assert node.get_info()['teleop_enabled'] is False
        for call in (lambda: node.teleop(.03, 0.), node.teleop_stop, node.emergency_stop,
                     lambda: node.publish_twist(0., 0.), lambda: node.send_goal(0., 0., 0.),
                     node.cancel_goal, lambda: node.route_command({'cmd': 'start'})):
            with pytest.raises(PermissionError, match='teleop_enabled=false'):
                call()
        client = create_app(node, default_web_dir()).test_client()
        for route, body in [('/api/teleop', {'linear': .03, 'angular': 0}),
                            ('/api/teleop/stop', {}), ('/api/estop', {}),
                            ('/api/nav/stop', {}), ('/api/goal', {'x': 0, 'y': 0})]:
            response = client.post(route, json=body)
            assert response.status_code == 403 and response.json['success'] is False
        node.publish_stop_on_exit()
        until = time.monotonic() + .6
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.02)
        assert probe.count_publishers(namespace + '/cmd_vel') == 0
        assert received == []
        assert node.initial_pose_pub is not None  # Localization setup is preserved.
    finally:
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        probe.destroy_node()
        context.shutdown()
