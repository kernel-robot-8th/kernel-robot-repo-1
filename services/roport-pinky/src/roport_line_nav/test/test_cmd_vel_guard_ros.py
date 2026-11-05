"""cmd_vel_guard 를 실제 rclpy 로 띄워 확인(로봇 없음, 별도 ROS_DOMAIN_ID + namespace)."""

import importlib.util
import time

import pytest

_MISSING = [m for m in ('rclpy', 'geometry_msgs') if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(_MISSING), reason=f'missing: {_MISSING}')

TEST_DOMAIN_ID = 195     # line_nav 197, 라인 추종 198, 콘솔 199 와 겹치지 않게
NS = '/roport_cmd_guard_pytest'


def test_guard_relays_then_zeroes_after_silence():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    from roport_line_nav.cmd_vel_guard import CmdVelGuard

    ctx = Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    try:
        guard = CmdVelGuard(context=ctx, namespace=NS)
        probe = rclpy.create_node('probe', context=ctx, namespace=NS)
        out = []
        probe.create_subscription(Twist, 'cmd_vel', lambda m: out.append(m.linear.x), 10)
        pub = probe.create_publisher(Twist, 'line_nav/cmd_vel', 10)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(guard)
        ex.add_node(probe)
        assert guard.pub.topic_name == f'{NS}/cmd_vel'

        def spin(sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                ex.spin_once(timeout_sec=0.01)

        spin(0.3)
        msg = Twist()
        msg.linear.x = 0.03
        for _ in range(10):              # 20 Hz 주행 명령
            pub.publish(msg)
            spin(0.05)
        assert out and all(v == 0.03 for v in out)
        n = len(out)
        spin(0.8)                        # line_nav 가 멈춘 것처럼 끊김
        assert out[n:] and all(v == 0.0 for v in out[n:])
        assert guard.logic.trips == 1
        ex.shutdown()
        guard.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)
