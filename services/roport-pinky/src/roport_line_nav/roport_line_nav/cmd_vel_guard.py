"""cmd_vel 감시 노드(watchdog). line_nav 와 다른 프로세스로 띄운다.

구독  line_nav/cmd_vel (geometry_msgs/Twist) — line_nav 의 속도 명령
발행  cmd_vel — pinky_bringup 이 받는 토픽. 받은 명령을 그대로 넘긴다(코드 상한으로 자름).

pinky_bringup 은 cmd_vel 타임아웃이 없어 마지막 속도를 계속 유지한다. line_nav 가 죽거나 멈춰서
움직이는 중에 명령이 timeout_sec 넘게 끊기면 여기서 0 을 몇 번 보낸다. line_nav 는 주행 중 IR 샘플마다
(약 20 Hz) 명령을 내므로 0.3 s 공백은 정상 주행에서 생기지 않는다.
이 노드 자체가 죽는 경우는 막지 못한다(근본 해결은 구동 드라이버의 타임아웃).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from roport_pinky_line.follower import HARD_MAX_ANGULAR, HARD_MAX_LINEAR

ZERO_BURST = 3          # 끊김 뒤 0 을 보내는 횟수(한 번 유실 대비)


def _clamp(v, limit):
    if not math.isfinite(v):
        return 0.0
    return max(-limit, min(limit, v))


class GuardLogic:
    """ROS 없는 판정부. relay(t, lin, ang) → 넘길 (lin, ang). tick(t) → 0 을 보낼지."""

    def __init__(self, timeout_sec=0.3):
        if not 0.05 <= timeout_sec <= 2.0:
            raise ValueError('timeout_sec must be in 0.05..2.0')
        self.timeout = timeout_sec
        self._last_t = None
        self._moving = False
        self._zeros = 0
        self.trips = 0           # 끊김으로 0 을 보낸 횟수(로그용)

    def relay(self, t, linear, angular):
        lin, ang = _clamp(linear, HARD_MAX_LINEAR), _clamp(angular, HARD_MAX_ANGULAR)
        self._last_t = t
        self._moving = lin != 0.0 or ang != 0.0
        self._zeros = 0
        return lin, ang

    def tick(self, t):
        if self._zeros:
            self._zeros -= 1
            return True
        if self._moving and self._last_t is not None and t - self._last_t > self.timeout:
            self._moving = False
            self._zeros = ZERO_BURST - 1
            self.trips += 1
            return True
        return False


class CmdVelGuard(Node):
    def __init__(self, **kwargs):
        super().__init__('cmd_vel_guard', **kwargs)
        timeout = float(self.declare_parameter('timeout_sec', 0.3).value)
        self.logic = GuardLogic(timeout)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'line_nav/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f'line_nav/cmd_vel -> cmd_vel, stop after {timeout:.2f} s silence')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, lin, ang):
        msg = Twist()
        msg.linear.x, msg.angular.z = lin, ang
        self.pub.publish(msg)

    def _on_cmd(self, msg):
        self._publish(*self.logic.relay(self._now(), msg.linear.x, msg.angular.z))

    def _tick(self):
        trips = self.logic.trips
        if self.logic.tick(self._now()):
            self._publish(0.0, 0.0)
            if self.logic.trips != trips:
                self.get_logger().warn('line_nav/cmd_vel silent while moving -> zero velocity')

    def stop_on_exit(self):
        if self.logic._moving:
            self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_on_exit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
