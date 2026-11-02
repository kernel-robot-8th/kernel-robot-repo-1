"""IR 라인 추종 노드 v0 (FFT1-115).

구독  ir_sensor/range (std_msgs/UInt16MultiArray, pinky_sensor_adc 약 20 Hz)
      line_follower/enable (std_msgs/Bool) — true 로 출발, false 로 정지
발행  cmd_vel (geometry_msgs/Twist) — 켜져 있을 때만. dry_run 이면 발행하지 않음
      line_follower/state (std_msgs/String, JSON) — 상태·오차·속도·IR 값

정지 경로: 끄기, 라인 놓침, 노드(세 채널 모두 라인), IR 끊김(ir_timeout_sec),
SIGINT/SIGTERM/SIGHUP 종료 때 0 속도를 발행한다. kill -9·전원 차단에는 보낼 수 없다.
pinky_bringup 에는 cmd_vel 타임아웃이 없으므로 이 노드를 강제 종료하면 마지막 속도가 남는다.
"""

import json
import os
import signal
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String, UInt16MultiArray

from roport_pinky_line.calibration import load_calibration
from roport_pinky_line.follower import FollowerConfig, LineFollower
from roport_pinky_line.line_sensor import LineSensor


def default_calibration_file():
    return os.path.join(get_package_share_directory('roport_pinky_line'), 'config', 'robots.yaml')


class LineFollowerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('line_follower', **kwargs)
        p = self.declare_parameter
        robot = str(p('robot', '').value)
        cal_file = str(p('calibration_file', '').value) or default_calibration_file()
        self.dry_run = bool(p('dry_run', False).value)
        self.ir_timeout_sec = float(p('ir_timeout_sec', 0.3).value)
        defaults = FollowerConfig()
        cfg = FollowerConfig(
            base_speed=float(p('base_speed', defaults.base_speed).value),
            min_speed=float(p('min_speed', defaults.min_speed).value),
            max_linear=float(p('max_linear', defaults.max_linear).value),
            max_angular=float(p('max_angular', defaults.max_angular).value),
            kp=float(p('kp', defaults.kp).value),
            kd=float(p('kd', defaults.kd).value),
            slow_gain=float(p('slow_gain', defaults.slow_gain).value),
            stop_at_node=bool(p('stop_at_node', defaults.stop_at_node).value),
            node_confirm=int(p('node_confirm', defaults.node_confirm).value),
        )
        start_enabled = bool(p('start_enabled', False).value)
        if not robot:
            raise ValueError('parameter "robot" is required (e.g. robot:=pinky_d660)')

        self.sensor = LineSensor(load_calibration(cal_file, robot))
        self.follower = LineFollower(cfg)
        self._last_ir = None
        self._last_reading = None
        self._last_cmd = None
        self._logged_state = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'line_follower/state', 10)
        self.create_subscription(UInt16MultiArray, 'ir_sensor/range', self._on_ir, 10)
        self.create_subscription(Bool, 'line_follower/enable', self._on_enable, 10)
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'robot={robot} calibration={cal_file} dry_run={self.dry_run} '
            f'base={cfg.base_speed} m/s max={cfg.max_linear} m/s, {cfg.max_angular} rad/s '
            f'kp={cfg.kp} kd={cfg.kd} stop_at_node={cfg.stop_at_node}')
        if start_enabled:
            self._set_enabled(True)
        else:
            self.get_logger().info(
                'Waiting: ros2 topic pub --once line_follower/enable '
                'std_msgs/msg/Bool "{data: true}"')

    # ---------------- 입력 ----------------
    def _on_enable(self, msg):
        self._set_enabled(msg.data)

    def _set_enabled(self, on):
        was = self.follower.enabled
        self.follower.set_enabled(on)
        self.get_logger().info(f'line follower {"enabled" if on else "disabled"}')
        if was and not on:
            self._publish_cmd(0.0, 0.0)  # 끌 때 한 번만 0. 그 뒤로는 cmd_vel 을 비워 둔다
        self._publish_state()

    def _on_ir(self, msg):
        now = time.monotonic()
        dt = 0.0 if self._last_ir is None else now - self._last_ir
        self._last_ir = now
        try:
            reading = self.sensor.update(msg.data)
        except ValueError as e:
            self.get_logger().warn(f'bad IR message: {e}', throttle_duration_sec=2.0)
            return
        self._last_reading = reading
        cmd = self.follower.step(reading, dt)
        if cmd is not None:
            self._publish_cmd(cmd.linear, cmd.angular)
        self._publish_state()

    def _watchdog(self):
        if self._last_ir is not None and time.monotonic() - self._last_ir <= self.ir_timeout_sec:
            return
        cmd = self.follower.ir_timeout()
        if cmd is None:
            return
        self.sensor.reset()  # 끊기기 전 값으로 다시 판정하지 않음
        self._last_reading = None
        self._publish_cmd(0.0, 0.0)
        self._publish_state()

    # ---------------- 출력 ----------------
    def _publish_cmd(self, linear, angular):
        self._last_cmd = (linear, angular)
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_state(self):
        state = self.follower.state
        if state != self._logged_state:
            self.get_logger().info(f'state: {self._logged_state} -> {state}')
            self._logged_state = state
        out = {
            'state': state,
            'enabled': self.follower.enabled,
            'dry_run': self.dry_run,
            'cmd': None if self._last_cmd is None else {
                'linear': round(self._last_cmd[0], 3), 'angular': round(self._last_cmd[1], 3)},
        }
        if self._last_reading is not None:
            out.update(self._last_reading.as_dict())
        self.state_pub.publish(String(data=json.dumps(out)))

    def publish_stop_on_exit(self):
        """종료 처리: 켜져 있었으면 0 속도를 발행하고 전달될 때까지 잠깐 기다린다."""
        if self.dry_run or not self.follower.enabled:
            return  # 꺼져 있으면 다른 주행 소스를 가리지 않도록 발행하지 않음
        try:
            self._publish_cmd(0.0, 0.0)
            acked = self.cmd_pub.wait_for_all_acked(Duration(seconds=0.5))
            self.get_logger().info(f'Published zero velocity on shutdown (acked={acked}).')
        except Exception as e:  # noqa: BLE001 - 종료 중이므로 로그만 남김
            print(f'[line_follower] failed to publish zero velocity on exit: {e}')


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # rclpy 기본 신호 처리기는 context 를 먼저 shutdown 해서 종료 시 0 속도 발행이 실패한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)  # SSH 끊김
    node = None
    try:
        node = LineFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 정리 도중 두 번째 신호(launch 가 보내는 SIGINT 등)로 끊기지 않게 한다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        if node is not None:
            node.publish_stop_on_exit()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
