"""노드 경로 주행 노드(FFT1-117). 물류 Pinky·스마트카트 공용.

구독  ir_sensor/range (std_msgs/UInt16MultiArray, pinky_sensor_adc 약 20 Hz)
      odom (nav_msgs/Odometry, pinky_bringup 엔코더) — 이동 거리·회전 각도
      line_nav/command (std_msgs/String, JSON)
        {"cmd": "start", "route": ["N16", "N14", ...], "start_offset_cm": 0}  (start_offset 생략 가능)
        {"cmd": "start", "route": ["N07", "N18", "N01"], "start_cross": true, "start_facing": "S"}
                               (IR 3개가 N07 을 지나는 교차선 위, 남쪽을 봄)
        {"cmd": "stop"}        {"cmd": "load_done"}   (상차 완료: N01·N11 에서 대기 중일 때)
        {"cmd": "dock", "node": "N11", "advance_cm": 1.5, "rotate_deg": 96, "move_cm": 2}
        {"cmd": "dock", "node": "N11", "reset": true}   (정밀 도킹 값 적용·저장, 주행 중 거부)
        {"cmd": "speed", "base_speed": 0.03, "rotate_speed": 0.4, "reverse_speed": 0.02}  (키 하나 이상)
        {"cmd": "pause"}  {"cmd": "resume"}  {"cmd": "back", "to": "N23"}   (FFT1-172 일시정지·라인 후진)
발행  cmd_vel (geometry_msgs/Twist) — 주행 중일 때만. dry_run 이면 발행하지 않음
      line_nav/state (std_msgs/String, JSON) — 상태·현재 구간·이벤트·IR·마지막 명령 결과

line_follower(roport_pinky_line)와 동시에 실행하지 않는다(둘 다 cmd_vel 을 발행).
정지: stop 명령, 오류(노드 놓침·라인 놓침 지속·회전 중 선 못 찾음·후진 초과), IR 끊김(ir_timeout_sec),
odom 끊김(odom_timeout_sec), SIGINT/SIGTERM/SIGHUP 종료 때 0 속도. kill -9·전원 차단에는 보낼 수 없고
pinky_bringup 에는 cmd_vel 타임아웃이 없어 마지막 속도가 남는다.
도킹 값 파일: 파라미터 dock_file(기본 ~/roport_dock.yaml, 빈 값 = 파일 안 씀). 시작 때 읽고(없거나 깨지면 경고 후
기본 0·90·0), dock 명령 때 임시 파일 + rename 으로 저장한 뒤 적용한다(저장 실패면 적용하지 않음).
"""

import json
import math
import os
import re
import signal
import time

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String, UInt16MultiArray

from roport_line_nav.dock import load_file as load_dock_file, save_file as save_dock_file
from roport_line_nav.executor import FACINGS, NavConfig, RouteExecutor
from roport_line_nav.graph import Graph, GraphError
from roport_line_nav.recorder import RouteRecorder
from roport_pinky_line.calibration import load_calibration
from roport_pinky_line.line_sensor import LineSensor

NODE_ID = re.compile(r'^[A-Z][A-Z0-9]{0,7}$')
MAX_ROUTE = 64
MAX_COMMAND_BYTES = 4096


def default_layout_file():
    try:
        share = get_package_share_directory('roport_gz_world')
    except PackageNotFoundError:
        return ''
    return os.path.join(share, 'layout', 'roport_airport.yaml')


def default_calibration_file():
    return os.path.join(get_package_share_directory('roport_pinky_line'), 'config', 'robots.yaml')


def parse_command(text):
    """line_nav/command JSON → (cmd, args). 잘못되면 ValueError."""
    if len(text.encode('utf-8')) > MAX_COMMAND_BYTES:
        raise ValueError('command too long')
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f'invalid JSON: {e.msg}') from None
    if not isinstance(data, dict) or not isinstance(data.get('cmd'), str):
        raise ValueError('command must be an object with "cmd"')
    cmd = data['cmd']
    if cmd in ('stop', 'load_done', 'pause', 'resume'):
        return cmd, {}
    if cmd == 'back':           # 지나온 노드인지·받을 수 있는 상태인지는 실행부가 검사
        to = data.get('to')
        if not isinstance(to, str) or not NODE_ID.match(to):
            raise ValueError('back needs "to" (node id)')
        return cmd, {'to': to}
    if cmd == 'dock':           # 범위·상차 노드 여부는 실행부(DockTable)가 검사
        node = data.get('node')
        if not isinstance(node, str) or not NODE_ID.match(node):
            raise ValueError('dock needs "node" (node id)')
        reset = data.get('reset', False)
        if not isinstance(reset, bool):
            raise ValueError('reset must be true or false')
        if reset:
            return cmd, {'node': node, 'reset': True}
        args = {'node': node, 'reset': False}
        for k in ('advance_cm', 'rotate_deg', 'move_cm'):
            v = data.get(k)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f'dock needs {k} (number)')
            args[k] = float(v)
        return cmd, args
    if cmd == 'speed':          # 범위는 실행부가 설정 상한으로 검사
        args = {}
        for k in ('base_speed', 'rotate_speed', 'reverse_speed'):
            v = data.get(k)
            if v is None:
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f'{k} must be a number')
            args[k] = float(v)
        if not args:
            raise ValueError('speed needs base_speed, rotate_speed and/or reverse_speed')
        return cmd, args
    if cmd != 'start':
        raise ValueError(f'unknown cmd {cmd!r}')
    route = data.get('route')
    if not isinstance(route, list) or not 2 <= len(route) <= MAX_ROUTE:
        raise ValueError(f'route must be a list of 2..{MAX_ROUTE} node ids')
    if not all(isinstance(r, str) and NODE_ID.match(r) for r in route):
        raise ValueError('route has an invalid node id')
    offset = data.get('start_offset_cm')
    if offset is not None:
        if isinstance(offset, bool) or not isinstance(offset, (int, float)) \
                or not math.isfinite(offset) or not 0.0 <= offset <= 50.0:
            raise ValueError('start_offset_cm must be a number in 0..50')
        offset = float(offset)
    cross = data.get('start_cross', False)
    if not isinstance(cross, bool):
        raise ValueError('start_cross must be true or false')
    facing = data.get('start_facing')
    if facing is not None and facing not in FACINGS:
        raise ValueError(f'start_facing must be one of {"/".join(FACINGS)}')
    if cross and facing is None:
        raise ValueError('start_cross needs start_facing')
    if facing is not None and not cross:
        raise ValueError('start_facing is only for start_cross')
    if cross and offset is not None:
        raise ValueError('start_cross cannot be combined with start_offset_cm')
    return cmd, {'route': route, 'start_offset_cm': offset,
                 'start_cross': cross, 'start_facing': facing}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LineNavNode(Node):
    def __init__(self, **kwargs):
        super().__init__('line_nav', **kwargs)
        p = self.declare_parameter
        robot = str(p('robot', '').value)
        if not robot:
            raise ValueError('parameter "robot" is required (e.g. robot:=pinky_d660)')
        cal_file = str(p('calibration_file', '').value) or default_calibration_file()
        layout_file = str(p('layout_file', '').value) or default_layout_file()
        if not layout_file:
            raise ValueError(
                'roport_gz_world not installed; set layout_file:=<roport_airport.yaml>')
        self.dry_run = bool(p('dry_run', False).value)
        self.ir_timeout_sec = float(p('ir_timeout_sec', 0.3).value)
        self.odom_timeout_sec = float(p('odom_timeout_sec', 0.5).value)
        # 경로마다 상세 기록 CSV(약 20 Hz), 최근 log_keep 개 (2026-11-04 결정)
        self.recorder = RouteRecorder(str(p('log_dir', '~/roport_logs').value),
                                      int(p('log_keep', 50).value))
        d = NavConfig()
        cfg = NavConfig(**{
            name: type(getattr(d, name))(p(name, getattr(d, name)).value)
            for name in ('base_speed', 'max_linear', 'max_angular', 'kp', 'kd', 'ir_ahead_cm',
                         'turn_advance_cm', 'rotate_speed', 'reverse_speed', 'reverse_max_cm',
                         'gate_min', 'gate_max', 'gate_early_cm', 'lost_timeout',
                         'approach_speed', 'approach_lead_cm', 'post_turn_slow_cm',
                         'pass_full_speed', 'tee_both',
                         'tee_window_cm', 'kp_speed_scale', 'gain_ref_speed',
                         'gap_max_cm', 'gap_heading_window_cm', 'gap_confirm', 'search_deg',
                         'ir_spacing_cm', 'side_rise_only', 'side_rise_off_samples',
                         'side_rise_tee', 'reverse_follow_speed', 'reverse_wn', 'reverse_zeta',
                         'reverse_lost_cm', 'reverse_max_heading_deg', 'reverse_short_cm')})

        self.graph = Graph.load(layout_file)
        self.sensor = LineSensor(load_calibration(cal_file, robot))
        dock_file = str(p('dock_file', '~/roport_dock.yaml').value)
        self.dock_file = os.path.expanduser(dock_file) if dock_file else ''
        if self.dock_file:
            dock, warning = load_dock_file(self.dock_file, self.graph)
            if warning:
                self.get_logger().warn(warning)
        else:
            dock = None
        self.ex = RouteExecutor(self.graph, cfg, dock)
        self._last_ir = None
        self._reading = None
        self._odom = None
        self._odom_t = None
        self._last_cmd = None
        self._last_command = None
        self._logged = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'line_nav/state', 10)
        self.create_subscription(UInt16MultiArray, 'ir_sensor/range', self._on_ir, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(String, 'line_nav/command', self._on_command, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f'robot={robot} layout={layout_file} graph={self.graph.version} '
            f'dry_run={self.dry_run} dock_file={self.dock_file or "(none)"} '
            f'dock={self.ex.dock.as_dict() or "defaults"} '
            f'base={cfg.base_speed} m/s approach={cfg.approach_speed} m/s '
            f'(from {cfg.approach_lead_cm} cm before node, straight pass {"full" if cfg.pass_full_speed else "slow"}) ir_ahead={cfg.ir_ahead_cm} cm '
            f'kp={cfg.kp} kd={cfg.kd} (x speed/{cfg.gain_ref_speed} above it: {cfg.kp_speed_scale}) '
            f'gap_max={cfg.gap_max_cm} cm search={cfg.search_deg} deg '
            f'side_rise={cfg.side_rise_only}(off {cfg.side_rise_off_samples}, tee {cfg.side_rise_tee}) '
            f'reverse_follow={cfg.reverse_follow_speed} m/s (wn {cfg.reverse_wn}, zeta {cfg.reverse_zeta})')

    # ---------------- 입력 ----------------
    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        self._odom = (pos.x * 100.0, pos.y * 100.0, yaw_of(msg.pose.pose.orientation))
        self._odom_t = time.monotonic()

    def _pose(self, now):
        if self._odom_t is not None and now - self._odom_t <= self.odom_timeout_sec:
            return self._odom
        return None

    def _on_command(self, msg):
        cmd = '?'
        try:
            cmd, args = parse_command(msg.data)
            if cmd == 'stop':
                self.ex.stop()
                ok, detail = True, 'stopped'
            elif cmd == 'load_done':
                ok = self.ex.load_done()
                detail = 'resumed' if ok else f'not loading ({self.ex.state})'
            elif cmd == 'speed':
                base, rot = self.ex.set_speed(**args)
                ok, detail = True, f'{base:.3f} m/s, {rot:.2f} rad/s'
                if 'reverse_speed' in args:
                    detail += f', reverse {self.ex.cfg.reverse_follow_speed:.3f} m/s'
            elif cmd == 'pause':
                ok, detail = True, self.ex.pause()
            elif cmd == 'resume':
                ok, detail = True, self.ex.resume()
            elif cmd == 'back':
                if self._reading is None:
                    raise ValueError('no IR yet')
                ok, detail = True, self.ex.back(args['to'])
            elif cmd == 'dock':
                ok, detail = True, self._apply_dock(args)
            else:
                if self._reading is None:
                    raise ValueError('no IR yet')
                self.sensor.reset()
                self.ex.start(args['route'], args['start_offset_cm'],
                              start_cross=args['start_cross'], start_facing=args['start_facing'])
                ok, detail = True, 'started'
        except (ValueError, GraphError, OSError) as e:
            ok, detail = False, str(e)
        self._last_command = {'cmd': cmd, 'ok': ok, 'detail': detail}
        self.get_logger().info(f'command {cmd}: {"ok" if ok else "rejected"} ({detail})')
        self._flush()

    def _apply_dock(self, args):
        """검사 → (dock_file 이면) 원자적 저장 → 적용. 주행 중·범위 밖·저장 실패면 예외(그대로 유지)."""
        if self.ex.active:
            raise ValueError(f'dock rejected while driving ({self.ex.state}); stop first')
        table = self.ex.dock.copy()
        node = args['node']
        if args['reset']:
            table.reset(node)
            v = table.get(node)
        else:
            v = table.set(node, args['advance_cm'], args['rotate_deg'], args['move_cm'])
        if self.dock_file:
            try:
                save_dock_file(self.dock_file, table)
            except OSError as e:
                raise OSError(f'dock not applied, save failed: {e.strerror or e}') from None
        self.ex.replace_dock(table, f'{node} ' + ('reset' if args['reset'] else
                                                  f'a={v.advance_cm:g} r={v.rotate_deg:g} b={v.move_cm:g}'))
        saved = 'saved' if self.dock_file else 'not saved (no dock_file)'
        return (f'{node} advance={v.advance_cm:g} cm rotate={v.rotate_deg:g} deg '
                f'move={v.move_cm:g} cm, {saved}')

    def _on_ir(self, msg):
        try:
            reading = self.sensor.update(msg.data)
        except ValueError as e:
            self.get_logger().warn(f'bad IR message: {e}', throttle_duration_sec=2.0)
            return
        now = time.monotonic()
        self._last_ir = now
        self._reading = reading
        pose = self._pose(now)
        cmd = self.ex.step(reading, pose, now, self._odom_t)
        if cmd is not None:
            self._publish_cmd(cmd.linear, cmd.angular)
        self._record(now, reading, cmd, pose)
        self._publish_state()

    def _record(self, now, reading, cmd, pose):
        if self.ex.active and not self.recorder.active:
            path = self.recorder.open(self.ex.route_shown, now)
            if path:
                self.get_logger().info(f'route log: {path}')
            elif self.recorder.error:
                self.get_logger().warn(self.recorder.error, once=True)
        if self.recorder.active:
            self.recorder.write(now, self.ex.as_dict(), reading,
                                None if cmd is None else (cmd.linear, cmd.angular), pose)
            if not self.ex.active:
                self.recorder.close()

    def _watchdog(self):
        if self._last_ir is not None and time.monotonic() - self._last_ir <= self.ir_timeout_sec:
            return
        if not self.ex.active:
            return
        self.ex.fail('no_ir')
        self.sensor.reset()
        self._reading = None
        self._flush()

    def _flush(self):
        """명령·감시로 상태가 바뀌었을 때 IR 샘플을 기다리지 않고 0 속도·상태를 낸다."""
        now = time.monotonic()
        cmd = self.ex.step(self._reading, self._pose(now), now, self._odom_t) \
            if self._reading is not None or not self.ex.active else None
        if cmd is not None:
            self._publish_cmd(cmd.linear, cmd.angular)
        self._record(now, self._reading, cmd, self._pose(now))
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
        st = self.ex.as_dict()
        if (st['state'], st['index']) != self._logged:
            self.get_logger().info(f'state {st["state"]} leg={st["leg"]} {st["reason"]}')
            self._logged = (st['state'], st['index'])
        st.update({
            'graph_version': self.graph.version, 'dry_run': self.dry_run,
            'cmd': None if self._last_cmd is None else {
                'linear': round(self._last_cmd[0], 3), 'angular': round(self._last_cmd[1], 3)},
            'ir': None if self._reading is None else self._reading.as_dict(),
            'odom_ok': self._odom_t is not None
            and time.monotonic() - self._odom_t <= self.odom_timeout_sec,
            'last_command': self._last_command,
        })
        self.state_pub.publish(String(data=json.dumps(st, ensure_ascii=False)))

    def publish_stop_on_exit(self):
        was_active = self.ex.active
        self.ex.stop('shutdown')
        if self.recorder.active:
            self.recorder.write(time.monotonic(), self.ex.as_dict(), self._reading, (0.0, 0.0),
                                self._odom)
            self.recorder.close()
        if self.dry_run or not was_active:
            return
        try:
            self._publish_cmd(0.0, 0.0)
            acked = self.cmd_pub.wait_for_all_acked(Duration(seconds=0.5))
            self.get_logger().info(f'Published zero velocity on shutdown (acked={acked}).')
        except Exception as e:  # noqa: BLE001 - 종료 중이므로 로그만 남김
            print(f'[line_nav] failed to publish zero velocity on exit: {e}')


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)
    node = None
    try:
        node = LineNavNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        if node is not None:
            node.publish_stop_on_exit()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
