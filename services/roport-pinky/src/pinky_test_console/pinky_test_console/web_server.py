#!/usr/bin/env python3
# Derived from Pinky Pro (https://github.com/pinklab-art/pinky_pro,
#   commit 014a09f289e6988894fbffde2624fdb1e257815b)
#   original file: pinky_navigation/scripts/nav2_web_server.py
# Original work licensed under the Apache License, Version 2.0.
# Modified for pinky_test_console (2026):
#   - teleop on /cmd_vel (10 Hz, 0.3 s deadman, server-side speed limits)
#   - emergency stop (zero velocity + Nav2 cancel), teleop cancels active Nav2 goal
#   - SaveMap to an absolute path under ~/roport_maps with name validation, map list
#   - status panel data (/proc/meminfo, loadavg, CPU temp, battery, console RSS)
#   - map/costmap sent only when changed (version numbers)
#   - zero velocity published on shutdown or spin exception
#   - IR raw value view and timed CSV recording (ir_sensor/range)
#   - ArUco marker recognition check (camera preview, ID checklist)
#   - AMCL map match check (scan vs map score, node initial poses, CSV record)
#   - node-route driving page (line_nav/state·command), fleet settings/test control (laptop)
# See LICENSE in this package for the full license text.

import _thread
import collections
import ipaddress
import json
import logging
import math
import os
import signal
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import Image, LaserScan
from slam_toolbox.srv import Reset, SaveMap
from rcl_interfaces.msg import Log
from std_msgs.msg import Float32, String, UInt16MultiArray
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from . import amcl_check, fleet, ir_record, map_store, marker_detect, route_view
from .smartcart import SmartcartMonitor
from .sysstats import read_system_stats
from .teleop import DEADMAN_TIMEOUT, MAX_ANGULAR, MAX_LINEAR, TeleopGuard
from .web_app import create_app, default_web_dir

TELEOP_PERIOD = 0.1     # s, /cmd_vel 발행 주기 10 Hz
WATCHDOG_PERIOD = 0.02  # s, deadman 검사 주기 (만료 후 최대 20 ms 안에 0 발행)
BATTERY_STALE_SEC = 15.0  # battery_publisher 는 5 s 주기
SAVE_MAP_TIMEOUT = 20.0   # s, map_saver_cli 실행 대기
SCAN_STALE_SEC = 1.0      # 라이다 스캔이 이보다 오래되면 정합 점수를 내지 않음
ROUTE_STALE_SEC = 1.0     # line_nav/state(약 20 Hz)가 이보다 오래되면 끊김으로 표시
ROS_LOG_KEEP = 2000       # /rosout 최근 줄 수(테스트 관제 로그 창용)
LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


def default_layout_yaml():
    """roport_gz_world 의 배치 yaml(노드 그래프). 설치되지 않았으면 ''."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('roport_gz_world'), 'layout',
                            'roport_airport.yaml')
    except Exception:  # noqa: BLE001 - PackageNotFoundError
        return ''


############################################################
# 유틸: Quaternion → yaw
############################################################
def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _call_and_wait(client, request, timeout):
    """executor 가 다른 스레드에서 spin 중일 때 서비스 응답을 기다린다."""
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        return None
    return future.result()


############################################################
# ROS2 노드 (Flask 브리지)
############################################################
class ConsoleNode(Node):
    def __init__(self, **node_kwargs):
        # node_kwargs: 테스트에서 namespace/context 를 바꿔 실제 /cmd_vel 과 분리할 때 사용
        super().__init__('pinky_test_console', **node_kwargs)

        self.declare_parameter('ip', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('mode', 'slam')
        self.declare_parameter('teleop_enabled', True)
        self.declare_parameter('map_dir', map_store.DEFAULT_MAP_DIR)
        self.declare_parameter('ir_dir', ir_record.DEFAULT_IR_DIR)
        self.declare_parameter('camera', 'picamera2')
        self.declare_parameter('rotate_180', False)
        self.declare_parameter('markers_yaml', '')
        self.declare_parameter('camera_hfov_deg', 0.0)
        self.declare_parameter('marker_record_dir', marker_detect.DEFAULT_RECORD_DIR)
        self.declare_parameter('layout_yaml', '')
        self.declare_parameter('amcl_dir', amcl_check.DEFAULT_AMCL_DIR)
        self.declare_parameter('robot', '')
        self.declare_parameter('fleet_file', '~/.roport/fleet.json')
        self.declare_parameter('repo_dir', '~/dev_ws/roport/kernel-robot-repo-1')
        self.declare_parameter('ssh_key', '~/.ssh/pinky_ed25519')
        self.declare_parameter('robot_port', 8080)
        self.declare_parameter('fleet_log_dir', '~/roport_fleet_logs')
        self.declare_parameter('nav_log_dir', '~/roport_logs')

        self.mode = self.get_parameter('mode').value
        self.teleop_enabled = self.get_parameter('teleop_enabled').value
        self.map_dir = map_store.resolve_map_dir(self.get_parameter('map_dir').value)
        self.ir = ir_record.IrRecorder(self.get_parameter('ir_dir').value)
        self.marker = None
        self.marker_error = None
        self.marker_recorder = None
        if self.mode == 'marker':
            self._start_marker()

        self.lock = threading.Lock()

        # 레이어별 최신 메시지와 버전 (버전이 바뀐 경우에만 데이터 전송)
        self.layers = {
            'map': {'msg': None, 'version': 0},
            'global': {'msg': None, 'version': 0},
            'local': {'msg': None, 'version': 0},
        }
        self.smartcart_monitor = SmartcartMonitor()
        self.create_subscription(Odometry, 'odom', self._smartcart_odom,
                                 QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.path_msg = None
        self.tf_pose = None  # (x, y, yaw)
        self.battery = {'percent': None, 'voltage': None}  # (value, monotonic time)

        # ---- map: TRANSIENT_LOCAL QoS (latched) ----
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, 'map', lambda m: self._layer_cb('map', m), latched_qos)
        self.create_subscription(Path, 'plan', self.path_callback, 10)

        # ---- costmap: costmap / costmap_raw 둘 다 시도 (원본과 동일) ----
        for topic in ('local_costmap/costmap', 'local_costmap/costmap_raw'):
            self.create_subscription(Costmap, topic, lambda m: self._layer_cb('local', m), 10)
        for topic in ('global_costmap/costmap', 'global_costmap/costmap_raw'):
            self.create_subscription(Costmap, topic, lambda m: self._layer_cb('global', m), 10)

        # ---- battery (pinky_bringup/battery_publisher.py, std_msgs/Float32, 5 s 주기) ----
        self.create_subscription(
            Float32, 'battery/percent', lambda m: self._battery_cb('percent', m), 10)
        self.create_subscription(
            Float32, 'battery/voltage', lambda m: self._battery_cb('voltage', m), 10)

        # ---- IR (pinky_sensor_adc main_node, 12비트 원시값 3개, 기본 20 Hz) ----
        self.create_subscription(
            UInt16MultiArray, 'ir_sensor/range', lambda m: self.ir.add_sample(m.data), 10)

        # ---- TF2: map -> base_link ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.1, self.update_pose_from_tf)

        # ---- Nav2 ----
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._is_navigating = False
        self.create_subscription(
            GoalStatusArray, 'navigate_to_pose/_action/status',
            self.nav_status_callback, latched_qos)
        self.cancel_client = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        # ---- AMCL 정합 확인 (mode:=amcl 에서만) ----
        self.amcl_frame = None      # 배치 yaml 의 실측 ↔ map 변환
        self.amcl_nodes = []
        self.amcl_error = None
        self.amcl_pose = None       # {x, y, yaw, std, stamp(monotonic)}
        self.scan_points = None     # (points, monotonic time)
        self._field = (None, None)  # (map version, DistanceField)
        self.amcl_recorder = amcl_check.AmclRecorder(self.get_parameter('amcl_dir').value)
        if self.mode == 'amcl':
            self._start_amcl()

        # ---- 노드 경로 주행 (mode:=route: 로봇의 line_nav 노드와 주고받음) ----
        self.route_state = None     # (dict, monotonic time)
        self._route_graph = None
        self.route_cmd_pub = None
        if self.mode in ('route', 'test'):
            self.route_cmd_pub = self.create_publisher(String, 'line_nav/command', 10)
            self.create_subscription(String, 'line_nav/state', self._route_state_cb, 10)

        # ---- ROS 로그(/rosout): 테스트 관제 로그 창에서 Pinky 쪽 로그를 보려고 최근 줄만 보관 ----
        self.ros_logs = collections.deque(maxlen=ROS_LOG_KEEP)
        self._ros_log_seq = 0
        self.create_subscription(Log, '/rosout', self._rosout_cb, 100)

        # ---- SLAM Toolbox ----
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.reset_client = self.create_client(Reset, '/slam_toolbox/reset')

        # ---- Teleop ----
        # pinky_bringup 의 cmd_vel 구독은 depth 10 기본 QoS(RELIABLE).
        self.cmd_vel_pub = (self.create_publisher(Twist, 'cmd_vel', 10)
                            if self.teleop_enabled else None)
        self.teleop_guard = TeleopGuard(MAX_LINEAR, MAX_ANGULAR, DEADMAN_TIMEOUT)
        # 안전 타이머는 STEADY 시계: use_sim_time 이고 시뮬레이터가 멈춰도 deadman 동작
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        if self.teleop_enabled:
            self.create_timer(TELEOP_PERIOD, self._teleop_tick, clock=steady)
            self.create_timer(WATCHDOG_PERIOD, self._watchdog_tick, clock=steady)

        self.get_logger().info(
            f'pinky_test_console started (mode={self.mode}, map_dir={self.map_dir}, '
            f'ir_dir={self.ir.ir_dir}, '
            f'limits: {MAX_LINEAR} m/s, {MAX_ANGULAR} rad/s, deadman {DEADMAN_TIMEOUT} s)')

    # ---------------- 콜백 ----------------
    def nav_status_callback(self, msg):
        """Nav2 액션 상태로 주행 중 여부 갱신 (출처 무관)."""
        active_states = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        navigating = any(s.status in active_states for s in msg.status_list)
        with self.lock:
            self._is_navigating = navigating

    def _layer_cb(self, key, msg):
        with self.lock:
            layer = self.layers[key]
            first = layer['msg'] is None
            layer['msg'] = msg
            layer['version'] += 1
        if first:
            self.get_logger().info(f'Received first {key} grid.')

    def path_callback(self, msg):
        with self.lock:
            self.path_msg = msg

    def _battery_cb(self, key, msg):
        with self.lock:
            self.battery[key] = (float(msg.data), time.monotonic())

    def update_pose_from_tf(self):
        """map -> base_link TF를 읽어서 (x,y,yaw)로 저장."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:  # noqa: BLE001 - 아직 TF가 준비 안 되었을 수 있음
            return
        t = trans.transform
        with self.lock:
            self.tf_pose = (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
            self.smartcart_monitor.observe('tf', trans)

    # ---------------- Teleop ----------------
    def _require_web_drive(self):
        if not self.teleop_enabled:
            raise PermissionError('수동 구동·웹 정지 사용 불가: teleop_enabled=false. 현장 정지 수단을 사용하세요')

    def publish_twist(self, linear, angular):
        self._require_web_drive()
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _teleop_tick(self):
        cmd = self.teleop_guard.next_command()
        if cmd is not None:
            self.publish_twist(*cmd)

    def _watchdog_tick(self):
        if self.teleop_guard.expire():
            self.publish_twist(0.0, 0.0)
            self.get_logger().info('[TELEOP] deadman: no input for '
                                   f'{DEADMAN_TIMEOUT:.1f} s -> zero velocity')

    def teleop(self, linear, angular):
        """웹 조종 입력. Nav2 주행 중이면 먼저 목표를 취소한다."""
        self._require_web_drive()
        nav_cancelled = False
        if self.is_navigating():
            self.get_logger().info('[TELEOP] input during Nav2 goal -> cancel goal first')
            nav_cancelled = self.cancel_goal()
        lin, ang = self.teleop_guard.update(linear, angular)
        return {'linear': lin, 'angular': ang, 'nav_cancelled': nav_cancelled}

    def teleop_stop(self):
        self._require_web_drive()
        if self.teleop_guard.stop():
            self.publish_twist(0.0, 0.0)

    def emergency_stop(self):
        """화면 상단 정지 버튼: 경로 주행 정지·속도 0 먼저, 그 다음 Nav2 취소."""
        self._require_web_drive()
        self.teleop_guard.stop()
        if self.route_cmd_pub is not None:
            self.route_cmd_pub.publish(String(data=json.dumps({'cmd': 'stop'})))
        self.publish_twist(0.0, 0.0)
        nav_cancelled = self.cancel_goal(wait_sec=0.3)
        self.get_logger().warn('[WEB] EMERGENCY STOP: zero velocity published, '
                               f'nav cancel={"sent" if nav_cancelled else "n/a"}')
        return {'nav_cancelled': nav_cancelled}

    def publish_stop_on_exit(self):
        """종료 처리: 속도 0 한 번 발행하고 전달될 때까지 잠깐 기다린다."""
        if not self.teleop_enabled:
            return
        try:
            subs = self.cmd_vel_pub.get_subscription_count()
            self.publish_twist(0.0, 0.0)
            acked = self.cmd_vel_pub.wait_for_all_acked(Duration(seconds=0.5))
            self.get_logger().info(
                f'Published zero velocity on shutdown (subscribers={subs}, acked={acked}).')
        except Exception as e:  # noqa: BLE001 - 종료 중이므로 로그만 남김
            print(f'[pinky_test_console] failed to publish zero velocity on exit: {e}')

    # ---------------- JSON 스냅샷 ----------------
    @staticmethod
    def _grid_json(msg, version, include_data):
        if hasattr(msg, 'info'):  # OccupancyGrid
            meta = msg.info
            w, h = meta.width, meta.height
        else:  # nav2_msgs/Costmap
            meta = msg.metadata
            w, h = meta.size_x, meta.size_y
        if len(msg.data) == 0:
            return None
        out = {
            'version': version,
            'width': w,
            'height': h,
            'resolution': meta.resolution,
            'origin': {
                'x': meta.origin.position.x,
                'y': meta.origin.position.y,
                'yaw': quat_to_yaw(meta.origin.orientation),
            },
        }
        if include_data:
            out['data'] = list(msg.data)
        return out

    def get_state_snapshot(self, client_versions, layers):
        with self.lock:
            grids = {k: (v['msg'], v['version']) for k, v in self.layers.items()}
            path_msg = self.path_msg
            tf_pose = self.tf_pose
            is_navigating = self._is_navigating

        out = {}
        for key, (msg, version) in grids.items():
            if msg is None or key not in layers:
                out[key] = None
                continue
            out[key] = self._grid_json(
                msg, version, include_data=client_versions.get(key) != version)

        out['pose'] = None
        if tf_pose is not None:
            out['pose'] = {'x': tf_pose[0], 'y': tf_pose[1], 'yaw': tf_pose[2]}
        out['path'] = []
        if path_msg is not None:
            out['path'] = [{'x': ps.pose.position.x, 'y': ps.pose.position.y}
                           for ps in path_msg.poses]
        out['navigating'] = is_navigating
        out['teleop'] = self.teleop_guard.snapshot()
        return out

    def get_info(self):
        return {
            'mode': self.mode,
            'teleop_enabled': self.teleop_enabled,
            'map_dir': self.map_dir,
            'ir_dir': self.ir.ir_dir,
            'amcl_dir': self.amcl_recorder.amcl_dir,
            'max_linear': MAX_LINEAR,
            'max_angular': MAX_ANGULAR,
            'deadman_sec': DEADMAN_TIMEOUT,
            'teleop_hz': round(1.0 / TELEOP_PERIOD),
        }

    # ---------------- 노드 경로 주행 ----------------
    def _route_state_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            with self.lock:
                self.route_state = (data, time.monotonic())

    def _rosout_cb(self, msg):
        with self.lock:
            self._ros_log_seq += 1
            self.ros_logs.append({
                'seq': self._ros_log_seq,
                'stamp': round(msg.stamp.sec + msg.stamp.nanosec * 1e-9, 3),
                'level': LOG_LEVELS.get(msg.level, str(msg.level)),
                'node': msg.name, 'msg': msg.msg[:500]})

    def ros_log_snapshot(self, after=0, limit=500):
        with self.lock:
            rows = [r for r in self.ros_logs if r['seq'] > after]
            last = self._ros_log_seq
        return {'entries': rows[-limit:], 'last': last}

    def log_files(self):
        """로봇에 남은 경로 상세 기록(route_*.csv)·ROS 실행 로그 폴더(~/.ros/log) 목록."""
        nav_dir = str(self.get_parameter('nav_log_dir').value)
        return {'route_logs': route_view.list_route_logs(nav_dir),
                'ros_logs': route_view.list_ros_log_dirs('~/.ros/log')}

    def route_snapshot(self):
        with self.lock:
            st = self.route_state
        age = None if st is None else round(time.monotonic() - st[1], 2)
        return {
            'mode': self.mode,
            'robot': str(self.get_parameter('robot').value),
            'available': self.mode in ('route', 'test'),
            'state': None if st is None else st[0],
            'age_sec': age,
            'stale': st is None or age > ROUTE_STALE_SEC,
        }

    def route_graph(self):
        if self._route_graph is None:
            path = os.path.expanduser(str(self.get_parameter('layout_yaml').value)) \
                or default_layout_yaml()
            if not path:
                raise route_view.RouteCommandError(
                    'layout not found (install roport_gz_world or set layout:=<yaml>)')
            self._route_graph = route_view.graph_view(path)
        return self._route_graph

    def route_command(self, cmd):
        """cmd = route_view.parse_route_command 결과. line_nav 가 받아 검사·실행한다."""
        self._require_web_drive()
        if self.route_cmd_pub is None:
            raise route_view.RouteCommandError('route commands need mode:=route or mode:=test')
        self.route_cmd_pub.publish(String(data=json.dumps(cmd)))
        self.get_logger().info(f'[WEB] line_nav command: {cmd}')
        return True

    def make_fleet(self):
        """mode:=fleet 의 FleetManager. 다른 모드는 None."""
        if self.mode != 'fleet':
            return None
        store = fleet.FleetStore(str(self.get_parameter('fleet_file').value))
        cmds = fleet.Commands(str(self.get_parameter('repo_dir').value),
                              str(self.get_parameter('ssh_key').value))
        mgr = fleet.FleetManager(
            store, cmds, client=fleet.RobotClient(int(self.get_parameter('robot_port').value)),
            log=fleet.FleetLog(str(self.get_parameter('fleet_log_dir').value)))
        mgr.start_polling()
        return mgr

    def get_status(self):
        stats = read_system_stats()
        now = time.monotonic()
        with self.lock:
            battery = dict(self.battery)
        for key, val in battery.items():
            if val is None:
                stats['battery_' + key] = None
            else:
                stats['battery_' + key] = {
                    'value': val[0],
                    'age_sec': round(now - val[1], 1),
                    'stale': now - val[1] > BATTERY_STALE_SEC,
                }
        stats['pid'] = os.getpid()
        return stats

    # ---------------- Goal 전송 ----------------
    def send_goal(self, x, y, yaw):
        self._require_web_drive()
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('navigate_to_pose Action Server not available.')
            return False

        # 조종 중이었다면 멈추고 Nav2 에 넘긴다 (두 명령 충돌 방지)
        self.teleop_stop()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'[WEB] send goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self.nav_client.send_goal_async(goal)
        # 즉각적인 UI 반응을 위해 우선 True로 표시 (이후 상태 토픽이 보정)
        with self.lock:
            self._is_navigating = True
        return True

    # ---------------- AMCL 초기 위치 설정 ----------------
    def set_initial_pose(self, x, y, yaw, var_xy=0.25, var_yaw=0.06853891909122467):
        """RViz의 '2D Pose Estimate'와 동일하게 /initialpose 퍼블리시 (공분산 기본값도 RViz)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 공분산 (x, y, yaw)
        msg.pose.covariance[0] = var_xy
        msg.pose.covariance[7] = var_xy
        msg.pose.covariance[35] = var_yaw
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'[WEB] set initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        return True

    # ---------------- Nav2 주행 상태/취소 ----------------
    def is_navigating(self):
        with self.lock:
            return self._is_navigating

    def cancel_goal(self, wait_sec=1.0):
        """Nav2 주행을 취소(정지). goal 출처와 무관하게 모든 goal 취소."""
        self._require_web_drive()
        if not self.cancel_client.wait_for_service(timeout_sec=wait_sec):
            self.get_logger().warn('navigate_to_pose cancel service not available.')
            return False
        # 빈 goal_info(0 UUID + 0 stamp) → 모든 활성 goal 취소
        self.cancel_client.call_async(CancelGoal.Request())
        with self.lock:
            self._is_navigating = False
        self.get_logger().info('[WEB] Requested cancel of all goals.')
        return True

    # ---------------- SLAM Toolbox 제어 ----------------
    def slam_reset(self):
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('/slam_toolbox/reset service not available.')
            return False
        self.reset_client.call_async(Reset.Request())
        self.get_logger().info('[WEB] Requested SLAM reset.')
        return True

    def slam_save_map(self, name, overwrite=False):
        """<map_dir>/<name>.pgm/.yaml 로 저장. 이름 검사는 map_store (쉘 주입 방지)."""
        if not name:
            name = time.strftime('pinky_map_%Y%m%d_%H%M%S')
        base = map_store.map_base_path(self.map_dir, name)  # MapNameError 가능
        if not overwrite and map_store.map_exists(self.map_dir, name):
            return {'success': False, 'exists': True, 'name': name,
                    'msg': f'{name} already exists'}

        if not self.save_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('/slam_toolbox/save_map service not available.')
            return {'success': False, 'name': name,
                    'msg': '/slam_toolbox/save_map not available (mode:=slam ?)'}

        os.makedirs(self.map_dir, exist_ok=True)
        started = time.time()
        req = SaveMap.Request()
        req.name = String(data=base)
        self.get_logger().info(f"[WEB] SaveMap -> '{base}'")
        resp = _call_and_wait(self.save_map_client, req, SAVE_MAP_TIMEOUT)
        if resp is None:
            return {'success': False, 'name': name, 'msg': 'save_map timed out'}

        yaml_path, pgm_path = base + '.yaml', base + '.pgm'
        written = all(os.path.isfile(p) and os.path.getmtime(p) >= started - 1.0
                      for p in (yaml_path, pgm_path))
        ok = resp.result == SaveMap.Response.RESULT_SUCCESS and written
        msg = 'saved' if ok else f'save_map result={resp.result}, files written={written}'
        self.get_logger().info(f'[WEB] SaveMap {name}: {msg}')
        return {'success': ok, 'name': name, 'yaml': yaml_path, 'pgm': pgm_path, 'msg': msg}

    def list_maps(self):
        return {'map_dir': self.map_dir, 'maps': map_store.list_maps(self.map_dir)}

    # ---------------- IR 측정 ----------------
    def ir_snapshot(self):
        return self.ir.snapshot()

    def ir_start(self, label, duration):
        """IrRecordError 가능 (라벨·시간 검사, 기록 중)."""
        state = self.ir.start(label, duration)
        self.get_logger().info(f'[WEB] IR record start: {label} ({duration:g} s)')
        return state

    def ir_cancel(self):
        return self.ir.cancel()

    def ir_records(self):
        records = self.ir.list_records()
        return {'ir_dir': self.ir.ir_dir, 'records': records,
                'suggestion': ir_record.suggest_thresholds(records)}

    def ir_record_path(self, name):
        return self.ir.record_path(name)

    # ---------------- AMCL 정합 확인 ----------------
    def _start_amcl(self):
        path = str(self.get_parameter('layout_yaml').value).strip()
        if path:
            try:
                self.amcl_frame, self.amcl_nodes = amcl_check.load_layout(path)
            except (OSError, amcl_check.AmclCheckError) as e:
                self.amcl_error = f'layout: {e}'
                self.get_logger().error(f'[AMCL] {self.amcl_error}')
        sensor_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_pose_cb, 10)
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')
        self.get_logger().info(f'[AMCL] layout={path or "-"} nodes={len(self.amcl_nodes)} '
                               f'record={self.amcl_recorder.amcl_dir}')

    def _scan_cb(self, msg):
        """스캔을 받은 시점의 map → 라이다 TF로 점을 map 좌표로 바꿔 둔다."""
        with self.lock:
            self.smartcart_monitor.observe('scan', msg)
        try:
            t = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time()).transform
        except Exception:  # noqa: BLE001 - AMCL 이 아직 map TF 를 안 냈을 수 있음
            return
        pose = (t.translation.x, t.translation.y, quat_to_yaw(t.rotation))
        points = amcl_check.scan_to_points(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, pose)
        with self.lock:
            self.scan_points = (points, time.monotonic())

    def _smartcart_odom(self, msg):
        t = msg.twist.twist
        values = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        stopped = all(math.isfinite(v) and abs(v) <= .01 for v in values)
        with self.lock:
            self.smartcart_monitor.observe('odom', msg, stopped=stopped)

    def smartcart_snapshot(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            status = self.smartcart_monitor.snapshot(now)
            amcl = dict(self.amcl_pose) if self.amcl_pose else None
            pose = self.tf_pose
            has_map = self.layers['map']['msg'] is not None
        if amcl:
            amcl['age_sec'] = round(time.monotonic() - amcl.pop('stamp'), 2)
        return {'sensors': status, 'amcl': amcl, 'pose': pose, 'map_loaded': has_map,
                'navigating': self.is_navigating(), 'nodes': self.amcl_nodes,
                'mode': self.mode, 'teleop_enabled': self.teleop_enabled, 'delivery_enabled': False,
                'delivery_reason': '실기 정차 좌표 미측정 · 배송 실행부 미연결'}

    def smartcart_initial_pose(self, node_id, yaw_deg, confirmed):
        with self.lock:
            if self.layers['map']['msg'] is None:
                raise amcl_check.AmclCheckError('지도를 먼저 수신해야 초기 위치를 적용할 수 있습니다')
            self.smartcart_monitor.require_initial_pose(
                self.get_clock().now().nanoseconds / 1e9, node_id, yaw_deg,
                confirmed, self._is_navigating)
        node = self.amcl_node(node_id)
        return self.amcl_initial_pose_cm(node['x_cm'], node['y_cm'], yaw_deg)

    def _amcl_pose_cb(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.smartcart_monitor.observe('amcl', msg)
            self.amcl_pose = {'x': p.position.x, 'y': p.position.y,
                              'yaw': quat_to_yaw(p.orientation),
                              'std': amcl_check.pose_std(msg.pose.covariance),
                              'stamp': time.monotonic()}

    def _distance_field(self, map_msg, version):
        cached_version, field = self._field
        if cached_version != version:
            info = map_msg.info
            field = amcl_check.DistanceField(info.width, info.height, info.resolution,
                                             info.origin.position.x, info.origin.position.y,
                                             map_msg.data)
            self._field = (version, field)
        return field

    def _amcl_score(self):
        with self.lock:
            map_msg, version = self.layers['map']['msg'], self.layers['map']['version']
            scan = self.scan_points
        if map_msg is None or scan is None or time.monotonic() - scan[1] > SCAN_STALE_SEC:
            return None, scan
        return amcl_check.match_score(scan[0], self._distance_field(map_msg, version)), scan

    def amcl_snapshot(self, map_version):
        score, scan = self._amcl_score()
        with self.lock:
            map_msg, version = self.layers['map']['msg'], self.layers['map']['version']
            amcl = dict(self.amcl_pose) if self.amcl_pose else None
        now = time.monotonic()
        out = {
            'map': None if map_msg is None else self._grid_json(
                map_msg, version, include_data=map_version != version),
            'scan': None if scan is None else {
                'points': [[round(x, 3), round(y, 3)] for x, y in scan[0]],
                'classes': score['classes'] if score else [],
                'age_sec': round(now - scan[1], 2)},
            'score': None if score is None else {k: v for k, v in score.items()
                                                 if k != 'classes'},
            'thresholds_cm': [round(t * 100) for t in amcl_check.THRESHOLDS],
            'amcl': None,
            'frame': None,
            'nodes': [],
            'error': self.amcl_error,
            'records': self.amcl_recorder.rows(10),
        }
        if amcl is not None:
            out['amcl'] = {'x': amcl['x'], 'y': amcl['y'], 'yaw': amcl['yaw'],
                           'std': list(amcl['std']), 'age_sec': round(now - amcl['stamp'], 1)}
            if self.amcl_frame is not None:
                x, y, yaw = self.amcl_frame.to_measured(amcl['x'], amcl['y'], amcl['yaw'])
                out['amcl'].update({'x_cm': round(x, 1), 'y_cm': round(y, 1),
                                    'yaw_deg': round(yaw, 1)})
        if self.amcl_frame is not None:
            f = self.amcl_frame
            out['frame'] = {'x_m': f.x_m, 'y_m': f.y_m, 'yaw_deg': f.yaw_deg}
            for n in self.amcl_nodes:
                mx, my, _ = f.to_map(n['x_cm'], n['y_cm'])
                out['nodes'].append(dict(n, map_x=round(mx, 4), map_y=round(my, 4)))
        return out

    def amcl_initial_pose_cm(self, x_cm, y_cm, yaw_deg):
        """실측 좌표(cm, 배치 yaml frame) 로 초기 위치. AmclCheckError: 배치 yaml 없음."""
        if self.amcl_frame is None:
            raise amcl_check.AmclCheckError('layout_yaml not loaded (layout:=<path>)')
        x, y, yaw = self.amcl_frame.to_map(x_cm, y_cm, yaw_deg)
        # 로봇을 노드·잰 위치에 정확히 놓는 시험이라 RViz 기본(σ 0.5 m)보다 좁게 준다.
        # 넓으면 제자리 갱신으로 수렴이 느리다(Gazebo N12: 갱신 15번 뒤에도 12 cm).
        return self.set_initial_pose(x, y, yaw, var_xy=amcl_check.INIT_STD_XY ** 2,
                                     var_yaw=math.radians(amcl_check.INIT_STD_YAW_DEG) ** 2)

    def amcl_node(self, node_id):
        for n in self.amcl_nodes:
            if n['id'] == node_id:
                return n
        raise amcl_check.AmclCheckError(f'unknown node {node_id!r}')

    def amcl_nomotion_update(self):
        """로봇을 움직이지 않고 AMCL 측정 갱신 한 번 (nav2_amcl request_nomotion_update)."""
        if not self.nomotion_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('request_nomotion_update not available (mode:=amcl ?)')
            return False
        self.nomotion_client.call_async(Empty.Request())
        return True

    def amcl_record(self, label, truth):
        score, _scan = self._amcl_score()
        if score is None:
            raise amcl_check.AmclCheckError(f'no scan within {SCAN_STALE_SEC:g} s')
        with self.lock:
            amcl = dict(self.amcl_pose) if self.amcl_pose else None
        frame = self.amcl_frame or amcl_check.Frame()
        row = self.amcl_recorder.record(frame, amcl, score, label, truth)
        self.get_logger().info(f'[AMCL] record {label or "-"}: {row}')
        return row

    def amcl_records_path(self):
        return self.amcl_recorder.path

    # ---------------- 마커 인식 확인 ----------------
    def _start_marker(self):
        """카메라를 열고 인식 스레드를 시작한다. 실패해도 콘솔은 뜨고 화면에 오류를 보인다."""
        try:
            kind, arg = marker_detect.parse_camera_spec(self.get_parameter('camera').value)
            path = os.path.expanduser(str(self.get_parameter('markers_yaml').value))
            dict_name, table = marker_detect.load_marker_table(path)
            if kind == 'picamera2':
                source = marker_detect.PiCameraSource()
            elif kind == 'v4l2':
                source = marker_detect.V4l2Source(arg)
            else:
                source = marker_detect.PushSource()
                self.create_subscription(Image, arg, lambda m: self._image_cb(source, m), 2)
            self.marker_recorder = marker_detect.RangeRecorder(
                self.get_parameter('marker_record_dir').value, table)
            self.marker = marker_detect.MarkerWorker(
                source, marker_detect.MarkerDetector(dict_name), table,
                rotate_180=bool(self.get_parameter('rotate_180').value),
                hfov_deg=float(self.get_parameter('camera_hfov_deg').value),
                recorder=self.marker_recorder)
            self.marker.start()
            self.get_logger().info(f'[MARKER] camera={kind}:{arg} dict={dict_name} '
                                   f'table={len(table)} markers')
        except Exception as e:  # noqa: BLE001 - picamera2 없음·장치 없음·yaml 오류
            self.marker_error = f'{type(e).__name__}: {e}'
            self.get_logger().error(f'[MARKER] camera not started: {self.marker_error}')

    def _image_cb(self, source, msg):
        try:
            source.push(marker_detect.ros_image_to_bgr(
                msg.height, msg.width, msg.encoding, msg.step, msg.data))
        except marker_detect.MarkerError as e:
            self.marker_error = str(e)

    def marker_snapshot(self):
        if self.marker is None:
            return {'error': self.marker_error or 'camera not started (mode:=marker ?)',
                    'fps': None, 'detections': [], 'checklist': []}
        snap = self.marker.snapshot()
        snap['error'] = snap['error'] or self.marker_error
        self.marker_recorder.poll()
        snap['recording'] = self.marker_recorder.state()
        snap['last_record'] = self.marker_recorder.last_result
        return snap

    def marker_jpeg(self):
        return self.marker.jpeg() if self.marker is not None else None

    def marker_reset(self):
        if self.marker is None:
            return False
        self.marker.reset_seen()
        return True

    def marker_record_start(self, distance_cm, duration, marker_id=None, note=''):
        """MarkerError 가능 (카메라 없음, 값 범위, 기록 중)."""
        if self.marker_recorder is None:
            raise marker_detect.MarkerError('camera not started (mode:=marker ?)')
        state = self.marker_recorder.start(distance_cm, duration, marker_id, note)
        self.get_logger().info(f'[MARKER] range record {distance_cm:g} cm, {duration:g} s')
        return state

    def marker_records(self):
        if self.marker_recorder is None:
            return {'path': None, 'records': []}
        return {'path': self.marker_recorder.path, 'records': self.marker_recorder.records()}

    def marker_records_path(self):
        return self.marker_recorder.path if self.marker_recorder is not None else None

    def close_marker(self):
        if self.marker is not None:
            self.marker.stop()


############################################################
# 실행부
############################################################
def _spin(executor, node):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except Exception as e:  # noqa: BLE001 - 콜백 예외: 정지 후 프로세스 종료
        node.get_logger().fatal(f'ROS spin failed: {e!r}; stopping robot and exiting')
        node.publish_stop_on_exit()
        _thread.interrupt_main()


def _raise_keyboard_interrupt(signum, _frame):
    raise KeyboardInterrupt(f'signal {signum}')


def main(args=None):
    # rclpy 기본 SIGINT 처리기는 context 를 먼저 shutdown 해서 종료 시 0 속도 발행이
    # 실패한다. 직접 처리한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ConsoleNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    # SIGINT 도 명시적으로 설치: nohup/스크립트로 실행되면 SIGINT 가 '무시'로 상속될 수 있다
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    # SSH 접속이 끊기면 SIGHUP: 기본 동작은 정리 없이 종료라 0 속도를 못 보낸다
    signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)
    spin_thread = threading.Thread(target=_spin, args=(executor, node), daemon=True)

    try:
        spin_thread.start()

        ip = str(node.get_parameter('ip').value)
        port = int(node.get_parameter('port').value)
        fleet_mgr = node.make_fleet()
        allowed = None
        if fleet_mgr is not None and ipaddress.IPv4Address(ip).is_loopback:
            allowed = {f'127.0.0.1:{port}', f'localhost:{port}'}
        app = create_app(node, default_web_dir(), fleet=fleet_mgr, allowed_hosts=allowed)
        # 10 Hz teleop + 2 Hz 상태 요청 로그가 화면을 덮지 않도록
        logging.getLogger('werkzeug').setLevel(logging.WARNING)
        node.get_logger().info(f'Web console: http://{ip}:{port}  (classroom Wi-Fi only)')
        app.run(host=ip, port=port, debug=False, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        # 터미널 Ctrl+C 는 ros2 launch 와 이 프로세스에 SIGINT 를 각각 보낸다(같은 프로세스 그룹).
        # 정리 도중 두 번째 신호가 KeyboardInterrupt 로 끊지 않도록 무시하고,
        # SIGTERM(launch 의 강제 종료 단계)은 기본 동작(즉시 종료)으로 돌린다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        node.publish_stop_on_exit()
        node.close_marker()
        executor.shutdown(timeout_sec=1.0)
        # spin 스레드가 wait 중인 채로 노드를 없애면 C++ terminate(abort)가 난다
        if spin_thread.is_alive() and spin_thread is not threading.current_thread():
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
