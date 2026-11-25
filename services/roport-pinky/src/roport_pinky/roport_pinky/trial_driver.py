"""관제 trial-v1(Pinky 주행 시험) MQTT 모듈 ↔ roport_line_nav 드라이버.

관제 키트(Confluence 19988482, pinky-mqtt-trial-v1-scenarios.zip)의 pinky_mqtt.trial_runner 가
create_driver(profile, role) 로 만들고 observe·validate_action·dispatch·safe_stop·drain_results 를 부른다.
MQTT·스키마·이력·재전송은 키트 모듈이 맡고, 여기서는 line_nav 상태를 관측값으로 바꾸고 명령을 넘긴다.

지원: route(start_maneuver=none), N12 mode_switch, Nav2 navigate, follow(고객 카드 추종, trial_follow.py), status.
선택 hook: set_boarding_alert(on) = 탑승 시간 알림(smartcart/boarding_alert). 정밀 접근: ROPORT_FINE_ALIGN_GOALS(fine_align.py).
출발 노드: 처음에는 ROPORT_PINKY_START_NODE(예: N16, 운영자가 놓은 칸). 그 뒤로는 line_nav 가
  마지막으로 확인한 노드. 정지·오류 뒤에는 모름(null) → 다음 route 는 관제 쪽에서 거부된다.
방향: 칸 노드(이웃 1개)는 칸 바깥 방향, 주행 뒤에는 line_nav 화면용 추정(pose_est). map = layout + frame.yaw.
도착: N01·N11(상차)은 반시계 90° 뒤 상차 대기(loading)에서 완료를 보낸다(사용자 결정 2026-11-05).
  그 자리에서 다음 route 가 오면 load_done 을 먼저 보내 진입선으로 돌린 뒤 출발한다.
속도: base = min(line_nav 현재 속도, speed_limit_mps). 상한은 line_nav max_linear.
"""

import json
import math
import os
import re
import threading
import time
from datetime import UTC, datetime

from roport_line_nav.graph import Graph, GraphError, plan_route

MOVING = ('follow', 'pass', 'advance', 'rotate', 'reverse')
DONE = ('loading', 'arrived', 'handoff')
HEADING = {'E': 0.0, 'N': math.pi / 2, 'W': math.pi, 'S': -math.pi / 2}
STATE_MAX_AGE = 1.0        # s, line_nav/state 가 이보다 오래되면 관측 불가
ODOM_MAX_AGE = 0.5
STOP_LINEAR = 0.005        # m/s, |v| 이하면 정지
STOP_ANGULAR = 0.02        # rad/s
START_TIMEOUT = 3.0        # s, line_nav 가 start 를 받아들였는지 확인하는 시간
CODE = re.compile(r'[^A-Za-z0-9_.:-]')


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _iso(t):
    return datetime.fromtimestamp(t, UTC).isoformat()


def error_code(reason, default='line_nav_failed'):
    """line_nav reason('line_lost on N02→N10') → 식별 코드('line_lost')."""
    head = (reason or '').split(' ')[0].split(':')[0]
    code = CODE.sub('', head)[:64]
    return code or default


class TrialLogic:
    """ROS 없는 판정부. 입력: line_nav 상태(dict)·odom 속도, 출력: line_nav 명령 dict 와 결과 큐."""

    def __init__(self, graph, profile, *, send, zero, start_node=None, now=time.time):
        self.graph = graph
        self.send, self.zero, self.now = send, zero, now
        self.frame_yaw = float(profile.get('frame_yaw_rad', math.radians(0.10)))
        self.map_id = profile['map_id']
        self.map_version = profile['map_version']
        if start_node is not None and start_node not in graph.nodes:
            raise ValueError(f'unknown start node {start_node}')
        self.node = start_node
        self.heading = self._bay_heading(start_node)
        self.st, self.st_t = None, None
        self.odom, self.odom_t = None, None
        self.active = None        # 관제 명령(dict)
        self.pending = None       # 상차 대기 중 받은 명령: load_done 뒤 출발
        self.sent_at = None
        self.own = False          # 이 드라이버가 line_nav 경로를 시작한 적이 있는지(방향 추정 사용)
        self.seq = 0
        self.seen = 0             # 처리한 line_nav event_seq
        self.seq_at_dispatch = 0
        self.results = []
        self.lock = threading.Lock()
        self.confirmed_return = False

    # ---------------------------------------------------------------- 입력
    def on_state(self, st):
        with self.lock:
            self.st, self.st_t = st, self.now()
            self._track()

    def on_odom(self, linear, angular):
        with self.lock:
            self.odom, self.odom_t = (linear, angular), self.now()

    # ---------------------------------------------------------------- hook
    def observe(self):
        with self.lock:
            return self._observe()

    def validate_action(self, command):
        with self.lock:
            if command['type'] != 'route.requested':
                raise ValueError('unsupported_operation')
            p = command['payload']
            if p['start_maneuver'] != 'none':
                raise ValueError('unsupported_maneuver')
            try:
                legs = plan_route(self.graph, p['route'], allow_junction_stop=True)
            except GraphError:
                raise ValueError('invalid_route') from None
            if p['graph_version'] != self.graph.version:
                raise ValueError('graph_version_mismatch')
            st = self.st
            if st is None or self.now() - self.st_t > STATE_MAX_AGE:
                raise ValueError('line_nav_unavailable')
            if p['speed_limit_mps'] < st['speed']['min_speed']:
                raise ValueError('speed_limit_too_low')
            if st['state'] == 'loading':
                if st['route'][-1] != p['start_node']:
                    raise ValueError('loading_elsewhere')
                # start_heading is the observed loading pose, before load_done turns.
                if abs(_wrap(self._observe()['heading_rad'] - p['start_heading_rad'])) > 0.35:
                    raise ValueError('loading_heading_mismatch')
            elif st['state'] in MOVING:
                raise ValueError('line_nav_busy')
            elif (self.own and st['state'] == 'arrived' and st.get('route')
                  and self.node == p['start_node'] == st['route'][-1]
                  and not self.graph.nodes[p['start_node']].bar
                  and (st.get('last_command') or {}).get('cmd') != 'stop'):
                # 교차점 hold 뒤에는 executor가 기억한 자세에서 필요한 회전을 한다.
                if not self._stopped(self.now()) or abs(_wrap(
                        self._observe()['heading_rad'] - p['start_heading_rad'])) > 0.35:
                    raise ValueError('junction_heading_mismatch')
            elif abs(_wrap(HEADING[legs[0].heading] + self.frame_yaw
                           - p['start_heading_rad'])) > 0.35:
                raise ValueError('start_heading_not_first_leg')

    def dispatch(self, command):
        with self.lock:
            self.active, self.seq = command, 0
            self.seen = self.st['event_seq'] if self.st else 0
            self.seq_at_dispatch = self.seen      # 이 뒤에 시작한 line_nav 경로만 이 명령의 것
            if self.st and self.st['state'] == 'loading':
                self.pending = command
                self.send({'cmd': 'load_done'})
            else:
                self._start(command)

    def safe_stop(self, reason):
        with self.lock:
            self.active = self.pending = None
            self.node = None
        self.send({'cmd': 'stop'})
        self.zero()

    def drain_results(self):
        with self.lock:
            out, self.results = self.results, []
            return out

    # ---------------------------------------------------------------- 내부
    def _bay_heading(self, nid):
        """칸 노드(이웃 1개)에 놓은 로봇은 칸 바깥을 본다. → map 기준 rad 또는 None."""
        if nid is None or len(self.graph.nodes[nid].links) != 1:
            return None
        return HEADING[self.graph.heading(nid, self.graph.nodes[nid].links[0])] + self.frame_yaw

    def _start(self, command):
        p = command['payload']
        base = min(self.st['speed']['base_speed'], p['speed_limit_mps'])
        self.send({'cmd': 'speed', 'base_speed': base})
        self.send({'cmd': 'start', 'route': list(p['route']), **(
            {'start_offset_cm': 0.0, 'confirmed_start': True} if self.confirmed_return else {}),
            **({'hold_at_goal': True} if not self.graph.nodes[p['route'][-1]].bar else {})})
        self.confirmed_return = False
        self.sent_at = self.now()
        self.node = p['route'][0]
        self.own = True

    def _stopped(self, now):
        if self.odom is None or now - self.odom_t > ODOM_MAX_AGE:
            return False
        v, w = self.odom
        return abs(v) <= STOP_LINEAR and abs(w) <= STOP_ANGULAR

    def _observe(self):
        now = self.now()
        st = self.st
        fresh = st is not None and now - self.st_t <= STATE_MAX_AGE
        moving = fresh and st['state'] in MOVING
        heading = self.heading
        if self.own and fresh and st.get('pose_est') and st.get('route'):
            heading = math.radians(st['pose_est']['heading_deg']) + self.frame_yaw
        stamps = [t for t in (self.st_t, self.odom_t) if t is not None]
        return {
            'observed_at': _iso(min(stamps) if stamps else now),
            'node_id': self.node,
            'mode': 'line' if fresh else 'unknown',
            'stopped': self._stopped(now),
            'line_enabled': bool(moving),
            'nav2_ready': False,
            'localization_ready': False,
            'drive_owner': 'line' if moving else 'none',
            'graph_version': st['graph_version'] if fresh else self.graph.version,
            'map_id': self.map_id,
            'map_version': self.map_version,
            'heading_rad': _wrap(heading if heading is not None else 0.0),
            'fault': None if fresh else 'line_nav_unavailable',
        }

    def _track(self):
        """새 line_nav 상태 → 현재 노드 갱신, 진행·완료·실패 결과."""
        st = self.st
        cmd = self.active
        if cmd is None:
            return
        route = cmd['payload']['route']
        if self.pending is not None:
            if st['state'] == 'arrived':          # 상차 뒤 진입선으로 돌기 끝
                self.pending = None
                self._start(cmd)
            elif st['state'] in ('error', 'stopped'):
                self._fail(error_code(st['reason']))
            return
        if self.sent_at is None:
            return
        started = st['route'] == route and st['route_event_seq0'] >= self.seq_at_dispatch
        if not started:
            last = st.get('last_command') or {}
            if last.get('cmd') == 'start' and last.get('ok') is False:
                self._fail('line_nav_rejected')
            elif self.now() - self.sent_at > START_TIMEOUT:
                self._fail('line_nav_no_response')
            return
        for ev in self._new_events(st):
            if ev['event'] != 'node':
                continue
            nid = ev['detail'].split(' ')[0]
            seq = self.seq + 1
            if seq >= len(route) or route[seq] != nid:
                self._fail('route_order_mismatch')
                return
            self.seq, self.node = seq, nid
            if seq < len(route) - 1:
                self.results.append((cmd, 'progress', self._details(route, seq)))
        if st['state'] in ('error', 'stopped'):
            self._fail(error_code(st['reason']))
        elif (st['state'] in DONE and self.seq == len(route) - 1
              and self._stopped(self.now())):
            self.results.append((cmd, 'completed', self._details(route, self.seq)))
            if st.get('pose_est'):
                self.heading = math.radians(st['pose_est']['heading_deg']) + self.frame_yaw
            self.active = None

    def _new_events(self, st):
        """event_seq 기준으로 처리하지 않은 이벤트(이번 경로 것만)."""
        evs = st.get('events') or []
        first = st['event_seq'] - len(evs) + 1
        out = [e for i, e in enumerate(evs, first) if i > self.seen]
        self.seen = st['event_seq']
        return out

    @staticmethod
    def _details(route, seq):
        return {'node_id': route[seq], 'seq': seq, 'total': len(route),
                'prev_node': route[seq - 1] if seq else None,
                'next_node': route[seq + 1] if seq + 1 < len(route) else None}

    def _fail(self, code):
        self.results.append((self.active, 'failed', {'error_code': code}))
        self.active = self.pending = None
        self.node = None


NAV_RETRY_FAULTS = frozenset(('nav2_goal_failed', 'nav2_recovery_limit',
    'nav2_recovery_timeout', 'gate_dock_marker_missing', 'gate_dock_marker_lost',
    'gate_dock_timeout'))
NAV2_COMMANDS = ('navigate.requested', 'follow.requested')
APPEARANCE_REGISTRATION = 'follow_registration.requested'
FOLLOW_BT = 'follow_target.xml'
FOLLOW_PERIOD = 0.1     # s, 추종 판정·goal_update 주기(ROS spin 스레드)
BOARDING_PERIOD = 0.5   # s, 탑승 알림 True 재발행 주기(status_indicator stale_sec 2 s 보다 짧게)

class TrialDriver:
    """rclpy 연결부. 별도 스레드에서 spin, 키트의 MQTT 스레드는 hook 만 부른다."""

    def __init__(self, profile, role, *, layout_file=None, start_node=None, map_path=None):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String

        if layout_file is None:
            from ament_index_python.packages import get_package_share_directory
            layout_file = os.path.join(get_package_share_directory('roport_gz_world'),
                                       'layout', 'roport_airport.yaml')
        graph = Graph.load(layout_file)
        import yaml
        with open(layout_file, encoding='utf-8') as file:
            self.frame = yaml.safe_load(file)['frame']
        self._String = String
        rclpy.init()
        self.node = rclpy.create_node('pinky_trial_driver')
        self.cmd_pub = self.node.create_publisher(String, 'line_nav/command', 10)
        from uuid import uuid4

        from .trial_nav2 import Nav2Bridge
        if not map_path:
            raise ValueError('ROPORT_PINKY_MAP must identify the Nav2 map YAML')
        self.lock = threading.RLock()
        self.mode, self.nav_command = 'line', None
        self.guard, self.guard_at = None, None
        self.halted, self.runner_seen = None, None
        self.failed_navigation = None
        self.session, self.lease_seq = str(uuid4()), 0
        self.lease_pub = self.node.create_publisher(String, 'trial/lease', 1)
        self.nav = Nav2Bridge(self.node, profile, map_path, self.lock)
        self.node.create_subscription(String, 'trial/guard_state', self._on_guard, 10)
        self.logic = TrialLogic(graph, profile, send=self._send, zero=self._zero,
                                start_node=start_node)
        self.node.create_subscription(String, 'line_nav/state', self._on_state, 10)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._init_follow()
        self._init_boarding()
        self.appearance = None
        if role == 'smartcart' and os.environ.get('ROPORT_APPEARANCE_CONFIG'):
            from roport_smartcart.fleet_appearance import FleetAppearance
            self.appearance = FleetAppearance.from_env(self.node, self.lock)
            self.appearance_timer = self.node.create_timer(.05, self._appearance_tick)
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        self.thread = threading.Thread(target=self._ex.spin, daemon=True)
        self.thread.start()
        self.node.get_logger().info(
            f'trial driver for {role}: start node {start_node}, graph {graph.version}')

    def _init_follow(self):
        """고객 카드 추종 입력·출력. 노트북 roport_follow 와 같은 ROS_DOMAIN_ID 가 필요하다."""
        from ament_index_python.packages import get_package_share_directory
        from geometry_msgs.msg import PointStamped, PoseStamped
        from std_msgs.msg import String

        from .trial_follow import FollowInput
        self._PoseStamped = PoseStamped
        self.follow_bt = os.path.join(get_package_share_directory('roport_pinky'),
                                      'behavior_trees', FOLLOW_BT)
        self.follow_input, self.follow, self.follow_results = FollowInput(), None, []
        # 노트북 발행자 QoS = reliable, volatile, depth 5. 여기서도 reliable depth 5.
        self.node.create_subscription(String, 'follow/status', self._on_follow_status, 5)
        self.node.create_subscription(PointStamped, 'follow/target', self._on_follow_target, 5)
        # Nav2 GoalUpdater(BT) 입력: bt_navigator 의 goal_updater_topic 기본값 goal_update.
        self.goal_update_pub = self.node.create_publisher(PoseStamped, 'goal_update', 1)
        self.follow_timer = self.node.create_timer(FOLLOW_PERIOD, self._follow_tick)

    def _init_boarding(self):
        """탑승 시간 알림 출력(표시·소리만, 주행과 무관). 발행자는 이 노드 하나.
        smartcart/boarding_alert std_msgs/Bool, QoS reliable·volatile depth 1 → roport_line_nav status_indicator
        (smartcart:=true, 마지막 수신 2 s 뒤 스스로 끔)."""
        from std_msgs.msg import Bool
        self._Bool = Bool
        self.boarding = False
        # 알림 전용 runner 생존 시각: heartbeat() 맨 앞에서 갱신(guard 고장·halted 와 무관). 구동권용 runner_seen 과 따로 둔다.
        self.boarding_runner_at = None
        self.boarding_pub = self.node.create_publisher(Bool, 'smartcart/boarding_alert', 1)
        self.boarding_timer = self.node.create_timer(BOARDING_PERIOD, self._boarding_tick)

    def set_boarding_alert(self, on):
        """선택 hook: 탑승 시간 알림 켜기·끄기. 진행 중 명령과 상관없이 부를 수 있다. observe() 에는 넣지 않는다.
        켜져 있는 동안 BOARDING_PERIOD 마다 True, 끌 때 False 한 번. safe_stop 은 알림을 끄지 않는다.
        runner 가 heartbeat() 를 부르고 있어야 한다: 1 s 넘게 없으면 켜자마자(또는 켜진 뒤) 스스로 꺼진다."""
        if not isinstance(on, bool):
            raise ValueError('invalid_alert')
        with self.lock:
            if on:
                self.boarding = True
                self._boarding_tick()
            else:
                self._boarding_off()

    def _boarding_off(self):
        self.boarding = False
        self.boarding_pub.publish(self._Bool(data=False))

    def _boarding_tick(self):
        """ROS 스레드 2 Hz(켤 때 한 번 바로). runner 가 heartbeat() 를 1 s 넘게 부르지 않으면 알림을 끈다:
        알림을 켠 쪽이 끊겨 끌 방법이 없다. halted·guard 고장(구동 불가)만으로는 끄지 않는다(safe_stop 과 같음)."""
        with self.lock:
            if not self.boarding:
                return
            seen = getattr(self, 'boarding_runner_at', None)
            if seen is None or time.monotonic() - seen > 1:
                self._boarding_off()
                return
            self.boarding_pub.publish(self._Bool(data=True))

    def _on_follow_status(self, msg):
        from .trial_follow import parse_status
        status = parse_status(msg.data)
        with self.lock:
            self.follow_input.on_status(status, time.monotonic())

    def _on_follow_target(self, msg):
        s = msg.header.stamp
        with self.lock:
            self.follow_input.on_target(s.sec + s.nanosec * 1e-9, msg.header.frame_id,
                                        msg.point.x, msg.point.y)

    def _ros_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _follow_transform(self, stamp, x, y):
        """base_footprint 점(그 stamp 의 자세) → map (x, y). TF 가 없으면 LookupError."""
        from rclpy.time import Time
        from tf2_ros import TransformException

        from .trial_nav2 import yaw
        try:
            tf = self.nav.tf.lookup_transform(
                'map', 'base_footprint', Time(nanoseconds=int(round(stamp * 1e9))))
        except TransformException as exc:
            raise LookupError(str(exc)) from None
        t = tf.transform
        a = yaw(t.rotation)
        return (t.translation.x + math.cos(a) * x - math.sin(a) * y,
                t.translation.y + math.sin(a) * x + math.cos(a) * y)

    def _publish_goal_update(self, goal):
        x, y, heading, stamp = goal
        msg = self._PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = int(math.floor(stamp))
        msg.header.stamp.nanosec = min(int(round((stamp % 1) * 1e9)), 999_999_999)
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        self.goal_update_pub.publish(msg)

    def _follow_tick(self):
        """ROS 스레드 10 Hz: 정지 조건이면 즉시 Nav2 취소·lease none, 아니면 새 목표를 GoalUpdater 로."""
        with self.lock:
            if (self.follow is None or not self.nav_command
                    or self.nav_command['type'] != 'follow.requested'):
                return
            if self.nav.result:
                code = self.nav.result[0]
                self._end_follow(code if code == 'nav2_not_ready' else 'follow_nav2_failed')
                return
            step = self.follow.step(self.follow_input, self._ros_now(), time.monotonic(),
                                    self._follow_transform, self.nav.snapshot().get('pose'))
            if step is None:
                return
            if step[0] == 'stop':
                self._end_follow(step[1])
            else:
                self._publish_goal_update(step[1])

    def _appearance_running(self):
        cmd = self.nav_command
        return bool(cmd and cmd['type'] == 'follow.requested'
                    and cmd['payload'].get('follow_mode') == 'appearance')

    def _appearance_tick(self):
        with self.lock:
            state = self.observe()
            controller_ready = bool(
                state['fault'] is None and state['mode'] == 'nav2'
                and state['nav2_ready'] and state['localization_ready']
                and not state['line_enabled'])
            following = self._appearance_running()
            try:
                error, result = self.appearance.tick(
                    following=following, controller_ready=controller_ready,
                    stopped=state['stopped'] and self.guard is not None
                    and self.guard['owner'] == 'none')
            except Exception as exc:  # ROS action replies and camera decodes may fail asynchronously.
                self.node.get_logger().error(f'Appearance tick failed: {type(exc).__name__}')
                pending = self.appearance.pending
                if following:
                    self._lease('none')
                self.appearance.stop(forget=True)
                error = 'follow_status_stale' if following else None
                result = (pending, 'failed', {'error_code': 'follow_registration_failed'}) if pending else None
            if error:
                self._end_follow(error)
            if result:
                self.follow_results.append(result)

    def _end_follow(self, code):
        cmd = self.nav_command
        if not self._appearance_running() and self.nav.navigation is not None:
            self.nav.navigation.update(phase='failed', error_code=code)
        self.safe_stop(code)      # Nav2 취소 + lease none(guard 가 0 속도) + 코드별 잠금
        self.follow_results.append((cmd, 'failed', {'error_code': code}))

    def _on_state(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        self.logic.on_state(st)

    def _on_odom(self, msg):
        t = msg.twist.twist
        self.logic.on_odom(t.linear.x, t.angular.z)

    def _send(self, body):
        self.cmd_pub.publish(self._String(data=json.dumps(body)))
        self.node.get_logger().info(f'line_nav command {body}')

    def _zero(self):
        self._lease('none')

    def _on_guard(self, msg):
        try:
            state = json.loads(msg.data)
            if not (-0.1 <= time.time() - state['sent_at'] <= 0.3
                    and isinstance(state['boot'], str)
                    and state['owner'] in ('line', 'nav2', 'none')
                    and type(state['blocked']) is bool):
                return
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            if self.guard and self.guard['boot'] != state['boot']:
                self.halted = 'drive_guard_restarted'
            self.guard, self.guard_at = state, time.monotonic()

    def _guard_fault(self, *, navigation_retry=False):
        if self.halted and not (navigation_retry and self.halted in NAV_RETRY_FAULTS):
            return self.halted
        if self.guard is None or time.monotonic() - self.guard_at > 0.3:
            return 'drive_guard_unavailable'
        if self.guard['blocked']:
            return 'drive_lease_expired'
        publishers = self.node.get_publishers_info_by_topic('cmd_vel')
        if len(publishers) != 1 or publishers[0].node_name != 'trial_guard':
            return 'drive_output_conflict'
        return None

    def _lease(self, owner):
        if self.guard is not None:
            self.lease_seq += 1
            self.lease_pub.publish(self._String(data=json.dumps({
                'boot': self.guard['boot'], 'session': self.session, 'seq': self.lease_seq,
                'sent_at': time.time(), 'owner': owner,
            })))

    def heartbeat(self):
        with self.lock:
            self.boarding_runner_at = time.monotonic()      # 탑승 알림 전용(구동권 판단 전)
            if self._guard_fault(navigation_retry=self.failed_navigation is not None):
                return
            self.runner_seen = time.monotonic()
            if self.halted:
                self._lease('none')
                return
            owner = 'line' if self.logic.active else 'none'
            if self._appearance_running():
                owner = 'nav2' if self.appearance.motion_requested else 'none'
            elif self.nav_command and self.nav_command['type'] in NAV2_COMMANDS and not self.nav.result:
                owner = 'nav2'
            self._lease(owner)

    def observe(self):
        with self.lock:
            state = self.logic.observe()
            nav = self.nav.snapshot()
            state.update(mode=self.mode, nav2_ready=nav['nav2_ready'],
                         localization_ready=nav['localization_ready'])
            if self.mode == 'nav2':
                state['drive_owner'] = 'nav2'
                state['stopped'] = state['stopped'] and nav['stopped']
                if nav['pose']:
                    state['heading_rad'] = _wrap(nav['pose']['yaw'])
                if not nav['nav2_ready'] or not nav['localization_ready']:
                    state['fault'] = 'nav2_not_ready'
            appearance = getattr(self, 'appearance', None)
            if appearance is not None and not self._appearance_running():
                state['stopped'] = state['stopped'] and appearance.quiescent()
            # Existing trial consumers reject unknown fields; enable only after both sides update.
            if os.environ.get('ROPORT_NAV_DIAGNOSTICS') == '1':
                diagnostic = self.nav.diagnostic()
                if diagnostic is not None:
                    state['navigation'] = diagnostic
            state['fault'] = self._guard_fault() or state['fault']
            if self.runner_seen is None or time.monotonic() - self.runner_seen > 1:
                state['fault'] = state['fault'] or 'runner_heartbeat_missing'
            follow = self._follow_observation(state)
            if follow is not None:
                state['follow'] = follow
            return state

    def _follow_observation(self, state):
        """observe().follow(선택 필드): 추종 명령이 살아 있는 동안만 넣는다.

        active 는 접수가 아니라 실제 구동 근거다. Nav2 가 추종 goal 을 수락했고(handle), 방금(0.3 s 이내)
        받은 trial_guard 응답의 owner 가 nav2 이며 막혀 있지 않고, 이 관측에 fault 가 없을 때만 true.
        구동권 전환 대기 중·goal 수락 전·종료 뒤에는 false. 고객이 서 있어 속도가 0 이어도 true 일 수 있다.
        """
        cmd = self.nav_command
        if not cmd or cmd['type'] != 'follow.requested':
            return None
        guard = self.guard
        guard_ready = bool(
            guard is not None and time.monotonic() - self.guard_at <= 0.3
            and guard['owner'] == 'nav2' and not guard['blocked']
            and state['fault'] is None and state['mode'] == 'nav2'
            and state['nav2_ready'] and state['localization_ready'] and not state['line_enabled'])
        if self._appearance_running():
            return {
                'active': guard_ready and self.appearance.motion_requested
                and self.appearance.live(),
                'follow_mode': 'appearance',
                **{key: cmd['payload'][key] for key in (
                    'follow_session_id', 'target_ref', 'perception_boot_id')},
            }
        active = bool(guard_ready and self.follow is not None
                      and self.nav.handle is not None and self.nav.result is None)
        follow = {'active': active, 'follow_mode': 'marker'}
        session = cmd['payload'].get('follow_session_id')
        if session is not None:
            follow['follow_session_id'] = session
        return follow

    def observe_motion(self):
        """표시용 관측만 제공한다. 기존 상태/완료/구동권 판단에는 쓰지 않는다."""
        with self.lock, self.logic.lock:
            nav = self.nav.snapshot()
            pose, source, observed = None, 'unavailable', self.logic.now()
            if nav.get('localization_ready') and nav.get('pose') and nav.get('observed_at'):
                pose, source = dict(nav['pose']), getattr(self, 'pose_source', 'tf')
                observed = datetime.fromisoformat(
                    nav.get('pose_observed_at') or nav['observed_at']).timestamp()
            elif (self.mode == 'line' and self.logic.own and self.logic.node is not None
                  and self.logic.st is not None
                  and self.logic.st['state'] not in ('stopped', 'error')):
                observed = self.logic.st_t
                estimate = self.logic.st.get('pose_est')
                if estimate and self.logic.now() - observed <= STATE_MAX_AGE:
                    x, y = estimate['x_cm'] / 100, estimate['y_cm'] / 100
                    angle = self.logic.frame_yaw
                    pose = {
                        'x': self.frame['x_m'] + math.cos(angle) * x - math.sin(angle) * y,
                        'y': self.frame['y_m'] + math.sin(angle) * x + math.cos(angle) * y,
                        'yaw': _wrap(math.radians(estimate['heading_deg']) + angle),
                    }
                    source = 'line_estimate'
            if pose is not None:
                pose['z'] = 0.0
            return {
                'observed_at': _iso(observed), 'map_id': self.logic.map_id,
                'map_version': self.logic.map_version, 'frame_id': 'map',
                'pose_source': source, 'pose': pose, 'velocity': None, 'joints': {},
            }

    def validate_navigation_retry(self, command):
        """Read-only admission for an explicit new command to the failed goal."""
        with self.lock:
            previous = self.failed_navigation
            if (previous is None or self.halted not in NAV_RETRY_FAULTS
                    or command['type'] != 'navigate.requested'
                    or command['payload'] != previous['payload']
                    or any(command.get(k) == previous.get(k) for k in ('command_id', 'execution_id'))
                    or self.mode != 'nav2' or self.nav_command or self.logic.active
                    or self._guard_fault(navigation_retry=True) or not self.nav.quiescent()):
                return False
            line, nav = self.logic.observe(), self.nav.snapshot()
            return bool(line['fault'] is None and line['stopped'] and not line['line_enabled']
                        and nav['stopped'] and nav['nav2_ready'] and nav['localization_ready'])

    def validate_action(self, command):
        with self.lock:
            if self.validate_navigation_retry(command):
                return
            state = self.observe()
            if state['fault']:
                raise ValueError(state['fault'])
            if self.nav_command or self.logic.active or not state['stopped'] or state['line_enabled']:
                raise ValueError('drive_busy')
            appearance = getattr(self, 'appearance', None)
            if appearance is not None and appearance.pending:
                raise ValueError('drive_busy')
            if command['type'] == 'route.requested':
                if self.mode != 'line':
                    raise ValueError('not_in_line_mode')
                return self.logic.validate_action(command)
            p = command['payload']
            if any(p.get(k) != state[k] for k in ('map_id', 'map_version')):
                raise ValueError('map_mismatch')
            if not state['nav2_ready'] or not state['localization_ready']:
                raise ValueError('nav2_not_ready')
            if command['type'] == 'mode_switch.requested':
                if p['to_mode'] == 'line':
                    if self.mode != 'nav2' or not self.nav.quiescent() or not self._return_aligned():
                        raise ValueError('N12_physical_alignment_required')
                elif self.mode != 'line' or state['node_id'] != 'N12' or self.logic.st['state'] != 'handoff':
                    raise ValueError('not_at_switch_node')
            elif command['type'] == APPEARANCE_REGISTRATION:
                if appearance is None:
                    raise ValueError('unsupported_operation')
                if self.mode != 'nav2' or not self.nav.quiescent() or self.guard['owner'] != 'none':
                    raise ValueError('registration_not_ready')
                appearance.validate_registration(p)
            elif command['type'] == 'follow.requested':
                self._admit_follow(p)
            elif command['type'] != 'navigate.requested' or self.mode != 'nav2' or p['frame_id'] != 'map':
                raise ValueError('unsupported_operation')

    def _admit_follow(self, payload):
        """읽기 전용 출발 검사. 첫 목표 (x, y, yaw, stamp) 를 돌려준다."""
        from .trial_follow import admit
        if self.mode != 'nav2':
            raise ValueError('not_in_nav2_mode')
        if not self.nav.quiescent():
            raise ValueError('drive_busy')
        if payload.get('follow_mode') == 'appearance':
            appearance = getattr(self, 'appearance', None)
            if appearance is None:
                raise ValueError('unsupported_operation')
            return appearance.validate_start(payload)
        if payload.get('follow_mode', 'marker') != 'marker':
            raise ValueError('unsupported_operation')
        if not os.path.isfile(self.follow_bt):
            raise ValueError('follow_bt_missing')
        return admit(payload, self.follow_input, self._ros_now(), time.monotonic(),
                     self._follow_transform, self.nav.snapshot().get('pose'))

    def _start_follow(self, command):
        from .trial_follow import FollowSession
        self.nav_command = command
        self.logic.node = None
        try:
            first = self._admit_follow(command['payload'])
        except ValueError as exc:      # 수락 뒤 입력이 끊김: 움직이지 않고 실패 보고
            self.nav_command = None
            self.follow_results.append((command, 'failed', {'error_code': str(exc)}))
            return
        self.follow = FollowSession(command['payload'], time.monotonic())
        self.follow.published, self.follow.published_at = first[3], time.monotonic()
        x, y, heading, stamp = first
        self.nav.start({'frame_id': 'map', 'goal_id': 'follow',
                        'pose': {'x': x, 'y': y, 'yaw': _wrap(heading)}},
                       behavior_tree=self.follow_bt, stamp=stamp,
                       # 추종은 몇 분씩 이어지고 Nav2 는 비우기·대기를 따로 누적해 센다(복구 1번 = 2) →
                       # 누적 상한 대신 BT 의 연속 재시도(3번)와 RECOVERY_TIMEOUT_S 로 막는다(11-20 실로봇).
                       max_recoveries=None)
        self._lease('nav2')

    def dispatch(self, command):
        with self.lock:
            if self.halted:
                if not self.validate_navigation_retry(command):
                    raise ValueError('recovery_required')
                self.halted = None
                self.failed_navigation = None
            appearance = getattr(self, 'appearance', None)
            if appearance is not None and command['type'] in (
                    'route.requested', 'navigate.requested', 'mode_switch.requested'):
                # Release the previous customer's target before a new route can own the drive.
                appearance.stop(forget=True)
            if command['type'] == 'route.requested':
                self._lease('line')
                return self.logic.dispatch(command)
            if command['type'] == APPEARANCE_REGISTRATION:
                return self.appearance.register(command)
            if command['type'] == 'follow.requested':
                if command['payload'].get('follow_mode') == 'appearance':
                    try:
                        self.appearance.start(command['payload'])
                    except ValueError as exc:
                        self.follow_results.append((command, 'failed', {'error_code': str(exc)}))
                        return
                    self.logic.node = None
                    self.nav_command = command
                    return
                return self._start_follow(command)
            self.nav_command = command
            if command['type'] == 'mode_switch.requested':
                self._lease('none')
                if command['payload']['to_mode'] == 'nav2':
                    self.mode = 'nav2'
                else:
                    self.return_since = self.return_sample = None
            else:
                self.logic.node = None
                self.nav.start(command['payload'])
                self._lease('nav2')

    def safe_stop(self, reason):
        from .trial_follow import SOFT_STOPS
        with self.lock:
            following = bool(self.nav_command and self.nav_command['type'] == 'follow.requested')
            self.follow = None
            appearance = getattr(self, 'appearance', None)
            if appearance is not None:
                appearance.stop(forget=reason not in SOFT_STOPS and reason != 'operator_stop')
            if (reason in NAV_RETRY_FAULTS and self.nav_command
                    and self.nav_command['type'] == 'navigate.requested'):
                self.failed_navigation = self.nav_command
            elif reason not in NAV_RETRY_FAULTS:
                self.failed_navigation = None
            # 고객 놓침·노트북 끊김·시간 종료·추종 중 운영자 정지는 정상 종료: 새 follow 를 다시 받는다.
            if not (reason in SOFT_STOPS or reason == 'follow_registration_failed'
                    or (following and reason == 'operator_stop')):
                self.halted = reason
            self.nav_command = None
            self.nav.cancel()
            self.logic.safe_stop(reason)

    def drain_results(self):
        with self.lock:
            follow, self.follow_results = getattr(self, 'follow_results', []), []
            return follow + self._drain_results()

    def _drain_results(self):
        with self.lock:
            active = self.nav_command or self.logic.active
            state = self.observe()
            if active and state['fault']:
                if self.nav.navigation:
                    self.nav.navigation.update(phase='failed', error_code=state['fault'])
                self.safe_stop(state['fault'])
                return [(active, 'failed', {'error_code': state['fault']})]
            results = self.logic.drain_results()
            cmd = self.nav_command
            if cmd is None or cmd['type'] == 'follow.requested':
                return results  # 추종 종료는 _follow_tick 이 follow_results 로 넣는다
            if cmd['type'] == 'mode_switch.requested':
                if cmd['payload']['to_mode'] == 'line':
                    if not self._return_aligned() or self.guard['owner'] != 'none' or not state['stopped']:
                        self.return_since = None
                        return results
                    sample = self.logic.st['ir_observed_at']
                    if self.return_sample != sample:
                        self.return_sample = sample
                        now = time.monotonic()
                        if self.return_since is None:
                            self.return_since = now
                        if now - self.return_since >= .5:
                            self.mode = 'line'
                            self.logic.node, self.logic.heading = 'N12', _wrap(math.pi + self.logic.frame_yaw)
                            self.logic.own = False
                            self.logic.confirmed_return = True
                            self.nav_command = None
                            results.append((cmd, 'completed', {}))
                    return results
                if self.guard['owner'] == 'none' and state['stopped']:
                    self.nav_command = None
                    results.append((cmd, 'completed', {}))
                return results
            if self.nav.result:
                code, ended = self.nav.result
                snap = self.nav.snapshot()
                p, pose = cmd['payload'], snap['pose']
                if code == 'SUCCEEDED' and not state['stopped'] and time.monotonic() - ended < 3:
                    return results
                if code == 'SUCCEEDED' and (not pose or not state['stopped']
                            or math.hypot(pose['x'] - p['pose']['x'], pose['y'] - p['pose']['y']) > p['position_tolerance_m']
                            or abs(_wrap(pose['yaw'] - p['pose']['yaw'])) > p['yaw_tolerance_rad']):
                    if time.monotonic() - ended < 3:
                        return results  # allow the final odom/TF sample to arrive
                    code = 'nav2_goal_not_reached'
                self._lease('none')
                if code != 'SUCCEEDED':
                    if self.nav.navigation is not None:
                        self.nav.navigation.update(phase='failed', error_code=code)
                    self.safe_stop(code)
                    self.nav_command = None
                    results.append((cmd, 'failed', {'error_code': code}))
                else:
                    self.nav_command = None
                    if self.nav.navigation is not None:
                        self.nav.navigation.update(phase='completed', error_code=None)
                    results.append((cmd, 'completed', {
                        'goal_id': p['goal_id'], 'goal_version': p['goal_version'],
                        'frame_id': 'map', 'pose': pose, 'nav2_result': 'SUCCEEDED',
                    }))
            return results

    def _return_aligned(self):
        """Use measured pose AND fresh calibrated IR; a map coordinate alone is insufficient."""
        snap = self.nav.snapshot()
        pose, st = snap.get('pose'), self.logic.st or {}
        ir, observed = st.get('ir'), st.get('ir_observed_at')
        if not pose or not ir or observed is None or not 0 <= time.time() - observed <= .3:
            return False
        n = self.logic.graph.nodes['N12']
        a = self.logic.frame_yaw
        x, y = (n.x + 3.) / 100, n.y / 100
        target = (self.frame['x_m'] + math.cos(a)*x - math.sin(a)*y,
                  self.frame['y_m'] + math.sin(a)*x + math.cos(a)*y)
        return (snap['localization_ready'] and snap['stopped']
                and math.hypot(pose['x']-target[0], pose['y']-target[1]) <= .05
                and abs(_wrap(pose['yaw'] - math.pi - a)) <= .1
                and not ir['lost'] and ir['on_line'][1]
                and math.isfinite(ir['error']) and abs(ir['error']) <= .25)

    def close(self):
        import rclpy
        if getattr(self, 'boarding_pub', None) is not None:
            with self.lock:
                if self.boarding:
                    self.boarding = False
                    try:
                        self.boarding_pub.publish(self._Bool(data=False))
                    except Exception:  # noqa: BLE001 — context 가 이미 끝났어도 아래 정리는 해야 한다
                        pass
        self._ex.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        if getattr(self, 'appearance', None) is not None:
            self.appearance.close()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def create_driver(profile, role):
    """pinky_mqtt.trial_runner --driver roport_pinky.trial_driver:create_driver"""
    return TrialDriver(profile, role,
                       layout_file=os.environ.get('ROPORT_PINKY_LAYOUT') or None,
                       start_node=os.environ.get('ROPORT_PINKY_START_NODE') or None,
                       map_path=os.environ.get('ROPORT_PINKY_MAP') or None)
