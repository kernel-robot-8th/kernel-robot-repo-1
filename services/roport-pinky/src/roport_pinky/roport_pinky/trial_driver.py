"""관제 trial-v1(Pinky 주행 시험) MQTT 모듈 ↔ roport_line_nav 드라이버.

관제 키트(Confluence 19988482, pinky-mqtt-trial-v1-scenarios.zip)의 pinky_mqtt.trial_runner 가
create_driver(profile, role) 로 만들고 observe·validate_action·dispatch·safe_stop·drain_results 를 부른다.
MQTT·스키마·이력·재전송은 키트 모듈이 맡고, 여기서는 line_nav 상태를 관측값으로 바꾸고 명령을 넘긴다.

지원: route.requested(start_maneuver = none, 경유 정차 없음), status. mode_switch·navigate 는 거부.
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
                legs = plan_route(self.graph, p['route'])
            except GraphError:
                raise ValueError('invalid_route') from None
            if p['graph_version'] != self.graph.version:
                raise ValueError('graph_version_mismatch')
            first = HEADING[legs[0].heading] + self.frame_yaw
            if abs(_wrap(first - p['start_heading_rad'])) > 0.35:
                raise ValueError('start_heading_not_first_leg')
            st = self.st
            if st is None or self.now() - self.st_t > STATE_MAX_AGE:
                raise ValueError('line_nav_unavailable')
            if p['speed_limit_mps'] < st['speed']['min_speed']:
                raise ValueError('speed_limit_too_low')
            if st['state'] == 'loading':
                if st['route'][-1] != p['start_node']:
                    raise ValueError('loading_elsewhere')
            elif st['state'] in MOVING:
                raise ValueError('line_nav_busy')

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
        self.send({'cmd': 'start', 'route': list(p['route'])})
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
        return {'seq': seq, 'total': len(route),
                'prev_node': route[seq - 1] if seq else None,
                'next_node': route[seq + 1] if seq + 1 < len(route) else None}

    def _fail(self, code):
        self.results.append((self.active, 'failed', {'error_code': code}))
        self.active = self.pending = None
        self.node = None


class TrialDriver:
    """rclpy 연결부. 별도 스레드에서 spin, 키트의 MQTT 스레드는 hook 만 부른다."""

    def __init__(self, profile, role, *, layout_file=None, start_node=None):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String

        if layout_file is None:
            from ament_index_python.packages import get_package_share_directory
            layout_file = os.path.join(get_package_share_directory('roport_gz_world'),
                                       'layout', 'roport_airport.yaml')
        graph = Graph.load(layout_file)
        self._String, self._Twist = String, Twist
        rclpy.init()
        self.node = rclpy.create_node('pinky_trial_driver')
        self.cmd_pub = self.node.create_publisher(String, 'line_nav/command', 10)
        self.vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.logic = TrialLogic(graph, profile, send=self._send, zero=self._zero,
                                start_node=start_node)
        self.node.create_subscription(String, 'line_nav/state', self._on_state, 10)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        threading.Thread(target=self._ex.spin, daemon=True).start()
        self.node.get_logger().info(
            f'trial driver for {role}: start node {start_node}, graph {graph.version}')

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
        self.vel_pub.publish(self._Twist())

    def observe(self):
        return self.logic.observe()

    def validate_action(self, command):
        return self.logic.validate_action(command)

    def dispatch(self, command):
        return self.logic.dispatch(command)

    def safe_stop(self, reason):
        self.node.get_logger().warn(f'safe_stop: {reason}')
        return self.logic.safe_stop(reason)

    def drain_results(self):
        return self.logic.drain_results()


def create_driver(profile, role):
    """pinky_mqtt.trial_runner --driver roport_pinky.trial_driver:create_driver"""
    return TrialDriver(profile, role,
                       layout_file=os.environ.get('ROPORT_PINKY_LAYOUT') or None,
                       start_node=os.environ.get('ROPORT_PINKY_START_NODE') or None)
