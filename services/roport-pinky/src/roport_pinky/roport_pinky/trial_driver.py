"""관제 trial-v1(Pinky 주행 시험) MQTT 모듈 ↔ roport_line_nav 드라이버.

관제 키트(Confluence 19988482, pinky-mqtt-trial-v1-scenarios.zip)의 pinky_mqtt.trial_runner 가
create_driver(profile, role) 로 만들고 observe·validate_action·dispatch·safe_stop·drain_results 를 부른다.
MQTT·스키마·이력·재전송은 키트 모듈이 맡고, 여기서는 line_nav 상태를 관측값으로 바꾸고 명령을 넘긴다.

지원: route(start_maneuver=none), N12 mode_switch, Nav2 navigate, status.
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
        self.session, self.lease_seq = str(uuid4()), 0
        self.lease_pub = self.node.create_publisher(String, 'trial/lease', 1)
        self.nav = Nav2Bridge(self.node, profile, map_path, self.lock)
        self.node.create_subscription(String, 'trial/guard_state', self._on_guard, 10)
        self.logic = TrialLogic(graph, profile, send=self._send, zero=self._zero,
                                start_node=start_node)
        self.node.create_subscription(String, 'line_nav/state', self._on_state, 10)
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        self.thread = threading.Thread(target=self._ex.spin, daemon=True)
        self.thread.start()
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

    def _guard_fault(self):
        if self.halted:
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
            if self._guard_fault():
                return
            self.runner_seen = time.monotonic()
            owner = 'line' if self.logic.active else 'none'
            if self.nav_command and self.nav_command['type'] == 'navigate.requested' and not self.nav.result:
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
                    state['heading_rad'] = nav['pose']['yaw']
                if not nav['nav2_ready'] or not nav['localization_ready']:
                    state['fault'] = 'nav2_not_ready'
            # Existing trial consumers reject unknown fields; enable only after both sides update.
            if os.environ.get('ROPORT_NAV_DIAGNOSTICS') == '1':
                diagnostic = self.nav.diagnostic()
                if diagnostic is not None:
                    state['navigation'] = diagnostic
            state['fault'] = self._guard_fault() or state['fault']
            if self.runner_seen is None or time.monotonic() - self.runner_seen > 1:
                state['fault'] = state['fault'] or 'runner_heartbeat_missing'
            return state

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

    def validate_action(self, command):
        with self.lock:
            state = self.observe()
            if state['fault']:
                raise ValueError(state['fault'])
            if self.nav_command or self.logic.active or not state['stopped'] or state['line_enabled']:
                raise ValueError('drive_busy')
            if command['type'] == 'route.requested':
                if self.mode != 'line':
                    raise ValueError('not_in_line_mode')
                return self.logic.validate_action(command)
            p = command['payload']
            if any(p[k] != state[k] for k in ('map_id', 'map_version')):
                raise ValueError('map_mismatch')
            if not state['nav2_ready'] or not state['localization_ready']:
                raise ValueError('nav2_not_ready')
            if command['type'] == 'mode_switch.requested':
                if self.mode != 'line' or state['node_id'] != 'N12' or self.logic.st['state'] != 'handoff':
                    raise ValueError('not_at_switch_node')
            elif command['type'] != 'navigate.requested' or self.mode != 'nav2' or p['frame_id'] != 'map':
                raise ValueError('unsupported_operation')

    def dispatch(self, command):
        with self.lock:
            if command['type'] == 'route.requested':
                self._lease('line')
                return self.logic.dispatch(command)
            self.nav_command = command
            if command['type'] == 'mode_switch.requested':
                self._lease('none')
                self.mode = 'nav2'
            else:
                self.logic.node = None
                self.nav.start(command['payload'])
                self._lease('nav2')

    def safe_stop(self, reason):
        with self.lock:
            self.halted = reason
            self.nav_command = None
            self.nav.cancel()
            self.logic.safe_stop(reason)

    def drain_results(self):
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
            if cmd is None:
                return results
            if cmd['type'] == 'mode_switch.requested':
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
                self.nav_command = None
                self._lease('none')
                if code != 'SUCCEEDED':
                    self.nav.navigation.update(phase='failed', error_code=code)
                    self.safe_stop(code)
                    results.append((cmd, 'failed', {'error_code': code}))
                else:
                    self.nav.navigation.update(phase='completed', error_code=None)
                    results.append((cmd, 'completed', {
                        'goal_id': p['goal_id'], 'goal_version': p['goal_version'],
                        'frame_id': 'map', 'pose': pose, 'nav2_result': 'SUCCEEDED',
                    }))
            return results

    def close(self):
        import rclpy
        self._ex.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def create_driver(profile, role):
    """pinky_mqtt.trial_runner --driver roport_pinky.trial_driver:create_driver"""
    return TrialDriver(profile, role,
                       layout_file=os.environ.get('ROPORT_PINKY_LAYOUT') or None,
                       start_node=os.environ.get('ROPORT_PINKY_START_NODE') or None,
                       map_path=os.environ.get('ROPORT_PINKY_MAP') or None)
