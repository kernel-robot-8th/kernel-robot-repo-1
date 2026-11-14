"""Nav2 action adapter: actual map/localization/pose evidence, never a timer success."""
import hashlib
import json
import math
import os
import time
from datetime import UTC, datetime
from pathlib import Path

import yaml


def map_reference(path, profile):
    from PIL import Image
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    img = path.parent / cfg['image']
    version = hashlib.sha256(path.read_bytes() + img.read_bytes()).hexdigest()[:16]
    if path.stem != profile['map_id'] or version != profile['map_version']:
        raise ValueError('local_map_version_mismatch')
    if cfg.get('mode', 'trinary') != 'trinary':
        raise ValueError('trial requires trinary map')
    with Image.open(img) as source:
        image = source.convert('RGB')
        width, height = image.size
        pixels = list(image.getdata())
    cells = []
    for row in range(height - 1, -1, -1):
        for rgb in pixels[row * width:(row + 1) * width]:
            probability = sum(rgb) / 765
            if not cfg['negate']:
                probability = 1 - probability
            cells.append(100 if probability > cfg['occupied_thresh'] else
                         0 if probability < cfg['free_thresh'] else -1)
    return cfg, width, height, cells


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


MAX_RECOVERIES = 2
RECOVERY_TIMEOUT_S = 30.0


class Nav2Bridge:
    def __init__(self, node, profile, map_path, lock):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose, FollowPath
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        self.node, self.lock = node, lock
        self.reference = map_reference(map_path, profile)
        self.action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node, spin_thread=False)
        self.docking = None
        self.dock_action = None
        self.dock_phase = None
        if os.environ.get('ROPORT_GATE_DOCK_CONFIG'):
            from .gate_docking import GateDocking
            self.docking = GateDocking(node, self.tf, os.environ['ROPORT_GATE_DOCK_CONFIG'])
            self.dock_action = ActionClient(node, FollowPath, 'follow_path')
        self.map_ok = False
        self.amcl = None
        self.scan = self.odom = None
        self.localization = None
        self._String = String
        self.status_pub = node.create_publisher(String, 'trial/localization_state', 1)
        self.states, self.pending_states = {}, {}
        self.state_clients = {name: node.create_client(GetState, name + '/get_state')
                              for name in ('amcl', 'bt_navigator')}
        self.goal = self.handle = self.result = None
        self.goal_ack = None
        self.pending_goal_acks = 0
        self.cancelled_goals = []
        self.generation = 0
        self.navigation = None
        self.recovery_started = None
        self.recovery_pose = None
        self.subs = [node.create_subscription(
            OccupancyGrid, 'map', self._map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)),
            node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)),
            node.create_subscription(LaserScan, 'scan', self._scan, qos_profile_sensor_data),
            node.create_subscription(Odometry, 'odom', self._odom, qos_profile_sensor_data)]
        self.timer = node.create_timer(0.5, self._states)
        self.pose_timer = node.create_timer(0.1, self._publish_localization)

    def _map(self, msg):
        cfg, width, height, cells = self.reference
        info = msg.info
        with self.lock:
            self.map_ok = (msg.header.frame_id == 'map'
                           and (info.width, info.height) == (width, height)
                           and abs(info.resolution - cfg['resolution']) < 1e-7
                           and abs(info.origin.position.x - cfg['origin'][0]) < 1e-6
                           and abs(info.origin.position.y - cfg['origin'][1]) < 1e-6
                           and abs(yaw(info.origin.orientation) - cfg['origin'][2]) < 1e-6
                           and list(msg.data) == cells)

    def _amcl(self, msg):
        with self.lock:
            self.amcl = msg

    def _scan(self, msg):
        with self.lock:
            self.scan = msg

    def _odom(self, msg):
        with self.lock:
            self.odom = msg

    def _states(self):
        from lifecycle_msgs.srv import GetState
        with self.lock:
            for name, client in self.state_clients.items():
                pending = self.pending_states.get(name)
                if pending and not pending[0].done():
                    if time.monotonic() - pending[1] <= 2:
                        continue
                    pending[0].cancel()
                if not client.service_is_ready():
                    self.states.pop(name, None)
                    continue
                future = client.call_async(GetState.Request())
                self.pending_states[name] = (future, time.monotonic())
                future.add_done_callback(lambda f, n=name: self._state(n, f))

    def _state(self, name, future):
        with self.lock:
            if not future.cancelled() and future.exception() is None:
                self.states[name] = (future.result().current_state.id, time.monotonic())

    def _localization_state(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        now = self.node.get_clock().now().nanoseconds * 1e-9
        def stamp(message):
            s = message.header.stamp
            return s.sec + s.nanosec * 1e-9
        def fresh(message, age, future=0.1):
            return message is not None and -future <= now - stamp(message) <= age

        ready = self.action.server_is_ready() and self.map_ok and all(
            name in self.states and self.states[name][0] == 3
            and time.monotonic() - self.states[name][1] < 2
            for name in self.state_clients)
        pose = None
        sources = {name: stamp(msg) if msg is not None else None
                   for name, msg in (('amcl_pose', self.amcl), ('scan', self.scan), ('odom', self.odom))}
        sources.update(map_to_base=None, map_to_odom=None, odom_to_base=None)
        stopped = odom_ok = False
        if fresh(self.odom, 0.5):
            v = self.odom.twist.twist
            odom_ok = all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.angular.z))
            stopped = odom_ok and all(abs(x) <= limit for x, limit in
                          ((v.linear.x, .005), (v.linear.y, .005), (v.angular.z, .02)))
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time())
            correction = self.tf.lookup_transform('map', 'odom', Time())
            odom_tf = self.tf.lookup_transform('odom', 'base_footprint', Time())
            sources.update(map_to_base=stamp(tf), map_to_odom=stamp(correction),
                           odom_to_base=stamp(odom_tf))
            amcl = self.amcl
            covariance = amcl.pose.covariance if amcl else []
            # AMCL pose is conditional; its map->odom TF is refreshed on every scan.
            # Default AMCL postdates that TF by transform_tolerance=1 s.
            if (ready and amcl and amcl.header.frame_id == 'map' and stamp(amcl) <= now + .1
                    and fresh(self.scan, 1.0) and odom_ok
                    and fresh(tf, .5) and fresh(odom_tf, .5) and fresh(correction, .5, 1.1)
                    and all(math.isfinite(v) and 0 <= v <= limit
                            for v, limit in zip((covariance[0], covariance[7], covariance[35]),
                                                (0.04, 0.04, 0.1)))):
                t = tf.transform
                candidate = {'x': t.translation.x, 'y': t.translation.y, 'yaw': yaw(t.rotation)}
                if all(math.isfinite(v) for v in candidate.values()):
                    pose = candidate
        except TransformException:
            pass
        observed = datetime.now(UTC)
        # Preserve TF evidence age across ROS/wall clocks; cached snapshots must not freshen it.
        pose_observed = None
        if pose:
            age = max(0., now - min(sources['map_to_base'], sources['odom_to_base']))
            pose_observed = datetime.fromtimestamp(observed.timestamp() - age, UTC).isoformat()
        return {'nav2_ready': bool(ready), 'localization_ready': pose is not None,
                'pose': pose, 'stopped': stopped, 'frame_id': 'map',
                'observed_at': observed.isoformat(), 'pose_observed_at': pose_observed,
                'source_stamps': sources}

    def _publish_localization(self):
        with self.lock:
            state = self._localization_state()
            self.localization = (state, time.monotonic())
            self._check_recovery(state)
            self._check_docking(state)
            report = dict(state)
            if self.navigation is not None:
                report['navigation'] = self.diagnostic()
            self.status_pub.publish(self._String(data=json.dumps(report, allow_nan=False)))

    def snapshot(self):
        with self.lock:
            if self.localization and time.monotonic() - self.localization[1] <= .3:
                return self.localization[0].copy()
            return {'nav2_ready': False, 'localization_ready': False,
                    'pose': None, 'stopped': False}

    def start(self, payload):
        from nav2_msgs.action import NavigateToPose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = payload['frame_id']
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        p = payload['pose']
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(p['x']), float(p['y'])
        goal.pose.pose.orientation.z = math.sin(p['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(p['yaw'] / 2)
        self.generation += 1
        generation = self.generation
        self.goal, self.handle, self.result = payload, None, None
        self.recovery_started = self.recovery_pose = None
        self.navigation = {
            'phase': 'navigating', 'goal_id': payload['goal_id'],
            'target_pose': dict(p), 'current_pose': None,
            'recoveries': 0, 'max_recoveries': MAX_RECOVERIES,
            'recovery_elapsed_s': 0.0, 'recovery_timeout_s': RECOVERY_TIMEOUT_S,
            'nav2_error_code': 0, 'action_status': 0, 'error_code': None,
        }
        if self.docking is not None:
            self.docking.reset(payload)
        self.dock_phase = None
        if self.docking is not None and self.docking.enabled(payload['goal_id']):
            # Control still owns the final target. Nav2 first stops 20 cm behind
            # it, then a measured marker may authorize the final approach.
            self.dock_phase = 'approach'
            goal.pose.pose.position.x -= .20 * math.cos(p['yaw'])
            goal.pose.pose.position.y -= .20 * math.sin(p['yaw'])
        self._send_goal(goal, generation)

    def _send_goal(self, goal, generation, *, client=None):
        action = self.action if client is None else client
        feedback = (lambda msg: self._feedback(msg, generation)) if client is None else None
        future = action.send_goal_async(goal, feedback_callback=feedback)
        self.goal_ack = future
        self.pending_goal_acks += 1
        future.add_done_callback(lambda f: self._accepted(f, generation))

    def _accepted(self, future, generation):
        with self.lock:
            try:
                handle = future.result()
                if generation != self.generation or self.result is not None:
                    if handle.accepted:
                        self._cancel_handle(handle)
                    return
                if not handle.accepted:
                    self._fail('nav2_goal_rejected')
                    return
                self.handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, generation))
            except (RuntimeError, ValueError, AttributeError):
                if generation == self.generation:
                    self._fail('nav2_action_error')
            finally:
                self.pending_goal_acks = getattr(self, 'pending_goal_acks', 1) - 1

    def _result(self, future, generation):
        with self.lock:
            if generation != self.generation or self.result is not None:
                return
            try:
                result = future.result()
                success = result.status == 4 and getattr(result.result, 'error_code', 0) == 0
                code = 'SUCCEEDED' if success else 'nav2_goal_failed'
                self.navigation['nav2_error_code'] = int(getattr(result.result, 'error_code', 0))
                self.navigation['action_status'] = int(result.status)
            except (RuntimeError, ValueError, AttributeError):
                code = 'nav2_action_error'
            if code == 'SUCCEEDED' and getattr(self, 'dock_phase', None) in ('approach', 'executing'):
                self.dock_phase = 'await_stop' if self.dock_phase == 'approach' else 'verify'
                self.dock_waiting_since = time.monotonic()
                self.handle = None  # action has terminated; no overlapping velocity owner
                return
            self.navigation['phase'] = 'navigating' if code == 'SUCCEEDED' else 'failed'
            self.navigation['error_code'] = None if code == 'SUCCEEDED' else code
            self.result = (code, time.monotonic())

    def _feedback(self, message, generation):
        with self.lock:
            if generation != self.generation or self.result is not None:
                return
            count = int(message.feedback.number_of_recoveries)
            if count > self.navigation['recoveries']:
                if self.recovery_started is None:
                    self.recovery_started = time.monotonic()
                    self.recovery_pose = self.snapshot().get('pose')
                self.navigation.update(phase='recovering', recoveries=count)
            if count > MAX_RECOVERIES:
                self._fail('nav2_recovery_limit')

    def _check_recovery(self, state):
        if not self.navigation or self.result is not None or self.goal is None:
            return
        pose = state.get('pose')
        self.navigation['current_pose'] = dict(pose) if pose else None
        if not pose or not state.get('localization_ready') or not state.get('nav2_ready'):
            self._fail('nav2_not_ready')
            return
        if self.recovery_started is None:
            return
        if self.recovery_pose is not None and math.hypot(
                pose['x'] - self.recovery_pose['x'], pose['y'] - self.recovery_pose['y']) >= .05:
            # Only measured TF progress ends a recovery episode, never feedback timing.
            self.recovery_started = self.recovery_pose = None
            self.navigation.update(phase='navigating', recovery_elapsed_s=0.0)
            return
        elapsed = time.monotonic() - self.recovery_started
        self.navigation['recovery_elapsed_s'] = round(elapsed, 3)
        if elapsed >= RECOVERY_TIMEOUT_S:
            self._fail('nav2_recovery_timeout')

    def _check_docking(self, state):
        if not getattr(self, 'dock_phase', None) or self.result is not None:
            return
        pose = state.get('pose')
        if not pose or not state.get('localization_ready') or not state.get('nav2_ready'):
            self._fail('nav2_not_ready')
            return
        now = self.node.get_clock().now().nanoseconds/1e9
        if self.dock_phase == 'approach':
            return
        if self.dock_phase == 'await_stop':
            if time.monotonic()-self.dock_waiting_since > 3:
                self._fail('gate_dock_stop_unconfirmed')
                return
            if state.get('stopped'):
                self.docking.begin(now)
                self.dock_phase = 'acquire'
            return
        try:
            target = self.docking.target(pose, now)
        except ValueError as exc:
            self._fail(str(exc))
            return
        if target is None:
            return
        if self.dock_phase == 'acquire' and state.get('stopped'):
            from geometry_msgs.msg import PoseStamped
            from nav2_msgs.action import FollowPath
            if not self.dock_action.server_is_ready():
                self._fail('gate_dock_controller_unavailable')
                return
            # Smac's same-cell path ends at START, losing a sub-cell correction.
            # This bounded final approach supplies its exact endpoint directly
            # to the normal collision-checking controller, never raw cmd_vel.
            goal = FollowPath.Goal()
            goal.path.header.frame_id = 'map'
            goal.path.header.stamp = self.node.get_clock().now().to_msg()
            for step in range(6):
                point = PoseStamped()
                point.header = goal.path.header
                point.pose.position.x = pose['x']+(target['x']-pose['x'])*step/5
                point.pose.position.y = pose['y']+(target['y']-pose['y'])*step/5
                point.pose.orientation.z = math.sin(target['yaw']/2)
                point.pose.orientation.w = math.cos(target['yaw']/2)
                goal.path.poses.append(point)
            goal.controller_id, goal.goal_checker_id = 'GateDock', 'gate_dock_goal_checker'
            self.dock_phase = 'executing'
            self.generation += 1  # ignore any late approach feedback
            self._send_goal(goal, self.generation, client=self.dock_action)
        elif self.dock_phase == 'verify' and state.get('stopped'):
            if (math.hypot(pose['x']-target['x'], pose['y']-target['y']) > .015
                    or abs(math.atan2(math.sin(pose['yaw']-target['yaw']),
                                      math.cos(pose['yaw']-target['yaw']))) > .08):
                self._fail('gate_dock_final_tolerance')
                return
            self.docking.active = False
            self.dock_phase = None
            self.result = ('SUCCEEDED', time.monotonic())

    def _fail(self, code):
        if getattr(self, 'docking', None) is not None:
            self.docking.active = False
        self.result = (code, time.monotonic())
        if self.navigation:
            self.navigation.update(phase='failed', error_code=code)
        if self.handle is not None:
            self.handle.cancel_goal_async()

    def diagnostic(self):
        with self.lock:
            if not self.navigation:
                return None
            diagnostic = dict(self.navigation)
            # Keep the original failure/terminal evidence; expose why retry is blocked.
            if any(f is None or (f.done() and (f.cancelled() or f.exception() is not None))
                   for f in self.cancelled_goals):
                diagnostic['error_code'] = 'nav2_cancel_result_unknown'
            return diagnostic

    def _cancel_handle(self, handle):
        # Keep terminal evidence after cancel(); a new control retry cannot
        # overlap a goal which has merely acknowledged the cancellation request.
        if not hasattr(self, 'cancelled_goals'):
            self.cancelled_goals = []
        terminal = handle.get_result_async() if hasattr(handle, 'get_result_async') else None
        self.cancelled_goals.append(terminal)
        handle.cancel_goal_async()

    def quiescent(self):
        if getattr(self, 'pending_goal_acks', 0):
            return False
        futures = list(self.cancelled_goals)
        if self.goal_ack is not None:
            futures.append(self.goal_ack)
        return all(f is not None and f.done() and not f.cancelled()
                   and f.exception() is None for f in futures)

    def cancel(self):
        if getattr(self, 'docking', None) is not None:
            self.docking.reset()
        self.dock_phase = None
        self.generation += 1  # a late goal acceptance must cancel itself
        if self.handle is not None:
            self._cancel_handle(self.handle)
        self.goal = self.handle = self.result = None
