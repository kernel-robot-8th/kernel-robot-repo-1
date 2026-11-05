"""Nav2 action adapter: actual map/localization/pose evidence, never a timer success."""
import hashlib
import math
import time
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


class Nav2Bridge:
    def __init__(self, node, profile, map_path, lock):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import Buffer, TransformListener
        self.node, self.lock = node, lock
        self.reference = map_reference(map_path, profile)
        self.action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node, spin_thread=False)
        self.map_ok = False
        self.amcl = None
        self.states, self.pending_states = {}, {}
        self.state_clients = {name: node.create_client(GetState, name + '/get_state')
                              for name in ('amcl', 'bt_navigator')}
        self.goal = self.handle = self.result = None
        self.generation = 0
        self.subs = [node.create_subscription(
            OccupancyGrid, 'map', self._map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)),
            node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl, 10)]
        self.timer = node.create_timer(0.5, self._states)

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

    def snapshot(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        ready = self.action.server_is_ready() and self.map_ok and all(
            name in self.states and self.states[name][0] == 3
            and time.monotonic() - self.states[name][1] < 2
            for name in self.state_clients)
        pose = None
        stamp = None
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            now = self.node.get_clock().now().nanoseconds * 1e-9
            amcl = self.amcl
            age = now - (amcl.header.stamp.sec + amcl.header.stamp.nanosec * 1e-9) if amcl else 99
            covariance = amcl.pose.covariance if amcl else []
            if (amcl and amcl.header.frame_id == 'map' and -0.1 <= age <= 2
                    and -0.1 <= now - stamp <= 0.5
                    and all(math.isfinite(v) and 0 <= v <= limit
                            for v, limit in zip((covariance[0], covariance[7], covariance[35]),
                                                (0.04, 0.04, 0.1)))):
                t = tf.transform
                candidate = {'x': t.translation.x, 'y': t.translation.y, 'yaw': yaw(t.rotation)}
                if all(math.isfinite(v) for v in candidate.values()):
                    pose = candidate
        except TransformException:
            pass
        return {'nav2_ready': bool(ready), 'localization_ready': pose is not None,
                'pose': pose, 'stamp': stamp}

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
        self.goal, self.result = payload, None
        future = self.action.send_goal_async(goal)
        future.add_done_callback(lambda f: self._accepted(f, generation))

    def _accepted(self, future, generation):
        with self.lock:
            try:
                handle = future.result()
                if generation != self.generation:
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.result = ('nav2_goal_rejected', time.monotonic())
                    return
                self.handle = handle
                handle.get_result_async().add_done_callback(lambda f: self._result(f, generation))
            except (RuntimeError, ValueError, AttributeError):
                if generation == self.generation:
                    self.result = ('nav2_action_error', time.monotonic())

    def _result(self, future, generation):
        with self.lock:
            if generation != self.generation:
                return
            try:
                result = future.result()
                success = result.status == 4 and getattr(result.result, 'error_code', 0) == 0
                code = 'SUCCEEDED' if success else 'nav2_goal_failed'
            except (RuntimeError, ValueError, AttributeError):
                code = 'nav2_action_error'
            self.result = (code, time.monotonic())

    def cancel(self):
        self.generation += 1  # a late goal acceptance must cancel itself
        if self.handle is not None:
            self.handle.cancel_goal_async()
        self.goal = self.handle = self.result = None
