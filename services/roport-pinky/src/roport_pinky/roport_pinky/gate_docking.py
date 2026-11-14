"""Opt-in gate docking from calibrated ArUco observations; no velocity publisher.

All motion remains a Nav2 goal through the existing controller/lease/watchdog.
The caller must keep the original control target and validate its final tolerance.
"""
import math
from collections import deque
from pathlib import Path

import yaml


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class DockPolicy:
    def __init__(self, data):
        required = {'version', 'field_verified', 'front_extent_m', 'clearance_m',
                    'image_topic', 'camera_info_topic', 'goals'}
        if not isinstance(data, dict) or set(data) != required:
            raise ValueError('gate_dock_invalid_config')
        if type(data['version']) is not int or data['version'] != 1 or data['field_verified'] is not True:
            raise ValueError('gate_dock_calibration_required')
        for key, low, high in [('front_extent_m', .04, .3), ('clearance_m', .04, .08)]:
            if not finite(data[key]) or not low <= data[key] <= high:
                raise ValueError('gate_dock_invalid_geometry')
        for key in ('image_topic', 'camera_info_topic'):
            if not isinstance(data[key], str) or not data[key].strip():
                raise ValueError('gate_dock_invalid_topic')
        goals = data['goals']
        if not isinstance(goals, dict) or not goals:
            raise ValueError('gate_dock_missing_goals')
        for name, entry in goals.items():
            if (not isinstance(name, str) or not name or not isinstance(entry, dict)
                    or set(entry) != {'marker_id', 'marker_size_m'}
                    or type(entry['marker_id']) is not int or not 0 <= entry['marker_id'] < 50
                    or not finite(entry['marker_size_m']) or not .02 <= entry['marker_size_m'] <= .2):
                raise ValueError('gate_dock_invalid_marker')
        self.data, self.goals = data, goals

    def target(self, marker, pose, command):
        """Marker origin/normal in map, measured using the image timestamp TF."""
        values = [marker[k] for k in ('x', 'y', 'nx', 'ny')]
        if not all(finite(v) for v in values):
            raise ValueError('gate_dock_invalid_observation')
        x, y, nx, ny = values
        length = math.hypot(nx, ny)
        if length < .9:  # wall marker must be close to vertical
            raise ValueError('gate_dock_invalid_plane')
        # ArUco +Z faces out of the printed sheet; the robot faces into it.
        heading = math.atan2(-ny, -nx)
        distance = self.data['front_extent_m'] + self.data['clearance_m']
        target = dict(x=x + nx/length*distance, y=y + ny/length*distance, yaw=heading)
        delta = math.hypot(target['x']-pose['x'], target['y']-pose['y'])
        if delta > .30 or abs(wrap(heading-pose['yaw'])) > .35:
            raise ValueError('gate_dock_approach_outside_bounds')
        expected = command['pose']
        if (math.hypot(target['x']-expected['x'], target['y']-expected['y']) > command['position_tolerance_m']
                or abs(wrap(heading-expected['yaw'])) > command['yaw_tolerance_rad']):
            raise ValueError('gate_dock_control_target_mismatch')
        return target


class GateDocking:
    """Read only camera inputs; stable frames, stale/lost/jumping marker => stop."""
    def __init__(self, node, tf, config):
        from sensor_msgs.msg import CameraInfo, CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.tf = node, tf
        path = Path(config)
        if path.stat().st_size > 65536:
            raise ValueError('gate_dock_config_too_large')
        self.policy = DockPolicy(yaml.safe_load(path.read_text()))
        # Load only when explicitly enabled. Ubuntu's python3-opencv includes ArUco.
        import cv2
        if not hasattr(cv2, 'aruco'):
            raise ValueError('gate_dock_aruco_unavailable')
        self.cv = cv2
        self.pending = deque(maxlen=4)
        self.timer = node.create_timer(.05, self._consume)
        self.info = None
        self.active = False
        self.frames = []
        self.error = None
        self.command = None
        self.started = self.last_image = self.last_detection = None
        self.subs = [node.create_subscription(CameraInfo, self.policy.data['camera_info_topic'],
                        self._info, qos_profile_sensor_data),
                     node.create_subscription(CompressedImage, self.policy.data['image_topic'],
                        self._image, qos_profile_sensor_data)]

    def _info(self, info):
        self.info = info

    def reset(self, command=None):
        self.active, self.command = False, command
        self.frames, self.error = [], None
        if hasattr(self, 'pending'):
            self.pending.clear()
        self.started = self.last_image = self.last_detection = None

    def enabled(self, goal_id):
        return goal_id in self.policy.goals

    def begin(self, now):
        self.active, self.started = True, now

    def _image(self, msg):
        if self.active:
            if len(msg.data) > 2*1024*1024:
                self.error = 'gate_dock_image_too_large'
                return
            self.pending.append(msg)

    def _consume(self):
        if not self.active or not self.pending:
            return
        from rclpy.time import Time
        msg = self.pending[0]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        now = self.node.get_clock().now().nanoseconds/1e9
        if not -.05 <= now-stamp <= .25:
            self.pending.popleft()
            return
        # TF and camera callbacks share one executor. Wait for the matching TF
        # in later ticks instead of blocking that executor or using latest TF.
        if not self.tf.can_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp)):
            return
        self.pending.popleft()
        self._process_image(msg)

    def _process_image(self, msg):
        if not self.active:
            return
        import numpy as np
        from rclpy.time import Time
        cv = self.cv
        now = self.node.get_clock().now().nanoseconds/1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if not -.05 <= now-stamp <= .25 or (self.last_image is not None and stamp <= self.last_image):
            return
        if self.last_image is not None and stamp-self.last_image < .15:
            return
        self.last_image = stamp
        if len(msg.data) > 8*1024*1024:
            self.error = 'gate_dock_image_too_large'
            return
        info = self.info
        if (info is None or not info.header.frame_id or info.header.frame_id != msg.header.frame_id
                or info.distortion_model != 'plumb_bob' or len(info.d) != 5
                or len(info.k) != 9 or not all(math.isfinite(v) for v in (*info.k, *info.d))
                or info.k[0] <= 0 or info.k[4] <= 0):
            self.error = 'gate_dock_camera_uncalibrated'
            return
        try:
            image = cv.imdecode(np.frombuffer(msg.data, np.uint8), cv.IMREAD_GRAYSCALE)
            if image is None or image.shape != (info.height, info.width):
                raise ValueError('gate_dock_image_size_mismatch')
            dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
            corners, ids, _ = cv.aruco.detectMarkers(image, dictionary)
            config = self.policy.goals[self.command['goal_id']]
            indexes = [] if ids is None else [i for i, ident in enumerate(ids.flatten())
                                             if int(ident) == config['marker_id']]
            if len(indexes) != 1:
                self.frames = []
                return
            points = corners[indexes[0]].reshape(4, 2)
            if min(np.linalg.norm(points[i]-points[(i+1)%4]) for i in range(4)) < 8:
                self.frames = []
                return
            h = config['marker_size_m']/2
            obj = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]], dtype=np.float64)
            k, d = np.array(info.k).reshape(3,3), np.array(info.d)
            ok, rvec, tvec = cv.solvePnP(obj, points, k, d, flags=cv.SOLVEPNP_IPPE_SQUARE)
            if not ok or not .05 <= float(tvec[2,0]) <= 1.2:
                raise ValueError('gate_dock_invalid_range')
            projection, _ = cv.projectPoints(obj, rvec, tvec, k, d)
            if float(np.sqrt(np.mean((projection.reshape(4,2)-points)**2))) > 2:
                raise ValueError('gate_dock_reprojection_error')
            tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            t, q = tf.transform.translation, tf.transform.rotation
            quat = np.array([q.x,q.y,q.z,q.w], dtype=float)
            if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > .001:
                raise ValueError('gate_dock_invalid_transform')
            x,y,z,w = quat
            rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            origin = rotation @ tvec.flatten() + np.array([t.x,t.y,t.z])
            normal = rotation @ cv.Rodrigues(rvec)[0][:,2]
            marker = dict(x=float(origin[0]), y=float(origin[1]), nx=float(normal[0]), ny=float(normal[1]))
            if self.frames and (math.hypot(marker['x']-self.frames[-1][1]['x'], marker['y']-self.frames[-1][1]['y']) > .03
                    or abs(wrap(math.atan2(marker['ny'],marker['nx'])-math.atan2(self.frames[-1][1]['ny'],self.frames[-1][1]['nx']))) > .1):
                raise ValueError('gate_dock_marker_jump')
            self.last_detection = stamp
            self.frames = (self.frames + [(stamp, marker)])[-3:]
        except (ValueError, cv.error) as exc:
            self.error = str(exc) if isinstance(exc, ValueError) else 'gate_dock_vision_error'
        except Exception:  # timestamp TF unavailable must never produce a guessed target
            self.error = 'gate_dock_transform_unavailable'

    def target(self, pose, now):
        if self.error:
            raise ValueError(self.error)
        if now-self.started > 25:
            raise ValueError('gate_dock_timeout')
        if self.last_detection is not None and now-self.last_detection > .5:
            raise ValueError('gate_dock_marker_lost')
        if not self.frames:
            if now-self.started > 2:
                raise ValueError('gate_dock_marker_missing')
            return None
        if len(self.frames) < 3 or self.frames[-1][0]-self.frames[0][0] < .1:
            return None
        return self.policy.target(self.frames[-1][1], pose, self.command)
