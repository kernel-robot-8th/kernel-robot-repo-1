"""Local pad + camera identity + Nav2. Run on the operator PC, never during bringup."""
import argparse
from collections import OrderedDict, deque
import hashlib
import json
import math
from pathlib import Path
from statistics import median
from threading import Condition, Event, Thread
import time
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan, Range
from tf2_ros import Buffer, TransformException, TransformListener

from .appearance_reid import DinoEncoder, normalized, padded_rgb
from .box_detector import SelectedProposals, ShoppingDetector, proposals
from .session_target import SessionTracker, _box
from .shopping_nav import (MIN_FOLLOW_GAP_M, RETARGET_ANGLE, ShoppingNav,
                           following_distances, target_bearing_change)
from .shopping_track import ProximityNotice, Track, bypass_waypoint, foot_position


class OnboardDinoEncoder:
    def __init__(self, path):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.add_session_config_entry('session.intra_op.allow_spinning', '0')
        self.net = ort.InferenceSession(str(path), sess_options=options, providers=['CPUExecutionProvider'])
        entry, = self.net.get_inputs()
        if entry.shape[1:] != [3, 140, 140] or self.net.get_outputs()[0].shape[-1] != 384:
            raise ValueError('expected a 140 px DINOv2 ViT-S/14 embedding model')
        self.input = entry.name

    def encode(self, crops, size):
        batch = np.stack([padded_rgb(crop, size) for crop in crops]).astype(np.float32) / 255.
        batch -= np.array([.485, .456, .406], dtype=np.float32)
        batch /= np.array([.229, .224, .225], dtype=np.float32)
        return normalized(self.net.run(None, {self.input: np.ascontiguousarray(batch.transpose(0,3,1,2))})[0])


class ShoppingEncoder:
    def __init__(self, backend):
        self.backend = backend
        self.cache = OrderedDict()

    def encode(self, crops):
        # The renderer often repeats identical pixels at rest. Never reuse a changed crop.
        keys = [(c.shape, hashlib.blake2b(c.tobytes(),digest_size=16).digest()) for c in crops]
        missing = [i for i,k in enumerate(keys) if k not in self.cache]
        if missing:
            features = self.backend.encode([crops[i] for i in missing], size=140)
            self.cache.update((keys[i], feature) for i,feature in zip(missing, features))
        result = np.array([self.cache[k] for k in keys])
        for k in keys:
            self.cache.move_to_end(k)
        while len(self.cache)>32:
            self.cache.popitem(last=False)
        return result


class Shopping:
    def __init__(self, node, detector, encoder, optical_frame=None, floor_z=0., calibration=None, compressed=False,
                 follow_distance_m=.34):
        self.node, self.detector, self.encoder = node, detector, encoder
        self.optical_frame, self.floor_z = optical_frame, floor_z
        self.lock, self.closed = Condition(), Event()
        self.nav, self.tf = ShoppingNav(node, follow_distance_m), Buffer()
        self.default_gap = follow_distance_m
        self.field_camera = calibration is not None
        self.gap_selected = False
        self.listener = TransformListener(self.tf, node)
        self.owner = self.packet = self.frozen = self.seed = self.info = None
        self.touch = self.scan_at = self.sonar_at = float('-inf')
        self.generation = 0
        self.order = 0
        self.waiting = deque(maxlen=5)
        self.armed = False
        self.track = Track()
        self.observed = float('-inf')
        self.drive = 'HOLD'
        self.reason = ''
        self.frame_box = None
        self.paused_at = None
        self.still_since = None
        self.odom_at = float('-inf')
        self.close_streak = 0
        self.grid = self.hit = None
        self.grid_at = self.hit_at = float('-inf')
        node.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.costmap, 1)
        if calibration is None:
            node.create_subscription(CameraInfo, 'camera/camera_info', self.camera_info, qos_profile_sensor_data)
        else:
            self.info = calibration
        if compressed or calibration is not None:
            node.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.queue_frame, qos_profile_sensor_data)
        else:
            node.create_subscription(Image, 'camera/image_raw', self.queue_frame, qos_profile_sensor_data)
        node.create_subscription(LaserScan, 'scan', self.scan, qos_profile_sensor_data)
        node.create_subscription(Range, 'us_sensor/range', self.sonar, qos_profile_sensor_data)
        node.create_subscription(Odometry, 'odom', self.odometry, qos_profile_sensor_data)
        self.proximity = ProximityNotice()
        self.display = node.create_publisher(String, 'smartcart/shopping_indicator', 1)
        self.last_display, self.display_at = '', float('-inf')
        self.worker = Thread(target=self.vision, daemon=True)
        self.worker.start()

    def camera_info(self, msg):
        self.info = msg

    def received(self, msg):
        age = (self.node.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
        return time.monotonic()-max(age, 0) if -.1 <= age < .45 else None

    def odometry(self, msg):
        stamp = self.received(msg)
        if stamp is None:
            return
        vel = msg.twist.twist
        if stamp-self.odom_at>.4 or abs(vel.linear.x)>.01 or abs(vel.angular.z)>.06:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = stamp
        self.odom_at = stamp

    def stopped(self, now):
        return (self.nav.boot is None or self.still_since is not None and now-self.still_since>.25
                and now-self.odom_at<.4 and now-self.nav.guard_at<.3 and self.nav.guard.get('owner')=='none'
                and not self.nav.pending and not self.nav.handle)

    def scan(self, msg):
        stamp = self.received(msg)
        if stamp and msg.ranges and any(r >= msg.range_min for r in msg.ranges):
            self.scan_at = stamp

    def costmap(self, msg):
        q = msg.info.origin.orientation
        if (msg.header.frame_id == 'map' and msg.info.resolution > 0
                and len(msg.data) == msg.info.width*msg.info.height and abs(q.w)>.99999):
            self.grid, self.grid_at = msg, time.monotonic()

    def sonar(self, msg):
        stamp = self.received(msg)
        if stamp and (msg.min_range <= msg.range <= msg.max_range or msg.range == math.inf):
            self.sonar_at = stamp
            if msg.range >= .55:
                self.hit = None
            else:
                try:
                    tf = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
                except TransformException:
                    try:
                        nearby = self.tf.lookup_transform('map', msg.header.frame_id, Time())
                    except TransformException:
                        return
                    if abs(Time.from_msg(nearby.header.stamp).nanoseconds -
                           Time.from_msg(msg.header.stamp).nanoseconds)>80_000_000:
                        return
                    tf = nearby.transform
                t, q = tf.translation, tf.rotation
                self.hit_at = stamp
                self.hit = (t.x+msg.range*(1-2*(q.y*q.y+q.z*q.z)),
                            t.y+msg.range*2*(q.x*q.y+q.w*q.z))

    def queue_frame(self, msg):
        # TF can lag several camera frames. Use the newest frame with a matching pose.
        self.waiting.append(msg)

    def camera(self, msg):
        stamp, info = self.received(msg), self.info
        if (stamp is None or info is None or not msg.header.frame_id
                or not self.optical_frame and msg.header.frame_id != info.header.frame_id):
            return
        child = self.optical_frame or info.header.frame_id
        try:
            transform = self.tf.lookup_transform('map', child, Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            try:
                nearby = self.tf.lookup_transform('map', child, Time())
            except TransformException:
                return
            skew = abs(Time.from_msg(nearby.header.stamp).nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)
            if skew > 80_000_000:
                return
            transform = nearby.transform
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None or img.shape[:2] != (info.height, info.width):
                return
        else:
            if (info.width, info.height) != (msg.width, msg.height) or msg.encoding not in ('bgr8', 'rgb8'):
                return
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.packet = (stamp, img, info, transform)
        return True

    def reset(self):
        self.nav.cancel()
        self.nav.path_failures = 0
        self.nav.retry_after = 0.
        self.nav.last_reached = None
        self.nav.follow_distance_m = self.default_gap
        self.gap_selected = False
        self.proximity = ProximityNotice()
        self.generation += 1
        self.frozen = self.seed = self.frame_box = None
        self.track = Track()
        self.close_streak = 0
        self.observed = float('-inf')
        self.armed = False
        self.reason = ''

    def command(self, action, body):
        with self.lock:
            now = time.monotonic()
            key = body.get('session')
            if not isinstance(key, str) or not 16 <= len(key) <= 64:
                raise ValueError('쇼핑 화면을 다시 열어 주세요')
            if self.owner != key:
                if action != 'capture' or self.owner and now-self.touch < 2:
                    raise ValueError('다른 화면에서 사용 중이거나 세션이 종료되었습니다')
                self.reset()
                self.owner = key
                self.order = 0
            if action != 'heartbeat':
                order = body.get('order')
                if type(order) is not int or order <= self.order:
                    raise ValueError('이전 요청이 만료되었습니다')
                self.order = order
            if action == 'heartbeat':
                if now-self.touch > 2:
                    self.armed = False
                self.touch = now
            elif action == 'capture':
                self.reset()
                if not self.packet or now-self.packet[0] > .55:
                    raise ValueError('카메라를 연결하고 있어요. 잠시 뒤 다시 눌러 주세요.')
                stamp, frame, _, _ = self.packet
                ok, jpeg = cv2.imencode('.jpg', frame)
                if not ok:
                    raise ValueError('사진을 다시 찍어 주세요.')
                self.touch = now
                if self.nav.pending is None and self.nav.handle is None:
                    self.nav.fault = None
                self.frozen = (uuid4().hex, (stamp, frame, None, jpeg.tobytes()))
            elif action == 'select':
                idx = body.get('index')
                if (not self.frozen or self.frozen[0] != body.get('frame')
                    or now-self.frozen[1][0] > 20):
                    raise ValueError('화면을 다시 찍고 대상을 선택해 주세요')
                _, frame, boxes, _ = self.frozen[1]
                if 'box' in body:
                    roi = body['box']
                    a,b,c,d = _box(roi, frame.shape[:2], normalized=True)
                    if np.std(cv2.cvtColor(frame[b:d,a:c], cv2.COLOR_BGR2GRAY)) < 8:
                        raise ValueError('무늬가 보이도록 대상의 몸통을 포함해 선택해 주세요')
                else:
                    if type(idx) is not int or not boxes or not 0 <= idx < len(boxes):
                        raise ValueError('다시 찍은 화면에서 대상을 선택해 주세요')
                    roi = boxes[idx]['box']
                self.seed = frame, tuple(roi)
                self.frozen = None
                self.generation += 1
                self.touch = now
            elif action == 'start':
                deadline = now+1.4
                while (not self.can_start(time.monotonic()) and self.owner == key and self.order == order
                       and self.seed is not None and time.monotonic() < deadline):
                    self.lock.wait(timeout=.08)
                if self.owner != key or self.order != order or not self.can_start(time.monotonic()):
                    raise ValueError(self.reason or '같은 대상과 주행 준비를 확인 중입니다')
                self.nav.fault = None
                self.nav.path_failures = 0
                self.nav.retry_after = 0.
                self.gap_selected = True
                self.armed = True
            elif action in ('stop', 'end'):
                self.armed = False
                self.proximity = ProximityNotice()
                if action == 'end':
                    if not self.stopped(now):
                        raise ValueError('카트가 멈추고 있어요. 잠시 뒤 다시 눌러 주세요.')
                    self.reset()
                    self.owner = None
            else:
                raise ValueError('알 수 없는 명령입니다')
            return self.status(key)

    def can_start(self, now):
        return bool(self.seed and self.track.state in ('FOLLOWING','CLOSE') and now-self.observed < .55
                    and (not self.field_camera or self.gap_selected or self.track.state == 'CLOSE')
                    and now-self.touch <= 1.5 and not self.reason
                    and (not self.nav.fault or self.nav.fault == 'PATH_BLOCKED' and self.stopped(now))
                    and self.nav.guard.get('owner')=='none' and not self.nav.handle and not self.nav.pending)

    def status(self, key):
        with self.lock:
            own = key == self.owner and self.owner is not None
            frozen = self.frozen if own else None
            return dict(stopped=self.stopped(time.monotonic()), simulation=self.node.get_parameter('use_sim_time').value, canStart=bool(own and not self.armed and self.can_start(time.monotonic())), selected=bool(own and self.seed), armed=self.armed if own else False,
                        state=self.track.state if own else 'SEARCHING', drive=self.drive if own else 'HOLD',
                        notice=self.indicator() if own else 'INACTIVE', reason=self.reason, age=round(time.monotonic()-self.observed, 2) if self.observed>0 else None,
                        position=self.track.samples[-1][1] if own and self.track.samples else None, frame=frozen[0] if frozen else None,
                        detecting=bool(frozen and frozen[1][2] is None),
                        boxes=[i['box'] for i in frozen[1][2]] if frozen and frozen[1][2] else [],
                        box=self.frame_box if own and time.monotonic()-self.observed<.55 else None)

    def indicator(self):
        if not self.owner or self.seed is None:
            return 'INACTIVE'
        if self.nav.fault == 'PATH_BLOCKED' or self.armed and self.nav.retry_after:
            return 'BLOCKED'
        if self.reason or self.nav.fault:
            return 'UNAVAILABLE'
        if not self.armed:
            return 'PAUSED'
        return 'COME_CLOSER' if self.proximity.active else 'SHOPPING'

    def image(self, frame_id):
        with self.lock:
            if frame_id:
                if not self.frozen or self.frozen[0] != frame_id:
                    return None
                return self.frozen[1][3]
            packet = self.packet
        if packet and time.monotonic()-packet[0] < .8:
            ok, jpeg = cv2.imencode('.jpg', packet[1], [cv2.IMWRITE_JPEG_QUALITY, 75])
            return jpeg.tobytes() if ok else None
        return None

    def detect_capture(self, frozen):
        frame_id, (stamp, frame, _, jpeg) = frozen
        try:
            boxes = sorted(proposals(self.detector, frame), key=lambda item: item['box'][0])
        except Exception as exc:
            self.node.get_logger().error(f'Selection detection failed: {exc}')
            boxes = []
        with self.lock:
            if self.frozen is frozen:
                self.frozen = frame_id, (stamp, frame, boxes, jpeg)
                self.node.get_logger().info(f'Selection candidates: {len(boxes)}')

    def vision(self):
        tracker, version, previous = None, -1, None
        templates, last_yolo = None, 0.
        while not self.closed.wait(.01):
            with self.lock:
                packet, generation, seed = self.packet, self.generation, self.seed
                frozen = self.frozen
            if frozen and frozen[1][2] is None:
                self.detect_capture(frozen)
                continue
            if not seed or packet is None or packet is previous:
                continue
            previous = packet
            stamp, frame, info, camera = packet
            h, w = frame.shape[:2]
            tic = time.monotonic()
            try:
                if generation != version:
                    templates = SelectedProposals(*seed)
                items = templates.detect(frame)
                template_count = len(items)
                scan_interval = 3. if len(items)==1 and self.track.state=='FOLLOWING' else .7
                if not items or time.monotonic()-last_yolo >= scan_interval:
                    # After selection, one full-frame YOLO pass sees people entering from either side.
                    detected = self.detector.full_frame(frame)
                    for item in detected:
                        if not any(SessionTracker._overlaps(item['box'], other['box']) for other in items):
                            items.append(item)
                    last_yolo = time.monotonic()
                after_detection = time.monotonic()
                boxes = [i['box'] for i in items]
                normalized = [dict(box=[a/w,b/h,c/w,d/h]) for a,b,c,d in boxes]
                if version != generation:
                    tracker = SessionTracker(self.encoder, lambda _: boxes, *seed)
                    version = generation
                pick, point, uncertain, edge = None, None, False, False
                if tracker:
                    with self.lock:
                        anchor = self.track.samples[-1][1] if self.track.samples else None
                    def near_anchor(box):
                        location = foot_position(box, info, camera, self.floor_z)
                        return anchor and location and math.dist(anchor, location) < .18
                    _, scores, match = tracker.step(frame, near_anchor=near_anchor)
                    if match['state'] == 'MATCH':
                        pick = match['index']
                        if pick >= template_count:
                            # Refresh the fast proposal crop; identity still uses the original portrait.
                            templates = SelectedProposals(frame, normalized[pick]['box'])
                        a,b,c,d = boxes[pick]
                        point = foot_position(boxes[pick], info, camera, self.floor_z)
                        edge = (a+c)/(2*w) < .28 or (a+c)/(2*w) > .72
                        uncertain = point is None
                    else:
                        uncertain = match['state']=='AMBIGUOUS' or match.get('reason')=='processing_error'
                with self.lock:
                    if generation != self.generation:
                        continue
                    if tracker:
                        cropped = pick is not None and point is None and boxes[pick][3]>=h-2
                        self.close_streak = self.close_streak+1 if cropped else 0
                        self.track.update(stamp, point, edge, uncertain, (a+c)/(2*w) if pick is not None else None)
                        if self.close_streak >= 3:
                            self.track.state = 'CLOSE'
                        self.frame_box = normalized[pick]['box'] if pick is not None else None
                        if self.track.state != getattr(self, '_logged_state', None) or time.monotonic()-getattr(self,'_logged_at',0)>2:
                            self._logged_at, self._logged_state = time.monotonic(), self.track.state
                            self.node.get_logger().info(f'{self.track.state} match={match} scores={[round(v["score"],2) for v in scores]} candidates={len(items)} pose={point} camera_age={tic-stamp:.2f} detect={after_detection-tic:.2f} match={time.monotonic()-after_detection:.2f}')
                        self.observed = stamp
                        self.lock.notify_all()
                        self.node.get_logger().debug(f'{self.track.state} target={point} latency={time.monotonic()-stamp:.3f}')
            except Exception as exc:
                with self.lock:
                    self.armed = False
                    self.frame_box = None
                self.node.get_logger().error(str(exc), throttle_duration_sec=3)

    def tick(self):
        for index in range(len(self.waiting)-1, -1, -1):
            if self.camera(self.waiting[index]):
                for _ in range(index+1):
                    self.waiting.popleft()
                break
        now = time.monotonic()
        try:
            base = self.tf.lookup_transform('map', 'base_footprint', Time())
            age = (self.node.get_clock().now().nanoseconds-Time.from_msg(base.header.stamp).nanoseconds)/1e9
            t, q = base.transform.translation, base.transform.rotation
            pose = (t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))) if -.15<=age<=1.2 else None
        except TransformException:
            pose = None
        with self.lock:
            guard_ready = self.nav.ready(now)
            if self.owner and now-self.touch > 2:
                self.reset()
                self.owner = None
            self.reason = ('주행 가드 또는 Nav2 연결 대기' if not guard_ready else
                           '지도 위치 확인 대기' if pose is None else
                           '라이다·초음파 연결 대기' if now-min(self.scan_at,self.sonar_at)>.5 else
                           '카메라 연결 대기' if self.packet is None or now-self.packet[0]>.5 else '')
            if now-self.touch > 1.5 or self.nav.fault or self.nav.guard.get('blocked'):
                self.armed = False
            target = self.track.target()
            if (self.field_camera and self.seed and not self.gap_selected and not self.armed and pose
                    and self.track.state == 'FOLLOWING' and now - self.observed < .55):
                distances = [math.dist(pose[:2], point) for _, point in list(self.track.samples)[-3:]]
                selected_distance = median(distances)
                self.gap_selected = True
                gap, advance_at = following_distances(self.nav.follow_distance_m)
                self.node.get_logger().info(
                    f'Selected at {selected_distance:.3f} m; '
                    f'following gap {gap:.3f} m; advance at {advance_at:.3f} m')
            if self.track.state=='CORNER' and now-self.track.samples[-1][0]>2:
                target = None
            if (self.track.state=='VERIFYING' and self.nav.anchor and pose and self.track.samples
                    and math.dist(self.nav.anchor, self.track.samples[-1][1]) < .14
                    and abs(target_bearing_change(pose, self.nav.anchor, self.track.samples[-1][1])) <= RETARGET_ANGLE):
                # A unique matched crop can sustain only the existing, already verified leg.
                target = self.nav.anchor
            fresh_result = now-self.observed < .6
            # Nav2 owns only this already checked waypoint. A pan or a slow
            # identity inference cannot invalidate the costmap path. Never use
            # a delayed result to start a leg, follow a new person, or drive
            # after camera/scan/sonar/lease loss.
            committed = bool(self.nav.detouring and self.nav.anchor and self.track.samples and
                             now - self.track.samples[-1][0] < 15 and
                             self.track.state in ('SEARCHING','VERIFYING','FOLLOWING') and
                             self.nav.handle and not self.nav.cancelled)
            if committed:
                target = self.nav.anchor
            # Pausing wheel permission is immediate. Reuse a pending Nav2 path after
            # a brief sensor/inference gap instead of restarting its recovery tree.
            gap = self.armed and guard_ready and self.nav.anchor is not None and (
                self.reason or not (fresh_result or committed) or self.track.state=='VERIFYING' and target is None)
            if gap and self.paused_at is None:
                self.paused_at = now
            if not gap:
                self.paused_at = None
            if gap and now-self.paused_at < 1. and self.track.state not in ('AMBIGUOUS','CLOSE'):
                self.nav.poll()
                self.nav.heartbeat(False, now)
                self.drive = 'HOLD'
            else:
                active = self.armed and not self.reason and (fresh_result or committed)
                via = None
                if (active and pose and target and not self.nav.handle and not self.nav.pending
                        and self.track.state=='FOLLOWING' and now-self.grid_at<3. and now-self.hit_at<.4):
                    via = bypass_waypoint(pose, target, self.hit, self.grid)
                self.drive = self.nav.tick(now, active, pose, target, via,
                                           allow_new=fresh_result and (self.track.state == 'FOLLOWING' or
                                               self.track.state == 'CORNER' and not self.nav.retry_after),
                                           direct_grid=self.grid if (self.track.state == 'FOLLOWING' and
                                               fresh_result and now-self.grid_at < 2.) else None)
            if self.nav.fault:
                self.armed = False
                if self.nav.fault != 'PATH_BLOCKED':
                    self.reason = '경로 확인 필요 · 대상을 다시 선택해 주세요'
            confirmed = self.track.target() if self.track.state == 'FOLLOWING' else None
            distance = math.dist(pose[:2], confirmed) if pose and confirmed else None
            if fresh_result or not self.armed or self.reason:
                self.proximity.update(now, self.armed and not self.reason and not committed, self.track.state, distance)
            notice = self.indicator()
            if notice != self.last_display or now-self.display_at >= .5:
                if notice != self.last_display:
                    self.node.get_logger().info(f'Shopping indicator {notice} nav_fault={self.nav.fault}')
                self.display.publish(String(data=notice))
                self.last_display, self.display_at = notice, now


def main():
    from .shopping_http import serve
    parser = argparse.ArgumentParser()
    parser.add_argument('--pad-dist', type=Path, required=True)
    parser.add_argument('--onnx', type=Path, required=True)
    encoders = parser.add_mutually_exclusive_group(required=True)
    encoders.add_argument('--encoder-source', type=Path)
    encoders.add_argument('--encoder-onnx', type=Path)
    parser.add_argument('--weights', type=Path)
    parser.add_argument('--port', default=8182, type=int)
    parser.add_argument('--optical-frame')
    parser.add_argument('--follow-distance-m', type=float,
                        help='following gap measured from base_footprint to the selected shopper')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--sim', action='store_true')
    parser.add_argument('--compressed', action='store_true', help='read the JPEG transport in Gazebo as on Pinky')
    parser.add_argument('--field-standalone', action='store_true')
    args = parser.parse_args()
    if bool(args.encoder_source) != bool(args.weights):
        parser.error('--weights must accompany --encoder-source')
    if args.follow_distance_m is None:
        args.follow_distance_m = MIN_FOLLOW_GAP_M if args.field_standalone else .34
    if not math.isfinite(args.follow_distance_m) or args.follow_distance_m < MIN_FOLLOW_GAP_M:
        parser.error(f'follow distance must be finite and at least {MIN_FOLLOW_GAP_M} m from base_footprint')
    import os
    if args.sim == args.field_standalone or (args.sim and (
            os.environ.get('ROS_DOMAIN_ID') in (None,'0','13') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST'
            or os.environ.get('ROS_STATIC_PEERS') or os.environ.get('ROS_DISCOVERY_SERVER'))):
        parser.error('choose an isolated --sim graph or explicitly run --field-standalone')
    calibration = None
    if args.field_standalone:
        if not args.calibration or not args.optical_frame:
            parser.error('field camera requires its calibrated optical frame and calibration YAML')
        import yaml
        raw = yaml.safe_load(args.calibration.read_text())
        if raw.get('optical_tf_verified') is not True:
            parser.error('camera optical transform requires field verification')
        calibration = CameraInfo()
        calibration.height, calibration.width = raw['image_height'], raw['image_width']
        calibration.k = raw['camera_matrix']['data']
        calibration.d = raw['distortion_coefficients']['data']
        calibration.distortion_model = raw['distortion_model']
    cv2.setNumThreads(1)
    detector = ShoppingDetector(args.onnx, {0:'adult', 1:'child', 2:'human'}, confidence=.22)
    if args.encoder_onnx:
        backend = OnboardDinoEncoder(args.encoder_onnx)
    else:
        backend = DinoEncoder(args.encoder_source, args.weights)
        backend.torch.set_num_threads(1)
    encoder = ShoppingEncoder(backend)
    rclpy.init()
    node = rclpy.create_node('shopping_follow', parameter_overrides=[Parameter('use_sim_time',value=args.sim)])
    shop = Shopping(node, detector, encoder, args.optical_frame, calibration=calibration,
                    compressed=args.compressed, follow_distance_m=args.follow_distance_m)
    node.create_timer(.05, shop.tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
    server = serve(shop, args.pad_dist, args.port)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        with shop.lock:
            shop.armed = False
            if rclpy.ok():
                shop.nav.cancel()
                shop.nav.heartbeat(False, time.monotonic())
        shop.closed.set()
        server.shutdown()
        server.server_close()
        shop.worker.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
