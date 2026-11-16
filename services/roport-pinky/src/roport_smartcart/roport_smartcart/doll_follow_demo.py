"""Supervised, low-speed paper-doll following demo through trial_guard."""

import argparse
import json
import math
import time
from uuid import uuid4


def sampled_at(stamp, ros_now, now, max_age=.4):
    """Use acquisition time, including DDS queue delay, rather than callback time."""
    value = stamp.sec + stamp.nanosec * 1e-9
    age = ros_now - value
    if value <= 0 or not math.isfinite(age) or not -.1 <= age <= max_age:
        return None
    return now - max(0., age)


def guard_body(data):
    try:
        body = json.loads(data)
        if (isinstance(body, dict) and isinstance(body.get('boot'), str) and body['boot']
                and body.get('owner') in ('none', 'line', 'nav2')
                and type(body.get('blocked')) is bool
                and type(body.get('sent_at')) in (int, float)
                and math.isfinite(body['sent_at'])):
            return body
    except (ValueError, TypeError):
        pass
    return {}


def front_distance(msg, rotation):
    """Select the ±30° sector after rotating scan rays into base_link."""
    if (not all(math.isfinite(v) for v in
                (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
            or msg.angle_increment == 0 or not 0 <= msg.range_min < msg.range_max):
        return None
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    x, y, z, w = (v / norm for v in (x, y, z, w))
    distances = []
    for i, distance in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        c, s = math.cos(angle), math.sin(angle)
        forward = (1 - 2*(y*y + z*z))*c + 2*(x*y - z*w)*s
        left = 2*(x*y + z*w)*c + (1 - 2*(x*x + z*z))*s
        if forward <= 0 or abs(math.atan2(left, forward)) > math.pi / 6:
            continue
        if distance == math.inf or math.isfinite(distance) and distance >= msg.range_min:
            distances.append(distance if distance < msg.range_max else math.inf)
    return min(distances) if distances else None


class DriveHandshake:
    """Request ownership at zero speed; latch stopped if acknowledgement is missing/lost."""

    def __init__(self):
        self.requested_at = None
        self.confirmed = False

    def start(self, now):
        self.requested_at, self.confirmed = now, False

    def check(self, owner, now):
        if self.requested_at is None:
            return 'failed'
        if self.confirmed:
            if owner == 'line':
                return 'confirmed'
            self.requested_at = None
            return 'failed'
        if now - self.requested_at >= .3:
            self.requested_at = None
            return 'failed'
        if owner == 'line':
            self.confirmed = True
            return 'confirmed'
        return 'waiting'


class FollowLogic:
    """Image-size following; stop on stale input, loss, or a near obstacle."""

    def __init__(self, label, reference_heights=None):
        self.label = label
        self.reference_heights = reference_heights or {}
        self.goal_height = None
        self.goal_range = None
        self.armed = False
        self.target = None
        self.width = 0
        self.frame_at = None
        self.seen_at = None
        self.started_at = None
        self.reason = 'CALIBRATE AT DESIRED DISTANCE'

    def observe(self, detections, width, now):
        hits = [d for d in detections if d['label'] == self.label]
        self.target = hits[0] if len(hits) == 1 else None
        self.width, self.frame_at = width, now
        if self.target is not None:
            self.seen_at = now

    def calibrate(self, range_m):
        if self.target is None or self.frame_at is None or time.monotonic() - self.frame_at > .4:
            self.reason = 'TARGET NOT VISIBLE'
            return False
        if range_m is None or not math.isfinite(range_m) or range_m <= .05:
            self.reason = 'MOVE DOLL BEYOND 5 CM'
            return False
        if self.height(self.target) < 25:
            self.reason = 'MOVE DOLL CLOSER'
            return False
        self.goal_height = self.apparent_scale(self.target)
        self.goal_range = range_m
        self.reason = 'CALIBRATED; START TO FOLLOW'
        return True

    @staticmethod
    def height(target):
        return max(p[1] for p in target['quad']) - min(p[1] for p in target['quad'])

    def apparent_scale(self, target):
        return self.height(target) / self.reference_heights.get(target['view'], 1)

    def arm(self, now):
        if self.goal_height is None or self.target is None:
            self.reason = 'CALIBRATE VISIBLE TARGET FIRST'
            return False
        self.armed, self.started_at = True, now
        self.reason = 'FOLLOWING'
        return True

    def disarm(self, reason='STOPPED'):
        self.armed = False
        self.reason = reason

    def command(self, now, sonar, scan):
        if not self.armed:
            return 0., 0.
        if self.started_at is None or now - self.started_at > 30:
            self.disarm('30 SECOND LIMIT; STOPPED')
        elif self.frame_at is None or now - self.frame_at > .4:
            self.disarm('CAMERA STALE; STOPPED')
        elif (sonar is None or scan is None or now - sonar[1] > .4
              or now - scan[1] > .4):
            self.disarm('RANGE SENSOR STALE; STOPPED')
        elif sonar[0] <= .05:
            self.reason = 'SONAR <=5 CM; STOPPED'
            return 0., 0.
        elif scan[0] < .25:
            self.reason = 'LIDAR <25 CM; STOPPED'
            return 0., 0.
        elif self.target is None:
            if self.seen_at is None or now - self.seen_at > 2.:
                self.disarm('TARGET LOST; STOPPED')
            else:
                self.reason = 'TARGET TEMPORARILY LOST'
            return 0., 0.
        if not self.armed:
            return 0., 0.
        quad = self.target['quad']
        center = sum(p[0] for p in quad) / 4
        error = (center - self.width / 2) / (self.width / 2)
        angular = max(-.25, min(.25, -.35 * error)) if abs(error) > .1 else 0.
        linear = (.025 if self.apparent_scale(self.target) < self.goal_height
                  and sonar[0] > self.goal_range and abs(error) < .3 else 0.)
        self.reason = 'FOLLOWING' if linear or angular else 'AT CALIBRATED DISTANCE'
        return linear, angular


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--references', required=True)
    parser.add_argument('--target', required=True, choices='ABC')
    args = parser.parse_args(argv)

    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from sensor_msgs.msg import CompressedImage, LaserScan, Range
    from std_msgs.msg import String
    from .doll_recognition import DollRecognizer

    cv2.setNumThreads(1)
    detector = DollRecognizer(args.references)
    reference_heights = {view: shape[0] for label, view, shape, *_ in detector.references
                         if label == args.target}
    logic = FollowLogic(args.target, reference_heights)
    rclpy.init()
    node = rclpy.create_node('smartcart_doll_follow_demo')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    velocity = node.create_publisher(Twist, 'line_nav/cmd_vel', 1)
    lease = node.create_publisher(String, 'trial/lease', 1)
    session, seq = uuid4().hex, 0
    guard, guard_at, boot = {}, None, None
    sonar = scan = None
    frame = None
    handshake = DriveHandshake()
    status = node.create_publisher(String, 'doll_follow/status', 1)
    preview = node.create_publisher(CompressedImage, 'doll_follow/preview/compressed', 1)

    def guard_update(msg):
        nonlocal guard, guard_at
        guard = guard_body(msg.data)
        age = time.time() - guard.get('sent_at', 0)
        guard_at = time.monotonic() - max(0., age) if guard and -.1 <= age <= .3 else None

    def capture_time(msg):
        return sampled_at(msg.header.stamp, node.get_clock().now().nanoseconds * 1e-9,
                          time.monotonic())

    def camera_update(msg):
        nonlocal frame
        captured_at = capture_time(msg)
        if captured_at is None:
            logic.disarm('CAMERA STALE; STOPPED')
            return
        if logic.frame_at is not None and captured_at - logic.frame_at < .16:
            return
        decoded = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            return
        detections = detector.detect(decoded)
        logic.observe(detections, decoded.shape[1], captured_at)
        frame = decoded
        for detection in detections:
            quad = np.int32(detection['quad'])
            color = (0, 220, 0) if detection['label'] == args.target else (0, 180, 255)
            cv2.polylines(frame, [quad], True, color, 2)
            x, y = quad.min(axis=0)
            cv2.putText(frame, detection['label'], (int(x), max(55, int(y))),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)

    def sonar_update(msg):
        nonlocal sonar
        if (math.isfinite(msg.range) and math.isfinite(msg.min_range)
                and math.isfinite(msg.max_range) and msg.min_range <= msg.range <= msg.max_range):
            captured_at = capture_time(msg)
            sonar = (msg.range, captured_at) if captured_at is not None else None
        else:
            sonar = None

    def scan_update(msg):
        nonlocal scan
        scan = None
        captured_at = capture_time(msg)
        if captured_at is None or not msg.header.frame_id:
            return
        try:
            transform = tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        distance = front_distance(msg, transform.transform.rotation)
        if distance is not None:
            scan = (distance, captured_at)

    node.create_subscription(String, 'trial/guard_state', guard_update, 1)
    node.create_subscription(CompressedImage, 'camera/image_raw/compressed', camera_update,
                             sensor_qos)
    node.create_subscription(Range, 'us_sensor/range', sonar_update, sensor_qos)
    node.create_subscription(LaserScan, 'scan', scan_update, sensor_qos)

    def single_publisher(topic, name):
        publishers = node.get_publishers_info_by_topic(topic)
        return len(publishers) == 1 and publishers[0].node_name == name

    def readiness_issue():
        now = time.monotonic()
        if guard_at is None or now - guard_at >= .3:
            return '주행 가드 상태 지연 또는 PC·로봇 시계 불일치'
        if not guard.get('boot') or guard.get('blocked') or guard.get('owner') not in ('none', 'line'):
            return '주행 가드 차단 또는 점유'
        if sonar is None or now - sonar[1] >= .4:
            return '초음파 데이터 지연'
        if scan is None or now - scan[1] >= .4:
            return '라이다 데이터 지연'
        if not single_publisher('cmd_vel', 'trial_guard'):
            return '모터 명령 발행자 충돌'
        if not single_publisher('trial/lease', node.get_name()):
            return '주행 lease 발행자 충돌'
        if not single_publisher('line_nav/cmd_vel', node.get_name()):
            return '주행 명령 발행자 충돌'
        return ''

    def ready():
        return not readiness_issue()

    def send(owner, linear=0., angular=0.):
        nonlocal seq
        if boot is None:
            return
        seq += 1
        lease.publish(String(data=json.dumps(dict(
            boot=boot, owner=owner, seq=seq, session=session, sent_at=time.time()))))
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        velocity.publish(command)

    def control(msg):
        nonlocal boot
        now = time.monotonic()
        if msg.data == 'stop':
            logic.disarm('OPERATOR STOPPED')
            send('none')
        elif msg.data == 'calibrate':
            logic.disarm('STOPPED')
            send('none')
            logic.calibrate(sonar[0] if sonar and now - sonar[1] < .4 else None)
        elif msg.data == 'start' and not logic.armed:
            issue = readiness_issue()
            if (not issue and guard.get('owner') == 'none'
                    and logic.frame_at is not None and now - logic.frame_at < .4):
                boot = guard['boot']
                if logic.arm(now):
                    handshake.start(now)
                    logic.reason = 'WAITING FOR DRIVE GUARD'
                    send('line')
            else:
                logic.reason = issue or '카메라 지연 또는 가드 점유'

    node.create_subscription(String, 'doll_follow/control', control, 1)
    node.get_logger().info('doll_follow/control: calibrate / start / stop; status: doll_follow/status')
    next_tick = time.monotonic()
    next_preview = next_tick
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now >= next_tick:
                next_tick = now + .1
                linear = angular = 0.
                if logic.armed and (not ready() or guard.get('boot') != boot):
                    logic.disarm('DRIVE GUARD / SENSOR NOT READY')
                if logic.armed:
                    state = handshake.check(guard.get('owner'), now)
                    if state == 'failed':
                        logic.disarm('DRIVE GUARD ACK MISSING OR LOST; START AGAIN')
                    elif state == 'waiting':
                        logic.reason = 'WAITING FOR DRIVE GUARD'
                        send('line')
                    else:
                        linear, angular = logic.command(now, sonar, scan)
                        if logic.armed:
                            send('line', linear, angular)
                if not logic.armed and boot is not None:
                    send('none')
                body = {
                    'reason': logic.reason, 'armed': logic.armed,
                    'target': (logic.target['label'] + ' · ' + logic.target['view']
                               if logic.target else '미검출'),
                    'guard': f"{guard.get('owner', '없음')} / {'차단' if guard.get('blocked') else '정상'}",
                    'sonar': f'{sonar[0] * 100:.1f} cm' if sonar else '없음',
                    'goal_range': (f'{logic.goal_range * 100:.1f} cm'
                                   if logic.goal_range is not None else '미설정'),
                    'lidar': f'{scan[0] * 100:.1f} cm' if scan else '없음',
                    'linear': f'{linear * 100:.1f} cm/s',
                    'angular': f'{angular:.2f} rad/s',
                    'readiness': readiness_issue() or '준비 완료',
                }
                status.publish(String(data=json.dumps(body)))
            if now >= next_preview:
                next_preview = now + .2
                image = frame.copy() if frame is not None else np.zeros((480, 640, 3), np.uint8)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    message = CompressedImage()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.format, message.data = 'jpeg', encoded.tobytes()
                    preview.publish(message)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        logic.disarm()
        # Shutdown may already have invalidated the ROS context; the robot watchdog remains authoritative.
        if rclpy.ok():
            try:
                for _ in range(3):
                    send('none')
                    time.sleep(.05)
            except RuntimeError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
