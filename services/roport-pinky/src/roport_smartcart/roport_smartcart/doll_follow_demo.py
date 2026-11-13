"""Supervised, low-speed paper-doll following demo through trial_guard."""

import argparse
import json
import math
import time
from uuid import uuid4


class FollowLogic:
    """Image-size following; stop on stale input, loss, or a near obstacle."""

    def __init__(self, label):
        self.label = label
        self.goal_height = None
        self.goal_view = None
        self.armed = False
        self.target = None
        self.width = 0
        self.frame_at = None
        self.seen_at = None
        self.started_at = None
        self.reason = 'PLACE DOLL 30 CM AHEAD; PRESS C'

    def observe(self, detections, width, now):
        hits = [d for d in detections if d['label'] == self.label]
        self.target = hits[0] if len(hits) == 1 else None
        self.width, self.frame_at = width, now
        if self.target is not None:
            self.seen_at = now

    def calibrate(self):
        if self.target is None or self.frame_at is None or time.monotonic() - self.frame_at > .4:
            self.reason = 'TARGET NOT VISIBLE'
            return False
        self.goal_height = self.height(self.target)
        if self.goal_height < 25:
            self.reason = 'MOVE DOLL CLOSER'
            return False
        self.goal_view = self.target['view']
        self.reason = 'CALIBRATED; PRESS SPACE TO FOLLOW'
        return True

    @staticmethod
    def height(target):
        return max(p[1] for p in target['quad']) - min(p[1] for p in target['quad'])

    def arm(self, now):
        if self.goal_height is None or self.target is None or self.target['view'] != self.goal_view:
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
        elif min(sonar[0], scan[0]) < .25:
            self.reason = 'OBSTACLE <25 CM; STOPPED'
            return 0., 0.
        elif self.target is None or self.target['view'] != self.goal_view:
            if self.seen_at is None or now - self.seen_at > .8:
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
        linear = .025 if self.height(self.target) < .8 * self.goal_height and abs(error) < .3 else 0.
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
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, LaserScan, Range
    from std_msgs.msg import String
    from .doll_recognition import DollRecognizer

    detector = DollRecognizer(args.references)
    logic = FollowLogic(args.target)
    rclpy.init()
    node = rclpy.create_node('smartcart_doll_follow_demo')
    velocity = node.create_publisher(Twist, 'line_nav/cmd_vel', 1)
    lease = node.create_publisher(String, 'trial/lease', 1)
    session, seq = uuid4().hex, 0
    guard, guard_at, boot = {}, None, None
    sonar = scan = None
    frame = None
    owned = False

    def guard_update(msg):
        nonlocal guard, guard_at
        try:
            guard = json.loads(msg.data)
            guard_at = time.monotonic()
        except (ValueError, TypeError):
            guard = {}

    def camera_update(msg):
        nonlocal frame
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -.1 <= age <= .4:
            return
        captured_at = time.monotonic() - max(0., age)
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
            sonar = (msg.range, time.monotonic())

    def scan_update(msg):
        nonlocal scan
        if (not math.isfinite(msg.angle_increment) or msg.angle_increment == 0
                or not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max)
                or msg.range_min >= msg.range_max):
            return
        rays = [distance for i, distance in enumerate(msg.ranges)
                if abs(msg.angle_min + i * msg.angle_increment) <= math.pi / 6]
        valid = [distance for distance in rays
                 if distance == math.inf or math.isfinite(distance) and distance >= msg.range_min]
        front = [distance for distance in rays
                 if math.isfinite(distance) and msg.range_min <= distance < msg.range_max]
        if valid:
            scan = (min(front, default=math.inf), time.monotonic())

    node.create_subscription(String, 'trial/guard_state', guard_update, 10)
    node.create_subscription(CompressedImage, 'camera/image_raw/compressed', camera_update,
                             qos_profile_sensor_data)
    node.create_subscription(Range, 'us_sensor/range', sonar_update, qos_profile_sensor_data)
    node.create_subscription(LaserScan, 'scan', scan_update, qos_profile_sensor_data)

    def single_publisher(topic, name):
        publishers = node.get_publishers_info_by_topic(topic)
        return len(publishers) == 1 and publishers[0].node_name == name

    def ready():
        return (guard_at is not None and time.monotonic() - guard_at < .3
                and guard.get('boot') and not guard.get('blocked')
                and guard.get('owner') in ('none', 'line')
                and single_publisher('cmd_vel', 'trial_guard')
                and single_publisher('trial/lease', node.get_name())
                and single_publisher('line_nav/cmd_vel', node.get_name())
                and sonar is not None and scan is not None
                and time.monotonic() - sonar[1] < .4 and time.monotonic() - scan[1] < .4)

    def send(owner, linear=0., angular=0.):
        nonlocal seq, owned
        if boot is None:
            return
        seq += 1
        lease.publish(String(data=json.dumps(dict(
            boot=boot, owner=owner, seq=seq, session=session, sent_at=time.time()))))
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        velocity.publish(command)
        owned = owner == 'line'

    window = 'Pinky doll follow demo - C calibrate, SPACE start/stop, Q quit'
    cv2.namedWindow(window)
    next_tick = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now >= next_tick:
                next_tick = now + .1
                if logic.armed and (not ready() or guard.get('boot') != boot):
                    logic.disarm('DRIVE GUARD / SENSOR NOT READY')
                if logic.armed:
                    linear, angular = logic.command(now, sonar, scan)
                    if logic.armed:
                        send('line', linear, angular)
                if not logic.armed and owned:
                    send('none')
            image = frame.copy() if frame is not None else np.zeros((480, 640, 3), np.uint8)
            cv2.rectangle(image, (0, 0), (image.shape[1], 42), (0, 0, 0), -1)
            cv2.putText(image, logic.reason[:46], (7, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        .55, (255, 255, 255), 2)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord('c'):
                logic.calibrate()
            if key == ord(' '):
                if logic.armed:
                    logic.disarm('OPERATOR STOPPED')
                elif (ready() and guard.get('owner') == 'none'
                      and logic.frame_at is not None and now - logic.frame_at < .4):
                    boot = guard['boot']
                    logic.arm(now)
                else:
                    logic.reason = 'GUARD, CAMERA, SCAN AND SONAR REQUIRED'
    finally:
        logic.disarm()
        if owned:
            for _ in range(3):
                send('none')
                time.sleep(.05)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
