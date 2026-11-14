"""Exercise the ROS adapter in an isolated domain with synthetic perception, no hardware."""
import json
import math
import os
import signal
import subprocess
import sys
import time

import pytest


def test_follow_requires_ack_and_stops_on_lost_ownership(tmp_path):
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import TransformStamped, Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, LaserScan, Range
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster

    ctx = Context()
    rclpy.init(context=ctx, domain_id=193)
    node = rclpy.create_node('trial_guard', context=ctx)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    camera = node.create_publisher(CompressedImage, 'camera/image_raw/compressed', qos_profile_sensor_data)
    sonar = node.create_publisher(Range, 'us_sensor/range', qos_profile_sensor_data)
    scan = node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
    guard = node.create_publisher(String, 'trial/guard_state', 1)
    motor = node.create_publisher(Twist, 'cmd_vel', 1)
    control = node.create_publisher(String, 'doll_follow/control', 1)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'rplidar_link'
    tf.transform.rotation.z = 1.  # actual Pinky sensor mount: yaw=pi
    broadcaster.sendTransform(tf)
    owner, accept, height, distance = 'none', True, 50, .08
    statuses, commands = [], []

    def lease(msg):
        nonlocal owner
        body = json.loads(msg.data)
        if accept or body['owner'] == 'none':
            owner = body['owner']

    node.create_subscription(String, 'trial/lease', lease, 1)
    node.create_subscription(String, 'doll_follow/status',
                             lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(Twist, 'line_nav/cmd_vel',
                             lambda m: commands.append(m.linear.x), 10)

    def tick():
        stamp = node.get_clock().now().to_msg()
        image = CompressedImage()
        image.header.stamp, image.format = stamp, 'jpeg'
        image.data = cv2.imencode('.jpg', np.full((100, 640, 3), height, np.uint8))[1].tobytes()
        camera.publish(image)
        reading = Range()
        reading.header.stamp, reading.min_range, reading.max_range = stamp, .02, 3.
        reading.range = distance
        sonar.publish(reading)
        rays = LaserScan()
        rays.header.stamp, rays.header.frame_id = stamp, 'rplidar_link'
        rays.angle_min, rays.angle_increment = -math.pi, math.pi / 2
        rays.range_min, rays.range_max = .02, 10.
        rays.ranges = [.8, .8, .06, .8, .8]  # rear obstruction must not stop forward following
        scan.publish(rays)
        guard.publish(String(data=json.dumps(dict(boot='test', owner=owner,
                                                  blocked=False, sent_at=time.time()))))
        motor.publish(Twist())

    node.create_timer(.05, tick)

    def wait_until(predicate, seconds=5.):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
            assert child.poll() is None, log_path.read_text()
        pytest.fail('timed out; last status: ' + str(statuses[-1:] or log_path.read_text()))

    code = '''
from roport_smartcart import doll_recognition, doll_follow_demo
class Detector:
    references = [('A', 'front', (100, 100))]
    def __init__(self, path): pass
    def detect(self, frame):
        h = int(frame[0, 0, 0])
        return [dict(label='A', view='front', quad=[[300,10],[340,10],[340,10+h],[300,10+h]])]
doll_recognition.DollRecognizer = Detector
doll_follow_demo.main(['--references', 'synthetic', '--target', 'A'])
'''
    log_path = tmp_path / 'demo.log'
    with log_path.open('w') as log:
        child = subprocess.Popen([sys.executable, '-c', code], stdout=log, stderr=log,
                                 env={**os.environ, 'ROS_DOMAIN_ID': '193',
                                      'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'})
        try:
            wait_until(lambda: statuses and statuses[-1]['readiness'] == '준비 완료')
            control.publish(String(data='calibrate'))
            wait_until(lambda: statuses[-1]['goal_range'] != '미설정')
            height, distance = 30, .6
            control.publish(String(data='start'))
            wait_until(lambda: any(v > 0 for v in commands))
            accept, owner = False, 'none'
            wait_until(lambda: not statuses[-1]['armed'] and 'ACK' in statuses[-1]['reason'])
            commands.clear()
            wait_until(lambda: len(commands) >= 3)
            assert commands and not any(commands)
            # Even a delayed grant cannot rearm a stopped demo.
            owner = 'line'
            wait_until(lambda: statuses[-1]['guard'].startswith('line'))
            assert not statuses[-1]['armed']
            assert not any(commands)
            owner = 'none'
            wait_until(lambda: statuses[-1]['guard'].startswith('none'))
            control.publish(String(data='start'))
            wait_until(lambda: statuses[-1]['reason'] == 'WAITING FOR DRIVE GUARD')
            wait_until(lambda: not statuses[-1]['armed'] and 'ACK' in statuses[-1]['reason'])
            assert not any(commands)
        finally:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            executor.shutdown()
            node.destroy_node()
            ctx.shutdown()
        assert child.returncode == 0, log_path.read_text()
