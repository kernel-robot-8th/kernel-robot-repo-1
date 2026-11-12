"""Actual Jazzy filter with synthetic sensors, isolated from Gazebo and hardware."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


def test_both_sensors_slow_stop_and_missing_sensor_holds(monkeypatch, tmp_path):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Range
    from tf2_ros import StaticTransformBroadcaster

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_STATIC_PEERS', raising=False)
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    context = Context()
    rclpy.init(context=context, domain_id=182)
    node = rclpy.create_node('sensor_filter_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    transforms = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'test_world'
    transform.child_frame_id = 'test_base'
    transform.transform.rotation.w = 1.
    transforms.sendTransform(transform)
    sonar = node.create_publisher(Range, 'us_sensor/range', qos_profile_sensor_data)
    scan = node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
    velocity = node.create_publisher(Twist, 'cmd_vel', 10)
    output = []
    node.create_subscription(Twist, 'cmd_vel_safe',
                             lambda m: output.append((m.linear.x, m.angular.z)), 10)
    config = Path(__file__).parents[1] / 'config/collision_monitor_sim.yaml'
    with (tmp_path / 'monitor.log').open('w') as log:
        child = subprocess.Popen([
            str(Path(get_package_prefix('nav2_collision_monitor')) / 'lib' /
                'nav2_collision_monitor' / 'collision_monitor'), '--ros-args',
            '--params-file', str(config), '-p', 'use_sim_time:=false',
            '-p', 'base_frame_id:=test_base', '-p', 'odom_frame_id:=test_base'],
            env={**os.environ, 'ROS_DOMAIN_ID': '182'}, stdout=log, stderr=log)
        try:
            lifecycle = node.create_client(ChangeState, '/collision_monitor/change_state')
            assert lifecycle.wait_for_service(timeout_sec=20)
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                future = lifecycle.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=5)
                assert future.done() and future.result().success

            def expect(us_distance, laser_distance, expected, *, omit_sonar=False,
                       linear=.1, angular=0., expected_angular=0.):
                output.clear()
                started = time.monotonic()
                while time.monotonic() - started < 4:
                    stamp = node.get_clock().now().to_msg()
                    if not omit_sonar:
                        r = Range()
                        r.header.stamp, r.header.frame_id = stamp, 'test_base'
                        r.radiation_type, r.field_of_view = Range.ULTRASOUND, .26
                        r.min_range, r.max_range, r.range = .02, .95, us_distance
                        sonar.publish(r)
                    s = LaserScan()
                    s.header.stamp, s.header.frame_id = stamp, 'test_base'
                    s.angle_min, s.angle_max, s.angle_increment = 0., 0., .1
                    s.range_min, s.range_max, s.ranges = .05, 8., [laser_distance]
                    scan.publish(s)
                    v = Twist()
                    v.linear.x, v.angular.z = linear, angular
                    velocity.publish(v)
                    executor.spin_once(timeout_sec=.02)
                    elapsed = time.monotonic() - started
                    if elapsed > (.8 if omit_sonar else .3) and len(output) >= 3:
                        if all(v == pytest.approx((expected, expected_angular))
                               for v in output[-3:]):
                            return
                pytest.fail(f'expected {expected}, last outputs={output[-5:]}')

            expect(.8, 2., .1)
            # A wall outside the padded rotation sweep must not trap departure.
            expect(.22, .25, 0., linear=0., angular=.5, expected_angular=.15)
            # Rotation remains blocked inside the padded body and on sensor loss.
            expect(.1, 2., 0., linear=0., angular=.5)
            expect(.8, 2., 0., linear=0., angular=.5, omit_sonar=True)
            expect(.3, 2., .03)
            expect(.1, 2., 0.)
            expect(.8, .1, 0.)
            expect(.8, 2., .1)
            expect(.8, 2., 0., omit_sonar=True)
            expect(.8, 2., .1)
        finally:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
