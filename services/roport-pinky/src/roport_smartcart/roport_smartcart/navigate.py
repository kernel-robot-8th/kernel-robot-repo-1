"""Explicit one-shot Nav2 client. No bringup, initial pose or raw velocity commands."""

import argparse
import json
import math
import os
import time


from .goals import load_destination
from .live_map import LiveMapCheck, reference_from_file


class Readiness:
    """Receipt and ROS timestamps must both be fresh; stop evidence is continuous."""

    def __init__(self, mode, clock=time.monotonic):
        self.mode, self.clock = mode, clock
        self.samples = {}
        self.stopped_since = None
        self.ros_time = None
        self.clock_changed = None
        self.clock_advances = 0
        self.amcl_stamp = None

    def tick(self, ros_time):
        if self.ros_time is not None and ros_time < self.ros_time:
            self.samples.clear()
            self.stopped_since = None
            self.clock_advances = 0
            self.amcl_stamp = None
        if self.ros_time is not None and ros_time > self.ros_time:
            self.clock_changed = self.clock()
            self.clock_advances += 1
        self.ros_time = ros_time

    def observe(self, name, stamp, *, stopped=None):
        now = self.clock()
        previous = self.samples.get(name)
        if not math.isfinite(stamp) or (previous and stamp <= previous[0]):
            return False
        if name == 'odom':
            if not stopped or (previous and now - previous[1] > 1.):
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = now
        self.samples[name] = stamp, now
        return True

    def observe_amcl(self, message):
        """A bad latest estimate immediately revokes any previous good evidence.

        Variance limits match trial_driver: x/y <= .04 m², yaw <= .1 rad².
        """
        pose = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if not math.isfinite(stamp) or (self.amcl_stamp is not None and stamp <= self.amcl_stamp):
            self.samples.pop('amcl', None)
            return False
        self.amcl_stamp = stamp
        q = pose.orientation
        covariance = message.pose.covariance
        # Identical Gazebo particles can produce negative variance at roundoff scale.
        min_variance = -1.e-12 if self.mode == 'sim' else 0.
        values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
        valid = (message.header.frame_id == 'map'
                 and all(math.isfinite(v) for v in values)
                 and abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.) <= 1e-3
                 and len(covariance) == 36
                 and all(math.isfinite(v) for v in covariance)
                 and all(covariance[i] >= min_variance for i in (0, 7, 14, 21, 28, 35))
                 and all(covariance[i] <= limit for i, limit in ((0, .04), (7, .04), (35, .1))))
        if not valid or not self.observe('amcl', stamp):
            self.samples.pop('amcl', None)
            return False
        return True

    def failures(self, *, require_stopped=True):
        """Explain the exact readiness rejection without changing its thresholds."""
        now = self.clock()
        failures = []
        if self.ros_time is None or self.ros_time <= 0:
            return ['ROS clock missing']
        if self.mode == 'sim' and (self.clock_advances < 2 or
                self.clock_changed is None or now - self.clock_changed > 1.):
            failures.append('simulation clock not advancing')
        for name in ('scan', 'odom', 'amcl', 'tf'):
            sample = self.samples.get(name)
            if sample is None:
                failures.append(f'{name}: missing observation')
                continue
            age = 5. if name == 'amcl' else 1.
            stamp_age = 5. if name == 'amcl' else 2.
            received_age, measured_age = now - sample[1], self.ros_time - sample[0]
            if received_age > age or not -.5 <= measured_age <= stamp_age:
                failures.append(f'{name}: receive_age={received_age:.3f}s '
                                f'stamp_age={measured_age:.3f}s')
        if require_stopped and (self.stopped_since is None or now - self.stopped_since < .5):
            failures.append('continuous odom stop not confirmed')
        return failures

    def ready(self, *, require_stopped=True):
        return not self.failures(require_stopped=require_stopped)


class NomotionRefresh:
    """Force the next laser observation update, including while blocked or moving.

    AMCL still applies its odometry model; this never resets the estimated pose.
    """

    def __init__(self, client, request_factory, clock=time.monotonic):
        self.client, self.request_factory, self.clock = client, request_factory, clock
        self.pending = None
        self.last_request = -math.inf
        self.error = 'AMCL request_nomotion_update service unavailable'

    def poll(self):
        if self.pending is not None:
            if not self.pending.done():
                return False
            try:
                self.pending.result()
            except Exception:
                self.error = 'AMCL observation request failed'
            self.pending = None
        now = self.clock()
        if now - self.last_request < 1. or not self.client.service_is_ready():
            return False
        self.last_request = now
        try:
            self.pending = self.client.call_async(self.request_factory())
            self.error = 'AMCL observation requested; waiting for fresh pose'
            return True
        except Exception:
            self.error = 'AMCL observation request transport failed'
            return False


def validate_goal(places, map_yaml, target, mode):
    return load_destination(places, map_yaml, target, environment=mode)


def goal_error(goal, transform):
    """Localization-based diagnostics, not a physical accuracy measurement."""
    pose = transform.transform
    q = pose.rotation
    yaw = math.atan2(2. * (q.w * q.z + q.x * q.y),
                     1. - 2. * (q.y * q.y + q.z * q.z))
    delta = yaw - goal.yaw
    return {'position_error_m': math.hypot(pose.translation.x - goal.x,
                                          pose.translation.y - goal.y),
            'yaw_error_rad': abs(math.atan2(math.sin(delta), math.cos(delta)))}


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--mode', choices=('sim', 'real'), required=True)
    result.add_argument('--domain-id', type=int, help='sim defaults to 198; real requires explicit domain')
    result.add_argument('--check-only', action='store_true', help='validate target configuration without ROS')
    result.add_argument('--places', required=True)
    result.add_argument('--map', required=True)
    result.add_argument('--target', required=True)
    result.add_argument('--return-to')
    result.add_argument('--timeout', type=float, default=120.)
    result.add_argument('--startup-wait', type=float, default=20.)
    return result


def main(args=None):
    argument_parser = parser()
    options = argument_parser.parse_args(args)
    if options.mode == 'real' and options.domain_id is None and not options.check_only:
        argument_parser.error('real mode requires explicit --domain-id')
    domain = 198 if options.domain_id is None else options.domain_id
    if not 0 <= domain <= 232:
        argument_parser.error('--domain-id must be in 0..232')
    if not all(math.isfinite(v) and v > 0 for v in (options.timeout, options.startup_wait)):
        raise ValueError('timeouts must be positive and finite')
    targets = [options.target] + ([options.return_to] if options.return_to else [])
    for target in targets:
        validate_goal(options.places, options.map, target, options.mode)
    if options.check_only:
        print(json.dumps({'mode': options.mode, 'targets': targets, 'configuration_valid': True,
                          'hardware_checked': False}), flush=True)
        return 0

    reference = reference_from_file(options.map)
    if options.mode == 'sim':
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from nav_msgs.srv import GetMap
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformException, TransformListener
    from .nav_client import NavClient

    rclpy.init(domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('smartcart_navigate', parameter_overrides=[
        Parameter('use_sim_time', value=options.mode == 'sim')])
    readiness = Readiness(options.mode)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    client = NavClient(node, action_name='navigate_to_pose', timeout_seconds=options.timeout,
                       environment=options.mode)

    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def odom(message):
        twist = message.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        stopped = all(math.isfinite(v) and abs(v) <= .01 for v in values)
        readiness.observe('odom', stamp(message), stopped=stopped)

    def scan(message):
        if any(math.isfinite(v) and message.range_min <= v <= message.range_max
               for v in message.ranges):
            readiness.observe('scan', stamp(message))

    live_map = LiveMapCheck(node.create_client(GetMap, 'map_server/map'), GetMap.Request,
                            reference)

    refresh = NomotionRefresh(node.create_client(Empty, 'request_nomotion_update'), Empty.Request)
    subscriptions = [
        node.create_subscription(Odometry, 'odom', odom, qos_profile_sensor_data),
        node.create_subscription(LaserScan, 'scan', scan, qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', readiness.observe_amcl, qos_profile_sensor_data),
    ]

    def step():
        rclpy.spin_once(node, timeout_sec=.05)
        readiness.tick(node.get_clock().now().nanoseconds / 1e9)
        try:
            transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            readiness.observe('tf', stamp(transform))
        except TransformException:
            pass
        client.poll()
        live_map.poll()

    def failures(*, require_stopped=True):
        return readiness.failures(require_stopped=require_stopped) + live_map.failures()

    def await_ready():
        live_map.recheck()
        deadline = time.monotonic() + options.startup_wait
        while time.monotonic() < deadline:
            refresh.poll()
            step()
            if not failures() and client.server_is_ready():
                return
        pending = failures() + ([] if client.server_is_ready() else ['Nav2 action server unavailable'])
        raise RuntimeError('readiness failed: ' + '; '.join(pending)
                           + '; ' + refresh.error)

    def cancel_and_hold():
        client.cancel()
        warned = 0.
        while client.lifecycle.busy:
            try:
                step()
            except KeyboardInterrupt:
                pass
            if time.monotonic() - warned > 5.:
                node.get_logger().error('Cancel pending: retaining ownership until terminal result; check robot locally')
                warned = time.monotonic()

    exit_code = 0
    try:
        expected_sim = options.mode == 'sim'
        if node.get_parameter('use_sim_time').value is not expected_sim:
            raise RuntimeError('node clock source does not match --mode')
        for target in targets:
            await_ready()
            validate_goal(options.places, options.map, target, options.mode)
            client.start(options.places, options.map, target)
            readiness_failures = []
            while client.lifecycle.busy:
                # AMCL may otherwise stop publishing while Nav2 waits at an obstacle.
                refresh.poll()
                step()
                rejected = failures(require_stopped=False)
                if rejected:
                    if not readiness_failures:
                        readiness_failures = rejected
                        node.get_logger().error('Canceling goal: ' + '; '.join(rejected))
                    client.cancel()
                if client.lifecycle.fault:
                    cancel_and_hold()
            await_ready()
            goal = validate_goal(options.places, options.map, target, options.mode)
            error = goal_error(goal, buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()))
            print(json.dumps({'target': target, **error, 'outcome': client.lifecycle.outcome,
                              'fault': client.lifecycle.fault, 'readiness_failures': readiness_failures,
                              'stopped': True}), flush=True)
            if readiness_failures or client.lifecycle.outcome != 'succeeded' or client.lifecycle.fault:
                exit_code = 1
                break
    except KeyboardInterrupt:
        cancel_and_hold()
        exit_code = 130
    except Exception as exc:
        cancel_and_hold()
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        client._client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code
