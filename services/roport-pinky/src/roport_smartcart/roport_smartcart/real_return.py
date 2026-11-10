"""Supervised hardware return: Nav2 to N12, real IR, shared line executor.

The caller supplies the reserved bay. This entry point does not infer customer
receipt or reserve a bay, and never falls back to Nav2 travel inside the line area.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import threading
import time
from pathlib import Path
from uuid import uuid4

import yaml

from .goals import Destination, load_destination


def line_route(bay):
    if bay == 'H2':
        return ['N12', 'N08', 'H2']
    if bay == 'H1':
        return ['N12', 'N08', 'N09', 'H1']
    raise ValueError('A reserved H1 or H2 bay is required')


def angle_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def n12_target(layout, graph, ir_ahead_cm=3.):
    n = graph.nodes['N12']
    x, y = layout.frame.to_world(n.x+ir_ahead_cm, n.y)
    heading = math.pi + math.radians(layout.frame.yaw_deg)
    return Destination(x, y, math.atan2(math.sin(heading), math.cos(heading)))


def measured_handoff(goal, nominal):
    # Accept a field correction within the existing acquisition envelope;
    # physical IR, pose and standstill remain mandatory during the handoff.
    if (math.hypot(goal.x-nominal.x, goal.y-nominal.y) > .05
            or angle_error(goal.yaw, nominal.yaw) > .1):
        raise ValueError('Measured N12 handoff is inconsistent with the surveyed graph')
    return goal


def validate_entry(entry, target):
    dx, dy = entry.x-target.x, entry.y-target.y
    # Enter from the public-space side of N12, never navigate to a parking bay.
    forward = dx*math.cos(target.yaw)+dy*math.sin(target.yaw)
    if not .05 <= math.hypot(dx, dy) <= .5 or forward > -.05:
        raise ValueError('return_entry must be on the public-space approach to N12')


def aligned(pose, target, reading, age):
    """A map pose alone cannot establish acquisition of the physical tape."""
    return bool(pose and reading is not None and 0 <= age <= .3
                and all(math.isfinite(pose[k]) for k in ('x', 'y', 'yaw'))
                and math.hypot(pose['x']-target.x, pose['y']-target.y) <= .05
                and angle_error(pose['yaw'], target.yaw) <= .1
                and not reading.lost and reading.on_line[1]
                and math.isfinite(reading.error) and abs(reading.error) <= .25)


class Acquisition:
    """Require a continuous half second of *new* IR samples while stopped."""
    def __init__(self):
        self.since = None
        self.last_sample = None

    def update(self, *, now, sample_time, good, stopped):
        if not good or not stopped:
            self.since = None
            self.last_sample = None
            return False
        if self.last_sample is not None and sample_time-self.last_sample > .3:
            self.since = None
        if self.last_sample is None or sample_time > self.last_sample:
            self.last_sample = sample_time
            if self.since is None:
                self.since = now
            return now-self.since >= .5
        return False


class HardwareReturn:
    def __init__(self, node, options, graph, calibration, target, profile):
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String, UInt16MultiArray
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.parameter_client import AsyncParameterClient
        from roport_pinky.trial_nav2 import Nav2Bridge
        from roport_pinky_line.line_sensor import LineSensor
        from roport_line_nav.executor import NavConfig, RouteExecutor
        from .nav_client import NavClient
        self.node, self.options, self.target = node, options, target
        self.goal_parameters = AsyncParameterClient(node, 'controller_server')
        self.saved_goal_tolerance = None
        self.bridge = Nav2Bridge(node, profile, options.map, threading.RLock())
        self.nav = NavClient(node, action_name='navigate_to_pose',
                             timeout_seconds=300, environment='real')
        self.line = RouteExecutor(graph, NavConfig(base_speed=options.line_speed,
            max_linear=options.line_speed, rotate_speed=.15, max_angular=.15))
        self.sensor = LineSensor(calibration)
        self.reading = None
        self.ir_at = None
        self.ir_seq = 0
        self.guard, self.guard_at, self.boot = {}, 0., None
        self.seq, self.session = 0, uuid4().hex
        self.phase, self.owner = 'READY', 'none'
        self.Twist, self.String = Twist, String
        self.velocity = node.create_publisher(Twist, 'line_nav/cmd_vel', 1)
        self.lease_pub = node.create_publisher(String, 'trial/lease', 1)
        self.status = node.create_publisher(String, 'smartcart/return_state', 1)
        node.create_subscription(String, 'trial/guard_state', self.on_guard, 10)
        node.create_subscription(UInt16MultiArray, 'ir_sensor/range', self.on_ir,
                                 qos_profile_sensor_data)
        self.last_status = 0.

    def on_guard(self, msg):
        self.guard = json.loads(msg.data)
        self.guard_at = time.monotonic()

    def on_ir(self, msg):
        try:
            reading = self.sensor.update(msg.data)
        except ValueError:
            # Invalid ADC values must not extend the valid-sample watchdog.
            self.reading = None
            return
        self.reading, self.ir_at = reading, time.monotonic()
        self.ir_seq += 1

    def guard_ok(self):
        return (self.guard.get('boot') == self.boot and self.boot is not None
                and not self.guard.get('blocked')
                and time.monotonic()-self.guard_at < .3)

    def lease(self, owner):
        if self.boot is not None:
            self.seq += 1
            self.lease_pub.publish(self.String(data=json.dumps(dict(
                boot=self.boot, owner=owner, seq=self.seq, session=self.session,
                sent_at=time.time()))))

    def spin(self):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=.01)
        self.nav.poll()
        self.lease(self.owner if self.guard_ok() else 'none')
        if time.monotonic()-self.last_status > .1:
            self.last_status = time.monotonic()
            self.status.publish(self.String(data=json.dumps({
                'phase': self.phase, 'bay': self.options.bay, 'drive_owner': self.owner,
                'line_route': self.line.route, 'line_state': self.line.state,
                'line_index': self.line.index, 'position_source':
                'IR+odometry' if self.phase == 'LINE' else 'AMCL+TF',
                'observed_at': time.time()})))
        return self.bridge.snapshot()

    def require_guard(self):
        if not self.guard_ok():
            self.owner = 'none'
            self.lease('none')
            raise RuntimeError('Drive guard stale, blocked or restarted')

    def stop_confirmed(self, timeout=10):
        since = None
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            state = self.spin()
            now = time.monotonic()
            since = (now if since is None else since) if state['stopped'] else None
            if since is not None and now-since >= .5:
                return state
        raise RuntimeError('Actual odometry stop not confirmed')

    def parameter_result(self, future):
        end = time.monotonic()+5
        while not future.done():
            self.spin()
            self.require_guard()
            if time.monotonic() >= end:
                raise RuntimeError('Goal tolerance parameter service timeout')
        try:
            result = future.result()
        except Exception as exc:
            raise RuntimeError('Goal tolerance parameter service failed') from exc
        if result is None:
            raise RuntimeError('Goal tolerance parameter service returned no result')
        return result

    def set_goal_tolerance(self, value):
        from rclpy.parameter import Parameter
        result = self.parameter_result(self.goal_parameters.set_parameters([
            Parameter('general_goal_checker.xy_goal_tolerance', value=value)]))
        if len(result.results) != 1 or not result.results[0].successful:
            raise RuntimeError('Goal tolerance parameter update rejected')

    def restore_goal_tolerance(self):
        if self.saved_goal_tolerance is not None:
            self.set_goal_tolerance(self.saved_goal_tolerance)
            self.saved_goal_tolerance = None

    def navigate(self, place):
        self.require_guard()
        if place == 'return_n12':
            result = self.parameter_result(self.goal_parameters.get_parameters([
                'general_goal_checker.xy_goal_tolerance']))
            if (len(result.values) != 1 or result.values[0].type != 3
                    or not math.isfinite(result.values[0].double_value)
                    or result.values[0].double_value <= 0):
                raise RuntimeError('Invalid original goal tolerance')
            self.saved_goal_tolerance = result.values[0].double_value
            self.set_goal_tolerance(.01)
        self.phase = 'NAV2_'+place.upper()
        self.nav.start(self.options.places, self.options.map, place)
        while self.nav.lifecycle.busy:
            state = self.spin()
            self.require_guard()
            if not state['nav2_ready'] or not state['localization_ready'] or self.nav.lifecycle.fault:
                raise RuntimeError('Nav2/localization readiness lost')
            self.owner = 'nav2' if self.nav.lifecycle.state == 'active' else 'none'
        self.owner = 'none'
        self.lease('none')
        if self.nav.lifecycle.outcome != 'succeeded':
            raise RuntimeError('Nav2 goal failed: '+str(self.nav.lifecycle.outcome))
        state = self.stop_confirmed()
        goal = load_destination(self.options.places, self.options.map, place, environment='real')
        p = state['pose']
        if (not state['localization_ready'] or not p
                or math.hypot(p['x']-goal.x, p['y']-goal.y) > .08
                or angle_error(p['yaw'], goal.yaw) > .1):
            raise RuntimeError('Nav2 terminal pose outside tolerance')
        self.restore_goal_tolerance()

    def run(self):
        self.phase = 'READY'
        end = time.monotonic()+30
        while time.monotonic() < end:
            state = self.spin()
            if self.boot is None and self.guard:
                self.boot = self.guard.get('boot')
            if (self.guard_ok() and state['nav2_ready'] and state['localization_ready']
                    and state['stopped'] and self.ir_at is not None and self.reading is not None
                    and time.monotonic()-self.ir_at < .3):
                break
        else:
            raise RuntimeError('Nav2, physical IR, localization and stopped state required')
        for topic, name in [('cmd_vel', 'trial_guard'), ('trial/lease', self.node.get_name()),
                            ('line_nav/cmd_vel', self.node.get_name())]:
            pubs = self.node.get_publishers_info_by_topic(topic)
            if len(pubs) != 1 or pubs[0].node_name != name:
                raise RuntimeError('Conflicting or missing publisher: '+topic)
        if self.guard.get('owner') != 'none':
            raise RuntimeError('Drive already owned')
        # The coordinator is a return command, never a Nav2 departure from a bay.
        from roport_gz_world.layout import Layout
        layout = Layout(yaml.safe_load(Path(self.options.layout).read_text()))
        p = state['pose']
        x, y = layout.frame.to_measured(p['x'], p['y'])
        walls = {w['name']: w for w in layout.raw['walls']}
        from_n12 = getattr(self.options, 'from_n12', False)
        if not from_n12 and (x < walls['zone03_left']['from'][0]
                             or y < walls['zone03_bottom']['from'][1]):
            raise RuntimeError('Return must start in public space; do not Nav2 out of H1/H2')
        self.stop_confirmed()
        if not from_n12:
            self.navigate('return_entry')
            self.navigate('return_n12')
        self.phase = 'ACQUIRE_IR'
        acquire = Acquisition()
        end = time.monotonic()+10
        while time.monotonic() < end:
            state = self.spin()
            self.require_guard()
            now = time.monotonic()
            age = math.inf if self.ir_at is None else now-self.ir_at
            good = state['localization_ready'] and aligned(state['pose'], self.target, self.reading, age)
            if acquire.update(now=now, sample_time=self.ir_at, good=good,
                              stopped=state['stopped'] and self.guard.get('owner') == 'none'):
                break
        else:
            raise RuntimeError('N12 physical tape not acquired; staying stopped')
        self.follow_line(line_route(self.options.bay))

    def follow_line(self, route):
        self.require_guard()
        self.stop_confirmed()
        self.line.start(route, start_offset_cm=0.)
        self.phase = 'LINE'
        previous = self.ir_seq
        end = time.monotonic()+150
        while time.monotonic() < end:
            self.spin()
            self.require_guard()
            now = time.monotonic()
            odom = self.bridge.odom
            if self.ir_at is None or now-self.ir_at > .3 or self.reading is None:
                raise RuntimeError('Physical IR unavailable')
            stamp = odom.header.stamp if odom else None
            ros_now = self.node.get_clock().now().nanoseconds/1e9
            if stamp is None or not -.1 <= ros_now-stamp.sec-stamp.nanosec/1e9 <= .5:
                raise RuntimeError('Odometry unavailable')
            if previous == self.ir_seq:
                continue
            previous = self.ir_seq
            from roport_pinky.trial_nav2 import yaw
            p = odom.pose.pose
            command = self.line.step(self.reading,
                (p.position.x*100., p.position.y*100., yaw(p.orientation)), now)
            terminal = 'handoff' if route[-1] == 'N12' else 'arrived'
            if self.line.state == terminal:
                self.owner = 'none'
                self.lease('none')
                self.stop_confirmed()
                self.phase = 'AT_N12' if terminal == 'handoff' else 'ARRIVED'
                self.spin()
                return
            if self.line.state in ('error', 'stopped'):
                raise RuntimeError('Line return failed: '+self.line.reason)
            if command is not None:
                self.owner = 'line'
                self.lease('line')
                msg = self.Twist()
                msg.linear.x, msg.angular.z = command.linear, command.angular
                self.velocity.publish(msg)
        raise RuntimeError('Line return timed out')

    def close(self):
        self.owner = 'none'
        self.lease('none')
        self.line.stop('return shutdown')
        self.velocity.publish(self.Twist())
        self.nav.cancel()
        # Retain the action until a terminal result; continue sending lease NONE.
        while self.nav.lifecycle.busy:
            self.spin()
        self.stop_confirmed()
        self.restore_goal_tolerance()
        for _ in range(10):
            self.spin()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='Explicitly authorize physical motion')
    parser.add_argument('--from-n12', action='store_true',
                        help='Test N12 handoff and line parking only; retain real pose/IR checks')
    for flag in ('places', 'map', 'layout', 'robot', 'calibration'):
        parser.add_argument('--'+flag, required=True)
    parser.add_argument('--bay', choices=('H1', 'H2'), required=True,
                        help='Bay already reserved by the caller/control server')
    parser.add_argument('--line-speed', type=float, choices=(.03, .06), default=.03)
    parser.add_argument('--domain-id', type=int, required=True,
                        help='Hardware ROS domain; must match ROS_DOMAIN_ID')
    options = parser.parse_args(argv)
    if not 0 <= options.domain_id <= 232 or os.environ.get('ROS_DOMAIN_ID') != str(options.domain_id):
        parser.error('--domain-id must be in 0..232 and match ROS_DOMAIN_ID')
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph, plan_route
    from roport_pinky_line.calibration import load_calibration
    layout = Layout(yaml.safe_load(Path(options.layout).read_text()))
    graph = Graph.load(options.layout)
    plan_route(graph, line_route(options.bay))
    calibration = load_calibration(options.calibration, options.robot)
    nominal = n12_target(layout, graph)
    target = measured_handoff(load_destination(options.places, options.map, 'return_n12',
                                               environment='real'), nominal)
    for place in (('return_n12',) if options.from_n12 else ('return_entry', 'return_n12')):
        goal = load_destination(options.places, options.map, place, environment='real')
        if place == 'return_entry':
            validate_entry(goal, target)
    if not options.drive:
        print('Hardware return configuration validated; no ROS node or motion initialized')
        return 0
    from roport_pinky.trial_nav2 import map_reference
    path = Path(options.map)
    cfg = yaml.safe_load(path.read_text())
    profile = {'map_id': path.stem, 'map_version': hashlib.sha256(
        path.read_bytes()+(path.parent/cfg['image']).read_bytes()).hexdigest()[:16]}
    map_reference(path, profile)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('smartcart_real_return')
    runner = None
    def interrupt(*_):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    result = 1
    try:
        runner = HardwareReturn(node, options, graph, calibration, target, profile)
        runner.run()
        result = 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        if runner:
            runner.phase = 'STOPPED'
        node.get_logger().error('Return stopped: '+str(exc))
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        try:
            if runner:
                runner.close()
        except RuntimeError as exc:
            result = 1
            node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
    return result
