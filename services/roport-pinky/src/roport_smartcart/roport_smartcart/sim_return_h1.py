"""Gazebo-only H1 return: Nav2 approach, N12 acquisition, shared line executor.

Ideal ground-truth alignment is a simulation fixture, not an ArUco or real IR
implementation. Explicit invocation only; never subscribes to MQTT missions.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def return_waypoints(position, n12):
    # Resume near the handoff without turning back toward the entrance wall.
    if math.hypot(position[0]-n12[0], position[1]-n12[1]) < .4 and abs(position[1]-n12[1]) < .15:
        return []
    return ['return_entry']


def ir_values(lines, pose, calibration):
    """Nine-point IR footprint on generated world tape boxes, metres/radians."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    values = []
    for i, lateral in enumerate((.02, 0., -.02)):
        px, py = x + .03*c - lateral*s, y + .03*s + lateral*c
        covered = 0
        for j in range(9):
            a = j*math.pi/4
            qx = px + (.004*math.cos(a) if j < 8 else 0.)
            qy = py + (.004*math.sin(a) if j < 8 else 0.)
            for box in lines:
                dx, dy = qx-box.cx, qy-box.cy
                bx = math.cos(box.yaw)*dx + math.sin(box.yaw)*dy
                by = -math.sin(box.yaw)*dx + math.cos(box.yaw)*dy
                if abs(bx) <= box.sx/2 and abs(by) <= box.sy/2:
                    covered += 1
                    break
        carpet, tape = calibration.carpet[i], calibration.tape[i]
        values.append(round(carpet-covered/9*(carpet-tape)))
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gate', choices=('gate1', 'gate2', 'gate3'))
    options = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '198':
        parser.error('Gazebo domain 198 only; use scripts/smartcart_sim.sh')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    for name in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER'):
        os.environ.pop(name, None)

    import yaml
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from roport_gz_world.layout import Layout
    from roport_line_nav.executor import NavConfig, RouteExecutor
    from roport_line_nav.graph import Graph
    from roport_pinky_line.calibration import load_calibration
    from roport_pinky_line.line_sensor import LineSensor
    from sensor_msgs.msg import LaserScan, Range
    from tf2_msgs.msg import TFMessage

    smart = Path(get_package_share_directory('roport_smartcart'))
    world = Path(get_package_share_directory('roport_gz_world'))
    layout_path = world/'layout/roport_airport.yaml'
    layout = Layout(yaml.safe_load(layout_path.read_text()))
    calibration = load_calibration(
        str(Path(get_package_share_directory('roport_pinky_line'))/'config/robots.yaml'),
        'pinky_d660')
    graph = Graph.load(layout_path)
    rclpy.init(domain_id=198, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sim_return_h1', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    samples, received = {}, {}

    def observe(name, message):
        previous = samples.get(name)
        stamp = message.header.stamp
        if previous is not None:
            old = previous.header.stamp
            if (stamp.sec, stamp.nanosec) <= (old.sec, old.nanosec):
                return
        samples[name], received[name] = message, time.monotonic()

    subscriptions = [node.create_subscription(kind, topic,
        lambda m, k=k: observe(k, m), qos_profile_sensor_data)
        for k, kind, topic in [('odom', Odometry, 'odom'), ('scan', LaserScan, 'scan'),
                              ('range', Range, 'us_sensor/range')]]
    def truth(message):
        matches = [t for t in message.transforms
                   if t.child_frame_id == 'pinky' and t.header.frame_id == 'roport_airport']
        if len(matches) == 1:
            observe('ground_truth', matches[0])

    subscriptions.append(node.create_subscription(
        TFMessage, 'sim/gazebo_poses', truth, qos_profile_sensor_data))
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)

    def fresh():
        now = time.monotonic()
        ros = node.get_clock().now().nanoseconds/1e9
        for name in ('odom', 'scan', 'range', 'ground_truth'):
            msg = samples.get(name)
            if msg is None or now-received[name] > .5:
                raise RuntimeError(f'{name} missing or stale')
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            if not -.1 <= ros-stamp <= .5:
                raise RuntimeError(f'{name} ROS timestamp stale')

    def pose():
        p = samples['ground_truth'].transform
        q = p.rotation
        values = (p.translation.x, p.translation.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('nonfinite Gazebo pose')
        return (p.translation.x, p.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def command(v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        publisher.publish(msg)

    def interrupted(_signal, _frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    child = None
    try:
        deadline = time.monotonic()+30
        while True:
            spin(.1)
            try:
                fresh()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise

        h1 = graph.nodes['H1']
        h1_world = layout.frame.to_world(h1.x, h1.y)
        current = pose()
        on_bar = LineSensor(calibration).update(ir_values(layout.lines, current, calibration)).node
        if (not options.gate and on_bar
                and math.hypot(current[0]-h1_world[0], current[1]-h1_world[1]) < .06
                and abs(wrap(current[2]-math.pi/2-math.radians(layout.frame.yaw_deg))) < .25):
            node.get_logger().info('H1 already parked; holding position')
            return
        if options.gate:
            measured = layout.frame.to_measured(*pose()[:2])
            if measured[0] < 138 or measured[1] < 94:
                raise RuntimeError('Gate demo must start inside the public Nav2 area; '
                                   'H1 outbound line handoff is not provided by this return demo')
        # Reuse the map/localization/readiness checks of the existing Nav2 client.
        n12 = graph.nodes['N12']
        n12_world = layout.frame.to_world(n12.x+3., n12.y)
        targets = ([options.gate, 'return_entry'] if options.gate else
                   return_waypoints(pose()[:2], n12_world))
        for target in targets:
            node.get_logger().info(f'Nav2 -> {target}')
            child = subprocess.Popen(['ros2', 'run', 'roport_smartcart', 'navigate',
                '--mode', 'sim', '--domain-id', '198', '--places',
                str(smart/'config/places_sim.yaml'), '--map', str(world/'maps/airport_map.yaml'),
                '--target', target, '--startup-wait', '30', '--timeout', '150'],
                start_new_session=True)
            while child.poll() is None:
                spin(.1)
            if child.returncode:
                raise RuntimeError(f'Nav2 {target} failed ({child.returncode})')
            child = None
        spin(1.)
        fresh()
        twist = samples['odom'].twist.twist
        if abs(twist.linear.x) > .01 or abs(twist.angular.z) > .01:
            raise RuntimeError('Nav2 stop not confirmed before line ownership')

        # Sim-only ideal docking fixture: align actual Gazebo pose to the tape.
        # Upgrade to measured IR/marker acquisition before physical deployment.
        n12 = graph.nodes['N12']
        target = layout.frame.to_world(n12.x+3., n12.y)
        heading = wrap(math.pi+math.radians(layout.frame.yaw_deg))
        if math.hypot(target[0]-pose()[0], target[1]-pose()[1]) > .45:
            raise RuntimeError('too far from N12 for simulation line acquisition')
        end = time.monotonic()+40
        while time.monotonic() < end:
            fresh()
            x, y, yaw = pose()
            dx, dy = target[0]-x, target[1]-y
            distance = math.hypot(dx, dy)
            angle = wrap((math.atan2(dy, dx) if distance > .004 else heading)-yaw)
            if distance <= .004 and abs(angle) < .025:
                break
            command(min(.03, distance) if distance > .004 and abs(angle) < .2 else 0.,
                    max(-.3, min(.3, 1.5*angle)))
            spin(.05)
        else:
            raise RuntimeError('N12 simulation alignment timed out')
        command()
        spin(.3)
        sensor = LineSensor(calibration)
        route = RouteExecutor(graph, NavConfig(base_speed=.06, rotate_speed=.45))
        route.start(['N12', 'N08', 'N09', 'H1'])
        node.get_logger().info('LINE -> N12 -> N08 -> N09 -> H1 (0.06 m/s, geometric IR)')
        end = time.monotonic()+150
        previous = None
        while time.monotonic() < end:
            fresh()
            x, y, yaw = pose()
            reading = sensor.update(ir_values(layout.lines, (x, y, yaw), calibration))
            odom = samples['odom'].pose.pose
            q = odom.orientation
            odom_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            output = route.step(reading, (odom.position.x*100., odom.position.y*100., odom_yaw),
                                node.get_clock().now().nanoseconds/1e9)
            if output is not None:
                command(output.linear, output.angular)
            state = (route.state, route.index)
            if state != previous:
                node.get_logger().info(f'line state={route.state}, leg={route.index}, {route.reason}')
                previous = state
            if route.state == 'arrived':
                node.get_logger().info(f'H1 ARRIVED: world=({x:.3f}, {y:.3f}), yaw={yaw:.3f}')
                return
            if route.state in ('error', 'stopped'):
                raise RuntimeError(f'H1 return stopped: {route.reason}')
            spin(.05)
        raise RuntimeError('H1 line return timed out')
    except KeyboardInterrupt:
        node.get_logger().warning('H1 return canceled')
        return 130
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        for _ in range(10):
            command()
            spin(.05)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
