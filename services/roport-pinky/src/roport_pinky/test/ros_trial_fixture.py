"""Loopback-only ROS peers; synthetic localization/action server, no motor node."""
import json
import math
import os
import threading
import time
from pathlib import Path

from test_trial_driver import LAYOUT, PROFILE, Rig

PINKY = Path(__file__).resolve().parents[3]


class RosTrial:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionServer, GoalResponse
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from roport_pinky.trial_driver import TrialDriver
        from roport_pinky.trial_guard import make_node
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
        os.environ['ROS_DOMAIN_ID'] = '194'
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        self.rig = Rig()
        self.pose = {'x': 0., 'y': 0., 'yaw': 0.}
        self.accept, self.succeed, self.place_at_goal = True, True, True
        self.driver = TrialDriver(PROFILE, 'logistics', layout_file=LAYOUT,
                                  start_node='N16', map_path=PINKY / 'maps/airport_map.yaml')
        self.guard = make_node()
        self.node = rclpy.create_node('trial_test_peers')
        self.pub_state = self.node.create_publisher(String, 'line_nav/state', 10)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', 10)
        self.pub_amcl = self.node.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
        self.pub_map = self.node.create_publisher(OccupancyGrid, 'map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self.node)
        self.node.create_subscription(String, 'line_nav/command',
                                      lambda m: self.rig.send(json.loads(m.data)), 10)
        cfg, w, h, cells = self.driver.nav.reference
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width, grid.info.height = w, h
        grid.info.resolution = cfg['resolution']
        grid.info.origin.position.x, grid.info.origin.position.y = cfg['origin'][:2]
        grid.info.origin.orientation.w = math.cos(cfg['origin'][2] / 2)
        grid.info.origin.orientation.z = math.sin(cfg['origin'][2] / 2)
        grid.data = cells
        self.grid = grid
        self.pub_map.publish(grid)

        def active(req, res):
            res.current_state.id = 3
            return res
        self.services = [self.node.create_service(GetState, n + '/get_state', active)
                         for n in ('amcl', 'bt_navigator')]

        def execute(handle):
            if self.place_at_goal:
                p = handle.request.pose.pose
                self.pose = {'x': p.position.x, 'y': p.position.y,
                                 'yaw': 2 * math.atan2(p.orientation.z, p.orientation.w)}
            if self.succeed:
                handle.succeed()
            else:
                handle.abort()
            return NavigateToPose.Result()
        self.action = ActionServer(self.node, NavigateToPose, 'navigate_to_pose', execute,
            goal_callback=lambda _: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT)

        def tick():
            for _ in range(10):
                self.rig.sim.tick(self.rig.ex)
            st = self.rig.ex.as_dict()
            st.update(graph_version=self.rig.graph.version, last_command=self.rig.last)
            self.pub_state.publish(String(data=json.dumps(st)))
            stamp = self.node.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.rig.sim.cmd
            self.pub_odom.publish(odom)
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'map', 'base_footprint', stamp
            tf.transform.translation.x, tf.transform.translation.y = self.pose['x'], self.pose['y']
            tf.transform.rotation.w = math.cos(self.pose['yaw'] / 2)
            tf.transform.rotation.z = math.sin(self.pose['yaw'] / 2)
            self.tf.sendTransform(tf)
            amcl = PoseWithCovarianceStamped()
            amcl.header.frame_id, amcl.header.stamp = 'map', stamp
            amcl.pose.pose.position.x, amcl.pose.pose.position.y = self.pose['x'], self.pose['y']
            amcl.pose.pose.orientation = tf.transform.rotation
            amcl.pose.covariance[0] = amcl.pose.covariance[7] = amcl.pose.covariance[35] = .001
            self.pub_amcl.publish(amcl)
        self.timer = self.node.create_timer(.02, tick)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.ex.add_node(self.guard)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        self.wait(lambda: self.driver.observe()['fault'] is None
                  and self.driver.observe()['nav2_ready'])

    def wait(self, condition, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.driver.heartbeat()
            if condition():
                return
            time.sleep(.02)
        raise AssertionError(self.driver.observe())

    def run(self, command):
        results = []
        self.driver.validate_action(command)
        self.driver.dispatch(command)
        def done():
            results.extend(self.driver.drain_results())
            return any(p in ('completed', 'failed') for _, p, _ in results)
        self.wait(done)
        return results

    def close(self):
        self.driver.safe_stop('test_closed')
        self.ex.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.action.destroy()
        self.guard.destroy_node()
        self.node.destroy_node()
        self.driver.close()
