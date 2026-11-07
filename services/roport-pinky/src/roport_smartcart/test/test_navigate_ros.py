"""Real CLI against isolated synthetic ROS peers; never publish motor commands."""

import os
import subprocess
import threading
import time

import pytest
import yaml

from roport_smartcart.goals import map_hashes


class NavigationPeer:
    def __init__(self, scenario):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from nav_msgs.srv import GetMap
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Empty
        from tf2_ros import TransformBroadcaster

        self.scenario = scenario
        self.goals = []
        self.cancellations = 0
        self.shutdown_requested = threading.Event()
        self.context = Context()
        rclpy.init(context=self.context, domain_id=192)
        self.node = rclpy.create_node('synthetic_navigation_peer', context=self.context)
        group = ReentrantCallbackGroup()
        self.tf = TransformBroadcaster(self.node)
        self.scan = self.node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.odom = self.node.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.amcl = self.node.create_publisher(PoseWithCovarianceStamped, 'amcl_pose',
                                                qos_profile_sensor_data)

        def get_map(request, response):
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.header.stamp = self.node.get_clock().now().to_msg()
            grid.info.resolution = .05
            grid.info.width = grid.info.height = 2
            grid.info.origin.orientation.w = 1.
            grid.data = [100 if self.bad_map else 0] * 4
            response.map = grid
            return response

        if scenario != 'map_missing':
            self.node.create_service(GetMap, '/map_server/map', get_map, callback_group=group)
        self.node.create_service(Empty, '/request_nomotion_update', lambda req, res: res,
                                 callback_group=group)

        def publish():
            stamp = self.node.get_clock().now().to_msg()
            laser = LaserScan()
            laser.header.frame_id, laser.header.stamp = 'laser', stamp
            laser.range_min, laser.range_max, laser.ranges = .05, 10., [2.] * 8
            self.scan.publish(laser)
            odom = Odometry()
            odom.header.frame_id, odom.header.stamp = 'odom', stamp
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.orientation.w = 1.
            self.odom.publish(odom)
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id, pose.header.stamp = 'map', stamp
            pose.pose.pose.position.x = pose.pose.pose.position.y = .025
            pose.pose.pose.orientation.w = 1.
            for index in (0, 7, 35):
                pose.pose.covariance[index] = 100. if self.bad_covariance else .001
            self.amcl.publish(pose)
            transform = TransformStamped()
            transform.header.frame_id, transform.header.stamp = 'map', stamp
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = transform.transform.translation.y = .025
            transform.transform.rotation.w = 1.
            self.tf.sendTransform(transform)

        def cancel(handle):
            self.cancellations += 1
            return CancelResponse.ACCEPT

        def execute(handle):
            self.goals.append(handle.request)
            deadline = time.monotonic() + (6. if scenario.endswith('_during_goal') else .8)
            while time.monotonic() < deadline and not self.shutdown_requested.is_set():
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(.02)
            handle.succeed()
            return NavigateToPose.Result()

        self.action = None
        def start_action():
            self.action = ActionServer(self.node, NavigateToPose, 'navigate_to_pose', execute,
                                       cancel_callback=cancel, callback_group=group)
            if scenario == 'action_delayed':
                self.action_timer.cancel()
        if scenario == 'action_delayed':
            self.action_timer = self.node.create_timer(3., start_action, callback_group=group)
        else:
            start_action()
        self.timer = self.node.create_timer(.05, publish, callback_group=group)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @property
    def bad_map(self):
        return self.scenario == 'map_wrong' or (
            self.scenario == 'map_changes_during_goal' and bool(self.goals))

    @property
    def bad_covariance(self):
        return self.scenario == 'covariance_high' or (
            self.scenario == 'covariance_worsens_during_goal' and bool(self.goals))

    def close(self):
        self.shutdown_requested.set()
        self.executor.shutdown(timeout_sec=3.)
        self.thread.join(timeout=3.)
        if self.action is not None:
            self.action.destroy()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def files(tmp_path):
    (tmp_path / 'test.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    map_file = tmp_path / 'test.yaml'
    map_file.write_text(yaml.safe_dump({
        'image': 'test.pgm', 'mode': 'trinary', 'resolution': .05, 'origin': [0., 0., 0.],
        'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}))
    places = tmp_path / 'places.yaml'
    places.write_text(yaml.safe_dump({
        'environment': 'real', 'map': {'frame': 'map', **map_hashes(map_file)},
        'places': {'gate1': {'measured': True, 'pose': [.025, .025, 0.]},
                   'home': {'measured': True, 'pose': [.075, .075, 0.]}}}))
    return places, map_file


@pytest.mark.parametrize('scenario', [
    'map_wrong', 'map_missing', 'covariance_high', 'healthy', 'action_delayed',
    'map_changes_during_goal', 'covariance_worsens_during_goal',
])
def test_cli_requires_live_map_and_confident_localization(files, monkeypatch, scenario):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    places, map_file = files
    peer = NavigationPeer(scenario)
    startup_wait = '8' if scenario == 'action_delayed' else '4'
    arguments = ['ros2', 'run', 'roport_smartcart', 'navigate', '--mode', 'real',
                 '--domain-id', '192', '--startup-wait', startup_wait, '--timeout', '8',
                 '--places', str(places), '--map', str(map_file), '--target', 'gate1']
    if scenario.endswith('_during_goal'):
        arguments += ['--return-to', 'home']
    try:
        result = subprocess.run(arguments, env={**os.environ, 'ROS_DOMAIN_ID': '192'},
                                capture_output=True, text=True, timeout=22)
        output = result.stdout + result.stderr
        if scenario in ('healthy', 'action_delayed'):
            assert result.returncode == 0, output
            assert len(peer.goals) == 1, output
            assert peer.cancellations == 0, output
            assert '"outcome": "succeeded"' in output, output
        elif scenario.endswith('_during_goal'):
            assert result.returncode != 0, output
            assert len(peer.goals) == 1, output
            assert peer.cancellations >= 1, output
        else:
            assert result.returncode != 0, output
            assert not peer.goals, output
    finally:
        peer.close()
