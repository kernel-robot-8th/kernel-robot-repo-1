"""Real Nav2 controller/progress plugin, synthetic odom/TF, isolated from hardware."""
import importlib.util
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import yaml


def test_real_controller_low_speed_turn_and_stall(tmp_path, monkeypatch):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from tf2_ros import TransformBroadcaster

    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    spec = importlib.util.spec_from_file_location('trial_launch',
        Path(__file__).resolve().parents[1] / 'launch/trial.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    # Production trial overrides are exercised by the real plugin, not a copy of its logic.
    params = {'controller_frequency': 20., 'enable_stamped_cmd_vel': False,
              'controller_plugins': ['FollowPath'], 'goal_checker_plugins': ['general_goal_checker'],
              'general_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
              'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
              'FollowPath.use_collision_detection': False,
              'FollowPath.use_regulated_linear_velocity_scaling': False,
              'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
              'FollowPath.lookahead_dist': .1, **launch.CONTROLLER_OVERRIDES}
    config = tmp_path / 'params.yaml'
    config.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': params},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'global_frame': 'odom', 'robot_base_frame': 'base_footprint',
            'rolling_window': True, 'width': 3, 'height': 3, 'resolution': .05,
            'robot_radius': .08, 'plugins': ['inflation_layer'], 'track_unknown_space': False,
            'inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
            'update_frequency': 20., 'publish_frequency': 1., 'always_send_full_costmap': True,
        }}}}))
    env = {**os.environ, 'ROS_DOMAIN_ID': '196', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    log = (tmp_path / 'controller.log').open('w+')
    process = subprocess.Popen([str(Path(get_package_prefix('nav2_controller')) /
        'lib/nav2_controller/controller_server'), '--ros-args', '--params-file', str(config),
        '-r', 'cmd_vel:=progress_test/cmd_vel'], env=env, stdout=log, stderr=log)
    context = Context()
    rclpy.init(context=context, domain_id=196)
    node = rclpy.create_node('trial_progress_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    pub = node.create_publisher(Odometry, 'odom', 10)
    tf_pub = TransformBroadcaster(node)
    state = {'x': 0., 'y': 0., 'yaw': math.pi / 2, 'v': 0., 'w': 0., 'freeze': False,
             'last': time.monotonic(), 'max_speed': 0., 'turns': 0}

    def velocity(msg):
        state['v'], state['w'] = msg.linear.x, msg.angular.z
        state['max_speed'] = max(state['max_speed'], abs(msg.linear.x))
        if abs(msg.angular.z) > .02:
            state['turns'] += 1
    node.create_subscription(Twist, 'progress_test/cmd_vel', velocity, 10)

    def tick():
        now = time.monotonic()
        dt, state['last'] = now - state['last'], now
        v, w = (0., 0.) if state['freeze'] else (state['v'], state['w'])
        state['yaw'] += w * dt
        state['x'] += math.cos(state['yaw']) * v * dt
        state['y'] += math.sin(state['yaw']) * v * dt
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'odom', 'base_footprint', stamp
        tf.transform.translation.x, tf.transform.translation.y = state['x'], state['y']
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(state['yaw']/2), math.cos(state['yaw']/2)
        tf_pub.sendTransform(tf)
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp, odom.child_frame_id = 'odom', stamp, 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state['x'], state['y']
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        pub.publish(odom)
    timer = node.create_timer(.02, tick)

    def wait(condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            assert process.poll() is None, (tmp_path / 'controller.log').read_text()
            executor.spin_once(timeout_sec=.02)
            if condition():
                return
        pytest.fail((tmp_path / 'controller.log').read_text()[-5000:])

    client = node.create_client(ChangeState, 'controller_server/change_state')
    action = ActionClient(node, FollowPath, 'follow_path')
    def start_path(distance):
        goal = FollowPath.Goal()
        goal.controller_id, goal.goal_checker_id, goal.progress_checker_id = 'FollowPath', 'general_goal_checker', 'progress_checker'
        goal.path.header.frame_id = 'odom'
        for i in range(21):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x, pose.pose.position.y = state['x'] + distance * i / 20, state['y']
            pose.pose.orientation.w = 1.
            goal.path.poses.append(pose)
        future = action.send_goal_async(goal)
        wait(future.done, 5)
        handle = future.result()
        assert handle.accepted
        return handle.get_result_async()

    try:
        wait(client.service_is_ready, 10)
        for transition in (1, 3):  # configure, activate
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            wait(future.done, 10)
            assert future.result().success, (tmp_path / 'controller.log').read_text()
        wait(action.server_is_ready, 5)
        begun = time.monotonic()
        result = start_path(.6)
        wait(result.done, 50)
        assert result.result().status == 4, result.result()
        assert result.result().result.error_code == 0
        assert time.monotonic() - begun > 10
        assert state['x'] >= .51 and state['turns'] > 0
        assert state['max_speed'] <= .03001
        # Disabling progress checking would pass the journey but miss this blocked robot.
        state['freeze'] = True
        result = start_path(.6)
        wait(result.done, 20)
        assert result.result().status == 6
        assert result.result().result.error_code == FollowPath.Result.FAILED_TO_MAKE_PROGRESS
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        timer.cancel()
        action.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        log.close()
