"""Trial-only stack. Bringup and calibration remain explicit operator actions."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CONTROLLER_OVERRIDES = {
    'general_goal_checker.xy_goal_tolerance': 0.08,
    'general_goal_checker.yaw_goal_tolerance': 0.10,
    'FollowPath.desired_linear_vel': 0.03,
    'FollowPath.min_approach_linear_velocity': 0.01,
    'FollowPath.rotate_to_heading_angular_vel': 0.4,
    'progress_checker_plugins': ['progress_checker'],
    'progress_checker.plugin': 'nav2_controller::PoseProgressChecker',
    'progress_checker.required_movement_radius': 0.05,
    'progress_checker.required_movement_angle': 0.15,
    'progress_checker.movement_time_allowance': 15.0,
}


def _nodes(context):
    max_angular = float(LaunchConfiguration('max_angular').perform(context))
    # The line executor's existing rotation floor is .15 rad/s.
    if not math.isfinite(max_angular) or not .15 <= max_angular <= 1.5:
        raise ValueError('max_angular must be finite and in [0.15, 1.5] rad/s')
    config = LaunchConfiguration('params_file')
    nodes = [
        Node(package='roport_pinky', executable='trial_guard', output='screen'),
        Node(package='roport_line_nav', executable='line_nav', output='screen',
             parameters=[{'robot': LaunchConfiguration('robot'),
                          'layout_file': LaunchConfiguration('layout_file'), 'base_speed': 0.03,
                          'max_linear': 0.03, 'max_angular': max_angular,
                          'rotate_speed': max_angular}],
             remappings=[('cmd_vel', 'line_nav/cmd_vel')]),
        Node(package='pinky_sensor_adc', executable='main_node', output='screen',
             condition=IfCondition(LaunchConfiguration('ir_node'))),
    ]
    servers = [('nav2_map_server', 'map_server'), ('nav2_amcl', 'amcl'),
               ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
               ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
               ('nav2_smoother', 'smoother_server'), ('nav2_velocity_smoother', 'velocity_smoother')]
    for package, name in servers:
        overrides = {'use_sim_time': False}
        remaps = []
        if name == 'map_server':
            overrides['yaml_filename'] = LaunchConfiguration('map')
        if name == 'controller_server':
            overrides.update(CONTROLLER_OVERRIDES)
            overrides['FollowPath.rotate_to_heading_angular_vel'] = max_angular
        if name in ('controller_server', 'behavior_server', 'velocity_smoother'):
            overrides['enable_stamped_cmd_vel'] = False
            remaps.append(('cmd_vel', 'trial/nav2_raw_cmd_vel'))
        if name == 'behavior_server':
            overrides['max_rotational_vel'] = max_angular
            overrides['min_rotational_vel'] = max_angular
        if name == 'velocity_smoother':
            overrides['max_velocity'] = [0.03, 0.0, max_angular]
            overrides['min_velocity'] = [-0.03, 0.0, -max_angular]
            remaps.append(('cmd_vel_smoothed', 'trial/nav2_cmd_vel'))
        nodes.append(Node(package=package, executable=name, name=name, output='screen',
                          parameters=[config, overrides], remappings=remaps))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='trial_lifecycle_manager', output='screen', parameters=[{
                          'autostart': True, 'node_names': [name for _, name in servers]}]))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument('robot'), DeclareLaunchArgument('map'),
            DeclareLaunchArgument('layout_file'),
            DeclareLaunchArgument('ir_node', default_value='false'),
            DeclareLaunchArgument('max_angular', default_value='0.4',
                                  description='Shared line/Nav2 cap in rad/s; use field-verified value'),
            DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
                FindPackageShare('pinky_navigation'), 'params', 'nav2_params.yaml']))]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
