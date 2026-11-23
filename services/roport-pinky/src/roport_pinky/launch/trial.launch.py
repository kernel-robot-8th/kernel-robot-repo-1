"""Trial-only stack. Bringup and calibration remain explicit operator actions."""
import math
import os

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
    # Keep the supplied base profile's tracking distances until hardware validation.
    'FollowPath.use_collision_detection': True,
    'progress_checker_plugins': ['progress_checker'],
    'progress_checker.plugin': 'nav2_controller::PoseProgressChecker',
    'progress_checker.required_movement_radius': 0.05,
    'progress_checker.required_movement_angle': 0.15,
    'progress_checker.movement_time_allowance': 15.0,
}


DOCK_CONTROLLER_OVERRIDES = {
    'controller_plugins': ['FollowPath', 'GateDock'],
    'goal_checker_plugins': ['general_goal_checker', 'gate_dock_goal_checker'],
    'gate_dock_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
    'gate_dock_goal_checker.xy_goal_tolerance': .01,
    'gate_dock_goal_checker.yaw_goal_tolerance': .05,
    'gate_dock_goal_checker.stateful': True,
    'GateDock.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'GateDock.desired_linear_vel': .02,
    'GateDock.min_approach_linear_velocity': .005,
    'GateDock.regulated_linear_scaling_min_speed': .005,
    # 감속 거리(approach_velocity_scaling_dist)는 여기서 넣지 않는다: 게이트 마커 주차는 params_file 에 적은 값을
    # 그대로 쓰고, 없으면 RPP 기본값이다(변경 전과 같음).
    'GateDock.lookahead_dist': .05,
    'GateDock.min_lookahead_dist': .03,
    'GateDock.max_lookahead_dist': .10,
    'GateDock.use_velocity_scaled_lookahead_dist': False,
    'GateDock.rotate_to_heading_angular_vel': .15,
    'GateDock.use_collision_detection': True,
    'GateDock.allow_reversing': False,
}


# 마커 없는 정밀 접근(FineAlign)에만 쓰는 GateDock 속도. 게이트 마커 주차(ROPORT_GATE_DOCK_CONFIG)에는 넣지 않는다.
# 이 값은 여기에만 둔다(params_file 에 적으면 같은 파일로 띄운 게이트 마커 주차의 감속 거리까지 바뀐다).
# 2026-11-23 d660: .005/.005 + 감속 거리 0.6 m 로는 20 cm 접근 전체가 감속 구간 → 25 s 시간 초과.
# 아래 값으로 19 s 성공(N12 정밀 접근 1회).
FINE_ALIGN_CONTROLLER_OVERRIDES = {
    'GateDock.min_approach_linear_velocity': .01,
    'GateDock.regulated_linear_scaling_min_speed': .015,
    'GateDock.approach_velocity_scaling_dist': .05,
}


def _docking_mode():
    """GateDock 제어기·gate_dock_goal_checker 를 넣는 방식: 'gate'(게이트 마커 주차), 'fine_align'(마커 없는
    정밀 접근) 또는 None. 드라이버(trial_nav2)와 같이 ROPORT_GATE_DOCK_CONFIG 가 있으면 게이트 마커 주차가 우선이다.
    ROPORT_FINE_ALIGN_GOALS 해석은 roport_pinky.fine_align.goals_from_env 와 같다(쉼표 구분, 빈 항목 무시)."""
    if os.environ.get('ROPORT_GATE_DOCK_CONFIG'):
        return 'gate'
    goals = [g.strip() for g in os.environ.get('ROPORT_FINE_ALIGN_GOALS', '').split(',')]
    return 'fine_align' if any(goals) else None


def _nodes(context):
    line_speed = float(LaunchConfiguration('line_speed').perform(context))
    if not math.isfinite(line_speed) or not .02 <= line_speed <= .08:
        raise ValueError('line_speed must be finite and in [0.02, 0.08] m/s')
    max_linear = float(LaunchConfiguration('max_linear').perform(context))
    # Retain the existing Nav2 trial speed range.
    if not math.isfinite(max_linear) or not .02 <= max_linear <= .15:
        raise ValueError('max_linear must be finite and in [0.02, 0.15] m/s')
    max_angular = float(LaunchConfiguration('max_angular').perform(context))
    # Retain the existing Nav2 trial rotation range.
    if not math.isfinite(max_angular) or not .15 <= max_angular <= 1.5:
        raise ValueError('max_angular must be finite and in [0.15, 1.5] rad/s')
    line_rotate_speed = float(LaunchConfiguration('line_rotate_speed').perform(context))
    if not math.isfinite(line_rotate_speed) or not .15 <= line_rotate_speed <= .4:
        raise ValueError('line_rotate_speed must be finite and in [0.15, 0.4] rad/s')
    goal_xy_tolerance = float(LaunchConfiguration('goal_xy_tolerance').perform(context))
    if not math.isfinite(goal_xy_tolerance) or not .01 <= goal_xy_tolerance <= .1:
        raise ValueError('goal_xy_tolerance must be finite and in [0.01, 0.1] m')
    planner_tolerance = float(LaunchConfiguration('planner_tolerance').perform(context))
    if not math.isfinite(planner_tolerance) or not .05 <= planner_tolerance <= .3:
        raise ValueError('planner_tolerance must be finite and in [0.05, 0.3] m')
    config = LaunchConfiguration('params_file')
    nodes = [
        Node(package='roport_pinky', executable='trial_guard', output='screen'),
        Node(package='roport_line_nav', executable='line_nav', output='screen',
             parameters=[{'robot': LaunchConfiguration('robot'),
                          'layout_file': LaunchConfiguration('layout_file'), 'base_speed': line_speed,
                          'max_linear': line_speed, 'max_angular': line_rotate_speed,
                          'rotate_speed': line_rotate_speed}],
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
        if name == 'bt_navigator':
            # Goal acknowledgements must survive short scheduling delays on the Pi.
            overrides['default_server_timeout'] = 500  # milliseconds
            overrides['default_nav_to_pose_bt_xml'] = LaunchConfiguration('recovery_bt')
        if name == 'planner_server':
            # The public-space goal is near the map edge; a .5 m search crosses it.
            # 추종(11-20 실로봇): 고객 앞 목표가 사이에 선 인형 위에 떨어지면 0.1 로는 계획 실패 → 시험 때 0.3
            overrides['GridBased.tolerance'] = planner_tolerance  # meters, default 0.1
            overrides['GridBased.plugin'] = LaunchConfiguration('planner_plugin')
        if name == 'controller_server':
            overrides.update(CONTROLLER_OVERRIDES)
            overrides['general_goal_checker.xy_goal_tolerance'] = goal_xy_tolerance
            overrides['FollowPath.desired_linear_vel'] = max_linear
            docking = _docking_mode()
            if docking:
                overrides.update(DOCK_CONTROLLER_OVERRIDES)
                if docking == 'fine_align':
                    overrides.update(FINE_ALIGN_CONTROLLER_OVERRIDES)
                overrides['GateDock.desired_linear_vel'] = min(.02, max_linear)
            overrides['FollowPath.rotate_to_heading_angular_vel'] = max_angular
        if name in ('controller_server', 'behavior_server', 'velocity_smoother'):
            overrides['enable_stamped_cmd_vel'] = False
            remaps.append(('cmd_vel', 'trial/nav2_raw_cmd_vel'))
        if name == 'behavior_server':
            overrides['max_rotational_vel'] = max_angular
            overrides['min_rotational_vel'] = max_angular
        if name == 'velocity_smoother':
            overrides['max_velocity'] = [max_linear, 0.0, max_angular]
            overrides['min_velocity'] = [-max_linear, 0.0, -max_angular]
            remaps.append(('cmd_vel_smoothed', 'trial/nav2_cmd_vel'))
        nodes.append(Node(package=package, executable=name, name=name, output='screen',
                          parameters=[config, overrides], remappings=remaps))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='trial_lifecycle_manager', output='screen', parameters=[{
                          'autostart': True, 'node_names': [name for _, name in servers]}]))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument('robot'), DeclareLaunchArgument('map'),
            DeclareLaunchArgument('planner_plugin', default_value='nav2_navfn_planner::NavfnPlanner'),
            DeclareLaunchArgument('recovery_bt', default_value=PathJoinSubstitution([
                FindPackageShare('roport_pinky'), 'behavior_trees', 'bounded_recovery.xml'])),
            DeclareLaunchArgument('layout_file'),
            DeclareLaunchArgument('ir_node', default_value='false'),
            DeclareLaunchArgument('line_speed', default_value='0.03',
                                  description='Line base speed and cap in m/s; match control route limit'),
            DeclareLaunchArgument('line_rotate_speed', default_value='0.15',
                                  description='Independent line rotation speed/cap, 0.15..0.4 rad/s'),
            DeclareLaunchArgument('max_linear', default_value='0.03',
                                  description='Nav2 controller/smoother cap in m/s'),
            DeclareLaunchArgument('max_angular', default_value='0.4',
                                  description='Nav2 rotation cap in rad/s; use field-verified value'),
            DeclareLaunchArgument('goal_xy_tolerance', default_value='0.08',
                                  description='Map pose XY tolerance in m; not measured wall clearance'),
            DeclareLaunchArgument('planner_tolerance', default_value='0.1',
                                  description='GridBased goal search tolerance in m, 0.05..0.3 (0.3 for follow tests)'),
            DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
                FindPackageShare('pinky_navigation'), 'params', 'nav2_params.yaml']))]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
