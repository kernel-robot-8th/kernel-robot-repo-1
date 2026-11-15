# IR 라인 추종 v0. bringup(모터)은 따로 실행한다.
#   ros2 launch roport_pinky_line line_follow.launch.py robot:=pinky_d660
#   ... robot:=pinky_d660 dry_run:=true  # cmd_vel 을 발행하지 않고 state 만 본다
#   ir_node:=false  pinky_sensor_adc main_node 를 이미 따로 실행 중일 때

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FLOAT_ARGS = {
    'base_speed': ('0.05', 'm/s at zero error'),
    'max_linear': ('0.10', 'm/s cap (hard cap 0.15 in code)'),
    'max_angular': ('0.6', 'rad/s cap (hard cap 1.5 in code)'),
    'kp': ('1.0', 'rad/s per unit error'),
    'kd': ('0.05', 'rad/s per unit error rate'),
}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot', description='pinky_764e | pinky_6422 | pinky_d660'),
        DeclareLaunchArgument('ir_node', default_value='true',
                              description='run pinky_sensor_adc main_node too'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='compute commands but never publish cmd_vel'),
        DeclareLaunchArgument('start_enabled', default_value='false',
                              description='start following without the enable topic'),
        DeclareLaunchArgument('stop_at_node', default_value='true',
                              description='stop when all three IR channels see tape'),
    ]
    args += [DeclareLaunchArgument(k, default_value=v, description=d)
             for k, (v, d) in FLOAT_ARGS.items()]

    params = {
        'robot': LaunchConfiguration('robot'),
        'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
        'start_enabled': ParameterValue(LaunchConfiguration('start_enabled'), value_type=bool),
        'stop_at_node': ParameterValue(LaunchConfiguration('stop_at_node'), value_type=bool),
    }
    params.update({k: ParameterValue(LaunchConfiguration(k), value_type=float)
                   for k in FLOAT_ARGS})

    return LaunchDescription(args + [
        Node(package='pinky_sensor_adc', executable='main_node', output='screen',
             condition=IfCondition(LaunchConfiguration('ir_node'))),
        Node(package='roport_pinky_line', executable='line_follower', output='screen',
             parameters=[params]),
    ])
