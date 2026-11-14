# 노드 경로 주행(FFT1-117). bringup(모터·odom)은 따로 실행한다. line_follower 와 같이 띄우지 않는다.
#   ros2 launch roport_line_nav line_nav.launch.py robot:=pinky_d660
#   ros2 topic pub --once /line_nav/command std_msgs/msg/String \
#     "{data: '{\"cmd\": \"start\", \"route\": [\"N14\", \"N16\"]}'}"
#   ir_node:=false  pinky_sensor_adc main_node 를 이미 따로 실행 중일 때
#   layout_file:=<roport_airport.yaml>  roport_gz_world 를 설치하지 않았을 때
#   cmd_guard:=true(기본)  line_nav → line_nav/cmd_vel → cmd_vel_guard → cmd_vel (0.3 s 끊기면 0)
#   dock_file:=~/roport_dock.yaml(기본)  상차 노드 정밀 도킹 값(FFT1-112), 빈 값이면 파일 안 씀
#   reverse_follow_speed:=0.02  back 명령 라인 후진 속도(FFT1-172, 0.01~0.03)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FLOAT_ARGS = {
    'base_speed': ('0.03', 'm/s line following'),
    'approach_speed': ('0.03', 'm/s near nodes, pass/advance (capped by base_speed)'),
    'approach_lead_cm': ('5.0', 'slow down this many cm before the expected node position'),
    'post_turn_slow_cm': ('10.0', 'after a node turn, approach_speed for this many cm (0 = off)'),
    'kp': ('1.0', 'rad/s per line error (x speed/gain_ref_speed above gain_ref_speed)'),
    'kd': ('0.05', 'line error rate gain'),
    'gain_ref_speed': ('0.06', 'm/s; kp grows with speed only above this'),
    'max_linear': ('0.06', 'm/s cap (hard cap 0.15 in code)'),
    'max_angular': ('0.6', 'rad/s cap (hard cap 1.5 in code)'),
    'rotate_speed': ('0.4', 'rad/s in-place rotation'),
    'reverse_speed': ('0.02', 'm/s reverse into P1/P2/P5/P6'),
    'ir_ahead_cm': ('3.0', 'center IR to wheel axle (measured 2026-11-03)'),
    'turn_advance_cm': ('4.2', 'drive after side line seen before rotating'),
    'gap_max_cm': ('8.0', 'line-gap recovery: max straight drive after losing the line (0 = off)'),
    'gap_heading_window_cm': ('4.0', 'line-gap recovery: heading from the last N cm on the line'),
    'search_deg': ('0.0', 'line-gap recovery: in-place search width (0 = off, default)'),
    'ir_spacing_cm': ('2.0', 'center to side IR (HW spec ~20 mm, not measured)'),
    'reverse_follow_speed': ('0.02', 'm/s line reverse (back command), 0.01..0.03'),
    'reverse_wn': ('0.8', 'rad/s line reverse state feedback natural frequency'),
    'reverse_zeta': ('0.8', 'line reverse state feedback damping'),
}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot', description='pinky_764e | pinky_6422 | pinky_d660'),
        DeclareLaunchArgument('ir_node', default_value='true',
                              description='run pinky_sensor_adc main_node too'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='never publish cmd_vel (route/IR check only)'),
        DeclareLaunchArgument('layout_file', default_value='',
                              description='layout yaml (default: roport_gz_world share)'),
        DeclareLaunchArgument('cmd_guard', default_value='true',
                              description='relay via cmd_vel_guard (zero after 0.3 s)'),
        DeclareLaunchArgument('dock_file', default_value='~/roport_dock.yaml',
                              description='load-node dock values yaml (empty = memory only)'),
    ]
    args += [DeclareLaunchArgument(k, default_value=v, description=d)
             for k, (v, d) in FLOAT_ARGS.items()]
    params = {
        'robot': LaunchConfiguration('robot'),
        'layout_file': LaunchConfiguration('layout_file'),
        'dock_file': ParameterValue(LaunchConfiguration('dock_file'), value_type=str),
        'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
    }
    params.update({k: ParameterValue(LaunchConfiguration(k), value_type=float)
                   for k in FLOAT_ARGS})
    return LaunchDescription(args + [
        Node(package='pinky_sensor_adc', executable='main_node', output='screen',
             condition=IfCondition(LaunchConfiguration('ir_node'))),
        Node(package='roport_line_nav', executable='line_nav', output='screen',
             parameters=[params], remappings=[('cmd_vel', 'line_nav/cmd_vel')],
             condition=IfCondition(LaunchConfiguration('cmd_guard'))),
        Node(package='roport_line_nav', executable='cmd_vel_guard', output='screen',
             condition=IfCondition(LaunchConfiguration('cmd_guard'))),
        Node(package='roport_line_nav', executable='line_nav', output='screen',
             parameters=[params], condition=UnlessCondition(LaunchConfiguration('cmd_guard'))),
    ])
