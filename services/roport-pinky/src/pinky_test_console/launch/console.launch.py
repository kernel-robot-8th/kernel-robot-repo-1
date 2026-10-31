# Combines the roles of Pinky Pro launch files (Apache License 2.0,
#   https://github.com/pinklab-art/pinky_pro commit 014a09f289e6988894fbffde2624fdb1e257815b):
#   pinky_navigation/launch/web_slam.launch.xml, web_nav2.launch.xml
# The originals are included unchanged from the installed pinky_navigation package:
#   mode:=slam -> pinky_navigation/launch/map_building.launch.xml (slam_toolbox online_sync)
#   mode:=nav  -> pinky_navigation/launch/bringup_launch.xml (map_server, amcl, Nav2)
#   mode:=ir   -> no include; pinky_sensor_adc main_node (IR/ultrasonic ADC read only)
#   mode:=marker -> no include; camera preview + ArUco recognition in the console process
# pinky_bringup/bringup_robot.launch.xml is NOT included: start it separately first.
"""Pinky test console.

  ros2 launch pinky_test_console console.launch.py mode:=slam
  ros2 launch pinky_test_console console.launch.py mode:=nav map:=$HOME/roport_maps/room.yaml
  ros2 launch pinky_test_console console.launch.py mode:=ir   (IR: http://<robot>:8080/ir.html)
  ros2 launch pinky_test_console console.launch.py mode:=marker markers_yaml:=~/markers.yaml
  (Gazebo) ... use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pinky_test_console.launch_plan import (
    camera_env, included_launch, resolve_plan, resolve_rotate_180)


def launch_setup(context):
    plan = resolve_plan(
        mode=LaunchConfiguration('mode').perform(context),
        map_path=LaunchConfiguration('map').perform(context),
        port=LaunchConfiguration('port').perform(context),
        use_sim_time=LaunchConfiguration('use_sim_time').perform(context),
        ir_node=LaunchConfiguration('ir_node').perform(context),
    )
    launch_file, launch_args = included_launch(plan)
    camera = LaunchConfiguration('camera').perform(context)

    actions = [
        LogInfo(msg=f'[pinky_test_console] mode={plan.mode} map={plan.map_yaml or "-"} '
                    f'port={plan.port} use_sim_time={plan.use_sim_time} '
                    f'include={launch_file or "-"} ir_node={plan.ir_node}'),
        Node(
            package='pinky_test_console',
            executable='web_server',
            name='pinky_test_console',
            output='screen',
            additional_env=camera_env(plan.mode, camera, os.environ),
            parameters=[{
                'ip': LaunchConfiguration('ip').perform(context),
                'port': plan.port,
                'mode': plan.mode,
                'map_dir': LaunchConfiguration('map_dir').perform(context),
                'ir_dir': LaunchConfiguration('ir_dir').perform(context),
                'camera': camera,
                'rotate_180': resolve_rotate_180(
                    camera, LaunchConfiguration('rotate_180').perform(context)),
                'markers_yaml': LaunchConfiguration('markers_yaml').perform(context),
                'camera_hfov_deg': float(LaunchConfiguration('camera_hfov_deg').perform(context)),
                'use_sim_time': plan.use_sim_time,
            }],
        ),
    ]
    if launch_file:
        # pinky_navigation 이 없으면 여기서 PackageNotFoundError 로 바로 알려준다
        launch_path = os.path.join(
            get_package_share_directory('pinky_navigation'), 'launch', launch_file)
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_path),
            launch_arguments=launch_args.items(),
        ))
    if plan.ir_node:
        # ADC 읽기만 한다(모터 구동 없음). 다른 터미널에서 이미 실행 중이면 ir_node:=false
        actions.append(Node(package='pinky_sensor_adc', executable='main_node', output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam', description='slam | nav | ir | marker'),
        DeclareLaunchArgument('map', default_value='',
                              description='map .yaml path (required for mode:=nav)'),
        DeclareLaunchArgument('port', default_value='8080', description='web port'),
        DeclareLaunchArgument('ip', default_value='0.0.0.0', description='bind address'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true for Gazebo/MuJoCo simulation'),
        DeclareLaunchArgument('map_dir', default_value='~/roport_maps',
                              description='folder for saved maps'),
        DeclareLaunchArgument('ir_dir', default_value='~/roport_ir',
                              description='folder for IR recordings (CSV)'),
        DeclareLaunchArgument('ir_node', default_value='auto',
                              description='run pinky_sensor_adc main_node: '
                                          'auto (mode:=ir) | true | false'),
        DeclareLaunchArgument('camera', default_value='picamera2',
                              description='mode:=marker camera: '
                                          'picamera2 | v4l2:<n> | topic:<name>'),
        DeclareLaunchArgument('rotate_180', default_value='auto',
                              description='rotate camera image: '
                                          'auto (picamera2 only) | true | false'),
        DeclareLaunchArgument('markers_yaml', default_value='',
                              description='ArUco ID table (docs/aruco/markers.yaml) for names'),
        DeclareLaunchArgument('camera_hfov_deg', default_value='0',
                              description='camera horizontal FOV for rough distance (0 = off)'),
        OpaqueFunction(function=launch_setup),
    ])
