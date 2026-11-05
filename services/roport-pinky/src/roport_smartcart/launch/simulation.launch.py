"""Isolated Gazebo/Nav2 plus a loopback web console; never sends a goal."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    smartcart = Path(get_package_share_directory('roport_smartcart'))
    airport = Path(get_package_share_directory('roport_gz_world'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('port', default_value='8081'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '198'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(smartcart / 'launch' / 'navigation.launch.py')), launch_arguments={
                'mode': 'sim', 'ros_domain_id': '198',
                'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
            }.items()),
        Node(package='pinky_test_console', executable='web_server',
             prefix=sys.executable, output='screen', parameters=[{
                 'mode': 'nav', 'use_sim_time': True, 'ip': '127.0.0.1',
                 'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                 'layout_yaml': str(airport / 'layout' / 'roport_airport.yaml'),
                 'teleop_enabled': True,
             }]),
    ])
