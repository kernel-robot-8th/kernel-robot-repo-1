"""Start the isolated simulator with one height-adjustable obstacle; no motion."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    height = float(LaunchConfiguration('height').perform(context))
    if not math.isfinite(height) or not .01 <= height <= .3:
        raise ValueError('height must be 0.01..0.30 metres')
    sdf = f'''<sdf version="1.9"><model name="sensor_obstacle">
      <static>true</static><pose>0.9 -0.3 {height/2} 0 0 0</pose><link name="box">
      <collision name="collision"><geometry><box><size>0.12 0.18 {height}</size>
      </box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.12 0.18 {height}</size>
      </box></geometry><material><diffuse>1 0.25 0.05 1</diffuse></material></visual>
      </link></model></sdf>'''
    share = Path(get_package_share_directory('roport_smartcart'))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(share / 'launch' / 'simulation.launch.py'))),
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'roport_airport', '-name', 'sensor_obstacle', '-string', sdf],
                 parameters=[{'use_sim_time': True}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('height', default_value='0.05'),
        OpaqueFunction(function=_launch),
    ])
