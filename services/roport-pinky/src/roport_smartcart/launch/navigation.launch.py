"""One Nav2 stack for isolated Gazebo simulation or explicitly configured hardware."""

import json
import os
from pathlib import Path
import shlex
import tempfile
from uuid import uuid4

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

from roport_smartcart.profiles import build_params


def _include(package, filename, **arguments):
    return IncludeLaunchDescription(AnyLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / 'launch' / filename)),
        launch_arguments=arguments.items())


def _boolean(value):
    if value not in ('true', 'false'):
        raise ValueError('boolean launch arguments must be true or false')
    return value == 'true'


def _launch(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mode = arg('mode')
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    gui, rviz = _boolean(arg('gui')), _boolean(arg('rviz'))
    domain = arg('ros_domain_id')
    if mode == 'real' and not domain:
        raise ValueError('real mode requires explicit ros_domain_id and map')
    domain = domain or '198'
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError('ros_domain_id must be an integer in [0, 232]')
    domain = str(int(domain))
    share = Path(get_package_share_directory('roport_smartcart'))
    map_file = arg('map')
    if not map_file and mode == 'sim':
        map_file = str(Path(get_package_share_directory('roport_gz_world')) /
                       'maps' / 'airport_map.yaml')
    if not map_file or not Path(map_file).is_file():
        raise ValueError('an existing map YAML is required')
    base = arg('base_params_file') or str(Path(
        get_package_share_directory('pinky_navigation')) / 'params' / 'nav2_params.yaml')
    overrides = arg('overrides_file') or str(share / 'config' / 'nav2_overrides.yaml')
    params = build_params(base, overrides, mode,
                          footprint=json.loads(arg('footprint')) if arg('footprint') else None,
                          max_linear=float(arg('max_linear')) if arg('max_linear') else None,
                          max_angular=float(arg('max_angular')) if arg('max_angular') else None)
    recovery = (Path(arg('recovery_bt')) if arg('recovery_bt') else
                Path(get_package_share_directory('roport_pinky')) /
                'behavior_trees' / 'bounded_recovery.xml')
    if arg('recovery_bt') and not recovery.is_file():
        raise ValueError('recovery_bt must be an existing local behavior tree')
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(recovery)
    sim_time = 'true' if mode == 'sim' else 'false'
    actions = [SetEnvironmentVariable('ROS_DOMAIN_ID', domain)]
    if mode == 'sim':
        # Recompose pinklab-art/pinky_pro pinky_gz_sim launch_sim.launch.xml
        # (Apache-2.0, upstream LICENSE retained) to make its forced GUI optional.
        description = Path(get_package_share_directory('pinky_description'))
        gazebo = Path(get_package_share_directory('pinky_gz_sim'))
        world = Path(get_package_share_directory('roport_gz_world')) / 'worlds' / 'roport_airport.sdf'
        resources = [os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                     str(description.parent), str(gazebo / 'models')]
        initial = params['amcl']['ros__parameters']['initial_pose']
        actions.extend([
            SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
            SetEnvironmentVariable('GZ_PARTITION', arg('gz_partition') or
                                   f'smartcart_{domain}_{uuid4().hex[:8]}'),
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(filter(None, resources))),
            _include('ros_gz_sim', 'gz_sim.launch.py',
                     gz_args='-r -s --headless-rendering ' + shlex.quote(str(world)),
                     on_exit_shutdown='true'),
            # Gazebo already publishes wheel joints; no self-subscribing joint publisher.
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(
                     Command(['xacro ', str(share / 'urdf' / 'pinky_sonar_sim.urdf.xacro'),
                              ' is_sim:=true']), value_type=str)}]),
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'pinky', '-topic', '/robot_description',
                            '-x', str(initial['x']), '-y', str(initial['y']),
                            '-z', str(initial['z'] + .1), '-Y', str(initial['yaw'])],
                 parameters=[{'use_sim_time': True}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 parameters=[{'config_file': str(gazebo / 'params' / 'pinky_bridge.yaml'),
                              'use_sim_time': True}],
                 remappings=[('cmd_vel', 'cmd_vel_safe')]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 arguments=['/sim/sonar_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                            '/model/pinky/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                 remappings=[('/model/pinky/pose', '/sim/gazebo_poses')],
                 parameters=[{'use_sim_time': True}]),
            Node(package='roport_smartcart', executable='sim_sonar', output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='roport_smartcart', executable='range_scan', output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='nav2_collision_monitor', executable='collision_monitor',
                 name='collision_monitor', output='screen',
                 parameters=[str(share / 'config' / 'collision_monitor_sim.yaml')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='sensor_lifecycle_manager', output='screen', parameters=[{
                     'use_sim_time': True, 'autostart': True,
                     'node_names': ['collision_monitor']}]),
        ])
        if gui:
            actions.append(_include('ros_gz_sim', 'gz_sim.launch.py',
                                    gz_args='-g', on_exit_shutdown='false'))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(Path(get_package_share_directory(
                                'nav2_bringup')) / 'rviz' / 'nav2_default_view.rviz')],
                            parameters=[{'use_sim_time': mode == 'sim'}]))
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', prefix='smartcart_nav_', suffix='.yaml',
                                         delete=False) as stream:
            temp_path = Path(stream.name)
            yaml.safe_dump(params, stream)
        actions.insert(0, RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temp_path.unlink(missing_ok=True))])))
        actions.append(_include('pinky_navigation', 'bringup_launch.xml',
                                map=str(Path(map_file).resolve()), params_file=str(temp_path),
                                use_sim_time=sim_time, use_composition='False',
                                autostart='True', slam='False'))
    except Exception:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)
        raise
    return actions


def generate_launch_description():
    defaults = {'mode': 'sim', 'gui': 'true', 'rviz': 'false', 'ros_domain_id': '',
                'gz_partition': '', 'map': '', 'base_params_file': '',
                'overrides_file': '', 'footprint': '', 'max_linear': '', 'max_angular': '',
                'recovery_bt': ''}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] +
                             [OpaqueFunction(function=_launch)])
