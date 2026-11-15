"""Hardware Nav2/lease guard/ADC only; a return goal requires a separate command.

Reuse the deployed trial settings but omit line_nav: real_return uses the same
RouteExecutor directly with actual ADC input, and owns its line velocity topic.
"""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    path = Path(get_package_share_directory('roport_pinky'))/'launch/trial.launch.py'
    spec = importlib.util.spec_from_file_location('return_trial_stack', path)
    trial = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trial)

    def nodes(context):
        return [action for action in trial._nodes(context)
                if not isinstance(action, Node) or action.node_executable != 'line_nav']

    actions = []
    for action in trial.generate_launch_description().entities:
        if isinstance(action, OpaqueFunction):
            continue
        if isinstance(action, DeclareLaunchArgument) and action.name == 'ir_node':
            action = DeclareLaunchArgument('ir_node', default_value='true',
                                           description='Physical ADC sensor; no simulated IR')
        if isinstance(action, DeclareLaunchArgument) and action.name == 'planner_plugin':
            action = DeclareLaunchArgument('planner_plugin',
                                           default_value='nav2_smac_planner::SmacPlanner2D')
        if isinstance(action, DeclareLaunchArgument) and action.name == 'recovery_bt':
            action = DeclareLaunchArgument('recovery_bt', default_value=PathJoinSubstitution([
                FindPackageShare('roport_smartcart'), 'config', 'bounded_field_recovery.xml']))
        actions.append(action)
    return LaunchDescription(actions+[OpaqueFunction(function=nodes)])
