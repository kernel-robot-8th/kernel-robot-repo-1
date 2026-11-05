"""Inspect plans only: no launch processes, ROS context or goal publication."""
import importlib.util
from pathlib import Path
import sys

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


@pytest.fixture
def plan(monkeypatch, tmp_path):
    path = Path(__file__).parents[1] / 'launch' / 'simulation.launch.py'
    spec = importlib.util.spec_from_file_location('smartcart_simulation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(tmp_path / name))
    return module.generate_launch_description().entities, tmp_path


def context(actions, **overrides):
    ctx = LaunchContext()
    ctx.environment.update(ROS_DOMAIN_ID='13', ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET')
    for action in actions:
        if isinstance(action, (DeclareLaunchArgument, SetEnvironmentVariable)):
            action.execute(ctx)
    ctx.launch_configurations.update(overrides)
    return ctx


def test_real_parent_domain_cannot_leak_into_simulation(plan):
    actions, _ = plan
    ctx = context(actions)
    assert ctx.environment['ROS_DOMAIN_ID'] == '198'
    assert ctx.environment['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert ctx.launch_configurations == {'gui': 'true', 'rviz': 'false', 'port': '8081'}
    child = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    args = {name: perform_substitutions(ctx, normalize_to_list_of_substitutions(value))
            for name, value in child.launch_arguments}
    assert args == {'mode': 'sim', 'ros_domain_id': '198', 'gui': 'true', 'rviz': 'false'}
    # The only process added here is the console; no CLI or goal sender exists.
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_package == 'pinky_test_console'
    assert nodes[0].node_executable == 'web_server'


def test_console_uses_sim_clock_loopback_and_typed_port(plan):
    actions, root = plan
    ctx = context(actions, port='8083')
    node = next(a for a in actions if isinstance(a, Node))
    params = evaluate_parameters(ctx, node._Node__parameters)[0]
    assert params == {'mode': 'nav', 'use_sim_time': True, 'ip': '127.0.0.1',
                      'port': 8083, 'teleop_enabled': True,
                      'layout_yaml': str(root / 'roport_gz_world/layout/roport_airport.yaml')}
    prefix = node._ExecuteLocal__process_description._Executable__prefix
    assert ''.join(p.perform(ctx) for p in prefix) == sys.executable


def test_port_is_integer_not_an_arbitrary_parameter_string(plan):
    actions, _ = plan
    ctx = context(actions, port='not-a-port')
    node = next(a for a in actions if isinstance(a, Node))
    with pytest.raises((ValueError, TypeError)):
        evaluate_parameters(ctx, node._Node__parameters)
