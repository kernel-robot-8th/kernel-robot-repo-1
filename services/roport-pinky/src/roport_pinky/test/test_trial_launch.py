"""Generate trial parameters without starting any ROS or motor process."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


@pytest.fixture
def trial_launch(monkeypatch):
    spec = importlib.util.spec_from_file_location('trial_launch_limits',
        Path(__file__).parents[1] / 'launch/trial.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    return module


def context(limit):
    ctx = LaunchContext()
    ctx.launch_configurations.update(max_angular=str(limit), robot='test_robot',
                                    map='/unused/map.yaml', layout_file='/unused/layout.yaml',
                                    params_file='/unused/params.yaml', ir_node='false')
    return ctx


@pytest.mark.parametrize('limit', [.15, .4, 1.5])
def test_all_trial_velocity_sources_respect_requested_cap(trial_launch, limit):
    nodes = {node['executable']: node for node in trial_launch._nodes(context(limit))}
    line = nodes['line_nav']['parameters'][0]
    controller = nodes['controller_server']['parameters'][1]
    behavior = nodes['behavior_server']['parameters'][1]
    smoother = nodes['velocity_smoother']['parameters'][1]
    assert line['max_linear'] == line['base_speed'] == .03
    assert line['max_angular'] == line['rotate_speed'] == limit
    assert controller['FollowPath.rotate_to_heading_angular_vel'] == limit
    assert behavior['min_rotational_vel'] == behavior['max_rotational_vel'] == limit
    assert smoother['max_velocity'] == [.03, 0., limit]
    assert smoother['min_velocity'] == [-.03, 0., -limit]
    # A field-specific cap must not change exclusive guard wiring.
    assert ('cmd_vel', 'line_nav/cmd_vel') in nodes['line_nav']['remappings']
    assert ('cmd_vel_smoothed', 'trial/nav2_cmd_vel') in nodes['velocity_smoother']['remappings']
    assert ('cmd_vel', 'trial/nav2_raw_cmd_vel') in nodes['behavior_server']['remappings']


@pytest.mark.parametrize('limit', [0., .149, 1.501, 'nan', 'inf', 'invalid'])
def test_invalid_trial_caps_fail_before_any_nodes_are_created(trial_launch, limit, monkeypatch):
    created = []
    monkeypatch.setattr(trial_launch, 'Node', lambda **kwargs: created.append(kwargs))
    with pytest.raises(ValueError):
        trial_launch._nodes(context(limit))
    assert created == []
