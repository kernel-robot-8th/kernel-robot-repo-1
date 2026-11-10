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


def context(limit, linear=.03, line_speed=.03, line_rotate_speed=.15):
    ctx = LaunchContext()
    ctx.launch_configurations.update(max_angular=str(limit), max_linear=str(linear), robot='test_robot',
                                    line_speed=str(line_speed), line_rotate_speed=str(line_rotate_speed),
                                    map='/unused/map.yaml', layout_file='/unused/layout.yaml',
                                    params_file='/unused/params.yaml', ir_node='false', goal_xy_tolerance='0.08')
    return ctx


@pytest.mark.parametrize('limit', [.15, .4, 1.5])
@pytest.mark.parametrize('linear', [.02, .03, .12, .15])
@pytest.mark.parametrize('line_speed', [.02, .03, .06, .08])
def test_all_trial_velocity_sources_respect_requested_cap(trial_launch, limit, linear, line_speed):
    nodes = {node['executable']: node for node in trial_launch._nodes(context(limit, linear, line_speed))}
    line = nodes['line_nav']['parameters'][0]
    controller = nodes['controller_server']['parameters'][1]
    behavior = nodes['behavior_server']['parameters'][1]
    smoother = nodes['velocity_smoother']['parameters'][1]
    assert line['max_linear'] == line['base_speed'] == line_speed
    assert controller['FollowPath.desired_linear_vel'] == linear
    assert line['max_angular'] == line['rotate_speed'] == .15
    assert controller['FollowPath.rotate_to_heading_angular_vel'] == limit
    assert behavior['min_rotational_vel'] == behavior['max_rotational_vel'] == limit
    assert smoother['max_velocity'] == [linear, 0., limit]
    assert smoother['min_velocity'] == [-linear, 0., -limit]
    assert nodes['bt_navigator']['parameters'][1]['default_server_timeout'] == 500
    assert nodes['planner_server']['parameters'][1]['GridBased.tolerance'] == .1
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


@pytest.mark.parametrize('linear', [0., .019, .151, 'nan', 'inf', 'invalid'])
def test_invalid_linear_caps_fail_before_any_nodes_are_created(trial_launch, linear, monkeypatch):
    created = []
    monkeypatch.setattr(trial_launch, 'Node', lambda **kwargs: created.append(kwargs))
    with pytest.raises(ValueError):
        trial_launch._nodes(context(.15, linear))
    assert created == []


@pytest.mark.parametrize('line_speed', [0., .019, .0801, .1, .151, 'nan', 'inf', 'invalid'])
def test_invalid_line_speeds_fail_before_any_nodes_are_created(trial_launch, line_speed, monkeypatch):
    created = []
    monkeypatch.setattr(trial_launch, 'Node', lambda **kwargs: created.append(kwargs))
    with pytest.raises(ValueError):
        trial_launch._nodes(context(.15, .12, line_speed))
    assert created == []


@pytest.mark.parametrize('rotation', [.15, .3, .4])
def test_line_rotation_is_independent_of_nav2(trial_launch, rotation):
    nodes = {n['executable']: n for n in trial_launch._nodes(
        context(.45, .12, .06, rotation))}
    line = nodes['line_nav']['parameters'][0]
    assert line['rotate_speed'] == line['max_angular'] == rotation
    assert nodes['controller_server']['parameters'][1][
        'FollowPath.rotate_to_heading_angular_vel'] == .45


@pytest.mark.parametrize('rotation', [0., .149, .401, 'nan', 'inf', 'invalid'])
def test_invalid_line_rotation_fails_before_nodes(trial_launch, rotation, monkeypatch):
    created = []
    monkeypatch.setattr(trial_launch, 'Node', lambda **kw: created.append(kw))
    with pytest.raises(ValueError, match='line_rotate_speed|convert'):
        trial_launch._nodes(context(.45, line_rotate_speed=rotation))
    assert created == []


def test_close_gate_tolerance_is_explicit_and_finite(trial_launch):
    ctx = context(.15)
    ctx.launch_configurations['goal_xy_tolerance'] = '0.02'
    nodes = {n['executable']: n for n in trial_launch._nodes(ctx)}
    assert nodes['controller_server']['parameters'][1]['general_goal_checker.xy_goal_tolerance'] == .02
    for invalid in ('nan', '0.001', '0.11'):
        ctx.launch_configurations['goal_xy_tolerance'] = invalid
        with pytest.raises(ValueError):
            trial_launch._nodes(ctx)
