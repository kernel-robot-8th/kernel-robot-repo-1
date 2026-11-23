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
                                    params_file='/unused/params.yaml', ir_node='false', goal_xy_tolerance='0.08',
                                    planner_tolerance='0.1')
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


def test_hardware_trial_preserves_base_tracking_and_collision_checks(trial_launch):
    nodes = {n['executable']: n for n in trial_launch._nodes(context(.15, .08))}
    controller = nodes['controller_server']['parameters'][1]
    assert controller['FollowPath.use_collision_detection'] is True
    assert not any('lookahead' in key for key in controller)
    assert not any('inflation' in key or 'footprint' in key for key in controller)


def test_docking_controller_is_opt_in_and_retains_collision_and_speed_caps(trial_launch, monkeypatch):
    monkeypatch.delenv('ROPORT_GATE_DOCK_CONFIG', raising=False)
    monkeypatch.delenv('ROPORT_FINE_ALIGN_GOALS', raising=False)
    normal = {n['executable']: n for n in trial_launch._nodes(context(.15))}
    assert 'GateDock.plugin' not in normal['controller_server']['parameters'][1]
    monkeypatch.setenv('ROPORT_GATE_DOCK_CONFIG', '/unused/local-calibrated.yaml')
    dock = {n['executable']: n for n in trial_launch._nodes(context(.15))}
    cfg = dock['controller_server']['parameters'][1]
    assert cfg['GateDock.desired_linear_vel'] == .02
    assert cfg['GateDock.use_collision_detection'] is True
    assert cfg['GateDock.allow_reversing'] is False
    assert cfg['gate_dock_goal_checker.xy_goal_tolerance'] == .01


@pytest.mark.parametrize('goals, enabled', [(None, False), ('', False), (' , ', False),
                                             ('N12_RETURN', True), (' N12_RETURN , G3', True)])
def test_fine_align_goals_also_enable_dock_controller(trial_launch, monkeypatch, goals, enabled):
    monkeypatch.delenv('ROPORT_GATE_DOCK_CONFIG', raising=False)
    if goals is None:
        monkeypatch.delenv('ROPORT_FINE_ALIGN_GOALS', raising=False)
    else:
        monkeypatch.setenv('ROPORT_FINE_ALIGN_GOALS', goals)
    cfg = {n['executable']: n for n in trial_launch._nodes(context(.15))}['controller_server']['parameters'][1]
    assert ('GateDock.plugin' in cfg) is enabled
    if not enabled:
        assert not any(k.startswith(('GateDock', 'gate_dock')) for k in cfg) and 'controller_plugins' not in cfg
        return
    assert cfg['controller_plugins'] == ['FollowPath', 'GateDock']
    assert cfg['goal_checker_plugins'] == ['general_goal_checker', 'gate_dock_goal_checker']
    assert cfg['gate_dock_goal_checker.xy_goal_tolerance'] == .01
    assert cfg['gate_dock_goal_checker.yaw_goal_tolerance'] == .05
    assert cfg['GateDock.desired_linear_vel'] == .02 and cfg['GateDock.use_collision_detection'] is True
    # 마커 없는 정밀 접근에만 새 속도(2026-11-23 d660: 20 cm 접근 25 s 시간 초과 → 19 s)
    assert cfg['GateDock.min_approach_linear_velocity'] == .01
    assert cfg['GateDock.regulated_linear_scaling_min_speed'] == .015
    assert cfg['GateDock.approach_velocity_scaling_dist'] == .05


@pytest.mark.parametrize('fine_align_goals', [None, 'N12_RETURN'])
def test_gate_marker_docking_keeps_its_previous_speeds(trial_launch, monkeypatch, fine_align_goals):
    """게이트 마커 주차(ROPORT_GATE_DOCK_CONFIG)는 변경 전 감속 값을 그대로 쓴다. FineAlign 환경 변수가 같이 있어도
    드라이버처럼 게이트 마커 주차가 우선이라 새 속도를 넣지 않는다. 감속 거리는 RPP 기본값(0.6 m)을 명시해
    params_file 이 FineAlign 용 값을 가진 파일이어도 바뀌지 않는다."""
    monkeypatch.setenv('ROPORT_GATE_DOCK_CONFIG', '/unused/local-calibrated.yaml')
    if fine_align_goals is None:
        monkeypatch.delenv('ROPORT_FINE_ALIGN_GOALS', raising=False)
    else:
        monkeypatch.setenv('ROPORT_FINE_ALIGN_GOALS', fine_align_goals)
    cfg = {n['executable']: n for n in trial_launch._nodes(context(.15))}['controller_server']['parameters'][1]
    assert cfg['GateDock.desired_linear_vel'] == .02
    assert cfg['GateDock.min_approach_linear_velocity'] == .005
    assert cfg['GateDock.regulated_linear_scaling_min_speed'] == .005
    assert cfg['GateDock.approach_velocity_scaling_dist'] == .6
    assert cfg['GateDock.use_collision_detection'] is True and cfg['GateDock.allow_reversing'] is False
    assert cfg['gate_dock_goal_checker.xy_goal_tolerance'] == .01
    assert cfg['gate_dock_goal_checker.yaw_goal_tolerance'] == .05


def test_fine_align_controller_values_match_real_params_yaml(trial_launch):
    """nav2_follow_real.yaml 의 GateDock 은 마커 없는 정밀 접근 값이다: launch 의 공통 값 + FineAlign 값과 같아야 한다."""
    import yaml
    dock = {**trial_launch.DOCK_CONTROLLER_OVERRIDES, **trial_launch.FINE_ALIGN_CONTROLLER_OVERRIDES}
    assert dock['GateDock.min_approach_linear_velocity'] == .01
    assert dock['GateDock.regulated_linear_scaling_min_speed'] == .015
    assert dock['GateDock.approach_velocity_scaling_dist'] == .05
    params = yaml.safe_load((Path(__file__).parents[1] / 'config/nav2_follow_real.yaml').read_text())
    controller = params['controller_server']['ros__parameters']
    for key, value in dock.items():
        if key.startswith('GateDock.'):
            assert controller['GateDock'][key.split('.', 1)[1]] == value, key
        elif key.startswith('gate_dock_goal_checker.'):
            assert controller['gate_dock_goal_checker'][key.split('.', 1)[1]] == value, key
        else:
            assert controller[key] == value, key


def test_planner_tolerance_default_and_follow_value(trial_launch):
    """기본 0.1 m(공공 공간 목표가 지도 끝 근처) 그대로, 추종 시험은 0.3 m(11-20 실로봇: 사이에 선 인형 위 목표)."""
    ctx = context(.15)
    nodes = {n['executable']: n for n in trial_launch._nodes(ctx)}
    assert nodes['planner_server']['parameters'][1]['GridBased.tolerance'] == .1
    ctx.launch_configurations['planner_tolerance'] = '0.3'
    nodes = {n['executable']: n for n in trial_launch._nodes(ctx)}
    assert nodes['planner_server']['parameters'][1]['GridBased.tolerance'] == .3
    for invalid in ('nan', '0.01', '0.5'):
        ctx.launch_configurations['planner_tolerance'] = invalid
        with pytest.raises(ValueError):
            trial_launch._nodes(ctx)
