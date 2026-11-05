"""Inspect launch plans only; never start Gazebo, Nav2 or hardware."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
import pytest


@pytest.fixture
def launch_module(monkeypatch, tmp_path):
    path = Path(__file__).parents[1] / 'launch' / 'navigation.launch.py'
    spec = importlib.util.spec_from_file_location('smartcart_nav_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: str(tmp_path / package))
    monkeypatch.setattr(module, 'build_params', lambda *a, **kw: {'test': kw, 'amcl': {'ros__parameters': {
                            'initial_pose': dict(x=.3, y=-.3, z=0., yaw=0.)}}})
    for directory in ('roport_gz_world/maps', 'roport_gz_world/worlds'):
        (tmp_path / directory).mkdir(parents=True, exist_ok=True)
    (tmp_path / 'roport_gz_world/maps/airport_map.yaml').write_text('image: map.pgm')
    return module, tmp_path


def context(**changes):
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'mode': 'sim', 'gui': 'true', 'rviz': 'false', 'ros_domain_id': '',
        'gz_partition': '', 'map': '', 'base_params_file': '',
        'overrides_file': '', 'footprint': '', 'max_linear': '', 'max_angular': '',
        **changes})
    return ctx


def include_arguments(action, ctx):
    return {name: values if isinstance(values, str) else ''.join(s.perform(ctx) for s in values)
            for name, values in action.launch_arguments}


def test_sim_is_isolated_and_gui_optional(launch_module):
    module, _ = launch_module
    ctx = context(gui='false')
    actions = module._launch(ctx)
    env = {''.join(x.perform(ctx) for x in a.name):
           ''.join(x.perform(ctx) for x in a.value)
           for a in actions if isinstance(a, SetEnvironmentVariable)}
    assert env['ROS_DOMAIN_ID'] == '198'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
    assert env['GZ_PARTITION'].startswith('smartcart_198_')
    includes = [include_arguments(a, ctx) for a in actions
                if isinstance(a, IncludeLaunchDescription)]
    assert sum('gz_args' in a for a in includes) == 1
    assert any('--headless-rendering' in a.get('gz_args', '') for a in includes)
    nav = next(a for a in includes if 'params_file' in a)
    assert nav['use_sim_time'] == 'true' and nav['use_composition'] == 'False'
    # Nav2 interpolates these launch arguments into PythonExpression conditions.
    assert eval('not ' + nav['slam'] + ' and True', {'__builtins__': {}})
    Path(nav['params_file']).unlink()


def test_real_plan_has_no_hardware_or_gazebo_nodes(launch_module):
    module, root = launch_module
    ctx = context(mode='real', ros_domain_id='17',
                  map=str(root / 'roport_gz_world/maps/airport_map.yaml'),
                  footprint='[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]]',
                  max_linear='0.1', max_angular='0.5')
    actions = module._launch(ctx)
    assert not any(isinstance(a, Node) for a in actions)
    includes = [include_arguments(a, ctx) for a in actions
                if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1 and includes[0]['use_sim_time'] == 'false'
    Path(includes[0]['params_file']).unlink()


@pytest.mark.parametrize('changes', [
    {'mode': 'real'}, {'mode': 'invalid'}, {'gui': 'yes'},
    {'ros_domain_id': '-1'}, {'ros_domain_id': '233'},
    {'mode': 'real', 'ros_domain_id': '0'},
])
def test_invalid_plan_fails_before_creating_processes(launch_module, changes):
    module, _ = launch_module
    with pytest.raises(ValueError):
        module._launch(context(**changes))


def test_gazebo_spawn_uses_amcl_pose_including_heading(launch_module, monkeypatch):
    module, _ = launch_module
    initial = dict(x=.42, y=-.27, z=.02, yaw=.7)
    monkeypatch.setattr(module, 'build_params', lambda *a, **kw: {
        'amcl': {'ros__parameters': {'initial_pose': initial}}})
    ctx = context(gui='false')
    actions = module._launch(ctx)
    spawn = next(a for a in actions if isinstance(a, Node))
    # Inspect only the ros_gz create user arguments; do not resolve or execute
    # the executable or ROS-generated command suffix.
    arguments = [''.join(part.perform(ctx) for part in argument)
                 for argument in spawn.cmd[1:13]]
    assert arguments[:4] == ['-name', 'pinky', '-topic', '/robot_description']
    values = dict(zip(arguments[4::2], arguments[5::2]))
    assert float(values['-x']) == initial['x']
    assert float(values['-y']) == initial['y']
    assert float(values['-z']) == pytest.approx(initial['z'] + .1)
    assert float(values['-Y']) == initial['yaw']
    includes = [include_arguments(a, ctx) for a in actions
                if isinstance(a, IncludeLaunchDescription)]
    Path(next(a['params_file'] for a in includes if 'params_file' in a)).unlink()
