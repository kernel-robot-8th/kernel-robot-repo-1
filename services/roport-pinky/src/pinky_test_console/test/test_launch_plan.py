import importlib.util
import os

import pytest

from pinky_test_console.launch_plan import (
    LaunchArgError,
    included_launch,
    parse_bool,
    resolve_plan,
)

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch', 'console.launch.py')


@pytest.fixture
def map_yaml(tmp_path):
    p = tmp_path / 'room.yaml'
    p.write_text('image: room.pgm\n')
    return str(p)


def test_slam_default():
    plan = resolve_plan('slam')
    assert plan.mode == 'slam'
    assert plan.map_yaml == ''
    assert plan.port == 8080
    assert plan.use_sim_time is False


def test_slam_ignores_map(map_yaml):
    assert resolve_plan('slam', map_path=map_yaml).map_yaml == ''


def test_mode_is_case_insensitive(map_yaml):
    assert resolve_plan(' SLAM ').mode == 'slam'
    assert resolve_plan('Nav', map_path=map_yaml).mode == 'nav'


def test_nav_with_map(map_yaml):
    plan = resolve_plan('nav', map_path=map_yaml, port='9090', use_sim_time='true')
    assert plan.mode == 'nav'
    assert plan.map_yaml == map_yaml
    assert plan.port == 9090
    assert plan.use_sim_time is True


def test_nav_map_tilde_expanded(tmp_path, monkeypatch, map_yaml):
    monkeypatch.setenv('HOME', str(tmp_path))
    assert resolve_plan('nav', map_path='~/room.yaml').map_yaml == map_yaml


@pytest.mark.parametrize('mode', ['', 'mapping', 'navigation', 'slam;nav', 'nav2'])
def test_invalid_mode(mode):
    with pytest.raises(LaunchArgError, match='mode must be one of'):
        resolve_plan(mode)


def test_nav_requires_map():
    with pytest.raises(LaunchArgError, match='requires map'):
        resolve_plan('nav')
    with pytest.raises(LaunchArgError, match='requires map'):
        resolve_plan('nav', map_path='   ')


def test_nav_map_missing(tmp_path):
    with pytest.raises(LaunchArgError, match='not found'):
        resolve_plan('nav', map_path=str(tmp_path / 'nope.yaml'))


def test_nav_map_not_yaml(tmp_path):
    pgm = tmp_path / 'room.pgm'
    pgm.write_bytes(b'P5')
    with pytest.raises(LaunchArgError, match='.yaml'):
        resolve_plan('nav', map_path=str(pgm))


@pytest.mark.parametrize('port', ['0', '65536', '-1', 'http', '80.5', ''])
def test_invalid_port(port):
    with pytest.raises(LaunchArgError, match='port'):
        resolve_plan('slam', port=port)


@pytest.mark.parametrize('text, value', [
    ('true', True), ('True', True), ('1', True), ('false', False), ('False', False), ('', False)])
def test_parse_bool(text, value):
    assert parse_bool(text) is value


def test_parse_bool_invalid():
    with pytest.raises(LaunchArgError):
        resolve_plan('slam', use_sim_time='maybe')


# ---------------- include 결정 ----------------

def test_included_launch_slam():
    assert included_launch(resolve_plan('slam')) == (
        'map_building.launch.xml', {'use_sim_time': 'False'})


def test_included_launch_nav(map_yaml):
    plan = resolve_plan('nav', map_path=map_yaml, use_sim_time='true')
    assert included_launch(plan) == (
        'bringup_launch.xml', {'map': map_yaml, 'use_sim_time': 'True'})


def test_ir_mode_plan():
    plan = resolve_plan('ir', map_path='/ignored.yaml')
    assert plan.mode == 'ir' and plan.map_yaml == '' and plan.ir_node is True
    assert included_launch(plan) == (None, {})
    assert resolve_plan('ir', ir_node='false').ir_node is False


def test_ir_node_only_on_request_in_other_modes():
    assert resolve_plan('slam').ir_node is False
    assert resolve_plan('slam', ir_node='true').ir_node is True
    with pytest.raises(LaunchArgError, match='ir_node'):
        resolve_plan('slam', ir_node='maybe')


def test_robot_bringup_never_included(map_yaml):
    for plan in (resolve_plan('slam'), resolve_plan('nav', map_path=map_yaml)):
        name, _ = included_launch(plan)
        assert 'bringup_robot' not in name


# ---------------- console.launch.py 자체 ----------------

class _FakeInclude:
    def __init__(self, source, launch_arguments=()):
        self.source = source
        self.launch_arguments = dict(launch_arguments)


def _run_setup(monkeypatch, **args):
    """console.launch.py 의 launch_setup 을 실제 LaunchContext 로 실행.

    pinky_navigation 이 설치되지 않은 PC 에서도 돌도록 share 경로와 include 만 가짜로 바꾼다.
    """
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    spec = importlib.util.spec_from_file_location('console_launch', LAUNCH_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, 'get_package_share_directory', lambda pkg: f'/fake/share/{pkg}')
    monkeypatch.setattr(mod, 'AnyLaunchDescriptionSource', lambda path: path)
    monkeypatch.setattr(mod, 'IncludeLaunchDescription', _FakeInclude)

    ctx = LaunchContext()
    for entity in mod.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = entity.default_value[0].text if entity.default_value else ''
            ctx.launch_configurations[entity.name] = args.get(entity.name, default)
    return mod.launch_setup(ctx)


def _parts(actions):
    from launch_ros.actions import Node
    nodes = [a for a in actions if isinstance(a, Node)]
    incs = [a for a in actions if isinstance(a, _FakeInclude)]
    assert len(nodes) == 1 and len(incs) == 1
    return nodes[0], incs[0]


def test_launch_slam(monkeypatch):
    node, inc = _parts(_run_setup(monkeypatch, mode='slam'))
    assert node.node_package == 'pinky_test_console'
    assert node.node_executable == 'web_server'
    assert inc.source == '/fake/share/pinky_navigation/launch/map_building.launch.xml'
    assert inc.launch_arguments == {'use_sim_time': 'False'}


def test_launch_nav(monkeypatch, map_yaml):
    _, inc = _parts(_run_setup(monkeypatch, mode='nav', map=map_yaml, use_sim_time='true'))
    assert inc.source == '/fake/share/pinky_navigation/launch/bringup_launch.xml'
    assert inc.launch_arguments == {'map': map_yaml, 'use_sim_time': 'True'}


def test_launch_rejects_bad_mode(monkeypatch):
    with pytest.raises(LaunchArgError, match='mode must be one of'):
        _run_setup(monkeypatch, mode='explore')


def test_launch_nav_without_map_fails(monkeypatch):
    with pytest.raises(LaunchArgError, match='requires map'):
        _run_setup(monkeypatch, mode='nav')


def test_launch_bad_port_fails(monkeypatch):
    with pytest.raises(LaunchArgError, match='port'):
        _run_setup(monkeypatch, mode='slam', port='99999')


def test_launch_ir_runs_sensor_node_without_include(monkeypatch):
    from launch_ros.actions import Node
    actions = _run_setup(monkeypatch, mode='ir')
    nodes = [(a.node_package, a.node_executable) for a in actions if isinstance(a, Node)]
    assert nodes == [('pinky_test_console', 'web_server'), ('pinky_sensor_adc', 'main_node')]
    assert not [a for a in actions if isinstance(a, _FakeInclude)]


def test_launch_ir_node_disabled(monkeypatch):
    from launch_ros.actions import Node
    actions = _run_setup(monkeypatch, mode='ir', ir_node='false')
    assert [a.node_executable for a in actions if isinstance(a, Node)] == ['web_server']
