"""Physical N12 acquisition must never be replaced by a simulated tape fixture."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from roport_smartcart.goals import Destination
from roport_smartcart.real_return import Acquisition, aligned, line_route, n12_target


def reading(**changes):
    data = dict(lost=False, on_line=(False, True, False), error=0.)
    return SimpleNamespace(**(data | changes))


def test_routes_use_n12_and_correct_bay():
    from roport_line_nav.graph import Graph, plan_route
    path = Path(__file__).parents[1].parent/'roport_gz_world/layout/roport_airport.yaml'
    graph = Graph.load(path)
    for bay, expected in [('H1', ['N12', 'N08', 'N09', 'H1']),
                          ('H2', ['N12', 'N08', 'H2'])]:
        assert line_route(bay) == expected
        assert plan_route(graph, line_route(bay))[-1].action == 'park'
    with pytest.raises(ValueError):
        line_route('N12')


def test_n12_target_places_actual_centre_sensor_on_tape_facing_n08():
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph
    path = Path(__file__).parents[1].parent/'roport_gz_world/layout/roport_airport.yaml'
    data = yaml.safe_load(path.read_text())
    data['frame']['yaw_deg'] = 23.
    layout, graph = Layout(data), Graph.from_dict(data)
    target = n12_target(layout, graph)
    x, y = layout.frame.to_world(graph.nodes['N12'].x, graph.nodes['N12'].y)
    assert (target.x+.03*math.cos(target.yaw), target.y+.03*math.sin(target.yaw)) == pytest.approx((x, y))


@pytest.mark.parametrize('pose,ir,age', [
    (None, reading(), 0.),
    ({'x': .06, 'y': 0., 'yaw': 0.}, reading(), 0.),
    ({'x': 0., 'y': .06, 'yaw': 0.}, reading(), 0.),
    ({'x': 0., 'y': 0., 'yaw': .11}, reading(), 0.),
    ({'x': math.nan, 'y': 0., 'yaw': 0.}, reading(), 0.),
    ({'x': 0., 'y': 0., 'yaw': 0.}, None, 0.),
    ({'x': 0., 'y': 0., 'yaw': 0.}, reading(lost=True), 0.),
    ({'x': 0., 'y': 0., 'yaw': 0.}, reading(on_line=(True, False, False)), 0.),
    ({'x': 0., 'y': 0., 'yaw': 0.}, reading(error=.26), 0.),
    ({'x': 0., 'y': 0., 'yaw': 0.}, reading(error=math.nan), 0.),
    ({'x': 0., 'y': 0., 'yaw': 0.}, reading(), .301),
])
def test_handoff_rejects_bad_pose_or_physical_ir(pose, ir, age):
    assert not aligned(pose, Destination(0., 0., 0.), ir, age)


def test_acquisition_requires_new_continuous_samples_and_stop():
    gate = Acquisition()
    assert not gate.update(now=1., sample_time=1., good=True, stopped=True)
    assert not gate.update(now=1.6, sample_time=1., good=True, stopped=True)
    assert not gate.update(now=1.6, sample_time=1.6, good=True, stopped=True)
    assert not gate.update(now=1.8, sample_time=1.8, good=True, stopped=False)
    for t in (2., 2.1, 2.2, 2.3, 2.4):
        assert not gate.update(now=t, sample_time=t, good=True, stopped=True)
    assert gate.update(now=2.51, sample_time=2.51, good=True, stopped=True)
    assert not gate.update(now=2.6, sample_time=2.6, good=False, stopped=True)


def test_no_drive_flag_never_initializes_ros(monkeypatch):
    import roport_smartcart.real_return as module
    from roport_pinky_line.line_sensor import Calibration
    path = Path(__file__).parents[1].parent/'roport_gz_world/layout/roport_airport.yaml'
    monkeypatch.setenv('ROS_DOMAIN_ID', '13')
    target = n12_target(__import__('roport_gz_world.layout', fromlist=['Layout']).Layout.load(path),
                        __import__('roport_line_nav.graph', fromlist=['Graph']).Graph.load(path))
    monkeypatch.setattr(module, 'load_destination', lambda *a, **k:
        Destination(target.x-.25*math.cos(target.yaw), target.y-.25*math.sin(target.yaw), target.yaw)
        if a[2] == 'return_entry' else target)
    monkeypatch.setattr('roport_pinky_line.calibration.load_calibration',
                        lambda *a: Calibration((2000,)*3, (300,)*3, (1000,)*3))
    monkeypatch.setattr('rclpy.init', lambda *a, **k: pytest.fail('No-drive must not initialize ROS'))
    assert module.main(['--places', 'unused', '--map', 'unused', '--layout', str(path),
                        '--robot', 'fixture', '--calibration', 'unused', '--bay', 'H2']) == 0


def test_invalid_adc_does_not_refresh_valid_ir_watchdog():
    from roport_smartcart.real_return import HardwareReturn
    from roport_pinky_line.line_sensor import Calibration, LineSensor
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.sensor = LineSensor(Calibration((2000,)*3, (300,)*3, (1000,)*3))
    runner.reading, runner.ir_at, runner.ir_seq = None, None, 0
    runner.on_ir(SimpleNamespace(data=[2000, 300, 2000]))
    last = runner.ir_at
    runner.on_ir(SimpleNamespace(data=[0, 300, 2000]))
    assert runner.reading is None and runner.ir_at == last and runner.ir_seq == 1


@pytest.mark.parametrize('blocked,boot,age', [(True, 'original', 0.),
                                           (False, 'restarted', 0.),
                                           (False, 'original', .31)])
def test_guard_loss_revokes_owner_before_error(monkeypatch, blocked, boot, age):
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.boot = 'original'
    runner.guard = dict(boot=boot, blocked=blocked)
    monkeypatch.setattr('roport_smartcart.real_return.time.monotonic', lambda: 10.)
    runner.guard_at = 10.-age
    runner.owner = 'line'
    sent = []
    runner.lease = sent.append
    with pytest.raises(RuntimeError):
        runner.require_guard()
    assert runner.owner == 'none' and sent == ['none']


def test_shutdown_releases_line_before_cancel_and_independent_stop():
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    events = []
    runner.owner = 'line'
    runner.lease = lambda x: events.append(('lease', x))
    runner.line = SimpleNamespace(stop=lambda reason: events.append(('line_stop', reason)))
    runner.Twist = lambda: 'zero'
    runner.velocity = SimpleNamespace(publish=lambda x: events.append(('velocity', x)))
    runner.nav = SimpleNamespace(cancel=lambda: events.append(('cancel', None)),
                                 lifecycle=SimpleNamespace(busy=False))
    runner.stop_confirmed = lambda: events.append(('actual_stop_confirmed', None))
    runner.spin = lambda: None
    runner.close()
    assert runner.owner == 'none'
    assert [x[0] for x in events] == ['lease', 'line_stop', 'velocity', 'cancel', 'actual_stop_confirmed']


def test_hardware_stack_has_adc_guard_and_nav2_without_other_line_controller():
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node
    path = Path(__file__).parents[1]/'launch/real_return_stack.launch.py'
    spec = importlib.util.spec_from_file_location('real_return_stack_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description().entities
    ctx = LaunchContext()
    ctx.launch_configurations.update(robot='pinky_764e', map='/fixture/map.yaml',
        layout_file='/fixture/layout.yaml', params_file='/fixture/nav.yaml')
    for action in actions:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    factory = next(a for a in actions if isinstance(a, OpaqueFunction))
    nodes = factory.execute(ctx)
    executables = [a.node_executable for a in nodes if isinstance(a, Node)]
    assert 'trial_guard' in executables and 'controller_server' in executables
    assert 'main_node' in executables and 'line_nav' not in executables
    assert ctx.launch_configurations['ir_node'] == 'true'
    assert 'real_return' not in executables  # stack launch sends no mission goal


@pytest.mark.parametrize('lose_ir', [False, True])
def test_return_coordinator_orders_nav_stop_ir_and_line(monkeypatch, lose_ir):
    """Exercise the coordinator with peers; never create a physical ROS node."""
    from roport_smartcart.real_return import HardwareReturn
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph
    path = Path(__file__).parents[1].parent/'roport_gz_world/layout/roport_airport.yaml'
    layout, graph = Layout.load(path), Graph.load(path)
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.options = SimpleNamespace(bay='H2', layout=str(path))
    runner.target = n12_target(layout, graph)
    clock, events = [10.], []
    monkeypatch.setattr('roport_smartcart.real_return.time.monotonic', lambda: clock[0])
    runner.phase, runner.owner, runner.boot = 'READY', 'none', None
    runner.guard = dict(boot='original', owner='none', blocked=False)
    runner.guard_at = clock[0]
    runner.ir_seq, runner.ir_at, runner.reading = 1, clock[0], reading()
    public = layout.frame.to_world(200., 150.)
    state = dict(nav2_ready=True, localization_ready=True, stopped=True,
                 pose=dict(x=public[0], y=public[1], yaw=0.))
    runner.node = SimpleNamespace(get_name=lambda: 'fixture',
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(
            node_name='trial_guard' if topic == 'cmd_vel' else 'fixture')],
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))))
    quaternion = SimpleNamespace(x=0., y=0., z=0., w=1.)
    pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.), orientation=quaternion)
    odom = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
                           pose=SimpleNamespace(pose=pose))
    runner.bridge = SimpleNamespace(odom=odom)
    runner.line = SimpleNamespace(route=[], state='idle', reason='')
    runner.nav = SimpleNamespace()
    def navigate(place):
        assert runner.owner == 'none'
        events.append(place)
        if place == 'return_n12':
            state['pose'] = dict(x=runner.target.x, y=runner.target.y, yaw=runner.target.yaw)
    runner.navigate = navigate
    def spin():
        clock[0] += .05
        runner.guard_at = clock[0]
        if runner.phase != 'LINE' or not lose_ir:
            runner.ir_at = clock[0]
            runner.ir_seq += 1
        odom.header.stamp.sec = int(clock[0])
        odom.header.stamp.nanosec = int((clock[0]%1)*1e9)
        return state
    runner.spin = spin
    runner.lease = lambda owner: events.append('lease:'+owner)
    def start(route, **kwargs):
        assert events[:2] == ['return_entry', 'return_n12']
        assert runner.owner == 'none' and runner.phase == 'ACQUIRE_IR'
        runner.line.route, runner.line.state = route, 'follow'
        events.append('line_start')
    steps = [0]
    def step(*args):
        steps[0] += 1
        if steps[0] == 2:
            runner.line.state = 'arrived'
        return SimpleNamespace(linear=.03, angular=0.)
    runner.line.start, runner.line.step = start, step
    runner.Twist = lambda: SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=0.))
    runner.velocity = SimpleNamespace(publish=lambda m: events.append('line_velocity'))
    if lose_ir:
        with pytest.raises(RuntimeError, match='Physical IR unavailable'):
            runner.run()
        assert 'line_velocity' not in events
    else:
        runner.run()
        assert events.index('line_start') < events.index('lease:line') < events.index('line_velocity')
        assert runner.phase == 'ARRIVED' and runner.owner == 'none'
        assert runner.line.route == ['N12', 'N08', 'H2']


@pytest.mark.parametrize('entry', [Destination(-.3, 0., 0.), Destination(.6, 0., 0.),
                                  Destination(0., 0., 0.), Destination(0., .3, 0.)])
def test_nav2_approach_cannot_be_a_bay_or_the_wrong_side(entry):
    from roport_smartcart.real_return import validate_entry
    with pytest.raises(ValueError):
        validate_entry(entry, Destination(0., 0., math.pi))


def test_nav2_approach_accepts_public_space_side():
    from roport_smartcart.real_return import validate_entry
    validate_entry(Destination(.25, .03, math.pi), Destination(0., 0., math.pi))
