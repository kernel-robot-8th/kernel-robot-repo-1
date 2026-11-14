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
                        '--robot', 'fixture', '--calibration', 'unused', '--bay', 'H2',
                        '--domain-id', '13']) == 0


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


@pytest.mark.parametrize('blocked', [False, True])
def test_spin_processes_queued_guard_before_renewing_owner(monkeypatch, blocked):
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.node, runner.owner, runner.last_status = object(), 'nav2', 10.
    calls, leases = [], []
    observed = {'fresh': False}

    def spin_once(node, *, timeout_sec):
        calls.append(timeout_sec)
        if len(calls) == 4:
            observed['fresh'] = not blocked

    monkeypatch.setattr('rclpy.spin_once', spin_once)
    monkeypatch.setattr('roport_smartcart.real_return.time.monotonic', lambda: 10.)
    runner.guard_ok = lambda: observed['fresh']
    runner.lease = leases.append
    runner.nav = SimpleNamespace(poll=lambda: None)
    runner.bridge = SimpleNamespace(snapshot=lambda: {'stopped': True})
    assert runner.spin() == {'stopped': True}
    assert calls == [.01] + [0.] * 8
    assert leases == ['none' if blocked else 'nav2']


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


@pytest.mark.parametrize('saved_tolerance', [None, .08])
def test_shutdown_releases_line_before_cancel_and_independent_stop(saved_tolerance):
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.saved_goal_tolerance = saved_tolerance
    events = []
    runner.owner = 'line'
    runner.lease = lambda x: events.append(('lease', x))
    runner.line = SimpleNamespace(stop=lambda reason: events.append(('line_stop', reason)))
    runner.Twist = lambda: 'zero'
    runner.velocity = SimpleNamespace(publish=lambda x: events.append(('velocity', x)))
    runner.nav = SimpleNamespace(cancel=lambda: events.append(('cancel', None)),
                                 lifecycle=SimpleNamespace(busy=False))
    runner.stop_confirmed = lambda: events.append(('actual_stop_confirmed', None))
    runner.set_goal_tolerance = lambda value: events.append(('restored_tolerance', value))
    runner.spin = lambda: None
    runner.close()
    assert runner.owner == 'none'
    expected = ['lease', 'line_stop', 'velocity', 'cancel', 'actual_stop_confirmed']
    assert [x[0] for x in events] == expected + (['restored_tolerance'] if saved_tolerance else [])
    assert runner.saved_goal_tolerance is None


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
    assert ctx.launch_configurations['planner_plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert ctx.launch_configurations['recovery_bt'].endswith('bounded_field_recovery.xml')
    assert 'real_return' not in executables  # stack launch sends no mission goal
    from launch_ros.utilities import evaluate_parameters
    controller = next(a for a in nodes if isinstance(a, Node) and a.node_executable == 'controller_server')
    dictionaries = [p for p in controller._Node__parameters if isinstance(p, dict)]
    assert evaluate_parameters(ctx, dictionaries)[0]['general_goal_checker.xy_goal_tolerance'] == .08


def test_field_recovery_bounds_retreat_and_keeps_both_collision_costmaps():
    import xml.etree.ElementTree as ET
    tree = ET.parse(Path(__file__).parents[1]/'config/bounded_field_recovery.xml')
    assert tree.find('.//RecoveryNode').get('number_of_retries') == '2'
    backup = tree.find('.//BackUp')
    assert float(backup.get('backup_dist')) == -.08
    assert float(backup.get('backup_speed')) == .02
    allowance = float(backup.get('time_allowance'))
    assert .08/(.02*.3)+1 <= allowance <= 18  # retain Collision Monitor's 30% slowdown
    assert float(tree.find('.//Timeout').get('msec'))/1000 > allowance+1
    assert tree.find('.//Spin') is None
    assert len(tree.findall('.//ClearEntireCostmap')) == 2


@pytest.mark.parametrize('first_terminal_x', [.04, .015])
def test_n12_subcell_path_keeps_exact_endpoint_and_restores_tolerance(monkeypatch, first_terminal_x):
    from builtin_interfaces.msg import Time
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.target = Destination(.04, 0., 0.)
    runner.reading = runner.ir_at = None
    runner.fine_send = runner.fine_handle = runner.fine_client = None
    runner.owner = 'none'
    start = dict(localization_ready=True, pose=dict(x=0., y=0., yaw=0.))
    end = dict(localization_ready=True, pose=dict(x=.04, y=0., yaw=0.))
    intermediate = dict(localization_ready=True, pose=dict(x=first_terminal_x, y=0., yaw=0.))
    states = iter([start, start, end, end] if first_terminal_x == .04 else
                  [start, start, intermediate, intermediate, end, end])
    runner.stop_confirmed = lambda: next(states)
    runner.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))))
    tolerances, goals, destroyed = [], [], []
    runner.set_goal_tolerance = tolerances.append
    runner.lease = lambda owner: None
    runner.require_guard = lambda: None
    done = lambda value: SimpleNamespace(done=lambda: True, result=lambda: value)
    runner.parameter_result = lambda future: future.result()
    runner.goal_parameters = SimpleNamespace(get_parameters=lambda names: done(
        SimpleNamespace(values=[SimpleNamespace(type=3, double_value=.08)])))
    result = SimpleNamespace(status=4, result=SimpleNamespace(error_code=0))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: done(result))

    class Client:
        def __init__(self, *args):
            pass

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            goals.append(goal)
            return done(handle)

        def destroy(self):
            destroyed.append(True)

    monkeypatch.setattr('rclpy.action.ActionClient', Client)
    runner.align_n12()
    assert len(goals) == (1 if first_terminal_x == .04 else 2)
    if len(goals) == 2:
        assert goals[1].path.poses[0].pose.position.x == first_terminal_x
    assert goals[0].path.poses[-1].pose.position.x == .04
    assert goals[0].path.poses[-1].pose.position.y == 0.
    assert goals[0].controller_id == 'FollowPath'
    assert tolerances == [.01, .08]
    assert runner.owner == 'none' and runner.fine_client is None and destroyed


def test_fine_alignment_cancel_works_without_fresh_guard():
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.owner = 'none'
    events = []
    pending = {'done': False}
    terminal = SimpleNamespace(done=lambda: pending['done'], result=lambda: object())
    ack = SimpleNamespace(done=lambda: True, result=lambda: object())
    runner.fine_handle = SimpleNamespace(accepted=True,
        cancel_goal_async=lambda: events.append('cancel') or ack,
        get_result_async=lambda: terminal)
    runner.fine_send = object()
    def spin():
        assert runner.owner == 'none'
        events.append('spin')
        pending['done'] = True
    runner.spin = spin
    runner.cancel_fine_alignment()
    assert events == ['cancel', 'spin']
    assert runner.fine_send is runner.fine_handle is None


@pytest.mark.parametrize('lose_ir', [False, True])
@pytest.mark.parametrize('from_n12,bad_pose', [(False, False), (True, False), (True, True)])
def test_return_coordinator_orders_nav_stop_ir_and_line(monkeypatch, lose_ir, from_n12, bad_pose):
    """Exercise the coordinator with peers; never create a physical ROS node."""
    from roport_smartcart.real_return import HardwareReturn
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph
    path = Path(__file__).parents[1].parent/'roport_gz_world/layout/roport_airport.yaml'
    layout, graph = Layout.load(path), Graph.load(path)
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.options = SimpleNamespace(bay='H2', layout=str(path), from_n12=from_n12)
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
    if from_n12:
        state['pose'] = dict(x=runner.target.x+(.06 if bad_pose else 0.),
                             y=runner.target.y, yaw=runner.target.yaw)
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
    runner.align_n12 = lambda: events.append('fine_align')
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
        if from_n12:
            assert 'return_entry' not in events and 'return_n12' not in events
        else:
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
    if bad_pose:
        with pytest.raises(RuntimeError, match='N12 physical tape not acquired'):
            runner.run()
        assert 'line_start' not in events and 'line_velocity' not in events
    elif lose_ir:
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


@pytest.mark.parametrize('destination,terminal,phase', [('N12', 'handoff', 'AT_N12'),
                                                      ('H2', 'arrived', 'ARRIVED')])
def test_shared_line_runner_distinguishes_n12_handoff_from_parking(destination, terminal, phase):
    import time
    from roport_smartcart.real_return import HardwareReturn
    r = HardwareReturn.__new__(HardwareReturn)
    r.owner, r.ir_seq, r.ir_at, r.reading = 'none', 0, time.monotonic(), reading()
    events = []
    r.require_guard = lambda: None
    r.stop_confirmed = lambda: events.append('actual_stop')
    r.line = SimpleNamespace(state='idle', reason='')
    r.line.start = lambda route, **kwargs: events.append(route)
    def step(*args):
        r.line.state = terminal
    r.line.step = step
    nanoseconds = time.time_ns()
    stamp = SimpleNamespace(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)
    pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                           orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    r.bridge = SimpleNamespace(odom=SimpleNamespace(header=SimpleNamespace(stamp=stamp),
                                                   pose=SimpleNamespace(pose=pose)))
    r.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=time.time_ns())))
    def spin():
        r.ir_seq += 1
        r.ir_at = time.monotonic()
    r.spin = spin
    r.lease = lambda owner: events.append(owner)
    route = ['H2', 'N08', 'N12'] if destination == 'N12' else ['N12', 'N08', 'H2']
    r.follow_line(route)
    assert r.phase == phase and r.owner == 'none'
    assert events == ['actual_stop', route, 'none', 'actual_stop']


@pytest.mark.parametrize('place,expected', [('return_entry', []), ('return_n12', [.01, .08])])
def test_only_n12_changes_goal_tolerance_and_restores_after_actual_stop(monkeypatch, place, expected):
    import roport_smartcart.real_return as module
    runner = module.HardwareReturn.__new__(module.HardwareReturn)
    runner.saved_goal_tolerance = None
    runner.options = SimpleNamespace(places='field', map='map')
    runner.require_guard = lambda: None
    runner.parameter_result = lambda result: result
    runner.goal_parameters = SimpleNamespace(get_parameters=lambda names: SimpleNamespace(
        values=[SimpleNamespace(type=3, double_value=.08)]))
    events = []
    runner.set_goal_tolerance = lambda value: events.append(value)
    runner.nav = SimpleNamespace(start=lambda *args: None,
        lifecycle=SimpleNamespace(busy=False, outcome='succeeded'))
    runner.lease = lambda owner: None
    runner.stop_confirmed = lambda: (events.append('stopped') or
        {'localization_ready': True, 'pose': {'x': 1., 'y': 2., 'yaw': 0.}})
    monkeypatch.setattr(module, 'load_destination', lambda *a, **k: Destination(1., 2., 0.))
    runner.navigate(place)
    assert [event for event in events if isinstance(event, float)] == expected
    if expected:
        assert events == [.01, 'stopped', .08]
    assert runner.saved_goal_tolerance is None


@pytest.mark.parametrize('results', [[], [SimpleNamespace(successful=False)]])
def test_goal_tolerance_update_rejection_is_not_ignored(results):
    from roport_smartcart.real_return import HardwareReturn
    runner = HardwareReturn.__new__(HardwareReturn)
    runner.parameter_result = lambda response: response
    runner.goal_parameters = SimpleNamespace(set_parameters=lambda parameters: SimpleNamespace(results=results))
    with pytest.raises(RuntimeError, match='update rejected'):
        runner.set_goal_tolerance(.01)


@pytest.mark.parametrize('domain,environment', [(13,'16'), (16,'13'), (233,'233'), (-1,'-1')])
def test_hardware_domain_must_be_explicit_valid_and_match_environment(monkeypatch, domain, environment):
    from roport_smartcart.real_return import main
    monkeypatch.setenv('ROS_DOMAIN_ID', environment)
    with pytest.raises(SystemExit) as exc:
        main(['--domain-id', str(domain), '--places','unused','--map','unused',
              '--layout','unused','--robot','fixture','--calibration','unused','--bay','H2'])
    assert exc.value.code == 2


@pytest.mark.parametrize('offset,angle,valid', [(.036, .025, True), (.051, 0., False),
                                              (0., .101, False)])
def test_physical_handoff_calibration_keeps_original_acquisition_envelope(offset, angle, valid):
    from roport_smartcart.real_return import measured_handoff
    nominal = Destination(0., 0., 0.)
    measured = Destination(offset, 0., angle)
    if valid:
        assert measured_handoff(measured, nominal) == measured
    else:
        with pytest.raises(ValueError):
            measured_handoff(measured, nominal)
