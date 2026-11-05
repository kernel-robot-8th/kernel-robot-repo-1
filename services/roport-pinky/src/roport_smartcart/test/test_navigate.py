"""Pure readiness/config tests: no ROS context or motor/action publishers."""
import pytest
import yaml

from roport_smartcart.goals import map_hashes
from roport_smartcart.navigate import Readiness, parser, validate_goal


def feed(readiness, wall, ros_time, *, stopped=True):
    readiness.tick(ros_time)
    for topic in ('scan', 'odom', 'amcl', 'tf'):
        readiness.observe(topic, ros_time, stopped=stopped if topic == 'odom' else None)


def test_sim_requires_advancing_clock_and_continuous_stop():
    wall = [10.]
    ready = Readiness('sim', clock=lambda: wall[0])
    feed(ready, wall, 1.)
    assert not ready.ready()
    wall[0] += .3
    feed(ready, wall, 1.3)
    assert not ready.ready()
    wall[0] += .3
    feed(ready, wall, 1.6)
    assert ready.ready()
    wall[0] += 1.1
    assert not ready.ready()


def test_motion_stale_odom_and_replayed_samples_cannot_prove_stop():
    wall = [10.]
    ready = Readiness('real', clock=lambda: wall[0])
    feed(ready, wall, 10.)
    wall[0] += .6
    feed(ready, wall, 10.6)
    assert ready.ready()
    wall[0] += .1
    feed(ready, wall, 10.7, stopped=False)
    assert ready.ready(require_stopped=False) and not ready.ready()
    assert not ready.observe('odom', 10.6, stopped=True)
    assert not ready.ready()
    wall[0] += 1.1
    feed(ready, wall, 11.8)
    assert not ready.ready()


def test_clock_rewind_drops_previous_stop_and_observations():
    wall = [10.]
    ready = Readiness('sim', clock=lambda: wall[0])
    for value in (10., 10.3, 10.6):
        wall[0] = value
        feed(ready, wall, value)
    assert ready.ready()
    ready.tick(1.)
    assert not ready.ready() and not ready.samples


@pytest.mark.parametrize('missing', ['scan', 'odom', 'amcl', 'tf'])
def test_each_required_input_is_checked_for_both_receive_and_ros_age(missing):
    wall = [10.]
    ready = Readiness('real', clock=lambda: wall[0])
    feed(ready, wall, 10.)
    wall[0] += .6
    feed(ready, wall, 10.6)
    assert ready.ready()
    del ready.samples[missing]
    assert not ready.ready()
    ready.observe(missing, 1., stopped=True if missing == 'odom' else None)
    assert not ready.ready()


@pytest.fixture
def configured(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\xff')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    places = tmp_path / 'places.yaml'
    places.write_text(yaml.safe_dump({
        'environment': 'sim', 'map': {'frame': 'map', **map_hashes(map_yaml)},
        'places': {'gate1': {'measured': True, 'pose': [1, 2, 0]}}
    }))
    return places, map_yaml


def test_mode_exact_match_and_map_revalidation(configured):
    places, map_yaml = configured
    assert validate_goal(places, map_yaml, 'gate1', 'sim').x == 1
    with pytest.raises(ValueError, match='environment'):
        validate_goal(places, map_yaml, 'gate1', 'real')
    map_yaml.write_text(map_yaml.read_text() + '\n')
    with pytest.raises(ValueError, match='map changed'):
        validate_goal(places, map_yaml, 'gate1', 'sim')


def test_cli_requires_explicit_mode_and_target():
    with pytest.raises(SystemExit):
        parser().parse_args(['--places', 'p', '--map', 'm', '--target', 'gate1'])
    options = parser().parse_args(['--mode', 'sim', '--places', 'p', '--map', 'm',
                                  '--target', 'gate1', '--return-to', 'home'])
    assert options.timeout == 120 and options.return_to == 'home'


def test_check_only_validates_without_importing_or_initializing_ros(configured, monkeypatch, capsys):
    import sys
    from roport_smartcart.navigate import main

    places, map_yaml = configured
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    assert main(['--mode', 'sim', '--places', str(places), '--map', str(map_yaml),
                 '--target', 'gate1', '--check-only']) == 0
    assert '"hardware_checked": false' in capsys.readouterr().out


def test_real_execution_requires_explicit_domain_before_ros_import():
    from roport_smartcart.navigate import main

    with pytest.raises(SystemExit):
        main(['--mode', 'real', '--places', 'unused', '--map', 'unused', '--target', 'gate1'])


def test_clock_mode_cannot_reuse_other_environment(configured):
    places, map_yaml = configured
    data = yaml.safe_load(places.read_text())
    data['environment'] = 'real'
    places.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='environment'):
        validate_goal(places, map_yaml, 'gate1', 'sim')


def test_nomotion_refresh_limits_rate_and_never_overlaps_pending_requests():
    from concurrent.futures import Future
    from roport_smartcart.navigate import NomotionRefresh

    class Client:
        ready = False

        def __init__(self):
            self.requests = []

        def service_is_ready(self):
            return self.ready

        def call_async(self, request):
            future = Future()
            self.requests.append((request, future))
            return future

    wall = [0.]
    client = Client()
    refresh = NomotionRefresh(client, lambda: 'empty request', clock=lambda: wall[0])
    assert not refresh.poll() and 'unavailable' in refresh.error
    client.ready = True
    assert refresh.poll() and len(client.requests) == 1
    wall[0] = 10.
    assert not refresh.poll()  # Old request pending: no duplicate even after 10 s.
    client.requests[-1][1].set_result(None)
    assert refresh.poll() and len(client.requests) == 2
    client.requests[-1][1].set_result(None)
    wall[0] = 10.5
    assert not refresh.poll()
    wall[0] = 11.
    assert refresh.poll() and len(client.requests) == 3


@pytest.mark.parametrize('topic,elapsed,expected', [
    ('amcl', 4.9, True), ('amcl', 5.1, False),
    ('scan', .9, True), ('scan', 1.1, False),
    ('odom', .9, True), ('odom', 1.1, False),
])
def test_sparse_amcl_allowed_while_scan_and_odom_require_live_receipt(topic, elapsed, expected):
    wall = [10.]
    ready = Readiness('real', clock=lambda: wall[0])
    feed(ready, wall, 10.)
    wall[0] += elapsed
    ready.tick(10. + elapsed)
    for name in ('scan', 'odom', 'amcl', 'tf'):
        if name != topic:
            ready.observe(name, ready.ros_time, stopped=False if name == 'odom' else None)
    assert ready.ready(require_stopped=False) is expected
    if not expected:
        assert any(reason.startswith(topic + ':') for reason in ready.failures(
            require_stopped=False))


def test_newly_received_old_amcl_pose_still_expires_by_measurement_time():
    wall = [10.]
    ready = Readiness('real', clock=lambda: wall[0])
    ready.tick(20.)
    for topic in ('scan', 'odom', 'tf'):
        ready.observe(topic, 20., stopped=False if topic == 'odom' else None)
    wall[0] += .1
    ready.tick(20.1)
    # A freshly delivered pose from an old processing backlog must not pass
    # merely because AMCL has a longer allowance than high-rate sensor topics.
    assert ready.observe('amcl', 15.)
    assert not ready.ready(require_stopped=False)
    assert any(reason.startswith('amcl:') for reason in ready.failures(require_stopped=False))
