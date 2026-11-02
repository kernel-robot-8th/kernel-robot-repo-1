"""Scripted building-block integration with synthetic results only.

No ROS, autonomous executor, real line route, sensor or physical stop is exercised.
"""
from dataclasses import asdict

import pytest
import yaml

from roport_smartcart.goals import load_destination, map_hashes
from roport_smartcart.mission import Mission, TransitionError
from roport_smartcart.navigation import NavigationLifecycle
from roport_smartcart.velocity_gate import Source, VelocityGate


@pytest.fixture
def parts(tmp_path):
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    places = tmp_path / 'places.yaml'
    places.write_text(yaml.safe_dump({
        'map': {'frame': 'map', **map_hashes(map_yaml)},
        'places': {
            'gate1': {'measured': True, 'pose': [1, 2, 0]},
            'wait_p1': {'measured': True, 'pose': [0, 0, 0]},
        },
    }))
    gate = VelocityGate(max_linear=.1, max_angular=.5, command_timeout=.2,
                        heartbeat_timeout=1., sensor_timeout=.5, clock=lambda: 10.)
    return Mission(), NavigationLifecycle(), gate, places, map_yaml


def loaded_waiting(mission):
    token = mission.start_loading()
    # These route results are explicitly fabricated, not automatic completion.
    token = mission.complete('load_route_finished', token)
    token = mission.complete('load_confirmed', token)
    mission.complete('wait_route_finished', token)
    assert mission.state == 'WAIT_DISPATCH' and mission.loaded


def enable_nav(gate):
    epoch = gate.request_source(Source.NAV)
    assert gate.confirm_source(epoch, stopped=True, nav_cancelled=True)
    assert gate.heartbeat() and gate.sensor_update() and gate.arm()
    assert gate.submit(Source.NAV, epoch, 0, .05, 0.)
    assert gate.output() == (.05, 0.)
    return epoch


def acknowledge_synthetic_stop(gate, navigation):
    epoch = gate.request_source(Source.NONE)
    assert gate.output() == (0., 0.)
    assert not gate.confirm_source(epoch, stopped=False, nav_cancelled=not navigation.busy)
    # A scripted test acknowledgement, not evidence of a real robot stopping.
    gate.confirm_source(epoch, stopped=True, nav_cancelled=not navigation.busy)
    assert gate.source == Source.NONE and not gate.armed


def test_scripted_loaded_delivery_and_explicit_return(parts):
    mission, nav, gate, places, map_yaml = parts
    loaded_waiting(mission)
    destination = load_destination(places, map_yaml, 'gate1')
    operation = mission.dispatch('gate1')
    effects = nav.start(destination)
    token = nav.token
    assert effects == (('send_goal', token, destination),)
    nav.goal_response(token, True)
    enable_nav(gate)
    nav.result(token, 'succeeded')
    assert nav.outcome == 'succeeded' and mission.state == 'TO_DELIVERY_NAV'
    acknowledge_synthetic_stop(gate, nav)
    operation = mission.complete('delivery_reached', operation)
    assert mission.state == 'WAIT_UNLOAD' and mission.loaded
    assert gate.output() == (0., 0.)
    operation = mission.complete('unload_confirmed', operation)
    assert not mission.loaded
    home = load_destination(places, map_yaml, 'wait_p1')
    nav.start(home)
    nav.goal_response(nav.token, True)
    enable_nav(gate)
    nav.result(nav.token, 'succeeded')
    acknowledge_synthetic_stop(gate, nav)
    mission.complete('home_reached', operation)
    assert mission.state == 'IDLE_EMPTY' and mission.destination is None
    assert gate.output() == (0., 0.)


def test_cancel_and_late_success_cannot_restart_mission_or_owner(parts):
    mission, nav, gate, places, map_yaml = parts
    loaded_waiting(mission)
    destination = load_destination(places, map_yaml, 'gate1')
    old_operation = mission.dispatch('gate1')
    nav.start(destination)
    old_nav_token = nav.token
    nav.goal_response(old_nav_token, True)
    old_epoch = enable_nav(gate)
    stop_operation = mission.stop(old_operation)
    assert nav.cancel() == (('cancel_goal', old_nav_token),)
    new_epoch = gate.request_source(Source.LINE)
    nav.cancel_response(old_nav_token, True)
    assert not gate.confirm_source(new_epoch, stopped=True, nav_cancelled=not nav.busy)
    assert gate.output() == (0., 0.) and mission.state == 'STOPPING'
    # Success can race a cancel: it releases navigation, not the canceled mission.
    nav.result(old_nav_token, 'succeeded')
    assert gate.confirm_source(new_epoch, stopped=True, nav_cancelled=not nav.busy)
    mission.stopped(stop_operation)
    before = asdict(mission)
    with pytest.raises(TransitionError):
        mission.complete('delivery_reached', old_operation)
    assert asdict(mission) == before and mission.loaded
    assert mission.state == 'CANCELED'
    assert gate.source == Source.LINE and not gate.armed
    assert not gate.submit(Source.NAV, old_epoch, 1, .05, 0.)
    nav.result(old_nav_token, 'succeeded')
    assert gate.output() == (0., 0.)
    assert mission.state == 'CANCELED'
