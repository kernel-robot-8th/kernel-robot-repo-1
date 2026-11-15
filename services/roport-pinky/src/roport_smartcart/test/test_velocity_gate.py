"""Fail-closed ownership and freshness checks without ROS or hardware."""

import math

import pytest

from roport_smartcart.velocity_gate import Source, VelocityGate


@pytest.fixture
def gate():
    clock = [10.0]
    item = VelocityGate(max_linear=.1, max_angular=.5, command_timeout=.2,
                        heartbeat_timeout=1., sensor_timeout=.5,
                        clock=lambda: clock[0])
    return item, clock


def enable(g, source=Source.LINE):
    epoch = g.request_source(source)
    assert g.confirm_source(epoch, stopped=True, nav_cancelled=True)
    assert g.heartbeat()
    assert g.sensor_update()
    assert g.arm()
    return epoch


def test_default_and_arm_require_owner_and_observations(gate):
    g, _ = gate
    assert g.output() == (0., 0.)
    assert not g.arm()
    epoch = g.request_source(Source.LINE)
    assert g.confirm_source(epoch, stopped=True, nav_cancelled=False)
    assert not g.arm()
    g.heartbeat()
    assert not g.arm()
    g.sensor_update()
    assert g.arm()
    assert g.output() == (0., 0.)


def test_inactive_and_replayed_commands(gate):
    g, _ = gate
    epoch = enable(g)
    assert not g.submit('NAV', epoch, 0, .1, 0.)
    assert not g.submit('unknown', epoch, 0, .1, 0.)
    assert not g.submit('LINE', epoch - 1, 0, .1, 0.)
    assert g.submit('LINE', epoch, 0, .1, .5)
    assert not g.submit('LINE', epoch, 0, -.1, -.5)
    assert g.output() == (.1, .5)


def test_command_expires_and_cannot_replay(gate):
    g, clock = gate
    epoch = enable(g)
    assert g.submit('LINE', epoch, 1, .1, 0.)
    clock[0] += .21
    assert g.output() == (0., 0.)
    assert not g.submit('LINE', epoch, 1, .1, 0.)
    assert g.submit('LINE', epoch, 2, .1, 0.)
    g.disarm()
    assert g.output() == (0., 0.)


def test_transition_requires_stop_and_old_nav_cancellation(gate):
    g, _ = gate
    old = enable(g, Source.NAV)
    assert g.submit('NAV', old, 0, .1, 0.)
    epoch = g.request_source(Source.LINE)
    assert g.output() == (0., 0.)
    assert not g.confirm_source(old, stopped=True, nav_cancelled=True)
    assert not g.confirm_source(epoch, stopped=False, nav_cancelled=True)
    assert not g.confirm_source(epoch, stopped=True, nav_cancelled=False)
    assert not g.arm()
    # Repeated requests cannot bypass the outstanding Nav2 cancellation.
    epoch = g.request_source(Source.LINE)
    assert not g.confirm_source(epoch, stopped=True, nav_cancelled=False)
    assert g.confirm_source(epoch, stopped=True, nav_cancelled=True)
    assert not g.arm()
    g.heartbeat()
    g.sensor_update()
    assert g.arm()
    assert not g.submit('NAV', old, 1, .1, 0.)
    assert g.output() == (0., 0.)
    assert g.submit('LINE', epoch, 0, -.1, 0.)
    g.request_source(Source.NONE)
    assert g.output() == (0., 0.)
    assert not g.arm()


def test_unknown_request_also_revokes(gate):
    g, _ = gate
    epoch = enable(g)
    g.submit('LINE', epoch, 0, .1, 0.)
    with pytest.raises(ValueError):
        g.request_source('unknown')
    assert g.output() == (0., 0.)


@pytest.mark.parametrize('field,value', [('linear', math.nan),
    ('angular', math.inf), ('linear', .101), ('angular', -.501)])
def test_invalid_velocity_latches_fault(gate, field, value):
    g, _ = gate
    epoch = enable(g)
    g.submit('LINE', epoch, 0, .1, 0.)
    values = {'linear': 0., 'angular': 0., field: value}
    assert not g.submit('LINE', epoch, 1, **values)
    assert g.fault
    assert g.output() == (0., 0.)
    assert not g.submit('LINE', epoch, 2, 0., 0.)
    assert g.arm()
    assert g.output() == (0., 0.)


@pytest.mark.parametrize('elapsed', [.51, 1.01])
def test_stale_observation_latches_even_if_refreshed_before_output(gate, elapsed):
    g, clock = gate
    epoch = enable(g)
    g.submit('LINE', epoch, 0, .1, 0.)
    clock[0] += elapsed
    g.heartbeat()
    g.sensor_update()
    assert g.fault
    assert g.output() == (0., 0.)
    assert not g.submit('LINE', epoch, 1, .1, 0.)
    assert g.arm()
    assert g.output() == (0., 0.)


def test_observation_replay_future_and_stale_arm(gate):
    g, clock = gate
    enable(g)
    assert not g.heartbeat(observed_at=clock[0])
    assert not g.sensor_update(observed_at=clock[0] + 1.)
    assert not g.sensor_update(observed_at=math.nan)
    clock[0] += 2.
    assert not g.arm()
    assert g.output() == (0., 0.)


@pytest.mark.parametrize('bad_clock', [9., math.nan, math.inf])
def test_invalid_clock_fails_closed(gate, bad_clock):
    g, clock = gate
    epoch = enable(g)
    assert g.submit('LINE', epoch, 0, .1, 0.)
    clock[0] = bad_clock
    assert g.output() == (0., 0.)
    assert g.fault
    assert not g.arm()


@pytest.mark.parametrize('name', ['max_linear', 'max_angular', 'command_timeout',
                                  'heartbeat_timeout', 'sensor_timeout'])
@pytest.mark.parametrize('bad', [0., -1., math.inf, math.nan])
def test_limits_required_positive_finite(name, bad):
    arguments = dict(max_linear=.1, max_angular=.5, command_timeout=.2,
                     heartbeat_timeout=1., sensor_timeout=.5)
    arguments[name] = bad
    with pytest.raises(ValueError):
        VelocityGate(**arguments)


@pytest.mark.parametrize('stamp', [8., 11., math.nan, math.inf])
def test_expired_or_invalid_input_timestamp(gate, stamp):
    g, _ = gate
    epoch = enable(g)
    assert not g.submit('LINE', epoch, 0, .1, 0., issued_at=stamp)
    assert g.output() == (0., 0.)
