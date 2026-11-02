from dataclasses import asdict

import pytest

from roport_smartcart.mission import Mission, TransitionError


def advance_to_dispatch(mission):
    token = mission.start_loading()
    token = mission.complete('load_route_finished', token)
    assert not mission.loaded
    token = mission.complete('load_confirmed', token)
    assert mission.loaded
    return mission.complete('wait_route_finished', token)


def test_delivery_cycle_requires_explicit_cargo_confirmations():
    mission = Mission()
    advance_to_dispatch(mission)
    token = mission.dispatch('gate1')
    token = mission.complete('delivery_reached', token)
    assert mission.state == 'WAIT_UNLOAD' and mission.loaded
    token = mission.complete('unload_confirmed', token)
    assert not mission.loaded
    mission.complete('home_reached', token)
    assert mission.state == 'IDLE_EMPTY' and mission.destination is None


@pytest.mark.parametrize('failed,terminal', [(False, 'CANCELED'), (True, 'FAULT')])
def test_stop_invalidates_late_result_and_keeps_cargo(failed, terminal):
    mission = Mission()
    advance_to_dispatch(mission)
    old = mission.dispatch('gate1')
    stop_token = mission.stop(old, failed=failed)
    before = asdict(mission)
    with pytest.raises(TransitionError):
        mission.complete('delivery_reached', old)
    assert asdict(mission) == before
    assert mission.state == 'STOPPING' and mission.loaded
    mission.stopped(stop_token)
    assert mission.state == terminal and mission.loaded


def test_invalid_events_and_reentrant_dispatch_do_not_mutate():
    mission = Mission()
    token = mission.start_loading()
    for event, received in [('load_confirmed', token),
                            ('load_route_finished', 'old'), ('unknown', token)]:
        before = asdict(mission)
        with pytest.raises(TransitionError):
            mission.complete(event, received)
        assert asdict(mission) == before
    with pytest.raises(TransitionError):
        mission.dispatch('gate1')
    token = mission.complete('load_route_finished', token)
    token = mission.complete('load_confirmed', token)
    mission.complete('wait_route_finished', token)
    before = asdict(mission)
    with pytest.raises(TransitionError):
        mission.dispatch(' ')
    assert asdict(mission) == before
    mission.dispatch('gate1')
    before = asdict(mission)
    with pytest.raises(TransitionError):
        mission.dispatch('gate2')
    assert asdict(mission) == before


@pytest.mark.parametrize('loaded', [False, True])
def test_restart_requires_manual_recovery(loaded):
    mission = Mission.after_restart(loaded=loaded)
    assert mission.loaded == loaded and mission.operation is None
    before = asdict(mission)
    with pytest.raises(TransitionError):
        mission.start_loading()
    with pytest.raises(TransitionError):
        mission.dispatch('gate1')
    assert asdict(mission) == before


def test_previous_run_token_cannot_complete_new_operation():
    mission = Mission()
    first = mission.start_loading()
    second = mission.complete('load_route_finished', first)
    before = asdict(mission)
    with pytest.raises(TransitionError):
        mission.complete('load_confirmed', first)
    assert asdict(mission) == before
    mission.complete('load_confirmed', second)
