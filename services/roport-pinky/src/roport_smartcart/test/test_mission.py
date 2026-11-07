from dataclasses import asdict

import pytest

from roport_smartcart.mission import Mission, TransitionError


def advance_to_dispatch(mission):
    token = mission.start_loading()
    token = mission.complete('load_route_finished', token)
    assert not mission.loaded
    mission.assign_wait_bay('wait_p1', token)
    token = mission.complete('load_confirmed', token)
    assert mission.loaded
    return mission.complete('wait_route_finished', token, place_id='wait_p1')


def test_delivery_cycle_requires_explicit_cargo_confirmations():
    mission = Mission()
    advance_to_dispatch(mission)
    token = mission.dispatch('gate1')
    token = mission.complete('delivery_reached', token)
    assert mission.state == 'WAIT_UNLOAD' and mission.loaded
    mission.assign_wait_bay('wait_p1', token)
    token = mission.complete('unload_confirmed', token)
    assert not mission.loaded
    mission.complete('home_reached', token, place_id='wait_p1')
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
    mission.assign_wait_bay('wait_p1', token)
    token = mission.complete('load_confirmed', token)
    mission.complete('wait_route_finished', token, place_id='wait_p1')
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
    mission.assign_wait_bay('wait_p2', second)
    mission.complete('load_confirmed', second)


@pytest.mark.parametrize('bay', ['wait_p1', 'wait_p2'])
def test_return_requires_fresh_reservation_and_matching_arrival(bay):
    mission = Mission()
    advance_to_dispatch(mission)
    token = mission.dispatch('gate1')
    assert mission.wait_bay is None
    token = mission.complete('delivery_reached', token)
    before = asdict(mission)
    with pytest.raises(TransitionError, match='reserved'):
        mission.complete('unload_confirmed', token)
    assert asdict(mission) == before and mission.loaded
    for invalid in ('home', 'gate1', None):
        with pytest.raises(TransitionError):
            mission.assign_wait_bay(invalid, token)
        assert asdict(mission) == before
    with pytest.raises(TransitionError):
        mission.assign_wait_bay(bay, 'stale')
    mission.assign_wait_bay(bay, token)
    token = mission.complete('unload_confirmed', token)
    before = asdict(mission)
    for wrong in (None, 'home', 'wait_p2' if bay == 'wait_p1' else 'wait_p1'):
        with pytest.raises(TransitionError, match='arrival'):
            mission.complete('home_reached', token, place_id=wrong)
        assert asdict(mission) == before
    mission.complete('home_reached', token, place_id=bay)
    assert mission.state == 'IDLE_EMPTY' and not mission.loaded


def test_loaded_return_requires_assigned_bay_and_matching_route_result():
    mission = Mission()
    token = mission.complete('load_route_finished', mission.start_loading())
    before = asdict(mission)
    with pytest.raises(TransitionError, match='reserved'):
        mission.complete('load_confirmed', token)
    assert asdict(mission) == before
    mission.assign_wait_bay('wait_p2', token)
    token = mission.complete('load_confirmed', token)
    before = asdict(mission)
    with pytest.raises(TransitionError, match='arrival'):
        mission.complete('wait_route_finished', token, place_id='wait_p1')
    assert asdict(mission) == before and mission.loaded
    mission.complete('wait_route_finished', token, place_id='wait_p2')
    assert mission.state == 'WAIT_DISPATCH'
