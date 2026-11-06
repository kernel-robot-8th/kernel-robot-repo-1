import math
import time

import pytest
from ros_trial_fixture import RosTrial
from test_trial_driver import PROFILE, TO_P7, route_cmd


def switch():
    return {'type': 'mode_switch.requested', 'payload': {
        **PROFILE, 'node_id': 'N12', 'from_mode': 'line', 'to_mode': 'nav2'}}


def navigate():
    return {'type': 'navigate.requested', 'payload': {
        **PROFILE, 'frame_id': 'map', 'goal_id': 'test', 'goal_version': 'v1',
        'pose': {'x': 2.576678, 'y': -.115347, 'yaw': .001745},
        'position_tolerance_m': .1, 'yaw_tolerance_rad': .15}}


@pytest.fixture
def trial():
    rig = RosTrial()
    try:
        yield rig
    finally:
        rig.close()


def test_real_driver_line_loading_departure_and_nav2_action(trial):
    d = trial.driver
    with pytest.raises(ValueError, match='not_at_switch_node'):
        d.validate_action(switch())
    assert trial.run(route_cmd(TO_P7, heading=-1.5690, speed=.03))[-1][1] == 'completed'
    assert trial.rig.ex.state == 'loading'
    assert abs(d.observe()['heading_rad'] - math.radians(.1)) < .1
    assert trial.run(route_cmd(['N11', 'N19', 'N24', 'N09', 'N08', 'N12'],
                              heading=math.radians(.1), speed=.03))[-1][1] == 'completed'
    assert trial.run(switch())[-1][1] == 'completed'
    result = trial.run(navigate())[-1]
    assert result[1] == 'completed', result
    assert result[2]['nav2_result'] == 'SUCCEEDED'
    assert result[2]['pose'] == pytest.approx(navigate()['payload']['pose'])


@pytest.mark.parametrize('case, expected', [('rejected', 'nav2_goal_rejected'),
    ('aborted', 'nav2_goal_failed'), ('wrong_pose', 'nav2_goal_not_reached')])
def test_nav2_negative_result_never_reports_complete(trial, case, expected):
    # Isolate Nav2 faults after the handoff; full line journey is covered above.
    trial.driver.mode = 'nav2'
    trial.accept = case != 'rejected'
    trial.succeed = case != 'aborted'
    trial.place_at_goal = case != 'wrong_pose'
    result = trial.run(navigate())[-1]
    assert result[1:] == ('failed', {'error_code': expected})
    assert trial.driver.halted == expected


def test_wrong_map_and_stale_localization_block_navigation(trial):
    trial.driver.mode = 'nav2'
    trial.grid.data[0] = 100 if trial.grid.data[0] != 100 else 0
    trial.pub_map.publish(trial.grid)
    trial.wait(lambda: not trial.driver.nav.map_ok)
    trial.wait(lambda: not trial.driver.nav.snapshot()['nav2_ready'])
    with pytest.raises(ValueError, match='nav2_not_ready'):
        trial.driver.validate_action(navigate())
    trial.driver.nav.map_ok = True
    trial.publish_amcl = False
    with trial.driver.lock:
        trial.driver.nav.amcl = None
    trial.wait(lambda: not trial.driver.nav.snapshot()['localization_ready'])


@pytest.mark.parametrize('moving, duration', [(False, 6.0), (True, 12.0)])
def test_periodic_localization_survives_stationary_and_3cm_s_sparse_amcl(trial, moving, duration):
    trial.driver.mode = 'nav2'
    trial.slow_motion = moving
    first = trial.amcl_count
    end = time.monotonic() + duration
    while time.monotonic() < end:
        trial.driver.heartbeat()
        state = trial.driver.observe()
        assert state['fault'] is None, state
        assert state['localization_ready']
        time.sleep(.05)
    assert trial.amcl_count - first <= (3 if moving else 0)
    reports = trial.localization_messages[-20:]
    assert len(reports) == 20 and all(m['localization_ready'] for m in reports)
    assert all(m['stopped'] is (not moving) for m in reports)
    assert len({m['observed_at'] for m in reports}) == 20
    assert len({m['source_stamps']['amcl_pose'] for m in reports}) <= 2
    if moving:
        assert reports[-1]['pose']['x'] > .30


@pytest.mark.parametrize('source', ['scan', 'odom', 'map_tf', 'odom_tf', 'amcl_active', 'status_timer'])
def test_fresh_heartbeat_never_hides_lost_localization_evidence(trial, source):
    trial.driver.mode = 'nav2'
    if source == 'amcl_active':
        trial.amcl_active = False
    elif source == 'status_timer':
        trial.driver.nav.pose_timer.cancel()
    else:
        setattr(trial, 'publish_' + source, False)
    trial.wait(lambda: not trial.driver.nav.snapshot()['localization_ready'], timeout=3)
    with pytest.raises(ValueError, match='nav2_not_ready'):
        trial.driver.validate_action(navigate())
