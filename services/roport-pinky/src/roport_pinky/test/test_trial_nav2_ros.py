import math

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
    assert trial.run(route_cmd(['N11', 'N19', 'N09', 'N08', 'N12'],
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
    with pytest.raises(ValueError, match='nav2_not_ready'):
        trial.driver.validate_action(navigate())
    trial.driver.nav.map_ok = True
    trial.driver.nav.amcl = None
    assert not trial.driver.nav.snapshot()['localization_ready']
