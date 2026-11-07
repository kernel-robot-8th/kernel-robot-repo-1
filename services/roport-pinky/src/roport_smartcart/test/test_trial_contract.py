"""Synthetic contract fixtures; no broker, hardware or copied deployment kit."""

from copy import deepcopy
from datetime import UTC, datetime, timedelta

import pytest

from roport_smartcart.trial_contract import (
    PROFILE, navigation_completed, navigation_destination, navigation_failed)

NOW = datetime(2026, 11, 5, 10, tzinfo=UTC)


@pytest.fixture
def command():
    return {'schema_version': '1.0', 'profile': PROFILE,
            'message_id': 'msg-1', 'type': 'navigate.requested', 'site_id': 'test-site',
            'source': 'control-test', 'target': 'pinky3', 'task_id': 'task-1',
            'step_id': 'step-1', 'command_id': 'cmd-1', 'execution_id': 'exec-1',
            'sent_at': NOW.isoformat(), 'expires_at': (NOW + timedelta(seconds=10)).isoformat(),
            'payload': {'binding_id': 'binding-test', 'expected_boot_id': 'boot-test',
                        'goal_id': 'gate1', 'goal_version': 'goal-test-v1',
                        'map_id': 'map-test', 'map_version': 'map-test-v1', 'frame_id': 'map',
                        'pose': {'x': 1., 'y': 2., 'yaw': 0.},
                        'position_tolerance_m': .1, 'yaw_tolerance_rad': .1}}


def parse(command, **changes):
    arguments = dict(validate_trial=lambda command: None,
                     profile={'profile': PROFILE, 'mode': 'simulation',
                              'map_id': 'map-test', 'map_version': 'map-test-v1'},
                     mode='sim', map_id='map-test', map_version='map-test-v1',
                     goal_versions={'gate1': 'goal-test-v1'}, now=NOW)
    return navigation_destination(command, **(arguments | changes))


@pytest.fixture
def observation():
    return {'observed_at': NOW.isoformat(), 'node_id': None, 'mode': 'nav2',
            'stopped': True, 'line_enabled': False, 'nav2_ready': True,
            'localization_ready': True, 'drive_owner': 'nav2', 'graph_version': 'graph-test',
            'map_id': 'map-test', 'map_version': 'map-test-v1', 'heading_rad': .02, 'fault': None}


def complete(command, observation, **changes):
    arguments = dict(state=observation, measured_pose={'x': 1.02, 'y': 2., 'yaw': .02},
                     measured_at=NOW.isoformat(), nav2_result='SUCCEEDED', now=NOW)
    return navigation_completed(command, **(arguments | changes))


def test_pose_and_versions_are_consumed_and_wire_validator_is_required(command):
    assert parse(command).x == 1.

    def invalid_wire(_):
        raise ValueError('wire rejected')

    with pytest.raises(ValueError, match='wire rejected'):
        parse(command, validate_trial=invalid_wire)


@pytest.mark.parametrize('change', [
    {'mode': 'real'}, {'map_version': 'different'}, {'goal_versions': {}},
    {'now': NOW + timedelta(seconds=4)}, {'now': NOW - timedelta(seconds=1)},
])
def test_stale_mode_map_and_goal_versions_are_rejected(command, change):
    with pytest.raises(ValueError):
        parse(command, **change)


def test_expired_command_and_unverified_real_profile_rejected(command):
    command['expires_at'] = (NOW + timedelta(seconds=1)).isoformat()
    with pytest.raises(ValueError, match='expired'):
        parse(command, now=NOW + timedelta(seconds=2))
    with pytest.raises(ValueError, match='field_verification'):
        parse(command, mode='real', profile={'profile': PROFILE, 'mode': 'hardware'})


def test_four_second_old_command_is_stale_even_before_ten_second_expiry(command):
    with pytest.raises(ValueError, match='stale'):
        parse(command, now=NOW + timedelta(seconds=4))


def test_completion_preserves_original_command_and_measured_pose(command, observation):
    original = deepcopy(command)
    request, phase, details = complete(command, observation)
    assert request is command and command == original and phase == 'completed'
    assert details['pose']['x'] == 1.02 != command['payload']['pose']['x']
    assert details['goal_version'] == 'goal-test-v1'
    assert details['nav2_result'] == 'SUCCEEDED'


@pytest.mark.parametrize('field,value', [
    ('stopped', False), ('line_enabled', True), ('nav2_ready', False),
    ('localization_ready', False), ('drive_owner', 'line'), ('fault', 'obstacle'),
    ('mode', 'line'), ('map_version', 'different'),
])
def test_incomplete_or_conflicting_state_cannot_report_success(command, observation, field, value):
    observation[field] = value
    with pytest.raises(ValueError):
        complete(command, observation)


@pytest.mark.parametrize('change', [
    {'nav2_result': 'ABORTED'}, {'measured_pose': {'x': 1.2, 'y': 2., 'yaw': 0.}},
    {'measured_pose': {'x': 1., 'y': 2., 'yaw': .2}},
    {'measured_at': (NOW - timedelta(seconds=4)).isoformat()},
])
def test_bad_result_outside_tolerance_or_stale_pose_rejected(command, observation, change):
    with pytest.raises(ValueError):
        complete(command, observation, **change)


def test_stale_state_and_failure_hook(command, observation):
    observation['observed_at'] = (NOW - timedelta(seconds=4)).isoformat()
    with pytest.raises(ValueError):
        complete(command, observation)
    assert navigation_failed(command, 'nav2_failed') == (
        command, 'failed', {'error_code': 'nav2_failed'})
    with pytest.raises(ValueError):
        navigation_failed(command, 'free text error')
