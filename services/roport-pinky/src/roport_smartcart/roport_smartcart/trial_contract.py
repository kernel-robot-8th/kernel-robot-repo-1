"""Pure trial-v1 navigation hooks; the supplied kit owns wire validation/MQTT.

No dispatch, mode switch or readiness is fabricated here. Runtime integration
must supply measured map identity, localization and exclusive drive ownership.
"""

from datetime import UTC, datetime
import math
import re

from .goals import Destination

PROFILE = 'roport-pinky-driving-trial-v1'
_ID = re.compile(r'[A-Za-z0-9_.:-]{1,128}\Z')
_STATE = ('node_id', 'mode', 'stopped', 'line_enabled', 'nav2_ready',
          'localization_ready', 'drive_owner', 'graph_version', 'map_id',
          'map_version', 'heading_rad', 'fault')


def _fresh(stamp, now):
    observed = datetime.fromisoformat(stamp)
    if observed.tzinfo is None or now.tzinfo is None:
        raise ValueError('timestamp_requires_offset')
    if not -.5 <= (now - observed).total_seconds() <= 3:
        raise ValueError('stale_observation_or_command')
    return observed


def navigation_destination(command, *, validate_trial, profile, mode, map_id,
                           map_version, goal_versions, now=None):
    """Validate a received coordinate command against local runtime identity.

    ``validate_trial`` must be the deployment kit's trial_wire.validate_trial.
    goal_versions is the locally approved {goal_id: goal_version} registry.
    These checks do not replace the kit's binding, boot, journal or owner guards.
    """
    validate_trial(command)
    now = now or datetime.now(UTC)
    if command['type'] != 'navigate.requested' or command['profile'] != PROFILE:
        raise ValueError('unsupported_operation')
    if profile.get('profile') != PROFILE or mode not in ('sim', 'real') or (
            profile.get('mode') != {'sim': 'simulation', 'real': 'hardware'}[mode]):
        raise ValueError('runtime_mode_mismatch')
    if mode == 'real' and profile.get('field_verified') is not True:
        raise ValueError('field_verification_required')
    # Deployment kit trial_wire.fresh applies this 3 s sent_at limit in addition
    # to expires_at (Pinky trial guide 19988482, section 5).
    _fresh(command['sent_at'], now)
    expiry = datetime.fromisoformat(command['expires_at'])
    if expiry.tzinfo is None or now >= expiry:
        raise ValueError('expired_command')
    p = command['payload']
    if any(p[k] != value or profile.get(k) != value for k, value in
           (('map_id', map_id), ('map_version', map_version))):
        raise ValueError('map_mismatch')
    if goal_versions.get(p['goal_id']) != p['goal_version']:
        raise ValueError('goal_version_mismatch')
    return Destination(**p['pose'])


def navigation_completed(command, *, state, measured_pose, measured_at,
                         nav2_result, now=None):
    """Build a result hook using an independently measured map pose and stamp.

    Call after navigation_destination validation; never pass the goal as a
    measured pose. Both observation sources must remain fresh at report time.
    """
    if command['type'] != 'navigate.requested':
        raise ValueError('unsupported_operation')
    now = now or datetime.now(UTC)
    state_time = _fresh(state['observed_at'], now)
    pose_time = _fresh(measured_at, now)
    p = command['payload']
    if (nav2_result != 'SUCCEEDED' or state['mode'] != 'nav2'
            or state['stopped'] is not True or state['line_enabled'] is not False
            or state['nav2_ready'] is not True or state['localization_ready'] is not True
            or state['drive_owner'] != 'nav2' or state['fault'] is not None
            or any(state[k] != p[k] for k in ('map_id', 'map_version'))):
        raise ValueError('navigation_not_completed')
    actual = Destination(**measured_pose)
    target = Destination(**p['pose'])
    yaw = math.atan2(math.sin(actual.yaw - target.yaw), math.cos(actual.yaw - target.yaw))
    if (math.hypot(actual.x - target.x, actual.y - target.y) > p['position_tolerance_m']
            or abs(yaw) > p['yaw_tolerance_rad']):
        raise ValueError('goal_tolerance_not_met')
    details = {key: state[key] for key in _STATE}
    details.update(observed_at=min(state_time, pose_time).isoformat(),
                   goal_id=p['goal_id'], goal_version=p['goal_version'], frame_id='map',
                   pose={'x': actual.x, 'y': actual.y, 'yaw': actual.yaw},
                   nav2_result='SUCCEEDED')
    return command, 'completed', details


def navigation_failed(command, error_code):
    """The kit adds observed state and invokes safe_stop when reporting failure."""
    if command['type'] != 'navigate.requested' or not isinstance(error_code, str) or (
            not _ID.fullmatch(error_code)):
        raise ValueError('invalid_navigation_failure')
    return command, 'failed', {'error_code': error_code}
