import threading
import time
from datetime import UTC, datetime
from types import SimpleNamespace

import pytest
from roport_pinky.trial_nav2 import Nav2Bridge, map_reference
from ros_trial_fixture import PINKY
from test_trial_driver import PROFILE


def test_late_goal_acceptance_is_cancelled_after_stop():
    bridge = object.__new__(Nav2Bridge)
    bridge.lock, bridge.generation = threading.RLock(), 2
    cancelled = []
    handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: cancelled.append(True))
    bridge._accepted(SimpleNamespace(result=lambda: handle), 1)
    assert cancelled == [True]
    bridge.result = None
    bridge._result(SimpleNamespace(result=lambda: pytest.fail('old result must be ignored')), 1)
    assert bridge.result is None


def test_local_map_hash_must_match_control_version():
    with pytest.raises(ValueError, match='local_map_version_mismatch'):
        map_reference(PINKY / 'maps/airport_map.yaml', {**PROFILE, 'map_version': 'old'})


def test_display_pose_time_preserves_tf_age_in_wall_clock():
    def stamped(seconds, **fields):
        whole = int(seconds)
        return SimpleNamespace(header=SimpleNamespace(frame_id='map', stamp=SimpleNamespace(
            sec=whole, nanosec=round((seconds-whole)*1e9))), **fields)
    bridge = object.__new__(Nav2Bridge)
    bridge.lock = threading.RLock()
    bridge.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1000*10**9)))
    bridge.action = SimpleNamespace(server_is_ready=lambda: True)
    bridge.map_ok = True
    bridge.state_clients = {'amcl': None, 'bt_navigator': None}
    bridge.states = {name: (3, time.monotonic()) for name in bridge.state_clients}
    bridge.amcl = stamped(990, pose=SimpleNamespace(covariance=[0.] * 36))
    bridge.scan = stamped(1000)
    vector = SimpleNamespace(x=0., y=0., z=0.)
    bridge.odom = stamped(1000, twist=SimpleNamespace(twist=SimpleNamespace(
        linear=vector, angular=vector)))
    transform = SimpleNamespace(translation=vector, rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    transforms = {('map', 'base_footprint'): stamped(999.7, transform=transform),
                  ('map', 'odom'): stamped(1001, transform=transform),
                  ('odom', 'base_footprint'): stamped(999.6, transform=transform)}
    bridge.tf = SimpleNamespace(lookup_transform=lambda parent, child, _: transforms[parent, child])
    state = bridge._localization_state()
    assert state['localization_ready']
    report = datetime.fromisoformat(state['observed_at'])
    pose = datetime.fromisoformat(state['pose_observed_at'])
    assert (report-pose).total_seconds() == pytest.approx(.4)
    assert (datetime.now(UTC)-pose).total_seconds() < .5
    bridge.localization = (state, time.monotonic())
    assert bridge.snapshot()['pose_observed_at'] == state['pose_observed_at']
