import threading
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
