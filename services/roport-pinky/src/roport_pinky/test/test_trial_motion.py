"""Read-only display hook: reuse ROS driver without starting ROS or motors."""
import copy
import math
import threading
from datetime import UTC, datetime
from types import SimpleNamespace

import pytest
import yaml
from test_trial_driver import LAYOUT, Rig
from roport_pinky.trial_driver import TrialDriver


def driver():
    rig = Rig()
    node = TrialDriver.__new__(TrialDriver)
    node.lock = threading.RLock()
    node.logic, node.mode = rig.logic, 'line'
    with open(LAYOUT) as stream:
        node.frame = yaml.safe_load(stream)['frame']
    node.nav = SimpleNamespace(snapshot=lambda: {'pose': None, 'localization_ready': False})
    return node, rig


def test_tf_display_pose_does_not_change_control_observation():
    node, rig = driver()
    before = copy.deepcopy(rig.logic.observe())
    stamp = datetime.now(UTC).isoformat()
    measured = {'x': .12, 'y': -.34, 'yaw': 1.2}
    node.nav.snapshot = lambda: {'pose': measured, 'localization_ready': True, 'observed_at': stamp}
    for mode in ('line', 'nav2'):
        node.mode = mode
        sample = node.observe_motion()
        assert sample['pose'] == {**measured, 'z': 0.0}
        assert sample['observed_at'] == stamp and sample['pose_source'] == 'tf'
    assert measured == {'x': .12, 'y': -.34, 'yaw': 1.2}
    assert rig.logic.observe() == before


def test_line_estimate_conversion_and_stale_or_nav2_fallback_rejection():
    node, rig = driver()
    rig.logic.own = True
    rig.logic.st['pose_est'] = {'x_cm': 120.0, 'y_cm': 30.0, 'heading_deg': 90}
    pose = node.observe_motion()['pose']
    a = math.radians(node.frame['yaw_deg'])
    assert pose['x'] == pytest.approx(node.frame['x_m'] + math.cos(a)*1.2 - math.sin(a)*.3)
    assert pose['y'] == pytest.approx(node.frame['y_m'] + math.sin(a)*1.2 + math.cos(a)*.3)
    assert pose['yaw'] == pytest.approx(math.pi/2 + a)
    node.mode = 'nav2'
    assert node.observe_motion()['pose'] is None
    node.mode = 'line'
    rig.clock.t += 2
    assert node.observe_motion()['pose'] is None
