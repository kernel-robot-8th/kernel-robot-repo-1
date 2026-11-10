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


def recovery_bridge():
    bridge = object.__new__(Nav2Bridge)
    bridge.lock, bridge.generation = threading.RLock(), 1
    bridge.goal = {'goal_id': 'G2'}
    bridge.result = None
    bridge.cancelled = []
    bridge.handle = SimpleNamespace(cancel_goal_async=lambda: bridge.cancelled.append(True))
    bridge.navigation = {'phase': 'navigating', 'recoveries': 0}
    bridge.recovery_started = bridge.recovery_pose = None
    bridge.snapshot = lambda: {'pose': {'x': 1., 'y': 1., 'yaw': 0.}}
    return bridge


def feedback(bridge, count, generation=1):
    bridge._feedback(SimpleNamespace(feedback=SimpleNamespace(number_of_recoveries=count)), generation)


def test_recovery_limit_cancels_and_late_success_cannot_replace_failure():
    bridge = recovery_bridge()
    feedback(bridge, 3, generation=0)
    assert bridge.navigation['recoveries'] == 0
    feedback(bridge, 2)
    assert bridge.navigation['phase'] == 'recovering'
    assert bridge.result is None
    feedback(bridge, 3)
    assert bridge.result[0] == 'nav2_recovery_limit'
    assert bridge.cancelled == [True]
    bridge._result(SimpleNamespace(result=lambda: pytest.fail('late result')), 1)
    assert bridge.result[0] == 'nav2_recovery_limit'


def test_recovery_timeout_does_not_restart_on_more_feedback(monkeypatch):
    bridge = recovery_bridge()
    monkeypatch.setattr(time, 'monotonic', lambda: 100.)
    feedback(bridge, 1)
    monkeypatch.setattr(time, 'monotonic', lambda: 129.)
    feedback(bridge, 2)
    state = {'localization_ready': True, 'nav2_ready': True,
             'pose': {'x': 1., 'y': 1., 'yaw': 0.}}
    bridge._check_recovery(state)
    assert bridge.result is None
    monkeypatch.setattr(time, 'monotonic', lambda: 130.)
    bridge._check_recovery(state)
    assert bridge.result[0] == 'nav2_recovery_timeout'
    assert bridge.cancelled == [True]


def test_measured_progress_ends_recovery_but_does_not_reset_retry_count():
    bridge = recovery_bridge()
    feedback(bridge, 1)
    bridge._check_recovery({'localization_ready': True, 'nav2_ready': True,
                           'pose': {'x': 1.06, 'y': 1., 'yaw': 0.}})
    assert bridge.navigation['phase'] == 'navigating'
    assert bridge.navigation['recoveries'] == 1
    assert bridge.recovery_started is None
    assert bridge.result is None
    bridge._check_recovery({'localization_ready': False, 'nav2_ready': True, 'pose': None})
    assert bridge.result[0] == 'nav2_not_ready'


def test_bounded_tree_limits_recovery_without_blind_motion():
    import xml.etree.ElementTree as ET
    tree = ET.parse(PINKY / 'src/roport_pinky/behavior_trees/bounded_recovery.xml')
    assert tree.find('.//RecoveryNode').attrib['number_of_retries'] == '2'
    assert tree.find('.//Timeout').attrib['msec'] == '10000'
    assert tree.find('.//Wait').attrib['wait_duration'] == '3.0'
    assert tree.find('.//Spin') is tree.find('.//BackUp') is None


def test_recovery_tree_ports_match_installed_nav2_jazzy_nodes():
    import xml.etree.ElementTree as ET
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    models = ET.parse(Path(get_package_share_directory('nav2_behavior_tree')) / 'nav2_tree_nodes.xml')
    ports = {m.attrib['ID']: {p.attrib['name'] for p in m if 'name' in p.attrib}
             for m in models.findall('.//TreeNodesModel/*')}
    tree = ET.parse(PINKY / 'src/roport_pinky/behavior_trees/bounded_recovery.xml')
    assert 'Wait' in ports
    for node in tree.iter():
        if node.tag in ports:
            supplied = set(node.attrib) - {'name'}
            assert supplied <= ports[node.tag], (node.tag, supplied - ports[node.tag])
