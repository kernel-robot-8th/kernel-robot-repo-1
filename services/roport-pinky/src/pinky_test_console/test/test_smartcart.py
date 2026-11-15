"""No ROS or robot commands: freshness, explicit heading and HTTP boundaries."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from pinky_test_console.amcl_check import AmclCheckError
from pinky_test_console.smartcart import SmartcartMonitor


def sample(stamp):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))))


def ready_monitor():
    clock = [10.]
    monitor = SmartcartMonitor(clock=lambda: clock[0])
    for now in (10., 10.6):
        clock[0] = now
        monitor.observe('scan', sample(now))
        monitor.observe('odom', sample(now), stopped=True)
    return monitor, clock


def test_initial_heading_and_placement_are_explicit():
    monitor, _ = ready_monitor()
    monitor.require_initial_pose(10.6, 'H1', 90., True, False)
    for node, yaw, confirmed, navigating in [('H1', None, True, False),
            ('H1', True, True, False), ('H1', 0, False, False),
            ('H1', 0, True, True), ('N11', 0, True, False),
            ('H2', float('nan'), True, False), ('H2', 181, True, False)]:
        with pytest.raises(AmclCheckError):
            monitor.require_initial_pose(10.6, node, yaw, confirmed, navigating)


def test_motion_and_stale_or_replayed_odom_refuse_initialization():
    monitor, clock = ready_monitor()
    monitor.observe('odom', sample(10.7), stopped=False)
    monitor.observe('odom', sample(10.6), stopped=True)
    with pytest.raises(AmclCheckError):
        monitor.require_initial_pose(10.7, 'H1', 90, True, False)
    clock[0] = 12.
    assert not monitor.snapshot(12.)['scan']['fresh']


def test_old_ros_timestamp_is_not_fresh_even_if_received_now():
    monitor, _ = ready_monitor()
    assert not monitor.snapshot(20.)['scan']['fresh']
    assert not monitor.snapshot(20.)['stopped']


def test_gui_routes_and_guarded_initial_pose():
    pytest.importorskip('flask')
    from pinky_test_console.web_app import create_app

    monitor, _ = ready_monitor()

    class Node:
        calls = []

        def smartcart_snapshot(self):
            return {'sensors': monitor.snapshot(10.6), 'delivery_enabled': False}

        def smartcart_initial_pose(self, node, yaw, confirmed):
            monitor.require_initial_pose(10.6, node, yaw, confirmed, False)
            self.calls.append((node, yaw))
            return True

    node = Node()
    web = Path(__file__).resolve().parents[1] / 'web'
    client = create_app(node, str(web)).test_client()
    assert client.get('/smartcart.html').status_code == 200
    assert client.get('/smartcart.js').status_code == 200
    assert client.get('/api/smartcart').json['delivery_enabled'] is False
    response = client.post('/api/smartcart/initialpose', json={'node': 'H1', 'placement_confirmed': True})
    assert response.status_code == 400 and not node.calls
    response = client.post('/api/smartcart/initialpose', json={
        'node': 'H1', 'yaw_deg': 90, 'placement_confirmed': True})
    assert response.status_code == 200 and node.calls == [('H1', 90)]
    assert client.post('/api/smartcart/initialpose', data='node=H1').status_code == 400
