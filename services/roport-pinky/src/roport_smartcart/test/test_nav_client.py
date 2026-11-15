"""Injected futures plus an isolated ROS action server; no robot topics."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from roport_smartcart.nav_client import NavClient
from roport_smartcart.goals import map_hashes
import yaml


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancel = Future()
        self.result_requested = False
        self.cancel_requested = False

    def get_result_async(self):
        self.result_requested = True
        return self.result

    def cancel_goal_async(self):
        self.cancel_requested = True
        return self.cancel


class Client:
    def __init__(self, *args):
        self.args = args
        self.responses = []
        self.goals = []
        self.ready = True
        self.error = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        if self.error:
            raise self.error
        self.goals.append(goal)
        future = Future()
        self.responses.append(future)
        return future


def make_adapter(**kwargs):
    stamp = SimpleNamespace(to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time())
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: stamp))
    return NavClient(node, action_name='/isolated_test/navigate_to_pose',
                     client_factory=Client, **kwargs)


@pytest.fixture
def configured(tmp_path):
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    places = tmp_path / 'places.yaml'
    places.write_text(yaml.safe_dump({
        'map': {'frame': 'map', **map_hashes(map_yaml)},
        'places': {'gate1': {'measured': True, 'pose': [1, 2, 3.141592653589793]}}
    }))
    return places, map_yaml, 'gate1'


def accept(adapter):
    handle = Handle()
    adapter._client.responses[-1].set_result(handle)
    return handle


def test_explicit_server_and_goal_mapping(configured):
    adapter = make_adapter()
    assert not adapter._client.goals
    adapter.start(*configured)
    goal = adapter._client.goals[0].pose
    assert goal.header.frame_id == 'map'
    assert goal.pose.position.x == 1 and goal.pose.position.y == 2
    assert goal.pose.orientation.z == pytest.approx(1)
    assert goal.pose.orientation.w == pytest.approx(0, abs=1e-15)
    handle = accept(adapter)
    assert handle.result_requested
    handle.result.set_result(SimpleNamespace(status=4))
    assert adapter.lifecycle.outcome == 'succeeded'


def test_cancel_while_submission_waits_for_acceptance_and_result(configured):
    adapter = make_adapter()
    adapter.start(*configured)
    adapter.cancel()
    handle = accept(adapter)
    assert handle.result_requested and handle.cancel_requested
    handle.cancel.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert adapter.lifecycle.busy
    with pytest.raises(RuntimeError):
        adapter.start(*configured)
    handle.result.set_result(SimpleNamespace(status=5))
    assert adapter.lifecycle.outcome == 'canceled'


@pytest.mark.parametrize('where', ['send', 'response', 'result', 'cancel'])
def test_transport_exception_retains_ownership(where, configured):
    adapter = make_adapter()
    if where == 'send':
        adapter._client.error = RuntimeError('transport')
    adapter.start(*configured)
    if where == 'response':
        adapter._client.responses[-1].set_exception(RuntimeError('transport'))
    elif where in {'result', 'cancel'}:
        handle = accept(adapter)
        if where == 'result':
            handle.result.set_exception(RuntimeError('transport'))
        else:
            adapter.cancel()
            handle.cancel.set_exception(RuntimeError('transport'))
    assert adapter.lifecycle.busy and adapter.lifecycle.fault == 'transport_error'
    with pytest.raises(RuntimeError):
        adapter.start(*configured)


def test_poll_timeout_before_acceptance_cancels_late_accepted_goal(configured):
    now = [0]
    adapter = make_adapter(timeout_seconds=2, monotonic=lambda: now[0])
    adapter.start(*configured)
    now[0] = 3
    adapter.poll()
    assert adapter.lifecycle.fault == 'timeout' and adapter.lifecycle.busy
    handle = accept(adapter)
    assert handle.cancel_requested and handle.result_requested


def test_stale_cancel_result_cannot_touch_next_goal(configured):
    adapter = make_adapter()
    adapter.start(*configured)
    old = accept(adapter)
    adapter.cancel()
    old.result.set_result(SimpleNamespace(status=5))
    adapter.start(*configured)
    token = adapter.lifecycle.token
    old.cancel.set_exception(RuntimeError('late'))
    assert adapter.lifecycle.token == token and adapter.lifecycle.fault is None
    assert adapter.lifecycle.state == 'submitting'


def test_invalid_coordinates_and_unavailable_server_never_send(configured):
    adapter = make_adapter()
    with pytest.raises(ValueError):
        adapter.start(configured[0], configured[1], 'unknown')
    adapter._client.ready = False
    with pytest.raises(RuntimeError):
        adapter.start(*configured)
    assert adapter.lifecycle.state == 'idle' and not adapter._client.goals


def test_isolated_ros_action_server_roundtrip(configured):
    import time
    from uuid import uuid4

    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionServer
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    context = Context()
    rclpy.init(context=context, domain_id=198)
    namespace = '/roport_smartcart_test_' + uuid4().hex
    server_node = rclpy.create_node('server', namespace=namespace, context=context)
    client_node = rclpy.create_node('client', namespace=namespace, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server_node)
    executor.add_node(client_node)
    seen = []

    def execute(handle):
        seen.append(handle.request.pose)
        handle.succeed()
        return NavigateToPose.Result()

    server = ActionServer(server_node, NavigateToPose,
                          namespace + '/navigate_to_pose', execute)
    adapter = NavClient(client_node, action_name=namespace + '/navigate_to_pose')
    try:
        assert adapter._client.wait_for_server(timeout_sec=3)
        adapter.start(*configured)
        deadline = time.monotonic() + 5
        while adapter.lifecycle.busy and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert adapter.lifecycle.outcome == 'succeeded'
        assert len(seen) == 1 and seen[0].header.frame_id == 'map'
    finally:
        adapter._client.destroy()
        server.destroy()
        executor.shutdown()
        client_node.destroy_node()
        server_node.destroy_node()
        context.shutdown()


def test_result_request_exception_still_sends_pending_cancel(configured):
    adapter = make_adapter()
    adapter.start(*configured)
    adapter.cancel()
    handle = Handle()

    def broken_result():
        raise RuntimeError('result transport failure')

    handle.get_result_async = broken_result
    adapter._client.responses[-1].set_result(handle)
    assert handle.cancel_requested
    assert adapter.lifecycle.busy and adapter.lifecycle.fault == 'transport_error'


@pytest.mark.parametrize('change', ['unmeasured', 'yaml', 'image'])
def test_unmeasured_or_changed_map_cannot_send_goal(configured, change):
    places, map_yaml, _ = configured
    adapter = make_adapter()
    if change == 'unmeasured':
        data = yaml.safe_load(places.read_text())
        data['places']['gate1']['measured'] = False
        places.write_text(yaml.safe_dump(data))
    else:
        target = map_yaml if change == 'yaml' else map_yaml.parent / 'map.pgm'
        target.write_bytes(target.read_bytes() + b'\n')
    with pytest.raises(ValueError):
        adapter.start(*configured)
    assert not adapter._client.goals
    assert adapter.lifecycle.state == 'idle'


def test_each_request_reloads_measurement_instead_of_caching(configured):
    places, _, _ = configured
    adapter = make_adapter()
    adapter.start(*configured)
    accept(adapter).result.set_result(SimpleNamespace(status=4))
    data = yaml.safe_load(places.read_text())
    data['places']['gate1']['measured'] = False
    places.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        adapter.start(*configured)
    assert len(adapter._client.goals) == 1
