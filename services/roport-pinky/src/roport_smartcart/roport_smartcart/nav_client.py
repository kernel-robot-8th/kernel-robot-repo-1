"""Explicit Nav2 transport adapter; constructing it never sends a goal.

A terminal action result or cancel acknowledgement does not establish physical
standstill. The owner must independently confirm odometry before switching modes.
"""

import math
import time
from threading import RLock

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from .navigation import NavigationLifecycle
from .goals import load_destination


class NavClient:
    """One owned goal, with manual ``poll`` for an optional response deadline.

    Transport failures retain ownership until a terminal action result arrives.
    The supplied node's executor must run callbacks; this class never spins/sleeps.
    """

    def __init__(self, node, *, action_name, client_factory=ActionClient,
                 timeout_seconds=None, monotonic=time.monotonic):
        if not isinstance(action_name, str) or not action_name.strip():
            raise ValueError('explicit action_name required')
        if timeout_seconds is not None and (
                not math.isfinite(timeout_seconds) or timeout_seconds <= 0):
            raise ValueError('timeout_seconds must be positive and finite')
        self.lifecycle = NavigationLifecycle()
        self._node = node
        self._client = client_factory(node, NavigateToPose, action_name)
        self._timeout = timeout_seconds
        self._clock = monotonic
        self._deadline = None
        self._handle = None
        self._lock = RLock()

    def start(self, places_file, map_yaml, place_id):
        # Validate before ownership changes or a request reaches Nav2.
        destination = load_destination(places_file, map_yaml, place_id)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(destination.x)
        goal.pose.pose.position.y = float(destination.y)
        goal.pose.pose.orientation.z = math.sin(destination.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(destination.yaw / 2)
        with self._lock:
            if not self._client.server_is_ready():
                raise RuntimeError('Nav2 action server is unavailable')
            effects = self.lifecycle.start(goal)
            self._handle = None
            self._deadline = (None if self._timeout is None else
                              self._clock() + self._timeout)
            self._apply(effects)
            return self.lifecycle.token

    def cancel(self):
        with self._lock:
            self._apply(self.lifecycle.cancel())

    def poll(self):
        with self._lock:
            if self._deadline is not None and self._clock() >= self._deadline:
                self._deadline = None
                self._apply(self.lifecycle.timeout(self.lifecycle.token))

    def _fault(self, token):
        if token == self.lifecycle.token and self.lifecycle.busy:
            self.lifecycle.fault = 'transport_error'

    def _apply(self, effects):
        for effect in effects:
            kind, token, *args = effect
            if token != self.lifecycle.token or not self.lifecycle.busy:
                continue
            try:
                if kind == 'send_goal':
                    future = self._client.send_goal_async(args[0])
                    future.add_done_callback(lambda f, t=token: self._goal_response(t, f))
                elif kind == 'cancel_goal':
                    future = self._handle.cancel_goal_async()
                    future.add_done_callback(lambda f, t=token: self._cancel_response(t, f))
            except Exception:
                self._fault(token)

    def _goal_response(self, token, future):
        with self._lock:
            if token != self.lifecycle.token or self.lifecycle.state != 'submitting':
                return
            effects = ()
            try:
                handle = future.result()
                effects = self.lifecycle.goal_response(token, handle.accepted)
                if handle.accepted:
                    self._handle = handle
                    # Even an already requested cancellation needs a terminal result.
                    result = handle.get_result_async()
                    result.add_done_callback(lambda f, t=token: self._result(t, f))
            except Exception:
                self._fault(token)
            # A result transport failure must not suppress an outstanding cancel.
            self._apply(effects)

    def _result(self, token, future):
        with self._lock:
            if token != self.lifecycle.token or not self.lifecycle.busy:
                return
            try:
                status = future.result().status
                statuses = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                            GoalStatus.STATUS_CANCELED: 'canceled',
                            GoalStatus.STATUS_ABORTED: 'aborted'}
                self.lifecycle.result(token, statuses.get(status, 'unknown'))
            except Exception:
                self._fault(token)

    def _cancel_response(self, token, future):
        with self._lock:
            if token != self.lifecycle.token or not self.lifecycle.busy:
                return
            try:
                self.lifecycle.cancel_response(token, bool(future.result().goals_canceling))
            except Exception:
                self._fault(token)
