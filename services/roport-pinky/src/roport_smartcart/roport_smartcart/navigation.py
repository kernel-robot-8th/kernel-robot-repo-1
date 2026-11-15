"""Transport-independent ownership of one NavigateToPose goal.

Effects are ``('send_goal', token, goal)`` or ``('cancel_goal', token)``.
The ROS adapter must return the same token in every callback. This class does
not establish physical standstill; the caller must also check odometry.
"""


class NavigationLifecycle:
    """Keep ownership until Nav2 supplies a terminal result, even on timeout."""

    def __init__(self):
        self.state = 'idle'
        self.token = 0
        self.outcome = None
        self.fault = None
        self._cancel_requested = False
        self._accepted = False

    @property
    def busy(self):
        return self.state in ('submitting', 'active', 'canceling')

    def start(self, goal):
        if self.busy:
            raise RuntimeError('Nav2 still owns the previous goal')
        self.token += 1
        self.state = 'submitting'
        self.outcome = None
        self.fault = None
        self._cancel_requested = False
        self._accepted = False
        return (('send_goal', self.token, goal),)

    def cancel(self):
        if not self.busy or self._cancel_requested:
            return ()
        self._cancel_requested = True
        if not self._accepted:
            return ()
        self.state = 'canceling'
        return (('cancel_goal', self.token),)

    def goal_response(self, token, accepted):
        if token != self.token or self.state != 'submitting':
            return ()
        if not accepted:
            self.state = 'finished'
            self.outcome = 'rejected'
            return ()
        self._accepted = True
        if self._cancel_requested:
            self.state = 'canceling'
            return (('cancel_goal', token),)
        self.state = 'active'
        return ()

    def cancel_response(self, token, accepted):
        if token == self.token and self.state == 'canceling' and not accepted:
            self.fault = 'cancel_rejected'
        # An accepted cancel request is not a terminal action result.
        return ()

    def result(self, token, status):
        if token != self.token or not self._accepted or not self.busy:
            return ()
        if status not in ('succeeded', 'canceled', 'aborted'):
            self.fault = 'invalid_result'
            return ()
        self.state = 'finished'
        self.outcome = status
        return ()

    def timeout(self, token):
        if token != self.token or not self.busy:
            return ()
        self.fault = 'timeout'
        # Missing responses cannot prove Nav2 has released motor ownership.
        return self.cancel()
