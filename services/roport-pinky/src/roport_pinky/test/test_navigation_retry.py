"""A cancellation acknowledgement must never authorize overlapping Nav2 goals."""
from concurrent.futures import Future

from roport_pinky.trial_nav2 import Nav2Bridge


def test_retry_waits_for_goal_acceptance_and_cancelled_goal_terminal_result():
    bridge = object.__new__(Nav2Bridge)
    ack, terminal = Future(), Future()
    bridge.goal_ack, bridge.cancelled_goals = ack, [terminal]
    assert not bridge.quiescent()
    ack.set_result('accepted')
    assert not bridge.quiescent()
    terminal.set_result('CANCELED')
    assert bridge.quiescent()
    unknown = Future()
    unknown.set_exception(RuntimeError('result transport lost'))
    bridge.cancelled_goals.append(unknown)
    assert not bridge.quiescent()
