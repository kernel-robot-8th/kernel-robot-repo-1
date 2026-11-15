"""A cancellation acknowledgement must never authorize overlapping Nav2 goals."""
from concurrent.futures import Future
from threading import RLock

import pytest

from roport_pinky.trial_nav2 import Nav2Bridge


def test_retry_waits_for_goal_acceptance_and_cancelled_goal_terminal_result():
    bridge = object.__new__(Nav2Bridge)
    bridge.handle = bridge.result = None
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


def test_retry_waits_until_completed_goal_acceptance_callback_runs():
    bridge = object.__new__(Nav2Bridge)
    bridge.handle = bridge.result = None
    ack = Future()
    ack.set_result('accepted')
    bridge.goal_ack, bridge.cancelled_goals = ack, []
    bridge.pending_goal_acks = 1
    assert not bridge.quiescent()
    bridge.pending_goal_acks = 0
    assert bridge.quiescent()


@pytest.mark.parametrize('evidence', ['pending', 'terminal', 'missing', 'exception', 'cancelled'])
def test_diagnostic_explains_unknown_cancel_result_without_unlocking_retry(evidence):
    bridge = object.__new__(Nav2Bridge)
    bridge.handle = bridge.result = None
    bridge.lock, bridge.goal_ack = RLock(), None
    bridge.navigation = {'phase': 'failed', 'error_code': 'nav2_goal_failed'}
    terminal = Future()
    if evidence == 'terminal':
        terminal.set_result('CANCELED')
    elif evidence == 'exception':
        terminal.set_exception(RuntimeError('result transport lost'))
    elif evidence == 'cancelled':
        terminal.cancel()
    bridge.cancelled_goals = [None if evidence == 'missing' else terminal]
    unknown = evidence in ('missing', 'exception', 'cancelled')
    expected = 'nav2_cancel_result_unknown' if unknown else 'nav2_goal_failed'
    assert bridge.diagnostic()['error_code'] == expected
    assert bridge.diagnostic()['error_code'] == expected
    assert bridge.navigation['error_code'] == 'nav2_goal_failed'
    assert bridge.quiescent() == (evidence == 'terminal')
    assert len(bridge.cancelled_goals) == 1
