"""Exercise delayed action callbacks and cancellation races without ROS."""

import pytest

from roport_smartcart.navigation import NavigationLifecycle


def test_cancel_while_submission_pending_waits_for_acceptance():
    nav = NavigationLifecycle()
    assert nav.start('gate1') == (('send_goal', 1, 'gate1'),)
    assert nav.cancel() == ()
    assert nav.busy and nav.state == 'submitting'
    assert nav.goal_response(1, True) == (('cancel_goal', 1),)
    nav.cancel_response(1, True)
    assert nav.busy
    with pytest.raises(RuntimeError):
        nav.start('gate2')
    nav.result(1, 'canceled')
    assert not nav.busy and nav.outcome == 'canceled'


def test_old_callbacks_cannot_finish_or_cancel_new_goal():
    nav = NavigationLifecycle()
    nav.start('gate1')
    nav.goal_response(1, False)
    nav.start('gate2')
    assert nav.goal_response(1, True) == ()
    nav.result(1, 'succeeded')
    nav.cancel_response(1, False)
    nav.timeout(1)
    assert nav.state == 'submitting' and nav.fault is None
    nav.goal_response(2, True)
    nav.result(1, 'aborted')
    assert nav.state == 'active'
    nav.result(2, 'succeeded')
    nav.result(2, 'aborted')
    assert nav.outcome == 'succeeded'


@pytest.mark.parametrize('accepted', [True, False])
def test_timeout_keeps_ownership_until_terminal_result(accepted):
    nav = NavigationLifecycle()
    nav.start('gate1')
    nav.goal_response(1, True)
    assert nav.timeout(1) == (('cancel_goal', 1),)
    nav.cancel_response(1, accepted)
    assert nav.busy
    assert nav.fault == ('timeout' if accepted else 'cancel_rejected')
    with pytest.raises(RuntimeError):
        nav.start('gate2')
    nav.result(1, 'aborted')
    assert not nav.busy


def test_submission_timeout_does_not_allow_new_goal_or_lose_delayed_handle():
    nav = NavigationLifecycle()
    nav.start('gate1')
    nav.timeout(1)
    with pytest.raises(RuntimeError):
        nav.start('gate2')
    assert nav.goal_response(1, True) == (('cancel_goal', 1),)
    assert nav.cancel() == ()
    nav.result(1, 'succeeded')
    assert nav.outcome == 'succeeded'


def test_nonterminal_result_does_not_release_goal():
    nav = NavigationLifecycle()
    nav.start('gate1')
    nav.result(1, 'succeeded')
    assert nav.state == 'submitting'
    nav.goal_response(1, True)
    nav.result(1, 'executing')
    assert nav.busy and nav.fault == 'invalid_result'
    nav.result(1, 'aborted')
    assert not nav.busy
