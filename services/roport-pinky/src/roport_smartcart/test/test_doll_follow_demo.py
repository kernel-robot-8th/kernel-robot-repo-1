"""The demo must stop on stale perception and keep its speed below the robot trial limits."""

import time

from roport_smartcart.doll_follow_demo import FollowLogic


def target(height=50, center=320, view='front'):
    return {'label': 'A', 'view': view,
            'quad': [[center - 20, 200], [center + 20, 200],
                     [center + 20, 200 + height], [center - 20, 200 + height]]}


def test_calibrated_follow_and_stop_conditions():
    logic = FollowLogic('A')
    now = time.monotonic()
    logic.observe([target(50)], 640, now)
    assert logic.calibrate() and logic.arm(now)
    logic.observe([target(30)], 640, now)
    assert logic.command(now, (.6, now), (.8, now)) == (.025, 0.)
    logic.observe([target(50)], 640, now)
    assert logic.command(now, (.6, now), (.8, now)) == (0., 0.)
    logic.observe([target(30, 500)], 640, now)
    linear, angular = logic.command(now, (.6, now), (.8, now))
    assert linear == 0. and -.25 <= angular < 0.
    assert logic.command(now, (.24, now), (.8, now)) == (0., 0.)
    assert logic.command(now, (.6, now), (.24, now)) == (0., 0.)
    assert logic.command(now + .5, (.6, now), (.8, now)) == (0., 0.)
    assert not logic.armed


def test_target_loss_stops_then_latches_and_wrong_view_never_moves():
    logic = FollowLogic('B')
    now = time.monotonic()
    assert not logic.arm(now)
    logic.observe([{**target(), 'label': 'B'}], 640, now)
    assert logic.calibrate() and logic.arm(now)
    logic.observe([{**target(30, view='back'), 'label': 'B'}], 640, now + .1)
    assert logic.command(now + .1, (.6, now + .1), (.8, now + .1)) == (0., 0.)
    logic.observe([], 640, now + .9)
    assert logic.command(now + .9, (.6, now + .9), (.8, now + .9)) == (0., 0.)
    assert not logic.armed
