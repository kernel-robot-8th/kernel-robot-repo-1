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
    assert logic.calibrate(.08) and logic.arm(now)
    logic.observe([target(30)], 640, now)
    assert logic.command(now, (.6, now), (.8, now)) == (.025, 0.)
    logic.observe([target(45)], 640, now)
    assert logic.command(now, (.6, now), (.8, now)) == (.025, 0.)
    logic.observe([target(50)], 640, now)
    assert logic.command(now, (.6, now), (.8, now)) == (0., 0.)
    logic.observe([target(30, 500)], 640, now)
    linear, angular = logic.command(now, (.6, now), (.8, now))
    assert linear == 0. and -.25 <= angular < 0.
    logic.observe([target(30)], 640, now)
    assert logic.command(now, (.20, now), (.8, now)) == (.025, 0.)
    assert logic.command(now, (.09, now), (.8, now)) == (.025, 0.)
    assert logic.command(now, (.06, now), (.8, now)) == (0., 0.)
    assert logic.command(now, (.05, now), (.8, now)) == (0., 0.)
    assert logic.reason == 'SONAR <=5 CM; STOPPED'
    assert logic.command(now, (.6, now), (.24, now)) == (0., 0.)
    assert logic.reason == 'LIDAR <25 CM; STOPPED'
    assert logic.command(now + .5, (.6, now), (.8, now)) == (0., 0.)
    assert not logic.armed


def test_target_loss_stops_then_latches_and_wrong_label_never_moves():
    logic = FollowLogic('B')
    now = time.monotonic()
    assert not logic.arm(now)
    logic.observe([{**target(), 'label': 'B'}], 640, now)
    assert logic.calibrate(.08) and logic.arm(now)
    logic.observe([target(30, view='back')], 640, now + .1)
    assert logic.command(now + .1, (.6, now + .1), (.8, now + .1)) == (0., 0.)
    logic.observe([], 640, now + 1.)
    assert logic.command(now + 1., (.6, now + 1.), (.8, now + 1.)) == (0., 0.)
    assert logic.armed
    logic.observe([], 640, now + 2.1)
    assert logic.command(now + 2.1, (.6, now + 2.1), (.8, now + 2.1)) == (0., 0.)
    assert not logic.armed


def test_same_doll_resumes_on_back_view_with_reference_size_correction():
    logic = FollowLogic('A', {'front': 394, 'back': 371})
    now = time.monotonic()
    logic.observe([target(50)], 640, now)
    assert logic.calibrate(.08) and logic.arm(now)
    logic.observe([], 640, now + .5)
    assert logic.command(now + .5, (.6, now + .5), (.8, now + .5)) == (0., 0.)
    logic.observe([target(45, view='back')], 640, now + .6)
    assert logic.command(now + .6, (.6, now + .6), (.8, now + .6)) == (.025, 0.)
    logic.observe([target(48, view='back')], 640, now + .7)
    assert logic.command(now + .7, (.6, now + .7), (.8, now + .7)) == (0., 0.)


def test_lidar_front_is_transformed_from_reversed_pinky_sensor_frame():
    import math
    from types import SimpleNamespace as NS
    from roport_smartcart.doll_follow_demo import front_distance

    scan = NS(angle_min=-math.pi, angle_increment=math.pi / 2,
              range_min=.02, range_max=10., ranges=[.18, .8, .06, .8, .18])
    # Pinky's rplidar_link has yaw=pi: scan zero is behind the robot.
    flipped = NS(x=0., y=0., z=1., w=0.)
    assert front_distance(scan, flipped) == .18
    scan.ranges = [.8, .8, .06, .8, .8]
    assert front_distance(scan, flipped) == .8
    assert front_distance(scan, NS(x=0., y=0., z=0., w=1.)) == .06
    scan.ranges = [math.nan, .8, .06, .8, math.nan]
    assert front_distance(scan, flipped) is None
    scan.ranges = [math.inf, .8, .06, .8, math.inf]
    assert front_distance(scan, flipped) == math.inf
    scan.angle_increment = math.nan
    assert front_distance(scan, flipped) is None


def test_sensor_queue_delay_and_clock_skew_do_not_refresh_freshness():
    from types import SimpleNamespace as NS
    import pytest
    from roport_smartcart.doll_follow_demo import sampled_at

    assert sampled_at(NS(sec=99, nanosec=800000000), 100., 10.) == pytest.approx(9.8)
    assert sampled_at(NS(sec=99, nanosec=500000000), 100., 10.) is None
    assert sampled_at(NS(sec=100, nanosec=200000000), 100., 10.) is None
    assert sampled_at(NS(sec=0, nanosec=0), 100., 10.) is None
    logic = FollowLogic('A')
    now = time.monotonic()
    logic.observe([target()], 640, now)
    assert logic.calibrate(.08) and logic.arm(now)
    assert logic.command(now, (.6, now - .5), (.8, now)) == (0., 0.)
    assert not logic.armed


def test_invalid_guard_json_is_ignored():
    import json
    from roport_smartcart.doll_follow_demo import guard_body

    for invalid in ('[]', 'null', '1', 'true', '"string"', '{broken', '{}',
                    '{"boot":"b","owner":"line","blocked":false,"sent_at":NaN}'):
        assert guard_body(invalid) == {}
    body = dict(boot='boot', owner='none', blocked=False, sent_at=100.)
    assert guard_body(json.dumps(body)) == body


def test_guard_ack_timeout_and_ownership_loss_require_a_new_start():
    from roport_smartcart.doll_follow_demo import DriveHandshake

    handshake = DriveHandshake()
    assert handshake.check('line', 10.) == 'failed'
    handshake.start(10.)
    assert handshake.check('none', 10.1) == 'waiting'
    assert handshake.check('none', 10.31) == 'failed'
    assert handshake.check('line', 10.4) == 'failed'  # late clock sync cannot start motion
    handshake.start(11.)
    assert handshake.check('line', 11.1) == 'confirmed'
    assert handshake.check('none', 11.2) == 'failed'


def test_guard_rejecting_clock_skew_never_confirms_ownership():
    import pytest
    pytest.importorskip('rclpy')
    from roport_pinky.trial_guard import LeaseGate
    from roport_smartcart.doll_follow_demo import DriveHandshake

    for offset in (.2, -.6):
        gate = LeaseGate()
        handshake = DriveHandshake()
        handshake.start(10.)
        for seq in range(5):
            now = 10. + seq * .1
            gate.lease(dict(boot=gate.boot, owner='line', seq=seq, session='test',
                            sent_at=100. + seq * .1 + offset), now, 100. + seq * .1)
            assert gate.owner == 'none'
            assert handshake.check(gate.owner, now) != 'confirmed'
        assert handshake.check(gate.owner, 10.4) == 'failed'


def test_thirty_second_limit_still_stops_with_fresh_inputs():
    logic = FollowLogic('A')
    now = time.monotonic()
    logic.observe([target()], 640, now)
    assert logic.calibrate(.08) and logic.arm(now)
    logic.observe([target(30)], 640, now + 31.)
    assert logic.command(now + 31., (.6, now + 31.), (.8, now + 31.)) == (0., 0.)
    assert not logic.armed and logic.reason == '30 SECOND LIMIT; STOPPED'
