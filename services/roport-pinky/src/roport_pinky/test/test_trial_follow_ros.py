"""follow.requested through real ROS topics, trial_guard and a synthetic NavigateToPose server.

Loopback-only (fixture domain 194). No bt_navigator: the action server only records the goal
(behavior_tree file, start pose) and runs until cancelled, like the follow tree.
"""
import json
import time

import pytest

from roport_pinky import trial_follow
from ros_trial_fixture import RosTrial
from test_trial_driver import PROFILE


class Laptop:
    """Stands in for roport_follow on the laptop: /follow/target + /follow/status per frame."""

    def __init__(self, trial):
        from geometry_msgs.msg import PointStamped, PoseStamped
        from std_msgs.msg import String
        self.trial, self.node = trial, trial.node
        self.state, self.card, self.x, self.y, self.on = 'tracking', 35, 1.0, 0.0, True
        self.target = self.node.create_publisher(PointStamped, 'follow/target', 5)
        self.status = self.node.create_publisher(String, 'follow/status', 5)
        self.updates = []
        self.node.create_subscription(PoseStamped, 'goal_update', self.updates.append, 10)
        self._Point, self._String = PointStamped, String
        self.timer = self.node.create_timer(.1, self.frame)

    def frame(self):
        if not self.on:
            return
        now = self.node.get_clock().now().nanoseconds * 1e-9 - .1   # camera latency
        seen = self.state == 'tracking'
        msg = self._Point()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now % 1) * 1e9)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        msg.point.x, msg.point.y = self.x, self.y
        if seen:
            self.target.publish(msg)
        self.status.publish(self._String(data=json.dumps({
            'state': self.state, 'card_id': self.card, 'seen': seen, 'age_s': 0.0,
            'x_m': self.x, 'y_m': self.y, 'others': [], 'rejected': 0,
            'stamp': stamp, 'frame_id': 'base_footprint'})))


def follow(card=35, **extra):
    return {'type': 'follow.requested', 'command_id': f'f{time.monotonic_ns()}',
            'execution_id': 'e', 'payload': {**PROFILE, 'card_id': card, 'standoff_m': .5, **extra}}


@pytest.fixture
def trial():
    rig = RosTrial()
    rig.driver.mode = 'nav2'
    rig.laptop = Laptop(rig)
    try:
        yield rig
    finally:
        rig.close()


def start(trial, command=None):
    command = command or follow()
    trial.wait(lambda: trial.driver.follow_input.target.get(35) is not None)
    trial.wait(lambda: _ok(trial.driver.validate_action, command))
    trial.driver.dispatch(command)
    trial.wait(lambda: trial.goals and trial.driver.guard['owner'] == 'nav2')
    return command


def _ok(check, command):
    try:
        check(command)
        return True
    except ValueError:
        return False


def finish(trial):
    results = []
    def done():
        results.extend(trial.driver.drain_results())
        return results
    trial.wait(done, timeout=5)
    return results


def test_follow_sends_tree_goal_updates_and_keeps_nav2_lease(trial):
    command = start(trial)
    goal = trial.goals[-1]
    assert goal.behavior_tree.endswith('behavior_trees/follow_target.xml')
    assert goal.pose.header.frame_id == 'map'
    # robot at map origin, customer 1.0 m ahead, 0.5 m standoff
    assert goal.pose.pose.position.x == pytest.approx(.5, abs=.02)
    trial.laptop.x = 1.4
    trial.wait(lambda: trial.laptop.updates
               and trial.laptop.updates[-1].pose.position.x == pytest.approx(.9, abs=.02))
    update = trial.laptop.updates[-1]
    assert update.header.frame_id == 'map'
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    assert stamp(update.header) > stamp(goal.pose.header)   # GoalUpdater only takes newer stamps
    end = time.monotonic() + 1.5
    while time.monotonic() < end:                            # lease outlives the 1 s guard timeout
        trial.driver.heartbeat()
        assert trial.driver.guard['owner'] == 'nav2' and not trial.driver.guard['blocked']
        time.sleep(.05)
    assert trial.driver.drain_results() == []
    assert trial.driver.observe()['drive_owner'] == 'nav2'
    trial.driver.safe_stop('operator_stop')
    trial.wait(lambda: trial.driver.guard['owner'] == 'none')
    trial.wait(trial.driver.nav.quiescent)
    assert command  # operator stop result is reported by the MQTT module (interrupted)


def test_lost_customer_stops_reports_and_accepts_a_new_follow(trial):
    command = start(trial)
    trial.laptop.state = 'lost'
    results = finish(trial)
    assert results == [(command, 'failed', {'error_code': 'follow_lost'})]
    trial.wait(lambda: trial.driver.guard['owner'] == 'none')
    trial.wait(trial.driver.nav.quiescent)                  # the follow goal was cancelled
    assert trial.driver.halted is None
    trial.laptop.state = 'tracking'
    start(trial, follow())


def test_laptop_gone_is_status_stale(trial):
    command = start(trial)
    trial.laptop.on = False
    t0 = time.monotonic()
    assert finish(trial) == [(command, 'failed', {'error_code': 'follow_status_stale'})]
    assert time.monotonic() - t0 < trial_follow.STATUS_MAX_AGE + 1.0      # status 유효 2 s(11-21) + 판정·종료 여유
    trial.wait(lambda: trial.driver.guard['owner'] == 'none')


def test_nav2_abort_during_follow_latches(trial):
    command = start(trial)
    trial.abort_follow = True
    assert finish(trial) == [(command, 'failed', {'error_code': 'follow_nav2_failed'})]
    assert trial.driver.halted == 'follow_nav2_failed'
    trial.abort_follow = False
    with pytest.raises(ValueError, match='follow_nav2_failed'):
        trial.driver.validate_action(follow())


def test_other_card_is_rejected_before_motion(trial):
    trial.laptop.card = 36
    trial.wait(lambda: trial.driver.follow_input.status.get(36) is not None)
    with pytest.raises(ValueError, match='follow_card_mismatch'):
        trial.driver.validate_action(follow())
    assert trial.goals == []
