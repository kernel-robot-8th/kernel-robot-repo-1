"""shelf_direct: ShelfRunner 실행 흐름. 가짜 로봇·정책, 장치 없이."""

import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.shelf_direct.contract import WIDTH
from roport_omx.shelf_direct.ports import (
    Observation,
    PolicyUnavailable,
    RobotNotReady,
    SafeStartRefused,
)
from roport_omx.shelf_direct.runner import ReturnWatch, RunSettings, ShelfRunner

GRIP_OPEN, GRIP_CLOSED = 70.0, 40.0


def pose(offset=0.0, grip=GRIP_CLOSED):
    """12개: 관절은 offset, 그리퍼(5, 11)는 grip."""
    return tuple(grip if i in (5, 11) else offset for i in range(WIDTH))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += max(s, 0.0)


class FakeRobot:
    def __init__(self, clock, path=None, refuse=False, send_error=None):
        self.clock = clock
        self.path = path or (lambda frame: pose())
        self.refuse, self.send_error = refuse, send_error
        self.freeze, self.release = threading.Event(), threading.Event()
        self.frame, self.sent, self.calls = 0, [], []

    def connect(self):
        self.calls.append("connect")
        if self.refuse:
            raise RobotNotReady("left gripper: motor not answering")

    def observe(self):
        self.clock.t += 0.002
        state = self.path(self.frame)
        self.frame += 1
        return Observation(state=state, images={}, at=self.clock.t)

    def send(self, action):
        if self.send_error:
            raise self.send_error
        self.sent.append(tuple(action))

    def start_poses(self):
        return {"left": {"shoulder_pan": 0.0}} if self.sent else {}

    def land(self, mode):
        self.calls.append(f"land {mode}")

    def disconnect(self):
        self.calls.append("disconnect")


class FakePolicy:
    def __init__(self, error_at=None, none_first=0):
        self.error_at, self.none_first, self.n, self.tasks = error_at, none_first, 0, []

    def new_episode(self, task):
        self.tasks.append(task)

    def act(self, obs):
        self.n += 1
        if self.error_at is not None and self.n >= self.error_at:
            raise PolicyUnavailable("no reply")
        if self.n <= self.none_first:
            return None
        return list(obs.state)


SHORT = RunSettings(episode_s=5)


def run(robot, policy=None, settings=SHORT, stop=None, **kw):
    runner = ShelfRunner(
        robot,
        policy or FakePolicy(),
        settings,
        clock=robot.clock,
        sleep=robot.clock.sleep,
        **kw,
    )
    return runner.run("INBOUND 2B", stop or threading.Event())


def go_and_return(frame):
    """0~9 처음 자세, 10~29 벗어남(30), 30~ 돌아와 그리퍼 놓음."""
    if frame < 10:
        return pose(0.0, GRIP_OPEN)
    if frame < 30:
        return pose(30.0, GRIP_OPEN)
    return pose(1.0, 50.0)


class RunnerTest(unittest.TestCase):
    def test_timeout_runs_at_fixed_rate_and_lands_soft(self):
        clock = Clock()
        robot = FakeRobot(clock)
        r = run(robot, settings=RunSettings(episode_s=3, end_on_return=False))
        self.assertEqual(r.end_reason, "timeout")
        self.assertAlmostEqual(r.frames, 90, delta=1)
        self.assertEqual(r.landing, "soft")
        self.assertEqual(robot.calls, ["connect", "land soft", "disconnect"])

    def test_return_detection_ends_and_fast_release_lands_fast(self):
        clock = Clock()
        robot = FakeRobot(clock, go_and_return)
        r = run(robot, settings=RunSettings(episode_s=10, fast_release=True))
        self.assertEqual(r.end_reason, "returned")
        self.assertEqual(r.landing, "fast")
        self.assertAlmostEqual(r.frames, 39, delta=1)  # 30번째부터 0.3초 유지

    def test_return_without_fast_release_lands_soft(self):
        r = run(FakeRobot(Clock(), go_and_return), settings=RunSettings(episode_s=10))
        self.assertEqual((r.end_reason, r.landing), ("returned", "soft"))

    def test_gripper_still_open_is_not_returned(self):
        def path(frame):
            return go_and_return(frame) if frame < 30 else pose(1.0, GRIP_OPEN)

        r = run(FakeRobot(Clock(), path), settings=RunSettings(episode_s=3))
        self.assertEqual(r.end_reason, "timeout")

    def test_end_check_placed_holds_only_when_aligner_ready(self):
        def placed(obs, now):
            return "placed" if now > 1.0 else None

        r = run(FakeRobot(Clock()), end_checks=[placed], hold_ready=lambda: True)
        self.assertEqual((r.end_reason, r.landing), ("placed", "hold"))
        r = run(FakeRobot(Clock()), end_checks=[placed], hold_ready=lambda: False)
        self.assertEqual((r.end_reason, r.landing), ("placed", "soft"))

    def test_stop_and_freeze(self):
        clock = Clock()
        robot = FakeRobot(clock)
        stop = threading.Event()
        stop.set()
        r = run(robot, stop=stop)
        self.assertEqual((r.end_reason, r.frames), ("stopped", 0))
        robot = FakeRobot(Clock())
        robot.freeze.set()
        r = run(robot)
        self.assertEqual((r.end_reason, r.landing), ("frozen", "soft"))

    def test_refused_connect_does_not_land(self):
        robot = FakeRobot(Clock(), refuse=True)
        r = run(robot)
        self.assertEqual(r.end_reason, "refused")
        self.assertEqual(robot.calls, ["connect"])

    def test_errors_still_land(self):
        for policy, robot_kw in (
            (FakePolicy(error_at=5), {}),
            (FakePolicy(), {"send_error": SafeStartRefused("first target too far")}),
            (FakePolicy(), {"send_error": OSError("bus")}),
        ):
            robot = FakeRobot(Clock(), **robot_kw)
            r = run(robot, policy)
            self.assertEqual(r.end_reason, "error", r.error)
            self.assertIn("land soft", robot.calls)
            self.assertEqual(robot.calls[-1], "disconnect")

    def test_no_action_is_a_stall_not_a_send(self):
        robot = FakeRobot(Clock())
        r = run(
            robot,
            FakePolicy(none_first=10),
            settings=RunSettings(episode_s=3, end_on_return=False),
        )
        self.assertEqual(r.stalls, 10)
        self.assertEqual(len(robot.sent), r.frames - 10)


class ReturnWatchTest(unittest.TestCase):
    def test_needs_leave_then_return_held(self):
        w = ReturnWatch(RunSettings())
        t = 0.0
        for _ in range(5):
            self.assertFalse(w(pose(0.0, 50.0), t))
        self.assertFalse(w(pose(5.0, 50.0), 1.0))  # 벗어나지 않았다
        self.assertFalse(w(pose(25.0, 50.0), 2.0))
        self.assertFalse(w(pose(2.0, 50.0), 3.0))
        self.assertFalse(w(pose(2.0, 50.0), 3.2))
        self.assertTrue(w(pose(2.0, 50.0), 3.31))


if __name__ == "__main__":
    unittest.main()
