"""OMX 3 실행 상태기계 검사 (가짜 시계·로봇·마커·정책, 표준 라이브러리만)."""

import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.omx3.contract import ErrorCode, Phase  # noqa: E402
from roport_omx.omx3.ports import Observation, PolicyUnavailable  # noqa: E402
from roport_omx.omx3.runner import Runner  # noqa: E402
from roport_omx.omx3.settings import Omx3Settings  # noqa: E402

S = Omx3Settings(timeout_s=5.0, home_timeout_s=5.0)
INIT = list(S.init_pose)
BOX = (300.0, 300.0)  # 창고 바닥 쪽
TRAY = (480.0, 70.0)  # 트레이 영역 안


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += max(dt, 1e-3)


class FakeRobot:
    """보낸 목표값을 바로 따라가는 팔."""

    def __init__(self, start=None, error=None, fail_at=None):
        self.state = list(start or INIT)
        self.error, self.fail_at = error, fail_at
        self.sent = self.held = 0

    def ready(self):
        return self.error

    def observe(self):
        return Observation(list(self.state), {"sent": self.sent})

    def send(self, action):
        self.sent += 1
        if self.fail_at is not None and self.sent >= self.fail_at:
            raise OSError("serial lost")
        self.state = list(action)

    def hold(self):
        self.held += 1


class FakeMarker:
    def __init__(self, where):
        self.where, self.resets, self.marked = where, 0, 0

    def reset(self):
        self.resets += 1

    def find(self, frame):
        return self.where(frame)

    def mark(self, frame):
        self.marked += 1
        return {"marked": frame}, self.where(frame)


class ScriptPolicy:
    """집기 흉내: away 스텝 동안 팔을 init에서 벗어나게 했다가 init으로 되돌린다."""

    def __init__(self, away=30, fail_at=None):
        self.away, self.fail_at, self.n, self.frames = away, fail_at, 0, []

    def reset(self):
        self.n = 0

    def act(self, state, marked_front):
        self.n += 1
        self.frames.append(marked_front)
        if self.fail_at is not None and self.n >= self.fail_at:
            raise PolicyUnavailable("no reply")
        if self.n < self.away:
            return [INIT[0] + 20, *INIT[1:5], 40.0]
        return list(INIT)


def box_until(robot, sent):
    # 팔이 sent번 움직이기 전에는 상자 위치, 그 뒤에는 트레이 위치
    return lambda frame: BOX if robot.sent < sent else TRAY


def make(robot, policy, where, clock=None):
    markers = []

    def factory(marker_id):
        markers.append(FakeMarker(where))
        return markers[-1]

    clock = clock or Clock()
    return Runner(robot, policy, factory, S, clock, clock.sleep), markers


class Success(unittest.TestCase):
    def test_phases_and_departure_ready(self):
        robot, policy = FakeRobot(), ScriptPolicy()
        runner, markers = make(robot, policy, box_until(robot, 20))
        phases = []
        out = runner.run(17, on_phase=lambda p, info: phases.append(p))
        self.assertEqual(out.phase, Phase.COMPLETE, out)
        self.assertTrue(out.departure_ready)
        self.assertEqual(
            phases,
            [Phase.MARKER_CHECK, Phase.HOMING, Phase.RUNNING, Phase.VERIFY, Phase.COMPLETE],
        )
        self.assertEqual(len(markers), 1)  # 명령 1개 = 마커 1개
        self.assertEqual(markers[0].resets, 1)
        # 정책에는 매번 원 그린 프레임이 들어간다
        self.assertTrue(all("marked" in f for f in policy.frames))

    def test_needs_consecutive_done_frames(self):
        # 트레이 검출이 한 프레임씩 끊기면 완료가 아니다. 끊김 없이 15프레임이어야 완료
        robot = FakeRobot()
        frames = iter([TRAY] + [TRAY, None] * 20)
        runner, _ = make(robot, ScriptPolicy(away=1), lambda f: next(frames, TRAY))
        out = runner.run(17)
        self.assertEqual(out.phase, Phase.COMPLETE, out)
        self.assertEqual(robot.sent, 20)  # 깜빡이는 40프레임 중 조건이 깨진 20프레임만 정책 실행

    def test_no_policy_action_while_confirming_done(self):
        # 조건이 처음 맞은 뒤 확인하는 15프레임 동안 정책 전송 0회(움찔거림 방지)
        robot, policy = FakeRobot(), ScriptPolicy(away=1)
        frames = iter([TRAY] + [None] * 5)  # 사전 점검 1 + 정책 실행 5프레임, 이후 트레이
        runner, _ = make(robot, policy, lambda f: next(frames, TRAY))
        out = runner.run(17)
        self.assertEqual(out.phase, Phase.COMPLETE, out)
        self.assertEqual(policy.n, 5)
        self.assertEqual(robot.sent, 5)

    def test_homes_slowly_from_away_pose(self):
        robot = FakeRobot(start=[10.0, *INIT[1:]])
        runner, _ = make(robot, ScriptPolicy(), box_until(robot, 40))
        out = runner.run(17)
        self.assertEqual(out.phase, Phase.COMPLETE, out)
        self.assertGreaterEqual(robot.sent, 10 / S.home_step)


class Rejected(unittest.TestCase):
    def test_no_motor_write_when_marker_missing_or_robot_down(self):
        for robot, where, code in (
            (FakeRobot(), lambda f: None, ErrorCode.MARKER_NOT_VISIBLE),
            (FakeRobot(error="not connected"), lambda f: BOX, ErrorCode.ROBOT_NOT_READY),
        ):
            with self.subTest(code=code):
                runner, _ = make(robot, ScriptPolicy(), where)
                self.assertEqual(runner.precheck(17), code)
                out = runner.run(17)
                self.assertEqual((out.phase, out.error_code), (Phase.REJECTED, code))
                self.assertEqual((robot.sent, robot.held), (0, 0))


class Failed(unittest.TestCase):
    def check(self, robot, policy, code, where=None, abort=None):
        runner, _ = make(robot, policy, where or (lambda f: BOX))
        out = runner.run(17, abort)
        self.assertEqual((out.phase, out.error_code), (Phase.FAILED, code), out)
        self.assertGreaterEqual(robot.held, 1)  # 실패하면 자세 유지
        return out

    def test_timeout(self):
        self.check(FakeRobot(), ScriptPolicy(away=10**6), ErrorCode.TIMEOUT)

    def test_inference_unavailable(self):
        self.check(FakeRobot(), ScriptPolicy(fail_at=5), ErrorCode.INFERENCE_UNAVAILABLE)

    def test_robot_error(self):
        self.check(FakeRobot(fail_at=5), ScriptPolicy(), ErrorCode.ROBOT_ERROR)

    def test_homing_failed(self):
        robot = FakeRobot(start=[90.0, *INIT[1:]])
        robot.send = lambda action: None  # 팔이 따라오지 않음
        self.check(robot, ScriptPolicy(), ErrorCode.HOMING_FAILED)

    def test_cancel(self):
        abort, policy = threading.Event(), ScriptPolicy(away=10**6)
        act = policy.act

        def act_then_abort(state, front):
            if policy.n >= 3:
                abort.set()
            return act(state, front)

        policy.act = act_then_abort
        self.check(FakeRobot(), policy, ErrorCode.CANCELED, abort=abort)

    def test_verify_failed_when_marker_leaves_tray(self):
        # 사전 점검 1회 + 루프 15프레임은 트레이에 보이고, VERIFY 재확인에서 사라짐
        robot = FakeRobot()
        frames = iter([TRAY] * (1 + S.done_hold_frames))
        runner, _ = make(robot, ScriptPolicy(away=1), lambda f: next(frames, None))
        out = runner.run(17)
        self.assertEqual((out.phase, out.error_code), (Phase.FAILED, ErrorCode.VERIFY_FAILED))
        self.assertFalse(out.placement_confirmed)
        self.assertTrue(out.arm_returned)


if __name__ == "__main__":
    unittest.main()
