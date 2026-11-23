"""shelf_direct: 안전 시작·착지와 양팔 Robot 어댑터. 가짜 모터 버스, lerobot·장치 없이."""

import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.shelf_direct.contract import CAMERAS, JOINTS, SIDES, WIDTH, join
from roport_omx.shelf_direct.ports import RobotNotReady, SafeStartRefused
from roport_omx.shelf_direct.robot import BiOmxRobot, RobotConfig
from roport_omx.shelf_direct.safety import Mover, SafeStart, SafetySettings, preflight

START = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -60.0,
    "elbow_flex": 50.0,
    "wrist_flex": 40.0,
    "wrist_roll": 0.0,
    "gripper": 60.0,
}


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


class FakeArm:
    """ArmIO: 목표를 바로 따라가는 버스. 기록: 쓴 목표, PWM, 토크, 재부팅."""

    def __init__(self, side, pose=None, *, grip_block=None):
        self.side = side
        self.pos = dict(pose or START)
        self.goals, self.pwm, self.events = [], [], []
        self.torque, self.closed = True, None
        self.grip_block = grip_block  # 그리퍼가 이 값 아래로 못 내려감(물건에 닿음)

    def read(self):
        return dict(self.pos)

    def write_goals(self, goals):
        self.goals.append(dict(goals))
        for m, v in goals.items():
            if m == "gripper" and self.grip_block is not None:
                v = max(v, self.grip_block)
            self.pos[m] = v

    def set_pwm(self, ratio):
        self.pwm.append(round(ratio, 3))

    def torque_off(self):
        self.torque = False
        self.events.append("torque_off")

    def reboot(self, joint):
        self.events.append(f"reboot {joint}")
        self.pos[joint] = max(-100.0, min(100.0, self.pos[joint] - 200.0))

    def close(self, *, torque_off):
        self.closed = torque_off
        if torque_off:
            self.torque = False


def settings(**kw):
    return SafetySettings(**kw)


class SafeStartTest(unittest.TestCase):
    def drive(self, ss, clock, target, steps):
        out = None
        for _ in range(steps):
            clock.sleep(1 / 30)
            out = ss.filter(target)
        return out

    def test_gripper_closes_then_slow_approach_then_normal(self):
        clock, arm = Clock(), FakeArm("left")
        ss = SafeStart(arm, settings(), clock)
        ss.begin()
        self.assertEqual(arm.pwm, [0.3])
        target = dict(START, shoulder_pan=2.0, gripper=50.0)
        first = ss.filter(target)
        self.assertEqual(ss.home, START)  # 첫 목표 직전 실측이 처음 자세
        self.assertEqual(first["shoulder_pan"], 0.0)  # gripper 단계: 팔은 그대로
        self.drive(ss, clock, target, 41)  # 60 → 47 at 10 u/s ≈ 1.3 s
        self.assertEqual(ss.phase, "approach")
        out = self.drive(ss, clock, target, 3)
        self.assertLessEqual(out["shoulder_pan"], 2.0)
        self.drive(ss, clock, target, 40)
        self.assertEqual(ss.phase, "normal")
        self.assertEqual(arm.pwm[-1], 1.0)
        self.assertEqual(ss.filter(dict(target, shoulder_pan=9.0))["shoulder_pan"], 9.0)

    def test_moving_target_switches_to_full_output_catch_up(self):
        clock, arm = Clock(), FakeArm("left", dict(START, gripper=47.0))
        ss = SafeStart(arm, settings(), clock)
        target = dict(START, gripper=47.0)
        self.drive(ss, clock, target, 2)
        self.assertEqual(ss.phase, "approach")
        moved = dict(target, shoulder_lift=-50.0)  # 시작 목표보다 10 움직임
        out = self.drive(ss, clock, moved, 1)
        self.assertEqual(arm.pwm[-1], 1.0)
        self.assertGreater(out["shoulder_lift"] - (-60.0), 15.0 / 30)  # 60 u/s

    def test_refuses_far_first_target_and_out_of_range_lift(self):
        ss = SafeStart(FakeArm("left"), settings(), Clock())
        with self.assertRaises(SafeStartRefused):
            ss.filter(dict(START, shoulder_lift=40.0))  # 100 차이
        ss = SafeStart(
            FakeArm("left", dict(START, elbow_flex=130.0)), settings(), Clock()
        )
        with self.assertRaises(SafeStartRefused):
            ss.filter(START)

    def test_rotation_joint_past_one_turn_is_rebooted(self):
        arm = FakeArm("right", dict(START, wrist_roll=150.0))
        ss = SafeStart(arm, settings(), Clock())
        ss.filter(START)
        self.assertEqual(arm.events, ["reboot wrist_roll"])
        self.assertEqual(ss.home["wrist_roll"], -50.0)


class MoverTest(unittest.TestCase):
    def mover(self, arm, clock, **kw):
        return Mover(arm, settings(), clock=clock, sleep=clock.sleep, **kw)

    def test_soft_land_goes_home_synchronized_opens_parks_fades(self):
        clock = Clock()
        away = dict(START, shoulder_pan=30.0, elbow_flex=20.0, gripper=40.0)
        arm = FakeArm("left", away)
        ok = self.mover(arm, clock).soft_land(START, threading.Event())
        self.assertTrue(ok)
        # 동기 이동: 중간 목표에서 두 관절의 진행률이 같다
        mid = arm.goals[len(arm.goals) // 4]
        p1 = (30.0 - mid["shoulder_pan"]) / 30.0
        p2 = (20.0 - mid["elbow_flex"]) / (20.0 - 50.0)
        self.assertAlmostEqual(p1, p2, places=6)
        self.assertEqual(arm.pos["shoulder_pan"], 0.0)
        self.assertEqual(arm.pos["wrist_roll"], 0.0)
        self.assertAlmostEqual(
            arm.pos["gripper"], 47.0
        )  # 처음 값(60)으로 연 뒤 park에서 닫힘
        self.assertEqual(arm.events[-1], "torque_off")
        self.assertEqual(arm.pwm[-1], 1.0)
        self.assertAlmostEqual(arm.pwm[-2], 0.0)

    def test_does_not_move_when_too_far_or_out_of_range(self):
        clock = Clock()
        arm = FakeArm("left", dict(START, shoulder_pan=-100.0))
        far_home = dict(START, shoulder_pan=60.0)  # 160 차이 > 150
        self.assertFalse(self.mover(arm, clock).soft_land(far_home, threading.Event()))
        self.assertEqual([g for g in arm.goals if "shoulder_pan" in g], [])
        self.assertFalse(arm.torque)

    def test_park_stops_gripper_on_contact(self):
        clock = Clock()
        arm = FakeArm("left", dict(START, gripper=70.0), grip_block=60.0)
        self.mover(arm, clock).park()
        self.assertGreaterEqual(arm.pos["gripper"], 60.0)

    def test_freeze_holds_until_release_then_lands(self):
        clock = Clock()
        arm = FakeArm("left", dict(START, shoulder_pan=20.0))
        freeze, release = threading.Event(), threading.Event()
        freeze.set()
        release.set()  # 이미 놓임: 기다린 뒤 이어서 착지
        ok = self.mover(arm, clock, freeze=freeze).soft_land(START, release)
        self.assertTrue(ok)
        self.assertEqual(arm.pos["shoulder_pan"], 0.0)

    def test_lifter_waypoints_come_first(self):
        clock = Clock()
        arm = FakeArm("left", dict(START, shoulder_lift=-40.0))
        up = {m: START[m] for m in JOINTS[:5]} | {"shoulder_lift": -45.0}
        self.mover(arm, clock, lifter=lambda present, side: [up]).go_home(START)
        lifted = next(
            i for i, g in enumerate(arm.goals) if g.get("shoulder_lift") == -45.0
        )
        self.assertLess(lifted, len(arm.goals) - 1)
        self.assertEqual(arm.pos["shoulder_lift"], -60.0)

    def test_fast_land_fades_from_full_output(self):
        clock = Clock()
        arm = FakeArm("left")
        self.mover(arm, clock).fast_land()
        self.assertEqual(arm.pwm[:5], [0.8, 0.6, 0.4, 0.2, 0.0])
        self.assertFalse(arm.torque)


class FakeCam:
    def __init__(self, name):
        self.name, self.is_connected = name, True

    def read_latest(self):
        return f"frame-{self.name}"

    def disconnect(self):
        self.is_connected = False


def config():
    return RobotConfig(cameras={c: f"/dev/video-{c}" for c in CAMERAS})


class RobotTest(unittest.TestCase):
    def make(self, *, problems=(), fail_side=None):
        clock = Clock()
        self.arms = {}

        def arm_factory(side, port, cfg):
            if side == fail_side:
                raise OSError("no port")
            self.arms[side] = FakeArm(side)
            return self.arms[side]

        return BiOmxRobot(
            config(),
            arm_factory=arm_factory,
            camera_factory=lambda cfg: {c: FakeCam(c) for c in CAMERAS},
            preflight_fn=lambda arms: list(problems),
            clock=clock,
            sleep=clock.sleep,
        ), clock

    def test_config_needs_both_ports_and_all_cameras(self):
        with self.assertRaises(ValueError):
            RobotConfig(cameras={"front": 0})
        with self.assertRaises(ValueError):
            RobotConfig(ports={"left": "/dev/x"}, cameras={c: 0 for c in CAMERAS})

    def test_preflight_problem_refuses_before_any_arm_opens(self):
        robot, _ = self.make(problems=["left gripper: motor not answering"])
        with self.assertRaises(RobotNotReady):
            robot.connect()
        self.assertEqual(self.arms, {})

    def test_failed_second_arm_lands_the_first(self):
        robot, _ = self.make(fail_side="right")
        with self.assertRaises(RobotNotReady):
            robot.connect()
        self.assertFalse(self.arms["left"].torque)

    def test_observe_send_and_land(self):
        robot, _ = self.make()
        robot.connect()
        obs = robot.observe()
        self.assertEqual(obs.state, tuple(join({s: START for s in SIDES})))
        self.assertEqual(sorted(obs.images), sorted(CAMERAS))
        robot.send(list(obs.state))
        self.assertEqual(robot.start_poses(), {"left": START, "right": START})
        self.assertEqual(len(self.arms["left"].goals), 1)
        with self.assertRaises(ValueError):
            robot.send([0.0] * (WIDTH - 1))
        self.arms["left"].pos["shoulder_pan"] = 10.0
        robot.land("soft")
        self.assertEqual(self.arms["left"].pos["shoulder_pan"], 0.0)
        self.assertTrue(all(a.closed for a in self.arms.values()))
        robot.disconnect()

    def test_hold_keeps_torque(self):
        robot, _ = self.make()
        robot.connect()
        robot.land("hold")
        self.assertTrue(all(a.torque and a.closed is False for a in self.arms.values()))


class PreflightTest(unittest.TestCase):
    def test_reports_bad_ticks_silent_motor_and_bad_port(self):
        class P:
            def __init__(self, out, rc=0):
                self.out, self.returncode = out, rc
                self.stdout = self

            def poll(self):
                return self.returncode

            def read(self):
                return self.out

            def kill(self):
                pass

        outs = iter([P("[100, 5000, null, 1, 2, 3]"), P("", rc=3)])
        problems = preflight(
            [("left", "/dev/a"), ("right", "/dev/b")], popen=lambda *a, **k: next(outs)
        )
        self.assertEqual(len(problems), 3)
        self.assertIn("shoulder_lift", problems[0])
        self.assertIn("elbow_flex", problems[1])
        self.assertIn("cannot open port", problems[2])


if __name__ == "__main__":
    unittest.main()
