"""shelf_direct: 놓음 판정 PlaceWatch·손끝 들기 연결.

가짜 판정기·계획기로 연결 동작을 보고, numpy·OpenCV가 있는 환경(현장 il3 venv)에서는 Panel에서 가져온
실제 판정기·계획기도 한 번 돌려 본다(CI의 core 환경에는 없어 건너뜀).
"""

import importlib.util
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.shelf_direct.contract import ARM_JOINTS, WIDTH
from roport_omx.shelf_direct.judge import PlaceWatch
from roport_omx.shelf_direct.lift import make_lifter
from roport_omx.shelf_direct.ports import Observation

HAVE_CV = all(importlib.util.find_spec(m) for m in ("numpy", "cv2"))
PRESENT = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -20.0,
    "elbow_flex": 30.0,
    "wrist_flex": 20.0,
    "wrist_roll": 0.0,
    "gripper": 40.0,
}


class FakeJudge:
    def __init__(self, placed_after=2):
        self.calls, self.placed_after, self.t0 = [], placed_after, None

    def reset(self, t0):
        self.t0, self.calls = t0, []

    def update(self, gray, joints, t):
        self.calls.append((gray, joints, t))
        return {"placed": len(self.calls) >= self.placed_after, "box": {"id": 16}}


def wait_for(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not cond():
        time.sleep(0.005)
    return cond()


def obs(at=1.0):
    return Observation(
        state=tuple(float(i) for i in range(WIDTH)), images={"front": "img"}, at=at
    )


class PlaceWatchTest(unittest.TestCase):
    def make(self, judge, clock, **kw):
        w = PlaceWatch(
            [16],
            judge_factory=lambda ids: (judge, "test"),
            to_gray=lambda i: f"gray-{i}",
            copy=lambda i: i,
            wall=clock,
            **kw,
        )
        self.addCleanup(w.close)
        return w

    def test_runs_at_rate_and_ends_on_placed(self):
        now = [100.0]
        judge = FakeJudge(placed_after=2)
        w = self.make(judge, lambda: now[0])
        w.reset()
        self.assertEqual(judge.t0, 100.0)
        self.assertIsNone(w.end_check(obs(), 0))
        self.assertTrue(wait_for(lambda: len(judge.calls) == 1))
        now[0] += 0.05  # 6 Hz보다 빠름: 넘기지 않는다
        w.end_check(obs(), 0)
        time.sleep(0.05)
        self.assertEqual(len(judge.calls), 1)
        gray, joints, _ = judge.calls[0]
        self.assertEqual(gray, "gray-img")
        self.assertEqual(joints["left_shoulder_pan"], 0.0)
        self.assertEqual(joints["right_gripper"], 11.0)
        now[0] += 0.2
        w.end_check(obs(), 0)
        self.assertTrue(wait_for(lambda: w.placed() is not None))
        self.assertEqual(w.end_check(obs(), 0), "placed")
        hit = w.placed()
        self.assertEqual((hit["version"], hit["judged_ts"]), ("test", 100.25))
        self.assertTrue(hit["judged_at"].endswith("Z"))

    def test_shadow_records_without_ending(self):
        now = [0.0]
        w = self.make(FakeJudge(placed_after=1), lambda: now[0], shadow=True)
        w.reset()
        now[0] = 1.0
        w.end_check(obs(), 0)
        self.assertTrue(wait_for(lambda: w.placed() is not None))
        self.assertIsNone(w.end_check(obs(), 0))
        self.assertEqual(w.summary()["updates"], 1)

    def test_judge_exception_is_recorded_not_raised(self):
        class Broken(FakeJudge):
            def update(self, *a):
                raise ValueError("bad frame")

        now = [1.0]
        w = self.make(Broken(), lambda: now[0])
        w.reset(0.0)
        w.end_check(obs(), 0)
        self.assertTrue(wait_for(lambda: w.summary()["errors"] == 1))
        self.assertIsNone(w.placed())


class FakeLifter:
    def __init__(self, result):
        self.result, self.calls = result, []

    def plan(self, joints, side, dz):
        self.calls.append((joints, side, dz))
        return self.result


class LiftTest(unittest.TestCase):
    def test_plan_feeds_both_arm_names_and_returns_arm_joints(self):
        wp = {f"left_{m}": PRESENT[m] for m in ARM_JOINTS} | {
            "left_shoulder_lift": -25.0
        }
        fake = FakeLifter(("lift", [wp]))
        plan = make_lifter(0.04, factory=lambda: (fake, 15.0))
        points = plan(PRESENT, "left")
        self.assertEqual(points, [{m: wp[f"left_{m}"] for m in ARM_JOINTS}])
        joints, side, dz = fake.calls[0]
        self.assertEqual((side, dz), ("left", 0.04))
        self.assertEqual(
            joints["right_shoulder_lift"], -20.0
        )  # 다른 팔은 같은 값으로 채움

    def test_abnormal_or_missing_plan_goes_home_directly(self):
        jump = {f"right_{m}": PRESENT[m] for m in ARM_JOINTS} | {
            "right_elbow_flex": 60.0
        }
        for result in (("none", []), ("lift", [jump]), ("lift", [{"right_x": 1}])):
            plan = make_lifter(factory=lambda r=result: (FakeLifter(r), 15.0))
            self.assertEqual(plan(PRESENT, "right"), [], result)
        self.assertEqual(
            make_lifter(0.0, factory=lambda: (FakeLifter(("lift", [jump])), 15.0))(
                PRESENT, "left"
            ),
            [],
        )

        def broken():
            raise ImportError("numpy")

        self.assertEqual(make_lifter(factory=broken)(PRESENT, "left"), [])


@unittest.skipUnless(HAVE_CV, "numpy·OpenCV 환경(현장 il3 venv)에서만")
class PanelVendorTest(unittest.TestCase):
    def test_real_lifter_plans_a_bounded_path(self):
        plan = make_lifter(0.04)
        points = plan(PRESENT, "left")
        prev = {m: PRESENT[m] for m in ARM_JOINTS}
        for p in points:
            self.assertLessEqual(max(abs(p[m] - prev[m]) for m in ARM_JOINTS), 15.0)
            prev = p

    def test_real_judge_on_blank_frame_is_not_placed(self):
        import numpy as np

        w = PlaceWatch([16])
        self.addCleanup(w.close)
        w.reset()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        w.feed(
            Observation(state=(0.0,) * WIDTH, images={"front": frame}, at=time.time())
        )
        self.assertTrue(wait_for(lambda: w.summary()["updates"] == 1, 5.0))
        last = w.summary()["last"]
        self.assertFalse(last["placed"])
        self.assertNotIn("error", last)


if __name__ == "__main__":
    unittest.main()
