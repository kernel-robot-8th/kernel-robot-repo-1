"""OMX 3 완료 판정 검사 (표준 라이브러리만)."""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.omx3 import judge  # noqa: E402
from roport_omx.omx3.settings import JOINTS, Omx3Settings  # noqa: E402

S = Omx3Settings()
INIT = list(S.init_pose)
TRAY = (480.0, 70.0)  # 트레이 영역 가운데
BOX = (300.0, 300.0)  # 창고 바닥 쪽


def moved(index, delta):
    state = list(INIT)
    state[index] += delta
    return state


class NearInit(unittest.TestCase):
    def test_tolerance_edge(self):
        tol = S.init_tolerance
        self.assertTrue(judge.near_init(INIT, S))
        for i in range(5):
            with self.subTest(joint=JOINTS[i]):
                self.assertTrue(judge.near_init(moved(i, tol), S))
                self.assertTrue(judge.near_init(moved(i, -tol), S))
                self.assertFalse(judge.near_init(moved(i, tol + 0.01), S))

    def test_gripper_is_not_an_arm_joint(self):
        self.assertTrue(judge.near_init(moved(5, -40), S))


class GripperOpen(unittest.TestCase):
    def test_threshold(self):
        state = list(INIT)
        state[5] = S.gripper_open_min
        self.assertTrue(judge.gripper_open(state, S))
        state[5] = S.gripper_open_min - 0.01
        self.assertFalse(judge.gripper_open(state, S))


class InTray(unittest.TestCase):
    def test_region_edges_are_inside(self):
        x0, y0, x1, y1 = S.tray_region
        for point in ((x0, y0), (x1, y1), (x0, y1), (x1, y0), TRAY):
            with self.subTest(point=point):
                self.assertTrue(judge.in_tray(point, S))

    def test_outside_or_missing(self):
        x0, y0, x1, y1 = S.tray_region
        for point in ((x0 - 0.5, y0), (x1 + 0.5, y1), (x0, y1 + 0.5), BOX, None):
            with self.subTest(point=point):
                self.assertFalse(judge.in_tray(point, S))


class IsDone(unittest.TestCase):
    def test_all_three_conditions(self):
        closed = moved(5, -(INIT[5] - 40.0))
        self.assertTrue(judge.is_done(INIT, TRAY, S))
        self.assertFalse(judge.is_done(INIT, BOX, S))  # 아직 상자 위치
        self.assertFalse(judge.is_done(INIT, None, S))  # 마커 안 보임
        self.assertFalse(judge.is_done(closed, TRAY, S))  # 그리퍼 닫힘
        self.assertFalse(judge.is_done(moved(0, 6), TRAY, S))  # 복귀 전

    def test_rejects_wrong_state_length(self):
        for state in (INIT[:5], [*INIT, 0.0]):
            with self.subTest(n=len(state)), self.assertRaises(ValueError):
                judge.is_done(state, TRAY, S)

    def test_init_pose_matches_joint_order(self):
        self.assertEqual(len(S.init_pose), len(JOINTS))
        self.assertEqual(JOINTS[-1], "gripper")


if __name__ == "__main__":
    unittest.main()
