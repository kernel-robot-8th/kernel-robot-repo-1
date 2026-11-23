"""Exercise alignment planning using synthetic positions and a controlled clock."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.act_support import Rejected
from roport_omx.start_alignment import StartAlignment


class StartAlignmentTests(unittest.TestCase):
    def test_pinned_start_remains_strict_with_distinct_demonstration_target_tolerance(
        self,
    ):
        config = {
            "start": [0] * 6,
            "target": [2] * 6,
            "start_tolerance": [0.2] * 6,
            "tracking_tolerance": [1] * 6,
            "max_rate": [1] * 6,
            "timeout_s": 10,
            "settled_samples": 5,
        }
        plan = StartAlignment(config, target_tolerance=[1] * 6)
        with self.assertRaisesRegex(Rejected, "registered alignment start"):
            plan.begin([0.3] * 6, 0)
        plan.begin([0] * 6, 0)
        self.assertEqual(plan.step([0] * 6, 1), [1] * 6)
        self.assertEqual(plan.step([1] * 6, 2), [2] * 6)
        for now in (2.1, 2.2, 2.3, 2.4):
            self.assertEqual(plan.step([1.2] * 6, now), [2] * 6)
            self.assertFalse(plan.done)
        self.assertIsNone(plan.step([1.2] * 6, 2.5))
        plan.check_hold([1.2] * 6)
        with self.assertRaises(Rejected):
            plan.check_hold([0.99] * 6)
        with self.assertRaises(Rejected):
            StartAlignment(config, target_tolerance=[1.01] * 6)

    def planner(self, **overrides):
        self.start = [0, 0, 0, 0, 0, 50]
        self.target = [2, -2, 0, 0, 0, 50]
        return StartAlignment(
            dict(
                {
                    "start": self.start,
                    "target": self.target,
                    "start_tolerance": [0.1] * 6,
                    "tracking_tolerance": [0.5] * 6,
                    "max_rate": [1, 2, 1, 1, 1, 1],
                    "timeout_s": 3,
                    "settled_samples": 2,
                },
                **overrides,
            )
        )

    def test_synchronizes_joint_targets_and_requires_fresh_settling_samples(self):
        plan = self.planner()
        plan.begin(self.start, 10)
        measured = self.start
        now = 10
        for timestamp in (10.1, 10.8, 11.4, 11.9, 12):
            command = plan.step(measured, timestamp)
            self.assertAlmostEqual(command[0], -command[1])
            for value, previous, rate in zip(
                command, measured, plan.max_rate, strict=True
            ):
                self.assertLessEqual(
                    abs(value - previous), rate * (timestamp - now) + 1e-12
                )
            measured, now = command, timestamp
            self.assertFalse(plan.done)

        self.assertEqual(command, self.target)
        self.assertEqual(plan.step(self.target, 12.1), self.target)
        self.assertEqual(plan.step(self.target, 12.1), self.target)
        self.assertFalse(plan.done)
        self.assertIsNone(plan.step(self.target, 12.2))
        self.assertTrue(plan.done)

    def test_rejects_outside_start_and_tracks_last_command(self):
        plan = self.planner()
        with self.assertRaisesRegex(Rejected, "registered alignment start"):
            plan.begin([0.2, 0, 0, 0, 0, 50], 0)
        plan.begin([0.05, 0, 0, 0, 0, 50], 0)
        self.assertEqual(plan.step([0.05, 0, 0, 0, 0, 50], 0)[0], 0.05)
        self.assertGreater(plan.step(self.start, 1)[0], 1)
        with self.assertRaisesRegex(Rejected, "tracking limit"):
            plan.step(self.start, 1.1)

    def test_rejects_invalid_time_and_timeout(self):
        plan = self.planner()
        with self.assertRaises(Rejected):
            plan.step(self.start, 5)
        plan.begin(self.start, 5)
        for bad_time in (4.999, float("nan"), float("inf")):
            with (
                self.subTest(bad_time=bad_time),
                self.assertRaisesRegex(Rejected, "clock"),
            ):
                plan.step(self.start, bad_time)
        with self.assertRaisesRegex(Rejected, "timed out"):
            plan.step(self.start, 8)

    def test_fresh_write_check_accepts_only_last_planned_target_and_tracking(self):
        plan = self.planner()
        plan.begin(self.start, 0)
        command = plan.step(self.start, 0.05)
        plan.check_write(self.start, command)
        with self.assertRaisesRegex(Rejected, "differs"):
            plan.check_write(self.start, self.target)
        with self.assertRaisesRegex(Rejected, "tracking limit"):
            plan.check_write(self.target, command)

    def test_zero_distance_and_out_of_tolerance_resets_settling(self):
        pose = [0, 0, 0, 0, 0, 50]
        plan = self.planner(target=pose)
        plan.begin(pose, 0)
        self.assertEqual(plan.step(pose, 0.1), pose)
        self.assertEqual(plan.step([0.2, 0, 0, 0, 0, 50], 0.2), pose)
        self.assertEqual(plan.step(pose, 0.3), pose)
        self.assertIsNone(plan.step(pose, 0.4))

    def test_requires_valid_limits_and_enough_time_to_reach_target(self):
        for overrides in (
            {"max_rate": [0] * 6},
            {"tracking_tolerance": [float("nan")] * 6},
            {"start_tolerance": [0] * 6},
            {"settled_samples": True},
        ):
            with self.subTest(overrides=overrides), self.assertRaises(Rejected):
                self.planner(**overrides)
        plan = self.planner(timeout_s=1)
        with self.assertRaisesRegex(Rejected, "timeout is too short"):
            plan.begin(self.start, 0)
