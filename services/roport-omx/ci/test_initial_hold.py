"""Reproduce torque-off register jitter without allowing physical drift."""

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.act_support import Rejected
from roport_omx.lerobot_runtime import LeRobotDevices, _seed_evaluation_hold


class InitialHoldTests(unittest.TestCase):
    def test_fault_diagnostic_failure_does_not_write_or_replace_stop_error(self):
        adapter = LeRobotDevices()
        bus = Mock()
        bus.sync_read.side_effect = [{"joint": 123}, OSError("unavailable")]
        adapter.profile = {"arm_id": "omx1"}
        adapter.arms = {"omx1": Mock(bus=bus)}
        adapter.capture_alignment_fault()
        self.assertEqual(
            adapter.alignment_fault["raw_registers"]["Goal_Position"], {"joint": 123}
        )
        self.assertIn("unavailable", adapter.alignment_fault["read_error"])
        bus.sync_write.assert_not_called()

    def bus(self, goal=913, after=912, torque=0):
        bus = Mock(motors={"shoulder": object(), "elbow": object()})
        present = {"shoulder": 912, "elbow": 3165}
        bus.sync_read.side_effect = [
            dict.fromkeys(bus.motors, 0),
            present,
            {"shoulder": goal, "elbow": 3164},
            {"shoulder": after, "elbow": 3165},
            dict.fromkeys(bus.motors, torque),
        ]
        return bus, present

    def test_one_raw_count_jitter_retains_exact_measured_goal_and_never_arms(self):
        bus, present = self.bus()
        _seed_evaluation_hold(bus)
        bus.sync_write.assert_called_once_with(
            "Goal_Position", present, normalize=False
        )
        bus.enable_torque.assert_not_called()

    def test_drift_bad_readback_and_torque_change_reject(self):
        for args in (
            {"goal": 914},
            {"after": 914},
            {"goal": 911, "after": 913},
            {"torque": 1},
        ):
            with self.subTest(args=args):
                bus, _ = self.bus(**args)
                with self.assertRaises(Rejected):
                    _seed_evaluation_hold(bus)
                bus.enable_torque.assert_not_called()

    def test_torque_on_or_malformed_position_never_writes(self):
        for reads in (
            [{"shoulder": 1, "elbow": 0}],
            [{"shoulder": False, "elbow": 0}],
            [{"shoulder": 0.0, "elbow": 0}],
            [{"shoulder": 0, "elbow": 0}, {"shoulder": 912}],
            [{"shoulder": 0, "elbow": 0}, {"shoulder": True, "elbow": 3165}],
        ):
            with self.subTest(reads=reads):
                bus, _ = self.bus()
                bus.sync_read.side_effect = reads
                with self.assertRaises(Rejected):
                    _seed_evaluation_hold(bus)
                bus.sync_write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
