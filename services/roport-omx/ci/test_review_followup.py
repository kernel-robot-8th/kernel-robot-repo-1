"""Review follow-up for the ACT runtime migration: no robot, no serial, no GPU."""

import os
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_test_support import synthetic_profile

from roport_omx import safe_pose
from roport_omx.act_support import Profile, Rejected
from roport_omx.lerobot_runtime import LeRobotDevices


class ReviewFollowupTests(unittest.TestCase):
    def test_reset_run_state_clears_previous_run_alignment_state(self):
        devices = LeRobotDevices()
        devices.applied_alignment_actions = 3
        devices.alignment_fault = {"reason": "old"}
        devices._alignment_velocity = {"shoulder": 10}
        devices.startup_torque_off_verified = True
        devices.reset_run_state()
        self.assertEqual(devices.applied_alignment_actions, 0)
        self.assertIsNone(devices.alignment_fault)
        self.assertIsNone(devices._alignment_velocity)
        self.assertFalse(devices.startup_torque_off_verified)

    def test_non_dict_policy_is_rejected_not_attribute_error(self):
        data = deepcopy(synthetic_profile())
        operation = next(iter(data["policies"]))
        phase = next(iter(data["policies"][operation]))
        data["policies"][operation][phase] = "not-a-dict"
        with self.assertRaises(Rejected):
            Profile(data)

    def test_safe_pose_file_is_fsynced_before_replace(self):
        order = []
        real_fsync, real_replace = os.fsync, os.replace
        with (
            patch.object(
                safe_pose.os,
                "fsync",
                lambda fd: (order.append("fsync"), real_fsync(fd)),
            ),
            patch.object(
                safe_pose.os,
                "replace",
                lambda a, b: (order.append("replace"), real_replace(a, b)),
            ),
            tempfile.TemporaryDirectory() as directory,
        ):
            safe_pose.save_safe_pose(Path(directory) / "p.json", "omx1", [0] * 6)
        self.assertEqual(order, ["fsync", "replace"])


if __name__ == "__main__":
    unittest.main()
