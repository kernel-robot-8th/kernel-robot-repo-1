"""Waiting-arm safe pose registration: no robot, no serial, no GPU."""

import json
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_test_support import synthetic_profile

from roport_omx.act_support import Rejected
from roport_omx.safe_pose import (
    apply_saved_safe_poses,
    load_safe_poses,
    propagate_safe_pose,
    register_safe_position,
    safe_pose_basis,
)

FOLDED = [0.0, -65.0, 50.0, 45.0, 0.0, 50.0]


def make_profile(directory, calibration="cal-1"):
    host = synthetic_profile()
    host["calibration_version"] = calibration
    host["safe_positions"] = {"omx1": list(FOLDED), "omx2": list(FOLDED)}
    return {
        "host_profile": host,
        "arm_id": "omx1",
        "safe_pose_file": str(Path(directory) / "safe_poses.json"),
    }


def still(pose):
    return lambda: {"measured": {"omx1": [0, 0, 0, 0, 0, 0], "omx2": list(pose)}}


class RegisterTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.dir = Path(temporary.name)
        self.profile = make_profile(self.dir)
        self.safe = self.profile["host_profile"]["safe_positions"]

    def register(self, inspect, profile=None):
        return register_safe_position(
            profile or self.profile, inspect, pause=lambda _: None
        )

    def test_registers_a_still_folded_pose_and_persists_it(self):
        pose = [1.0, -66.0, 51.0, 44.0, 0.5, 50.0]
        result = self.register(still(pose))
        self.assertTrue(result["registered"] and result["persisted"])
        self.assertEqual(self.safe["omx2"], pose)
        saved = json.loads((self.dir / "safe_poses.json").read_text())
        self.assertEqual(saved["omx2"]["pose"], pose)

    def test_a_moving_arm_is_rejected(self):
        readings = iter([FOLDED, [1.0, -65.0, 50.0, 45.0, 0.0, 50.0]])
        with self.assertRaises(Rejected):
            self.register(lambda: {"measured": {"omx2": next(readings)}})
        self.assertEqual(self.safe["omx2"], FOLDED)

    def test_an_extended_arm_is_rejected(self):
        with self.assertRaises(Rejected):
            self.register(still([0.0, -20.0, 10.0, 0.0, 0.0, 50.0]))

    def test_a_pose_outside_the_joint_bounds_is_rejected(self):
        self.profile["host_profile"]["joint_bounds"][1] = [-60.0, 100.0]
        with self.assertRaises(Rejected):
            self.register(still(FOLDED))

    def test_a_large_jump_from_the_previous_registration_is_rejected(self):
        self.safe["omx2"] = [0.0, -90.0, 90.0, 90.0, 0.0, 50.0]
        with self.assertRaises(Rejected):
            self.register(still(FOLDED))

    def test_a_failed_save_leaves_the_safe_pose_unchanged(self):
        self.profile["safe_pose_file"] = str(self.dir / "missing-dir" / "safe.json")
        with self.assertRaises(OSError):
            self.register(still([1.0, -66.0, 51.0, 44.0, 0.5, 50.0]))
        self.assertEqual(self.safe["omx2"], FOLDED)


class SavedPoseTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.dir = Path(temporary.name)
        self.profile = make_profile(self.dir, "cal-1")
        self.path = self.profile["safe_pose_file"]
        register_safe_position(self.profile, still(FOLDED), pause=lambda _: None)

    def test_a_pose_saved_under_one_calibration_is_ignored_under_another(self):
        same = safe_pose_basis(self.profile["host_profile"])
        other = safe_pose_basis(
            {**self.profile["host_profile"], "calibration_version": "cal-2"}
        )
        self.assertEqual(load_safe_poses(self.path, same), {"omx2": FOLDED})
        self.assertEqual(load_safe_poses(self.path, other), {})

    def test_a_pose_saved_under_other_units_is_ignored(self):
        degrees = safe_pose_basis({**self.profile["host_profile"], "use_degrees": True})
        self.assertEqual(load_safe_poses(self.path, degrees), {})

    def test_loading_without_a_basis_keeps_the_old_behaviour(self):
        self.assertEqual(load_safe_poses(self.path), {"omx2": FOLDED})

    def test_a_missing_or_damaged_file_and_bad_entries_are_ignored(self):
        self.assertEqual(load_safe_poses(self.dir / "absent.json"), {})
        damaged = self.dir / "damaged.json"
        damaged.write_text("{not json")
        self.assertEqual(load_safe_poses(damaged), {})
        bad = self.dir / "bad.json"
        bad.write_text(
            json.dumps(
                {
                    "omx1": {"pose": [0, 0, 0]},
                    "omx2": {"pose": [0, 0, 0, 0, 0, float("nan")]},
                }
            )
        )
        self.assertEqual(load_safe_poses(bad), {})

    def test_apply_uses_only_known_arms_that_fit_the_bounds(self):
        host = deepcopy(self.profile["host_profile"])
        host["joint_bounds"][1] = [-60.0, 100.0]
        self.assertEqual(
            apply_saved_safe_poses(host, {"omx2": FOLDED, "omx9": FOLDED}), []
        )
        host["joint_bounds"][1] = [-100.0, 100.0]
        moved = [0.0, -70.0, 50.0, 45.0, 0.0, 50.0]
        self.assertEqual(
            apply_saved_safe_poses(host, {"omx2": moved, "omx9": moved}), ["omx2"]
        )
        self.assertEqual(host["safe_positions"]["omx2"], moved)

    def test_propagate_skips_profiles_where_the_pose_does_not_fit(self):
        fits = {"host_profile": deepcopy(self.profile["host_profile"])}
        tight = {"host_profile": deepcopy(self.profile["host_profile"])}
        tight["host_profile"]["joint_bounds"][1] = [-60.0, 100.0]
        result = propagate_safe_pose({"fits": fits, "tight": tight}, "omx2", FOLDED)
        self.assertEqual(result, {"applied": ["fits"], "skipped": ["tight"]})


if __name__ == "__main__":
    unittest.main()
