"""Rejection diagnostics must retain the paired path and never advance it."""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roport_omx.act_support import Rejected
from roport_omx.demonstration_guard import DemonstrationGuard


class GuardDiagnosticsTests(unittest.TestCase):
    def test_rejection_reports_target_error_at_a_state_compatible_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            path.write_text(
                json.dumps(
                    [
                        {"state": [0] * 6, "action": [0] * 6},
                        {"state": [2] * 6, "action": [4] * 6},
                    ]
                )
            )
            guard = DemonstrationGuard(
                {
                    "path": str(path),
                    "state_tolerance": [1] * 6,
                    "action_tolerance": [1] * 6,
                    "lookahead_frames": 2,
                }
            )
            with self.assertRaises(Rejected) as caught:
                guard.check([0] * 6, [4] * 6)
            prefix = "ACT target leaves the reviewed demonstration path: "
            self.assertTrue(str(caught.exception).startswith(prefix))
            detail = json.loads(str(caught.exception)[len(prefix) :])
            self.assertEqual(detail["reference_frame"], 0)
            self.assertEqual(detail["state_matching_frames"], 1)
            self.assertEqual(detail["state_delta"], [0] * 6)
            self.assertEqual(detail["action_delta"], [4] * 6)
            self.assertEqual(detail["action_tolerance"], [1] * 6)
            self.assertEqual(detail["action"], [4] * 6)
            self.assertEqual(guard.possible_frames, {0})
            guard.check([0] * 6, [0] * 6)

    def test_nonfinite_input_is_rejected_without_advancing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            path.write_text(json.dumps([{"state": [0] * 6, "action": [0] * 6}]))
            guard = DemonstrationGuard(
                {
                    "path": str(path),
                    "state_tolerance": [1] * 6,
                    "action_tolerance": [1] * 6,
                    "lookahead_frames": 1,
                }
            )
            with self.assertRaises(Rejected):
                guard.check([0] * 6, [float("nan")] * 6)
            self.assertEqual(guard.possible_frames, {0})

    def test_repeated_pose_does_not_skip_an_intervening_excursion(self):
        rest = {"state": [0] * 6, "action": [0] * 6}
        excursion = {"state": [2] * 6, "action": [2] * 6}
        next_motion = {"state": [0] * 6, "action": [4] * 6}
        reference = [rest, excursion, rest, next_motion]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            path.write_text(json.dumps(reference))
            guard = DemonstrationGuard(
                {
                    "path": str(path),
                    "state_tolerance": [1] * 6,
                    "action_tolerance": [1] * 6,
                    "lookahead_frames": 3,
                }
            )
            for _ in range(2):
                guard.check(**rest)
            with self.assertRaises(Rejected):
                guard.check(**next_motion)
            for frame in reference[1:]:
                guard.check(**frame)

    def test_lookahead_keeps_adjacent_matches_and_unique_forward_progress(self):
        rest = {"state": [0] * 6, "action": [0] * 6}
        moving = {"state": [2] * 6, "action": [2] * 6}
        reached = {"state": [4] * 6, "action": [4] * 6}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            path.write_text(json.dumps([rest, rest, moving, reached]))
            guard = DemonstrationGuard(
                {
                    "path": str(path),
                    "state_tolerance": [1] * 6,
                    "action_tolerance": [1] * 6,
                    "lookahead_frames": 3,
                }
            )
            guard.check(**rest)
            self.assertEqual(guard.possible_frames, {0, 1})
            guard.check(**reached, advance=False)
            self.assertEqual(guard.possible_frames, {0, 1})
            guard.check(**reached)
            self.assertEqual(guard.possible_frames, {3})
            with self.assertRaises(Rejected):
                guard.check(**moving)


if __name__ == "__main__":
    unittest.main()
