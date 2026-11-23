"""The whole-evaluation deadline is checked right before the motor write, not only at the loop start."""

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_test_support import MockClock, synthetic_profile
from test_lerobot_runtime import FakeDevices, FakeWriter

from roport_omx.lerobot_runtime import evaluate_local_act

INFERENCE_S = 0.06


class EvaluationDeadlineTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.clock = MockClock()
        self.profile = {
            "device_mode": "fake",
            "authorization": {"supervisor": True},
            "host_profile": synthetic_profile(),
            "operation": "INBOUND",
            "arm_id": "omx1",
            "fps": 100,
            "max_frames": 5,  # whole evaluation: 5 / 100 = 50 ms
            "shared_lock_path": str(self.root / "devices.lock"),
            "writer_factory": lambda request, profile: FakeWriter(),
            "wall_clock": self.clock.wall,
            "act_checkpoint": str(self.root / "checkpoint"),
            "shelf_level": 1,
            "camera_mapping": {},
            "supervise_local_act": lambda _: "run",
        }
        self.devices = FakeDevices(self.clock, self.profile)

    def evaluate(self):
        request = {
            "schema_version": "omx.workflow/1",
            "request_id": "deadline-job",
            "profile_id": "fake",
            "operation": "evaluate",
            "parameters": {},
        }
        virtual_time = SimpleNamespace(
            monotonic=self.clock.monotonic,
            time=self.clock.wall,
            time_ns=lambda: int(self.clock.wall() * 1e9),
        )

        def slow_inference(*_):
            self.clock.advance(INFERENCE_S)  # inside the per-inference limit (1 s)
            return SimpleNamespace(tolist=lambda: [1, 0, 0, 0, 0, 50])

        with (
            patch("roport_omx.lerobot_runtime.time", virtual_time),
            patch("roport_omx.panel_act.PanelAct") as factory,
        ):
            factory.return_value.select_action.side_effect = slow_inference
            return evaluate_local_act(request, self.profile, self.devices)

    def test_an_inference_that_ends_after_the_evaluation_deadline_never_writes(self):
        result = self.evaluate()
        self.assertEqual(result["state"], "FAILED")
        self.assertIn("duration", result["error"])
        self.assertEqual(self.devices.actions, [])

    def test_within_the_deadline_the_action_is_still_written(self):
        self.profile["max_frames"] = 50  # 500 ms
        self.evaluate()
        self.assertGreaterEqual(len(self.devices.actions), 1)


if __name__ == "__main__":
    unittest.main()
