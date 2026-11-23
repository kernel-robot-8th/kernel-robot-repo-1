"""Warmup must not leave synthetic actions in the live policy queue."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.panel_act import PanelAct


class WarmupTests(unittest.TestCase):
    def policy(self):
        policy = PanelAct.__new__(PanelAct)
        policy.torch = Mock()
        policy.model = Mock()
        policy.model.config.input_features = {
            "observation.state": SimpleNamespace(shape=(6,)),
            "observation.images.shelf": SimpleNamespace(shape=(3, 480, 640)),
            "observation.environment_state": SimpleNamespace(shape=(2,)),
        }
        policy.reset = Mock()
        policy.select_action = Mock()
        return policy

    def test_warms_both_floor_inputs_and_resets_queue(self):
        policy = self.policy()
        policy.warmup()
        self.assertEqual([c.args[1] for c in policy.select_action.call_args_list], [1, 2])
        self.assertEqual(policy.reset.call_count, 3)
        self.assertEqual(policy.torch.cuda.synchronize.call_count, 2)
        self.assertEqual(policy.torch.zeros.call_count, 2)

    def test_failed_warmup_clears_queue_and_propagates_failure(self):
        policy = self.policy()
        policy.select_action.side_effect = RuntimeError("GPU unavailable")
        with self.assertRaisesRegex(RuntimeError, "GPU unavailable"):
            policy.warmup()
        self.assertEqual(policy.reset.call_count, 2)


if __name__ == "__main__":
    unittest.main()
