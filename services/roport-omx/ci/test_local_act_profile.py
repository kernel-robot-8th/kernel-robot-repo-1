"""Validate local ACT registrations without initializing devices or loading a model."""

import sys
import unittest
from copy import deepcopy
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.act_support import Profile, Rejected
from roport_omx.lerobot_runtime import OMX_FEATURES, _validated


class LocalActProfileTests(unittest.TestCase):
    def setUp(self):
        self.host = {
            "profile_version": "omx.local-act/1",
            "calibration_version": "test-calibration",
            "feature_names": list(OMX_FEATURES),
            "feature_units": ["range_m100_100"] * 5 + ["range_0_100"],
            "use_degrees": False,
            "joint_directions": [1] * 6,
            "roles": {"INBOUND": "omx2", "OUTBOUND": "omx1"},
            "joint_bounds": [[-10, 10]] * 5 + [[0, 100]],
            "safe_positions": {"omx1": [0] * 5 + [50], "omx2": [0] * 5 + [50]},
            "safe_tolerance": [1] * 6,
            "max_step": [1] * 6,
            "timing": {
                "clock_skew": 0.1,
                "observation_max_age": 0.4,
                "inference_timeout": 0.2,
            },
        }
        self.profile = {
            "host_profile": self.host,
            "device_mode": "hardware",
            "authorization": {"supervisor": True, "hardware": True},
            "act_checkpoint": "/checkpoint",
            "operation": "OUTBOUND",
            "arm_id": "omx1",
            "fps": 30,
            "max_frames": 60,
        }

    def validate(self, operation="evaluate", **changes):
        request = {
            "schema_version": "omx.workflow/1",
            "operation": operation,
            "request_id": "preflight",
            "profile_id": "local-act-test",
            "parameters": {},
        }
        return _validated(request, dict(self.profile, **changes), operation)

    def test_full_episode_profile_has_no_dedicated_phase_registration(self):
        self.assertEqual(self.validate(), self.host)
        with self.assertRaises(Rejected):
            Profile(self.host)
        with self.assertRaisesRegex(Rejected, "full-episode checkpoint evaluation"):
            self.validate("record")
        with self.assertRaisesRegex(Rejected, "full-episode checkpoint evaluation"):
            self.validate(act_checkpoint=None)
        with self.assertRaisesRegex(Rejected, "operation/assigned arm mismatch"):
            self.validate(arm_id="omx2")

    def test_full_episode_requires_calibration_normalization_and_motion_limits(self):
        invalid_values = {
            "calibration_version": "",
            "feature_names": list(reversed(OMX_FEATURES)),
            "use_degrees": True,
            "joint_directions": [True] * 6,
            "joint_bounds": [[-10, 10]] * 5 + [[-1, 100]],
            "max_step": [0] * 6,
            "safe_positions": {"omx1": [0] * 6},
            "safe_tolerance": [-1] * 6,
            "timing": {"observation_max_age": 0.2},
        }
        for field, value in invalid_values.items():
            with self.subTest(field=field), self.assertRaises(Rejected):
                host = deepcopy(self.host)
                host[field] = value
                self.validate(host_profile=host)
