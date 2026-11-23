"""Test helpers for the local ACT runtime tests: a synthetic evaluation profile and a mock clock."""

from copy import deepcopy
from datetime import UTC, datetime

from roport_omx.act_support import PHASES, PROFILE_VERSION


def synthetic_profile():
    """Invented MOCK values only; this fixture cannot enable physical activation."""
    limits = [[-100, 100]] * 5 + [[0, 100]]
    return {
        "synthetic_fixture": True,
        "profile_version": PROFILE_VERSION,
        "workcell_id": "shelf-01",
        "site_id": "mock-site",
        "control_id": "control",
        "roles": {"INBOUND": "omx1", "OUTBOUND": "omx2"},
        "slot_ids": ["mock-slot"],
        "package_ids": ["mock-package"],
        "policy_bundle_version": "mock-bundle",
        "feature_schema_version": "mock-features",
        "calibration_version": "mock-calibration",
        "workcell_profile_version": "mock-workcell",
        "feature_names": [
            "joint1.pos",
            "joint2.pos",
            "joint3.pos",
            "joint4.pos",
            "joint5.pos",
            "gripper.pos",
        ],
        "feature_units": ["range_m100_100"] * 5 + ["range_0_100"],
        "joint_directions": [1] * 6,
        "use_degrees": False,
        "joint_bounds": deepcopy(limits),
        "max_step": [5] * 6,
        "safe_positions": {"omx1": [0, 0, 0, 0, 0, 50], "omx2": [0, 0, 0, 0, 0, 50]},
        "safe_tolerance": [1] * 6,
        "timing": {
            "clock_skew": 0.1,
            "permit_max_age": 5,
            "permit_lease": 2,
            "observation_max_age": 2,
            "wait_timeout": 3,
            "inference_timeout": 1,
            "phase_timeout": 10,
        },
        "policies": {
            operation: {
                phase: {
                    "policy_id": f"mock-{operation}-{phase}",
                    "model_version": "scripted-mock-v1",
                    "entry_bounds": deepcopy(limits),
                    "exit_bounds": deepcopy(limits),
                }
                for phase in PHASES
            }
            for operation in ("INBOUND", "OUTBOUND")
        },
    }


class MockClock:
    def __init__(self):
        self.now = 1_800_000_000.0
        self.elapsed = 0.0

    def wall(self):
        return self.now

    def monotonic(self):
        return self.elapsed

    def advance(self, seconds=0.01):
        self.now += seconds
        self.elapsed += seconds

    def stamp(self, offset=0):
        return datetime.fromtimestamp(self.now + offset, UTC).isoformat()
