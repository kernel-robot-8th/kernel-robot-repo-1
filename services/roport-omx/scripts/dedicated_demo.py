"""Explicit synthetic source/mock demo. No serial, motors, trained ACT or simulator."""

import argparse
import sys
import tempfile
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.dedicated import (
    PHASES,
    PROFILE_VERSION,
    VERDICTS,
    DedicatedHost,
    Profile,
)


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


def mock_envelopes(clock, operation="INBOUND"):
    """Synthetic command/permit pair using the shared MQTT envelope contract."""
    p = synthetic_profile()
    arm = p["roles"][operation]
    command = {
        "schema_version": "1.0",
        "message_id": "mock-command-message",
        "type": "shelf.execute.requested",
        "site_id": "mock-site",
        "source": "control",
        "target": "shelf-01",
        "task_id": "mock-task",
        "step_id": "mock-step",
        "command_id": f"mock-command-{operation}",
        "execution_id": f"mock-execution-{operation}",
        "sent_at": clock.stamp(),
        "expires_at": clock.stamp(60),
        "payload": {
            "profile_version": PROFILE_VERSION,
            "workcell_id": "shelf-01",
            "manipulation_mode": "DEDICATED",
            "operation": operation,
            "resource_token": "mock-resource",
            "permit_id": "mock-permit",
            "assigned_arm_id": arm,
            "inactive_arm_id": "omx2" if arm == "omx1" else "omx1",
            "robot_ids": [arm],
            "order_id": "mock-order",
            "package_id": "mock-package",
            "expected_package_ids": ["mock-package"],
            "slot_id": "mock-slot",
            "pinky_id": "pinky1" if operation == "INBOUND" else "pinky3",
            **{
                k: p[k]
                for k in (
                    "policy_bundle_version",
                    "feature_schema_version",
                    "calibration_version",
                    "workcell_profile_version",
                )
            },
        },
    }
    permit = deepcopy(command)
    permit.update(
        type="workcell.permit",
        message_id="mock-permit-message",
        expires_at=clock.stamp(5),
    )
    permit["payload"] = {
        "profile_version": PROFILE_VERSION,
        "workcell_id": "shelf-01",
        "permit_id": "mock-permit",
        "resource_token": "mock-resource",
        "execution_id": command["execution_id"],
        "control_boot_id": "mock-control-boot",
        "host_boot_id": "mock-host-boot",
        "pinky_id": command["payload"]["pinky_id"],
        "pinky_boot_id": "mock-pinky-boot",
        "docking_execution_id": "mock-docking",
        "evidence_seq": 1,
        "docked": True,
        "stopped": True,
        "allowed": True,
        "source_observed_at": clock.stamp(),
        "expires_at": clock.stamp(5),
    }
    return command, permit


def mock_observe(host, clock, seq):
    for arm, positions in synthetic_profile()["safe_positions"].items():
        host.observe(arm, positions, observation_seq=seq, captured_at=clock.stamp())


def run_mock(operation="INBOUND"):
    clock = MockClock()
    with tempfile.TemporaryDirectory() as directory:
        host = DedicatedHost(
            Profile.from_dict(synthetic_profile()),
            Path(directory) / "journal.sqlite",
            host_boot_id="mock-host-boot",
            wall_clock=clock.wall,
            monotonic_clock=clock.monotonic,
        )
        command, permit = mock_envelopes(clock, operation)
        mock_observe(host, clock, 1)
        host.update_permit(permit)
        host.accept_command(command)
        seq = 1
        for index, phase in enumerate(PHASES):
            reset = host.reset_request()
            host.acknowledge_reset({**reset, "type": "policy.reset.ack"})
            request = host.infer_request()
            reply = {
                **request,
                "type": "action.candidate",
                "action_seq": 0,
                "actions": [[0, 0, 0, 0, 0, 50]],
            }
            host.validate_candidate(reply)
            assert host.next_action() == (0, 0, 0, 0, 0, 50)
            host.phase_boundary()
            reset = host.reset_request()
            host.acknowledge_reset({**reset, "type": "policy.reset.ack"})
            clock.advance()
            seq += 1
            mock_observe(host, clock, seq)
            evidence = {
                "execution_id": command["execution_id"],
                "phase": phase,
                "phase_revision": host.phase_revision,
                "verdict_kind": VERDICTS[index],
                "confirmer": "mock-supervisor",
                "positive": True,
                "observation_seq": seq,
                "captured_at": clock.stamp(),
            }
            if index < 3:
                evidence["next_policy_id"] = host.profile["policies"][operation][
                    PHASES[index + 1]
                ]["policy_id"]
            host.confirm(evidence)
        result = host.status()
        host.close()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mock",
        action="store_true",
        required=True,
        help="explicitly run synthetic fixture only",
    )
    args = parser.parse_args()
    if args.mock:
        import json

        print(
            json.dumps(
                {
                    "source_mock_only": True,
                    "operations": [run_mock(op) for op in ("INBOUND", "OUTBOUND")],
                },
                indent=2,
            )
        )


if __name__ == "__main__":
    main()
