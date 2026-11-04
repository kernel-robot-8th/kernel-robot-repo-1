from copy import deepcopy
from datetime import UTC, datetime

import pytest

from roport_control_server.adapters.mqtt.messages import decode, encode, topic
from roport_control_server.executor.dedicated import (
    PermitRelay,
    make_dedicated_command,
)

NOW = datetime(2026, 1, 1, tzinfo=UTC).timestamp()


def command(operation="INBOUND"):
    active, inactive, pinky = (
        ("omx1", "omx2", "pinky1")
        if operation == "INBOUND"
        else ("omx2", "omx1", "pinky3")
    )
    return make_dedicated_command(
        {
            "operation": operation,
            "manipulation_mode": "DEDICATED",
            "resource_token": "mock-resource",
            "permit_id": "mock-permit",
            "assigned_arm_id": active,
            "inactive_arm_id": inactive,
            "order_id": "mock-order",
            "package_id": "mock-package",
            "expected_package_ids": ["mock-package"],
            "slot_id": "mock-slot",
            "pinky_id": pinky,
            "policy_bundle_version": "mock-bundle",
            "feature_schema_version": "mock-features",
            "calibration_version": "mock-calibration",
            "workcell_profile_version": "mock-profile",
        },
        site_id="test",
        source="control-test",
        task_id="mock-task",
        step_id="mock-step",
        command_id="mock-command",
        execution_id="mock-execution",
        now=NOW,
        expires_at=NOW + 30,
        roles={"INBOUND": "omx1", "OUTBOUND": "omx2"},
    )


def relay(cmd=None):
    return PermitRelay(
        command() if cmd is None else cmd,
        control_boot_id="control-test-boot",
        host_boot_id="host-test-boot",
        pinky_boot_id="pinky-test-boot",
        docking_execution_id="mock-docking",
        max_source_age=2,
        permit_ttl=1,
        clock_skew=0.1,
    )


def evidence(seq=1, observed=NOW):
    return {
        "pinky_id": "pinky1",
        "pinky_boot_id": "pinky-test-boot",
        "docking_execution_id": "mock-docking",
        "evidence_seq": seq,
        "docked": True,
        "stopped": True,
        "source_observed_at": datetime.fromtimestamp(observed, UTC).isoformat(),
    }


@pytest.mark.parametrize("operation,arm", [("INBOUND", "omx1"), ("OUTBOUND", "omx2")])
def test_one_arm_assignment_uses_existing_envelope(operation, arm):
    cmd = decode(encode(command(operation)))
    assert cmd["payload"]["robot_ids"] == [arm]
    assert cmd["payload"]["manipulation_mode"] == "DEDICATED"


@pytest.mark.parametrize(
    "field,value,error",
    [
        (
            "expected_package_ids",
            ["mock-package", "other"],
            "UNSUPPORTED_PACKAGE_COUNT",
        ),
        ("manipulation_mode", None, "INVALID_COMMAND"),
        ("assigned_arm_id", "omx2", "invalid_arm_assignment"),
        ("pinky_id", "pinky3", "invalid_pinky_assignment"),
    ],
)
def test_invalid_assignment_rejected(field, value, error):
    # Reuse the public producer input; do not manually bypass its validation.
    payload = command()["payload"]
    payload[field] = value
    with pytest.raises(ValueError, match=error):
        make_dedicated_command(
            payload,
            site_id="test",
            source="control-test",
            task_id="task",
            step_id="step",
            command_id="cmd",
            execution_id="exec",
            now=NOW,
            expires_at=NOW + 30,
            roles={"INBOUND": "omx1", "OUTBOUND": "omx2"},
        )


def test_permit_is_targeted_expiring_nonrenewable_evidence():
    producer = relay()
    first = producer.relay(evidence(), now=NOW)
    assert decode(encode(first)) == first
    assert first["payload"]["allowed"] is True
    with pytest.raises(ValueError, match="old_evidence"):
        producer.relay(evidence(), now=NOW + 0.5)
    with pytest.raises(ValueError, match="stale_or_future_evidence"):
        producer.relay(evidence(2, NOW), now=NOW + 0.5)
    revoked = producer.relay(
        {**evidence(2, NOW + 0.1), "stopped": False}, now=NOW + 0.1
    )
    assert revoked["payload"]["allowed"] is False
    assert first["payload"]["expires_at"] < revoked["payload"]["expires_at"]


@pytest.mark.parametrize(
    "field,value,error",
    [
        ("pinky_id", "pinky2", "binding"),
        ("pinky_boot_id", "rebooted", "binding"),
        ("docking_execution_id", "old-dock", "binding"),
        ("docked", 1, "flags"),
        ("stopped", "true", "flags"),
        ("evidence_seq", True, "old_evidence"),
    ],
)
def test_bad_source_evidence_rejected(field, value, error):
    item = evidence()
    item[field] = value
    with pytest.raises(ValueError, match=error):
        relay().relay(item, now=NOW)


@pytest.mark.parametrize("observed", [NOW - 2, NOW + 1])
def test_stale_future_evidence_rejected(observed):
    with pytest.raises(ValueError, match="stale_or_future"):
        relay().relay(evidence(observed=observed), now=NOW)


def test_permit_topic_is_only_for_workcell_and_requires_expiry():
    assert topic("test", "workcell", "shelf-01", "permit").endswith("/permit")
    with pytest.raises(ValueError, match="shared workcell"):
        topic("test", "robot", "pinky1", "permit")
    bad = deepcopy(relay().relay(evidence(), now=NOW))
    del bad["expires_at"]
    with pytest.raises(ValueError):
        encode(bad)
