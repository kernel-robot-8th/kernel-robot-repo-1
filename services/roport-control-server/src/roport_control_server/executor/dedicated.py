"""Dedicated shelf command/permit production; no DB, robot or departure side effects."""

from copy import deepcopy
from datetime import UTC, datetime
from math import isfinite
from uuid import uuid4

from roport_control_server.adapters.mqtt.messages import encode, timestamp

PROFILE = "omx.dedicated/1"
_ASSIGNMENT = (
    "operation",
    "resource_token",
    "permit_id",
    "assigned_arm_id",
    "inactive_arm_id",
    "order_id",
    "package_id",
    "slot_id",
    "pinky_id",
    "policy_bundle_version",
    "feature_schema_version",
    "calibration_version",
    "workcell_profile_version",
)


def _text(value):
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError("invalid_identifier")
    return value


def _seconds(value):
    if type(value) not in {int, float} or not isfinite(value) or value <= 0:
        raise ValueError("positive timing limit required")
    return value


def _iso(epoch):
    if type(epoch) not in {int, float} or not isfinite(epoch):
        raise ValueError("finite UTC epoch required")
    return datetime.fromtimestamp(epoch, UTC).isoformat()


def make_dedicated_command(
    assignment: dict,
    *,
    site_id: str,
    source: str,
    task_id: str,
    step_id: str,
    command_id: str,
    execution_id: str,
    now: float,
    expires_at: float,
    roles: dict[str, str],
) -> dict:
    """Build one assigned package command; roles come from the approved profile."""
    if not isinstance(assignment, dict) or set(roles) != {"INBOUND", "OUTBOUND"}:
        raise ValueError("invalid_assignment")
    if set(roles.values()) != {"omx1", "omx2"}:
        raise ValueError("invalid_role_mapping")
    payload = {key: _text(assignment.get(key)) for key in _ASSIGNMENT}
    operation = payload["operation"]
    if operation not in roles:
        raise ValueError("invalid_operation")
    active = roles[operation]
    inactive = roles["OUTBOUND" if operation == "INBOUND" else "INBOUND"]
    if (payload["assigned_arm_id"], payload["inactive_arm_id"]) != (active, inactive):
        raise ValueError("invalid_arm_assignment")
    if payload["pinky_id"] not in (
        {"pinky1", "pinky2"} if operation == "INBOUND" else {"pinky3"}
    ):
        raise ValueError("invalid_pinky_assignment")
    expected = assignment.get("expected_package_ids")
    if expected != [payload["package_id"]]:
        raise ValueError("UNSUPPORTED_PACKAGE_COUNT")
    if assignment.get("manipulation_mode") != "DEDICATED":
        raise ValueError("INVALID_COMMAND")
    payload.update(
        profile_version=PROFILE,
        manipulation_mode="DEDICATED",
        workcell_id="shelf-01",
        robot_ids=[active],
        expected_package_ids=expected.copy(),
    )
    command = {
        "schema_version": "1.0",
        "message_id": str(uuid4()),
        "type": "shelf.execute.requested",
        "site_id": site_id,
        "source": source,
        "target": "shelf-01",
        "task_id": task_id,
        "step_id": step_id,
        "command_id": command_id,
        "execution_id": execution_id,
        "sent_at": _iso(now),
        "expires_at": _iso(expires_at),
        "payload": payload,
    }
    encode(command)
    return command


class PermitRelay:
    """Relay only new Pinky source evidence, never a repeated control heartbeat.

    shortcut: the binding and source cursor are in memory; after control restart,
    reconcile resources and issue a new permit/execution instead of restoring a lease.
    """

    def __init__(
        self,
        command: dict,
        *,
        control_boot_id: str,
        host_boot_id: str,
        pinky_boot_id: str,
        docking_execution_id: str,
        max_source_age: float,
        permit_ttl: float,
        clock_skew: float,
    ):
        encode(command)
        if command["payload"].get("profile_version") != PROFILE:
            raise ValueError("unsupported_profile")
        self.command = deepcopy(command)
        self.boots = {
            key: _text(value)
            for key, value in {
                "control_boot_id": control_boot_id,
                "host_boot_id": host_boot_id,
                "pinky_boot_id": pinky_boot_id,
                "docking_execution_id": docking_execution_id,
            }.items()
        }
        self.max_source_age = _seconds(max_source_age)
        self.permit_ttl = _seconds(permit_ttl)
        self.clock_skew = _seconds(clock_skew)
        self.last_seq = -1
        self.last_observed = float("-inf")

    def relay(self, evidence: dict, *, now: float) -> dict:
        command = self.command
        assignment = command["payload"]
        if timestamp(command["expires_at"]).timestamp() <= now:
            raise ValueError("expired_command")
        if any(
            evidence.get(key) != value
            for key, value in {
                "pinky_id": assignment["pinky_id"],
                "pinky_boot_id": self.boots["pinky_boot_id"],
                "docking_execution_id": self.boots["docking_execution_id"],
            }.items()
        ):
            raise ValueError("evidence_binding_mismatch")
        seq = evidence.get("evidence_seq")
        if type(seq) is not int or seq < 0 or seq <= self.last_seq:
            raise ValueError("old_evidence")
        observed = timestamp(evidence.get("source_observed_at")).timestamp()
        if (
            observed <= self.last_observed
            or observed > now + self.clock_skew
            or now - observed >= self.max_source_age
        ):
            raise ValueError("stale_or_future_evidence")
        if any(type(evidence.get(k)) is not bool for k in ("docked", "stopped")):
            raise ValueError("invalid_evidence_flags")
        expires = min(
            now + self.permit_ttl,
            observed + self.max_source_age,
            timestamp(command["expires_at"]).timestamp(),
        )
        payload = {
            "profile_version": PROFILE,
            **{
                key: assignment[key]
                for key in ("workcell_id", "permit_id", "resource_token", "pinky_id")
            },
            "execution_id": command["execution_id"],
            **self.boots,
            "evidence_seq": seq,
            "docked": evidence["docked"],
            "stopped": evidence["stopped"],
            "allowed": evidence["docked"] and evidence["stopped"],
            "source_observed_at": evidence["source_observed_at"],
            "expires_at": _iso(expires),
        }
        result = {
            **{
                key: command[key]
                for key in (
                    "schema_version",
                    "site_id",
                    "source",
                    "target",
                    "task_id",
                    "step_id",
                    "command_id",
                    "execution_id",
                )
            },
            "message_id": str(uuid4()),
            "type": "workcell.permit",
            "sent_at": _iso(now),
            "expires_at": _iso(expires),
            "payload": payload,
        }
        encode(result)
        self.last_seq, self.last_observed = seq, observed
        return result
