"""One-package dedicated shelf cycle; callers own durable resource transactions."""

from copy import deepcopy
from datetime import datetime
from math import isfinite

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
_BINDING = ("site_id", "task_id", "step_id", "command_id", "execution_id")


def check_identifier(value):
    if not isinstance(value, str) or not value or len(value) > 128:
        raise ValueError("invalid_identifier")
    return value


def dedicated_payload(assignment, roles):
    """Validated command payload; shared by command creation and cycle begin."""
    if (
        not isinstance(assignment, dict)
        or not isinstance(roles, dict)
        or set(roles) != {"INBOUND", "OUTBOUND"}
    ):
        raise ValueError("invalid_assignment")
    if set(roles.values()) != {"omx1", "omx2"}:
        raise ValueError("invalid_role_mapping")
    payload = {key: check_identifier(assignment.get(key)) for key in _ASSIGNMENT}
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
    return payload


def _check_command(command):
    """Re-validate a built command; arm roles are re-derived from its own payload."""
    if not isinstance(command, dict) or not isinstance(command.get("payload"), dict):
        raise ValueError("invalid_command")  # noqa: TRY004 - one rejection type
    for key in (*_BINDING, "source"):
        check_identifier(command.get(key))
    payload = command["payload"]
    operation = payload.get("operation")
    if operation not in {"INBOUND", "OUTBOUND"}:
        raise ValueError("dedicated_operation_required")
    other = "OUTBOUND" if operation == "INBOUND" else "INBOUND"
    roles = {
        operation: payload.get("assigned_arm_id"),
        other: payload.get("inactive_arm_id"),
    }
    if dedicated_payload(payload, roles) != payload:
        raise ValueError("invalid_command")


def _stamp(value):
    try:
        parsed = datetime.fromisoformat(value)
        if parsed.tzinfo is None:
            raise ValueError("offset required")
        return parsed.timestamp()
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid_observation_time") from exc


class DedicatedCycle:
    """Explicit source/mock ordering, never a persisted production scheduler.

    shortcut: in-memory cycle only; operational DB/outbox and restart
    reconciliation must be integrated before enabling physical task dispatch.
    """

    def __init__(self, *, max_evidence_age: float, clock_skew: float):
        if any(
            type(v) not in {int, float} or not isfinite(v) or v <= 0
            for v in (max_evidence_age, clock_skew)
        ):
            raise ValueError("explicit positive evidence age/skew required")
        self.max_evidence_age, self.clock_skew = max_evidence_age, clock_skew
        self.state = "EMPTY"
        self.command = None
        self.stored = None
        self.host_boot_id = None
        self.docking_execution_id = None
        self.pinky_boot_id = None
        self.evidence_seq = -1
        self.evidence_time = float("-inf")
        self.result = None  # recorded completion fact, never refreshed
        self.arm_safety = None  # latest both-arm safety used for departure

    def _fresh(self, observed_at, now):
        if type(now) not in {int, float} or not isfinite(now):
            raise ValueError("finite_now_required")
        age = now - _stamp(observed_at)
        if age < -self.clock_skew or age >= self.max_evidence_age:
            raise ValueError("stale_or_future_evidence")

    def begin(
        self,
        command,
        *,
        p7_cart_id,
        cart_empty,
        host_boot_id,
        pinky_boot_id,
        docking_execution_id,
        evidence_seq,
        observed_at,
        now,
    ):
        self._fresh(observed_at, now)
        _check_command(command)
        if now >= _stamp(command.get("expires_at")):
            raise ValueError("expired_command")
        payload = command["payload"]
        operation = payload["operation"]
        if p7_cart_id != payload.get("pinky_id") or type(cart_empty) is not bool:
            raise ValueError("p7_assignment_mismatch")
        if not all(
            isinstance(v, str) and v
            for v in (host_boot_id, pinky_boot_id, docking_execution_id)
        ):
            raise ValueError("boot_and_docking_binding_required")
        if type(evidence_seq) is not int or evidence_seq < 0:
            raise ValueError("source_sequence_required")
        if operation == "INBOUND":
            if (
                self.state != "EMPTY"
                or p7_cart_id not in {"pinky1", "pinky2"}
                or cart_empty
            ):
                raise ValueError("inbound_preconditions_failed")
        else:
            if self.state != "STORED" or p7_cart_id != "pinky3" or not cart_empty:
                raise ValueError("outbound_requires_p7_clear_and_empty_pinky3")
            if any(
                payload.get(k) != self.stored[k]
                for k in ("order_id", "package_id", "slot_id")
            ):
                raise ValueError("stored_package_mismatch")
        self.command = deepcopy(command)
        self.host_boot_id = host_boot_id
        self.pinky_boot_id = pinky_boot_id
        self.docking_execution_id = docking_execution_id
        self.evidence_seq = evidence_seq
        self.evidence_time = _stamp(observed_at)
        self.result = None
        self.arm_safety = None
        self.state = operation + "_ACTIVE"

    def _check_safe_snapshot(self, event, now, *, require_fresh=True):
        """Host SAFE_RETURNED snapshot of this execution and Host boot.

        require_fresh=False accepts a late completion fact (timestamps must still
        parse); departure stays blocked until a fresh snapshot is supplied.
        """
        stamp_check = (
            self._fresh if require_fresh else lambda value, _now: _stamp(value)
        )
        command = self.command
        if not isinstance(event, dict) or not isinstance(event.get("payload"), dict):
            raise ValueError("invalid_result_event")  # noqa: TRY004
        p = event["payload"]
        if (
            any(event.get(k) != command[k] for k in _BINDING)
            or event.get("source") != "shelf-01"
            or event.get("target") != command["source"]
        ):
            raise ValueError("result_binding_mismatch")
        if event.get("boot_id") != self.host_boot_id:
            raise ValueError("host_restarted_reconcile")
        stamp_check(event.get("observed_at"), now)
        if (
            event.get("type") != "shelf.execute.completed"
            or p.get("state") != "SAFE_RETURNED"
        ):
            raise ValueError("physical_result_not_complete")
        if any(
            p.get(key) != value or type(p.get(key)) is not type(value)
            for key, value in command["payload"].items()
        ):
            raise ValueError("result_assignment_mismatch")
        if (
            p.get("active_arm_safe") is not True
            or p.get("inactive_arm_safe") is not True
        ):
            raise ValueError("both_arm_safety_required")
        observations = p.get("arms")
        if not isinstance(observations, dict) or set(observations) != {
            command["payload"]["assigned_arm_id"],
            command["payload"]["inactive_arm_id"],
        }:
            raise ValueError("both_arm_observations_required")
        for observation in observations.values():
            if not isinstance(observation, dict) or observation.get("safe") is not True:
                raise ValueError("both_arm_safety_required")
            stamp_check(observation.get("captured_at"), now)
        expected_location = (
            "slot" if command["payload"]["operation"] == "INBOUND" else "pinky"
        )
        if p.get("location") != expected_location:
            raise ValueError("unexpected_package_location")
        return deepcopy(observations)

    def completed(self, event, *, now):
        if self.state not in {"INBOUND_ACTIVE", "OUTBOUND_ACTIVE"}:
            raise ValueError("no_active_assignment")
        # Command expiry is not checked: a physical result must never be dropped.
        # A late (stale) completion is still recorded; approve_departure needs a
        # fresh arm_safety from refresh_arm_safety() first.
        self.arm_safety = self._check_safe_snapshot(event, now, require_fresh=False)
        self.result = deepcopy(event)
        self.state = self.command["payload"]["operation"] + "_READY_TO_DEPART"

    def refresh_arm_safety(self, event, *, now):
        """Replace departure arm evidence; the recorded completion stays as is."""
        if self.state not in {"INBOUND_READY_TO_DEPART", "OUTBOUND_READY_TO_DEPART"}:
            raise ValueError("departure_not_ready")
        arms = self._check_safe_snapshot(event, now)
        if any(
            _stamp(arms[arm]["captured_at"]) < _stamp(old["captured_at"])
            for arm, old in self.arm_safety.items()
        ):
            raise ValueError("arm_observation_regressed")
        self.arm_safety = arms

    def _check_pinky_evidence(self, evidence, now):
        self._fresh(evidence.get("source_observed_at"), now)
        if (
            evidence.get("pinky_id") != self.command["payload"]["pinky_id"]
            or evidence.get("docking_execution_id") != self.docking_execution_id
            or evidence.get("pinky_boot_id") != self.pinky_boot_id
        ):
            raise ValueError("pinky_boot_or_docking_mismatch")
        seq = evidence.get("evidence_seq")
        if (
            type(seq) is not int
            or seq <= self.evidence_seq
            or _stamp(evidence["source_observed_at"]) < self.evidence_time
        ):
            raise ValueError("new_pinky_evidence_required")

    def _consume_evidence(self, evidence):
        self.evidence_seq = evidence["evidence_seq"]
        self.evidence_time = _stamp(evidence["source_observed_at"])

    def approve_departure(self, evidence, *, now):
        if self.state not in {"INBOUND_READY_TO_DEPART", "OUTBOUND_READY_TO_DEPART"}:
            raise ValueError("departure_not_ready")
        for observation in self.arm_safety.values():
            self._fresh(observation["captured_at"], now)
        self._check_pinky_evidence(evidence, now)
        if (
            evidence.get("pinky_id") != self.command["payload"]["pinky_id"]
            or evidence.get("docking_execution_id") != self.docking_execution_id
            or evidence.get("stopped") is not True
            or evidence.get("docked") is not True
        ):
            raise ValueError("fresh_assigned_pinky_required")
        self._consume_evidence(evidence)
        self.state = self.command["payload"]["operation"] + "_WAIT_P7_CLEAR"
        return True

    def p7_cleared(self, evidence, *, now):
        if self.state not in {"INBOUND_WAIT_P7_CLEAR", "OUTBOUND_WAIT_P7_CLEAR"}:
            raise ValueError("departure_not_approved")
        self._check_pinky_evidence(evidence, now)
        if (
            evidence.get("pinky_id") != self.command["payload"]["pinky_id"]
            or evidence.get("docking_execution_id") != self.docking_execution_id
            or evidence.get("p7_clear") is not True
            or evidence.get("docked") is not False
        ):
            raise ValueError("assigned_pinky_exit_required")
        self._consume_evidence(evidence)
        if self.state == "INBOUND_WAIT_P7_CLEAR":
            self.stored = {
                k: self.command["payload"][k]
                for k in ("order_id", "package_id", "slot_id")
            }
            self.state = "STORED"
        else:
            self.state = "DELIVERY_READY"
        self.command = None
        self.result = None
        self.arm_safety = None
