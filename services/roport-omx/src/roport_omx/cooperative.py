"""shelf-01 Host for OMX1·2 COOPERATIVE tasks (profile roport-omx-task-v1, test stage).

Checks control commands, records them in the journal before acting, gives one task at a time to a
local executor and turns its observations into events kept in an outbox until control's event.ack.
Test agreement (2026-11-07, FFT1-153): completion means both arms are back at home with grippers
open. Package placement is not observed, so placement/quantity/location are reported as null.
No MQTT, HTTP or motor access here; the executor owns the device.
"""

import json
import re
from copy import deepcopy
from datetime import UTC, datetime
from typing import Protocol
from uuid import uuid4

from .mqtt_link.messages import check_command, check_expiry, validate

PROFILE = "roport-omx-task-v1"
WORKCELL = "shelf-01"
ARMS = ("omx1", "omx2")
ARM_FLAGS = ("connected", "grasping", "returned", "safe")
OPERATIONS = {
    "shelf_inbound.requested": "INBOUND",
    "shelf_outbound.requested": "OUTBOUND",
}
TASK_FIELDS = {
    "binding_id",
    "expected_boot_id",
    "operation",
    "workcell_id",
    "manipulation_mode",
    "assigned_arm_id",
    "robot_ids",
    "package_id",
    "quantity",
    "slot_id",
    "pinky_id",
    "dock_session_id",
    "resource_token",
    "model_version",
    "calibration_version",
}
ECHO_FIELDS = (
    "operation",
    "manipulation_mode",
    "assigned_arm_id",
    "robot_ids",
    "package_id",
    "slot_id",
    "pinky_id",
    "dock_session_id",
    "model_version",
    "calibration_version",
)
# shelf-01 is 2 rows x 3 columns: row 1 = lower, 2 = upper; a/b/c = left to right seen from the
# arms. Values are the slot's ArUco marker IDs (docs/aruco/markers.yaml v1.4, S1A..S2C).
SLOTS = {"1a": 28, "1b": 29, "1c": 30, "2a": 31, "2b": 32, "2c": 33}
_ID = re.compile(r"[A-Za-z0-9_.:-]{1,128}\Z")
_IDS = ("type", "task_id", "step_id", "command_id", "execution_id")


class Rejected(ValueError):
    """Request refused before anything started; args[0] is the error code."""


class StorageError(RuntimeError):
    """Journal write failed and memory was rolled back. Do not ACK; restart and reconcile."""


class Executor(Protocol):
    """Local device owner. Unknown values are None, never assumed True."""

    def readiness(self) -> dict:
        """{"armed": bool|None, "busy": bool, "error": str|None, "arms": {arm: flags}}"""

    def start(self, task: dict) -> None:
        """Begin one task; raise to refuse. Must return quickly.

        An executor may also define check(task), raising Rejected before acceptance.
        """

    def observe(self) -> dict:
        """{"state": "running"|"finished"|"failed", "step": str, "error": str|None, "arms": ...}"""


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds")


def _fingerprint(message: dict) -> str:
    return json.dumps(
        {k: v for k, v in message.items() if k not in {"message_id", "sent_at"}},
        sort_keys=True,
        allow_nan=False,
    )


def _arms(raw) -> dict:
    raw = raw if isinstance(raw, dict) else {}
    result = {}
    for arm in ARMS:
        flags = raw.get(arm) if isinstance(raw.get(arm), dict) else {}
        result[arm] = {
            k: flags[k] if isinstance(flags.get(k), bool) else None for k in ARM_FLAGS
        }
    return result


def _returned(arms: dict) -> bool:
    return all(
        (a["connected"], a["returned"], a["safe"], a["grasping"])
        == (True, True, True, False)
        for a in arms.values()
    )


def _released(arms: dict) -> bool | None:
    grasping = [a["grasping"] for a in arms.values()]
    if any(g is True for g in grasping):
        return False
    return True if all(g is False for g in grasping) else None


class CooperativeHost:
    """Single-threaded: call handle() per delivery and poll() from the same loop."""

    def __init__(
        self,
        journal,
        executor: Executor,
        *,
        site: str,
        control: str,
        binding: str,
        require_armed: bool = True,
    ):
        self.journal, self.executor = journal, executor
        self.site, self.control, self.binding = site, control, binding
        self.require_armed = require_armed
        self.boot_id = str(uuid4())
        self.failed = False
        identity = {
            "site": site,
            "control": control,
            "binding": binding,
            "workcell": WORKCELL,
            "profile": PROFILE,
        }
        # Copy: rollback after a failed save restores journal.data, which must stay untouched.
        data = deepcopy(journal.data)
        if data and data.get("identity") != identity:
            raise ValueError(
                "journal belongs to another site/control/binding; "
                "use a separate --journal for this registration"
            )
        self._data = {
            "identity": identity,
            "commands": data.get("commands", {}),
            "outbox": data.get("outbox", []),
            "active": data.get("active"),
            "hold": data.get("hold"),
        }
        if self._data["active"] is not None:
            # A previous process left a task running: its physical result is unknown.
            self._data["hold"], self._data["active"] = self._data["active"], None
            self._save()
        elif not data:
            self._save()

    @property
    def hold(self) -> str | None:
        """Command whose result needs an operator check before any new task."""
        return self._data["hold"]

    def clear_hold(self) -> None:
        """Operator confirmed arms, package and Pinky after a failure or restart."""
        self._usable()
        self._data["hold"] = None
        self._save()

    def pending(self) -> list[dict]:
        """Events not yet acknowledged by control; resend with the same message_id."""
        return list(self._data["outbox"])

    def handle(
        self, message: dict, *, channel: str, qos: int = 1, retained: bool = False
    ) -> list[dict]:
        """Return events to publish for a delivery on the shelf-01 `cmd` or `ack` topic.

        Raises ValueError for deliveries that get no reply, StorageError when the journal
        could not record the result (the caller must not ACK it).
        """
        self._usable()
        if qos != 1 or retained:
            raise ValueError("task link requires non-retained QoS1")
        validate(message)
        if (message["site_id"], message["target"], message["source"]) != (
            self.site,
            WORKCELL,
            self.control,
        ):
            raise ValueError("routing_mismatch")
        if channel == "ack":
            if message["type"] != "event.ack":
                raise ValueError("ack channel carries only event.ack")
            self._ack(message)
            return []
        if channel != "cmd" or not message["type"].endswith(".requested"):
            raise ValueError("invalid command channel/type")
        record = self._data["commands"].get(message["command_id"])
        if record is not None:
            if record["fingerprint"] == _fingerprint(message):
                return list(record["events"])
            # Not remembered: the original command keeps its own result.
            return [
                self._event(
                    self._new(message),
                    "rejected",
                    "REJECTED",
                    {"error_code": "IDEMPOTENCY_CONFLICT"},
                )
            ]
        record = self._new(message)
        try:
            check_command(
                message,
                site=self.site,
                target=WORKCELL,
                source=self.control,
                retained=False,
            )
            check_expiry(message)
            if message.get("profile") != PROFILE:
                raise Rejected("UNSUPPORTED_PROFILE")
            if message["type"] == "status.requested":
                return self._status(message, record)
            task = self._check_task(message)
            if self._data["hold"] is not None:
                raise Rejected("HOST_HOLD")
            if self._data["active"] is not None:
                raise Rejected("HOST_BUSY")
            ready = self._readiness()
            if ready["error"]:
                raise Rejected("EXECUTOR_ERROR")
            if ready["busy"]:
                raise Rejected("EXECUTOR_BUSY")
            if self.require_armed and ready["armed"] is not True:
                raise Rejected("NOT_ARMED")
            check = getattr(self.executor, "check", None)
            if check is not None:
                check(task)  # executor-specific refusal (e.g. no model for this slot)
        except ValueError as exc:
            code = exc.args[0] if isinstance(exc, Rejected) else "INVALID_REQUEST"
            detail = {} if isinstance(exc, Rejected) else {"detail": str(exc)}
            return self._remember(
                message, record, "rejected", "REJECTED", {"error_code": code, **detail}
            )

        record["task"] = task
        self._data["active"] = message["command_id"]
        # Saved with the accepted event before the executor is called.
        events = self._remember(
            message, record, "accepted", "ACCEPTED", self._echo(task)
        )
        try:
            self.executor.start(task)
        except Exception as exc:  # noqa: BLE001 - any refusal ends this execution
            return events + self._finish(record, "EXECUTOR_START_FAILED", {}, str(exc))
        return events

    def poll(self) -> list[dict]:
        """Turn executor observations of the active task into progress/completed/failed."""
        self._usable()
        command_id = self._data["active"]
        if command_id is None:
            return []
        record = self._data["commands"][command_id]
        try:
            seen = self.executor.observe()
        except Exception:  # noqa: BLE001 - keep waiting; the device keeps its own safety
            return []
        state = seen.get("state")
        if state == "running":
            step = seen.get("step") if isinstance(seen.get("step"), str) else None
            if step == record["last_step"]:
                return []
            record["last_step"] = step
            return self._push(
                record,
                self._event(
                    record,
                    "progress",
                    "RUNNING",
                    {**self._echo(record["task"]), "step": step},
                ),
            )
        if state not in {"finished", "failed"}:
            return []
        arms = _arms(seen.get("arms"))
        if state == "finished" and _returned(arms):
            self._data["active"] = None
            payload = {
                **self._echo(record["task"]),
                "quantity_confirmed": None,
                "package_location": None,
                "placement_confirmed": None,
                "grasp_released": True,
                "arms": arms,
                "departure_ready": True,
                "evidence": "arm_return_only",
            }
            return self._push(
                record, self._event(record, "completed", "COMPLETE", payload)
            )
        code = seen.get("error") if state == "failed" else "ARM_NOT_RETURNED"
        return self._finish(record, code or "EXECUTION_FAILED", arms)

    def _finish(
        self, record: dict, code: str, arms: dict, detail: str | None = None
    ) -> list[dict]:
        self._data["active"] = None
        self._data["hold"] = record["ids"]["command_id"]
        arms = _arms(arms)
        payload = {
            **self._echo(record["task"]),
            "error_code": code,
            "last_step": record["last_step"],
            "arms": arms,
            "grasp_released": _released(arms),
            "departure_ready": False,
        }
        if detail:
            payload["detail"] = detail[:200]
        return self._push(record, self._event(record, "failed", "FAILED", payload))

    def _status(self, message: dict, record: dict) -> list[dict]:
        if set(message["payload"]) != {"binding_id"}:
            raise Rejected("UNSUPPORTED_FIELDS")
        if message["payload"]["binding_id"] != self.binding:
            raise Rejected("BINDING_MISMATCH")
        ready = self._readiness()
        active = self._data["active"]
        payload = {
            "ready": not ready["error"]
            and not ready["busy"]
            and active is None
            and self._data["hold"] is None
            and (ready["armed"] is True or not self.require_armed),
            "armed": ready["armed"],
            "require_armed": self.require_armed,
            "active": self._data["commands"][active]["ids"] if active else None,
            "hold": self._data["hold"],
            "executor_error": ready["error"],
            "arms": _arms(ready.get("arms")),
            "supported_profiles": [PROFILE],
            "slots": list(SLOTS),
        }
        return self._remember(message, record, "completed", "STATUS", payload)

    def _check_task(self, message: dict) -> dict:
        operation = OPERATIONS.get(message["type"])
        if operation is None:
            raise Rejected("UNSUPPORTED_TYPE")
        p = message["payload"]
        if set(p) != TASK_FIELDS:
            raise Rejected(
                "UNSUPPORTED_FIELDS" if set(p) - TASK_FIELDS else "MISSING_FIELDS"
            )
        if p["binding_id"] != self.binding:
            raise Rejected("BINDING_MISMATCH")
        if p["expected_boot_id"] != self.boot_id:
            raise Rejected("BOOT_MISMATCH")
        if (
            p["operation"],
            p["workcell_id"],
            p["manipulation_mode"],
            p["assigned_arm_id"],
            p["robot_ids"],
        ) != (operation, WORKCELL, "COOPERATIVE", None, list(ARMS)):
            raise Rejected("COOPERATIVE_SHELF_REQUIRED")
        if type(p["quantity"]) is not int or p["quantity"] != 1:
            raise Rejected("QUANTITY_UNSUPPORTED")
        for key in TASK_FIELDS - {
            "binding_id",
            "expected_boot_id",
            "operation",
            "workcell_id",
            "manipulation_mode",
            "assigned_arm_id",
            "robot_ids",
            "quantity",
        }:
            if not isinstance(p[key], str) or not _ID.fullmatch(p[key]):
                raise Rejected("INVALID_FIELD")
        if p["slot_id"] not in SLOTS:
            raise Rejected("UNKNOWN_SLOT")
        task = {k: p[k] for k in TASK_FIELDS - {"binding_id", "expected_boot_id"}}
        return {
            **task,
            "command_id": message["command_id"],
            "execution_id": message["execution_id"],
        }

    def _readiness(self) -> dict:
        try:
            ready = self.executor.readiness()
        except Exception as exc:  # noqa: BLE001 - an unreachable executor is not ready
            return {
                "armed": None,
                "busy": False,
                "error": str(exc)[:200] or "unreachable",
                "arms": {},
            }
        armed = ready.get("armed")
        return {
            "armed": armed if isinstance(armed, bool) else None,
            "busy": ready.get("busy") is not False,
            "error": ready.get("error"),
            "arms": ready.get("arms"),
        }

    @staticmethod
    def _echo(task: dict | None) -> dict:
        return {k: task[k] for k in ECHO_FIELDS} if task else {}

    def _new(self, message: dict) -> dict:
        return {
            "fingerprint": _fingerprint(message),
            "ids": {k: message[k] for k in _IDS},
            "task": None,
            "events": [],
            "revision": 0,
            "last_step": None,
        }

    def _remember(
        self, message: dict, record: dict, kind: str, phase: str, payload: dict
    ) -> list[dict]:
        self._data["commands"][message["command_id"]] = record
        return self._push(record, self._event(record, kind, phase, payload))

    def _event(self, record: dict, kind: str, phase: str, payload: dict) -> dict:
        record["revision"] += 1
        now = _now()
        ids = record["ids"]
        return {
            "schema_version": "1.0",
            "profile": PROFILE,
            "message_id": str(uuid4()),
            "type": f"{ids['type'].removesuffix('.requested')}.{kind}",
            "site_id": self.site,
            "source": WORKCELL,
            "target": self.control,
            "sent_at": now,
            **{k: ids[k] for k in _IDS if k != "type"},
            "payload": {
                "binding_id": self.binding,
                "boot_id": self.boot_id,
                "observed_at": now,
                "phase_revision": record["revision"],
                "phase": phase,
                **payload,
            },
        }

    def _push(self, record: dict, event: dict) -> list[dict]:
        """Journal first; the caller publishes only what was saved."""
        record["events"].append(event)
        self._data["outbox"].append(event)
        self._save()
        return [event]

    def _ack(self, message: dict) -> None:
        payload = message["payload"]
        if payload.get("status") != "stored":
            raise ValueError("unsupported_ack")
        outbox = self._data["outbox"]
        acked = payload.get("acked_message_id")
        event = next((e for e in outbox if e["message_id"] == acked), None)
        if event is None:
            return
        if any(
            event[k] != message[k]
            for k in ("task_id", "step_id", "command_id", "execution_id")
        ):
            raise ValueError("ack correlation mismatch")
        self._data["outbox"] = [e for e in outbox if e is not event]
        self._save()

    def _usable(self) -> None:
        if self.failed:
            raise StorageError("journal failed; restart the Host and reconcile")

    def _save(self) -> None:
        try:
            self.journal.save(self._data)
        except Exception as exc:
            # Unsaved commands must never answer a retry as if they had been accepted.
            self.failed = True
            self._data = deepcopy(self.journal.data)
            raise StorageError("journal save failed; result not recorded") from exc
