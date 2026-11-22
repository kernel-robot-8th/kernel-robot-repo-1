"""shelf-01 Host for the field MQTT contract v2 (profile roport-control-v2, Fleet control).

Same device side as the v1 test Host (cooperative.py): one task at a time through a local
executor, the journal written before acting, results kept in an outbox until control's
event_stored. Only the MQTT side differs (Confluence "메세지 규약 정리", interfaces/mqtt):

- topics roport/workcell/shelf-01/{cmd,event,ack} and roport/robot/{omx1,omx2}/{cmd,event,ack};
- shelf_transfer_command (store = Pinky → shelf, retrieve = shelf → Pinky) for the B column with
  both arms; arm_action_command (one arm, columns A/C) is refused: two-arm missions only for now;
- robot_stop_command stops the running task: the original command ends interrupted and the stop
  completed once the executor saw the run end;
- robot_state for shelf-01, omx1 and omx2 answers every robot_state_request (and a 1 Hz tick).

What the Host can observe is reported, nothing else (unknown stays unknown):
- Pinky docking: the Pinky's own robot_state (stopped at N11 with the arrival command named in
  the transfer). No workcell sensor confirms the alignment, so docking.alignment is "unknown".
- placement/release: the executor's completion evidence (aligner or placement judge for store,
  the tray check for retrieve) with both arms home and grippers released.
No MQTT, HTTP or motor access here; the executor owns the device.
"""

import json
import logging
import re
import time
from copy import deepcopy
from datetime import UTC, datetime
from uuid import uuid4

from .cooperative import RELEASE_CHECKS, Rejected, StartRefused, StorageError, _arms
from .mqtt_link import field_wire

log = logging.getLogger(__name__)

PROFILE = field_wire.PROFILE
WORKCELL = "shelf-01"
ARMS = ("omx1", "omx2")
ENDPOINTS = (WORKCELL, *ARMS)
PINKIES = ("pinky1", "pinky2", "pinky3")
NODE = "N11"  # the Pinky stop in front of the shelf's B column (Fleet SLOTS)
SLOTS = ("1b", "2b")  # both arms together; A (omx1) and C (omx2) are one-arm slots
OPERATIONS = {"store": "INBOUND", "retrieve": "OUTBOUND"}
# Completion evidence that saw the box where it belongs (cooperative.Executor.observe)
PLACED = {
    "INBOUND": {"shelf_align", "shelf_load_not_aligned"},
    "OUTBOUND": {"pinky_load"},
}
SENT_AGE_S = (
    -0.5,
    3.0,
)  # a new transfer's sent_at against this PC's clock (spec, Pinky alike)
OBSERVATION_MAX_AGE_S = (
    3.0  # Pinky state / readiness sample older than this proves nothing
)
STOP_TIMEOUT_S = 30.0  # the panel's safe stop lands the arms within a few seconds
TERMINAL = {"completed", "rejected", "failed", "interrupted"}
_REASON = re.compile(r"[^A-Za-z0-9_.:-]+")


def _iso(at: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if at is None else at, UTC).isoformat(
        timespec="milliseconds"
    )


def _reason(code) -> str:
    text = _REASON.sub("_", str(code or "unknown")).strip("_").lower()
    return text[:128] or "unknown"


def _fingerprint(message: dict) -> str:
    # A resend keeps message_id, sent_at and the body (spec); compare everything.
    return json.dumps(message, sort_keys=True, allow_nan=False)


def _holding(flags: dict) -> str:
    grasping = flags.get("grasping")
    return (
        "holding" if grasping is True else "empty" if grasping is False else "unknown"
    )


def _held(arms: dict) -> str:
    states = {_holding(a) for a in arms.values()}
    return (
        "holding"
        if "holding" in states
        else states.pop()
        if len(states) == 1
        else "unknown"
    )


def _home(flags: dict) -> bool:
    return (
        flags["connected"],
        flags["returned"],
        flags["safe"],
        flags["grasping"],
    ) == (
        True,
        True,
        True,
        False,
    )


class FieldHost:
    """Single-threaded: call handle() per delivery and poll() from the same loop.

    readiness: optional callable returning (sample, observed_unix_s) from a sampler thread, so a
    slow panel read never delays the MQTT loop; default reads the executor directly.
    """

    def __init__(
        self,
        journal,
        executor,
        *,
        control: str,
        require_armed: bool = True,
        require_pinky: bool = True,
        readiness=None,
    ):
        self.journal, self.executor = journal, executor
        self.control = control
        self.require_armed, self.require_pinky = require_armed, require_pinky
        self._sample = readiness or (lambda: (self._read(), time.time()))
        self.boot_version = "boot-" + str(uuid4())
        self.failed = False
        self.state_sequence = 0
        self.pinkies: dict[str, dict] = {}
        identity = {"control": control, "workcell": WORKCELL, "profile": PROFILE}
        data = deepcopy(journal.data)
        if data and data.get("identity") != identity:
            raise ValueError(
                "journal belongs to another control/profile; use a new --journal for v2"
            )
        self._data = {
            "identity": identity,
            "commands": data.get("commands", {}),
            "outbox": data.get("outbox", []),
            "active": data.get("active"),
            "hold": data.get("hold"),
            "stop": None,  # a stop never survives a restart: the new boot re-checks
        }
        self._stop_since: float | None = None
        if self._data["active"] is not None:
            # A previous process left a task running: its physical result is unknown.
            self._data["hold"], self._data["active"] = self._data["active"], None
            self._save()
        elif not data or data.get("stop"):
            self._save()

    # --- operator side (panel [HOLD 해제], same files as v1) ---------------------------------
    @property
    def boot_id(self) -> str:
        return self.boot_version

    @property
    def active(self) -> str | None:
        return self._data["active"]

    @property
    def hold(self) -> str | None:
        return self._data["hold"]

    def clear_hold(self) -> None:
        self._usable()
        self._data["hold"] = None
        self._save()

    def hold_info(self) -> dict | None:
        command_id = self._data["hold"]
        if command_id is None:
            return None
        record = self._data["commands"].get(command_id) or {}
        last = next(
            (
                e
                for e in reversed(record.get("events") or [])
                if e["payload"]["status"] in {"failed", "interrupted"}
            ),
            None,
        )
        return {
            "command_id": command_id,
            "error_code": (last or {}).get("payload", {}).get("reason")
            or "result_unknown",
            "detail": record.get("detail"),
            "mission_result": "MISSION_HOLD",
            "last_step": record.get("last_step"),
            "since": (last or {}).get("sent_at"),
        }

    def release_hold(self, request: dict) -> dict:
        result = {
            "request_id": request.get("request_id"),
            "hold": self._data["hold"],
            "cleared": False,
        }
        checks = (
            request.get("checks") if isinstance(request.get("checks"), dict) else {}
        )
        if self._data["hold"] is None:
            return result | {"reason": "NO_HOLD"}
        if request.get("hold") != self._data["hold"]:
            return result | {"reason": "HOLD_CHANGED"}
        if self._data["active"] is not None:
            return result | {"reason": "TASK_ACTIVE"}
        if not all(checks.get(k) is True for k in RELEASE_CHECKS):
            return result | {"reason": "CHECKS_MISSING"}
        ready = self._read()
        if ready["busy"]:
            return result | {"reason": "EXECUTOR_BUSY"}
        arms = _arms(ready.get("arms"))
        if not all(
            (a["connected"], a["returned"], a["grasping"]) == (True, True, False)
            for a in arms.values()
        ):
            return result | {"reason": "ARMS_NOT_HOME", "arms": arms}
        self.clear_hold()
        return result | {"cleared": True, "by": request.get("by")}

    # --- MQTT side ------------------------------------------------------------------------
    def pending(self) -> list[dict]:
        """command_status events not yet stored by control; resend unchanged."""
        return list(self._data["outbox"])

    def observe_pinky(self, message: dict) -> None:
        """A Pinky's robot_state (its own topic, QoS1, not retained: checked by the caller)."""
        if message["type"] != "robot_state" or message["source"] not in PINKIES:
            raise ValueError("not a Pinky state")
        p = message["payload"]
        old = self.pinkies.get(message["source"])
        if (
            old
            and old["boot_version"] == p["boot_version"]
            and p["sequence"] <= old["sequence"]
        ):
            return
        self.pinkies[message["source"]] = {
            "boot_version": p["boot_version"],
            "sequence": p["sequence"],
            "observed_at": p["observed_at"],
            "active_command_id": p["active_command_id"],
            "state": deepcopy(p["state"]),
            "received": time.monotonic(),
        }

    def handle(
        self,
        message: dict,
        *,
        endpoint: str,
        channel: str,
        qos: int = 1,
        retained: bool = False,
    ) -> list[dict]:
        """Events to publish for a delivery on `endpoint`'s cmd/ack topic.

        Raises ValueError for deliveries that get no reply (logged by the caller), StorageError
        when the journal could not record the result (the caller must not ACK it).
        """
        self._usable()
        if qos != 1 or retained:
            raise ValueError("task link requires non-retained QoS1")
        field_wire.validate(message)
        if (
            endpoint not in ENDPOINTS
            or message["target"] != endpoint
            or message["source"] != self.control
        ):
            raise ValueError("routing_mismatch")
        kind = message["type"]
        if channel == "ack":
            if kind != "event_stored":
                raise ValueError("ack channel carries only event_stored")
            self._ack(message["payload"]["acked_message_id"])
            return []
        if channel != "cmd":
            raise ValueError("invalid channel")
        if kind == "robot_state_request":
            return [self.state(endpoint)]
        if "command_id" not in message:
            raise ValueError(f"{kind} is not a command")
        record = self._data["commands"].get(message["command_id"])
        if record is not None:
            if record["fingerprint"] == _fingerprint(message):
                return list(record["events"])
            # Never answer under the original command's ID: its own result stands (spec).
            raise ValueError(
                "idempotency_conflict: command_id reused with another body"
            )
        record = {
            "fingerprint": _fingerprint(message),
            "command": deepcopy(message),
            "endpoint": endpoint,
            "task": None,
            "events": [],
            "sequence": 0,
            "last_step": None,
        }
        self._data["commands"][message["command_id"]] = record
        try:
            if datetime.now(UTC) >= field_wire.timestamp(message["expires_at"]):
                raise Rejected("expired_command")
            if message["payload"]["expected_boot_version"] != self.boot_version:
                raise Rejected("boot_version_mismatch")
            if kind == "robot_stop_command":
                return self._stop_request(record)
            if kind == "arm_action_command":
                raise Rejected("unsupported_one_arm_command")  # two-arm missions only
            if kind != "shelf_transfer_command" or endpoint != WORKCELL:
                raise Rejected("unsupported_command")
            age = (
                datetime.now(UTC) - field_wire.timestamp(message["sent_at"])
            ).total_seconds()
            if not SENT_AGE_S[0] <= age <= SENT_AGE_S[1]:
                raise Rejected("stale_message")
            task = self._check_transfer(message)
        except Rejected as exc:
            return self._status(record, "rejected", reason=_reason(exc.args[0]))
        except ValueError as exc:
            record["detail"] = str(exc)[:200]
            return self._status(record, "rejected", reason="invalid_command")

        record["task"] = task
        self._data["active"] = message["command_id"]
        events = self._status(record, "accepted")  # saved before the executor is called
        try:
            if datetime.now(UTC) >= field_wire.timestamp(message["expires_at"]):
                raise StartRefused("expired_command", hold=False)
            self.executor.start(task)
        except StartRefused as exc:  # refused before any motion, with its own reason
            return events + self._end(
                record, "failed", _reason(exc.code), hold=exc.hold, detail=exc.detail
            )
        except Exception as exc:  # noqa: BLE001 - any refusal ends this execution
            return events + self._end(
                record, "failed", "executor_start_failed", detail=str(exc)
            )
        return events

    def poll(self) -> list[dict]:
        """Executor observations of the active task → in_progress / completed / failed."""
        self._usable()
        command_id = self._data["active"]
        stop = self._data["stop"]
        if command_id is None:
            return self._settle_stop_without_task() if stop else []
        record = self._data["commands"][command_id]
        events = self._watch_pinky(record)
        try:
            seen = self.executor.observe()
        except Exception:  # noqa: BLE001 - keep waiting; the device keeps its own safety
            return events + self._stop_timeout()
        state = seen.get("state")
        if state in {"running", "placed"}:  # v2 reports once, after the alignment
            step = seen.get("step") if isinstance(seen.get("step"), str) else state
            if step != record["last_step"]:
                record["last_step"] = step
                events += self._status(record, "in_progress")
            return events + self._stop_timeout()
        if state not in {"finished", "failed"}:
            return events + self._stop_timeout()
        return events + self._terminal(record, seen)

    def state(self, endpoint: str) -> dict:
        """robot_state for shelf-01, omx1 or omx2 from the latest readiness sample."""
        try:
            ready, observed = self._sample()
        except Exception as exc:  # noqa: BLE001 - an unreadable executor is not ready
            ready, observed = (
                {"armed": None, "busy": False, "error": str(exc), "arms": {}},
                time.time(),
            )
        stale = time.time() - observed > OBSERVATION_MAX_AGE_S
        arms = _arms(ready.get("arms"))
        busy = ready.get("busy") is not False
        active = self._data["active"]
        idle = (
            active is None
            and self._data["hold"] is None
            and self._data["stop"] is None
            and not busy
            and not stale
            and not ready.get("error")
            and (ready.get("armed") is True or not self.require_armed)
        )
        if self._data["hold"] is not None:
            fault = "host_hold"
        elif stale:
            fault = "observation_stale"
        elif ready.get("error") and active is None:
            fault = _reason(ready["error"])
        else:
            fault = None
        package = (
            (self._data["commands"][active]["task"] or {}).get("package_id")
            if active
            else None
        )
        per_arm = {}
        for name in ARMS:
            flags = arms[name]
            holding = _holding(flags) if not busy and not stale else "unknown"
            per_arm[name] = {
                "stopped": not busy and not stale and flags["connected"] is True,
                "ready": idle and _home(flags),
                "home_reached": not busy and not stale and flags["returned"] is True,
                "holding_state": holding,
                "holding_package_id": package if holding == "holding" else None,
                "fault": fault,
            }
        if endpoint == WORKCELL:
            state = {
                "stopped": all(a["stopped"] for a in per_arm.values()),
                "ready": all(a["ready"] for a in per_arm.values()),
                "fault": fault,
                "arms": [
                    {
                        "robot_id": name,
                        "boot_version": self.boot_version,
                        "active_command_id": active,
                        **per_arm[name],
                    }
                    for name in ARMS
                ],
                "docking": self._docking(),
            }
        else:
            state = per_arm[endpoint]
        self.state_sequence += 1
        return field_wire.build_message(
            "robot_state",
            endpoint,
            self.control,
            {
                "boot_version": self.boot_version,
                "observed_at": _iso(observed),
                "sequence": self.state_sequence,
                "active_command_id": active,
                "state": state,
            },
        )

    # --- transfer -------------------------------------------------------------------------
    def _check_transfer(self, message: dict) -> dict:
        p = message["payload"]
        if any(a["expected_boot_version"] != self.boot_version for a in p["arms"]):
            raise Rejected("participating_arms_mismatch")
        if p["slot_id"] not in SLOTS:
            raise Rejected("cooperative_slot_mismatch")
        if self._data["hold"] is not None:
            raise Rejected("host_hold")
        if self._data["active"] is not None or self._data["stop"] is not None:
            raise Rejected("resource_busy")
        ready = self._read()
        if ready["error"]:
            raise Rejected("executor_error")
        if ready["busy"]:
            raise Rejected("executor_busy")
        if self.require_armed and ready["armed"] is not True:
            raise Rejected("not_armed")
        if not all(_home(a) for a in _arms(ready.get("arms")).values()):
            raise Rejected("arm_not_safely_home")
        if self.require_pinky:
            self._check_pinky(p)
        task = {
            "operation": OPERATIONS[p["operation"]],
            "workcell_id": WORKCELL,
            "manipulation_mode": "COOPERATIVE",
            "assigned_arm_id": None,
            "robot_ids": list(ARMS),
            "package_id": p["package_id"],
            "quantity": 1,
            "slot_id": p["slot_id"],
            "pinky_id": p["pinky_id"],
            "arrival_command_id": p["arrival_command_id"],
            "expected_pinky_boot_version": p["expected_pinky_boot_version"],
            "model_version": p["model_version"],
            "calibration_version": p["calibration_version"],
            "command_id": message["command_id"],
            "execution_id": message["command_id"],
            "expires_at": message["expires_at"],
        }
        check = getattr(self.executor, "check", None)
        if check is not None:
            check(
                task
            )  # executor refusal (no route/model for this box), Rejected(code)
        return task

    def _pinky_seen(self, pinky: str) -> dict | None:
        seen = self.pinkies.get(pinky)
        if not seen or time.monotonic() - seen["received"] > OBSERVATION_MAX_AGE_S:
            return None
        return seen

    def _check_pinky(self, p: dict) -> None:
        seen = self._pinky_seen(p["pinky_id"])
        if seen is None:
            raise Rejected("pinky_observation_stale")
        if seen["boot_version"] != p["expected_pinky_boot_version"]:
            raise Rejected("pinky_boot_mismatch")
        if not self._parked(seen, p["arrival_command_id"]):
            raise Rejected("pinky_visit_not_confirmed")

    @staticmethod
    def _parked(seen: dict, arrival: str | None) -> bool:
        s = seen["state"]
        return (
            s.get("node") == NODE
            and s.get("stopped") is True
            and s.get("line_enabled") is False
            and s.get("drive_owner") == "none"
            and s.get("fault") is None
            and seen["active_command_id"] is None
            and (arrival is None or s.get("arrival_command_id") == arrival)
        )

    def _docking(self) -> dict | None:
        active = self._data["active"]
        task = self._data["commands"][active]["task"] if active else None
        for pinky in [task["pinky_id"]] if task else PINKIES:
            seen = self._pinky_seen(pinky)
            if seen and self._parked(seen, None):
                return {
                    "pinky_id": pinky,
                    "pinky_boot_version": seen["boot_version"],
                    "arrival_command_id": seen["state"].get("arrival_command_id"),
                    "observed_at": seen["observed_at"],
                    "alignment": "unknown",  # no workcell sensor confirms the alignment
                }
        return None

    def _watch_pinky(self, record: dict) -> list[dict]:
        """A Pinky seen leaving during the task stops the arms (a lost state proves nothing)."""
        task = record["task"]
        if not self.require_pinky or record.get("abort") or task is None:
            return []
        seen = self._pinky_seen(task["pinky_id"])
        if seen is None:
            return []
        if seen["boot_version"] == task["expected_pinky_boot_version"] and self._parked(
            seen, task["arrival_command_id"]
        ):
            return []
        record["abort"] = "pinky_left"
        log.warning(
            "Pinky %s left %s during the task; stopping the arms",
            task["pinky_id"],
            NODE,
        )
        self._request_stop()
        self._save()
        return []

    def _terminal(self, record: dict, seen: dict) -> list[dict]:
        arms = _arms(seen.get("arms"))
        task = record["task"]
        stop = self._data["stop"]
        detail = seen.get("detail") if isinstance(seen.get("detail"), str) else None
        if seen["state"] == "finished" and all(_home(a) for a in arms.values()):
            evidence = seen.get("evidence")
            if evidence in PLACED[task["operation"]] and not record.get("abort"):
                events = self._end(
                    record,
                    "completed",
                    result={
                        "package_id": task["package_id"],
                        "slot_id": task["slot_id"],
                        "pinky_id": task["pinky_id"],
                        "arrival_command_id": task["arrival_command_id"],
                        "placement_confirmed": True,
                        "release_confirmed": True,
                        "arms": [
                            {
                                "robot_id": name,
                                "boot_version": self.boot_version,
                                "stopped": True,
                                "home_reached": True,
                                "holding_package_id": None,
                                "holding_state": "empty",
                            }
                            for name in ARMS
                        ],
                    },
                    hold=False,
                    evidence=evidence,
                    align_error=seen.get("align_error"),
                )
            else:
                reason = record.get("abort") or "placement_or_release_unconfirmed"
                events = self._end(
                    record, "failed", reason, detail=f"evidence {evidence}"
                )
            return events + self._finish_stop(stopped=True)
        settled = seen["state"] == "failed" and (
            seen.get("operator_stop") is True or seen.get("recovered") is True
        )
        if (
            stop is not None
            and stop["target"] == record["command"]["command_id"]
            and seen.get("operator_stop") is True
        ):
            events = self._end(
                record,
                "interrupted",
                stop["reason"],
                result={
                    "stopped": True,
                    "holding_state": _held(arms),
                    "holding_package_id": None,
                },
                hold=bool(record.get("abort")) or not settled,
                detail=detail,
            )
            return events + self._finish_stop(stopped=True)
        code = seen.get("error") if seen["state"] == "failed" else "arm_not_returned"
        events = self._end(
            record,
            "failed",
            record.get("abort") or _reason(code or "execution_failed"),
            hold=bool(record.get("abort")) or not settled,
            detail=detail,
        )
        return events + self._finish_stop(stopped=True)

    # --- stop -----------------------------------------------------------------------------
    def _stop_request(self, record: dict) -> list[dict]:
        p = record["command"]["payload"]
        if self._data["stop"] is not None:
            raise Rejected("stop_already_pending")
        target = p["target_command_id"]
        if target == record["command"]["command_id"]:
            raise Rejected("invalid_stop_target")
        known = self._data["commands"].get(target)
        if known is None:
            raise Rejected("unknown_target_command")
        if self._data["active"] not in {None, target}:
            raise Rejected(
                "target_not_active"
            )  # another task runs; never stop it by mistake
        self._data["stop"] = {
            "command_id": record["command"]["command_id"],
            "target": target,
            "reason": _reason(p["reason"]),
        }
        events = self._status(record, "accepted")
        if target == self._data["active"]:
            if not callable(getattr(self.executor, "stop", None)):
                self._data["stop"] = None
                return events + self._end(
                    record, "failed", "physical_stop_adapter_unavailable", hold=False
                )
            self._request_stop()
            self._save()
        return events  # an ended target settles on the next poll

    def _request_stop(self) -> None:
        if self._stop_since is None:
            self._stop_since = time.monotonic()
            try:
                self.executor.stop()
            except Exception as exc:  # noqa: BLE001 - the timeout reports the unconfirmed stop
                log.warning("executor stop failed: %s", exc)

    def _settle_stop_without_task(self) -> list[dict]:
        """Stop for a command that already ended (or a previous boot's): idle arms are stopped."""
        ready = self._read()
        if ready["busy"]:
            return self._stop_timeout()
        return self._finish_stop(stopped=True)

    def _finish_stop(self, *, stopped: bool) -> list[dict]:
        stop = self._data["stop"]
        if stop is None:
            return []
        self._data["stop"], self._stop_since = None, None
        record = self._data["commands"][stop["command_id"]]
        if stopped:
            return self._end(
                record,
                "completed",
                result={"stopped": True},
                hold=False,
                keep_active=True,
            )
        return self._end(
            record,
            "failed",
            "physical_stop_not_confirmed",
            hold=False,
            keep_active=True,
        )

    def _stop_timeout(self) -> list[dict]:
        if self._data["stop"] is None:
            return []
        since = self._stop_since if self._stop_since is not None else time.monotonic()
        self._stop_since = since
        if time.monotonic() - since < STOP_TIMEOUT_S:
            return []
        # The arms may still move: the running task stays active and the workcell holds.
        self._data["hold"] = self._data["active"] or self._data["stop"]["target"]
        return self._finish_stop(stopped=False)

    # --- events and journal ---------------------------------------------------------------
    def _end(
        self,
        record: dict,
        status: str,
        reason: str | None = None,
        *,
        result: dict | None = None,
        hold: bool = True,
        detail: str | None = None,
        keep_active: bool = False,
        **extra,
    ) -> list[dict]:
        if not keep_active and self._data["active"] == record["command"]["command_id"]:
            self._data["active"] = None
        if status != "completed" and hold:
            self._data["hold"] = record["command"]["command_id"]
        if detail:
            record["detail"] = detail[:200]
        record.update(
            {k: v for k, v in extra.items() if isinstance(v, str)}
        )  # journal only
        return self._status(record, status, reason=reason, result=result)

    def _status(
        self, record: dict, status: str, *, reason=None, result=None
    ) -> list[dict]:
        record["sequence"] += 1
        command = record["command"]
        payload = {
            "boot_version": self.boot_version,
            "observed_at": _iso(),
            "status": status,
            "sequence": record["sequence"],
        }
        if reason is not None:
            payload["reason"] = reason
        if result is not None:
            payload["result"] = result
        event = field_wire.build_message(
            "command_status",
            record["endpoint"],
            self.control,
            payload,
            command_id=command["command_id"],
        )
        record["status"] = status
        record["events"].append(event)
        self._data["outbox"].append(event)
        self._save()  # journal first; the caller publishes only what was saved
        return [event]

    def _ack(self, acked: str) -> None:
        outbox = self._data["outbox"]
        if not any(e["message_id"] == acked for e in outbox):
            return
        self._data["outbox"] = [e for e in outbox if e["message_id"] != acked]
        self._save()

    def _read(self) -> dict:
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

    def _usable(self) -> None:
        if self.failed:
            raise StorageError("journal failed; restart the Host and reconcile")

    def _save(self) -> None:
        try:
            self.journal.save(self._data)
        except Exception as exc:
            self.failed = True
            self._data = deepcopy(self.journal.data)
            raise StorageError("journal save failed; result not recorded") from exc
