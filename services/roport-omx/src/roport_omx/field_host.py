"""r8 OMX command owner. Sensor observations, not commands, establish docking/completion."""

import logging
import re
import time
from copy import deepcopy
from datetime import UTC, datetime
from uuid import uuid4

from .mqtt_link.field_wire import build_message, validate

log = logging.getLogger(__name__)

ARMS = ("omx1", "omx2", "omx3")
SLOTS = {"omx1": {"1a", "2a"}, "omx2": {"1c", "2c"}, "shelf-01": {"1b", "2b"}}
NODES = {"omx1": "N21", "omx2": "N22", "shelf-01": "N11", "omx3": "N01"}
TERMINAL = {"completed", "failed", "rejected", "interrupted"}


def _now():
    return datetime.now(UTC).isoformat()


def _fresh(value, max_age):
    try:
        age = (datetime.now(UTC) - datetime.fromisoformat(value)).total_seconds()
        return -1 <= age <= max_age
    except (ValueError, TypeError):
        return False


def _flags(raw):
    """Preserve unknown readings; a missing motor observation never means stopped."""
    raw = raw if isinstance(raw, dict) else {}
    safe = raw.get("safe") is True and raw.get("connected") is True
    home = raw.get("returned") is True
    stopped = raw.get("stopped") is True
    held = raw.get("grasping")
    return {
        "stopped": stopped,
        "ready": safe and stopped and home and held is False,
        "home_reached": home,
        "holding_state": "holding"
        if held is True
        else "empty"
        if held is False
        else "unknown",
        "holding_package_id": raw.get("holding_package_id"),
        "fault": None if safe else "arm_observation_unavailable",
    }


class FieldOmxHost:
    """One process owns every supplied executor; physical state is never inferred from MQTT ACKs.

    Executors provide readiness/start/observe and abort. alignment(command) supplies a fresh
    local sensor record: pinky_id, pinky_boot_version, arrival_command_id, observed_at,
    alignment. The callback must not manufacture evidence from the command's identity.
    """

    def __init__(
        self,
        journal,
        executors,
        *,
        control="control-01",
        alignment=None,
        freshness_s=3.0,
        watchdog_s=3.0,
    ):
        self.journal, self.executors, self.control = journal, executors, control
        self.alignment = alignment or (lambda command: None)
        self.freshness_s, self.watchdog_s = freshness_s, watchdog_s
        self.boot_version = str(uuid4())
        self.pinkies = {}
        self.retired_boots = {}
        self.renewals = set()
        self.deadline = None
        self.last_progress = None
        self.stop_deadline = None
        self.state_sequence = {}
        self.failed = False
        identity = {
            "control": control,
            "targets": sorted(executors),
            "profile": "roport-control-v2",
        }
        self.data = deepcopy(journal.data)
        if self.data and self.data.get("identity") != identity:
            raise ValueError("OMX journal belongs to a different owner or endpoint set")
        if not self.data:
            self.data = {
                "identity": identity,
                "commands": {},
                "outbox": [],
                "active": None,
                "hold": None,
            }
        recovering = self.data["active"] or self.data.get("recovery_command_id")
        if recovering:
            stop = self.data.pop("stop", None)
            self._fail(
                self.data["commands"][recovering]["command"],
                "host_restarted_outcome_unknown",
            )
            if stop:
                self._event(stop, "failed", reason="host_restarted_outcome_unknown")
        self._save()

    def _save(self):
        try:
            self.journal.save(self.data)
        except Exception:
            self.failed = True
            self.data = deepcopy(self.journal.data)
            raise

    def pending(self):
        return deepcopy(self.data["outbox"])

    def observe_pinky(self, message):
        validate(message)
        if (
            message["target"] != self.control
            or message["type"] != "robot_state"
            or message["source"]
            not in {
                "pinky1",
                "pinky2",
                "pinky3",
            }
        ):
            raise ValueError("expected Pinky robot_state")
        p = message["payload"]
        old = self.pinkies.get(message["source"])
        if (
            old
            and p["boot_version"] == old["boot_version"]
            and p["sequence"] <= old["sequence"]
        ):
            return
        retired = self.retired_boots.setdefault(message["source"], set())
        if p["boot_version"] in retired:
            return
        if old and p["boot_version"] != old["boot_version"]:
            retired.add(old["boot_version"])
        self.pinkies[message["source"]] = deepcopy(p)

    def _event(self, command, status, result=None, reason=None):
        record = self.data["commands"][command["command_id"]]
        sequence = record.get("sequence", 0) + 1
        payload = {
            "boot_version": self.boot_version,
            "observed_at": _now(),
            "sequence": sequence,
            "status": status,
            "result": result or {},
        }
        if reason is not None:
            payload["reason"] = (
                re.sub(r"[^A-Za-z0-9_.:-]", "_", str(reason))[:128] or "unknown_error"
            )
        event = build_message(
            "command_status",
            command["target"],
            self.control,
            payload,
            command_id=command["command_id"],
        )
        record.update(sequence=sequence, status=status, last_event=event)
        self.data["outbox"].append(event)
        self._save()
        return event

    def _pinky(self, command):
        p = command["payload"]
        if command["type"] == "arm_action_command":
            place = p.get("source_location", {})
            if place.get("kind") != "pinky":
                place = p.get("destination_location", {})
            return place.get("robot_id")
        return p.get("pinky_id")

    def _visit(self, command):
        p = command["payload"]
        if p.get("operation") == "return_home":
            return
        pinky = self._pinky(command)
        seen = self.pinkies.get(pinky)
        if not seen or not _fresh(seen.get("observed_at"), self.freshness_s):
            raise ValueError("pinky_observation_stale")
        s = seen["state"]
        if (
            seen["boot_version"] != p["expected_pinky_boot_version"]
            or s.get("arrival_command_id") != p["arrival_command_id"]
            or s.get("node") != NODES[command["target"]]
            or s.get("stopped") is not True
            or s.get("line_enabled") is not False
            or s.get("drive_owner") != "none"
            or seen.get("active_command_id") is not None
            or s.get("mode") != "line"
            or s.get("fault") is not None
        ):
            raise ValueError("pinky_visit_not_confirmed")
        observed = self.alignment(command)
        if (
            not isinstance(observed, dict)
            or observed.get("alignment") != "confirmed"
            or not _fresh(observed.get("observed_at"), self.freshness_s)
            or observed.get("pinky_id") != pinky
            or observed.get("pinky_boot_version") != seen["boot_version"]
            or observed.get("arrival_command_id") != p["arrival_command_id"]
        ):
            raise ValueError("docking_alignment_unconfirmed")

    def _task(self, command):
        target, p, kind = command["target"], command["payload"], command["type"]
        if p["expected_boot_version"] != self.boot_version:
            raise ValueError("boot_version_mismatch")
        if kind == "shelf_transfer_command":
            if target != "shelf-01" or p["slot_id"] not in SLOTS[target]:
                raise ValueError("cooperative_slot_mismatch")
            if {a["robot_id"] for a in p["arms"]} != {"omx1", "omx2"} or any(
                a["expected_boot_version"] != self.boot_version for a in p["arms"]
            ):
                raise ValueError("participating_arms_mismatch")
            arms, operation, slot = ["omx1", "omx2"], p["operation"], p["slot_id"]
        elif kind == "arm_action_command":
            if target not in {"omx1", "omx2"}:
                raise ValueError("invalid_independent_arm")
            arms, operation, slot = [target], p["operation"], None
            if operation == "transfer":
                src, dest = p["source_location"], p["destination_location"]
                if src["kind"] == "pinky" and dest["kind"] == "shelf":
                    operation, slot = "store", dest["slot_id"]
                elif src["kind"] == "shelf" and dest["kind"] == "pinky":
                    operation, slot = "retrieve", src["slot_id"]
                else:
                    raise ValueError("invalid_transfer_locations")
                if slot not in SLOTS[target]:
                    raise ValueError("independent_slot_mismatch")
        elif kind == "warehouse_load_command" and target == "omx3":
            arms, operation, slot = ["omx3"], "load", None
        else:
            raise ValueError("unsupported_omx_command")
        task = {
            **deepcopy(p),
            "command_id": command["command_id"],
            "expires_at": command["expires_at"],
            "robot_ids": arms,
            "assigned_arm_id": target if len(arms) == 1 else None,
            "operation": {
                "store": "INBOUND",
                "retrieve": "OUTBOUND",
                "load": "LOAD",
                "return_home": "RETURN_HOME",
            }[operation],
        }
        if slot is not None:
            task["slot_id"] = slot
        return task

    def _ready(self, target, arms):
        ready = self.executors[target].readiness()
        if ready.get("armed") is not True or ready.get("busy") or ready.get("error"):
            raise ValueError("executor_not_ready")
        if any(not _flags(ready.get("arms", {}).get(a))["ready"] for a in arms):
            raise ValueError("arm_not_safely_home")

    def state(self, target):
        self.state_sequence[target] = self.state_sequence.get(target, 0) + 1
        try:
            ready = self.executors[target].readiness()
        except Exception:  # noqa: BLE001 - unreadable device cannot be ready
            ready = {"error": "executor_unavailable", "arms": {}}
        arm_names = ["omx1", "omx2"] if target == "shelf-01" else [target]
        arms = [
            {
                "robot_id": a,
                "boot_version": self.boot_version,
                "active_command_id": self.data["active"]
                or self.data.get("recovery_command_id"),
                **_flags(ready.get("arms", {}).get(a)),
            }
            for a in arm_names
        ]
        fault = self.data["hold"] or ready.get("error")
        ok = (
            not fault
            and not self.data["active"]
            and ready.get("armed") is True
            and all(a["ready"] for a in arms)
        )
        if target == "shelf-01":
            docking = None
            for record in reversed(list(self.data["commands"].values())):
                if record["command"]["target"] == target:
                    docking = self.alignment(record["command"])
                    break
            state = {
                "stopped": all(a["stopped"] for a in arms),
                "ready": bool(ok),
                "fault": fault,
                "arms": arms,
                "docking": docking,
            }
        else:
            state = {
                k: v
                for k, v in arms[0].items()
                if k not in {"robot_id", "boot_version", "active_command_id"}
            }
            state.update(ready=bool(ok), fault=fault or state["fault"])
        return build_message(
            "robot_state",
            target,
            self.control,
            {
                "boot_version": self.boot_version,
                "observed_at": _now(),
                "sequence": self.state_sequence[target],
                "active_command_id": self.data["active"]
                or self.data.get("recovery_command_id"),
                "state": state,
            },
        )

    def handle(self, message, *, qos=1, retained=False):
        if self.failed:
            raise RuntimeError("host journal unavailable")
        validate(message)
        if (
            qos != 1
            or retained
            or message["source"] != self.control
            or message["target"] not in self.executors
        ):
            raise ValueError("untrusted OMX command")
        kind, p = message["type"], message["payload"]
        if kind == "event_stored":
            self.data["outbox"] = [
                e
                for e in self.data["outbox"]
                if not (
                    e["message_id"] == p["acked_message_id"]
                    and e["source"] == message["target"]
                )
            ]
            self._save()
            return []
        if kind == "robot_state_request":
            if p:
                active = self.data["commands"].get(self.data["active"])
                if (
                    active
                    and not self.data.get("stop")
                    and active["command"]["target"] == message["target"]
                    and p["expected_boot_version"] == self.boot_version
                    and p["keep_running_command_id"] == self.data["active"]
                    and _fresh(message["sent_at"], 1.0)
                    and datetime.fromisoformat(message["expires_at"])
                    > datetime.now(UTC)
                    and message["message_id"] not in self.renewals
                ):
                    self.renewals.add(message["message_id"])
                    self.deadline = time.monotonic() + self.watchdog_s
            return [self.state(message["target"])]
        command_id = message.get("command_id")
        if not command_id:
            raise ValueError("expected physical command")
        previous = self.data["commands"].get(command_id)
        if previous:
            if previous["command"] != message:
                raise ValueError("immutable_command_conflict")
            return (
                [deepcopy(previous["last_event"])] if previous.get("last_event") else []
            )
        self.data["commands"][command_id] = {"command": deepcopy(message)}
        self._save()
        try:
            if datetime.fromisoformat(message["expires_at"]) <= datetime.now(UTC):
                raise ValueError("expired_command")
            if p["expected_boot_version"] != self.boot_version:
                raise ValueError("boot_version_mismatch")
            if kind == "robot_stop_command":
                return self._stop(message)
            if self.data["hold"] or self.data["active"]:
                raise ValueError("workcell_held_or_busy")
            expected = getattr(self.executors[message["target"]], "versions", {})
            if any(
                not expected.get(k) or p.get(k) != expected[k]
                for k in ("model_version", "calibration_version")
            ):
                raise ValueError("executor_version_mismatch")
            task = self._task(message)
            self._visit(message)
            self._ready(message["target"], task["robot_ids"])
            check = getattr(self.executors[message["target"]], "check", None)
            if check:
                check(task)
            if datetime.fromisoformat(message["expires_at"]) <= datetime.now(UTC):
                raise ValueError("expired_command")
        except (ValueError, KeyError) as exc:
            return [self._event(message, "rejected", reason=str(exc))]
        self.data["active"] = command_id
        accepted = self._event(message, "accepted")
        # Durable acceptance may block too; no device has started yet.
        if datetime.fromisoformat(message["expires_at"]) <= datetime.now(UTC):
            self.data["active"] = None
            return [
                accepted,
                self._event(message, "rejected", reason="expired_command"),
            ]
        self.deadline = time.monotonic() + self.watchdog_s
        self.renewals.clear()
        self.last_progress = None
        try:
            self.executors[message["target"]].start(task)
        except Exception:  # noqa: BLE001 - uncertain device start must hold
            return [accepted, self._fail(message, "executor_start_failed")]
        return [accepted]

    def _fail(self, command, reason):
        self.data["hold"] = str(reason)
        # A failed outcome does not prove that the device stopped; retain its stop target.
        self.data["recovery_command_id"] = command["command_id"]
        abort = getattr(self.executors[command["target"]], "abort", None)
        if abort:
            try:
                abort()
            except Exception:
                log.exception("Device abort failed; physical stop remains unconfirmed")
        self.data["active"] = None
        pending_stop = self.data.pop("stop", None)
        if pending_stop:
            self._event(pending_stop, "failed", reason=reason)
        if self.data["commands"][command["command_id"]].get("status") in TERMINAL:
            self._save()
            return deepcopy(self.data["commands"][command["command_id"]]["last_event"])
        return self._event(command, "failed", reason=reason)

    def _stop(self, stop):
        if self.data.get("stop"):
            return [self._event(stop, "rejected", reason="stop_already_pending")]
        active = self.data["commands"].get(
            self.data["active"] or self.data.get("recovery_command_id")
        )
        if (
            not active
            or active["command"]["command_id"] != stop["payload"]["target_command_id"]
        ):
            return [self._event(stop, "rejected", reason="target_command_not_active")]
        original = active["command"]
        involved = {original["target"]}
        if original["target"] == "shelf-01":
            involved.update(("omx1", "omx2"))
        if stop["target"] not in involved:
            return [self._event(stop, "rejected", reason="wrong_stop_target")]
        executor = self.executors[original["target"]]
        abort = getattr(executor, "abort", None)
        if abort is None:
            return [
                self._fail(original, "physical_stop_unavailable"),
                self._event(stop, "failed", reason="physical_stop_unavailable"),
            ]
        self.data["active"] = original["command_id"]
        self.data["hold"] = "operator_stop_requires_reconciliation"
        self.data["stop"] = deepcopy(stop)
        accepted = self._event(stop, "accepted")
        try:
            abort()
        except Exception:  # noqa: BLE001 - abort errors leave the physical result unknown
            self.data.pop("stop")
            return [
                accepted,
                self._fail(original, "physical_stop_failed"),
                self._event(stop, "failed", reason="physical_stop_failed"),
            ]
        self.stop_deadline = time.monotonic() + self.watchdog_s
        self._save()
        return [accepted]

    def poll(self):
        command_id = self.data["active"]
        if not command_id or self.failed:
            return []
        command = self.data["commands"][command_id]["command"]
        executor = self.executors[command["target"]]
        try:
            observed = executor.observe()
            if self.data.get("stop"):
                if (
                    observed.get("state") == "stopped"
                    and observed.get("stopped") is True
                ):
                    stop = self.data.pop("stop")
                    self.data["active"] = None
                    self.data.pop("recovery_command_id", None)
                    events = []
                    if self.data["commands"][command_id].get("status") not in TERMINAL:
                        events.append(
                            self._event(command, "interrupted", reason="operator_stop")
                        )
                    events.append(self._event(stop, "completed", {"stopped": True}))
                    return events
                if self.stop_deadline and time.monotonic() > self.stop_deadline:
                    stop = self.data.pop("stop")
                    return [
                        self._fail(command, "physical_stop_not_confirmed"),
                        self._event(
                            stop, "failed", reason="physical_stop_not_confirmed"
                        ),
                    ]
                return []
            self._visit(command)
            if self.deadline is None or time.monotonic() > self.deadline:
                return [self._fail(command, "control_watchdog_expired")]
            if observed.get("state") == "failed":
                return [self._fail(command, observed.get("error", "executor_failed"))]
            if observed.get("state") == "running":
                if observed.get("step") == self.last_progress:
                    return []
                self.last_progress = observed.get("step")
                return [self._event(command, "in_progress")]
            if observed.get("state") != "finished":
                return []
            task = self._task(command)
            arms = [
                {
                    "robot_id": a,
                    "boot_version": self.boot_version,
                    **_flags(observed.get("arms", {}).get(a)),
                }
                for a in task["robot_ids"]
            ]
            if not all(a["ready"] for a in arms):
                return [self._fail(command, "participating_arm_not_returned")]
            p = command["payload"]
            if p.get("operation") == "return_home":
                result = {
                    k: arms[0][k]
                    for k in (
                        "stopped",
                        "home_reached",
                        "holding_state",
                        "holding_package_id",
                    )
                }
            else:
                if (
                    observed.get("placement_confirmed") is not True
                    or observed.get("release_confirmed") is not True
                ):
                    return [self._fail(command, "placement_or_release_unconfirmed")]
                result = {
                    "package_id": p["package_id"],
                    "pinky_id": self._pinky(command),
                    "arrival_command_id": p["arrival_command_id"],
                    "placement_confirmed": True,
                    "release_confirmed": True,
                    "arms": [
                        {k: v for k, v in a.items() if k not in {"ready", "fault"}}
                        for a in arms
                    ],
                }
                if task.get("slot_id"):
                    result["slot_id"] = task["slot_id"]
                if command["target"] == "omx3":
                    size = observed.get("package_size", "unknown")
                    result["package_size"] = (
                        size if size in {"small", "large"} else "unknown"
                    )
            self.data["active"] = None
            return [self._event(command, "completed", result)]
        except Exception as exc:
            if self.failed:
                raise
            return [self._fail(command, str(exc) or "executor_observation_failed")]
