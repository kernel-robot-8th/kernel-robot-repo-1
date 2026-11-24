"""shelf-01 Host for the field MQTT contract v2 (profile roport-control-v2, Fleet control).

Same device side as the v1 test Host (cooperative.py): one task at a time through a local
executor, the journal written before acting, results kept in an outbox until control's
event_stored. Only the MQTT side differs (Confluence "메세지 규약 정리", interfaces/mqtt):

- topics roport/workcell/shelf-01/{cmd,event,ack} and roport/robot/{omx1,omx2}/{cmd,event,ack};
- shelf_transfer_command (store = Pinky → shelf, retrieve = shelf → Pinky) for the B column with
  both arms; arm_action_command (one arm, columns A/C) is refused: two-arm missions only for now;
- robot_stop_command stops the running task: the original command ends interrupted and the stop
  completed only once the run ended AND a readiness sample taken after the stop shows the arms
  idle and read (else physical_stop_not_confirmed); the workcell then holds (contract);
- keep-running: a running task needs control's robot_state_request with keep_running_command_id
  (same boot, fresh, new message) at least every WATCHDOG_S, or the arms are stopped and the task
  fails control_watchdog_expired with a HOLD; local_safety() runs it, also without a broker;
- hold_release_command releases a HOLD with the operator's checks, except a HOLD only a person
  at the shelf may release (emergency stop, unknown result, unconfirmed stop);
- robot_state for shelf-01, omx1 and omx2 answers every robot_state_request (and a 1 Hz tick).

What the Host can observe is reported, nothing else (unknown stays unknown):
- Pinky docking: the Pinky's own robot_state (fresh observed_at, current boot, stopped at N11
  with the arrival named in the transfer). No workcell docking sensor is required (TK 2026-11-23:
  the docking_alignment check was dropped); docking.alignment is reported "unknown".
- placement/release: the executor's completion evidence (aligner or placement judge for store,
  the tray check for retrieve) with both arms home and grippers released.
- model/calibration: a transfer runs only when the executor's route declares exactly the
  command's model_version and calibration_version.
No MQTT, HTTP or motor access here; the executor owns the device.
"""

import json
import logging
import re
import threading
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
# Freshness limits agreed with control on 2026-11-23 (TK: control moved its limits to 8 s; the
# spec's 3 s were proposals to be fixed after field measurement). Change both sides together.
FRESH_S = 8.0
SENT_AGE_S = (
    -0.5,
    FRESH_S,
)  # a new transfer's sent_at against this PC's clock
OBSERVATION_MAX_AGE_S = (
    FRESH_S  # Pinky state / readiness sample older than this proves nothing
)
STOP_TIMEOUT_S = 30.0  # the panel's safe stop lands the arms within a few seconds
# Stop observed: two separate bus reads of both arms after the stop request, every joint within
# STILL_TOL (panel joint units) of the previous read (the panel reads the bus about every 10 s).
STILL_TOL = 1.0
WATCHDOG_S = FRESH_S  # keep-running renewals: stop/HOLD after this long without one
KEEP_RUNNING_SENT_S = 1.0  # a renewal's sent_at age (field_host.py alike)
OBSERVED_FUTURE_S = (
    1.0  # a Pinky observed_at this far ahead of this PC's clock still counts
)
# A HOLD only a person at the shelf releases (panel [HOLD 해제]), never hold_release_command.
LOCAL_RELEASE = {
    "panel_emergency_stop",
    "panel_killed",
    "result_unavailable",
    "host_restarted_outcome_unknown",
    "physical_stop_failed",
    "physical_stop_not_confirmed",
}
ACCEPT_SAMPLE_MAX_AGE_S = (
    2.0  # a sampler reading this recent stands for "now" when accepting
)
ACCEPT_WAIT_S = (
    3.0  # a transfer waits this long for a recent readiness sample, then is rejected
)
START_INLINE_S = (
    0.2  # wait this long in handle() for a start before leaving it to poll()
)
TERMINAL = {"completed", "rejected", "failed", "interrupted"}
_REASON = re.compile(r"[^A-Za-z0-9_.:-]+")


def _iso(at: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if at is None else at, UTC).isoformat(
        timespec="milliseconds"
    )


class NotYet(Exception):
    """No recent readiness sample: decide the transfer from poll() once one arrives."""


def _reason(code) -> str:
    text = _REASON.sub("_", str(code or "unknown")).strip("_").lower()
    return text[:128] or "unknown"


def _read_time(value) -> float | None:
    """The panel's bus read time (/api/arms "at": ISO, local time when no zone; same PC) as epoch
    seconds, or None when missing or unreadable."""
    if not isinstance(value, str):
        return None
    try:
        at = datetime.fromisoformat(value)
    except ValueError:
        return None
    return at.timestamp()  # a naive time is this PC's local time, as the panel wrote it


def _fresh(value, max_age_s: float, future_s: float = OBSERVED_FUTURE_S) -> bool:
    try:
        age = (datetime.now(UTC) - field_wire.timestamp(value)).total_seconds()
    except (TypeError, ValueError):
        return False
    return -future_s <= age <= max_age_s


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
        report_placement: bool = False,
        readiness=None,
        watchdog_s: float = WATCHDOG_S,
    ):
        self.journal, self.executor = journal, executor
        self.control = control
        self.require_armed, self.require_pinky = require_armed, require_pinky
        # Shelf store reported completed the moment the box is placed and both hands are clear
        # (the executor's "placed", panel hand-over judge); the alignment then runs on without
        # further reports for this command (TK 2026-11-24, control asked to accept it).
        self.report_placement = report_placement
        self.watchdog_s, self.stop_timeout_s = watchdog_s, STOP_TIMEOUT_S
        self._sample = readiness or (lambda: (self._read(), time.time()))
        # with a sampler thread a stale reading never turns into a panel read in the MQTT loop
        # (#156 review): the transfer waits (in memory, nothing answered) for the next sample
        self._sampled = readiness is not None
        self._waiting: dict | None = None
        self.boot_version = "boot-" + str(uuid4())
        self.failed = False
        self.state_sequence = 0
        self.pinkies: dict[str, dict] = {}
        self.retired_boots: dict[str, set[str]] = {}
        self._deadline: float | None = None  # keep-running watchdog of the active task
        # The executor's start runs off the MQTT loop (a panel start takes seconds): while it waits the
        # loop keeps reading Pinky states and renewals and publishing robot_state (2026-11-23 field
        # run: a 3 s blocking start made the Host see its own Pinky observation as stale and stop).
        self._starting: dict | None = (
            None  # {"command_id", "thread", "error", "done_at"}
        )
        self._renewals: set[str] = set()
        self._stop_wall: float | None = None  # wall time of the stop request
        # (bus read time, joints) of reads the panel finished after the stop request
        self._stop_reads: list[tuple[float, dict]] = []
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
            "hold_reason": data.get("hold_reason"),
            "local_release_only": data.get("local_release_only", False),
            "stop": None,  # a stop never survives a restart: the new boot re-checks
        }
        self._stop_since: float | None = None
        stop, active = data.get("stop"), self._data["active"]
        if active is not None:
            # A previous process left a task running: its physical result is unknown.
            self._end(
                self._data["commands"][active],
                "failed",
                "host_restarted_outcome_unknown",
            )
        if stop and stop["command_id"] in self._data["commands"]:
            record = self._data["commands"][stop["command_id"]]
            if record.get("status") not in TERMINAL:
                self._end(
                    record,
                    "failed",
                    "host_restarted_outcome_unknown",
                    hold=False,
                    keep_active=True,
                )
        if not data or stop or active is not None:
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
        self._data.update(hold=None, hold_reason=None, local_release_only=False)
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
            "error_code": self._data.get("hold_reason")
            or (last or {}).get("payload", {}).get("reason")
            or "result_unknown",
            "local_release_only": self._data.get("local_release_only") is True,
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
        """A Pinky's robot_state (its own topic, QoS1, not retained: checked by the caller).

        Only states addressed to this control; a boot the Pinky already replaced never comes back.
        """
        field_wire.validate(message)
        if (
            message["type"] != "robot_state"
            or message["source"] not in PINKIES
            or message["target"] != self.control
        ):
            raise ValueError("not a Pinky state for this control")
        p = message["payload"]
        old = self.pinkies.get(message["source"])
        retired = self.retired_boots.setdefault(message["source"], set())
        if p["boot_version"] in retired:
            return
        if (
            old
            and old["boot_version"] == p["boot_version"]
            and p["sequence"] <= old["sequence"]
        ):
            return
        if old and old["boot_version"] != p["boot_version"]:
            retired.add(old["boot_version"])
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
            self._keep_running(message, endpoint)
            return [self.state(endpoint)]
        if "command_id" not in message:
            raise ValueError(f"{kind} is not a command")
        waiting = self._waiting["record"] if self._waiting else None
        if (
            waiting is not None
            and waiting["command"]["command_id"] == message["command_id"]
        ):
            if waiting["fingerprint"] == _fingerprint(message):
                return []  # still waiting for a readiness sample: decided from poll()
            raise ValueError(
                "idempotency_conflict: command_id reused with another body"
            )
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
            "timing": {},  # wall-clock marks of this command (journal only, see _mark)
        }
        self._mark(record, "sent", field_wire.timestamp(message["sent_at"]).timestamp())
        self._mark(record, "received")
        self._data["commands"][message["command_id"]] = record
        try:
            if datetime.now(UTC) >= field_wire.timestamp(message["expires_at"]):
                raise Rejected("expired_command")
            if message["payload"]["expected_boot_version"] != self.boot_version:
                raise Rejected("boot_version_mismatch")
            if (
                kind == "robot_stop_command"
            ):  # a late stop still stops (no sent_at check)
                return self._stop_request(record)
            if kind == "arm_action_command":
                raise Rejected("unsupported_one_arm_command")  # two-arm missions only
            if (
                kind not in {"shelf_transfer_command", "hold_release_command"}
                or endpoint != WORKCELL
            ):
                raise Rejected("unsupported_command")
            age = (
                datetime.now(UTC) - field_wire.timestamp(message["sent_at"])
            ).total_seconds()
            if not SENT_AGE_S[0] <= age <= SENT_AGE_S[1]:
                raise Rejected("stale_message")
            if kind == "hold_release_command":
                return self._hold_release(record)
            task = self._check_transfer(message)
        except NotYet:
            return self._defer(record)
        except Rejected as exc:
            return self._status(record, "rejected", reason=_reason(exc.args[0]))
        except ValueError as exc:
            record["detail"] = str(exc)[:200]
            return self._status(record, "rejected", reason="invalid_command")
        return self._accept(record, task)

    def _defer(self, record: dict) -> list[dict]:
        command_id = record["command"]["command_id"]
        del self._data["commands"][
            command_id
        ]  # nothing journaled or answered for it yet
        self._waiting = {"record": record, "since": time.monotonic()}
        log.info(
            "%s: readiness sample not recent; deciding on the next one", command_id
        )
        return []

    def _retry_waiting(self) -> list[dict]:
        waiting, self._waiting = self._waiting, None
        record = waiting["record"]
        message = record["command"]
        self._data["commands"][message["command_id"]] = record
        try:
            if datetime.now(UTC) >= field_wire.timestamp(message["expires_at"]):
                raise Rejected("expired_command")
            task = self._check_transfer(message)
        except NotYet:
            if time.monotonic() - waiting["since"] < ACCEPT_WAIT_S:
                del self._data["commands"][message["command_id"]]
                self._waiting = waiting
                return []
            return self._status(record, "rejected", reason="executor_state_stale")
        except Rejected as exc:
            return self._status(record, "rejected", reason=_reason(exc.args[0]))
        except ValueError as exc:
            record["detail"] = str(exc)[:200]
            return self._status(record, "rejected", reason="invalid_command")
        return self._accept(record, task)

    def _accept(self, record: dict, task: dict) -> list[dict]:
        message = record["command"]
        self._mark(record, "checked")
        record["task"] = task
        record["arrival_consumed"] = True  # one transfer per Pinky arrival (contract)
        self._data["active"] = message["command_id"]
        self._deadline = time.monotonic() + self.watchdog_s
        self._renewals.clear()
        self._stop_since = self._stop_wall = None
        events = self._status(record, "accepted")  # saved before the executor is called
        self._mark(record, "accepted")
        if datetime.now(UTC) >= field_wire.timestamp(message["expires_at"]):
            return events + self._end(record, "failed", "expired_command", hold=False)
        self._begin_start(record, task)
        # A quick start (or refusal) is settled here; a slow one is followed up by poll()
        self._starting["thread"].join(START_INLINE_S)
        return events + (self._start_outcome(record) or [])

    # --- executor start off the MQTT loop -------------------------------------------------
    def _begin_start(self, record: dict, task: dict) -> None:
        starting = {
            "command_id": record["command"]["command_id"],
            "error": None,
            "done_at": None,
        }

        def run() -> None:
            try:
                self.executor.start(task)
            except BaseException as exc:  # noqa: BLE001 - handed back to the loop in poll()
                starting["error"] = exc
            finally:
                starting["done_at"] = time.time()

        self._mark(record, "start_sent")
        starting["thread"] = threading.Thread(
            target=run, name="executor-start", daemon=True
        )
        self._starting = starting
        starting["thread"].start()

    def _start_outcome(self, record: dict) -> list[dict] | None:
        """None while the start is still on its way; [] once started; events if it was refused."""
        starting = self._starting
        if (
            starting is None
            or starting["command_id"] != record["command"]["command_id"]
        ):
            return []
        if starting["thread"].is_alive():
            return None
        self._starting = None
        self._mark(record, "start_done", starting["done_at"])
        exc = starting["error"]
        if exc is None:
            return []
        if isinstance(
            exc, StartRefused
        ):  # refused before any motion, with its own reason
            return self._end(
                record, "failed", _reason(exc.code), hold=exc.hold, detail=exc.detail
            )
        return self._end(record, "failed", "executor_start_failed", detail=str(exc))

    def _late_start(self) -> None:
        """A start that returned after its task had already ended (stopped, watchdog, Pinky left):
        the run it may have begun gets the stop again, so nothing keeps moving unasked."""
        starting = self._starting
        if starting is None or starting["thread"].is_alive():
            return
        self._starting = None
        if starting["error"] is None:
            log.warning(
                "start of %s returned after the task ended; stopping again",
                starting["command_id"],
            )
            try:
                self.executor.stop()
            except Exception as exc:  # noqa: BLE001
                log.warning("executor stop failed: %s", exc)

    # --- timing ---------------------------------------------------------------------------
    def _mark(self, record: dict, name: str, at: float | None = None) -> None:
        """First time this command reached `name` (unix s). Logged with the time since received."""
        timing = record.setdefault("timing", {})
        if name in timing:
            return
        timing[name] = round(time.time() if at is None else at, 3)
        base = timing.get("received")
        if base is not None and name not in {"sent", "received"}:
            log.info(
                "timing %s %s +%.2fs",
                record["command"]["command_id"][:8],
                name,
                timing[name] - base,
            )

    def poll(self) -> list[dict]:
        """Executor observations of the active task → in_progress / completed / failed."""
        self._usable()
        if self._waiting is not None:  # nothing is active while a transfer waits
            return self._retry_waiting()
        command_id = self._data["active"]
        if command_id is None:
            self._late_start()
            return self._settle_stop()
        events = self.local_safety()
        if self._data["active"] is None:
            self._late_start()
            return events
        record = self._data["commands"][command_id]
        started = self._start_outcome(record)
        if started is None:  # the panel is still taking the start: keep the loop free
            return events + self._settle_stop()
        if started:
            return events + started
        if record.get(
            "run_ended"
        ):  # stopped run over: only the stop observation is missing
            return events + self._settle_stop()
        try:
            seen = self.executor.observe()
        except Exception:  # noqa: BLE001 - keep waiting; the device keeps its own safety
            return events + self._settle_stop()
        state = seen.get("state")
        if (
            state == "placed"
            and self.report_placement
            and record["task"]["operation"] == "INBOUND"
            and not record.get("placed_reported")
            and self._data["stop"]
            is None  # stopping: the stop's own result only (#156 review)
        ):
            events += self._report_placed(record, seen)
        if state in {"running", "placed"}:
            step = seen.get("step") if isinstance(seen.get("step"), str) else state
            self._mark(record, "step:" + step)
            if step != record["last_step"]:
                record["last_step"] = step
                if not record.get(
                    "placed_reported"
                ):  # completed is control's last word
                    events += self._status(record, "in_progress")
            return events + self._settle_stop()
        if state not in {"finished", "failed"}:
            return events + self._settle_stop()
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
        if active is not None and stale:
            # During a task the panel holds the arms (starting the run, loading the model), so a
            # slow arm read is expected, not a fault (2026-11-23 23:06 field run: Fleet held on a
            # participant fault while the panel started). Report what is known now - a task is
            # running, the arms are unknown - with the time of this report, so control keeps
            # receiving a current state; a real fault still ends the task through observe().
            busy, observed = True, time.time()
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
        elif active is not None:
            fault = (
                None  # a running task reports through command_status, not a state fault
            )
        elif stale:
            fault = "observation_stale"
        elif ready.get("error"):
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

    # --- keep-running and local safety ---------------------------------------------------
    def _keep_running(self, message: dict, endpoint: str) -> None:
        """A renewal counts only for this boot's active task, fresh and seen once (contract)."""
        p = message["payload"]
        if not p:
            return  # a plain state query never keeps the arms moving
        active = self._data["active"]
        if (
            active is None
            or self._data["stop"] is not None
            or endpoint != WORKCELL
            or p["expected_boot_version"] != self.boot_version
            or p["keep_running_command_id"] != active
            or not _fresh(message["sent_at"], KEEP_RUNNING_SENT_S)
            or field_wire.timestamp(message["expires_at"]) <= datetime.now(UTC)
            or message["message_id"] in self._renewals
        ):
            return
        self._renewals.add(message["message_id"])
        self._deadline = time.monotonic() + self.watchdog_s

    def local_safety(self) -> list[dict]:
        """Stop the arms of the active task without control's renewal or a confirmed Pinky visit.

        Needs no broker (the caller runs it while reconnecting too); the failure waits in the
        outbox. The task ends at once with a HOLD; the stop itself is the executor's.
        """
        self._usable()
        command_id = self._data["active"]
        if command_id is None or self._data["stop"] is not None:
            return []
        record = self._data["commands"][command_id]
        # #156 review: the renewal watchdog stays on through the alignment (control keeps the
        # command open until the final report); the Pinky check too, until control has stored
        # the placement report - only then may the Pinky leave while the arms still align
        if self._deadline is None or time.monotonic() > self._deadline:
            reason = "control_watchdog_expired"
        elif self.require_pinky and not record.get("placed_stored"):
            reason = self._visit_lost(record["command"])
        else:
            reason = None
        if reason is None:
            return []
        log.warning("%s: %s; stopping the arms", command_id, reason)
        self._request_stop()
        return self._end(record, "failed", reason)

    # --- transfer -------------------------------------------------------------------------
    def _check_transfer(self, message: dict) -> dict:
        p = message["payload"]
        if any(a["expected_boot_version"] != self.boot_version for a in p["arms"]):
            raise Rejected("participating_arms_mismatch")
        if p["slot_id"] not in SLOTS:
            raise Rejected("cooperative_slot_mismatch")
        if self._data["hold"] is not None:
            raise Rejected("host_hold")
        if (
            self._data["active"] is not None
            or self._data["stop"] is not None
            or self._waiting is not None
        ):
            raise Rejected("resource_busy")
        for other in self._data["commands"].values():
            q = other["command"]["payload"]
            if (
                other.get("arrival_consumed")
                and q.get("pinky_id") == p["pinky_id"]
                and q.get("arrival_command_id") == p["arrival_command_id"]
            ):
                raise Rejected("arrival_already_used")
        ready = self._recent_ready()  # never a slow panel call inside the MQTT loop
        if ready["error"]:
            raise Rejected("executor_error")
        if ready["busy"]:
            raise Rejected("executor_busy")
        if self.require_armed and ready["armed"] is not True:
            raise Rejected("not_armed")
        if not all(_home(a) for a in _arms(ready.get("arms")).values()):
            raise Rejected("arm_not_safely_home")
        if self.require_pinky:
            self._check_pinky(message)
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
        models_ready = getattr(self.executor, "models_ready", None)
        if self._sampled and callable(models_ready) and not models_ready():
            # #156 review: the panel's first model list is still loading off the loop (it starts
            # it); the check would otherwise wait for the panel here, inside the MQTT loop
            raise NotYet
        check = getattr(self.executor, "check", None)
        if check is not None:
            check(
                task
            )  # executor refusal (no route/model for this box), Rejected(code)
        # Only the exact model/calibration pair the route was set up with (#135 review).
        versions = getattr(self.executor, "versions", None)
        expected = versions(task) if callable(versions) else None
        if not isinstance(expected, dict) or any(
            not expected.get(k) or p[k] != expected[k]
            for k in ("model_version", "calibration_version")
        ):
            raise Rejected("executor_version_mismatch")
        return task

    def _pinky_seen(self, pinky: str) -> dict | None:
        """The Pinky's latest state, if received and observed (its own clock) within 3 s."""
        seen = self.pinkies.get(pinky)
        if (
            not seen
            or time.monotonic() - seen["received"] > OBSERVATION_MAX_AGE_S
            or not _fresh(seen["observed_at"], OBSERVATION_MAX_AGE_S)
        ):
            return None
        return seen

    def _check_pinky(self, message: dict) -> None:
        p = message["payload"]
        seen = self._pinky_seen(p["pinky_id"])
        if seen is None:
            raise Rejected("pinky_observation_stale")
        if seen["boot_version"] != p["expected_pinky_boot_version"]:
            raise Rejected("pinky_boot_mismatch")
        if not self._parked(seen, p["arrival_command_id"]):
            raise Rejected("pinky_visit_not_confirmed")

    def _visit_lost(self, command: dict) -> str | None:
        """Why the Pinky visit of a running transfer no longer holds, or None."""
        p = command["payload"]
        seen = self._pinky_seen(p["pinky_id"])
        if seen is None:
            return "pinky_observation_stale"  # a lost state cannot show the Pinky still there
        if seen["boot_version"] != p["expected_pinky_boot_version"] or not self._parked(
            seen, p["arrival_command_id"]
        ):
            return "pinky_left"
        return None

    @staticmethod
    def _parked(seen: dict, arrival: str | None) -> bool:
        s = seen["state"]
        return (
            s.get("node") == NODE
            and s.get("mode") == "line"
            and s.get("stopped") is True
            and s.get("line_enabled") is False
            and s.get("drive_owner") == "none"
            and s.get("fault") is None
            and seen["active_command_id"] is None
            and (arrival is None or s.get("arrival_command_id") == arrival)
        )

    def _docking(self) -> dict | None:
        active = self._data["active"]
        record = self._data["commands"][active] if active else None
        for pinky in [record["task"]["pinky_id"]] if record else PINKIES:
            seen = self._pinky_seen(pinky)
            if seen and self._parked(seen, None):
                return {
                    "pinky_id": pinky,
                    "pinky_boot_version": seen["boot_version"],
                    "arrival_command_id": seen["state"].get("arrival_command_id"),
                    "observed_at": seen["observed_at"],
                    "alignment": "unknown",  # no workcell docking sensor (check dropped)
                }
        return None

    def _terminal(self, record: dict, seen: dict) -> list[dict]:
        arms = _arms(seen.get("arms"))
        task = record["task"]
        stop = self._data["stop"]
        detail = seen.get("detail") if isinstance(seen.get("detail"), str) else None
        if stop is not None and stop["target"] == record["command"]["command_id"]:
            # Stopped by control: the run is over, but the arms may still slow down or move to a
            # safe pose. The original command stays active until the stop is observed
            # (_settle_stop: interrupted + stop completed) or the stop times out (failed).
            record["run_ended"] = True
            if detail:
                record["detail"] = detail[:200]
            self._save()
            return self._settle_stop()
        if seen["state"] == "finished" and all(_home(a) for a in arms.values()):
            evidence = seen.get("evidence")
            if evidence in PLACED[task["operation"]]:
                # always named (#156 review): control tells an aligned load from a placed but
                # crooked one (shelf_load_not_aligned holds the slot) with or without a first report
                extra = {"evidence": evidence}
                return self._end(
                    record,
                    "completed",
                    result=extra
                    | {
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
            return self._end(
                record,
                "failed",
                "placement_or_release_unconfirmed",
                detail=f"evidence {evidence}",
            )
        settled = seen["state"] == "failed" and (
            seen.get("operator_stop") is True or seen.get("recovered") is True
        )
        code = seen.get("error") if seen["state"] == "failed" else "arm_not_returned"
        return self._end(
            record,
            "failed",
            _reason(code or "execution_failed"),
            hold=not settled,
            detail=detail,
        )

    # --- HOLD release by control (hold_release_command, #133) ----------------------------
    def _hold_release(self, record: dict) -> list[dict]:
        p = record["command"]["payload"]
        held = self._data["hold"]
        if held is None or p["hold_command_id"] != held:
            raise Rejected("hold_changed")
        if self._data.get("local_release_only"):
            raise Rejected("local_emergency_release_required")
        if not all(p["confirmed"].get(k) is True for k in ("arms", "box", "pinky")):
            raise Rejected("operator_checks_required")
        if self._data["active"] is not None or self._data["stop"] is not None:
            raise Rejected("resource_busy")
        ready = self._read()  # measured now, after the operator's checks
        if ready["error"] or ready["busy"]:
            raise Rejected("executor_busy_or_fault")
        if self.require_armed and ready["armed"] is not True:
            raise Rejected("not_armed")
        if not all(_home(a) for a in _arms(ready.get("arms")).values()):
            raise Rejected("arm_not_safely_home")
        if datetime.now(UTC) >= field_wire.timestamp(record["command"]["expires_at"]):
            raise Rejected("expired_command")
        self._data.setdefault("recoveries", []).append(
            {
                "hold_command_id": held,
                "fault": self._data.get("hold_reason"),
                "request_id": record["command"]["command_id"],
                "operator": p["operator"],
                "at": _iso(),
            }
        )
        self._data.update(hold=None, hold_reason=None, local_release_only=False)
        return self._end(
            record, "completed", result={"released_hold": held}, hold=False
        )

    # --- stop -----------------------------------------------------------------------------
    def _stop_request(self, record: dict) -> list[dict]:
        p = record["command"]["payload"]
        if self._data["stop"] is not None:
            raise Rejected("stop_already_pending")
        target = p["target_command_id"]
        if target == record["command"]["command_id"]:
            raise Rejected("invalid_stop_target")
        waiting = self._waiting["record"] if self._waiting else None
        if waiting is not None and waiting["command"]["command_id"] == target:
            # a stop for a transfer that waits for its readiness sample: it never starts
            self._waiting = None
            self._data["commands"][target] = waiting
            events = self._status(waiting, "rejected", reason="stop_requested")
            return events + self._status(
                record, "rejected", reason="target_command_not_active"
            )
        if target != self._data["active"]:
            raise Rejected("target_command_not_active")
        if not callable(getattr(self.executor, "stop", None)):
            raise Rejected("physical_stop_adapter_unavailable")
        self._data["stop"] = {
            "command_id": record["command"]["command_id"],
            "target": target,
            "reason": _reason(p["reason"]),
        }
        events = self._status(record, "accepted")
        self._request_stop()
        self._save()
        return events

    def _request_stop(self) -> None:
        if self._stop_since is None:
            self._stop_since, self._stop_wall = time.monotonic(), time.time()
            self._stop_reads = []
            try:
                self.executor.stop()
            except Exception as exc:  # noqa: BLE001 - the timeout reports the unconfirmed stop
                log.warning("executor stop failed: %s", exc)

    def _stop_confirmed(self) -> dict | None:
        """The arms seen standing still after the stop request (their latest flags), or None.

        Needs a fresh sample taken after the request with the run over (not busy, no error), both
        arms connected and safe, and two separate bus reads of both arms (`arms_read`) since the
        request whose joints all stay within STILL_TOL. busy=false with connected arms alone is
        no proof: they may still slow down or move to a safe pose (#135 review).
        """
        if self._stop_wall is None:
            return None
        try:
            ready, observed = self._sample()
        except Exception:  # noqa: BLE001 - no sample, no proof
            return None
        if observed < self._stop_wall or time.time() - observed > OBSERVATION_MAX_AGE_S:
            return None
        if ready.get("busy") is not False or ready.get("error"):
            return None
        arms = _arms(ready.get("arms"))
        if not all(a["connected"] is True and a["safe"] is True for a in arms.values()):
            return None
        read = ready.get("arms_read")
        joints = read.get("joints") if isinstance(read, dict) else None
        read_at = _read_time(read.get("at")) if isinstance(read, dict) else None
        # only a bus read the panel finished after the stop request counts (#135 review: a cached
        # read from before the stop can arrive in a fresh readiness sample)
        if (
            not isinstance(joints, dict)
            or read_at is None
            or read_at <= self._stop_wall
        ):
            return None
        if not self._stop_reads or read_at > self._stop_reads[-1][0]:
            self._stop_reads = [*self._stop_reads[-1:], (read_at, deepcopy(joints))]
        if len(self._stop_reads) < 2:
            return None
        (first_at, before), (second_at, after) = self._stop_reads
        if not first_at < second_at:  # the second must be a later read
            return None
        for name in ARMS:
            old, new = before.get(name), after.get(name)
            if not isinstance(old, dict) or not isinstance(new, dict) or not new:
                return None
            if set(old) != set(new):
                return None
            for joint, value in new.items():
                prior = old[joint]
                if not all(
                    isinstance(v, (int, float)) and not isinstance(v, bool)
                    for v in (value, prior)
                ):
                    return None
                if abs(value - prior) > STILL_TOL:
                    return None
        return arms

    def _settle_stop(self) -> list[dict]:
        """Complete a pending stop once the arms were seen still after the run ended; past
        stop_timeout_s the stop failed and the workcell holds (person at the shelf only)."""
        stop = self._data["stop"]
        if stop is None:
            return []
        active = self._data["active"]
        record = self._data["commands"][active] if active else None
        if record is None or record.get("run_ended"):
            arms = self._stop_confirmed()
            if arms is not None:
                events = []
                if record is not None:
                    events = self._end(
                        record,
                        "interrupted",
                        stop["reason"],
                        result={
                            "stopped": True,
                            "holding_state": _held(arms),
                            "holding_package_id": None,
                        },
                    )
                return events + self._finish_stop(stopped=True)
        since = self._stop_since if self._stop_since is not None else time.monotonic()
        self._stop_since = since
        if time.monotonic() - since < self.stop_timeout_s:
            return []
        events = []
        if record is not None:  # never seen standing still: the arms may still move
            events = self._end(record, "failed", "physical_stop_not_confirmed")
        self._data.update(
            hold=active or stop["target"],
            hold_reason="physical_stop_not_confirmed",
            local_release_only=True,
        )
        return events + self._finish_stop(stopped=False)

    def _finish_stop(self, *, stopped: bool) -> list[dict]:
        stop = self._data["stop"]
        if stop is None:
            return []
        self._data["stop"] = None
        self._stop_since = self._stop_wall = None
        self._stop_reads = []
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
        if record.get("task") is not None:
            self._mark(record, "ended")
        if status != "completed" and hold:
            # a person-only HOLD stays so when another one replaces it
            local = reason in LOCAL_RELEASE or (
                self._data["hold"] is not None
                and self._data.get("local_release_only") is True
            )
            self._data.update(
                hold=record["command"]["command_id"],
                hold_reason=reason or "result_unknown",
                local_release_only=local,
            )
        if detail:
            record["detail"] = detail[:200]
        record.update(
            {k: v for k, v in extra.items() if isinstance(v, str)}
        )  # journal only
        if record.get("placed_reported"):
            # second report of a store completed at placement: the alignment's outcome
            # (completed shelf_align / shelf_load_not_aligned with the arms home, or failed);
            # control sends the next shelf command after it (TK 2026-11-24)
            record["after_placed"] = {"status": status, "reason": reason}
        return self._status(record, status, reason=reason, result=result)

    def _report_placed(self, record: dict, seen: dict) -> list[dict]:
        """completed at placement: the box sits in its slot, both hands clear (judge), the arms
        are still working (the aligner takes them over), so they are reported as they are."""
        task = record["task"]
        record["placed_reported"] = True
        record["evidence"] = "shelf_placed"
        self._mark(record, "placed")
        return self._status(
            record,
            "completed",
            result={
                "package_id": task["package_id"],
                "slot_id": task["slot_id"],
                "pinky_id": task["pinky_id"],
                "arrival_command_id": task["arrival_command_id"],
                "placement_confirmed": True,
                "release_confirmed": seen.get("grasp_released") is True,
                "evidence": "shelf_placed",
                "arms": [
                    {
                        "robot_id": name,
                        "boot_version": self.boot_version,
                        "stopped": False,
                        "home_reached": False,
                        "holding_package_id": None,
                        "holding_state": "empty"
                        if seen.get("grasp_released") is True
                        else "unknown",
                    }
                    for name in ARMS
                ],
            },
        )

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
        event = next((e for e in outbox if e["message_id"] == acked), None)
        if event is None:
            return
        result = event["payload"].get("result") or {}
        if (
            event["type"] == "command_status"
            and result.get("evidence") == "shelf_placed"
        ):
            record = self._data["commands"].get(event.get("command_id"))
            if record is not None:
                record["placed_stored"] = (
                    True  # control stored it: the Pinky may now leave
                )
        self._data["outbox"] = [e for e in outbox if e["message_id"] != acked]
        self._save()

    def _recent_ready(self) -> dict:
        """The sampler's reading if it is recent; NotYet with a sampler (the caller waits for the
        next one), a direct read without one (tests, one-shot use).

        A transfer's checks run inside the MQTT loop; a direct panel read there (status plus the
        arms of every routed model) can take seconds, and while it waits no Pinky state is read
        (2026-11-23 23:41 field run: 2.8 s of checks, then pinky_observation_stale at once)."""
        try:
            sample, observed = self._sample()
        except Exception:  # noqa: BLE001 - no sample yet
            if self._sampled:
                raise NotYet from None
            return self._read()
        if not 0 <= time.time() - observed <= ACCEPT_SAMPLE_MAX_AGE_S:
            if self._sampled:
                raise NotYet
            return self._read()
        return self._normalize(sample)

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
        return self._normalize(ready)

    @staticmethod
    def _normalize(ready: dict) -> dict:
        armed = ready.get("armed")
        return {
            "armed": armed if isinstance(armed, bool) else None,
            "busy": ready.get("busy") is not False,
            "error": ready.get("error"),
            "arms": ready.get("arms"),
            "arms_read": ready.get("arms_read"),
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
