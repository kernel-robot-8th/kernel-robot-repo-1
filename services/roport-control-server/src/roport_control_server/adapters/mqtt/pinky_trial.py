"""Portable Linux Pinky trial endpoint. Inject ROS actions; this module has no ROS."""

import logging
import math
from copy import deepcopy
from datetime import UTC, datetime
from queue import Empty
from time import monotonic
from uuid import uuid4

from .journal import Journal
from .messages import check_command, check_expiry, decode, timestamp, topic
from .motion import motion_message, motion_topic
from .mqtt import MqttTransport
from .trial_wire import check_result, fresh, reply, validate_trial

LOG = logging.getLogger(__name__)


class PinkyTrialClient:
    """Call poll/report from ONE owner thread; ROS callbacks should enqueue results.

    observe() must return a fresh measured STATE. dispatch(command) starts an async
    action and returns promptly. safe_stop(reason) cancels every drive source.
    A driver-level watchdog is still mandatory if this Python process dies/stalls.
    """

    def __init__(
        self,
        config,
        *,
        site,
        control,
        robot,
        binding_id,
        journal_path,
        observe,
        validate_action,
        dispatch,
        safe_stop,
        observe_motion=None,
    ):
        if robot not in {"pinky1", "pinky2", "pinky3"}:
            raise ValueError("Pinky slot required")
        self.site, self.control, self.robot, self.binding = (
            site,
            control,
            robot,
            binding_id,
        )
        self.observe, self.dispatch, self.safe_stop = observe, dispatch, safe_stop
        self.validate_action = validate_action
        self.observe_motion = observe_motion
        self.motion_sequence, self.motion_errors, self.motion_sent = 0, 0, 0.0
        self.boot = str(uuid4())
        self.journal = Journal(journal_path)
        identity = {
            "site": site,
            "control": control,
            "robot": robot,
            "binding_id": binding_id,
        }
        if self.journal.data and self.journal.data.get("identity") != identity:
            self.journal.close()
            raise ValueError("journal belongs to another binding")
        if not self.journal.data:
            self.journal.save(
                {"identity": identity, "records": {}, "outbox": {}, "active": None}
            )
        self.halted = "recovery_required" if self.journal.data["active"] else None
        self.last_poll = monotonic()
        self.last_send = 0.0
        self.transport = MqttTransport(
            config, [topic(site, "robot", robot, c) for c in ("cmd", "ack")]
        )

    def start(self):
        if self.halted:
            self.safe_stop(self.halted)
        self.transport.start()

    def publish_motion(self):
        if self.observe_motion is None or monotonic() - self.motion_sent < 0.2:
            return
        self.motion_sent = monotonic()
        self.motion_sequence += 1
        try:
            message = motion_message(
                self.site,
                self.robot,
                self.binding,
                self.boot,
                self.motion_sequence,
                self.observe_motion(),
            )
            self.transport.publish_motion(motion_topic(self.site, self.robot), message)
        except Exception as exc:  # noqa: BLE001 - display plugins must not interrupt the owner loop.
            # A display sample cannot authorize motion or interrupt an approved action.
            self.motion_errors += 1
            if self.motion_errors == 1 or self.motion_errors % 50 == 0:
                LOG.warning(
                    "Display motion unavailable (%s errors): %s",
                    self.motion_errors,
                    type(exc).__name__,
                )

    def _state(self):
        state = dict(self.observe())
        if self.halted:
            state["fault"] = self.halted
        return state

    def _event(self, request, phase, revision, **details):
        return reply(
            request,
            request["type"].split(".")[0] + "." + phase,
            {
                **self._state(),
                "binding_id": self.binding,
                "boot_id": self.boot,
                "phase_revision": revision,
                **details,
            },
        )

    def _send(self, event):
        self.transport.publish(topic(self.site, "robot", self.robot, "event"), event)

    def _record(self, request, event, *, active):
        data = deepcopy(self.journal.data)
        key = request["command_id"]
        if len(data["records"]) >= 256 and key not in data["records"]:
            raise ValueError("trial history full")
        data["records"][key] = {"command": request, "event": event}
        data["outbox"][event["message_id"]] = event
        data["active"] = key if active else None
        self.journal.save(data)

    def _guard(self, request):
        p, s = request["payload"], self._state()
        if (
            not -0.5
            <= (datetime.now(UTC) - timestamp(s["observed_at"])).total_seconds()
            <= 3
        ):
            raise ValueError("stale_robot_observation")
        if self.halted or s["fault"] is not None:
            raise ValueError("recovery_required")
        if p["expected_boot_id"] != self.boot:
            raise ValueError("boot_mismatch")
        if self.journal.data["active"]:
            raise ValueError("busy")
        if not s["stopped"] or s["line_enabled"]:
            raise ValueError("not_stopped")
        if request["type"] == "route.requested":
            angle = s["heading_rad"] - p["start_heading_rad"]
            if (
                s["mode"] != "line"
                or s["drive_owner"] not in {"none", "line"}
                or s["node_id"] != p["start_node"]
                or s["graph_version"] != p["graph_version"]
                or abs(math.atan2(math.sin(angle), math.cos(angle))) > 0.35
            ):
                raise ValueError("route_start_mismatch")
        else:
            if any(s[k] != p[k] for k in ("map_id", "map_version")):
                raise ValueError("map_mismatch")
            if request["type"] == "mode_switch.requested":
                if s["node_id"] != "N12" or s["mode"] != "line":
                    raise ValueError("not_at_switch_node")
            elif (
                s["mode"] != "nav2"
                or not s["nav2_ready"]
                or not s["localization_ready"]
                or s["drive_owner"] != "nav2"
            ):
                raise ValueError("nav2_not_ready")

    def handle(self, delivery):
        request = decode(delivery.payload)
        validate_trial(request)
        if delivery.retained or delivery.qos != 1:
            raise ValueError("trial requires non-retained QoS1")
        if (request["site_id"], request["source"], request["target"]) != (
            self.site,
            self.control,
            self.robot,
        ):
            raise ValueError("routing_mismatch")
        if delivery.topic == topic(self.site, "robot", self.robot, "ack"):
            if request["type"] != "event.ack":
                raise ValueError("invalid ack type")
            data = deepcopy(self.journal.data)
            event = data["outbox"].get(request["payload"]["acked_message_id"])
            if event:
                if any(
                    event[k] != request[k]
                    for k in ("task_id", "step_id", "command_id", "execution_id")
                ):
                    raise ValueError("ack correlation mismatch")
                del data["outbox"][event["message_id"]]
                self.journal.save(data)
            return
        if delivery.topic != topic(
            self.site, "robot", self.robot, "cmd"
        ) or not request["type"].endswith(".requested"):
            raise ValueError("invalid command channel/type")
        check_command(
            request,
            site=self.site,
            target=self.robot,
            source=self.control,
            retained=False,
        )
        if request["payload"]["binding_id"] != self.binding:
            raise ValueError("binding_mismatch")
        if request["type"] == "status.requested":
            check_expiry(request)
            fresh(request)
            active = self.journal.data["active"]
            if (
                not active
                or request["task_id"]
                == self.journal.data["records"][active]["command"]["task_id"]
            ):
                self.last_poll = monotonic()
            self._send(self._event(request, "completed", 1))
            return
        old = self.journal.data["records"].get(request["command_id"])
        if old:
            if old["command"] != request:
                raise ValueError("idempotency_conflict")
            self._send(old["event"])
            return
        try:
            check_expiry(request)
            fresh(request)
            self._guard(request)
            self.validate_action(deepcopy(request))
        except ValueError as exc:
            # A rejection cannot replace another active action or grant a new lease.
            self._send(self._event(request, "rejected", 1, error_code=str(exc)))
            return
        accepted = self._event(request, "accepted", 1)
        self._record(request, accepted, active=True)
        self.last_poll = monotonic()
        self._send(accepted)
        try:
            self.dispatch(deepcopy(request))
        except BaseException:
            self.halt("dispatch_failed")
            raise

    def report(self, request, phase, **details):
        """Report measured progress/result; no automatic completion after receipt."""
        active = self.journal.data["active"]
        if not active:
            return False
        current = self.journal.data["records"][active]["command"]
        if any(request[k] != current[k] for k in ("command_id", "execution_id")):
            return False
        try:
            self._report(phase, **details)
            return True
        except BaseException:
            self.halt("invalid_action_report")
            raise

    def _report(self, phase, **details):
        if phase not in {"progress", "completed", "failed"}:
            raise ValueError("report requires progress, completed or failed")
        if self.halted or not self.journal.data["active"]:
            raise ValueError("no recoverable active action")
        old = self.journal.data["records"][self.journal.data["active"]]
        event = self._event(
            old["command"],
            phase,
            old["event"]["payload"]["phase_revision"] + 1,
            **details,
        )
        check_result(old["command"], event)
        fresh(event)
        if phase == "failed":
            self.safe_stop(details["error_code"])
        self._record(old["command"], event, active=phase not in {"completed", "failed"})
        self._send(event)

    def halt(self, reason):
        self.halted = reason
        self.safe_stop(reason)
        # Keep the active record unresolved. Restart must not resume the action.

    def poll(self, timeout=0.1):
        try:
            if not self.transport.ready or (
                self.journal.data["active"] and monotonic() - self.last_poll > 3
            ):
                self.halt("control_link_lost")
            try:
                delivery = self.transport.receive(timeout)
            except Empty:
                delivery = None
            if delivery:
                try:
                    self.handle(delivery)
                except ValueError as exc:
                    LOG.warning("Rejected trial message: %s", exc)
                self.transport.ack(delivery)
            if self.transport.ready and monotonic() - self.last_send >= 1:
                for event in list(self.journal.data["outbox"].values()):
                    self._send(event)
                self.last_send = monotonic()
        except BaseException:
            self.halt("endpoint_error")
            raise

    def close(self):
        try:
            self.safe_stop("endpoint_closed")
        finally:
            self.transport.stop()
            self.journal.close()
