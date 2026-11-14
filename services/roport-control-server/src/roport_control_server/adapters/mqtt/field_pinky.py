"""The same v2 endpoint for physical TrialDriver and VirtualPinky.

Only the driver boundary uses the previous internal command representation.
No task, step, execution, binding or site identifier crosses MQTT.
"""

import logging
from copy import deepcopy
from datetime import UTC, datetime
from queue import Empty
from time import monotonic
from uuid import uuid4

from . import field_wire as wire
from .journal import Journal
from .mqtt import MqttTransport
from .trial_wire import PROFILE, check_result

OPERATIONS = {
    "line_route_command": "route",
    "set_drive_mode_command": "mode_switch",
    "nav_goal_command": "navigate",
}
LOAD_NODES = {"N01", "N11", "N21", "N22"}
LOG = logging.getLogger(__name__)


def driver_command(message, profile, binding):
    """Translate an approved wire command once; retain its exact body in the journal."""
    p = deepcopy(message["payload"])
    p["binding_id"] = binding
    p["expected_boot_id"] = p.pop("expected_boot_version")
    p.pop("departure", None)
    kind = OPERATIONS[message["type"]]
    if kind == "route":
        p["goal"] = {
            "node_id": p["goal"]["node"],
            "location_id": None,
            "on_arrival": "hold",
        }
    else:
        if p["map_version"] != profile["map_version"]:
            raise ValueError("map_mismatch")
        p["map_id"] = profile["map_id"]
        if kind == "mode_switch":
            if p["graph_version"] != profile["graph_version"]:
                raise ValueError("graph_mismatch")
            p["node_id"] = p.pop("node")
            p.pop("graph_version")
        else:
            p.setdefault("goal_id", "coordinate_goal")
            p["goal_version"] = profile["map_version"]
    return {
        "schema_version": "1.0",
        "profile": PROFILE,
        "site_id": profile["site_id"],
        "type": kind + ".requested",
        "source": message["source"],
        "target": message["target"],
        "message_id": message["message_id"],
        "command_id": message["command_id"],
        "execution_id": message["command_id"],
        "task_id": message["command_id"],
        "step_id": kind,
        "sent_at": message["sent_at"],
        "expires_at": message["expires_at"],
        "payload": p,
    }


class FieldPinkyClient:
    def __init__(self, connection, profile, role, journal_path, driver):
        self.profile, self.driver = profile, driver
        self.robot = profile["robots"][role]["robot_id"]
        self.binding = profile["robots"][role]["binding_id"]
        self.control = profile["control_id"]
        self.boot = str(uuid4())
        self.journal = Journal(journal_path)
        identity = {
            "profile": wire.PROFILE,
            "robot": self.robot,
            "control": self.control,
        }
        if self.journal.data and self.journal.data.get("identity") != identity:
            self.journal.close()
            raise ValueError("journal belongs to another endpoint/contract")
        if not self.journal.data:
            self.journal.save(
                {
                    "identity": identity,
                    "records": {},
                    "outbox": {},
                    "active": None,
                    "arrival": None,
                }
            )
        self.halted = "recovery_required" if self.journal.data["active"] else None
        data = deepcopy(self.journal.data)
        data["arrival"] = None  # A new endpoint must confirm a new physical visit.
        self.journal.save(data)
        self.sequence, self.motion_sequence = 0, 0
        self.last_keep = monotonic()
        self.last_send = self.last_motion = 0.0
        self.keep_messages = set()
        self.transport = MqttTransport(
            connection, [wire.topic(self.robot, c) for c in ("cmd", "ack")]
        )

    def start(self):
        if self.halted:
            self.driver.safe_stop(self.halted)
        self.transport.start()

    def _observation(self):
        state = dict(self.driver.observe())
        if self.halted:
            state["fault"] = self.halted
        return state

    def _state_payload(self, state):
        return {
            "node": state["node_id"],
            "arrival_command_id": self.journal.data["arrival"],
            **{
                key: state[key]
                for key in (
                    "mode",
                    "stopped",
                    "line_enabled",
                    "nav2_ready",
                    "localization_ready",
                    "drive_owner",
                    "graph_version",
                    "map_version",
                    "heading_rad",
                    "fault",
                )
            },
        }

    def state_message(self):
        self.sequence += 1
        s = self._observation()
        return wire.build_message(
            "robot_state",
            self.robot,
            self.control,
            {
                "boot_version": self.boot,
                "observed_at": s["observed_at"],
                "sequence": self.sequence,
                "active_command_id": self.journal.data["active"],
                "state": self._state_payload(s),
            },
        )

    def _send(self, event):
        self.transport.publish(wire.topic(self.robot, "event"), event)

    def _connection_lost(self):
        # Keep the process alive to report HOLD after reconnect; never resume motion.
        if not self.halted:
            self.halted = "control_link_lost"
            self.driver.safe_stop(self.halted)
            LOG.warning("MQTT link lost; motion held until reconciliation")

    def _event(self, command, status, sequence, result=None, reason=None):
        p = {
            "boot_version": self.boot,
            "observed_at": self.driver.observe()["observed_at"],
            "sequence": sequence,
            "status": status,
        }
        if result is not None:
            p["result"] = result
        if reason:
            p["reason"] = reason
        return wire.build_message(
            "command_status",
            self.robot,
            self.control,
            p,
            command_id=command["command_id"],
        )

    def _save_event(self, command, event, active=False):
        data = deepcopy(self.journal.data)
        data["records"][command["command_id"]] = {
            "command": command,
            "event": event,
        }
        data["outbox"][event["message_id"]] = event
        if active:
            data["active"] = command["command_id"]
        elif data["active"] == command["command_id"]:
            data["active"] = None
        if event["payload"]["status"] == "accepted" and command["type"] in OPERATIONS:
            data["arrival"] = None
        if (
            event["payload"]["status"] == "completed"
            and command["type"] == "line_route_command"
        ):
            data["arrival"] = command["command_id"]
        self.journal.save(data)
        self._send(event)

    def _fresh(self, message):
        now = datetime.now(UTC)
        if not -0.5 <= (now - wire.timestamp(message["sent_at"])).total_seconds() <= 3:
            raise ValueError("stale_message")
        if "expires_at" in message and wire.timestamp(message["expires_at"]) <= now:
            raise ValueError("expired_command")

    def handle(self, delivery):
        m = wire.decode(delivery.payload)
        if (
            delivery.retained
            or delivery.qos != 1
            or m["source"] != self.control
            or m["target"] != self.robot
        ):
            raise ValueError("routing_or_delivery_mismatch")
        if m["type"] == "event_stored":
            if delivery.topic != wire.topic(self.robot, "ack"):
                raise ValueError("invalid_ack_channel")
            data = deepcopy(self.journal.data)
            data["outbox"].pop(m["payload"]["acked_message_id"], None)
            self.journal.save(data)
            return
        if delivery.topic != wire.topic(self.robot, "cmd"):
            raise ValueError("invalid_command_channel")
        if m["type"] == "robot_state_request":
            self._fresh(m)
            p = m["payload"]
            if p:
                if (
                    p["expected_boot_version"] != self.boot
                    or p["keep_running_command_id"] != self.journal.data["active"]
                ):
                    raise ValueError("keep_running_mismatch")
                if m["message_id"] not in self.keep_messages:
                    if len(self.keep_messages) >= 100000:
                        raise ValueError("keep_running_history_full")
                    self.keep_messages.add(m["message_id"])
                    self.last_keep = monotonic()
            self._send(self.state_message())
            return
        old = self.journal.data["records"].get(m["command_id"])
        if old:
            if old["command"] != m:
                raise ValueError("idempotency_conflict")
            self._send(old["event"])
            return
        try:
            self._fresh(m)
            if m["payload"]["expected_boot_version"] != self.boot:
                raise ValueError("boot_mismatch")
            if len(self.journal.data["records"]) >= 1024:
                raise ValueError("command_history_full")
            if m["type"] == "robot_stop_command":
                if m["payload"]["target_command_id"] != self.journal.data["active"]:
                    raise ValueError("stop_target_mismatch")
                self._save_event(
                    m,
                    self._event(
                        m, "accepted", 1, self._state_payload(self._observation())
                    ),
                    active=True,
                )
                self.driver.safe_stop("operator_stop")
                return
            if (
                m["type"] not in OPERATIONS
                or self.halted
                or self.journal.data["active"]
            ):
                raise ValueError("busy_or_recovery_required")
            s = self._observation()
            if (
                not -0.5
                <= (
                    datetime.now(UTC) - wire.timestamp(s["observed_at"])
                ).total_seconds()
                <= 3
                or not s["stopped"]
                or s["line_enabled"]
            ):
                raise ValueError("fresh_stop_required")
            p = m["payload"]
            if m["type"] == "line_route_command":
                if (
                    s["node_id"] != p["start_node"]
                    or s["graph_version"] != p["graph_version"]
                ):
                    raise ValueError("route_start_mismatch")
                departure = p.get("departure")
                if (p["start_node"] in LOAD_NODES or departure) and (
                    not departure
                    or departure["arrival_command_id"] != self.journal.data["arrival"]
                ):
                    raise ValueError("current_arrival_required")
            legacy = driver_command(m, self.profile, self.binding)
            self.driver.validate_action(deepcopy(legacy))
        except ValueError as exc:
            self._save_event(
                m,
                self._event(
                    m,
                    "rejected",
                    1,
                    self._state_payload(self._observation()),
                    reason=str(exc),
                ),
            )
            return
        self._save_event(
            m,
            self._event(m, "accepted", 1, self._state_payload(s)),
            active=True,
        )
        self.last_keep = monotonic()
        self.driver.dispatch(deepcopy(legacy))

    def report(self, legacy, phase, **details):
        try:
            return self._report(legacy, phase, **details)
        except (ConnectionError, TimeoutError):
            self._connection_lost()
            return False

    def _report(self, legacy, phase, **details):
        active = self.journal.data["active"]
        if not active or legacy["command_id"] != active:
            return False
        record = self.journal.data["records"][active]
        if record["command"]["type"] == "robot_stop_command":
            return False
        observation = self._observation()
        payload = {
            **observation,
            "binding_id": self.binding,
            "boot_id": self.boot,
            "phase_revision": record["event"]["payload"]["sequence"] + 1,
            **details,
        }
        event = {
            **legacy,
            "type": legacy["type"].split(".")[0] + "." + phase,
            "source": self.robot,
            "target": self.control,
            "payload": payload,
        }
        check_result(legacy, event)
        result = self._state_payload(observation)
        for key, value in details.items():
            if key not in {"error_code", "goal_version", "goal_id"}:
                result[
                    {"node_id": "node", "nav2_result": "navigation_result"}.get(
                        key, key
                    )
                ] = value
        status = {
            "progress": "in_progress",
            "completed": "completed",
            "failed": "failed",
        }[phase]
        outcome = self._event(
            record["command"],
            status,
            payload["phase_revision"],
            result,
            details.get("error_code"),
        )
        self._save_event(record["command"], outcome, active=phase == "progress")
        if phase == "failed":
            self.driver.safe_stop(details["error_code"])
        self._send(self.state_message())
        return True

    def _finish_stop(self):
        active = self.journal.data["active"]
        if not active:
            return
        rec = self.journal.data["records"][active]
        if (
            rec["command"]["type"] != "robot_stop_command"
            or not self.driver.observe()["stopped"]
        ):
            return
        original = self.journal.data["records"][
            rec["command"]["payload"]["target_command_id"]
        ]
        self._save_event(
            original["command"],
            self._event(
                original["command"],
                "interrupted",
                original["event"]["payload"]["sequence"] + 1,
                {"stopped": True},
                "operator_stop",
            ),
        )
        self._save_event(
            rec["command"],
            self._event(rec["command"], "completed", 2, {"stopped": True}),
        )
        self.halted = "operator_stop"

    def poll(self, timeout=0.1):
        try:
            self._poll(timeout)
        except (ConnectionError, TimeoutError):
            self._connection_lost()

    def _poll(self, timeout):
        if not self.transport.ready or (
            self.journal.data["active"] and monotonic() - self.last_keep > 3
        ):
            self._connection_lost()
        try:
            delivery = self.transport.receive(timeout)
        except Empty:
            delivery = None
        if delivery:
            try:
                self.handle(delivery)
            except ValueError as exc:
                # Log the reason, never the payload; invalid input cannot renew the watchdog.
                LOG.warning("Rejected field MQTT message: %s", exc)
            self.transport.ack(delivery)
        self._finish_stop()
        if self.transport.ready and monotonic() - self.last_send >= 1:
            self._send(self.state_message())
            for event in self.journal.data["outbox"].values():
                self._send(event)
            self.last_send = monotonic()

    def publish_motion(self):
        try:
            self._publish_motion()
        except (ConnectionError, TimeoutError):
            self._connection_lost()

    def _publish_motion(self):
        if not self.transport.ready:
            self._connection_lost()
            return
        observe = getattr(self.driver, "observe_motion", None)
        if observe is None or monotonic() - self.last_motion < 0.2:
            return
        self.last_motion = monotonic()
        self.motion_sequence += 1
        p = observe()
        pose = p["pose"]
        if pose:
            pose = {key: pose[key] for key in ("x", "y", "yaw")}
        message = wire.build_message(
            "robot_pose",
            self.robot,
            self.control,
            {
                "boot_version": self.boot,
                "observed_at": p["observed_at"],
                "sequence": self.motion_sequence,
                "map_version": p["map_version"],
                "frame_id": p["frame_id"],
                "pose_source": p["pose_source"],
                "pose": pose,
            },
        )
        self.transport.publish_motion(wire.topic(self.robot, "motion"), message)

    def close(self):
        try:
            self.driver.safe_stop("endpoint_closed")
        finally:
            try:
                self.transport.stop()
            finally:
                self.journal.close()
