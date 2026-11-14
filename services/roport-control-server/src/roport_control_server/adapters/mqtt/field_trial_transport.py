"""Keep the existing trial/UI journal model behind the actual v2 MQTT boundary."""

import json
from copy import deepcopy
from dataclasses import replace

from . import field_wire as wire
from .mqtt import MqttTransport
from .trial_wire import PROFILE, STATE


class FieldTrialTransport:
    def __init__(self, connection, subscriptions, profile, state):
        self.profile, self.state = profile, state
        self.sent, self.arrivals, self.status_queries = {}, {}, {}
        self.bindings = {
            r["robot_id"]: r["binding_id"] for r in profile["robots"].values()
        }
        self.raw = MqttTransport(connection, [self._topic(t) for t in subscriptions])

    def _topic(self, value):
        parts = value.split("/")
        if len(parts) == 6 and parts[3] == "robot" and parts[4].startswith("pinky"):
            return wire.topic(parts[4], parts[5])
        return value

    @property
    def ready(self):
        return self.raw.ready

    @property
    def fault(self):
        return self.raw.fault

    def start(self, *args, **kwargs):
        return self.raw.start(*args, **kwargs)

    def stop(self):
        self.raw.stop()

    def ack(self, delivery):
        self.raw.ack(delivery)

    def publish(self, destination, message):
        if not message["target"].startswith("pinky"):
            return self.raw.publish(destination, message)
        kind, old = message["type"], message["payload"]
        state = self.state()
        p = deepcopy(old)
        p.pop("binding_id", None)
        if "expected_boot_id" in p:
            p["expected_boot_version"] = p.pop("expected_boot_id")
        expires = message.get("expires_at")
        command_id = message["command_id"]
        if kind == "status.requested":
            self.status_queries[message["target"]] = deepcopy(message)
            command_id, p, expires = None, {}, None
            active = state.get("active")
            if active and active["target"] == message["target"]:
                p = {
                    "expected_boot_version": active["payload"]["expected_boot_id"],
                    "keep_running_command_id": active["command_id"],
                }
                expires = message["expires_at"]
            new_kind = "robot_state_request"
        elif kind == "event.ack":
            command_id = None
            p = {"acked_message_id": old["acked_message_id"]}
            new_kind = "event_stored"
        else:
            new_kind = {
                "route.requested": "line_route_command",
                "mode_switch.requested": "set_drive_mode_command",
                "navigate.requested": "nav_goal_command",
            }[kind]
            self.sent[command_id] = deepcopy(message)
            p.pop("map_id", None)
            p.pop("goal_version", None)
            if kind == "route.requested":
                p["goal"] = {"node": old["goal"]["node_id"], "on_arrival": "hold"}
                arrival = self.arrivals.get(message["target"])
                if old["start_node"] in {"N01", "N11", "N21", "N22"}:
                    transfers = [
                        t
                        for t in state.get("transfers", [])
                        if t["robot_id"] == message["target"]
                        and t["node_id"] == old["start_node"]
                    ]
                    if not arrival or not transfers:
                        raise ValueError(
                            "trial departure needs current arrival and explicit transfer confirmation"
                        )
                    # Trial-only manual confirmation is an audited action, never a warehouse completion claim.
                    p["departure"] = {
                        "arrival_command_id": arrival,
                        "transfer_command_id": transfers[-1].get("event_id")
                        or transfers[-1]["stop_session_id"],
                    }
            elif kind == "mode_switch.requested":
                p["node"] = p.pop("node_id")
                p["graph_version"] = self.profile["graph_version"]
        m = wire.build_message(
            new_kind,
            message["source"],
            message["target"],
            p,
            command_id=command_id,
            expires_at=expires,
        )
        m.update(message_id=message["message_id"], sent_at=message["sent_at"])
        wire.validate(m)
        self.raw.publish(self._topic(destination), m)

    def _context(self, identifier):
        if identifier in self.sent:
            return self.sent[identifier]
        state = self.state()
        candidates = [
            state.get("active"),
            *state.get("events", {}).values(),
            *state.get("completed", {}).values(),
        ]
        for item in candidates:
            if item and item.get("command_id") == identifier:
                return item
        raise ValueError("unknown command result")

    def receive(self, timeout=1):
        delivery = self.raw.receive(timeout)
        if delivery.topic.startswith("roport/v1/"):
            return delivery
        m = wire.decode(delivery.payload)
        robot, p = m["source"], m["payload"]
        if robot not in self.bindings or m["target"] != self.profile["control_id"]:
            raise ValueError("unexpected robot event")
        if m["type"] == "robot_pose":
            pose = p["pose"]
            old = {
                "schema_version": "1.1",
                "profile": "roport-state-dev-v1",
                "message_id": m["message_id"],
                "type": "motion",
                "site_id": self.profile["site_id"],
                "source": robot,
                "target": "control",
                "sent_at": m["sent_at"],
                "payload": {
                    "binding_id": self.bindings[robot],
                    "boot_id": p["boot_version"],
                    "sequence": p["sequence"],
                    "observed_at": p["observed_at"],
                    "map_id": self.profile["map_id"],
                    "map_version": p["map_version"],
                    "frame_id": p["frame_id"],
                    "pose_source": p["pose_source"],
                    "pose": {**pose, "z": 0.0} if pose else None,
                    "velocity": None,
                    "joints": {},
                },
            }
        else:
            if m["type"] == "robot_state":
                s = p["state"]
                self.arrivals[robot] = s["arrival_command_id"]
                context = self.status_queries.get(robot) or {
                    "task_id": self.state().get("task_id", m["message_id"]),
                    "step_id": "status",
                    "command_id": m["message_id"],
                    "execution_id": m["message_id"],
                }
                kind = "status.completed"
            elif m["type"] == "command_status":
                context = self._context(m["command_id"])
                operation = context["type"].split(".")[0]
                phase = {"in_progress": "progress", "interrupted": "failed"}.get(
                    p["status"], p["status"]
                )
                kind = operation + "." + phase
                s = p.get("result")
                if not s or not all(
                    k in s for k in ("node", "mode", "graph_version", "map_version")
                ):
                    raise ValueError("trial status requires actual robot observation")
                if phase == "completed" and operation == "route":
                    self.arrivals[robot] = m["command_id"]
            else:
                raise ValueError("unsupported trial event")
            result = {
                key: s["node"]
                if key == "node_id"
                else self.profile["map_id"]
                if key == "map_id"
                else s[key]
                for key in STATE
            }
            result.update(
                binding_id=self.bindings[robot],
                boot_id=p["boot_version"],
                observed_at=p["observed_at"],
                phase_revision=p["sequence"],
            )
            if m["type"] == "command_status":
                if "reason" in p:
                    result["error_code"] = p["reason"]
                for key in (
                    "seq",
                    "total",
                    "prev_node",
                    "next_node",
                    "pose",
                    "frame_id",
                ):
                    if key in s:
                        result[key] = s[key]
                if "navigation_result" in s:
                    result.update(
                        nav2_result=s["navigation_result"],
                        goal_id=context["payload"]["goal_id"],
                        goal_version=context["payload"]["goal_version"],
                    )
            old = {
                key: context[key]
                for key in ("task_id", "step_id", "command_id", "execution_id")
            }
            old.update(
                schema_version="1.0",
                profile=PROFILE,
                site_id=self.profile["site_id"],
                message_id=m["message_id"],
                type=kind,
                source=robot,
                target=self.profile["control_id"],
                sent_at=m["sent_at"],
                payload=result,
            )
        channel = "motion" if m["type"] == "robot_pose" else "event"
        legacy_topic = f"roport/v1/{self.profile['site_id']}/robot/{robot}/{channel}"
        return replace(
            delivery,
            topic=legacy_topic,
            payload=json.dumps(old, allow_nan=False).encode(),
        )
