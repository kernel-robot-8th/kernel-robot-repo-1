"""One trial owner: warehouse cycle, dummy transfers, smartcart and Nav2.

Run with --config FILE --journal FILE. Nothing moves until stdin 'start'.
This isolated CLI does not register real robots in the development web registry.
"""

import argparse
import itertools
import json
import math
import selectors
import sys
from copy import deepcopy
from datetime import UTC, datetime
from queue import Empty
from time import monotonic
from uuid import uuid4

from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import decode, timestamp, topic
from roport_control_server.adapters.mqtt.mqtt import MqttTransport
from roport_control_server.adapters.mqtt.trial_wire import (
    PROFILE,
    check_result,
    command,
    fresh,
    mqtt_config,
    reply,
    validate_trial,
)

ROUTES = (
    "to_loading",
    "to_delivery",
    "return_warehouse",
    "cart_to_delivery",
    "cart_to_switch",
)
WAITS = {
    "to_loading": "WAIT_LOAD",
    "to_delivery": "WAIT_INBOUND",
    "return_warehouse": "WAIT_REQUEST",
    "cart_to_delivery": "WAIT_CART_LOAD",
}
DUMMIES = {
    "WAIT_LOAD": "to_delivery",
    "WAIT_INBOUND": "return_warehouse",
    "WAIT_CART_LOAD": "cart_to_switch",
}


def validate_config(config):
    if config["profile"] != PROFILE or config["mode"] not in {"simulation", "hardware"}:
        raise ValueError("trial profile/mode required")
    scenario = config.get("scenario", "two_pinky")
    if scenario not in {"single_pinky", "two_pinky"}:
        raise ValueError("unknown trial scenario")
    single_robot = scenario == "single_pinky"
    robots = config["robots"]
    roles = {"logistics"} if single_robot else {"logistics", "smartcart"}
    if set(robots) != roles:
        raise ValueError("robot roles do not match trial scenario")
    if len({r["robot_id"] for r in robots.values()}) != len(roles) or len(
        {r["binding_id"] for r in robots.values()}
    ) != len(roles):
        raise ValueError("roles require distinct robot slots and bindings")
    for r in robots.values():
        if r["robot_id"] not in {"pinky1", "pinky2", "pinky3"}:
            raise ValueError("unknown Pinky slot")
    nodes = config["nodes"]
    for node, links in nodes.items():
        if any(
            target == node or target not in nodes or node not in nodes[target]
            for target in links
        ):
            raise ValueError("graph must have symmetric known edges")
    route_steps = ("to_delivery", "cart_to_switch") if single_robot else ROUTES
    if set(config["routes"]) != set(route_steps):
        raise ValueError("routes do not match trial scenario")
    for step in route_steps:
        p = config["routes"][step]
        command(
            site=config["site_id"],
            control=config["control_id"],
            robot=robots["logistics"]["robot_id"],
            task="validation",
            step=step,
            kind="route",
            payload={**p, "binding_id": "validation", "expected_boot_id": "validation"},
        )
        if p["graph_version"] != config["graph_version"] or any(
            b not in nodes.get(a, []) for a, b in zip(p["route"], p["route"][1:])
        ):
            raise ValueError("route does not match selected graph")
    routes = config["routes"]
    if single_robot:
        path = routes["to_delivery"]["route"]
        if (
            path[0] != "N16"
            or path[-1] != "N11"
            or [n for n in path if n in {"N18", "N19"}] != ["N18", "N19"]
            or routes["cart_to_switch"]["route"][0] != "N11"
        ):
            raise ValueError("single Pinky requires N16 via N18/N19 to N11")
    else:
        for a, b in itertools.pairwise(ROUTES[:3]):
            if routes[a]["route"][-1] != routes[b]["route"][0]:
                raise ValueError("logistics legs are disconnected")
        if (
            routes["return_warehouse"]["route"][-1] != routes["to_loading"]["route"][0]
            or routes["cart_to_delivery"]["route"][-1]
            != routes["to_delivery"]["route"][-1]
            or routes["cart_to_switch"]["route"][0]
            != routes["cart_to_delivery"]["route"][-1]
        ):
            raise ValueError("trial stops/return mismatch")
    if routes["cart_to_switch"]["route"][-1] != "N12":
        raise ValueError("trial switch requires N12")
    command(
        site=config["site_id"],
        control=config["control_id"],
        robot=robots["logistics" if single_robot else "smartcart"]["robot_id"],
        task="validation",
        step="navigate",
        kind="navigate",
        payload={
            **config["nav_goal"],
            "binding_id": "validation",
            "expected_boot_id": "validation",
        },
    )
    if any(config["nav_goal"][k] != config[k] for k in ("map_id", "map_version")):
        raise ValueError("goal map mismatch")
    if (
        type(config["command_timeout_s"]) not in (int, float)
        or not math.isfinite(config["command_timeout_s"])
        or not 1 <= config["command_timeout_s"] <= 600
    ):
        raise ValueError("command timeout must be 1..600 seconds")
    if config["mode"] == "hardware" and config.get("field_verified") is not True:
        raise ValueError(
            "verify starts, manoeuvres, goal and local watchdog before hardware"
        )


class TrialControl:
    def __init__(self, config, transport, journal):
        validate_config(config)
        self.config, self.transport, self.journal = deepcopy(config), transport, journal
        self.single_robot = config.get("scenario") == "single_pinky"
        if journal.data and journal.data.get("config") != config:
            raise ValueError("journal configuration changed; reconcile old trial first")
        if not journal.data:
            journal.save(
                {
                    "config": config,
                    "task_id": str(uuid4()),
                    "phase": "READY",
                    "active": None,
                    "completed": {},
                    "events": {},
                    "statuses": {},
                    "boot_ids": {},
                    "transfers": [],
                    "gate": None,
                    "request": None,
                    "fault": None,
                }
            )
        self.pending_status, self.heard = {}, {}
        self.last_poll = 0.0
        self.deadline = None
        if self.state["phase"] not in {"READY", "COMPLETE", "HOLD"}:
            self.hold("control_restarted")

    @property
    def state(self):
        return self.journal.data

    def hold(self, reason):
        state = deepcopy(self.state)
        state.update(phase="HOLD", fault=reason)
        self.journal.save(state)
        # Stop renewing the active robot's lease. Its local controller must stop.

    def _robot(self, step):
        if self.single_robot:
            return self.config["robots"]["logistics"]
        return self.config["robots"][
            "smartcart"
            if step.startswith("cart_") or step in {"mode_switch", "navigate"}
            else "logistics"
        ]

    def _status(self, robot):
        device = robot["robot_id"]
        status = self.state["statuses"].get(device)
        if not status or monotonic() - self.heard.get(device, -math.inf) > 3:
            raise ValueError("robot status unavailable/stale")
        if (
            status["fault"] is not None
            or not status["stopped"]
            or status["line_enabled"]
        ):
            raise ValueError("robot is not safely waiting")
        return status

    def _send(self, step):
        robot = self._robot(step)
        status = self._status(robot)
        if step in ROUTES:
            kind, payload = "route", self.config["routes"][step]
            if status["node_id"] != payload["start_node"] or status["mode"] != "line":
                raise ValueError("unexpected route start")
        elif step == "mode_switch":
            kind, payload = (
                "mode_switch",
                {
                    "node_id": "N12",
                    "from_mode": "line",
                    "to_mode": "nav2",
                    "map_id": self.config["map_id"],
                    "map_version": self.config["map_version"],
                },
            )
        else:
            kind, payload = "navigate", self.config["nav_goal"]
        cmd = command(
            site=self.config["site_id"],
            control=self.config["control_id"],
            robot=robot["robot_id"],
            task=self.state["task_id"],
            step=step,
            kind=kind,
            payload={
                **payload,
                "binding_id": robot["binding_id"],
                "expected_boot_id": status["boot_id"],
            },
        )
        state = deepcopy(self.state)
        state.update(phase=step, active=cmd, gate=None)
        self.journal.save(state)
        self.deadline = monotonic() + self.config["command_timeout_s"]
        self.transport.publish(
            topic(self.config["site_id"], "robot", robot["robot_id"], "cmd"), cmd
        )

    def start(self):
        if self.state["phase"] != "READY":
            raise ValueError("trial already started")
        for robot in self.config["robots"].values():
            self._status(robot)
        self._send("to_delivery" if self.single_robot else "to_loading")

    def dummy(self, token):
        phase = self.state["phase"]
        gate = self.state["gate"]
        if phase not in DUMMIES or not gate or token != gate["stop_session_id"]:
            raise ValueError("dummy does not match current stop session")
        status = self._status(self._robot(DUMMIES[phase]))
        if self.single_robot and (
            status["node_id"] != "N11" or status["mode"] != "line"
        ):
            raise ValueError("single Pinky dummy requires a stop at N11")
        state = deepcopy(self.state)
        state["transfers"].append(
            {
                **gate,
                "simulated": True,
                "source": "trial-operator",
                "observed_at": datetime.now(UTC).isoformat(),
            }
        )
        if self.single_robot and phase == "WAIT_INBOUND":
            state["phase"] = "WAIT_CART_LOAD"
            state["gate"] = {
                **gate,
                "step_id": "cart_load_assumed",
                "arrival_execution_id": gate["stop_session_id"],
                "stop_session_id": str(uuid4()),
                "assumed_role": "smartcart",
            }
            self.journal.save(state)
        else:
            self.journal.save(state)
            self._send(DUMMIES[phase])

    def request(self, items):
        if self.state["phase"] != "WAIT_REQUEST" or items != [
            {"package_id": self.config["package_id"], "quantity": 1}
        ]:
            raise ValueError(
                "trial accepts its one stored package after logistics return"
            )
        self._status(self.config["robots"]["smartcart"])
        state = deepcopy(self.state)
        state["request"] = {"request_id": str(uuid4()), "items": items}
        self.journal.save(state)
        self._send("cart_to_delivery")

    def _ack(self, event):
        ack = reply(
            event,
            "event.ack",
            {"acked_message_id": event["message_id"], "status": "stored"},
        )
        self.transport.publish(
            topic(self.config["site_id"], "robot", event["source"], "ack"), ack
        )

    def handle(self, delivery):
        event = decode(delivery.payload)
        validate_trial(event)
        if event["type"].split(".")[1] not in {
            "accepted",
            "progress",
            "completed",
            "failed",
            "rejected",
        }:
            raise ValueError("invalid event channel/type")
        robot = next(
            (
                r
                for r in self.config["robots"].values()
                if r["robot_id"] == event["source"]
            ),
            None,
        )
        if (
            delivery.retained
            or delivery.qos != 1
            or robot is None
            or delivery.topic
            != topic(self.config["site_id"], "robot", event["source"], "event")
            or event["target"] != self.config["control_id"]
            or event["site_id"] != self.config["site_id"]
            or event["task_id"] != self.state["task_id"]
            or event["payload"].get("binding_id") != robot["binding_id"]
        ):
            raise ValueError("untrusted/unrelated trial event")
        if event["message_id"] in self.state["events"]:
            if self.state["events"][event["message_id"]] != event:
                raise ValueError("conflicting event ID")
            self._ack(event)
            return
        p, device = event["payload"], event["source"]
        if event["type"] == "status.completed":
            cmd = self.pending_status.pop(event["command_id"], None)
            if cmd is None or timestamp(cmd["expires_at"]) <= datetime.now(UTC):
                return
            check_result(cmd, event)
            fresh(event)
            old_boot = self.state["boot_ids"].get(device)
            if old_boot and old_boot != p["boot_id"]:
                self.hold("robot_restarted")
                return
            if any(
                p[k] != self.config[k]
                for k in ("graph_version", "map_id", "map_version")
            ):
                self.hold("robot_version_mismatch")
                return
            current = self.state["statuses"].get(device)
            if current and timestamp(p["observed_at"]) < timestamp(
                current["observed_at"]
            ):
                return
            state = deepcopy(self.state)
            state["statuses"][device], state["boot_ids"][device] = p, p["boot_id"]
            self.journal.save(state)
            self.heard[device] = monotonic()
            if p["fault"] is not None:
                self.hold("robot_fault")
            return
        cmd = self.state["active"]
        if not cmd or event["command_id"] != cmd["command_id"]:
            return
        if any(
            p[k] != self.config[k] for k in ("graph_version", "map_id", "map_version")
        ):
            raise ValueError("robot version changed during action")
        check_result(cmd, event)
        fresh(event)
        previous = [
            e
            for e in self.state["events"].values()
            if e["command_id"] == cmd["command_id"]
        ]
        revision = max((e["payload"]["phase_revision"] for e in previous), default=0)
        if p["phase_revision"] <= revision:
            raise ValueError("conflicting or reordered phase revision")
        if event["type"] == "route.progress":
            previous_seq = max(
                (e["payload"].get("seq", -1) for e in previous), default=-1
            )
            if p["seq"] < previous_seq:
                raise ValueError("route progress moved backwards")
        state = deepcopy(self.state)
        state["events"][event["message_id"]] = event
        state["statuses"][device] = p
        self.journal.save(state)
        self.heard[device] = monotonic()
        self._ack(event)  # Application ACK only after fsync; separate from PUBACK.
        phase = event["type"].split(".")[1]
        if phase in {"failed", "rejected"}:
            self.hold(p["error_code"])
        elif phase == "completed" and self.state["phase"] != "HOLD":
            step = cmd["step_id"]
            state = deepcopy(self.state)
            state["completed"][step] = event
            state["active"] = None
            self.deadline = None
            if step in WAITS:
                state["phase"] = WAITS[step]
                if state["phase"] in DUMMIES:
                    state["gate"] = {
                        "task_id": state["task_id"],
                        "step_id": step,
                        "stop_session_id": cmd["execution_id"],
                        "node_id": p["node_id"],
                        "robot_id": device,
                        "package_id": self.config["package_id"],
                        "quantity": 1,
                    }
            elif step == "navigate":
                state["phase"] = "COMPLETE"
            self.journal.save(state)
            if step in {"cart_to_switch", "mode_switch"}:
                self._send("mode_switch" if step == "cart_to_switch" else "navigate")

    def poll(self, timeout=0.1):
        try:
            if not self.transport.ready:
                self.hold("mqtt_disconnected")
                return
            now = monotonic()
            if self.deadline and now > self.deadline:
                self.hold("execution_timeout_unknown")
                self.deadline = None
            if self.state["phase"] not in {"READY", "COMPLETE", "HOLD"} and any(
                now - self.heard.get(r["robot_id"], -math.inf) > 3
                for r in self.config["robots"].values()
            ):
                self.hold("robot_status_timeout")
            if now - self.last_poll >= 1 and self.state["phase"] not in {
                "HOLD",
                "COMPLETE",
            }:
                # A fresh response can arrive after the next 1 Hz request.
                utc_now = datetime.now(UTC)
                self.pending_status = {
                    key: cmd
                    for key, cmd in self.pending_status.items()
                    if timestamp(cmd["expires_at"]) > utc_now
                }
                for robot in self.config["robots"].values():
                    cmd = command(
                        site=self.config["site_id"],
                        control=self.config["control_id"],
                        robot=robot["robot_id"],
                        task=self.state["task_id"],
                        step="status",
                        kind="status",
                        payload={"binding_id": robot["binding_id"]},
                    )
                    self.pending_status[cmd["command_id"]] = cmd
                    self.transport.publish(
                        topic(
                            self.config["site_id"], "robot", robot["robot_id"], "cmd"
                        ),
                        cmd,
                    )
                self.last_poll = now
            try:
                delivery = self.transport.receive(timeout)
            except Empty:
                return
            try:
                self.handle(delivery)
            except ValueError:
                # A malformed/current event cannot unlock any following route.
                self.hold("invalid_trial_event")
            self.transport.ack(delivery)
        except (ConnectionError, TimeoutError):
            self.hold("transport_outcome_unknown")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--journal", required=True)
    parser.add_argument("--hardware-authorized", action="store_true")
    args = parser.parse_args()
    with open(args.config) as file:
        config = json.load(file)
    validate_config(config)
    connection = mqtt_config(config["control_id"])
    if config["mode"] == "hardware" and not args.hardware_authorized:
        parser.error("hardware trial requires field checks and --hardware-authorized")
    if config["mode"] == "simulation" and connection.host not in {
        "localhost",
        "127.0.0.1",
        "::1",
    }:
        parser.error("simulation profile is loopback-only")
    journal = Journal(args.journal)
    transport = MqttTransport(
        connection,
        [
            topic(config["site_id"], "robot", r["robot_id"], "event")
            for r in config["robots"].values()
        ],
    )
    control = TrialControl(config, transport, journal)
    selector = selectors.DefaultSelector()
    selector.register(sys.stdin, selectors.EVENT_READ)
    try:
        transport.start()
        print(
            "Inputs: start | dummy <stop_session_id> | request <package_id> | status | stop",
            flush=True,
        )
        previous = None
        while True:
            control.poll()
            visible = (
                control.state["phase"],
                control.state["fault"],
                control.state["gate"],
            )
            if visible != previous:
                print(
                    json.dumps(
                        {"phase": visible[0], "fault": visible[1], "gate": visible[2]},
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                previous = deepcopy(visible)
            if selector.select(0):
                line = sys.stdin.readline()
                if not line:
                    break
                fields = line.strip().split()
                try:
                    if fields == ["start"]:
                        control.start()
                    elif len(fields) == 2 and fields[0] == "dummy":
                        control.dummy(fields[1])
                    elif len(fields) == 2 and fields[0] == "request":
                        control.request([{"package_id": fields[1], "quantity": 1}])
                    elif fields == ["status"]:
                        print(json.dumps(control.state, ensure_ascii=False), flush=True)
                    elif fields == ["stop"]:
                        break
                    else:
                        print("Unknown input", flush=True)
                except ValueError as exc:
                    print(f"Not advanced: {exc}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if control.state["phase"] != "COMPLETE":
            control.hold("operator_stopped")
        transport.stop()
        journal.close()
        selector.close()


if __name__ == "__main__":
    main()
