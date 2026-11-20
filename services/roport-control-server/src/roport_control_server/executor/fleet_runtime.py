"""One MQTT owner for request-driven fleet work; real and virtual endpoints use v2."""

import argparse
import json
import signal
from contextlib import ExitStack
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from queue import Empty
from time import monotonic
from uuid import uuid4

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.mqtt import MqttTransport
from roport_control_server.adapters.mqtt.trial_wire import mqtt_config

from .fleet import ROLES, TARGETS, FleetControl
from .fleet_recovery import new_stop, stop_current
from .fleet_socket import FleetSocket


def expiry(seconds=5):
    return (datetime.now(UTC) + timedelta(seconds=seconds)).isoformat()


class FleetRuntime:
    def __init__(self, control, transport):
        self.control, self.transport = control, transport
        self.last_poll = -float("inf")

    def publish(self, kind, target, payload, **extra):
        message = wire.build_message(
            kind, self.control.config["control_id"], target, payload, **extra
        )
        self.transport.publish(
            wire.topic(target, "ack" if kind == "event_stored" else "cmd"), message
        )
        return message

    def handle(self, delivery):
        event = wire.decode(delivery.payload)
        source, p = event["source"], event["payload"]
        channel = "motion" if event["type"] == "robot_pose" else "event"
        if (
            source not in TARGETS
            or event["target"] != self.control.config["control_id"]
            or delivery.retained
            or delivery.topic != wire.topic(source, channel)
            or delivery.qos != (0 if channel == "motion" else 1)
        ):
            raise ValueError("unrelated MQTT event")
        # Delivery age is not observation age; replayed old state must not renew readiness.
        age = (datetime.now(UTC) - wire.timestamp(p["observed_at"])).total_seconds()
        if event["type"] in {"robot_state", "robot_pose"} and not -1 <= age <= 3:
            return
        if event["type"] == "robot_state":
            if source in ROLES and any(
                p["state"].get(k) != self.control.config[k]
                for k in ("graph_version", "map_version")
            ):
                raise ValueError("robot graph/map mismatch")
            self.control.observe(event, age=age)
        elif event["type"] == "robot_pose":
            observed = self.control.observed.get(source)
            old = self.control.poses.get(source)
            if (
                observed
                and p["boot_version"] == observed["boot_version"]
                and p["frame_id"] == "map"
                and p["map_version"] == self.control.config["map_version"]
                and (
                    not old
                    or old["boot_version"] != p["boot_version"]
                    or p["sequence"] > old["sequence"]
                )
            ):
                self.control.poses[source] = p
        elif event["type"] == "command_status" and self.stored_result(event):
            self.publish(
                "event_stored", source, {"acked_message_id": event["message_id"]}
            )

    def stored_result(self, event):
        c = self.control
        stop = next(
            (
                a["stop"]
                for a in c.data["commands"].values()
                if a.get("stop", {}).get("command_id") == event["command_id"]
            ),
            None,
        )
        if stop:
            if (
                event["source"] != stop["target"]
                or event["payload"]["boot_version"]
                != stop["payload"]["expected_boot_version"]
            ):
                raise ValueError("stop response source/boot mismatch")
            data = deepcopy(c.data)
            previous = data["events"].get(event["message_id"])
            if previous and previous != event:
                raise ValueError("conflicting stop event")
            data["events"][event["message_id"]] = event
            c.save(data)
            return True
        return c.result(event)

    def poll(self, timeout=0.02):
        try:
            self._poll(timeout)
        except (ConnectionError, TimeoutError):
            if self.control.data["commands"]:
                self.control.hold("mqtt_delivery_outcome_unknown")

    def _poll(self, timeout):
        c = self.control
        if not self.transport.ready:
            if c.data["commands"]:
                c.hold("mqtt_disconnected")
            return
        c.check_active()
        try:
            delivery = self.transport.receive(timeout)
        except Empty:
            delivery = None
        if delivery:
            try:
                self.handle(delivery)
            except ValueError as exc:
                c.hold(str(exc))
            self.transport.ack(delivery)
            # Apply new positions before renewing permission for the active route.
            c.check_active()
        for cid, command in list(c.data["commands"].items()):
            if (
                not c.data["fault"]
                or command.get("stop_published")
                or command.get("manual_stop")
            ):
                continue
            active = command["message"]
            target = active["target"]
            boot = active["payload"]["expected_boot_version"]
            observed_boot = c.observed.get(target, {}).get("boot_version", boot)
            stop = command.get("stop")
            if stop is None:
                if active["target"] in ROLES and any(
                    e.get("command_id") == cid
                    and e["payload"]["status"]
                    in {"completed", "rejected", "failed", "interrupted"}
                    for e in c.data["events"].values()
                ):
                    continue  # Pinky has already ended this command; recovery still checks idle state.
                stop = new_stop(c.config["control_id"], active, boot)
                data = deepcopy(c.data)
                data["commands"][cid]["stop"] = stop
                c.save(data)
            if not stop_current(stop, observed_boot, datetime.now(UTC)):
                continue
            try:
                self.transport.publish(wire.topic(target, "cmd"), stop)
            except ValueError as exc:
                if str(exc) != "expired_command":
                    raise
                continue
            data = deepcopy(c.data)
            data["commands"][cid]["stop_published"] = True
            c.save(data)
        now = monotonic()
        if now - self.last_poll >= 1:
            active = (
                {
                    a["message"]["target"]: a["message"]
                    for a in c.data["commands"].values()
                }
                if c.data["enabled"]
                else {}
            )
            for robot in TARGETS:
                payload, extra = {}, {}
                if robot in active:
                    payload = {
                        "expected_boot_version": active[robot]["payload"][
                            "expected_boot_version"
                        ],
                        "keep_running_command_id": active[robot]["command_id"],
                    }
                    extra = {"expires_at": expiry(3)}
                self.publish("robot_state_request", robot, payload, **extra)
            self.last_poll = now
        action = c.next_action()
        if action:
            request_id, (target, kind, payload) = action
            message = wire.build_message(
                kind,
                c.config["control_id"],
                target,
                payload,
                command_id=str(uuid4()),
                expires_at=expiry(),
            )
            c.dispatched(request_id, message)
            try:
                self.transport.publish(wire.topic(target, "cmd"), message)
            except (ConnectionError, TimeoutError):
                c.hold("command publish outcome unknown")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--journal", required=True)
    parser.add_argument("--socket", required=True)
    args = parser.parse_args()
    with open(args.config) as file:
        config = json.load(file)
    if config.get("mode") != "simulation":
        parser.error("a file-journal fleet requires simulation mode")
    connection = mqtt_config(config["control_id"])
    if connection.host not in {
        "localhost",
        "127.0.0.1",
        "::1",
    }:
        parser.error("simulation requires isolated loopback broker")
    with ExitStack() as stack:
        store = Journal(args.journal)
        stack.callback(store.close)
        control = FleetControl(config, store)
        transport = MqttTransport(
            connection,
            [wire.topic(r, "event") for r in TARGETS]
            + [wire.topic(r, "motion") for r in ROLES],
        )
        transport.start()
        stack.callback(transport.stop)
        web = FleetSocket(args.socket, control)
        stack.callback(web.close)
        runtime = FleetRuntime(control, transport)

        def terminate(*_):
            raise KeyboardInterrupt

        previous = signal.signal(signal.SIGTERM, terminate)
        stack.callback(signal.signal, signal.SIGTERM, previous)
        print("Fleet owner ready; enable explicitly in the web UI.", flush=True)
        try:
            while True:
                runtime.poll()
                web.poll()
        except KeyboardInterrupt:
            control.hold("owner_stopped")


if __name__ == "__main__":
    main()
