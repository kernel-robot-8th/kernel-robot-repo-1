"""One MQTT owner for request-driven fleet work; real and virtual endpoints use v2."""

import argparse
import asyncio
import json
import os
import signal
import threading
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
            self.control.observe(event)
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
            if not c.data["fault"] or command.get("stop"):
                continue
            active = command["message"]
            stop = wire.build_message(
                "robot_stop_command",
                c.config["control_id"],
                active["target"],
                {
                    "expected_boot_version": active["payload"]["expected_boot_version"],
                    "target_command_id": active["command_id"],
                    "reason": "control_hold",
                },
                command_id=str(uuid4()),
                expires_at=expiry(),
            )
            data = deepcopy(c.data)
            data["commands"][cid]["stop"] = stop
            c.save(data)
            self.transport.publish(wire.topic(active["target"], "cmd"), stop)
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


def service_tick(control, store):
    for task in store.pending_tasks():
        payload = task["request"]
        if task["state"] == "WAITING_MANIFEST":
            continue
        children = []
        try:
            if (
                payload["operation"] != "prepare_order"
                and payload["pickup_point_id"] not in control.config["goals"]
            ):
                raise ValueError("unknown calibrated destination")
            requests = [
                {
                    "request_id": task["task_id"] + ":" + package["package_id"],
                    "operation": "inbound"
                    if payload["operation"] == "prepare_order"
                    else "outbound",
                    "package_id": package["package_id"],
                    "goal": None
                    if payload["operation"] == "prepare_order"
                    else payload["pickup_point_id"],
                }
                for package in payload["packages"]
            ]
            # Reject an invalid manifest before creating any robot work.
            for request in requests:
                control.validate_request(request)
            for request in requests:
                children.append(control.submit(request))
            if children:
                state = (
                    "COMPLETED"
                    if all(t["status"] == "COMPLETE" for t in children)
                    else "HOLD"
                    if any(t["status"] == "HOLD" for t in children)
                    else "QUEUED"
                    if all(t["status"] == "QUEUED" for t in children)
                    else "RUNNING"
                )
                store.update(
                    task["task_id"], state, {"stages": [t["stage"] for t in children]}
                )
        except ValueError as exc:
            store.update(task["task_id"], "HOLD", {"reason": str(exc)})
    for receipt in store.receipts():
        for request_id, child in list(control.data["tasks"].items()):
            if (
                request_id.startswith(receipt["task_id"] + ":")
                and child["request"]["package_id"]
                in receipt["message"]["payload"]["package_ids"]
                and child["stage"] == "await_receipt"
            ):
                control.receipt(request_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--journal", help="simulation only; operational state uses ROPORT_DATABASE_URL"
    )
    parser.add_argument("--socket", required=True)
    parser.add_argument("--hardware-authorized", action="store_true")
    args = parser.parse_args()
    with open(args.config) as file:
        config = json.load(file)
    if config.get("mode") not in {"simulation", "hardware"}:
        parser.error("mode must be simulation or hardware")
    if config["mode"] == "hardware" and (
        not args.hardware_authorized or config.get("field_verified") is not True
    ):
        parser.error("hardware requires field verification and explicit authorization")
    connection = mqtt_config(config["control_id"])
    if config["mode"] == "simulation" and connection.host not in {
        "localhost",
        "127.0.0.1",
        "::1",
    }:
        parser.error("simulation requires isolated loopback broker")
    if args.journal and config["mode"] != "simulation":
        parser.error("file journal is for simulation; use operational PostgreSQL")
    with ExitStack() as stack:
        if args.journal:
            store = Journal(args.journal)
        else:
            from roport_control_server.adapters.db.service_tasks import OperationalStore

            store = OperationalStore(os.environ["ROPORT_DATABASE_URL"])
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
        service_store = None
        if os.environ.get("ROPORT_SERVICE_WSS_URL"):
            from roport_control_server.adapters.db.service_tasks import ServiceTaskStore
            from roport_control_server.adapters.wss.service import ServiceLink

            service_store = ServiceTaskStore(os.environ["ROPORT_DATABASE_URL"])
            stop = threading.Event()
            link = ServiceLink(
                os.environ["ROPORT_SERVICE_WSS_URL"],
                os.environ["ROPORT_SERVICE_TOKEN"],
                os.environ["ROPORT_SERVICE_SITE"],
                service_store,
            )
            thread = threading.Thread(
                target=lambda: asyncio.run(link.run(stop)), daemon=True
            )
            thread.start()
            stack.callback(stop.set)

        def terminate(*_):
            raise KeyboardInterrupt

        previous = signal.signal(signal.SIGTERM, terminate)
        stack.callback(signal.signal, signal.SIGTERM, previous)
        print("Fleet owner ready; enable explicitly in the web UI.", flush=True)
        last_service_poll = -float("inf")
        try:
            while True:
                runtime.poll()
                web.poll()
                if service_store and monotonic() - last_service_poll >= 1:
                    service_tick(control, service_store)
                    last_service_poll = monotonic()
        except KeyboardInterrupt:
            control.hold("owner_stopped")


if __name__ == "__main__":
    main()
