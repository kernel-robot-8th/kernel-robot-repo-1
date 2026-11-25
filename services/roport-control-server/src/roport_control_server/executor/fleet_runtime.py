"""One MQTT owner for request-driven fleet work; real and virtual endpoints use v2."""

import argparse
import asyncio
import json
import logging
import os
import signal
import threading
from concurrent.futures import ThreadPoolExecutor
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
from roport_control_server.contracts.service_policy import dispatch_at

from .fleet import ROLES, TARGETS, FleetControl
from .fleet_recovery import new_stop, stop_current
from .fleet_socket import FleetSocket

LOG = logging.getLogger(__name__)


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
            LOG.warning(
                "Rejected %s from %s: clock_skew_or_delay "
                "(observation_age_s=%.3f; allowed=-1..3)",
                event["type"],
                source,
                age,
            )
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
            if p.get("result", {}).get("evidence") == "shelf_placed":
                self.control.shelf_placement_acked(
                    event["command_id"], event["message_id"]
                )

    def stored_result(self, event):
        c = self.control
        release = c.data.get("host_releases", {}).get(event["command_id"])
        if release:
            cmd = release["command"]
            p = event["payload"]
            if (
                event["type"] != "command_status"
                or event["source"] != cmd["target"]
                or p["status"] not in {"completed", "rejected"}
                or (
                    p["status"] == "completed"
                    and (
                        p["boot_version"] != cmd["payload"]["expected_boot_version"]
                        or p.get("result", {}).get("released_hold")
                        != cmd["payload"]["hold_command_id"]
                    )
                )
            ):
                raise ValueError("host release result mismatch")
            if release["status"] not in {"pending", "unconfirmed"}:
                return c.data["events"].get(event["message_id"]) == event
            data = deepcopy(c.data)
            data["host_releases"][event["command_id"]]["status"] = p["status"]
            data["events"][event["message_id"]] = event
            c.save(data)
            return True
        stop = next(
            (
                a["stop"]
                for a in c.data["commands"].values()
                if a.get("stop", {}).get("command_id") == event["command_id"]
            ),
            None,
        )
        if stop is None:
            stop = c.data.get("follow_stops", {}).get(event["command_id"])
        if stop:
            p = event["payload"]
            if (
                event["source"] != stop["target"]
                or p["boot_version"] != stop["payload"]["expected_boot_version"]
            ):
                raise ValueError("stop response source/boot mismatch")
            previous = c.data["events"].get(event["message_id"])
            if previous:
                if previous != event:
                    raise ValueError("conflicting stop event")
                return True
            data = deepcopy(c.data)
            data["events"][event["message_id"]] = event
            cid = stop["payload"]["target_command_id"]
            command = data["commands"].get(cid)
            if (
                not data["fault"]
                and command
                and command.get("stop") == stop
                and stop["payload"]["reason"] == "follow_pause"
            ):
                status = p["status"]
                if status in {"rejected", "failed", "interrupted"}:
                    c.hold("follow_stop_" + status, data=data)
                    return True
                if status == "completed":
                    if p.get("result", {}).get("stopped") is not True:
                        c.hold("follow_stop_unconfirmed", data=data)
                        return True
                    if not stop_current(stop, p["boot_version"], datetime.now(UTC)):
                        c.hold("follow_stop_expired", data=data)
                        return True
                    command["pause_stop_completed_at"] = p["observed_at"]
                    if command.get("pause_interrupted"):
                        c.pause_follow(data, cid, p["observed_at"])
            c.save(data)
            return True
        return c.result(event)

    def publish_stops(self):
        c = self.control
        for cid, command in list(c.data["commands"].items()):
            pause = (
                not c.data["fault"]
                and c.data["tasks"][command["task"]]["stage"] == "follow_stopping"
            )
            stop = command.get("stop")
            if pause and command.get("follow_stopped_at"):
                continue  # Terminal result received; keep the reservation until fresh idle state.
            if (
                pause
                and stop
                and not stop_current(
                    stop,
                    c.observed.get(stop["target"], {}).get("boot_version"),
                    datetime.now(UTC),
                )
            ):
                c.hold("follow_stop_expired")
                continue
            if (
                (not c.data["fault"] and not pause)
                or command.get("stop_published")
                or command.get("manual_stop")
            ):
                continue
            active = command["message"]
            target = active["target"]
            boot = active["payload"]["expected_boot_version"]
            observed_boot = c.observed.get(target, {}).get("boot_version", boot)
            if stop is None:
                if active["target"] in ROLES and any(
                    e.get("command_id") == cid
                    and e["payload"]["status"]
                    in {"completed", "rejected", "failed", "interrupted"}
                    for e in c.data["events"].values()
                ):
                    continue  # Pinky has already ended this command; recovery still checks idle state.
                stop = new_stop(
                    c.config["control_id"],
                    active,
                    boot,
                    reason="follow_pause" if pause else "control_hold",
                )
                data = deepcopy(c.data)
                data["commands"][cid]["stop"] = stop
                if pause:
                    data.setdefault("follow_stops", {})[stop["command_id"]] = stop
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
        self.publish_stops()
        now = monotonic()
        if now - self.last_poll >= 1:
            for request_id, release in c.data.get("host_releases", {}).items():
                if release["status"] != "pending":
                    continue
                command = release["command"]
                if wire.timestamp(command["expires_at"]) <= datetime.now(UTC):
                    data = deepcopy(c.data)
                    data["host_releases"][request_id]["status"] = "unconfirmed"
                    c.save(data)
                else:
                    self.transport.publish(wire.topic("shelf-01", "cmd"), command)
            active = (
                {
                    a["message"]["target"]: a["message"]
                    for a in c.data["commands"].values()
                    if c.data["tasks"][a["task"]]["stage"] != "follow_stopping"
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


def service_tick(control, store, *, now=None):
    now = now or datetime.now(UTC)
    for receipt in store.receipts():
        payload = receipt["message"]["payload"]
        for request_id, child in list(control.data["tasks"].items()):
            if (
                request_id.startswith(receipt["task_id"] + ":")
                and child["request"]["package_id"] in payload["package_ids"]
                and child["stage"] == "await_receipt"
            ):
                try:
                    control.receipt(
                        request_id,
                        payload.get("follow_card"),
                        follow_mode=payload.get("follow_mode"),
                        session_id=payload["receipt_id"],
                    )
                except ValueError as exc:
                    control.hold(str(exc))
    for request in store.follow_requests():
        try:
            control.follow_request(
                request["message"]["payload"], sent_at=request["message"]["sent_at"]
            )
        except ValueError:
            LOG.warning("follow_request_rejected")
        store.follow_processed(request["request_id"])
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
                    **(
                        {"marker": package["marker"]}
                        if package.get("marker") is not None
                        else {}
                    ),
                    "goal": None
                    if payload["operation"] == "prepare_order"
                    else payload["pickup_point_id"],
                }
                for package in payload["packages"]
            ]
            # Reject an invalid manifest before creating any robot work.
            for request in requests:
                if control.config["mode"] == "hardware" and "marker" not in request:
                    raise ValueError(
                        "physical package marker missing from service manifest"
                    )
                control.validate_request(request)
            if (
                payload["operation"] == "customer_service"
                and not task["dispatch_locked"]
                and now < dispatch_at(payload, control.config.get("service_policy", {}))
            ):
                continue
            if store.claim(task["task_id"], payload["order_revision"]) is None:
                continue
            for request in requests:
                children.append(control.submit(request))
            if children:
                settled = all(t["status"] in {"COMPLETE", "CANCELED"} for t in children)
                state = (
                    "FAILED"
                    if settled and any(t["status"] == "CANCELED" for t in children)
                    else "COMPLETED"
                    if settled
                    else "HOLD"
                    if any(t["status"] == "HOLD" for t in children)
                    else "QUEUED"
                    if all(t["status"] == "QUEUED" for t in children)
                    else "RUNNING"
                )
                store.update(
                    task["task_id"],
                    state,
                    {
                        "stages": [t["stage"] for t in children],
                        "packages": [control.service_projection(t) for t in children],
                    },
                )
        except ValueError as exc:
            store.update(task["task_id"], "HOLD", {"reason": str(exc)})


def main(*, field_loader=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--journal", help="simulation file journal; omit for PostgreSQL"
    )
    parser.add_argument("--socket", required=True)
    parser.add_argument("--hardware-authorized", action="store_true")
    if field_loader:
        group = parser.add_mutually_exclusive_group()
        group.add_argument("--mock-omx", action="store_true")
        group.add_argument("--real-shelf", choices=("1b", "2b"))
        parser.add_argument(
            "--shelf-clear-confirmed", action="store_true", required=True
        )
    args = parser.parse_args()
    if field_loader:
        if not args.hardware_authorized or args.journal:
            parser.error("field owner requires hardware authorization and PostgreSQL")
        from pathlib import Path

        config = field_loader(
            Path(args.config), args.mock_omx, shelf_slot=args.real_shelf
        )
    else:
        with open(args.config) as file:
            config = json.load(file)
        if config.get("mode") != "simulation":
            parser.error(
                "standalone owner requires simulation; use the validated field launcher"
            )
    if os.environ.get("ROPORT_BOARDING_ALERT_TEST") == "1":
        config["boarding_alert_after_s"] = 20
    connection = mqtt_config(config["control_id"])
    if connection.host not in {
        "localhost",
        "127.0.0.1",
        "::1",
    }:
        parser.error("Fleet requires a local loopback broker (simulation or field)")
    database = os.environ.get("ROPORT_DATABASE_URL")
    service_url = os.environ.get("ROPORT_SERVICE_WSS_URL")
    if bool(args.journal) == bool(database):
        parser.error("choose --journal or ROPORT_DATABASE_URL")
    if service_url and (
        not database
        or not os.environ.get("ROPORT_SERVICE_TOKEN")
        or not os.environ.get("ROPORT_SERVICE_SITE")
    ):
        parser.error(
            "WSS requires PostgreSQL, ROPORT_SERVICE_TOKEN and ROPORT_SERVICE_SITE"
        )
    with ExitStack() as stack:
        service_store = link = None
        if database:
            from roport_control_server.adapters.db.service_tasks import (
                OperationalStore,
                ServiceTaskStore,
            )

            store = OperationalStore(database)
        else:
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
        if service_url:
            from roport_control_server.adapters.wss.service import ServiceLink

            service_store = ServiceTaskStore(database)
            link = ServiceLink(
                service_url,
                os.environ["ROPORT_SERVICE_TOKEN"],
                os.environ["ROPORT_SERVICE_SITE"],
                service_store,
            )
            worker = stack.enter_context(ThreadPoolExecutor(max_workers=1))
            stop = threading.Event()
            future = worker.submit(asyncio.run, link.run(stop))
            stack.callback(stop.set)

        def terminate(*_):
            raise KeyboardInterrupt

        previous = signal.signal(signal.SIGTERM, terminate)
        stack.callback(signal.signal, signal.SIGTERM, previous)
        print("Fleet owner ready; send enable through the local socket.", flush=True)
        last_service_poll = -float("inf")
        try:
            while True:
                runtime.poll()
                web.poll()
                if link and future.done():
                    future.result()  # Propagate storage/worker errors instead of silently losing WSS.
                    raise RuntimeError("service link owner ended")
                if service_store and monotonic() - last_service_poll >= 1:
                    service_tick(control, service_store)
                    last_service_poll = monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            control.hold("owner_stopped")
            if config["mode"] == "hardware" and control.data["commands"]:
                runtime.publish_stops()


if __name__ == "__main__":
    main()
