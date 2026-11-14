"""Customer HTTP -> WSS -> operational DB -> MQTT/robot drivers, stopping at Nav2 input.

Explicit local integration runner, not the default unit suite. Only sensors/motors
and Nav2 I/O are simulated; progress comes exclusively from FleetRuntime events.
"""

import argparse
import asyncio
import getpass
import http.cookiejar
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from contextlib import ExitStack, contextmanager
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from pathlib import Path
from unittest.mock import patch
from uuid import uuid4

import psycopg

SERVICE = Path(__file__).resolve().parents[1]
REPO = SERVICE.parents[1]
sys.path.insert(0, str(SERVICE / "scripts"))
import test_fleet_mqtt as fleet_test
from mqtt_helpers import local_broker
from test_omx_device_mqtt import panel_device_executor, runner_executor
from virtual_pinky import SimulatedNav, VirtualPinky

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    ServiceTaskStore,
    migrate_service,
)
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.wss.service import ServiceLink
from roport_control_server.executor.fleet_runtime import service_tick


class CaptureNav(SimulatedNav):
    """Record the real driver's Nav2 request without generating arrival/completion."""

    def __init__(self, pose, *, speed_mps):
        super().__init__(pose, speed_mps=speed_mps)
        self.received = []

    def start(self, payload):
        self.received.append(deepcopy(payload))
        super().start(payload)

    def tick(self, dt):
        self.observed_at = (
            time.time()
        )  # Readiness stays live; pose/result never advance.


class BoundaryPinky(VirtualPinky):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.nav = CaptureNav(self.map_pose(), speed_mps=self.nav.speed_mps)


def free_port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, default=str) + "\n")


@contextmanager
def databases(pg_bin, output):
    with tempfile.TemporaryDirectory(prefix="rp-customer-e2e-") as folder:
        root = Path(folder)
        port = free_port()
        subprocess.run(
            [
                str(pg_bin / "initdb"),
                "-D",
                str(root / "data"),
                "-A",
                "trust",
                "--no-locale",
                "--encoding=UTF8",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        subprocess.run(
            [
                str(pg_bin / "pg_ctl"),
                "-D",
                str(root / "data"),
                "-l",
                str(root / "postgres.log"),
                "-o",
                f"-k {root} -h 127.0.0.1 -p {port}",
                "-w",
                "start",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        try:
            for name in ("service_test", "control_test"):
                subprocess.run(
                    [
                        str(pg_bin / "createdb"),
                        "-h",
                        "127.0.0.1",
                        "-p",
                        str(port),
                        name,
                    ],
                    check=True,
                )
            yield port
        finally:
            subprocess.run(
                [
                    str(pg_bin / "pg_ctl"),
                    "-D",
                    str(root / "data"),
                    "-w",
                    "stop",
                    "-m",
                    "fast",
                ],
                check=True,
                stdout=subprocess.DEVNULL,
            )
            shutil.copyfile(root / "postgres.log", output / "postgres.log")


@contextmanager
def java_service(pg_port, output):
    project = REPO / "services/roport-service-server"
    with (output / "java-build.log").open("w") as log:
        subprocess.run(
            ["./gradlew", "--no-daemon", "bootJar"],
            cwd=project,
            check=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    port, token = free_port(), uuid4().hex
    env = {
        k: v for k, v in os.environ.items() if not k.startswith(("SERVICE_", "SERVER_"))
    }
    env.update(
        SERVICE_DB_URL=f"jdbc:postgresql://127.0.0.1:{pg_port}/service_test",
        SERVICE_DB_USER=getpass.getuser(),
        SERVICE_DB_PASSWORD="",
        SERVICE_CONTROL_ENABLED="true",
        SERVICE_CONTROL_SITE="E2E",
        SERVICE_CONTROL_TOKEN=token,
        SERVICE_DEV_LOGIN="true",
        SERVER_ADDRESS="127.0.0.1",
        SERVER_PORT=str(port),
    )
    with (output / "java-service.log").open("w") as log:
        process = subprocess.Popen(
            [
                "java",
                "-jar",
                str(project / "build/libs/roport-service-server-0.1.0-SNAPSHOT.jar"),
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        url = f"http://127.0.0.1:{port}"
        try:
            deadline = time.monotonic() + 30
            while True:
                if process.poll() is not None:
                    raise RuntimeError("Java service exited; see java-service.log")
                try:
                    with urllib.request.urlopen(
                        url + "/actuator/health", timeout=1
                    ) as response:
                        if response.status == 200:
                            break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Java service startup timeout") from None
                    time.sleep(0.1)
            yield url, token
        finally:
            process.terminate()
            process.wait(10)


def seed_customer(dsn, package):
    """Seed account/order/catalog only; shelf and task progress come from robot events."""
    now = datetime.now(UTC)
    pickup = datetime.fromtimestamp(
        ((int((now + timedelta(hours=1)).timestamp()) // 1800) + 1) * 1800, UTC
    )
    boarding = now + timedelta(hours=4)
    with psycopg.connect(dsn) as db:
        db.execute(
            "INSERT INTO members(member_id,login_id) VALUES('e2e-member','e2e-login')"
        )
        db.execute(
            "INSERT INTO orders(order_id,order_no,member_id,site_id,revision,business_status,ordered_at) VALUES('e2e-order','e2e-order','e2e-member','E2E',0,'CONFIRMED',now())"
        )
        db.execute(
            "INSERT INTO pickup_locations(location_id,site_id,name,control_point_ref,is_active) VALUES('e2e-location','E2E','Test destination','PUBLIC_SPACE_TRIAL',true)"
        )
        db.execute(
            "INSERT INTO flights(flight_id,flight_no,service_date,boarding_start,boarding_end,departure_at,gate_code,revision) VALUES('e2e-flight','e2e-flight',CURRENT_DATE,%s,%s,%s,'G1',0)",
            (boarding, boarding, boarding),
        )
        db.execute(
            "INSERT INTO packages(package_id,order_id,package_revision,label_ref,manifest_locked_at) VALUES(%s,'e2e-order',0,'test-only',now())",
            (package,),
        )
    return {
        "orderId": "e2e-order",
        "locationId": "e2e-location",
        "flightId": "e2e-flight",
        "pickupAt": pickup.isoformat(),
    }


def reserve(url, request):
    client = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar())
    )

    def post(path, body):
        with client.open(
            urllib.request.Request(
                url + path,
                data=json.dumps(body).encode(),
                headers={"Content-Type": "application/json"},
            ),
            timeout=5,
        ) as response:
            return json.load(response)

    post("/api/session", {"loginId": "e2e-login"})
    return post("/api/pickups", request)


@contextmanager
def service_link(url, token, store):
    stop, errors = threading.Event(), []
    link = ServiceLink(
        url.replace("http://", "ws://") + "/ws/v1/sites/E2E/control",
        token,
        "E2E",
        store,
    )

    def run():
        try:
            asyncio.run(link.run(stop))
        except Exception as exc:  # noqa: BLE001 - propagate a worker failure to the test
            errors.append(exc)

    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    try:
        yield errors
    finally:
        stop.set()
        worker.join(5)
        if worker.is_alive():
            raise RuntimeError("service link did not stop")


def exercise(output, pg_port, url, token, case):
    service_dsn = f"postgresql://127.0.0.1:{pg_port}/service_test"
    control_dsn = f"postgresql://127.0.0.1:{pg_port}/control_test"
    migrate_service(control_dsn)
    store = ServiceTaskStore(control_dsn)
    package = {"left": "small-left", "right": "small-right", "large": "large"}[case]
    customer_request = seed_customer(service_dsn, package)

    def storage(path):
        return (
            OperationalStore(control_dsn)
            if path.name == "control.json"
            else Journal(path)
        )

    devices, panels = {}, []

    def executors(*_args, **_kwargs):
        prepared = fleet_test.virtual_executors_original(
            0.01, package_sizes={"large": "large"}
        )
        markers = {name: 17 for name in [package, "fill-left-1", "fill-left-2"]}
        for target, column in [("omx3", None), ("omx1", "a"), ("omx2", "c")]:
            device = runner_executor(
                target,
                routes=[
                    (direction, str(row) + column)
                    for direction in ["INBOUND", "OUTBOUND"]
                    for row in [1, 2]
                ]
                if column
                else (),
                marker_ids=markers,
                package_sizes={
                    name: "large" if name == "large" else "small" for name in markers
                },
                versions={
                    "model_version": "sim-model",
                    "calibration_version": "sim-cal",
                },
            )
            devices[target] = device
            prepared[target] = device.executor
        panel = panel_device_executor(
            stack,
            package_id=package,
            versions={"model_version": "sim-model", "calibration_version": "sim-cal"},
            arrival_observation=lambda task: env.host.pinkies[task["pinky_id"]][
                "state"
            ]["arrival_command_id"],
        )
        panels.append(panel)
        prepared["shelf-01"] = panel.executor
        return prepared

    with ExitStack() as stack:
        broker = stack.enter_context(local_broker(output))
        # Existing MQTT integration fixture, substituting a real PostgreSQL store
        # and a Nav2 observation boundary; command/state processing is unchanged.
        stack.enter_context(patch.object(fleet_test, "Journal", storage))
        stack.enter_context(patch.object(fleet_test, "VirtualPinky", BoundaryPinky))
        # Preserve the shared test factory only for the unused/cooperative endpoint.
        stack.enter_context(
            patch.object(
                fleet_test,
                "virtual_executors_original",
                fleet_test.virtual_executors,
                create=True,
            )
        )
        stack.enter_context(patch.object(fleet_test, "virtual_executors", executors))
        environment = fleet_test.fleet_mqtt.__wrapped__(broker, output)
        env = next(environment)
        stack.callback(environment.close)
        errors = stack.enter_context(service_link(url, token, store))
        control = env.control
        cart = next(client for client in env.clients if client.robot == "pinky3")
        last_tick = -float("inf")

        def pump(predicate, timeout=90):
            def check():
                nonlocal last_tick
                assert not errors, errors
                for panel in panels:
                    panel.tick()
                if time.monotonic() - last_tick >= 0.15:
                    service_tick(control, store)
                    last_tick = time.monotonic()
                return predicate()

            env.pump(check, timeout=timeout)

        inbound = (
            ["fill-left-1", "fill-left-2", package] if case == "right" else [package]
        )
        for item in inbound:
            request_id = env.submit(item, "inbound")
            pump(
                lambda request_id=request_id: (
                    control.data["tasks"][request_id]["status"] == "COMPLETE"
                )
            )
        slot = control.data["tasks"]["inbound-" + package]["slot"]
        assert slot.endswith({"left": "a", "right": "c", "large": "b"}[case])
        assert control.data["slots"][slot] == {
            "state": "occupied",
            "package_id": package,
        }
        write_json(output / "inbound-complete.json", control.data)

        reservation = reserve(url, customer_request)
        pump(lambda: bool(store.pending_tasks()))
        task = store.pending_tasks()[0]
        assert task["request"]["reservation_id"] == reservation["pickupId"]
        assert task["request"]["packages"] == [{"package_id": package, "quantity": 1}]
        child_id = task["task_id"] + ":" + package
        pump(lambda: bool(cart.driver.nav.received))
        assert len(cart.driver.nav.received) == 1
        goal = cart.driver.nav.received[0]
        assert goal["pose"] == control.config["goals"]["PUBLIC_SPACE_TRIAL"]["pose"]
        assert cart.driver.mode == "nav2" and cart.driver.nav.result is None
        assert control.data["tasks"][child_id]["stage"] == "navigate"
        assert control.data["tasks"][child_id]["status"] != "COMPLETE"
        assert control.data["slots"][slot]["state"] == "empty"

        def service_reports_navigation():
            with psycopg.connect(service_dsn) as db:
                row = db.execute(
                    "SELECT state,progress_payload FROM control_task_views WHERE task_ref=%s",
                    (task["task_id"],),
                ).fetchone()
                return (
                    row
                    and row[0] == "RUNNING"
                    and row[1]["progress"]["stages"] == ["navigate"]
                )

        pump(service_reports_navigation)
        pump(lambda: not store.outbox())
        with psycopg.connect(service_dsn) as db:
            service_state = db.execute(
                "SELECT state,progress_payload FROM control_task_views WHERE task_ref=%s",
                (task["task_id"],),
            ).fetchone()
            assert (
                db.execute(
                    "SELECT business_status FROM orders WHERE order_id='e2e-order'"
                ).fetchone()[0]
                == "CONFIRMED"
            )
            assert db.execute("SELECT count(*) FROM receipts").fetchone()[0] == 0
            assert (
                db.execute(
                    "SELECT count(*) FROM control_link_outbox WHERE acknowledged_at IS NULL"
                ).fetchone()[0]
                == 0
            )
            service_events = [
                row[0]
                for row in db.execute(
                    "SELECT message FROM control_link_inbox ORDER BY received_at"
                ).fetchall()
            ]
        commands = [
            record["command"] for record in cart.journal.data["records"].values()
        ]
        assert [cmd["type"] for cmd in commands][-2:] == [
            "set_drive_mode_command",
            "nav_goal_command",
        ]
        assert any(
            cmd["type"] == "line_route_command"
            and cmd["payload"]["goal"]["node"] == "N12"
            for cmd in commands
        )
        assert all(event["type"] != "task.completed" for event in service_events)
        result = {
            "passed": True,
            "case": case,
            "package": package,
            "slot": slot,
            "customer_input": "authenticated development login + actual POST /api/pickups",
            "service_task_id": task["task_id"],
            "control_child_task_id": child_id,
            "service_state": service_state[0],
            "stage": "navigate",
            "nav_goal_received_by_actual_trial_driver": goal,
            "nav_goal_completion_generated": False,
            "customer_receipt_count": 0,
            "mqtt_cart_command_types": [cmd["type"] for cmd in commands],
            "omx_boundary": "real FieldOmxHost + actual Warehouse/IndependentArm Runner + PanelFieldExecutor/HTTP PanelExecutor; fake device/vision ports",
            "omx_panel_starts": panels[0].panel.starts,
            "omx_device_actions": {
                target: {
                    "actions": device.robot.sent,
                    "held": device.robot.held,
                    "policy_steps": device.policy.n,
                    "marker_calls": len(device.markers),
                }
                for target, device in devices.items()
            },
            "hardware_connected": False,
        }
        write_json(output / "result.json", result)
        write_json(
            output / "customer-request.json",
            {"request": customer_request, "response": reservation},
        )
        write_json(output / "service-events.json", service_events)
        write_json(output / "control-snapshot.json", control.data)
        write_json(output / "cart-commands.json", commands)
        write_json(
            output / "profiles.json", {"control": control.config, "robots": env.robots}
        )
        print(json.dumps(result, ensure_ascii=False, default=str), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--case", choices=("left", "right", "large"), default="left")
    parser.add_argument("--pg-bin", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        with (
            databases(args.pg_bin, args.output) as pg_port,
            java_service(pg_port, args.output) as (url, token),
        ):
            exercise(args.output, pg_port, url, token, args.case)
    except BaseException as exc:
        write_json(
            args.output / "failure.json",
            {"type": type(exc).__name__, "detail": str(exc)},
        )
        raise


if __name__ == "__main__":
    main()
