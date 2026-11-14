"""Actual fleet owner/socket/API with synthetic status; no MQTT or robot motion."""

import json
import signal
import socket
import sys
import tempfile
import threading
import time
from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import uvicorn

from roport_control_server.adapters.api.app import create_app
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.executor.fleet import TARGETS, FleetControl
from roport_control_server.executor.fleet_socket import FleetSocket

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "tools"))
from prepare_pinky_trial import prepare

MAP = SERVICE.parent / "roport-pinky/maps/airport_map.yaml"
LAYOUT = SERVICE.parent / "roport-pinky/src/roport_gz_world/layout/roport_airport.yaml"
TOKEN = "fleet-test-only-operation-key"


@contextmanager
def owner(folder, mode="simulation"):
    profile = prepare(MAP, LAYOUT, simulation=True, single_robot=False)
    goal = {
        k: v
        for k, v in profile["nav_goal"].items()
        if k not in {"goal_version", "map_id"}
    }
    config = {
        **profile,
        "mode": mode,
        "goals": {"gate": goal},
        "n12_goal": goal,
        "omx": {
            r: {"model_version": "test-model", "calibration_version": "test-cal"}
            for r in TARGETS
            if not r.startswith("pinky")
        },
    }
    journal = Journal(folder / "fleet.json")
    control = FleetControl(config, journal)
    for robot in TARGETS:
        node = {"pinky1": "N16", "pinky2": "N17", "pinky3": "H1"}.get(robot)
        state = {
            "stopped": True,
            "fault": None,
            "ready": True,
            "home_reached": True,
            "holding_state": "empty",
            "holding_package_id": None,
        }
        if node:
            state.update(
                node=node,
                mode="line",
                line_enabled=False,
                heading_rad=0,
                arrival_command_id=None,
                nav2_ready=True,
                localization_ready=True,
                drive_owner="none",
                map_version=config["map_version"],
                graph_version=config["graph_version"],
            )
        control.observed[robot] = {
            "boot_version": robot + "-test-boot",
            "sequence": 1,
            "active_command_id": None,
            "state": state,
        }
        control.heard[robot] = time.monotonic()
    channel = FleetSocket(folder / "s", control)
    done, fresh = threading.Event(), threading.Event()
    fresh.set()
    freshness_lock = threading.Lock()

    def loop():
        while not done.wait(0.005):
            with freshness_lock:
                if fresh.is_set():
                    control.heard.update(dict.fromkeys(TARGETS, time.monotonic()))
            channel.poll()

    worker = threading.Thread(target=loop)
    worker.start()
    app = create_app(
        MAP, fleet_socket=channel.path, trial_layout=LAYOUT, trial_token=TOKEN
    )
    try:
        yield SimpleNamespace(
            app=app,
            control=control,
            channel=channel,
            fresh=fresh,
            freshness_lock=freshness_lock,
            done=done,
        )
    finally:
        done.set()
        worker.join(2)
        channel.close()
        journal.close()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "simulation"
    with (
        tempfile.TemporaryDirectory(prefix="rp-fleet-web-") as folder,
        owner(Path(folder), mode) as fixture,
    ):
        if "--receipt" in sys.argv:
            # Seed a completed physical delivery to test only the receipt boundary.
            data = deepcopy(fixture.control.data)
            data["slots"]["1a"] = {"state": "occupied", "package_id": "delivered-box"}
            fixture.control.save(data)
            fixture.control.submit(
                {
                    "request_id": "await-receipt",
                    "operation": "outbound",
                    "package_id": "delivered-box",
                    "goal": "gate",
                }
            )
            data = deepcopy(fixture.control.data)
            data["tasks"]["await-receipt"].update(
                stage="await_receipt", status="AWAITING_RECEIPT"
            )
            data["slots"]["1a"] = {"state": "empty", "package_id": None}
            fixture.control.save(data)
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        server = uvicorn.Server(
            uvicorn.Config(
                fixture.app, host="127.0.0.1", port=port, log_level="warning"
            )
        )
        worker = threading.Thread(target=server.run)
        worker.start()

        def stop(*_):
            server.should_exit = True

        def toggle_freshness(*_):
            if fixture.fresh.is_set():
                fixture.fresh.clear()
            else:
                fixture.fresh.set()

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGUSR1, toggle_freshness)
        try:
            deadline = time.monotonic() + 10
            while not server.started:
                if not worker.is_alive() or time.monotonic() >= deadline:
                    raise RuntimeError("fleet API did not start")
                time.sleep(0.02)
            print(
                json.dumps({"url": f"http://127.0.0.1:{port}", "token": TOKEN}),
                flush=True,
            )
            worker.join()
        finally:
            server.should_exit = True
            worker.join(5)


if __name__ == "__main__":
    main()
