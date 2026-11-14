"""Original web UI + three actual Pinky drivers + virtual sensors + OMX + local MQTT.

All generated settings/journals stay in a new persistent run directory. No hardware.
"""

import argparse
import json
import math
import os
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from copy import deepcopy
from datetime import datetime
from pathlib import Path

from prepare_pinky_trial import prepare
from run_virtual_trial import LAYOUT, MAP, SERVICE, VIRTUAL_SPEED_MPS, free_port

from roport_control_server.adapters.api.maps import load_map
from roport_control_server.executor.plans import shortest_route


def profiles():
    trial = prepare(MAP, LAYOUT, simulation=True)
    for route in trial["routes"].values():
        route["speed_limit_mps"] = VIRTUAL_SPEED_MPS
    grid = load_map(MAP, LAYOUT)
    positions = {n["id"]: n["position"] for n in grid["layout"]["nodes"]}
    # The IR bar is 3 cm in front of the body; use the same graph return calibration.
    import yaml

    layout = yaml.safe_load(LAYOUT.read_text())
    yaw = math.radians(layout["frame"]["yaw_deg"])
    x, y = positions["N12"]
    goal = {
        k: v
        for k, v in trial["nav_goal"].items()
        if k not in {"goal_version", "map_id"}
    }
    back = {
        **goal,
        "goal_id": "N12_RETURN",
        "pose": {
            "x": x + 0.03 * math.cos(yaw),
            "y": y + 0.03 * math.sin(yaw),
            "yaw": math.atan2(math.sin(math.pi + yaw), math.cos(math.pi + yaw)),
        },
        "position_tolerance_m": 0.03,
        "yaw_tolerance_rad": 0.08,
    }
    fleet = {
        **trial,
        "node_positions": positions,
        # Two padded turning footprints + front IR and node-detection allowance.
        "node_clearance_m": 0.35,
        "speed_limit_mps": VIRTUAL_SPEED_MPS,
        "goals": {"PUBLIC_SPACE_TRIAL": goal},
        "n12_goal": back,
        "omx": {
            r: {"model_version": "sim-model", "calibration_version": "sim-cal"}
            for r in ["omx1", "omx2", "omx3", "shelf-01"]
        },
    }
    robots = {}
    for robot, role, start in [
        ("pinky1", "logistics", "N16"),
        ("pinky2", "logistics", "N17"),
        ("pinky3", "smartcart", "H1"),
    ]:
        p = deepcopy(trial)
        p["robots"][role]["robot_id"] = robot
        if robot == "pinky2":
            route = p["routes"]["to_loading"]
            route.update(
                start_node=start, route=shortest_route(p["nodes"], start, "N01")
            )
        robots[robot] = (role, p)
    return fleet, robots


def fetch(url):
    with urllib.request.urlopen(
        url + "/api/snapshot?mode=fleet", timeout=2
    ) as response:
        return json.load(response)["fleet"]


def send(url, key, state, action, **extra):
    request = urllib.request.Request(
        url + "/api/fleet/commands",
        json.dumps(
            {"action": action, "session_id": state["session_id"], **extra}
        ).encode(),
        headers={
            "Content-Type": "application/json",
            "Origin": url,
            "X-Roport-Trial-Token": key,
        },
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--mosquitto",
        default=os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto"),
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="explicitly run one small inbound/outbound/return; 1x physical timing",
    )
    parser.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args()
    if not args.mosquitto:
        parser.error("Mosquitto required")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", args.port))
        except OSError:
            parser.error(
                f"Port {args.port} is busy. Stop the previous run with Ctrl+C "
                "or choose another --port."
            )
    os.umask(0o077)
    root = (
        args.output
        or Path.home()
        / "roport-trial"
        / "fleet"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    ).resolve()
    root.mkdir(parents=True, mode=0o700, exist_ok=False)
    profile, robots = profiles()
    mqtt_port, token = free_port(), secrets.token_urlsafe(24)
    (root / "fleet.json").write_text(json.dumps(profile, indent=2))
    (root / "ui-key.txt").write_text(token + "\n")
    (root / "package-sizes.json").write_text(
        json.dumps({"package-small-1": "small", "package-large-1": "large"})
    )
    (root / "mosquitto.conf").write_text(
        f"listener {mqtt_port} 127.0.0.1\nallow_anonymous true\npersistence false\n"
    )
    env = {
        k: v
        for k, v in os.environ.items()
        if not k.startswith("ROPORT_") and k != "PYTHONPATH"
    }
    env.update(
        PYTHONPATH=str(SERVICE / "src"),
        ROPORT_MQTT_HOST="127.0.0.1",
        ROPORT_MQTT_PORT=str(mqtt_port),
        ROPORT_MQTT_TLS="0",
        ROPORT_MAP=str(MAP),
        ROPORT_MAP_LAYOUT=str(LAYOUT),
        ROPORT_FLEET_SOCKET=str(root / "f"),
        ROPORT_TRIAL_TOKEN=token,
    )
    processes, logs = [], []

    def start(name, command, extra=None):
        log = (root / (name + ".log")).open("w")
        logs.append(log)
        proc = subprocess.Popen(
            command,
            cwd=SERVICE,
            env={**env, **(extra or {})},
            stdout=log,
            stderr=log,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        processes.append((name, proc))

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    result = {"result": "interrupted"}
    try:
        start("broker", [args.mosquitto, "-c", str(root / "mosquitto.conf")])
        deadline = time.monotonic() + 10
        while True:
            try:
                with socket.create_connection(("127.0.0.1", mqtt_port), timeout=0.1):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise RuntimeError("broker startup timeout")
                time.sleep(0.05)
        for robot, (role, p) in robots.items():
            config = root / (robot + "-config.json")
            config.write_text(json.dumps(p, indent=2))
            start(
                robot,
                [
                    sys.executable,
                    "scripts/virtual_pinky.py",
                    "--config",
                    str(config),
                    "--role",
                    role,
                    "--journal",
                    str(root / (robot + ".json")),
                ],
            )
        start(
            "omx",
            [
                sys.executable,
                str(SERVICE.parent / "roport-omx/scripts/field_host.py"),
                "--mock",
                "--control",
                profile["control_id"],
                "--journal",
                str(root / "omx.json"),
                "--package-sizes",
                str(root / "package-sizes.json"),
            ],
            {"PYTHONPATH": str(SERVICE.parent / "roport-omx/src")},
        )
        start(
            "owner",
            [
                sys.executable,
                "-m",
                "roport_control_server.executor.fleet_runtime",
                "--config",
                str(root / "fleet.json"),
                "--journal",
                str(root / "control.json"),
                "--socket",
                str(root / "f"),
            ],
        )
        start(
            "api",
            [
                sys.executable,
                "-m",
                "uvicorn",
                "roport_control_server.adapters.api.app:app",
                "--host",
                "127.0.0.1",
                "--port",
                str(args.port),
            ],
        )
        url = f"http://127.0.0.1:{args.port}"
        info = {
            "url": url + "/ui/?mode=fleet",
            "key_file": str(root / "ui-key.txt"),
            "run_dir": str(root),
            "pids": {name: p.pid for name, p in processes},
            "simulation": True,
        }
        (root / "run.json").write_text(json.dumps(info, indent=2))
        print(json.dumps(info), flush=True)
        began = time.monotonic()
        submitted = False
        ready_shown = False
        expected_robots = robots.keys() | profile["omx"].keys()
        while True:
            for name, proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError(f"{name} exited: see {root / (name + '.log')}")
            try:
                state = fetch(url)
            except (OSError, ValueError, KeyError):
                if time.monotonic() - began > 20:
                    raise RuntimeError("UI startup timeout") from None
                time.sleep(0.1)
                continue
            if not ready_shown:
                if (
                    state.get("available")
                    and expected_robots <= state.get("robots", {}).keys()
                    and not state.get("fault")
                ):
                    print(
                        f"\n가상 관제 준비 완료 · 최신 로컬 코드: {SERVICE}\n"
                        f"웹 UI: {info['url']}\n"
                        f"기록: {root}\n"
                        "종료·코드 수정 후 재실행: Ctrl+C → 같은 명령 실행\n"
                        f"조작 키: {token}",
                        flush=True,
                    )
                    ready_shown = True
                elif time.monotonic() - began > 20:
                    raise RuntimeError(f"Fleet startup timeout: see {root}")
            if args.verify and state.get("available"):
                if state["fault"]:
                    raise RuntimeError(state["fault"])
                if not submitted and len(state["robots"]) == 7:
                    send(
                        url,
                        token,
                        state,
                        "submit",
                        request={
                            "request_id": "verify-in",
                            "operation": "inbound",
                            "package_id": "package-small-1",
                            "goal": None,
                        },
                    )
                    send(url, token, state, "enable")
                    submitted = True
                tasks = state["tasks"]
                if (
                    tasks.get("verify-in", {}).get("status") == "COMPLETE"
                    and "verify-out" not in tasks
                ):
                    send(
                        url,
                        token,
                        state,
                        "submit",
                        request={
                            "request_id": "verify-out",
                            "operation": "outbound",
                            "package_id": "package-small-1",
                            "goal": "PUBLIC_SPACE_TRIAL",
                        },
                    )
                if tasks.get("verify-out", {}).get("stage") == "await_receipt":
                    send(url, token, state, "receipt", request_id="verify-out")
                if tasks.get("verify-out", {}).get("status") == "COMPLETE":
                    result = {"result": "passed", "tasks": tasks}
                    break
                if time.monotonic() - began > args.timeout:
                    raise RuntimeError("verification timeout")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        result = {"result": "failed", "reason": str(exc)}
        raise
    finally:
        (root / "outcome.json").write_text(json.dumps(result, indent=2))
        for _, process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(5)
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
