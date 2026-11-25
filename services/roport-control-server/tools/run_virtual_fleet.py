"""Original web UI + three actual Pinky drivers + virtual sensors + OMX + local MQTT.

All generated settings/journals stay in a new persistent run directory. No hardware.
"""

import argparse
import json
import os
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from urllib.parse import urlsplit

from fleet_profiles import profiles
from run_virtual_trial import LAYOUT, MAP, SERVICE, free_port


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
    parser.add_argument(
        "--d660-line-loss",
        action="store_true",
        help="replay the N10 to N18 IR gap on simulated pinky1",
    )
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--mosquitto",
        default=os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto"),
    )
    parser.add_argument(
        "--verify",
        nargs="?",
        choices=("small", "large"),
        const="small",
        help="explicitly run inbound/outbound/return (default: small); 1x timing",
    )
    parser.add_argument(
        "--service-env", type=Path, help="isolated service launcher configuration"
    )
    parser.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args()
    if args.d660_line_loss and (args.verify or args.service_env):
        parser.error("use the line-loss replay with a standalone simulation")
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
    if args.d660_line_loss:
        profile["speed_limit_mps"] = 0.06  # d660 field command limit
    service_env = {}
    if args.service_env:
        service_env = json.loads(args.service_env.read_text())
        if set(service_env) - {"ROPORT_FOLLOW_PREVIEW_TOKEN"} != {
            "ROPORT_DATABASE_URL",
            "ROPORT_SERVICE_WSS_URL",
            "ROPORT_SERVICE_TOKEN",
            "ROPORT_SERVICE_SITE",
        }:
            parser.error(
                "service-env requires the four service settings and optional preview token"
            )
        db = urlsplit(service_env["ROPORT_DATABASE_URL"])
        link = urlsplit(service_env["ROPORT_SERVICE_WSS_URL"])
        if (
            db.hostname not in {"localhost", "127.0.0.1", "::1"}
            or not db.path.endswith("_test")
            or link.hostname not in {"localhost", "127.0.0.1", "::1"}
        ):
            parser.error(
                "service simulation requires loopback and an isolated _test DB"
            )
        # Test reservations are in the future; this run explicitly dispatches immediately.
        profile["service_policy"] = {"dispatch_lead_s": 86400}
    mqtt_port, token = free_port(), secrets.token_urlsafe(24)
    (root / "fleet.json").write_text(json.dumps(profile, indent=2))
    (root / "ui-key.txt").write_text(token + "\n")
    (root / "package-sizes.json").write_text(
        json.dumps(
            {
                "package-small-1": "small",
                "package-large-1": "large",
                "15": "small",
                "17": "large",
            }
        )
    )
    (root / "mosquitto.conf").write_text(
        f"listener {mqtt_port} 127.0.0.1\nallow_anonymous true\npersistence false\n"
    )
    env = {
        k: v
        for k, v in os.environ.items()
        if k == "ROPORT_BOARDING_ALERT_TEST"
        or (not k.startswith("ROPORT_") and k != "PYTHONPATH")
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
    preview_ports = {
        robot: free_port() for robot, (role, _) in robots.items() if role == "smartcart"
    }
    env.update(
        ROPORT_FOLLOW_FRAME_SOURCES=json.dumps(
            {r: f"http://127.0.0.1:{port}" for r, port in preview_ports.items()}
        ),
        ROPORT_FOLLOW_FRAME_TOKEN=secrets.token_urlsafe(32),
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
                    *(
                        ["--preview-port", str(preview_ports[robot])]
                        if robot in preview_ports
                        else []
                    ),
                    *(
                        ["--fault", "d660-line-loss"]
                        if args.d660_line_loss and robot == "pinky1"
                        else []
                    ),
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
                *([] if service_env else ["--journal", str(root / "control.json")]),
                "--socket",
                str(root / "f"),
            ],
            service_env,
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
            service_env,
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
        package = f"package-{args.verify}-1" if args.verify else None
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
                    if args.d660_line_loss:
                        print(
                            f"가상 기체 재배치 · line_lost 정지 후 별도 터미널에서 "
                            f"kill -USR1 {info['pids']['pinky1']}",
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
                            "package_id": package,
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
                            "package_id": package,
                            "goal": "PUBLIC_SPACE_TRIAL",
                        },
                    )
                if tasks.get("verify-out", {}).get("stage") == "await_receipt":
                    send(url, token, state, "receipt", request_id="verify-out")
                if tasks.get("verify-out", {}).get("status") == "COMPLETE":
                    result = {"result": "passed", "case": args.verify, "tasks": tasks}
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
