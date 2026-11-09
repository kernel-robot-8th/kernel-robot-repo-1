"""Persistent, isolated virtual Pinky + real MQTT/owner/API/original web UI."""

import argparse
import hashlib
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
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from prepare_pinky_trial import prepare

from roport_control_server.executor.pinky_trial import validate_config

SERVICE = Path(__file__).resolve().parents[1]
PINKY = SERVICE.parent / "roport-pinky"
MAP = PINKY / "maps/airport_map.yaml"
LAYOUT = PINKY / "src/roport_gz_world/layout/roport_airport.yaml"


def simulation_profile(scenario, record=None):
    profile = prepare(MAP, LAYOUT, simulation=True, single_robot=scenario == "single")
    baseline = None
    if record:
        raw = Path(record).read_bytes()
        data = json.loads(raw)
        original = data["config"]
        if original["mode"] != "hardware":
            raise ValueError("baseline must be a hardware journal")
        for key in ("scenario", "graph_version", "map_id", "map_version", "nodes"):
            if original[key] != profile[key]:
                raise ValueError(
                    f"record/current {key} mismatch; do not replay an obsolete map"
                )
        profile = deepcopy(original)
        rows = []
        for step, done in data["completed"].items():
            accepted = next(
                (
                    e
                    for e in data["events"].values()
                    if e["execution_id"] == done["execution_id"]
                    and e["type"].endswith(".accepted")
                ),
                None,
            )
            rows.append(
                {
                    "step": step,
                    "result": done["type"],
                    "node": done["payload"].get("node_id"),
                    "seconds": round(
                        (
                            datetime.fromisoformat(done["sent_at"])
                            - datetime.fromisoformat(accepted["sent_at"])
                        ).total_seconds(),
                        3,
                    )
                    if accepted
                    else None,
                }
            )
        baseline = {
            "source": str(Path(record).resolve()),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "phase": data["phase"],
            "results": rows,
        }
    else:
        # Reproduce the field run's 0.03 cap, not the later unverified 0.06 setting.
        for route in profile["routes"].values():
            route["speed_limit_mps"] = 0.03
    identity = uuid4().hex[:12]
    profile.update(
        mode="simulation",
        field_verified=False,
        site_id=f"virtual-{identity}",
        control_id=f"control-{identity}",
    )
    for robot in profile["robots"].values():
        robot["binding_id"] = str(uuid4())
    validate_config(profile)
    return profile, baseline


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def fetch(url):
    with urllib.request.urlopen(
        url + "/api/snapshot?mode=trial", timeout=2
    ) as response:
        return json.load(response)["trial"]


def advance(url, token, trial):
    """Explicit --verify only: approve simulated transfer/request gates via the same HTTP API."""
    action = (
        "start"
        if trial["can_start"]
        else "dummy"
        if trial.get("can_dummy")
        else "request"
        if trial.get("can_request")
        else None
    )
    if not action:
        return
    body = {k: trial[k] for k in ("session_id", "task_id", "phase")}
    body["action"] = action
    if action == "dummy":
        body["stop_session_id"] = trial["gate"]["stop_session_id"]
    if action == "request":
        body["items"] = [{"package_id": trial["package_id"], "quantity": 1}]
    request = urllib.request.Request(
        url + "/api/trial/commands",
        json.dumps(body).encode(),
        headers={
            "Content-Type": "application/json",
            "Origin": url,
            "x-roport-trial-token": token,
        },
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        if not json.load(response)["accepted"]:
            raise RuntimeError("virtual command not accepted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=("single", "two", "smartcart"),
        default="two",
        help="default two provides all three editable presets in the UI",
    )
    parser.add_argument(
        "--record", type=Path, help="hardware control journal; never modified"
    )
    parser.add_argument("--scope", choices=("line-only", "nav2"), default="nav2")
    parser.add_argument("--nav2", choices=("simulated", "off"), default="simulated")
    parser.add_argument(
        "--fault", choices=("none", "line-loss", "odom-stale"), default="none"
    )
    parser.add_argument(
        "--output", type=Path, help="new run directory (must not exist)"
    )
    parser.add_argument(
        "--mosquitto",
        default=os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto"),
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="automatically approve SIMULATED gates, then exit",
    )
    args = parser.parse_args()
    if not args.mosquitto:
        parser.error("install Mosquitto or set MOSQUITTO_BIN / --mosquitto")
    profile, baseline = simulation_profile(args.scenario, args.record)
    os.umask(0o077)
    root = (
        args.output
        or Path.home()
        / "roport-trial/virtual"
        / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + uuid4().hex[:6])
    ).resolve()
    if len(os.fsencode(root / "s")) >= 104:
        parser.error("run path too long for Unix socket; choose a shorter --output")
    mqtt_port, port = free_port(), free_port()
    while port == mqtt_port:
        port = free_port()
    url = f"http://127.0.0.1:{port}"
    token = secrets.token_urlsafe(24)

    def write_inputs():
        root.mkdir(mode=0o700, parents=True, exist_ok=False)
        (root / "trial.json").write_text(json.dumps(profile, indent=2) + "\n")
        (root / "ui-key.txt").write_text(token + "\n")
        (root / "mosquitto.conf").write_text(
            f"listener {mqtt_port} 127.0.0.1\nallow_anonymous true\npersistence false\n"
        )

    write_inputs()
    env = {
        k: v
        for k, v in os.environ.items()
        if not k.startswith("ROPORT_") and k != "PYTHONPATH"
    }
    env.update(
        ROPORT_MQTT_HOST="127.0.0.1",
        ROPORT_MQTT_PORT=str(mqtt_port),
        ROPORT_MQTT_TLS="0",
        ROPORT_MAP=str(MAP),
        ROPORT_TRIAL_LAYOUT=str(LAYOUT),
        ROPORT_TRIAL_SOCKET=str(root / "s"),
        ROPORT_TRIAL_TOKEN=token,
        ROPORT_VIRTUAL_RESET_FILE=str(root / "reset.request"),
        ROPORT_STORAGE="disabled",
        XDG_STATE_HOME=str(root / "state"),
    )
    processes, logs = [], []

    def start(name, command):
        log = (root / f"{name}.log").open("w")
        logs.append(log)
        process = subprocess.Popen(
            command,
            cwd=SERVICE,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        processes.append((name, process))

    def stop_children():
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
        processes.clear()
        logs.clear()

    info = {
        "url": url + "/ui/",
        "run_dir": str(root),
        "key_file": str(root / "ui-key.txt"),
        "simulation": True,
        "time_scale": 1,
        "nav2": args.nav2,
        "fault": args.fault,
        "baseline": baseline,
        "source_sha256": {},
    }
    print(
        f"SIMULATION ONLY · 1x · Nav2={args.nav2} · 기록: {root}",
        file=sys.stderr,
        flush=True,
    )
    outcome = {"result": "interrupted"}

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)

    def start_children():
        start("broker", [args.mosquitto, "-c", str(root / "mosquitto.conf")])
        deadline = time.monotonic() + 10
        while True:
            if processes[0][1].poll() is not None:
                raise RuntimeError(f"broker exited; see {root / 'broker.log'}")
            try:
                with socket.create_connection(("127.0.0.1", mqtt_port), timeout=0.1):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    raise RuntimeError("broker startup timeout") from None
                time.sleep(0.05)
        for role in profile["robots"]:
            if args.scenario == "smartcart" and role != "smartcart":
                continue
            start(
                role,
                [
                    sys.executable,
                    "scripts/virtual_pinky.py",
                    "--config",
                    str(root / "trial.json"),
                    "--role",
                    role,
                    "--journal",
                    str(root / f"{role}.json"),
                    "--nav2",
                    args.nav2,
                    "--fault",
                    args.fault,
                ],
            )
        start(
            "owner",
            [
                sys.executable,
                "-m",
                "roport_control_server.executor.pinky_trial",
                "--config",
                str(root / "trial.json"),
                "--journal",
                str(root / "control.json"),
                "--socket",
                str(root / "s"),
                "--scope",
                args.scope,
                *(["--smartcart-only"] if args.scenario == "smartcart" else []),
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
                str(port),
            ],
        )
        info["pids"] = {name: proc.pid for name, proc in processes}
        for path in [
            MAP,
            MAP.with_suffix(".pgm"),
            LAYOUT,
            *PINKY.glob("src/roport_line_nav/roport_line_nav/*.py"),
            *PINKY.glob("src/roport_pinky_line/roport_pinky_line/*.py"),
            PINKY / "src/roport_pinky/roport_pinky/trial_driver.py",
            PINKY / "src/roport_line_nav/test/sim.py",
            SERVICE / "scripts/virtual_pinky.py",
            Path(__file__),
            *SERVICE.glob("src/roport_control_server/executor/*.py"),
            *SERVICE.glob("src/roport_control_server/adapters/mqtt/*.py"),
        ]:
            info["source_sha256"][str(path.relative_to(SERVICE.parent.parent))] = (
                hashlib.sha256(path.read_bytes()).hexdigest()
            )
        (root / "run.json").write_text(json.dumps(info, indent=2) + "\n")

    try:
        start_children()
        started = time.monotonic()
        ready = False
        while True:
            for name, proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError(f"{name} exited; see {root / (name + '.log')}")
            try:
                trial = fetch(url)
            except (OSError, TypeError):
                if time.monotonic() - started > 20:
                    raise RuntimeError("API startup/liveness timeout") from None
                time.sleep(0.1)
                continue
            reset_file = root / "reset.request"
            if reset_file.exists():
                try:
                    request = json.loads(reset_file.read_text())
                except json.JSONDecodeError:
                    time.sleep(0.05)  # The API may still be finishing its small write.
                    continue
                if any(request.get(k) != trial[k] for k in ("session_id", "task_id")):
                    reset_file.rename(root / f"reset-rejected-{uuid4().hex[:6]}.json")
                    continue
                stop_children()
                (root / "outcome.json").write_text(
                    json.dumps(
                        {
                            "result": "reset",
                            "phase": trial["phase"],
                            "session_id": trial["session_id"],
                        },
                        indent=2,
                    )
                    + "\n"
                )
                archive = root.with_name(f"{root.name}-before-reset-{uuid4().hex[:6]}")
                root.rename(archive)
                (archive / "run.json").write_text(
                    json.dumps(
                        {
                            **info,
                            "run_dir": str(archive),
                            "key_file": str(archive / "ui-key.txt"),
                        },
                        indent=2,
                    )
                    + "\n"
                )
                profile, baseline = simulation_profile(args.scenario, args.record)
                write_inputs()
                info.update(
                    baseline=baseline,
                    previous_run_dir=str(archive),
                    reset_count=info.get("reset_count", 0) + 1,
                )
                start_children()
                started, ready = time.monotonic(), False
                continue
            if not ready:
                if not trial["can_start"]:
                    if time.monotonic() - started > 20:
                        raise RuntimeError("robot readiness timeout; see control.json")
                    time.sleep(0.1)
                    continue
                ready = True
                print(
                    json.dumps({k: info[k] for k in ("url", "run_dir", "key_file")}),
                    flush=True,
                )
            if args.verify:
                if trial["phase"] in {"COMPLETE", "HOLD"}:
                    outcome = {
                        "result": trial["phase"],
                        "seconds": round(time.monotonic() - started, 2),
                    }
                    if trial["phase"] != "COMPLETE":
                        raise RuntimeError(
                            "virtual trial entered HOLD; inspect control.json"
                        )
                    print("PASS: virtual scenario COMPLETE", flush=True)
                    break
                if time.monotonic() - started > 1800:
                    raise RuntimeError("virtual verification exceeded 30 minutes")
                advance(url, token, trial)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        outcome = {"result": "failed", "error": str(exc)}
        raise
    finally:
        stop_children()
        if root.exists():
            (root / "outcome.json").write_text(json.dumps(outcome, indent=2) + "\n")


if __name__ == "__main__":
    main()
