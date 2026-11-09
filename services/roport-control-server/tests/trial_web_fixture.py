"""Browser-test fixture: isolated loopback MQTT, fake Pinky and separate owner/API."""

import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

from mqtt_helpers import local_broker, wait_until

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "tools"))
from prepare_pinky_trial import prepare


def main():
    processes, logs = [], []
    with tempfile.TemporaryDirectory(prefix="rp-web-") as folder:
        root = Path(folder)
        pinky = SERVICE.parent / "roport-pinky"
        map_path = pinky / "maps/airport_map.yaml"
        layout = pinky / "src/roport_gz_world/layout/roport_airport.yaml"
        config = root / "trial.json"
        profile = prepare(
            map_path,
            layout,
            simulation=True,
            single_robot=len(sys.argv) < 3 or sys.argv[2] == "single",
        )
        smartcart_only = len(sys.argv) > 2 and sys.argv[2] == "smartcart"
        with_omx = len(sys.argv) > 4 and sys.argv[4] == "omx"
        if with_omx:
            profile["omx"] = {
                "binding_id": "browser-omx",
                "slot_id": "1a",
                "model_version": "sim-model",
                "calibration_version": "sim-cal",
                "execution_mode": "mock",
            }
        config.write_text(json.dumps(profile))
        with local_broker(root) as (mqtt_port, _, _):
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                port = reservation.getsockname()[1]
            token = "browser-test-only-operation-key"
            env = {
                **os.environ,
                "ROPORT_MQTT_HOST": "127.0.0.1",
                "ROPORT_MQTT_PORT": str(mqtt_port),
                "ROPORT_MQTT_TLS": "0",
                "ROPORT_MAP": str(map_path),
                "ROPORT_TRIAL_LAYOUT": str(layout),
                "ROPORT_TRIAL_SOCKET": str(root / "s"),
                "ROPORT_TRIAL_TOKEN": token,
                "XDG_STATE_HOME": str(root / "state"),
                "ROPORT_STORAGE": "disabled",
            }

            def start(args):
                log = (root / f"process-{len(processes)}.log").open("w")
                logs.append(log)
                process = subprocess.Popen(
                    [sys.executable, *args],
                    cwd=SERVICE,
                    env=env,
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=log,
                )
                processes.append(process)

            try:
                if with_omx:
                    start(
                        [
                            "tests/omx_web_fixture.py",
                            str(config),
                            str(root / "omx.json"),
                        ]
                    )
                for role in profile["robots"]:
                    if smartcart_only and role != "smartcart":
                        continue
                    if (
                        len(sys.argv) > 3
                        and sys.argv[3] != "all"
                        and role != sys.argv[3]
                    ):
                        continue
                    start(
                        [
                            "scripts/mock_pinky_trial.py",
                            "--config",
                            str(config),
                            "--role",
                            role,
                            "--journal",
                            str(root / f"{role}.json"),
                        ]
                    )
                start(
                    [
                        "-m",
                        "roport_control_server.executor.pinky_trial",
                        "--config",
                        str(config),
                        "--journal",
                        str(root / "control.json"),
                        "--socket",
                        str(root / "s"),
                        "--scope",
                        sys.argv[1] if len(sys.argv) > 1 else "nav2",
                        *(["--smartcart-only"] if smartcart_only else []),
                    ]
                )
                start(
                    [
                        "-m",
                        "uvicorn",
                        "roport_control_server.adapters.api.app:app",
                        "--host",
                        "127.0.0.1",
                        "--port",
                        str(port),
                        "--ws-max-size",
                        "4096",
                        "--ws-max-queue",
                        "4",
                    ]
                )
                url = f"http://127.0.0.1:{port}"

                def ready():
                    if any(p.poll() is not None for p in processes):
                        raise RuntimeError("trial fixture process exited")
                    try:
                        with urllib.request.urlopen(
                            url + "/api/snapshot?mode=trial", timeout=1
                        ) as response:
                            trial = json.load(response)["trial"]
                            return (
                                trial["available"]
                                if len(sys.argv) > 3
                                else trial["can_start"]
                            )
                    except (OSError, TypeError):
                        return False

                wait_until(ready, timeout=15)
                print(json.dumps({"url": url, "token": token}), flush=True)
                while True:
                    time.sleep(0.2)
            except KeyboardInterrupt:
                pass
            finally:
                for process in reversed(processes):
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
