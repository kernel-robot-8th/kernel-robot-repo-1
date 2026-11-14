"""One-command startup with current sources, authenticated UI and owned cleanup."""

import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
from pathlib import Path
from urllib.request import urlopen

import pytest
from mqtt_helpers import wait_until

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "tools"))
from run_virtual_fleet import fetch, send
from run_virtual_trial import free_port


@pytest.mark.parametrize("termination", [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_script_starts_current_checkout_and_cleans_up(termination):
    binary = os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")
    if not binary:
        if os.environ.get("REQUIRE_MQTT_TESTS") == "1":
            pytest.fail("Mosquitto required for launcher test")
        pytest.skip("Mosquitto required for launcher test")
    with tempfile.TemporaryDirectory(prefix="rp-fleet-ui-") as directory:
        base = Path(directory)
        port = free_port()
        previous_key = None
        for attempt in range(2 if termination == signal.SIGINT else 1):
            root = base / str(attempt)
            output = base / f"stdout-{attempt}"
            with output.open("w") as log:
                proc = subprocess.Popen(
                    [
                        "bash",
                        str(SERVICE / "tools/start_virtual_ui.sh"),
                        "--output",
                        str(root),
                        "--port",
                        str(port),
                    ],
                    cwd=base,
                    env={
                        **os.environ,
                        "MOSQUITTO_BIN": binary,
                        "UV_OFFLINE": "1",
                        "UV_PROJECT_ENVIRONMENT": str(base / "wrong-venv"),
                        "PYTHONPATH": str(base / "old-source"),
                        "ROPORT_MQTT_HOST": "192.0.2.1",
                        "ROPORT_MQTT_PASSWORD": "must-not-inherit",
                    },
                    stdout=log,
                    stderr=log,
                    start_new_session=True,
                )
                try:

                    def ready(proc=proc, output=output):
                        assert proc.poll() is None, "launcher exited before ready"
                        return "조작 키: " in output.read_text()

                    wait_until(ready, timeout=30)
                    info = json.loads((root / "run.json").read_text())
                    key = (root / "ui-key.txt").read_text().strip()
                    assert key != previous_key
                    previous_key = key
                    lines = output.read_text().splitlines()
                    assert lines[-1].startswith("조작 키: ")
                    assert lines[-1].removeprefix("조작 키: ") == key
                    assert (root / "ui-key.txt").stat().st_mode & 0o077 == 0
                    url = f"http://127.0.0.1:{port}"
                    state = fetch(url)
                    assert state["available"] and not state["fault"]
                    assert len(state["robots"]) == 7
                    assert state["tasks"] == {}
                    assert send(url, key, state, "enable")["accepted"]
                    for name, pid in info["pids"].items():
                        if name == "broker":
                            continue
                        env = dict(
                            entry.split(b"=", 1)
                            for entry in Path(f"/proc/{pid}/environ")
                            .read_bytes()
                            .split(b"\0")
                            if b"=" in entry
                        )
                        source = (
                            SERVICE.parent / "roport-omx/src"
                            if name == "omx"
                            else SERVICE / "src"
                        )
                        assert env[b"PYTHONPATH"] == os.fsencode(source)
                        assert b"ROPORT_MQTT_PASSWORD" not in env
                    with urlopen(url + "/ui/fleet.js", timeout=3) as response:
                        assert (
                            response.read()
                            == (
                                SERVICE
                                / "src/roport_control_server/spatial_assets/fleet.js"
                            ).read_bytes()
                        )
                finally:
                    if proc.poll() is None:
                        proc.send_signal(termination)
                    proc.wait(timeout=30)
            assert proc.returncode == 0
            assert (root / "control.json").exists()
            assert (
                json.loads((root / "outcome.json").read_text())["result"]
                == "interrupted"
            )
            for pid in info["pids"].values():
                with pytest.raises(ProcessLookupError):
                    os.kill(pid, 0)
        assert (base / "0/control.json").exists()


def test_busy_port_keeps_existing_listener_and_creates_no_run(tmp_path):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        root = tmp_path / "must-not-create"
        result = subprocess.run(
            [
                sys.executable,
                str(SERVICE / "tools/run_virtual_fleet.py"),
                "--mosquitto",
                "not-started",
                "--output",
                str(root),
                "--port",
                str(listener.getsockname()[1]),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 2 and "is busy" in result.stderr
        assert "조작 키:" not in result.stdout
        assert not root.exists()
        with socket.create_connection(listener.getsockname(), timeout=1):
            pass
