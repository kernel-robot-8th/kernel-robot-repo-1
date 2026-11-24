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


def wait_for_shutdown(proc, timeout=30):
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        # Each launcher child owns a separate session. Freeze the parent while
        # enumerating its actual children, including before run.json exists.
        proc.send_signal(signal.SIGSTOP)
        try:
            children = Path(f"/proc/{proc.pid}/task/{proc.pid}/children")
            for child in children.read_text().split() if children.exists() else ():
                pid = int(child)
                try:
                    if os.getpgid(pid) == pid:
                        os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        finally:
            proc.kill()
        proc.wait()
        raise


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
                    wait_for_shutdown(proc)
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


@pytest.mark.parametrize("case", [None, "small", "large"])
def test_explicit_verification_submits_selected_package(broker, tmp_path, case):
    # The shared broker fixture enforces MQTT availability; the launcher owns its broker.
    binary = os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")
    root = tmp_path / "run"
    port = free_port()
    with (tmp_path / "launcher.log").open("w") as log:
        proc = subprocess.Popen(
            [
                sys.executable,
                str(SERVICE / "tools/run_virtual_fleet.py"),
                "--mosquitto",
                binary,
                "--output",
                str(root),
                "--port",
                str(port),
                "--verify",
                *([case] if case else []),
            ],
            stdout=log,
            stderr=log,
        )
        try:

            def submitted():
                assert proc.poll() is None, "verification exited before submission"
                try:
                    state = fetch(f"http://127.0.0.1:{port}")
                except (OSError, KeyError):
                    return False
                task = state.get("tasks", {}).get("verify-in")
                if not task:
                    return False
                assert task["request"]["package_id"] == f"package-{case or 'small'}-1"
                return True

            wait_until(submitted, timeout=30)
        finally:
            proc.send_signal(signal.SIGINT)
            wait_for_shutdown(proc)
    assert proc.returncode == 0
    for pid in json.loads((root / "run.json").read_text())["pids"].values():
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)


def test_timeout_stops_isolated_child_session(tmp_path):
    child_file = tmp_path / "child"
    owner = subprocess.Popen(
        [
            sys.executable,
            "-c",
            (
                "import pathlib,subprocess,sys,time; "
                "child=subprocess.Popen(['sleep','60'], start_new_session=True); "
                "pathlib.Path(sys.argv[1]).write_text(str(child.pid)); time.sleep(60)"
            ),
            str(child_file),
        ]
    )
    try:
        wait_until(lambda: child_file.exists() and child_file.read_text(), timeout=3)
        child = int(child_file.read_text())
        with pytest.raises(subprocess.TimeoutExpired):
            wait_for_shutdown(owner, timeout=0.01)
        stat = Path(f"/proc/{child}/stat")

        def child_has_exited():
            try:
                return stat.read_text().split()[2] == "Z"
            except (FileNotFoundError, ProcessLookupError):
                return True

        wait_until(child_has_exited, timeout=3)
    finally:
        owner.kill()
        owner.wait()


def test_d660_line_loss_keeps_evidence(tmp_path):
    from roport_control_server.adapters.mqtt.mqtt import MqttConfig
    from roport_control_server.executor.fleet import TARGETS

    """Replay the field IR failure; optional Chromium reload shares this actual owner."""
    import time
    from datetime import UTC, datetime
    from queue import Empty

    from roport_control_server.adapters.mqtt import field_wire as wire
    from roport_control_server.adapters.mqtt.mqtt import MqttTransport

    binary = os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")
    if not binary:
        if os.environ.get("REQUIRE_MQTT_TESTS") == "1":
            pytest.fail("Mosquitto required")
        pytest.skip("Mosquitto required")
    port = free_port()
    url = f"http://127.0.0.1:{port}"
    # Keep the Unix socket path below the platform limit even with long pytest paths.
    with tempfile.TemporaryDirectory(prefix="rp-line-") as folder:
        root = Path(folder) / "run"
        with (tmp_path / "launcher.log").open("w") as log:
            proc = subprocess.Popen(
                [
                    sys.executable,
                    "-u",
                    "tools/run_virtual_fleet.py",
                    "--d660-line-loss",
                    "--output",
                    str(root),
                    "--port",
                    str(port),
                    "--mosquitto",
                    binary,
                ],
                cwd=SERVICE,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:

                def ready():
                    assert proc.poll() is None, "simulation launcher exited"
                    return "조작 키:" in (tmp_path / "launcher.log").read_text()

                wait_until(ready, timeout=30)
                broker_port = int((root / "mosquitto.conf").read_text().split()[1])
                spy = MqttTransport(
                    MqttConfig("127.0.0.1", "line-loss-spy", broker_port),
                    [wire.topic(r, "cmd") for r in TARGETS],
                )
                spy.start()
                try:
                    key = (root / "ui-key.txt").read_text().strip()
                    state = fetch(url)
                    send(
                        url,
                        key,
                        state,
                        "submit",
                        request={
                            "request_id": "field-replay",
                            "operation": "inbound",
                            "package_id": "package-small-1",
                            "goal": None,
                        },
                    )
                    send(url, key, state, "enable")
                    messages = []

                    def drain():
                        assert spy.ready, "command observer disconnected"
                        while True:
                            try:
                                delivery = spy.receive(0)
                            except Empty:
                                return
                            messages.append(wire.decode(delivery.payload))
                            spy.ack(delivery)

                    def held():
                        time.sleep(0.2)
                        drain()
                        return fetch(url)["fault"]

                    wait_until(held, timeout=120)
                    cutoff = datetime.now(UTC)

                    def evidence():
                        with urlopen(
                            url + "/api/snapshot?mode=fleet", timeout=3
                        ) as response:
                            data = json.load(response)
                        row = next(
                            r for r in data["robots"] if r["robot_id"] == "pinky1"
                        )
                        f = data["fleet"]
                        assert f["tasks"]["field-replay"]["reason"] == "line_lost", f[
                            "fault"
                        ]
                        assert f["resources"]["edge:N10-N18"] == "field-replay"
                        assert (
                            not f["enabled"]
                            and f["tasks"]["field-replay"]["status"] == "HOLD"
                        )
                        if (
                            f["poses"]["pinky1"]["pose"] is not None
                            or row["status"]["node"] is not None
                        ):
                            return None
                        assert row["pose_stale"] and not row["pose_live"]
                        assert (
                            row["pose"] and "마지막 확인 노드 N10" in row["pose_source"]
                        )
                        return row["pose"], f["resources"]

                    wait_until(evidence)
                    before = evidence()
                    # Queue competing work while the failed robot owns the leg.
                    send(
                        url,
                        key,
                        state,
                        "submit",
                        request={
                            "request_id": "second",
                            "operation": "inbound",
                            "package_id": "second-box",
                            "goal": None,
                        },
                    )

                    # Observe two new, non-renewing status polls after HOLD.
                    def two_polls():
                        drain()
                        return (
                            sum(
                                m["type"] == "robot_state_request"
                                and m["target"] == "pinky1"
                                and wire.timestamp(m["sent_at"]) > cutoff
                                for m in messages
                            )
                            >= 2
                        )

                    wait_until(two_polls, timeout=10)
                    assert evidence() == before
                    stopped = fetch(url)
                    (cid,) = stopped["commands"]
                    note = "N16 정지·N14 방향·통로 확인"
                    if os.environ.get("ROPORT_TEST_BROWSER") == "1":
                        browser = subprocess.Popen(
                            [
                                "node",
                                "tests/line_loss_reload.mjs",
                                url,
                                str(tmp_path / "line-loss-reload.png"),
                                str(root),
                            ],
                            cwd=SERVICE,
                        )
                        try:

                            def browser_finished():
                                drain()
                                return browser.poll() is not None

                            wait_until(browser_finished, timeout=100)
                            assert browser.returncode == 0
                        finally:
                            if browser.poll() is None:
                                browser.kill()
                            browser.wait()

                    else:
                        from urllib.error import HTTPError

                        def confirm(boot):
                            return send(
                                url,
                                key,
                                state,
                                "reconcile",
                                command_id=cid,
                                outcome="repositioned_n16",
                                note=note,
                                confirmed=True,
                                boot_version=boot,
                            )

                        with pytest.raises(HTTPError) as denied:
                            confirm(stopped["robots"]["pinky1"]["boot_version"])
                        assert denied.value.code == 409
                        os.kill(
                            json.loads((root / "run.json").read_text())["pids"][
                                "pinky1"
                            ],
                            signal.SIGUSR1,
                        )
                        wait_until(
                            lambda: (
                                fetch(url)["robots"]["pinky1"]["state"]["node"] == "N16"
                            )
                        )
                        parked = fetch(url)
                        assert parked["resources"] == before[1]
                        confirm(parked["robots"]["pinky1"]["boot_version"])
                        send(url, key, state, "release_hold", note=note)
                    standby = fetch(url)
                    assert standby["fault"] is None and not standby["enabled"]
                    assert (
                        not standby["commands"]
                        and "edge:N10-N18" not in standby["resources"]
                    )
                    assert (
                        standby["tasks"]["field-replay"]["stage"] == "warehouse_route"
                    )
                    assert standby["tasks"]["second"]["status"] == "QUEUED"
                    assert standby["slots"] == stopped["slots"]
                    assert (
                        standby["recoveries"][cid]["confirmed_boot"]
                        == standby["robots"]["pinky1"]["boot_version"]
                    )
                    hold_cutoff = cutoff
                    cutoff = datetime.now(UTC)
                    wait_until(two_polls, timeout=10)

                    work = [
                        (m["target"], m["type"])
                        for m in messages
                        if m["type"]
                        not in {"robot_state_request", "robot_stop_command"}
                    ]
                    assert work == [("pinky1", "line_route_command")], work
                    assert not any(
                        m["payload"].get("keep_running_command_id")
                        for m in messages
                        if wire.timestamp(m["sent_at"]) > hold_cutoff
                    )
                    final = fetch(url)
                    assert final["tasks"]["second"]["status"] == "QUEUED"
                    (tmp_path / "line-loss-result.json").write_text(
                        json.dumps(
                            {
                                "route": next(
                                    m["payload"]["route"]
                                    for m in messages
                                    if m["type"] == "line_route_command"
                                ),
                                "reason": stopped["tasks"]["field-replay"]["reason"],
                                "reserved_by": stopped["resources"]["edge:N10-N18"],
                                "last_confirmed_node": stopped["last_nodes"]["pinky1"],
                                "failed_node": stopped["robots"]["pinky1"]["state"][
                                    "node"
                                ],
                                "standby_node": final["robots"]["pinky1"]["state"][
                                    "node"
                                ],
                                "standby_enabled": final["enabled"],
                                "standby_commands": final["commands"],
                                "recovery": final["recoveries"][cid],
                                "second_request": final["tasks"]["second"]["status"],
                                "work_commands": work,
                                "browser_reload": os.environ.get("ROPORT_TEST_BROWSER")
                                == "1",
                            },
                            indent=2,
                        )
                        + "\n"
                    )
                finally:
                    spy.stop()
            finally:
                proc.terminate()
                proc.wait(timeout=30)
