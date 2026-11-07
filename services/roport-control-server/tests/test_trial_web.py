import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from copy import deepcopy
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from mock_pinky_trial import FakePinky
from mqtt_helpers import wait_until
from test_pinky_trial import MemoryTransport, prepare_module

from roport_control_server.adapters.api.app import create_app
from roport_control_server.adapters.api.maps import load_map
from roport_control_server.adapters.api.trial import TrialView
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.executor.pinky_trial import TrialControl
from roport_control_server.executor.trial_socket import TrialSocket

SERVICE = Path(__file__).resolve().parents[1]
MAP = SERVICE.parent / "roport-pinky/maps/airport_map.yaml"
LAYOUT = SERVICE.parent / "roport-pinky/src/roport_gz_world/layout/roport_airport.yaml"
TOKEN = "test-only-trial-operation-key"
HEADERS = {"origin": "http://testserver", "x-roport-trial-token": TOKEN}


@pytest.fixture
def single_profile():
    return prepare_module.prepare(MAP, LAYOUT, simulation=True, single_robot=True)


def view(path):
    return TrialView(path, TOKEN, load_map(MAP, LAYOUT))


def fence(snapshot, action="start"):
    trial = snapshot["trial"]
    command = {k: trial[k] for k in ("session_id", "task_id", "phase")}
    command["action"] = action
    if action == "request":
        command["items"] = [{"package_id": trial["package_id"], "quantity": 1}]
    if action == "dummy":
        command["stop_session_id"] = trial["gate"]["stop_session_id"]
    return command


@pytest.fixture
def local_owner(tmp_path, single_profile):
    journal = Journal(tmp_path / "control.json")
    transport = MemoryTransport()
    control = TrialControl(single_profile, transport, journal)
    status = {**FakePinky(single_profile, "logistics").state, "boot_id": "boot-one"}
    state = deepcopy(control.state)
    state["statuses"]["pinky1"] = status
    journal.save(state)
    control.heard["pinky1"] = time.monotonic()
    channel = TrialSocket(tmp_path / "s", control)
    done = threading.Event()

    def loop():
        while not done.wait(0.005):
            channel.poll()

    worker = threading.Thread(target=loop)
    worker.start()
    app = create_app(
        MAP, trial_socket=channel.path, trial_layout=LAYOUT, trial_token=TOKEN
    )
    try:
        with TestClient(app) as client:
            yield client, control, channel, transport
    finally:
        done.set()
        worker.join(2)
        channel.close()
        journal.close()


def test_trial_start_auth_reconnect_duplicate_and_stop(local_owner):
    client, control, _, transport = local_owner
    bootstrap = client.get("/api/bootstrap?mode=trial").json()
    assert bootstrap["trial"] and len(bootstrap["robots"]) == 6
    assert bootstrap["map"]["id"] == "airport_map"
    assert bootstrap["map"]["layout"]["mounts"]
    assert client.get("/api/bootstrap").json()["mode"] == "trial"
    assert client.get("/api/bootstrap?mode=offline").json()["mode"] == "offline"
    assert bootstrap["trial_nodes"]["N16"]
    assert transport.sent == []  # Starting the web app doesn't start a robot.
    with client.websocket_connect("/api/state?mode=trial", headers=HEADERS) as ws:
        first = ws.receive_json()
        assert first["trial"]["can_start"]
        assert first["robots"][0]["pose_source"].startswith("마지막 확인 노드")
    # No browser connection remains; the owner and task are still alive.
    command = fence(first)
    for headers in (
        {},
        {**HEADERS, "origin": "http://elsewhere"},
        {**HEADERS, "x-roport-trial-token": "wrong"},
    ):
        assert (
            client.post(
                "/api/trial/commands", headers=headers, json=command
            ).status_code
            == 403
        )
    assert not transport.sent
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=command).status_code
        == 200
    )
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=command).status_code
        == 409
    )
    assert len(transport.sent) == 1
    with client.websocket_connect("/api/state?mode=trial", headers=HEADERS) as ws:
        second = ws.receive_json()
        assert second["trial"]["task_id"] == first["trial"]["task_id"]
        assert second["trial"]["phase"] == "to_delivery"
    assert (
        client.post(
            "/api/trial/commands", headers=HEADERS, json=fence(first, "stop")
        ).status_code
        == 200
    )
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "operator_stopped"


def test_trial_state_staleness_and_no_invented_nav2_pose(local_owner):
    client, control, _, _ = local_owner
    control.heard["pinky1"] = time.monotonic() - 5
    snapshot = client.get("/api/snapshot?mode=trial").json()
    assert snapshot["robots"][0]["stale"]
    assert not snapshot["trial"]["can_start"]
    assert (
        client.post(
            "/api/trial/commands", headers=HEADERS, json=fence(snapshot)
        ).status_code
        == 409
    )
    state = deepcopy(control.state)
    state["statuses"]["pinky1"]["mode"] = "nav2"
    state["statuses"]["pinky1"]["node_id"] = "N12"
    control.journal.save(state)
    assert client.get("/api/snapshot?mode=trial").json()["robots"][0]["pose"] is None


def test_map_mismatch_blocks_start_but_allows_hold(local_owner):
    client, control, _, _ = local_owner
    control.config["map_version"] = "other-map"
    snapshot = client.get("/api/snapshot?mode=trial").json()
    assert snapshot["robots"][0]["pose"] is None
    assert not snapshot["trial"]["can_start"]
    assert client.get("/api/bootstrap?mode=trial").status_code == 503
    assert (
        client.post(
            "/api/trial/commands", headers=HEADERS, json=fence(snapshot)
        ).status_code
        == 409
    )
    assert (
        client.post(
            "/api/trial/commands", headers=HEADERS, json=fence(snapshot, "stop")
        ).status_code
        == 200
    )


def test_old_dummy_and_expired_local_requests_do_not_advance(local_owner):
    client, control, channel, _ = local_owner
    state = deepcopy(control.state)
    state.update(
        phase="WAIT_INBOUND",
        gate={"stop_session_id": "gate-one", "step_id": "to_delivery"},
    )
    state["statuses"]["pinky1"]["node_id"] = "N11"
    control.journal.save(state)
    snapshot = client.get("/api/snapshot?mode=trial").json()
    request = fence(snapshot, "dummy")
    expired = {**request, "expires_at": time.time() - 1}
    with pytest.raises(ValueError, match="expired"):
        channel.dispatch(expired)
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=request).status_code
        == 200
    )
    assert control.state["phase"] == "WAIT_CART_LOAD"
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=request).status_code
        == 409
    )
    assert len(control.state["transfers"]) == 1


def test_closed_socket_disables_actions_and_advances_age(local_owner):
    _, _, channel, _ = local_owner
    projector = view(channel.path)
    before = projector.snapshot()
    projector.socket_path = str(channel.path) + "-absent"
    after = projector.snapshot()
    assert after["robots"][0]["stale"]
    assert after["robots"][0]["age"] >= before["robots"][0]["age"]
    assert not after["trial"]["can_start"]
    assert not after["trial"]["can_stop"]
    assert after["errors"]


def test_socket_never_overwrites_existing_owner(local_owner):
    _, control, channel, _ = local_owner
    with pytest.raises(OSError):
        TrialSocket(channel.path, control)
    assert view(channel.path).snapshot()["trial"]["phase"] == "READY"


def test_socket_oversize_and_partial_clients_do_not_block_owner(local_owner):
    _, _, channel, _ = local_owner
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as idle:
        idle.connect(str(channel.path))
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as large:
            large.settimeout(1)
            large.connect(str(channel.path))
            large.sendall(b"x" * 4097)
            assert json.loads(large.recv(65536))["ok"] is False
        assert view(channel.path).snapshot()["trial"]["phase"] == "READY"


@pytest.mark.parametrize("single", [True, False], ids=["single", "two"])
@pytest.mark.parametrize("scope", ["line-only", "nav2"])
def test_real_mqtt_web_full_trial_and_exclusive_owner(tmp_path, broker, scope, single):
    profile = prepare_module.prepare(MAP, LAYOUT, simulation=True, single_robot=single)
    config = tmp_path / "trial.json"
    config.write_text(json.dumps(profile))
    channel = tmp_path / "s"
    env = {
        **os.environ,
        "ROPORT_MQTT_HOST": "127.0.0.1",
        "ROPORT_MQTT_PORT": str(broker[0]),
        "ROPORT_MQTT_TLS": "0",
        "XDG_STATE_HOME": str(tmp_path / "state"),
    }
    processes = []
    logs = []

    def start(args):
        log = (tmp_path / f"process-{len(processes)}.log").open("w")
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
        return process

    try:
        for role in profile["robots"]:
            start(
                [
                    "scripts/mock_pinky_trial.py",
                    "--config",
                    str(config),
                    "--role",
                    role,
                    "--journal",
                    str(tmp_path / f"{role}.json"),
                ]
            )
        owner_args = [
            "-m",
            "roport_control_server.executor.pinky_trial",
            "--config",
            str(config),
            "--journal",
            str(tmp_path / "control.json"),
            "--socket",
            str(channel),
            "--scope",
            scope,
        ]
        owner = start(owner_args)
        wait_until(lambda: channel.exists())
        projector = view(channel)
        wait_until(lambda: projector.snapshot()["trial"]["can_start"], timeout=10)
        # Even another journal/socket must not create a second MQTT owner.
        duplicate = start(
            [
                *owner_args[:],
                "--journal",
                str(tmp_path / "other.json"),
                "--socket",
                str(tmp_path / "other.sock"),
            ]
        )
        assert duplicate.wait(5) != 0
        app = create_app(
            MAP, trial_socket=channel, trial_layout=LAYOUT, trial_token=TOKEN
        )
        with TestClient(app) as client:
            first = client.get("/api/snapshot?mode=trial").json()
            assert (
                client.post(
                    "/api/trial/commands", headers=HEADERS, json=fence(first)
                ).status_code
                == 200
            )
            phases = (
                ("WAIT_INBOUND", "WAIT_CART_LOAD")
                if single
                else ("WAIT_LOAD", "WAIT_INBOUND", "WAIT_REQUEST", "WAIT_CART_LOAD")
            )
            for phase in phases:
                wait_until(
                    lambda phase=phase: projector.snapshot()["trial"]["phase"] == phase,
                    timeout=10,
                )
                current = client.get("/api/snapshot?mode=trial").json()
                command = fence(
                    current, "request" if phase == "WAIT_REQUEST" else "dummy"
                )
                if phase == "WAIT_REQUEST":
                    wrong = {
                        **command,
                        "items": [{"package_id": "wrong", "quantity": 1}],
                    }
                    assert (
                        client.post(
                            "/api/trial/commands", headers=HEADERS, json=wrong
                        ).status_code
                        == 409
                    )
                    assert projector.snapshot()["trial"]["phase"] == "WAIT_REQUEST"
                assert (
                    client.post(
                        "/api/trial/commands", headers=HEADERS, json=command
                    ).status_code
                    == 200
                )
                assert (
                    client.post(
                        "/api/trial/commands", headers=HEADERS, json=command
                    ).status_code
                    == 409
                )
            wait_until(
                lambda: projector.snapshot()["trial"]["phase"] == "COMPLETE", timeout=15
            )
            final = client.get("/api/snapshot?mode=trial").json()
            assert final["tasks"][0]["progress"] == 100
            assert final["trial"]["scope"] == scope
            cart = next(
                r
                for r in final["robots"]
                if r["robot_id"] == ("pinky1" if single else "pinky3")
            )
            assert final["trial"]["active_robot"] == cart["robot_id"]
            if not single:
                assert final["robots"][0]["status"]["node_id"] == "N16"
                assert final["robots"][0]["pose_source"].startswith("마지막 확인 노드")
            if scope == "nav2":
                assert cart["pose_source"] == "Nav2 완료 응답 위치"
            else:
                assert cart["status"]["node_id"] == "N12"
                assert cart["status"]["stopped"]
                assert "Nav2 제외" in final["trial"]["label"]
        journal = json.loads((tmp_path / "control.json").read_text())
        assert len(journal["transfers"]) == (2 if single else 3)
        expected = set(profile["routes"])
        if scope == "nav2":
            expected.update({"mode_switch", "navigate"})
        assert set(journal["completed"]) == expected
        robot_journal = json.loads(
            (tmp_path / ("logistics.json" if single else "smartcart.json")).read_text()
        )
        if scope == "line-only":
            assert all(
                record["command"]["type"] == "route.requested"
                for record in robot_journal["records"].values()
            )
        owner.send_signal(signal.SIGINT)
        assert owner.wait(5) == 0
        assert not channel.exists()
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


def test_start_blocks_wrong_node_and_unassigned_slots_stay_unknown(local_owner):
    client, control, _, transport = local_owner
    state = deepcopy(control.state)
    state["statuses"]["pinky1"]["node_id"] = "N11"
    control.journal.save(state)
    snapshot = client.get("/api/snapshot?mode=trial").json()
    assert not snapshot["trial"]["can_start"]
    assert snapshot["trial"]["start_error"] == "unexpected route start"
    for row in snapshot["robots"][1:]:
        assert row["pose"] is None and row["stale"] and row["status"] == {}
    assert (
        client.post(
            "/api/trial/commands", headers=HEADERS, json=fence(snapshot)
        ).status_code
        == 409
    )
    assert transport.sent == []


def test_original_ui_bootstraps_without_owner_and_keeps_storage_disabled(tmp_path):
    app = create_app(
        MAP,
        layout_path=LAYOUT,
        trial_socket=tmp_path / "absent",
        trial_layout=tmp_path / "unused-legacy-layout",
        trial_token=TOKEN,
    )
    with TestClient(app) as client:
        response = client.get("/api/bootstrap")
        assert response.status_code == 200
        data = response.json()
        assert data["mode"] == "trial" and len(data["robots"]) == 6
        assert data["map"]["layout"]["mounts"]
        assert TOKEN not in response.text
        snapshot = client.get("/api/snapshot?mode=trial").json()
        assert snapshot["trial"] is None and all(r["stale"] for r in snapshot["robots"])
        assert client.get("/api/robots").status_code == 404
        assert client.get("/api/pinky/settings").status_code == 200


@pytest.mark.parametrize("field", ["session_id", "task_id", "phase"])
def test_stale_command_fence_never_dispatches(local_owner, field):
    client, _, _, transport = local_owner
    request = fence(client.get("/api/snapshot?mode=trial").json())
    request[field] = "previous"
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=request).status_code
        == 409
    )
    assert transport.sent == []


@pytest.mark.parametrize(
    "raw,content_type,code",
    [
        ("{", "application/json", 400),
        ("[]", "application/json", 400),
        ("{}", "text/plain", 415),
        (" " * 4097, "application/json", 413),
    ],
)
def test_trial_http_input_limits(local_owner, raw, content_type, code):
    client, _, _, transport = local_owner
    response = client.post(
        "/api/trial/commands",
        headers={**HEADERS, "content-type": content_type},
        content=raw,
    )
    assert response.status_code == code
    assert transport.sent == []


def test_websocket_is_read_only_and_socket_is_private(local_owner):
    from starlette.websockets import WebSocketDisconnect

    client, _, channel, transport = local_owner
    assert channel.path.stat().st_mode & 0o777 == 0o600
    with client.websocket_connect("/api/state?mode=trial", headers=HEADERS) as ws:
        first = ws.receive_json()
        ws.send_json(fence(first))
        with pytest.raises(WebSocketDisconnect) as closed:
            while True:
                ws.receive_json()
        assert closed.value.code == 1008
    assert transport.sent == []


def test_trial_configuration_is_opt_in(tmp_path):
    with TestClient(create_app()) as client:
        assert client.get("/api/bootstrap").json()["mode"] == "offline"
        assert client.get("/api/bootstrap?mode=trial").status_code == 503
        assert client.get("/api/snapshot?mode=trial").status_code == 503
        assert (
            client.post("/api/trial/commands", headers=HEADERS, json={}).status_code
            == 503
        )
    with (
        pytest.raises(ValueError, match="24 characters"),
        TestClient(create_app(MAP, layout_path=LAYOUT, trial_socket=tmp_path / "s")),
    ):
        pass


def test_two_robot_start_requires_both_start_nodes(tmp_path):
    profile = prepare_module.prepare(MAP, LAYOUT, simulation=True)
    journal = Journal(tmp_path / "control.json")
    try:
        transport = MemoryTransport()
        control = TrialControl(profile, transport, journal)
        state = deepcopy(control.state)
        for role, robot in profile["robots"].items():
            state["statuses"][robot["robot_id"]] = FakePinky(profile, role).state
            control.heard[robot["robot_id"]] = time.monotonic()
        state["statuses"]["pinky3"]["node_id"] = "N11"
        journal.save(state)
        assert control.start_error() == "unexpected route start"
        with pytest.raises(ValueError, match="unexpected route start"):
            control.start()
        assert transport.sent == []
    finally:
        journal.close()
