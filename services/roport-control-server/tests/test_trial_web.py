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
def single_profile(request):
    return prepare_module.prepare(
        MAP, LAYOUT, simulation=True, single_robot=getattr(request, "param", True)
    )


def view(path):
    return TrialView(path, TOKEN, load_map(MAP, LAYOUT))


def fence(snapshot, action="start"):
    trial = snapshot["trial"]
    command = {k: trial[k] for k in ("session_id", "task_id", "phase")}
    command["action"] = action
    if action == "dummy":
        command["stop_session_id"] = trial["gate"]["stop_session_id"]
    if action == "request":
        command["items"] = [{"package_id": trial["package_id"], "quantity": 1}]
    return command


@pytest.fixture
def local_owner(tmp_path, single_profile):
    journal = Journal(tmp_path / "control.json")
    transport = MemoryTransport()
    control = TrialControl(single_profile, transport, journal)
    state = deepcopy(control.state)
    for role, robot in single_profile["robots"].items():
        robot_id = robot["robot_id"]
        state["statuses"][robot_id] = {
            **FakePinky(single_profile, role).state,
            "boot_id": "boot-one",
        }
        control.heard[robot_id] = time.monotonic()
    journal.save(state)
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


def test_motion_rejection_diagnostics_reach_ui_without_blocking_start(local_owner):
    from roport_control_server.adapters.mqtt.motion import MotionReceiver

    client, control, _, _ = local_owner
    control.motion = MotionReceiver(control.config, MemoryTransport())
    control.motion.rejected = 2
    control.motion.rejection_reasons = {"clock_skew_or_expired": 2}
    snapshot = client.get("/api/snapshot?mode=trial").json()
    assert snapshot["trial"]["motion_diagnostics"] == {
        "rejected": 2,
        "reasons": {"clock_skew_or_expired": 2},
    }
    assert snapshot["trial"]["can_start"]
    assert control.state["phase"] == "READY" and control.state["fault"] is None


def test_trial_start_auth_reconnect_duplicate_and_stop(local_owner):
    client, control, _, transport = local_owner
    bootstrap = client.get("/api/bootstrap?mode=trial").json()
    assert bootstrap["trial"] and len(bootstrap["robots"]) == 6
    assert bootstrap["map"]["id"] == "airport_map"
    assert bootstrap["map"]["layout"]["mounts"]
    assert client.get("/api/bootstrap").json()["mode"] == "trial"
    assert client.get("/api/bootstrap?mode=offline").json()["mode"] == "offline"
    assert bootstrap["trial_nodes"]["N16"]
    assert {"N21", "N22", "N23", "N24"} <= bootstrap["trial_nodes"].keys()
    assert control.config["graph_version"] == "line-v14-20261106"
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
    row = client.get("/api/snapshot?mode=trial").json()["robots"][0]
    assert row["pose"] == snapshot["robots"][0]["pose"]
    assert row["pose_stale"] and not row["pose_live"]
    assert "마지막 확인" in row["pose_source"]


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
        plan_index=1,
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
    time.sleep(0.21)  # Snapshot reads share a bounded 200 ms cache.
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


@pytest.mark.parametrize("scenario", ["single", "two", "smartcart"])
@pytest.mark.parametrize("scope", ["line-only", "nav2"])
def test_real_mqtt_web_full_trial_and_exclusive_owner(
    tmp_path, broker, scope, scenario
):
    single, smartcart_only = scenario == "single", scenario == "smartcart"
    profile = prepare_module.prepare(MAP, LAYOUT, simulation=True, single_robot=single)
    roles = (
        {"smartcart": profile["robots"]["smartcart"]}
        if smartcart_only
        else profile["robots"]
    )
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
        for role in roles:
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
        if smartcart_only:
            owner_args.append("--smartcart-only")
        owner = start(owner_args)
        wait_until(lambda: channel.exists())
        projector = view(channel)
        wait_until(lambda: projector.snapshot()["trial"]["can_start"], timeout=10)
        # Even another journal/socket must not create a second MQTT owner.
        env["ROPORT_MQTT_HOST"] = "localhost"
        duplicate = start(
            [
                *owner_args[:],
                "--journal",
                str(tmp_path / "other.json"),
                "--socket",
                str(tmp_path / "other.sock"),
            ]
        )
        env["ROPORT_MQTT_HOST"] = "127.0.0.1"
        assert duplicate.wait(5) != 0
        assert json.loads((tmp_path / "other.json").read_text())["phase"] == "READY"
        app = create_app(
            MAP, trial_socket=channel, trial_layout=LAYOUT, trial_token=TOKEN
        )
        with TestClient(app) as client:
            first = client.get("/api/snapshot?mode=trial").json()
            assert first["trial"]["smartcart_only"] is smartcart_only
            if smartcart_only:
                assert first["robots"][0]["status"] == {}
                assert first["robots"][2]["status"]["node_id"] == "H1"
            assert (
                client.post(
                    "/api/trial/commands", headers=HEADERS, json=fence(first)
                ).status_code
                == 200
            )
            waits = (
                ("WAIT_CART_LOAD",)
                if smartcart_only
                else (
                    ("WAIT_INBOUND", "WAIT_CART_LOAD")
                    if single
                    else ("WAIT_LOAD", "WAIT_INBOUND", "WAIT_REQUEST", "WAIT_CART_LOAD")
                )
            )
            for phase in waits:
                wait_until(
                    lambda phase=phase: projector.snapshot()["trial"]["phase"] == phase,
                    timeout=10,
                )
                current = client.get("/api/snapshot?mode=trial").json()
                command = fence(
                    current, "request" if phase == "WAIT_REQUEST" else "dummy"
                )
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
            cart = final["robots"][0 if single else 2]
            if scenario == "two":
                assert final["robots"][0]["status"]["node_id"] == "N16"
                assert final["robots"][0]["pose_source"].startswith("마지막 확인 노드")
                assert final["trial"]["request"]["items"] == [
                    {"package_id": profile["package_id"], "quantity": 1}
                ]
            if scope == "nav2":
                assert cart["pose_source"].startswith("Nav2 완료 응답 위치")
                assert cart["pose_stale"]
            else:
                assert cart["status"]["node_id"] == "N12"
                assert cart["status"]["stopped"]
                assert "Nav2 제외" in final["trial"]["label"]
        journal = json.loads((tmp_path / "control.json").read_text())
        assert len(journal["transfers"]) == (
            1 if smartcart_only else 2 if single else 3
        )
        expected = (
            {"cart_to_delivery", "cart_to_switch"}
            if smartcart_only
            else set(profile["routes"])
        )
        if scope == "nav2":
            expected.update({"mode_switch", "navigate"})
        assert set(journal["completed"]) == expected
        if smartcart_only:
            assert set(journal["statuses"]) == {
                profile["robots"]["smartcart"]["robot_id"]
            }
            assert not (tmp_path / "logistics.json").exists()
            assert all(
                e["source"] == profile["robots"]["smartcart"]["robot_id"]
                for e in journal["events"].values()
            )
        for role in roles:
            robot_journal = json.loads((tmp_path / f"{role}.json").read_text())
            assert all(
                record["command"]["target"] == profile["robots"][role]["robot_id"]
                for record in robot_journal["records"].values()
            )
            if scope == "line-only":
                assert all(
                    record["command"]["type"] == "route.requested"
                    for record in robot_journal["records"].values()
                )
        owner.send_signal(signal.SIGTERM if scope == "nav2" else signal.SIGINT)
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


def test_virtual_reset_requires_launcher_auth_and_current_simulation(
    local_owner, tmp_path
):
    client, control, _, transport = local_owner
    url = "/api/trial/commands"
    snapshot = client.get("/api/snapshot?mode=trial").json()
    request = fence(snapshot, "reset")
    assert not snapshot["trial"]["reset_available"]
    assert client.post(url, headers=HEADERS, json=request).status_code == 409
    marker = tmp_path / "reset.request"
    client.app.state.trial.reset_path = marker
    for headers in (
        {"origin": "http://testserver"},
        {**HEADERS, "origin": "http://other"},
    ):
        assert client.post(url, headers=headers, json=request).status_code == 403
    assert not marker.exists()
    for field in ("session_id", "task_id"):
        assert (
            client.post(
                url, headers=HEADERS, json={**request, field: "old"}
            ).status_code
            == 409
        )
    control.config["mode"] = "hardware"
    assert not client.get("/api/snapshot?mode=trial").json()["trial"]["reset_available"]
    assert client.post(url, headers=HEADERS, json=request).status_code == 403
    control.config["mode"] = "simulation"
    state_before = deepcopy(control.state)
    for _ in range(2):
        assert client.post(url, headers=HEADERS, json=request).status_code == 200
    assert json.loads(marker.read_text()) == request
    snapshot = client.get("/api/snapshot?mode=trial").json()
    assert snapshot["trial"]["reset_pending"] and not snapshot["trial"]["can_reset"]
    assert client.post(url, headers=HEADERS, json=fence(snapshot)).status_code == 409
    assert control.state == state_before and not transport.sent


@pytest.mark.parametrize("single_profile", [False], indirect=True)
@pytest.mark.parametrize("phase", ["READY", "WAIT_REQUEST"])
@pytest.mark.parametrize(
    "problem", ["cart_node", "cart_stale", "cart_moving", "warehouse", "broker"]
)
def test_two_robot_actions_recheck_both_robots_without_recording_false_request(
    local_owner, phase, problem
):
    client, control, _, transport = local_owner
    state = deepcopy(control.state)
    state["phase"] = phase
    if problem == "cart_node":
        state["statuses"]["pinky3"]["node_id"] = "N11"
    elif problem == "cart_stale":
        control.heard["pinky3"] -= 4
    elif problem == "cart_moving":
        state["statuses"]["pinky3"]["stopped"] = False
    elif problem == "warehouse":
        state["statuses"]["pinky1"]["node_id"] = "N11"
    else:
        transport.ready = False
    control.journal.save(state)
    action = "start" if phase == "READY" else "request"
    snapshot = client.get("/api/snapshot?mode=trial").json()
    assert not snapshot["trial"][f"can_{action}"]
    response = client.post(
        "/api/trial/commands", headers=HEADERS, json=fence(snapshot, action)
    )
    assert response.status_code == 409
    assert control.state == state and not transport.sent


@pytest.mark.parametrize("single_profile", [False], indirect=True)
@pytest.mark.parametrize(
    "items",
    [
        [],
        None,
        [{"package_id": "wrong", "quantity": 1}],
        [{"package_id": "trial-package-1", "quantity": True}],
        [{"package_id": "trial-package-1", "quantity": 2}],
    ],
)
def test_two_robot_request_rejects_wrong_package_and_quantity(local_owner, items):
    client, control, _, transport = local_owner
    state = deepcopy(control.state)
    state["phase"] = "WAIT_REQUEST"
    control.journal.save(state)
    request = fence(client.get("/api/snapshot?mode=trial").json(), "request")
    request["items"] = items
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=request).status_code
        == 409
    )
    assert control.state == state and not transport.sent


@pytest.mark.parametrize("single_profile", [False], indirect=True)
def test_two_robot_dummy_does_not_record_transfer_before_start_validation(local_owner):
    client, control, _, transport = local_owner
    state = deepcopy(control.state)
    state.update(phase="WAIT_LOAD", gate={"stop_session_id": "current-stop"})
    control.journal.save(state)  # Still N16, not the expected N01 loading stop.
    request = fence(client.get("/api/snapshot?mode=trial").json(), "dummy")
    assert (
        client.post("/api/trial/commands", headers=HEADERS, json=request).status_code
        == 409
    )
    assert control.state == state and not transport.sent


def test_live_motion_crosses_nav2_and_freezes_without_authorizing_tasks(local_owner):
    from uuid import uuid4

    from test_motion import delivery as motion_delivery
    from test_motion import sample

    from roport_control_server.adapters.mqtt.motion import MotionReceiver

    client, control, _, _ = local_owner
    state = deepcopy(control.state)
    state["statuses"]["pinky1"]["boot_id"] = str(uuid4())
    control.journal.save(state)
    motion = MotionReceiver(control.config, MemoryTransport())
    control.motion = motion
    statuses = control.state["statuses"]
    boot = statuses["pinky1"]["boot_id"]
    motion.consume(motion_delivery(sample(control.config, boot)), statuses)
    time.sleep(0.21)  # Allow the shared snapshot cache to observe the new sample.
    row = client.get("/api/snapshot?mode=trial").json()["robots"][0]
    assert row["pose"] == [0.1, 0.2, 0, 0.3] and row["pose_live"]
    control.heard["pinky1"] = time.monotonic() - 5
    time.sleep(0.21)  # Allow the shared snapshot cache to observe the new sample.
    row = client.get("/api/snapshot?mode=trial").json()["robots"][0]
    assert row["stale"] and row["pose_live"]  # Motion cannot refresh task readiness.
    state = deepcopy(control.state)
    state["statuses"]["pinky1"].update(mode="nav2", node_id=None)
    control.journal.save(state)
    time.sleep(0.21)  # Allow the shared snapshot cache to observe the new sample.
    row = client.get("/api/snapshot?mode=trial").json()["robots"][0]
    assert row["pose"] == [0.1, 0.2, 0, 0.3]
    state_before = deepcopy(control.state)
    motion.samples["pinky1"]["received"] -= 2
    time.sleep(0.21)  # Allow the shared snapshot cache to observe the new sample.
    row = client.get("/api/snapshot?mode=trial").json()["robots"][0]
    assert row["pose"] == [0.1, 0.2, 0, 0.3] and row["pose_stale"]
    assert row["pose_age"] >= 2
    assert control.state == state_before
    # New browser/API projection can recover the last received point from the owner.
    observer = view("unused")
    observer.rpc = lambda request: client.app.state.trial.rpc(request)
    assert observer.snapshot()["robots"][0]["pose"] == row["pose"]
    state = deepcopy(control.state)
    state["statuses"]["pinky1"]["boot_id"] = str(uuid4())
    control.journal.save(state)
    time.sleep(0.21)
    assert client.get("/api/snapshot?mode=trial").json()["robots"][0]["pose"] is None


@pytest.mark.parametrize("motion_newer", [False, True])
def test_final_position_uses_latest_observation(local_owner, motion_newer):
    from datetime import UTC, datetime, timedelta
    from uuid import uuid4

    from test_motion import delivery as motion_delivery
    from test_motion import sample

    from roport_control_server.adapters.mqtt.motion import MotionReceiver

    client, control, _, _ = local_owner
    state = deepcopy(control.state)
    boot = str(uuid4())
    state["statuses"]["pinky1"].update(boot_id=boot, mode="nav2", node_id=None)
    now = datetime.now(UTC)
    state["phase"] = "COMPLETE"
    state["completed"]["navigate"] = {
        "type": "navigate.completed",
        "source": "pinky1",
        "payload": {
            "observed_at": (now - timedelta(seconds=0.2)).isoformat(),
            "pose": {"x": 0.15, "y": 0.25, "yaw": 0.4},
        },
    }
    control.journal.save(state)
    control.motion = MotionReceiver(control.config, MemoryTransport())
    observation = sample(
        control.config,
        boot,
        observed_at=(now - timedelta(seconds=0.1 if motion_newer else 0.3)).isoformat(),
    )
    control.motion.consume(motion_delivery(observation), state["statuses"])
    row = client.get("/api/snapshot?mode=trial").json()["robots"][0]
    assert row["pose"] == ([0.1, 0.2, 0, 0.3] if motion_newer else [0.15, 0.25, 0, 0.4])
    assert row["pose_live"] is motion_newer


@pytest.mark.parametrize("single_profile", [False], indirect=True)
@pytest.mark.parametrize("failure", [TimeoutError, ConnectionError])
@pytest.mark.parametrize("action", ["start", "dummy", "request"])
def test_web_publish_failure_preserves_unknown_outcome(
    local_owner, monkeypatch, failure, action
):
    client, control, _, transport = local_owner
    state = deepcopy(control.state)
    if action in {"dummy", "request"}:
        phase = "WAIT_CART_LOAD" if action == "dummy" else "WAIT_REQUEST"
        state["phase"] = phase
        state["plan_index"] = next(
            i for i, step in enumerate(control.plan["steps"]) if step["id"] == phase
        )
        if action == "dummy":
            state["gate"] = {"stop_session_id": "current-gate"}
            state["statuses"]["pinky3"].update(
                node_id="N11",
                heading_rad=control.config["routes"]["cart_to_switch"][
                    "start_heading_rad"
                ],
            )
    control.journal.save(state)
    request = fence(client.get("/api/snapshot?mode=trial").json(), action)
    attempts = []

    def publish(destination, message):
        attempts.append(message)
        raise failure("simulated publish failure")

    monkeypatch.setattr(transport, "publish", publish)
    response = client.post("/api/trial/commands", headers=HEADERS, json=request)
    assert response.status_code == 409
    assert "transport_outcome_unknown" in response.text
    assert len(attempts) == 1
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "transport_outcome_unknown"
    assert control.state["active"]["command_id"] == attempts[0]["command_id"]
    current = client.get("/api/snapshot?mode=trial").json()
    assert current["trial"]["available"] and not current["trial"]["can_start"]
    control.hold("operator_stopped")
    assert control.state["fault"] == "transport_outcome_unknown"


def test_socket_journal_failure_still_stops_owner(local_owner, monkeypatch):
    client, control, channel, transport = local_owner
    request = fence(client.get("/api/snapshot?mode=trial").json())

    def save(_state):
        raise OSError("simulated disk failure")

    monkeypatch.setattr(control.journal, "save", save)
    with pytest.raises(OSError, match="disk failure"):
        channel.dispatch({**request, "expires_at": time.time() + 1})
    assert not transport.sent


def test_large_snapshot_returns_error_without_killing_owner(local_owner, monkeypatch):
    _, control, channel, _ = local_owner
    projector = view(channel.path)
    with monkeypatch.context() as patch:
        patch.setattr(channel, "snapshot", lambda: {"oversize": "x" * 65536})
        with pytest.raises(ValueError, match="message limit"):
            projector.rpc({"action": "snapshot"})
    assert projector.rpc({"action": "snapshot"})["state"]["phase"] == "READY"
    assert control.state["fault"] is None


@pytest.mark.parametrize(
    "mode,url,peer,expected",
    [
        ("hardware", "http://127.0.0.1", "127.0.0.1", 200),
        ("hardware", "http://localhost", "127.0.0.1", 200),
        ("hardware", "http://[::1]", "::1", 200),
        ("hardware", "https://control.example", "192.0.2.10", 200),
        ("hardware", "http://control.example", "192.0.2.10", 403),
        ("hardware", "http://control.example", "127.0.0.1", 403),
        ("hardware", "http://localhost", "192.0.2.10", 403),
        ("simulation", "http://control.example", "192.0.2.10", 200),
    ],
)
def test_hardware_command_transport_boundary(local_owner, mode, url, peer, expected):
    _, control, channel, transport = local_owner
    control.config["mode"] = mode  # MemoryTransport only; no hardware is started.
    app = create_app(
        MAP, layout_path=LAYOUT, trial_socket=channel.path, trial_token=TOKEN
    )
    with TestClient(app, base_url=url, client=(peer, 12345)) as client:
        request = fence(client.get("/api/snapshot?mode=trial").json())
        response = client.post(
            "/api/trial/commands",
            json=request,
            headers={
                "origin": url,
                "x-roport-trial-token": TOKEN,
                "x-forwarded-proto": "https",
            },
        )
        assert response.status_code == expected
        assert len(transport.sent) == (1 if expected == 200 else 0)


@pytest.mark.parametrize("peer,expected", [("127.0.0.1", 200), ("192.0.2.10", 403)])
def test_hardware_https_proxy_must_be_trusted(local_owner, peer, expected):
    from uvicorn.middleware.proxy_headers import ProxyHeadersMiddleware

    _, control, channel, transport = local_owner
    control.config["mode"] = "hardware"
    app = ProxyHeadersMiddleware(
        create_app(
            MAP, layout_path=LAYOUT, trial_socket=channel.path, trial_token=TOKEN
        ),
        trusted_hosts=["127.0.0.1"],
    )
    with TestClient(
        app, base_url="http://control.example", client=(peer, 12345)
    ) as client:
        request = fence(client.get("/api/snapshot?mode=trial").json())
        response = client.post(
            "/api/trial/commands",
            json=request,
            headers={
                "origin": "https://control.example",
                "x-roport-trial-token": TOKEN,
                "x-forwarded-proto": "https",
                "x-forwarded-for": "192.0.2.20",
            },
        )
        assert response.status_code == expected
        assert len(transport.sent) == (1 if expected == 200 else 0)


def test_snapshot_reads_share_cache_and_commands_refresh(local_owner, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor

    _, _, channel, _ = local_owner
    projector = view(channel.path)
    calls = []
    original = projector.rpc

    def rpc(request):
        calls.append(request["action"])
        return original(request)

    monkeypatch.setattr(projector, "rpc", rpc)
    with ThreadPoolExecutor(max_workers=8) as pool:
        snapshots = list(pool.map(lambda _: projector.snapshot(), range(20)))
    assert calls == ["snapshot"]
    assert len({s["revision"] for s in snapshots}) == 20
    projector.command(fence(snapshots[-1]))
    assert projector.snapshot()["trial"]["phase"] == "to_delivery"
    assert calls == ["snapshot", "snapshot", "start", "snapshot"]
    projector.socket_path += "-absent"
    time.sleep(0.21)
    assert not projector.snapshot()["trial"]["available"]
    for _ in range(10):
        assert not projector.snapshot()["trial"]["available"]
    assert calls.count("snapshot") == 4


def test_late_owner_snapshot_supplies_routes(local_owner):
    _, _, channel, _ = local_owner
    projector = view(str(channel.path) + "-absent")
    assert projector.bootstrap()["trial_routes"] == []
    projector.socket_path = str(channel.path)
    assert (
        projector.snapshot()["trial"]["routes"] == projector.bootstrap()["trial_routes"]
    )
    assert projector.snapshot()["trial"]["routes"]


def test_startup_failure_keeps_unused_journal_ready_and_cleans_socket(
    tmp_path, single_profile, monkeypatch
):
    from roport_control_server.executor import pinky_trial

    profile = tmp_path / "trial.json"
    profile.write_text(json.dumps(single_profile))
    journal = tmp_path / "control.json"
    channel = tmp_path / "s"
    monkeypatch.setenv("ROPORT_MQTT_HOST", "127.0.0.1")
    monkeypatch.setenv("ROPORT_MQTT_TLS", "0")
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "trial",
            "--config",
            str(profile),
            "--journal",
            str(journal),
            "--socket",
            str(channel),
        ],
    )

    class Unavailable(MemoryTransport):
        def __init__(self, *_args):
            super().__init__()

        def start(self):
            raise ConnectionError("broker unavailable")

    monkeypatch.setattr(pinky_trial, "MqttTransport", Unavailable)
    previous_handler = signal.getsignal(signal.SIGTERM)
    for _ in range(2):
        with pytest.raises(ConnectionError, match="broker unavailable"):
            pinky_trial.main()
        state = json.loads(journal.read_text())
        assert state["phase"] == "READY" and state["fault"] is None
        assert not channel.exists()
        assert signal.getsignal(signal.SIGTERM) == previous_handler


@pytest.mark.parametrize(
    "action",
    ["configure", "save_plan", "load_plan", "route_plan", "submit_delivery", "reset"],
)
def test_hardware_plan_and_reset_requests_also_require_secure_transport(
    local_owner, action
):
    _, control, channel, transport = local_owner
    control.config["mode"] = "hardware"
    projector = view(channel.path)
    request = fence(projector.snapshot(), action)
    with pytest.raises(PermissionError, match="HTTPS"):
        projector.command(request)
    assert not transport.sent
    assert control.state["phase"] == "READY"
