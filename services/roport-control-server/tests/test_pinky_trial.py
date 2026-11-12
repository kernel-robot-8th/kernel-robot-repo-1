import importlib.util
import json
import shutil
import subprocess
import sys
import time
from contextlib import closing
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Queue
from types import SimpleNamespace

import pytest
from mock_pinky_trial import FakePinky

from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import encode, topic
from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport
from roport_control_server.adapters.mqtt.pinky_trial import PinkyTrialClient
from roport_control_server.adapters.mqtt.trial_wire import (
    command,
    reply,
    validate_trial,
)
from roport_control_server.executor.pinky_trial import TrialControl, validate_config

SERVICE = Path(__file__).resolve().parents[1]
prepare_spec = importlib.util.spec_from_file_location(
    "prepare_trial", SERVICE / "tools/prepare_pinky_trial.py"
)
prepare_module = importlib.util.module_from_spec(prepare_spec)
prepare_spec.loader.exec_module(prepare_module)


@pytest.fixture
def profile():
    pinky = SERVICE.parent / "roport-pinky"
    return prepare_module.prepare(
        pinky / "maps/airport_map.yaml",
        pinky / "src/roport_gz_world/layout/roport_airport.yaml",
        simulation=True,
    )


@pytest.fixture
def single_profile():
    pinky = SERVICE.parent / "roport-pinky"
    return prepare_module.prepare(
        pinky / "maps/airport_map.yaml",
        pinky / "src/roport_gz_world/layout/roport_airport.yaml",
        simulation=True,
        single_robot=True,
    )


class MemoryTransport:
    ready = True

    def __init__(self):
        self.sent = []
        self.incoming = Queue()
        self.acked = []

    def publish(self, destination, message):
        self.sent.append((destination, deepcopy(message)))

    def receive(self, timeout):
        return self.incoming.get_nowait()

    def ack(self, message):
        self.acked.append(message)

    def stop(self):
        pass


def delivery(message, channel="cmd", *, retained=False):
    device = message["source"] if channel == "event" else message["target"]
    return SimpleNamespace(
        payload=encode(message),
        topic=topic(message["site_id"], "robot", device, channel),
        retained=retained,
        qos=1,
    )


@pytest.fixture
def pair(profile, tmp_path):
    fake = FakePinky(profile, "logistics")
    robot = profile["robots"]["logistics"]
    client = PinkyTrialClient(
        MqttConfig("127.0.0.1", "test-endpoint"),
        site=profile["site_id"],
        control=profile["control_id"],
        robot=robot["robot_id"],
        binding_id=robot["binding_id"],
        journal_path=tmp_path / "robot.json",
        observe=fake.observe,
        validate_action=fake.validate_action,
        dispatch=fake.dispatch,
        safe_stop=fake.stop,
    )
    client.transport = MemoryTransport()
    request = command(
        site=profile["site_id"],
        control=profile["control_id"],
        robot=robot["robot_id"],
        task="trial-task",
        step="to_loading",
        kind="route",
        payload={
            **profile["routes"]["to_loading"],
            "binding_id": robot["binding_id"],
            "expected_boot_id": client.boot,
        },
    )
    yield client, fake, request
    client.close()


def test_durable_duplicate_and_conflicting_command(pair):
    client, fake, request = pair
    client.handle(delivery(request))
    client.handle(delivery(request))
    assert fake.execution_count == 1
    fake.finish(client)
    client.handle(delivery(request))
    assert fake.execution_count == 1
    assert client.transport.sent[-1][1]["type"] == "route.completed"
    conflict = deepcopy(request)
    conflict["payload"]["speed_limit_mps"] = 0.05
    with pytest.raises(ValueError, match="idempotency"):
        client.handle(delivery(conflict))
    assert fake.execution_count == 1


@pytest.mark.parametrize(
    "change",
    ["retained", "expired", "binding", "boot", "node", "heading", "version", "busy"],
)
def test_no_dispatch_on_bad_command(pair, change):
    client, fake, request = pair
    if change == "expired":
        request["sent_at"] = (datetime.now(UTC) - timedelta(seconds=20)).isoformat()
        request["expires_at"] = (datetime.now(UTC) - timedelta(seconds=10)).isoformat()
    elif change == "binding":
        request["payload"]["binding_id"] = "other-binding"
    elif change == "boot":
        request["payload"]["expected_boot_id"] = "old-boot"
    elif change == "node":
        fake.state["node_id"] = "N02"
    elif change == "heading":
        fake.state["heading_rad"] = 1.5
    elif change == "version":
        fake.state["graph_version"] = "old-version"
    elif change == "busy":
        client.handle(delivery(request))
        request = deepcopy(request)
        request["command_id"] = "second-command"
    try:
        client.handle(delivery(request, retained=change == "retained"))
    except ValueError:
        pass
    assert fake.execution_count == (1 if change == "busy" else 0)


def test_restart_does_not_resume_uncertain_action(pair, profile, tmp_path):
    client, fake, request = pair
    client.handle(delivery(request))
    old_boot = client.boot
    client.journal.close()
    resumed = PinkyTrialClient(
        MqttConfig("127.0.0.1", "test-endpoint"),
        site=profile["site_id"],
        control=profile["control_id"],
        robot="pinky1",
        binding_id=client.binding,
        journal_path=tmp_path / "robot.json",
        observe=fake.observe,
        validate_action=fake.validate_action,
        dispatch=fake.dispatch,
        safe_stop=fake.stop,
    )
    resumed.transport = MemoryTransport()
    try:
        assert resumed.boot != old_boot and resumed.halted == "recovery_required"
        resumed.handle(delivery(request))
        assert fake.execution_count == 1
    finally:
        resumed.close()


def test_store_failure_happens_before_robot_dispatch(pair, monkeypatch):
    client, fake, request = pair

    def fail(data):
        raise OSError("disk full")

    monkeypatch.setattr(client.journal, "save", fail)
    with pytest.raises(OSError):
        client.handle(delivery(request))
    assert fake.execution_count == 0


def test_journal_exclusive_and_atomic(tmp_path, monkeypatch):
    journal = Journal(tmp_path / "state.json")
    try:
        journal.save({"value": 1})
        with pytest.raises(BlockingIOError):
            Journal(journal.path)

        def fail(*args):
            raise OSError("rename failed")

        monkeypatch.setattr(
            "roport_control_server.adapters.mqtt.journal.os.replace", fail
        )
        with pytest.raises(OSError):
            journal.save({"value": 2})
        assert json.loads(journal.path.read_text()) == {"value": 1}
        with pytest.raises(OSError):
            journal.save({"value": 3})
    finally:
        journal.close()


@pytest.mark.parametrize(
    "change",
    ["unknown_field", "bool_number", "wrong_endpoint", "nonfinite", "wrong_profile"],
)
def test_strict_wire_schema(pair, change):
    _, _, request = pair
    if change == "unknown_field":
        request["payload"]["drive_now"] = True
    elif change == "bool_number":
        request["payload"]["speed_limit_mps"] = True
    elif change == "wrong_endpoint":
        request["payload"]["goal"]["node_id"] = "N19"
    elif change == "nonfinite":
        request["payload"]["start_heading_rad"] = float("nan")
    else:
        request["profile"] = "other-profile"
    with pytest.raises(ValueError):
        validate_trial(request)


@pytest.mark.parametrize("change", ["edge", "switch", "shared_stop", "hardware"])
def test_invalid_trial_config(profile, change):
    if change == "edge":
        profile["nodes"]["N16"] = []
    elif change == "switch":
        profile["routes"]["cart_to_switch"]["route"][-1] = "N08"
    elif change == "shared_stop":
        profile["routes"]["cart_to_delivery"]["goal"]["node_id"] = "N19"
    else:
        profile["mode"] = "hardware"
    with pytest.raises(ValueError):
        validate_config(profile)


@pytest.fixture
def control_memory(profile, tmp_path):
    transport = MemoryTransport()
    journal = Journal(tmp_path / "control.json")
    control = TrialControl(profile, transport, journal)
    for role, robot in profile["robots"].items():
        fake = FakePinky(profile, role)
        state = deepcopy(control.state)
        state["statuses"][robot["robot_id"]] = {
            **fake.observe(),
            "binding_id": robot["binding_id"],
            "boot_id": "test-boot",
        }
        state["boot_ids"][robot["robot_id"]] = "test-boot"
        journal.save(state)
        control.heard[robot["robot_id"]] = time.monotonic()
    yield control
    journal.close()


def test_wrong_dummy_request_and_restart_are_fenced(control_memory):
    control = control_memory
    with pytest.raises(ValueError):
        control.dummy("previous-stop")
    with pytest.raises(ValueError):
        control.request([{"package_id": "trial-package-1", "quantity": 1}])
    control.start()
    resumed = TrialControl(control.config, control.transport, control.journal)
    assert resumed.state["phase"] == "HOLD"
    assert len(control.transport.sent) == 1


@pytest.mark.parametrize("role", ["logistics", "smartcart"])
@pytest.mark.parametrize("problem", ["stale", "restart"])
def test_either_robot_loss_holds_two_robot_scenario(control_memory, role, problem):
    control = control_memory
    device = control.config["robots"][role]["robot_id"]
    control.start()
    control.poll(0)
    if problem == "stale":
        control.heard[device] -= 4
    else:
        cmd = next(c for c in control.pending_status.values() if c["target"] == device)
        event = reply(
            cmd,
            "status.completed",
            {
                **control.state["statuses"][device],
                "observed_at": datetime.now(UTC).isoformat(),
                "phase_revision": 1,
                "boot_id": "new-boot",
            },
        )
        control.transport.incoming.put(delivery(event, "event"))
    control.poll(0)
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == (
        "robot_status_timeout" if problem == "stale" else "robot_restarted"
    )
    assert control.state["completed"] == {}
    assert sum(c["type"] == "route.requested" for _, c in control.transport.sent) == 1


def status_reply(control, request):
    robot = control.config["robots"]["logistics"]
    return reply(
        request,
        "status.completed",
        {
            **FakePinky(control.config, "logistics").observe(),
            "binding_id": robot["binding_id"],
            "boot_id": "test-boot",
            "phase_revision": 1,
        },
    )


@pytest.mark.parametrize("running", [False, True])
def test_delayed_status_survives_next_poll(control_memory, monkeypatch, running):
    control = control_memory
    clock = [time.monotonic()]
    monkeypatch.setattr(
        "roport_control_server.executor.pinky_trial.monotonic", lambda: clock[0]
    )
    if running:
        control.start()
    control.poll(0)
    request = next(
        c for c in control.pending_status.values() if c["target"] == "pinky1"
    )
    event = status_reply(control, request)
    clock[0] += 1.2
    control.poll(0)
    control.transport.incoming.put(delivery(event, "event"))
    control.poll(0)
    assert control.state["statuses"]["pinky1"] == event["payload"]
    assert control.heard["pinky1"] == clock[0]
    assert request["command_id"] not in control.pending_status
    assert control.state["phase"] == ("to_loading" if running else "READY")


def test_pending_status_expires_without_accumulating(control_memory):
    control = control_memory
    control.poll(0)
    expired_ids = set(control.pending_status)
    for request in control.pending_status.values():
        request["expires_at"] = (datetime.now(UTC) - timedelta(seconds=1)).isoformat()
    control.last_poll -= 1
    control.poll(0)
    assert len(control.pending_status) == 2
    assert not expired_ids.intersection(control.pending_status)


def test_expired_status_request_cannot_refresh_robot(control_memory):
    control = control_memory
    control.poll(0)
    request = next(
        c for c in control.pending_status.values() if c["target"] == "pinky1"
    )
    event = status_reply(control, request)
    request["expires_at"] = (datetime.now(UTC) - timedelta(seconds=1)).isoformat()
    before, heard = deepcopy(control.state), dict(control.heard)
    control.transport.incoming.put(delivery(event, "event"))
    control.poll(0)
    assert control.state == before
    assert control.heard == heard


def test_older_status_cannot_replace_newer_state(control_memory):
    control = control_memory
    control.poll(0)
    request = next(
        c for c in control.pending_status.values() if c["target"] == "pinky1"
    )
    event = status_reply(control, request)
    state = deepcopy(control.state)
    state["statuses"]["pinky1"].update(
        node_id="N14",
        observed_at=(datetime.now(UTC) + timedelta(milliseconds=100)).isoformat(),
    )
    control.journal.save(state)
    heard = dict(control.heard)
    control.transport.incoming.put(delivery(event, "event"))
    control.poll(0)
    assert control.state == state
    assert control.heard == heard


@pytest.mark.parametrize("field", ["sent_at", "observed_at"])
def test_pending_request_does_not_allow_stale_status(control_memory, field):
    control = control_memory
    control.poll(0)
    request = next(
        c for c in control.pending_status.values() if c["target"] == "pinky1"
    )
    event = status_reply(control, request)
    target = event if field == "sent_at" else event["payload"]
    target[field] = (datetime.now(UTC) - timedelta(seconds=4)).isoformat()
    before = deepcopy(control.state["statuses"])
    control.transport.incoming.put(delivery(event, "event"))
    control.poll(0)
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "invalid_trial_event"
    assert control.state["statuses"] == before


def test_pending_requests_do_not_extend_robot_watchdog(control_memory, monkeypatch):
    control = control_memory
    clock = [time.monotonic()]
    monkeypatch.setattr(
        "roport_control_server.executor.pinky_trial.monotonic", lambda: clock[0]
    )
    control.start()
    control.poll(0)
    clock[0] += 3.1
    control.poll(0)
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "robot_status_timeout"
    assert control.state["completed"] == {}


@pytest.mark.parametrize("kind", ["route", "status", "mode_switch", "navigate", "ack"])
def test_wrong_event_type_holds_without_crashing(control_memory, kind):
    control = control_memory
    control.start()
    event = deepcopy(control.state["active"])
    event["source"], event["target"] = event["target"], event["source"]
    binding = event["payload"]["binding_id"]
    fence = {"binding_id": binding, "expected_boot_id": "test-boot"}
    event["type"] = kind + ".requested"
    if kind == "status":
        event["payload"] = {"binding_id": binding}
    elif kind == "mode_switch":
        event["payload"] = {
            **fence,
            "node_id": "N12",
            "from_mode": "line",
            "to_mode": "nav2",
            "map_id": control.config["map_id"],
            "map_version": control.config["map_version"],
        }
    elif kind == "navigate":
        event["payload"] = {**fence, **control.config["nav_goal"]}
    elif kind == "ack":
        event["type"] = "event.ack"
        event["payload"] = {"acked_message_id": "old-event", "status": "stored"}
        del event["expires_at"]
    validate_trial(event)
    incoming = delivery(event, "event")
    control.transport.incoming.put(incoming)
    control.poll(0)
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "invalid_trial_event"
    assert control.state["completed"] == {}
    assert control.transport.acked == [incoming]
    assert sum(m["type"] == "route.requested" for _, m in control.transport.sent) == 1


@pytest.mark.parametrize(
    "change",
    [
        "node",
        "moving",
        "line_enabled",
        "boot",
        "stale",
        "wrong_step",
        "map",
        "other_robot",
        "binding",
    ],
)
def test_bad_completion_cannot_advance(control_memory, pair, change):
    control = control_memory
    client, fake, _ = pair
    state = deepcopy(control.state)
    state["statuses"]["pinky1"]["boot_id"] = client.boot
    state["boot_ids"]["pinky1"] = client.boot
    control.journal.save(state)
    control.start()
    cmd = control.state["active"]
    client.handle(delivery(cmd))
    fake.finish(client)
    event = deepcopy(client.transport.sent[-1][1])
    if change == "node":
        event["payload"]["node_id"] = "N19"
    elif change == "moving":
        event["payload"]["stopped"] = False
    elif change == "line_enabled":
        event["payload"]["line_enabled"] = True
    elif change == "boot":
        event["payload"]["boot_id"] = "restarted"
    elif change == "stale":
        event["sent_at"] = (datetime.now(UTC) - timedelta(seconds=10)).isoformat()
    elif change == "map":
        event["payload"]["map_version"] = "changed-map"
    elif change == "other_robot":
        other = control.config["robots"]["smartcart"]
        event["source"] = other["robot_id"]
        event["payload"]["binding_id"] = other["binding_id"]
    elif change == "binding":
        event["payload"]["binding_id"] = "other-binding"
    else:
        event["step_id"] = "older-step"
    with pytest.raises(ValueError):
        control.handle(delivery(event, "event"))
    assert control.state["phase"] == "to_loading"
    assert len(control.transport.sent) == 1


@pytest.mark.parametrize("single_robot", [False, True])
def test_complete_trial_over_real_mosquitto(
    profile, single_profile, broker, tmp_path, single_robot
):
    if single_robot:
        profile = single_profile
    port = broker[0]
    transport = MqttTransport(
        MqttConfig("127.0.0.1", "trial-control", port=port),
        [
            topic(profile["site_id"], "robot", r["robot_id"], "event")
            for r in profile["robots"].values()
        ],
    )
    journal = Journal(tmp_path / "control.json")
    control = TrialControl(profile, transport, journal)
    clients, fakes = [], []
    try:
        for role, robot in profile["robots"].items():
            fake = FakePinky(profile, role)
            client = PinkyTrialClient(
                MqttConfig("127.0.0.1", robot["robot_id"], port=port),
                site=profile["site_id"],
                control=profile["control_id"],
                robot=robot["robot_id"],
                binding_id=robot["binding_id"],
                journal_path=tmp_path / f"{role}.json",
                observe=fake.observe,
                validate_action=fake.validate_action,
                dispatch=fake.dispatch,
                safe_stop=fake.stop,
            )
            client.start()
            clients.append(client)
            fakes.append(fake)
        transport.start()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            control.poll(0.01)
            for client, fake in zip(clients, fakes, strict=True):
                client.poll(0.01)
                fake.finish(client)
            phase = control.state["phase"]
            assert phase != "HOLD", control.state["fault"]
            if phase == "READY" and len(control.state["statuses"]) == len(
                profile["robots"]
            ):
                control.start()
            elif phase.startswith("WAIT_"):
                if phase == "WAIT_REQUEST":
                    assert "return_warehouse" in control.state["completed"]
                    control.request(
                        [{"package_id": profile["package_id"], "quantity": 1}]
                    )
                else:
                    token = control.state["gate"]["stop_session_id"]
                    control.dummy(token)
                    if single_robot and phase == "WAIT_INBOUND":
                        assert control.state["phase"] == "WAIT_CART_LOAD"
                        assert control.state["active"] is None
                        assert control.state["gate"]["stop_session_id"] != token
                        assert control.state["gate"]["arrival_execution_id"] == token
                        assert fakes[0].execution_count == 1
                        with pytest.raises(ValueError):
                            control.dummy(token)
            if phase == "COMPLETE":
                break
        assert control.state["phase"] == "COMPLETE", control.state["phase"]
        assert len(control.state["completed"]) == (4 if single_robot else 7)
        assert len(control.state["transfers"]) == (2 if single_robot else 3)
        assert [f.execution_count for f in fakes] == ([4] if single_robot else [3, 4])
        if single_robot:
            assert list(control.state["completed"]) == [
                "to_delivery",
                "cart_to_switch",
                "mode_switch",
                "navigate",
            ]
            assert {e["source"] for e in control.state["events"].values()} == {"pinky1"}
        assert all(t["simulated"] for t in control.state["transfers"])
        for event in control.state["events"].values():
            validate_trial(event)
        deadline = time.monotonic() + 5
        while (
            any(client.journal.data["outbox"] for client in clients)
            and time.monotonic() < deadline
        ):
            for client in clients:
                client.poll(0.01)
            control.poll(0.01)
        assert all(not client.journal.data["outbox"] for client in clients)
    finally:
        for client in clients:
            client.close()
        transport.stop()
        journal.close()


def test_portable_folder_imports_without_control_installation(tmp_path):
    source = SERVICE / "src/roport_control_server/adapters/mqtt"
    shutil.copytree(
        source, tmp_path / "pinky_mqtt", ignore=shutil.ignore_patterns("__pycache__")
    )
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "from pinky_mqtt.pinky_trial import PinkyTrialClient; from pinky_mqtt.trial_wire import SCHEMAS; assert len(SCHEMAS) == 19",
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_stale_local_sensor_does_not_start_motion(pair):
    client, fake, request = pair
    old = fake.observe()
    old["observed_at"] = (datetime.now(UTC) - timedelta(seconds=5)).isoformat()
    client.observe = lambda: old
    client.handle(delivery(request))
    assert fake.execution_count == 0
    assert (
        client.transport.sent[-1][1]["payload"]["error_code"]
        == "stale_robot_observation"
    )


def test_endpoint_watchdog_calls_injected_stop(pair):
    from queue import Empty

    client, fake, request = pair
    client.handle(delivery(request))
    fake.state.update(stopped=False, line_enabled=True)
    client.last_poll = time.monotonic() - 4

    def empty(timeout):
        raise Empty

    client.transport.receive = empty
    client.poll()
    assert client.halted == "control_link_lost"
    assert fake.state["stopped"] and not fake.state["line_enabled"]
    assert client.journal.data["active"] is not None


@pytest.mark.parametrize(
    "change",
    ["line_enabled", "nav2_ready", "localization_ready", "drive_owner", "node", "pose"],
)
def test_mode_and_navigation_completion_require_evidence(profile, change):
    from roport_control_server.adapters.mqtt.trial_wire import check_result, reply

    kind = "navigate" if change == "pose" else "mode_switch"
    payload = (
        profile["nav_goal"]
        if kind == "navigate"
        else {
            "node_id": "N12",
            "from_mode": "line",
            "to_mode": "nav2",
            "map_id": profile["map_id"],
            "map_version": profile["map_version"],
        }
    )
    request = command(
        site=profile["site_id"],
        control=profile["control_id"],
        robot="pinky3",
        task="test-task",
        step=kind,
        kind=kind,
        payload={
            **payload,
            "binding_id": "test-binding",
            "expected_boot_id": "test-boot",
        },
    )
    state = {
        **FakePinky(profile, "smartcart").observe(),
        "node_id": "N12",
        "mode": "nav2",
        "stopped": True,
        "line_enabled": False,
        "nav2_ready": True,
        "localization_ready": True,
        "drive_owner": "nav2",
        "binding_id": "test-binding",
        "boot_id": "test-boot",
        "phase_revision": 2,
    }
    if change in {"nav2_ready", "localization_ready"}:
        state[change] = False
    elif change == "line_enabled":
        state[change] = True
    elif change == "drive_owner":
        state[change] = "line"
    elif change == "node":
        state["node_id"] = "N11"
    else:
        state.update(
            goal_id=payload["goal_id"],
            goal_version=payload["goal_version"],
            frame_id="map",
            nav2_result="SUCCEEDED",
            pose={"x": 2.0, "y": 0.0, "yaw": 0.0},
        )
    event = reply(request, kind + ".completed", state)
    with pytest.raises(ValueError):
        check_result(request, event)


def test_failed_action_holds_control(control_memory, pair):
    control = control_memory
    client, fake, _ = pair
    state = deepcopy(control.state)
    state["statuses"]["pinky1"]["boot_id"] = client.boot
    state["boot_ids"]["pinky1"] = client.boot
    control.journal.save(state)
    control.start()
    client.handle(delivery(control.state["active"]))
    client.report(control.state["active"], "failed", error_code="line_lost")
    control.handle(delivery(client.transport.sent[-1][1], "event"))
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "line_lost"
    assert fake.execution_count == 1


def test_single_profile_uses_measured_paths_and_public_goal(single_profile):
    import math

    import yaml
    from PIL import Image

    validate_config(single_profile)
    assert list(single_profile["robots"]) == ["logistics"]
    assert single_profile["routes"]["to_delivery"]["route"] == [
        "N16",
        "N14",
        "N02",
        "N10",
        "N18",
        "N07",
        "N23",
        "N19",
        "N11",
    ]
    assert single_profile["routes"]["cart_to_switch"]["route"] == [
        "N11",
        "N19",
        "N24",
        "N09",
        "N08",
        "N12",
    ]
    goal = single_profile["nav_goal"]
    assert goal["frame_id"] == "map"
    assert goal["pose"]["x"] == pytest.approx(2.576678, abs=1e-6)
    assert goal["pose"]["y"] == pytest.approx(-0.115347, abs=1e-6)
    # Check +/-15 cm around the candidate; this is not a Nav2 path test.
    path = SERVICE.parent / "roport-pinky/maps/airport_map.yaml"
    meta = yaml.safe_load(path.read_text())
    with Image.open(path.parent / meta["image"]) as image:
        col = math.floor((goal["pose"]["x"] - meta["origin"][0]) / meta["resolution"])
        row = (
            image.height
            - 1
            - math.floor((goal["pose"]["y"] - meta["origin"][1]) / meta["resolution"])
        )
        radius = math.ceil(0.15 / meta["resolution"])
        for x in range(col - radius, col + radius + 1):
            for y in range(row - radius, row + radius + 1):
                assert 0 <= x < image.width and 0 <= y < image.height
                gray = image.getpixel((x, y)) / 255
                occupancy = gray if meta["negate"] else 1 - gray
                assert occupancy < meta["free_thresh"]


@pytest.mark.parametrize(
    "change", ["extra_robot", "start", "via", "arrival", "switch", "scenario"]
)
def test_single_profile_rejects_wrong_scenario(single_profile, change):
    config = single_profile
    if change == "extra_robot":
        config["robots"]["smartcart"] = {"robot_id": "pinky3", "binding_id": "extra"}
    elif change == "scenario":
        config["scenario"] = "unknown"
    else:
        route = config["routes"][
            "cart_to_switch" if change == "switch" else "to_delivery"
        ]
        if change == "start":
            route["route"] = route["route"][1:]
        elif change == "via":
            route["route"] = ["N16", "N14", "N15", "N05", "N07", "N23", "N19", "N11"]
        else:
            route["route"] = route["route"][:-1]
        route["start_node"] = route["route"][0]
        route["goal"]["node_id"] = route["route"][-1]
    with pytest.raises(ValueError):
        validate_config(config)


def test_single_hardware_template_requires_field_checks():
    pinky = SERVICE.parent / "roport-pinky"
    config = prepare_module.prepare(
        pinky / "maps/airport_map.yaml",
        pinky / "src/roport_gz_world/layout/roport_airport.yaml",
        single_robot=True,
    )
    assert config["mode"] == "hardware" and config["field_verified"] is False
    assert config["routes"]["to_delivery"]["start_heading_rad"] == -1.5690
    assert (
        abs(config["routes"]["cart_to_switch"]["start_heading_rad"] - 0.001745329)
        < 1e-8
    )
    assert config["nav_goal"]["pose"] is not None
    with pytest.raises(ValueError):
        validate_config(config)


@pytest.mark.parametrize("change", ["node", "mode", "moving", "line_enabled"])
def test_single_dummy_requires_same_safe_n11_stop(single_profile, tmp_path, change):
    journal = Journal(tmp_path / "control.json")
    control = TrialControl(single_profile, MemoryTransport(), journal)
    try:
        status = FakePinky(single_profile, "logistics").observe()
        status.update(node_id="N11", mode="line", stopped=True, line_enabled=False)
        if change == "node":
            status["node_id"] = "N19"
        elif change == "mode":
            status["mode"] = "nav2"
        elif change == "moving":
            status["stopped"] = False
        else:
            status["line_enabled"] = True
        state = deepcopy(control.state)
        state.update(phase="WAIT_INBOUND", gate={"stop_session_id": "arrival"})
        state["statuses"]["pinky1"] = status
        journal.save(state)
        control.heard["pinky1"] = time.monotonic()
        with pytest.raises(ValueError):
            control.dummy("arrival")
        assert control.state == state and control.transport.sent == []
    finally:
        journal.close()


def test_fault_status_is_observed_and_first_hold_survives_shutdown(control_memory):
    control = control_memory
    control.poll(0)
    request = next(
        c for c in control.pending_status.values() if c["target"] == "pinky1"
    )
    event = status_reply(control, request)
    event["payload"]["fault"] = "local_watchdog"
    control.transport.incoming.put(delivery(event, "event"))
    control.poll(0)
    assert control.state["fault"] == "robot_fault"
    assert control.state["statuses"]["pinky1"]["fault"] == "local_watchdog"
    sent = len(control.transport.sent)
    control.hold("operator_stopped")
    control.transport.ready = False
    control.poll(0)
    assert control.state["fault"] == "robot_fault"
    assert len(control.transport.sent) == sent


def test_scope_is_persisted_and_cannot_change_on_resume(single_profile, tmp_path):
    journal = Journal(tmp_path / "scope.json")
    try:
        TrialControl(single_profile, MemoryTransport(), journal, scope="line-only")
        assert journal.data["scope"] == "line-only"
        with pytest.raises(ValueError, match="scope changed"):
            TrialControl(single_profile, MemoryTransport(), journal)
    finally:
        journal.close()


@pytest.mark.parametrize("problem", ["node", "stale", "fault"])
def test_smartcart_only_checks_one_robot_and_never_commands_logistics(
    profile, tmp_path, problem
):
    with closing(Journal(tmp_path / "cart.json")) as journal:
        transport = MemoryTransport()
        control = TrialControl(profile, transport, journal, smartcart_only=True)
        cart = profile["robots"]["smartcart"]
        status = {
            **FakePinky(profile, "smartcart").observe(),
            "binding_id": cart["binding_id"],
            "boot_id": "cart-boot",
        }
        journal.data["statuses"][cart["robot_id"]] = deepcopy(status)
        control.heard[cart["robot_id"]] = time.monotonic()
        control.poll(0)
        if problem == "node":
            journal.data["statuses"][cart["robot_id"]]["node_id"] = "N16"
        elif problem == "stale":
            control.heard[cart["robot_id"]] -= 4
        else:
            journal.data["statuses"][cart["robot_id"]]["fault"] = (
                "drive_guard_unavailable"
            )
        with pytest.raises(ValueError):
            control.start()
        journal.data["statuses"][cart["robot_id"]] = status
        control.heard[cart["robot_id"]] = time.monotonic()
        control.start()
        assert control.state["phase"] == "cart_to_delivery"
        assert control.state["transfers"] == []
        assert {cmd["target"] for _, cmd in transport.sent} == {cart["robot_id"]}
        with pytest.raises(ValueError, match="role selection changed"):
            TrialControl(profile, transport, journal)


def test_smartcart_only_requires_two_role_config(single_profile, tmp_path):
    with (
        closing(Journal(tmp_path / "cart.json")) as journal,
        pytest.raises(ValueError, match="two_pinky"),
    ):
        TrialControl(single_profile, MemoryTransport(), journal, smartcart_only=True)


def test_optional_navigation_diagnostics_are_strict_and_legacy_compatible(pair):
    client, _fake, _request = pair
    request = command(
        site=client.site,
        control=client.control,
        robot=client.robot,
        task="diagnostic-test",
        step="status",
        kind="status",
        payload={"binding_id": client.binding},
    )
    event = client._event(request, "completed", 1)
    validate_trial(event)
    event["payload"]["navigation"] = {
        "phase": "recovering",
        "goal_id": "G2",
        "target_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "current_pose": None,
        "recoveries": 1,
        "max_recoveries": 2,
        "recovery_elapsed_s": 3.0,
        "recovery_timeout_s": 30.0,
        "nav2_error_code": 0,
        "action_status": 0,
        "error_code": None,
    }
    validate_trial(event)
    event["payload"]["navigation"]["unexpected"] = True
    with pytest.raises(ValueError, match="unexpected"):
        validate_trial(event)
