"""Fault injection only: actual native endpoint/driver or isolated loopback MQTT."""

import json
import time
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from queue import Empty
from uuid import uuid4

import pytest
from mqtt_helpers import wait_until
from test_field_pinky import RecordingTransport, deliver, endpoint, request  # noqa: F401
from test_fleet import dispatch, fleet, profile, submit  # noqa: F401
from test_fleet_mqtt import fleet_mqtt  # noqa: F401
from virtual_pinky import VirtualPinky

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.field_pinky import FieldPinkyClient
from roport_control_server.adapters.mqtt.mqtt import Delivery, MqttConfig, MqttTransport
from roport_control_server.executor.fleet_runtime import FleetRuntime


def stamp(offset=0):
    return (datetime.now(UTC) + timedelta(seconds=offset)).isoformat()


def assert_rejected(client, driver, message, reason):
    deliver(client, message)
    event = client.journal.data["records"][message["command_id"]]["event"]
    assert event["payload"]["status"] == "rejected", event
    assert reason in event["payload"]["reason"], event
    assert client.journal.data["active"] is None
    assert driver.logic.active is None
    assert driver.nav_command is None


@pytest.mark.parametrize(
    ("fault", "reason"),
    [
        ("boot", "boot_mismatch"),
        ("expired", "expired_command"),
        ("stale_sent", "stale_message"),
        ("future_sent", "stale_message"),
        ("moving", "fresh_stop_required"),
        ("wrong_node", "route_start_mismatch"),
        ("wrong_graph", "route_start_mismatch"),
        ("wrong_mode", "not_in_line_mode"),
        ("stale_observation", "fresh_stop_required"),
    ],
)
def test_route_admission_faults(
    endpoint,  # noqa: F811 - shared pytest fixture
    monkeypatch,
    fault,
    reason,
):
    client, driver = endpoint
    message = request(client)
    if fault == "boot":
        message["payload"]["expected_boot_version"] = "previous-boot"
    elif fault == "expired":
        message.update(sent_at=stamp(-2), expires_at=stamp(-1))
    elif fault in {"stale_sent", "future_sent"}:
        message["sent_at"] = stamp(-5 if fault == "stale_sent" else 5)
    elif fault == "wrong_graph":
        message["payload"]["graph_version"] = "old-graph"
    elif fault == "wrong_mode":
        driver.mode = "nav2"
    else:
        state = driver.observe()
        if fault == "moving":
            state["stopped"] = False
        elif fault == "wrong_node":
            state["node_id"] = "N17"
        else:
            state["observed_at"] = stamp(-5)
        monkeypatch.setattr(driver, "observe", lambda: deepcopy(state))
    assert_rejected(client, driver, message, reason)


def mode_request(client, driver):
    # Valid stopped N12/handoff baseline; no ROS discovery or physical movement.
    driver.logic.node = "N12"
    driver.logic.st["state"] = "handoff"
    return request(
        client,
        "set_drive_mode_command",
        {
            "expected_boot_version": client.boot,
            "node": "N12",
            "from_mode": "line",
            "to_mode": "nav2",
            "graph_version": client.profile["graph_version"],
            "map_version": client.profile["map_version"],
        },
        "mode-1",
    )


@pytest.mark.parametrize(
    ("fault", "reason"),
    [
        ("no_nav2", "nav2_not_ready"),
        ("stale_localization", "nav2_not_ready"),
        ("wrong_node", "not_at_switch_node"),
        ("wrong_graph", "graph"),
    ],
)
def test_mode_switch_requires_matching_physical_readiness(
    endpoint,  # noqa: F811 - shared pytest fixture
    fault,
    reason,
):
    client, driver = endpoint
    message = mode_request(client, driver)
    if fault == "no_nav2":
        driver.nav.ready = False
    elif fault == "stale_localization":
        driver.nav.observed_at = time.time() - 1
    elif fault == "wrong_node":
        driver.logic.node = "N11"
    else:
        message["payload"]["graph_version"] = "old-graph"
    assert_rejected(client, driver, message, reason)


def test_valid_n12_baseline_is_accepted(
    endpoint,  # noqa: F811 - shared pytest fixture
):
    client, driver = endpoint
    message = mode_request(client, driver)
    deliver(client, message)
    assert client.journal.data["active"] == message["command_id"]
    assert driver.mode == "nav2"


@pytest.mark.parametrize("fault", ["wrong_boot", "wrong_command", "expired", "replay"])
def test_invalid_keepalive_cannot_postpone_stop(
    endpoint,  # noqa: F811 - shared pytest fixture
    fault,
):
    client, driver = endpoint
    deliver(client, request(client))
    keep = wire.build_message(
        "robot_state_request",
        client.control,
        client.robot,
        {"expected_boot_version": client.boot, "keep_running_command_id": "route-1"},
        expires_at=stamp(3),
    )
    if fault == "replay":
        deliver(client, keep)
    elif fault == "wrong_boot":
        keep["payload"]["expected_boot_version"] = "previous-boot"
    elif fault == "wrong_command":
        keep["payload"]["keep_running_command_id"] = "previous-command"
    else:
        keep.update(sent_at=stamp(-2), expires_at=stamp(-1))
    before = client.last_keep = time.monotonic() - 4
    if fault == "replay":
        deliver(client, keep)
    else:
        with pytest.raises(ValueError, match="keep_running_mismatch|expired_command"):
            deliver(client, keep)
    assert client.last_keep == before
    client.poll(0)
    assert client.halted == "control_link_lost"
    assert driver.logic.active is None
    assert client.journal.data["active"] == "route-1"  # unresolved, not completed


@pytest.mark.parametrize("fault", ["retained", "qos0"])
def test_unsafe_delivery_flags_do_not_start_motion(
    endpoint,  # noqa: F811 - shared pytest fixture
    fault,
):
    client, driver = endpoint
    message = request(client)
    with pytest.raises(ValueError, match="routing_or_delivery"):
        client.handle(
            Delivery(
                wire.topic(client.robot, "cmd"),
                wire.encode(message),
                fault == "retained",
                1,
                0 if fault == "qos0" else 1,
                0,
            )
        )
    assert client.journal.data["active"] is None
    assert driver.logic.active is None


def event_delivery(message):
    return Delivery(
        wire.topic(message["source"], "event"), wire.encode(message), False, 1, 1, 0
    )


@pytest.mark.parametrize("fault", ["old_sequence", "stale_observation", "old_boot"])
def test_control_state_fences(
    fleet,  # noqa: F811 - shared pytest fixture
    fault,
):
    control = fleet
    key = submit(control, "state-fault")
    command = dispatch(control)
    robot = command["target"]
    old = deepcopy(control.observed[robot])
    payload = {**deepcopy(old), "sequence": old["sequence"] + 1, "observed_at": stamp()}
    if fault == "old_sequence":
        payload["sequence"] = old["sequence"] - 1
        payload["state"]["node"] = "N11"
    elif fault == "stale_observation":
        payload["observed_at"] = stamp(-5)
        payload["state"]["node"] = "N11"
    else:
        payload["boot_version"] = "previous-boot"
    message = wire.build_message(
        "robot_state", robot, control.config["control_id"], payload
    )
    FleetRuntime(control, RecordingTransport()).handle(event_delivery(message))
    if fault == "old_boot":
        assert control.data["tasks"][key]["status"] == "HOLD"
        assert control.next_action() is None
    else:
        assert control.observed[robot] == old
        assert next(iter(control.data["commands"].values()))["message"] == command
        assert control.data["tasks"][key]["stage"] == "warehouse_route"


@pytest.mark.parametrize(
    "fault", ["old_command", "wrong_boot", "reverse_sequence", "late_after_timeout"]
)
def test_completion_cannot_release_a_different_or_held_operation(
    fleet,  # noqa: F811 - shared pytest fixture
    fault,
):
    control = fleet
    key = submit(control, "completion-fault")
    command = dispatch(control)
    payload = {
        "boot_version": command["payload"]["expected_boot_version"],
        "observed_at": stamp(),
        "sequence": 2,
        "status": "completed",
        "result": {
            "node": command["payload"]["goal"]["node"],
            "stopped": True,
            "mode": "line",
        },
    }
    message = wire.build_message(
        "command_status",
        command["target"],
        command["source"],
        payload,
        command_id=command["command_id"],
    )
    if fault == "old_command":
        message["command_id"] = "previous-command"
    elif fault == "wrong_boot":
        message["payload"]["boot_version"] = "previous-boot"
    elif fault == "reverse_sequence":
        progress = deepcopy(message)
        progress.update(message_id=str(uuid4()))
        progress["payload"].update(status="in_progress", sequence=3)
        assert control.result(progress)
    else:
        next(iter(control.data["commands"].values()))["sent_at"] = -1e9
        control.check_active()
        assert control.data["tasks"][key]["status"] == "HOLD"
    runtime = FleetRuntime(control, RecordingTransport())
    if fault == "wrong_boot":
        with pytest.raises(ValueError, match="source/boot mismatch"):
            runtime.handle(event_delivery(message))
    else:
        runtime.handle(event_delivery(message))
    assert next(iter(control.data["commands"].values()))["message"] == command
    assert control.data["tasks"][key]["stage"] == "warehouse_route"
    assert all(slot["state"] == "empty" for slot in control.data["slots"].values())


def disconnect_active_trial(env, broker):
    key = env.submit("broker-fault", "inbound")
    env.pump(lambda: any(c.journal.data["active"] for c in env.clients))
    active = next(c for c in env.clients if c.journal.data["active"])
    original = deepcopy(next(iter(env.control.data["commands"].values()))["message"])
    runtime = FleetRuntime(env.control, env.transport)
    broker[1].terminate()
    broker[1].wait(timeout=5)
    wait_until(
        lambda: (
            not env.transport.ready and all(not c.transport.ready for c in env.clients)
        )
    )
    runtime.poll(0)
    errors = []
    for client in env.clients:
        try:
            client.poll(0)
        except ConnectionError as exc:
            errors.append(exc)
        client.driver.tick()
    assert env.control.data["tasks"][key]["status"] == "HOLD"
    assert active.halted == "control_link_lost"
    assert active.driver.logic.active is None
    assert active.driver.observe()["stopped"]
    (env.tmp_path / "disconnect-evidence.json").write_text(
        json.dumps(
            {
                "robot": active.robot,
                "control_fault": env.control.data["fault"],
                "task_status": env.control.data["tasks"][key]["status"],
                "endpoint_halt": active.halted,
                "driver_active": active.driver.logic.active,
                "driver_stopped": active.driver.observe()["stopped"],
                "exceptions": [str(exc) for exc in errors],
            },
            indent=2,
        )
        + "\n"
    )
    return active, original, errors


def test_broker_disconnect_and_restart_do_not_resume_active_motion(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    broker,
):
    env = fleet_mqtt
    active, original, errors = disconnect_active_trial(env, broker)
    if errors:
        raise errors[
            0
        ]  # Preserve the unexpected propagation, after recording safe stop.
    broker[2]()
    wait_until(
        lambda: env.transport.ready and all(c.transport.ready for c in env.clients),
        timeout=10,
    )
    observer = MqttTransport(
        MqttConfig("127.0.0.1", "reconnected-state-observer", port=env.port),
        [wire.topic(active.robot, "event")],
    )
    observer.start()
    env.stack.callback(observer.stop)
    # Replay the exact prior command after subscriptions return.
    env.transport.publish(wire.topic(active.robot, "cmd"), original)
    env.transport.publish(
        wire.topic(active.robot, "cmd"),
        wire.build_message("robot_state_request", active.control, active.robot, {}),
    )
    state = None
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        active.poll(0)
        active.publish_motion()
        try:
            delivery = observer.receive(0.01)
        except Empty:
            continue
        message = wire.decode(delivery.payload)
        observer.ack(delivery)
        if message["type"] == "robot_state":
            state = message
            break
        time.sleep(0.01)
    assert state is not None
    assert state["payload"]["boot_version"] == active.boot
    assert state["payload"]["state"]["fault"] == "control_link_lost"
    assert active.driver.logic.active is None
    assert active.halted == "control_link_lost"
    assert env.control.next_action() is None
    assert not env.executors["omx3"].tasks


def test_broker_restart_and_fresh_processes_preserve_recovery_hold(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
    broker,
):
    env = fleet_mqtt
    active, original, _ = disconnect_active_trial(env, broker)
    active_robot = active.robot
    for client in env.clients:
        client.close()  # Explicit endpoint restart still preserves unresolved work.
    broker[2]()
    replacements = []
    for robot, (role, config) in env.robots.items():
        driver = VirtualPinky(config, role, threaded=False)
        env.stack.callback(driver.close)
        client = FieldPinkyClient(
            MqttConfig("127.0.0.1", "broker-restart-" + robot, port=env.port),
            config,
            role,
            env.tmp_path / (robot + ".json"),
            driver,
        )
        client.start()
        env.stack.callback(client.close)
        replacements.append(client)
    env.clients[:] = replacements
    wait_until(lambda: env.transport.ready, timeout=10)
    target = next(c for c in replacements if c.robot == active_robot)
    env.transport.publish(wire.topic(active_robot, "cmd"), original)
    runtime = FleetRuntime(env.control, env.transport)
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        for client in replacements:
            client.poll(0)
            client.driver.tick()
        runtime.poll(0)
        time.sleep(0.01)
    assert target.halted == "recovery_required"
    assert target.driver.logic.active is None
    assert target.driver.observe()["stopped"]
    assert env.control.data["fault"]
    assert env.control.next_action() is None
    assert not env.executors["omx3"].tasks


def test_active_robot_restart_rejects_old_command_without_second_dispatch(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
):
    env = fleet_mqtt
    key = env.submit("robot-restart", "inbound")
    env.pump(lambda: any(c.journal.data["active"] for c in env.clients))
    index = next(i for i, c in enumerate(env.clients) if c.journal.data["active"])
    old = env.clients[index]
    command = deepcopy(next(iter(env.control.data["commands"].values()))["message"])
    old.close()
    role, config = env.robots[old.robot]
    replacement = FieldPinkyClient(
        MqttConfig("127.0.0.1", "fault-restart-" + old.robot, port=env.port),
        config,
        role,
        env.tmp_path / (old.robot + ".json"),
        old.driver,
    )
    replacement.start()
    env.stack.callback(replacement.close)
    env.clients[index] = replacement
    env.transport.publish(wire.topic(old.robot, "cmd"), command)
    env.pump(lambda: bool(env.control.data["fault"]))
    assert replacement.boot != old.boot
    assert replacement.halted == "recovery_required"
    assert replacement.driver.logic.active is None
    assert env.control.data["tasks"][key]["status"] == "HOLD"
    assert not env.executors["omx3"].tasks
