"""Native wire admission around the real TrialDriver, without ROS or motor I/O."""

import math
import time
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from queue import Empty
from types import SimpleNamespace

import pytest
from test_virtual_pinky import simulation_profile
from virtual_pinky import VirtualPinky

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.field_pinky import (
    FieldPinkyClient,
    driver_command,
)
from roport_control_server.adapters.mqtt.mqtt import Delivery, MqttConfig


class RecordingTransport:
    ready = True

    def __init__(self):
        self.messages = []

    def publish(self, destination, message):
        wire.validate(message)
        self.messages.append((destination, message))

    def receive(self, timeout):
        raise Empty

    def stop(self):
        pass


@pytest.fixture
def endpoint(tmp_path):
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    client = FieldPinkyClient(
        MqttConfig("127.0.0.1", "wire-unit"),
        profile,
        "logistics",
        tmp_path / "pinky.json",
        driver,
    )
    client.transport = RecordingTransport()
    try:
        yield client, driver
    finally:
        client.close()
        driver.close()


def request(client, kind="line_route_command", payload=None, identifier="route-1"):
    if payload is None:
        route = deepcopy(client.profile["routes"]["to_delivery"])
        route["goal"] = {"node": route["goal"]["node_id"], "on_arrival": "hold"}
        payload = {"expected_boot_version": client.boot, **route}
    return wire.build_message(
        kind,
        client.control,
        client.robot,
        payload,
        command_id=identifier,
        expires_at=(datetime.now(UTC) + timedelta(seconds=30)).isoformat(),
    )


def deliver(client, message, *, retained=False):
    channel = "ack" if message["type"] == "event_stored" else "cmd"
    client.handle(
        Delivery(
            wire.topic(client.robot, channel), wire.encode(message), retained, 1, 1, 0
        )
    )


def test_actual_wire_is_minimal_and_boot_fenced(endpoint):
    client, driver = endpoint
    message = request(client)
    assert wire.decode(wire.encode(message)) == message
    assert wire.topic("pinky1", "cmd") == "roport/robot/pinky1/cmd"
    for extra in ("site_id", "execution_id", "binding_robot", "version"):
        invalid = {**message, extra: "old"}
        with pytest.raises(ValueError, match="envelope"):
            wire.validate(invalid)
    invalid = deepcopy(message)
    invalid["payload"]["expected_boot_version"] = "previous-process"
    deliver(client, invalid)
    assert (
        client.journal.data["records"]["route-1"]["event"]["payload"]["reason"]
        == "boot_mismatch"
    )
    assert driver.logic.active is None


def test_batched_progress_preserves_detected_node_in_mqtt_result(endpoint):
    client, driver = endpoint
    message = request(client)
    deliver(client, message)
    route = message["payload"]["route"]
    for _ in range(10000):
        driver.heartbeat()
        driver.tick()
        if driver.logic.seq >= 2:
            break
    assert driver.observe()["node_id"] == route[2]
    events = driver.drain_results()
    assert [
        (d["seq"], d["node_id"]) for _, phase, d in events if phase == "progress"
    ] == [
        (1, route[1]),
        (2, route[2]),
    ]
    for original, phase, details in events:
        # _report runs the production check_result before publishing each event.
        assert client.report(original, phase, **details)
    results = [
        msg["payload"]["result"]
        for _, msg in client.transport.messages
        if msg["type"] == "command_status" and msg["payload"]["status"] == "in_progress"
    ]
    assert [(r["seq"], r["node"]) for r in results] == [(1, route[1]), (2, route[2])]


def test_duplicate_never_dispatches_twice_and_changed_body_is_rejected(endpoint):
    client, driver = endpoint
    message = request(client)
    deliver(client, message)
    original = driver.logic.active
    deliver(client, message)
    assert driver.logic.active is original
    changed = deepcopy(message)
    changed["payload"]["speed_limit_mps"] = 0.04
    with pytest.raises(ValueError, match="idempotency_conflict"):
        deliver(client, changed)
    assert driver.logic.active is original
    with pytest.raises(ValueError, match="routing_or_delivery"):
        deliver(client, message, retained=True)


def test_command_expiring_during_accepted_publish_never_dispatches(
    endpoint, monkeypatch
):
    client, driver = endpoint
    now = datetime.now(UTC)
    message = request(client)
    message["expires_at"] = (now + timedelta(seconds=1)).isoformat()
    clock = [now]

    class Clock:
        @staticmethod
        def now(_tz):
            return clock[0]

    monkeypatch.setattr(
        "roport_control_server.adapters.mqtt.field_pinky.datetime", Clock
    )
    publish = client.transport.publish

    def delayed_publish(destination, event):
        publish(destination, event)
        if event["payload"].get("status") == "accepted":
            clock[0] += timedelta(seconds=2)

    monkeypatch.setattr(client.transport, "publish", delayed_publish)
    dispatched = []
    monkeypatch.setattr(driver, "dispatch", dispatched.append)
    deliver(client, message)
    assert dispatched == []
    assert client.halted == "expired_command"
    assert client.journal.data["active"] is None
    assert [event["payload"]["status"] for _, event in client.transport.messages] == [
        "accepted",
        "failed",
    ]
    assert client.journal.data["records"]["route-1"]["event"]["payload"]["reason"] == (
        "expired_command"
    )


def test_query_and_replayed_keep_do_not_renew_watchdog(endpoint):
    client, _ = endpoint
    deliver(client, request(client))
    client.last_keep = time.monotonic() - 2
    before = client.last_keep
    deliver(
        client,
        wire.build_message("robot_state_request", client.control, client.robot, {}),
    )
    assert client.last_keep == before
    keep = wire.build_message(
        "robot_state_request",
        client.control,
        client.robot,
        {"expected_boot_version": client.boot, "keep_running_command_id": "route-1"},
        expires_at=(datetime.now(UTC) + timedelta(seconds=3)).isoformat(),
    )
    deliver(client, keep)
    assert client.last_keep > before
    client.last_keep = before
    deliver(client, keep)
    assert client.last_keep == before
    client.last_keep = time.monotonic() - 4
    client.poll(0)
    assert client.halted == "control_link_lost"
    assert client.journal.data["active"] == "route-1"


@pytest.mark.parametrize("legacy_record", [False, True])
def test_endpoint_restart_requires_reconciliation(endpoint, tmp_path, legacy_record):
    client, _ = endpoint
    command = request(client)
    deliver(client, command)
    record = client.journal.data["records"][command["command_id"]]
    assert "legacy" not in record
    event = deepcopy(record["event"])
    if legacy_record:
        # Old journals included a redundant driver-format copy of each command.
        data = deepcopy(client.journal.data)
        data["records"][command["command_id"]]["legacy"] = driver_command(
            command, client.profile, client.binding
        )
        client.journal.save(data)
    old_boot = client.boot
    client.journal.close()
    new_driver = VirtualPinky(client.profile, "logistics", threaded=False)
    replacement = FieldPinkyClient(
        MqttConfig("127.0.0.1", "restart"),
        client.profile,
        "logistics",
        tmp_path / "pinky.json",
        new_driver,
    )
    replacement.transport = RecordingTransport()
    try:
        assert replacement.boot != old_boot
        assert replacement.halted == "recovery_required"
        assert replacement.journal.data["arrival"] is None
        deliver(replacement, command)
        assert replacement.transport.messages[-1][1] == event
        assert new_driver.logic.active is None
        assert replacement.halted == "recovery_required"
    finally:
        replacement.close()
        new_driver.close()


def test_real_arrival_is_required_before_departure(endpoint):
    client, driver = endpoint
    deliver(client, request(client))
    for _ in range(7000):
        driver.heartbeat()
        driver.tick()
        for original, phase, details in driver.drain_results():
            client.report(original, phase, **details)
        if client.journal.data["active"] is None:
            break
    assert client.journal.data["arrival"] == "route-1"
    route = deepcopy(client.profile["routes"]["cart_to_switch"])
    route["goal"] = {"node": route["goal"]["node_id"], "on_arrival": "hold"}
    payload = {"expected_boot_version": client.boot, **route}
    deliver(client, request(client, payload=payload, identifier="missing-departure"))
    assert (
        client.journal.data["records"]["missing-departure"]["event"]["payload"][
            "reason"
        ]
        == "current_arrival_required"
    )
    payload["departure"] = {
        "arrival_command_id": "route-1",
        "transfer_command_id": "confirmed-transfer",
    }
    deliver(client, request(client, payload=payload, identifier="approved-departure"))
    assert client.journal.data["active"] == "approved-departure"
    assert client.journal.data["arrival"] is None
    assert driver.last["cmd"] == "load_done"


def test_n12_reverse_requires_pose_ir_and_continuous_stopped_samples(endpoint):
    client, driver = endpoint
    driver.mode = "nav2"
    driver.sim.place("N12", "W")
    driver.nav.pose = driver.map_pose()
    driver.nav.observed_at = time.time()
    for _ in range(3):
        driver._feed()
    mode = request(
        client,
        "set_drive_mode_command",
        {
            "expected_boot_version": client.boot,
            "node": "N12",
            "from_mode": "nav2",
            "to_mode": "line",
            "graph_version": client.profile["graph_version"],
            "map_version": client.profile["map_version"],
        },
        "return-mode",
    )
    driver.sim.rects = []
    for _ in range(3):
        driver._feed()
    assert not driver._return_aligned()
    from sim import tape_rects

    driver.sim.rects = tape_rects(driver.graph)
    for _ in range(3):
        driver._feed()
    assert driver._return_aligned()
    deliver(client, mode)
    assert client.journal.data["active"] == "return-mode"
    assert driver.drain_results() == []
    deadline = time.monotonic() + 0.7
    while time.monotonic() < deadline:
        driver.heartbeat()
        driver.tick()
        for original, phase, details in driver.drain_results():
            client.report(original, phase, **details)
        time.sleep(0.02)
    assert client.journal.data["active"] is None
    assert driver.observe()["mode"] == "line"
    assert driver.observe()["node_id"] == "N12"
    assert driver.logic.confirmed_return
    assert -math.pi <= driver.observe()["heading_rad"] <= math.pi


def test_docking_selector_is_preserved_at_driver_boundary(endpoint):
    client, _ = endpoint
    nav = deepcopy(client.profile["nav_goal"])
    nav.pop("map_id")
    nav.pop("goal_version")
    nav.update(expected_boot_version=client.boot, goal_id="N12_RETURN")
    message = request(client, "nav_goal_command", nav, "return-navigation")
    assert (
        driver_command(message, client.profile, client.binding)["payload"]["goal_id"]
        == "N12_RETURN"
    )


def test_v2_navigation_retry_waits_for_terminal_result_and_preserves_failure(
    endpoint, monkeypatch
):
    client, driver = endpoint
    # No sensor thread runs in this fixture; disk fsync time must not age its samples.
    wall, monotonic = time.time(), time.monotonic()
    clock = SimpleNamespace(time=lambda: wall, monotonic=lambda: monotonic)
    monkeypatch.setattr("virtual_pinky.time", clock)
    monkeypatch.setattr("roport_pinky.trial_driver.time", clock)
    monkeypatch.setattr(driver.logic, "now", clock.monotonic)
    driver.mode = "nav2"
    nav = deepcopy(client.profile["nav_goal"])
    nav.pop("map_id")
    nav.pop("goal_version")
    nav.update(expected_boot_version=client.boot, goal_id="G2")
    original = request(client, "nav_goal_command", nav, "failed-navigation")
    deliver(client, original)
    assert client.journal.data["active"] == original["command_id"]
    driver.nav.result = ("nav2_goal_failed", time.monotonic())
    for command, phase, details in driver.drain_results():
        client.report(command, phase, **details)
    assert client.journal.data["active"] is None
    failed = deepcopy(driver.failed_navigation)
    driver.nav.quiescent = lambda: False
    pending = request(client, "nav_goal_command", nav, "pending-retry")
    deliver(client, pending)
    rejected = client.journal.data["records"][pending["command_id"]]["event"]
    assert rejected["payload"]["status"] == "rejected"
    assert rejected["payload"]["reason"] == "recovery_required"
    assert driver.failed_navigation == failed
    assert driver.nav_command is None
    driver.nav.quiescent = lambda: True
    other = request(client, "nav_goal_command", {**nav, "goal_id": "G3"}, "other-goal")
    deliver(client, other)
    rejected_other = client.journal.data["records"][other["command_id"]]["event"]
    assert rejected_other["payload"]["reason"] == "recovery_required"
    assert driver.failed_navigation == failed
    terminal = request(client, "nav_goal_command", nav, "terminal-retry")
    deliver(client, terminal)
    assert client.journal.data["active"] == terminal["command_id"]
    assert driver.halted is None
    assert driver.nav_command["payload"] == failed["payload"]


@pytest.mark.parametrize(
    ("operation", "error"),
    [
        ("receive", ConnectionError),
        ("ack", ConnectionError),
        ("accepted", ConnectionError),
        ("accepted", TimeoutError),
        ("report", ConnectionError),
        ("motion", ConnectionError),
    ],
)
def test_transport_failure_holds_without_exiting_endpoint(
    endpoint, monkeypatch, operation, error
):
    client, driver = endpoint
    command = request(client)

    def disconnected(*_args, **_kwargs):
        raise error("simulated MQTT disconnect")

    if operation in {"motion", "report"}:
        deliver(client, command)
        if operation == "motion":
            monkeypatch.setattr(
                client.transport, "publish_motion", disconnected, raising=False
            )
            client.publish_motion()
        else:
            monkeypatch.setattr(client.transport, "publish", disconnected)
            route = command["payload"]["route"]
            legacy = driver_command(command, client.profile, client.binding)
            assert not client.report(
                legacy,
                "progress",
                seq=0,
                total=len(route),
                node_id=route[0],
                prev_node=None,
                next_node=route[1],
            )
            event = client.journal.data["records"]["route-1"]["event"]
            assert event["payload"]["status"] == "in_progress"
            assert event["message_id"] in client.journal.data["outbox"]
    else:
        deliveries = iter(
            [
                Delivery(
                    wire.topic(client.robot, "cmd"),
                    wire.encode(command),
                    False,
                    1,
                    1,
                    0,
                )
            ]
        )
        monkeypatch.setattr(
            client.transport, "receive", lambda _timeout: next(deliveries)
        )
        monkeypatch.setattr(
            client.transport, "ack", lambda _delivery: None, raising=False
        )
        method = {"receive": "receive", "ack": "ack", "accepted": "publish"}[operation]
        monkeypatch.setattr(client.transport, method, disconnected)
        client.poll(0)
    assert client.halted == "control_link_lost"
    assert driver.logic.active is None
    driver.tick()
    assert driver.observe()["stopped"]
    # Current fault remains visible after the adapter is connected again.
    assert client.state_message()["payload"]["state"]["fault"] == "control_link_lost"


def test_journal_failure_still_propagates_and_close_always_releases_lock(
    endpoint, monkeypatch
):
    client, _ = endpoint

    def disk_failure(_data):
        raise OSError("simulated journal write failure")

    with monkeypatch.context() as patch:
        patch.setattr(client.journal, "save", disk_failure)
        with pytest.raises(OSError, match="journal write"):
            deliver(client, request(client))

    def transport_failure():
        raise ConnectionError("simulated transport close failure")

    with monkeypatch.context() as patch:
        patch.setattr(client.transport, "stop", transport_failure)
        with pytest.raises(ConnectionError, match="transport close"):
            client.close()
    assert client.journal._lock.closed
