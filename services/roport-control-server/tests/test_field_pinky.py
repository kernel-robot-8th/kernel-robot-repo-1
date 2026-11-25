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


@pytest.mark.parametrize("recovery", ["settled", "timeout", "invalid"])
def test_route_completion_rechecks_stop_without_exiting_endpoint(
    endpoint, monkeypatch, recovery
):
    client, driver = endpoint
    command = request(client)
    deliver(client, command)
    route = command["payload"]["route"]
    legacy = driver_command(command, client.profile, client.binding)
    observation = {
        **driver.observe(),
        "node_id": route[-1],
        "stopped": recovery == "invalid",
        "line_enabled": False,
        "drive_owner": "none",
    }
    monkeypatch.setattr(driver, "observe", lambda: observation.copy())
    details = {
        "node_id": route[-1],
        "seq": len(route) - 1,
        "total": len(route),
        "prev_node": route[-2],
        "next_node": None,
    }
    if recovery == "invalid":
        details["prev_node"] = "wrong-node"
        assert client.report(legacy, "completed", **details)
    else:
        assert not client.report(legacy, "completed", **details)
        assert client.journal.data["active"] == "route-1"
        assert client.journal.data["arrival"] is None
        if recovery == "settled":
            observation["stopped"] = True
        else:
            legacy_pending, details_pending, _ = client.pending_route_completion
            client.pending_route_completion = (legacy_pending, details_pending, 0)
        client.poll(0)
    result = client.journal.data["records"]["route-1"]["event"]["payload"]
    assert client.journal.data["active"] is None
    assert client.pending_route_completion is None
    if recovery == "settled":
        assert result["status"] == "completed"
        assert result["result"]["stopped"] is True
        assert client.journal.data["arrival"] == "route-1"
    else:
        assert result["status"] == "failed"
        assert result["reason"] == (
            "route_stop_unconfirmed"
            if recovery == "timeout"
            else "invalid_driver_result"
        )
        assert client.journal.data["arrival"] is None
        assert driver.logic.active is None
    assert client.transport.ready
    client.poll(0)


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


def test_host_metrics_off_unless_period_given(endpoint):
    client, _ = endpoint
    client.publish_host_metrics()  # RecordingTransport has no sample publisher
    assert client.transport.messages == []


@pytest.mark.parametrize("period", [0, -1, float("nan"), float("inf")])
def test_host_metrics_period_must_be_positive(tmp_path, period):
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    try:
        with pytest.raises(ValueError):
            FieldPinkyClient(
                MqttConfig("127.0.0.1", "wire-unit"),
                profile,
                "logistics",
                tmp_path / "pinky.json",
                driver,
                host_metrics_period_s=period,
            )
    finally:
        driver.close()


def test_host_metrics_uses_configured_temp_path(tmp_path):
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    temp = tmp_path / "temp1_input"
    temp.write_text("48500\n")
    client = FieldPinkyClient(
        MqttConfig("127.0.0.1", "wire-unit"),
        profile,
        "logistics",
        tmp_path / "pinky.json",
        driver,
        host_metrics_period_s=5,
        host_temp_path=str(temp),
    )
    samples = []

    class Transport(RecordingTransport):
        def publish_host_metrics(self, destination, message, expiry_s):
            samples.append(message)

    client.transport = Transport()
    try:
        client.publish_host_metrics()
        assert samples[0]["payload"]["cpu_temp_c"] == 48.5
    finally:
        driver.close()


def test_host_metrics_unexpected_error_never_stops_motion(tmp_path, caplog):
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    client = FieldPinkyClient(
        MqttConfig("127.0.0.1", "wire-unit"),
        profile,
        "logistics",
        tmp_path / "pinky.json",
        driver,
        host_metrics_period_s=5,
    )

    class Transport(RecordingTransport):
        def publish_host_metrics(self, destination, message, expiry_s):
            raise RuntimeError("telemetry bug")

    client.transport = Transport()
    stops = []
    driver.safe_stop = stops.append
    try:
        client.publish_host_metrics()
        assert "host metrics not sent" in caplog.text
        assert client.halted is None and stops == []
    finally:
        driver.close()


def test_host_metrics_rate_and_failure_never_stop_motion(tmp_path, caplog):
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    client = FieldPinkyClient(
        MqttConfig("127.0.0.1", "wire-unit"),
        profile,
        "logistics",
        tmp_path / "pinky.json",
        driver,
        host_metrics_period_s=5,
    )
    samples = []

    class Transport(RecordingTransport):
        fail = False

        def publish_host_metrics(self, destination, message, expiry_s):
            if self.fail:
                raise ConnectionError("broker gone")
            wire.validate(message)
            samples.append((destination, message, expiry_s))

    client.transport = Transport()
    stops = []
    driver.safe_stop = stops.append
    try:
        client.publish_host_metrics()
        client.publish_host_metrics()  # within the period: no second sample
        assert len(samples) == 1
        destination, message, expiry = samples[0]
        assert destination == wire.topic(client.robot, "host") and expiry == 5
        assert message["payload"]["boot_version"] == client.boot
        assert message["payload"]["sequence"] == 1
        client.transport.fail = True
        client.last_host_metrics = -math.inf
        client.publish_host_metrics()
        assert "host metrics not sent" in caplog.text
        assert client.halted is None and stops == []
        client.transport.fail = False
        client.last_host_metrics = -math.inf
        client.publish_host_metrics()
        assert samples[-1][1]["payload"]["sequence"] == 3
    finally:
        client.close()
        driver.close()


@pytest.fixture
def follow_endpoint(endpoint, monkeypatch):
    client, driver = endpoint
    state = driver.observe()
    state.pop("observed_at", None)
    state.update(
        mode="nav2",
        stopped=True,
        line_enabled=False,
        nav2_ready=True,
        localization_ready=True,
        drive_owner="none",
        fault=None,
    )
    dispatched, stopped = [], []
    monkeypatch.setattr(
        driver,
        "observe",
        lambda: {"observed_at": datetime.now(UTC).isoformat(), **state},
    )
    monkeypatch.setattr(driver, "validate_action", lambda command: None)
    monkeypatch.setattr(driver, "dispatch", dispatched.append)

    def safe_stop(reason):
        stopped.append(reason)
        state.update(stopped=True, fault=reason, drive_owner="none")

    monkeypatch.setattr(driver, "safe_stop", safe_stop)
    return client, state, dispatched, stopped


def follow_request(client, kind="follow_registration_command", **payload):
    return request(
        client,
        kind,
        {
            "expected_boot_version": client.boot,
            "map_version": client.profile["map_version"],
            "follow_session_id": "follow-session-1",
            **payload,
        },
        identifier="follow-1",
    )


def registration_evidence(**changed):
    return {
        "follow_session_id": "follow-session-1",
        "frame_ref": "frame-1",
        "image_ref": "image-1",
        "perception_boot_id": "perception-1",
        "expires_at": (datetime.now(UTC) + timedelta(seconds=60)).isoformat(),
        "candidates": [{"candidate_id": "candidate-1", "box": [0.1, 0.2, 0.4, 0.8]}],
        **changed,
    }


def last_status(client):
    return client.journal.data["records"]["follow-1"]["event"]["payload"]


def test_registration_waits_for_async_driver_result_and_replays_once(follow_endpoint):
    client, _, dispatched, stopped = follow_endpoint
    data = deepcopy(client.journal.data)
    data["arrival"] = "last-physical-arrival"
    client.journal.save(data)
    message = follow_request(client, operation="capture")
    deliver(client, message)
    deliver(client, message)
    assert len(dispatched) == 1
    assert dispatched[0]["type"] == "follow_registration.requested"
    assert last_status(client)["status"] == "accepted"
    assert client.journal.data["active"] == "follow-1"
    assert "follow_active" not in last_status(client)["result"]
    assert client.report(dispatched[0], "completed", **registration_evidence())
    assert last_status(client)["status"] == "completed"
    assert client.journal.data["arrival"] == "last-physical-arrival"
    assert last_status(client)["result"]["stopped"] is True
    assert client.journal.data["active"] is None
    deliver(client, message)
    assert len(dispatched) == 1
    assert last_status(client)["status"] == "completed"
    assert stopped == []


@pytest.mark.parametrize(
    "changed,reason",
    [
        ({"stopped": False}, "fresh_stop_required"),
        ({"line_enabled": True}, "fresh_stop_required"),
        ({"observed_at": "2020-01-01T00:00:00+00:00"}, "fresh_stop_required"),
        ({"mode": "line"}, "registration_not_ready"),
        ({"nav2_ready": False}, "registration_not_ready"),
        ({"localization_ready": False}, "registration_not_ready"),
        ({"drive_owner": "line"}, "registration_not_ready"),
    ],
)
def test_registration_requires_fresh_stopped_nav2(follow_endpoint, changed, reason):
    client, state, dispatched, _ = follow_endpoint
    state.update(changed)
    deliver(client, follow_request(client, operation="capture"))
    assert last_status(client)["status"] == "rejected"
    assert last_status(client)["reason"] == reason
    assert dispatched == []


def test_registration_unsupported_driver_is_explicitly_rejected(
    follow_endpoint, monkeypatch
):
    client, _, dispatched, _ = follow_endpoint

    def unsupported(command):
        raise ValueError("unsupported_operation")

    monkeypatch.setattr(client.driver, "validate_action", unsupported)
    deliver(client, follow_request(client, operation="capture"))
    assert last_status(client)["reason"] == "unsupported_operation"
    assert last_status(client)["status"] == "rejected"
    assert dispatched == []
    assert client.journal.data["active"] is None


@pytest.mark.parametrize(
    "details",
    [
        {"follow_session_id": "old-session"},
        {"expires_at": "2020-01-01T00:00:00+00:00"},
        {"candidates": [{"candidate_id": "same", "box": [0, 0, 1, 1]}] * 2},
        {"stopped": False},
    ],
)
def test_bad_registration_result_fails_closed_without_ending_runner(
    follow_endpoint, details
):
    client, _, dispatched, stopped = follow_endpoint
    deliver(client, follow_request(client, operation="capture"))
    assert not client.report(
        dispatched[0], "completed", **registration_evidence(**details)
    )
    assert last_status(client)["status"] == "failed"
    assert last_status(client)["reason"] == "invalid_registration_result"
    assert stopped == ["invalid_registration_result"]
    assert client.journal.data["active"] is None
    client.poll(0)


def test_registration_result_cannot_claim_stop_while_observation_moves(follow_endpoint):
    client, state, dispatched, stopped = follow_endpoint
    deliver(client, follow_request(client, operation="capture"))
    state["stopped"] = False
    assert not client.report(
        dispatched[0], "completed", stopped=True, **registration_evidence()
    )
    assert stopped == ["invalid_registration_result"]


def test_registration_dispatch_exception_is_reported_without_ending_runner(
    follow_endpoint, monkeypatch
):
    client, _, _, stopped = follow_endpoint

    def unavailable(command):
        raise RuntimeError("camera unavailable")

    monkeypatch.setattr(client.driver, "dispatch", unavailable)
    deliver(client, follow_request(client, operation="capture"))
    assert last_status(client)["status"] == "failed"
    assert last_status(client)["reason"] == "registration_failed"
    assert client.journal.data["active"] is None
    assert stopped == ["registration_failed"]


def appearance_request(client):
    return follow_request(
        client,
        "follow_target_command",
        follow_mode="appearance",
        target_ref="target-1",
        perception_boot_id="perception-1",
    )


def active_follow():
    return {
        "active": True,
        "follow_mode": "appearance",
        "follow_session_id": "follow-session-1",
        "target_ref": "target-1",
        "perception_boot_id": "perception-1",
    }


def test_follow_active_requires_observed_target_and_is_not_forged_by_progress(
    follow_endpoint,
):
    client, state, dispatched, _ = follow_endpoint
    message = appearance_request(client)
    deliver(client, message)
    assert dispatched[0]["type"] == "follow.requested"
    assert "card_id" not in dispatched[0]["payload"]
    assert last_status(client)["status"] == "accepted"
    client.poll(0)
    assert last_status(client)["status"] == "accepted"
    assert client.report(
        dispatched[0], "progress", follow_active=True, follow=active_follow()
    )
    assert last_status(client)["result"]["follow_active"] is False
    assert "follow" not in last_status(client)["result"]
    state.update(follow=active_follow(), drive_owner="nav2", stopped=False)
    client.poll(0)
    assert last_status(client)["status"] == "in_progress"
    assert last_status(client)["result"]["follow_active"] is True
    assert last_status(client)["result"]["follow"] == active_follow()
    sequence = last_status(client)["sequence"]
    client.poll(0)
    deliver(client, message)
    assert last_status(client)["sequence"] == sequence
    assert len(dispatched) == 1


@pytest.mark.parametrize(
    "changed",
    [
        {"follow_mode": "marker"},
        {"follow_session_id": "another-session"},
        {"perception_boot_id": "restarted"},
        {"target_ref": "another-person"},
        {"active": 1},
    ],
)
@pytest.mark.parametrize("via_progress", [False, True])
def test_invalid_follow_observation_stops_instead_of_confirming(
    follow_endpoint, changed, via_progress
):
    client, state, dispatched, stopped = follow_endpoint
    deliver(client, appearance_request(client))
    state.update(
        follow={**active_follow(), **changed}, drive_owner="nav2", stopped=False
    )
    if via_progress:
        assert not client.report(dispatched[0], "progress")
    else:
        client.poll(0)
    assert last_status(client)["status"] == "failed"
    assert last_status(client)["reason"] == "invalid_follow_observation"
    assert stopped == ["invalid_follow_observation"]
    assert client.journal.data["active"] is None


def test_legacy_marker_follow_needs_actual_active_evidence_too(follow_endpoint):
    client, state, _, _ = follow_endpoint
    message = follow_request(client, "follow_target_command", card_id=35)
    deliver(client, message)
    client.poll(0)
    assert last_status(client)["status"] == "accepted"
    state.update(
        follow={
            "active": True,
            "follow_mode": "marker",
            "follow_session_id": "follow-session-1",
        },
        drive_owner="nav2",
        stopped=False,
    )
    client.poll(0)
    assert last_status(client)["result"]["follow_active"] is True


def stop_request(client, target="route-1", identifier="stop-1"):
    return request(
        client,
        "robot_stop_command",
        {
            "expected_boot_version": client.boot,
            "target_command_id": target,
            "reason": "operator_stop",
        },
        identifier,
    )


@pytest.mark.parametrize("limit", ["age", "history", "both"])
def test_stop_bypasses_motion_admission_limits(endpoint, limit):
    client, driver = endpoint
    deliver(client, request(client))
    if limit in {"history", "both"}:
        record = client.journal.data["records"]["route-1"]
        client.journal.data["records"].update(
            {f"past-{i}": deepcopy(record) for i in range(1023)}
        )
        ordinary = request(client, identifier="another-route")
        deliver(client, ordinary)
        assert (
            client.journal.data["records"]["another-route"]["event"]["payload"][
                "reason"
            ]
            == "command_history_full"
        )
    command = stop_request(client)
    if limit in {"age", "both"}:
        command["sent_at"] = (datetime.now(UTC) - timedelta(seconds=5)).isoformat()
    deliver(client, command)
    assert client.journal.data["active"] == "stop-1"
    assert driver.logic.active is None
    # The non-threaded driver needs a new odom sample after the large journal write.
    driver.tick()
    client._finish_stop()
    event = client.journal.data["records"]["stop-1"]["event"]
    assert event["payload"]["status"] == "completed"
    assert (
        client.journal.data["records"]["route-1"]["event"]["payload"]["status"]
        == "interrupted"
    )
    deliver(client, command)
    assert client.transport.messages[-1][1] == event
    changed = deepcopy(command)
    changed["payload"]["reason"] = "changed_reason"
    with pytest.raises(ValueError, match="idempotency_conflict"):
        deliver(client, changed)


@pytest.mark.parametrize(
    ("fault", "reason"),
    [
        ("expired", "expired_command"),
        ("boot", "boot_mismatch"),
        ("target", "stop_target_mismatch"),
    ],
)
def test_stop_preserves_expiry_boot_and_target_fences(endpoint, fault, reason):
    client, driver = endpoint
    deliver(client, request(client))
    active = driver.logic.active
    command = stop_request(client)
    if fault == "expired":
        command["sent_at"] = (datetime.now(UTC) - timedelta(seconds=5)).isoformat()
        command["expires_at"] = (datetime.now(UTC) - timedelta(seconds=1)).isoformat()
    elif fault == "boot":
        command["payload"]["expected_boot_version"] = "old-boot"
    else:
        command["payload"]["target_command_id"] = "unknown-command"
    deliver(client, command)
    assert (
        client.journal.data["records"]["stop-1"]["event"]["payload"]["reason"] == reason
    )
    assert client.journal.data["active"] == "route-1"
    assert driver.logic.active is active


@pytest.mark.parametrize(
    "condition",
    [
        "completed",
        "unknown",
        "failed",
        "old_boot",
        "newer_active",
        "moving",
        "stale",
        "line_enabled",
    ],
)
def test_stop_of_completed_target_requires_idle_fresh_stop(
    endpoint, monkeypatch, condition
):
    client, driver = endpoint
    route = request(client)
    deliver(client, route)
    driver.safe_stop("test_finished")
    client._save_event(route, client._event(route, "completed", 2, {"stopped": True}))
    state = driver.observe()
    original = client.journal.data["records"]["route-1"]
    if condition == "failed":
        original["event"]["payload"]["status"] = "failed"
    elif condition == "old_boot":
        original["event"]["payload"]["boot_version"] = "previous-boot"
    elif condition == "newer_active":
        client.journal.data["active"] = "newer-route"
    elif condition == "moving":
        state["stopped"] = False
    elif condition == "stale":
        state["observed_at"] = (datetime.now(UTC) - timedelta(seconds=5)).isoformat()
    elif condition == "line_enabled":
        state["line_enabled"] = True
    monkeypatch.setattr(driver, "observe", lambda: state.copy())
    stops = []
    monkeypatch.setattr(driver, "safe_stop", stops.append)
    before = deepcopy(client.journal.data)
    command = stop_request(client, "unknown" if condition == "unknown" else "route-1")
    deliver(client, command)
    event = client.journal.data["records"]["stop-1"]["event"]
    if condition == "completed":
        assert event["payload"]["status"] == "completed"
        assert event["payload"]["result"]["stopped"] is True
        deliver(client, command)
        assert client.transport.messages[-1][1] == event
    else:
        assert event["payload"]["status"] == "rejected"
        assert event["payload"]["reason"] == (
            "fresh_stop_required"
            if condition in {"moving", "stale", "line_enabled"}
            else "stop_target_mismatch"
        )
    assert client.journal.data["records"]["route-1"] == before["records"]["route-1"]
    assert client.journal.data["active"] == before["active"]
    assert client.journal.data["arrival"] == before["arrival"]
    assert client.halted is None and stops == []


@pytest.mark.parametrize("condition", ["moving", "line_enabled", "stale", "future"])
def test_active_stop_waits_for_fresh_stopped_observation(
    endpoint, monkeypatch, condition
):
    client, driver = endpoint
    deliver(client, request(client))
    deliver(client, stop_request(client))
    state = driver.observe()
    if condition in {"stale", "future"}:
        offset = -5 if condition == "stale" else 5
        state["observed_at"] = (
            datetime.now(UTC) + timedelta(seconds=offset)
        ).isoformat()
    else:
        state["stopped"] = condition != "moving"
        state["line_enabled"] = condition == "line_enabled"
    monkeypatch.setattr(driver, "observe", lambda: state.copy())
    client._finish_stop()
    assert client.journal.data["active"] == "stop-1"
    assert (
        client.journal.data["records"]["route-1"]["event"]["payload"]["status"]
        == "accepted"
    )
    state.update(
        stopped=True, line_enabled=False, observed_at=datetime.now(UTC).isoformat()
    )
    client._finish_stop()
    assert client.journal.data["active"] is None
    assert (
        client.journal.data["records"]["stop-1"]["event"]["payload"]["status"]
        == "completed"
    )
