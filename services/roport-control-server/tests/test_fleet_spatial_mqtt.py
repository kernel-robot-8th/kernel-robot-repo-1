"""Physical-fault/external-movement state injection over actual loopback MQTT."""

import json
import time
from copy import deepcopy
from datetime import UTC, datetime
from queue import Empty

from test_fleet_mqtt import fleet_mqtt  # noqa: F401

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport
from roport_control_server.executor.fleet_runtime import FleetRuntime


def observer(env):
    spy = MqttTransport(
        MqttConfig("127.0.0.1", "spatial-command-observer", port=env.port),
        [wire.topic("pinky1", "cmd")],
    )
    spy.start()
    env.stack.callback(spy.stop)
    return spy


def collect(spy):
    events = []
    while True:
        try:
            delivery = spy.receive(0)
        except Empty:
            return events
        events.append(wire.decode(delivery.payload))
        spy.ack(delivery)


def inject(env, *, node, mode, fault):
    client = next(c for c in env.clients if c.robot == "pinky2")
    event = client.state_message()
    event["payload"].update(
        sequence=event["payload"]["sequence"] + 100000,
        observed_at=datetime.now(UTC).isoformat(),
    )
    event["payload"]["state"].update(node=node, mode=mode, fault=fault)
    # Valid current boot and increasing sequence, not an invalid/replayed packet.
    # Lower-sequence normal fixture state cannot overwrite this injected fault.
    client.transport.publish(wire.topic("pinky2", "event"), event)
    env.pump(
        lambda: (
            bool(env.control.data["fault"])
            or env.control.observed["pinky2"]["sequence"]
            == event["payload"]["sequence"]
        )
    )
    assert env.control.observed["pinky2"] == event["payload"]
    assert env.control.clock() - env.control.heard["pinky2"] < 1
    return event


def save(env, name, **evidence):
    (env.tmp_path / "spatial-evidence.json").write_text(
        json.dumps(
            {
                "case": name,
                "injection_scope": "external movement or sensor fault; not normal VirtualPinky behavior",
                **evidence,
                "control": env.control.snapshot(),
                "robots": {
                    c.robot: {
                        "active": c.journal.data["active"],
                        "halted": c.halted,
                        "state": c.driver.observe(),
                    }
                    for c in env.clients
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n"
    )


def test_fresh_unknown_other_robot_position_blocks_new_route_over_mqtt(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
):
    env = fleet_mqtt
    spy = observer(env)
    event = inject(env, node=None, mode="unknown", fault="line_nav_unavailable")
    admitted = deepcopy(env.control.observed["pinky2"])
    key = env.submit("unknown-position", "inbound")
    deadline = time.monotonic() + 0.25
    env.pump(lambda: bool(env.control.data["fault"]) or time.monotonic() >= deadline)
    commands = collect(spy)
    save(
        env,
        "unknown_nonparticipant_position",
        injected_event=event,
        admitted_state_before_request=admitted,
        observed_commands=commands,
        request_id=key,
    )
    assert env.control.observed["pinky2"] == event["payload"]
    assert not [m for m in commands if m["type"] == "line_route_command"], (
        "A line route was published after fresh unknown-position/fault state was accepted"
    )
    assert all(c.journal.data["active"] is None for c in env.clients)


def test_new_obstacle_on_active_route_holds_and_stops_over_mqtt(
    fleet_mqtt,  # noqa: F811 - shared pytest fixture
):
    env = fleet_mqtt
    spy = observer(env)
    key = env.submit("route-intrusion", "inbound")
    env.pump(lambda: any(c.journal.data["active"] for c in env.clients))
    original = deepcopy(next(iter(env.control.data["commands"].values()))["message"])
    assert original["target"] == "pinky1"
    route = original["payload"]["route"]
    middle = route[-2]
    assert middle not in {route[0], route[-1]}
    collect(spy)  # Discard pre-injection traffic, not an immediate conflict stop.
    event = inject(env, node=middle, mode="line", fault=None)
    admitted = deepcopy(env.control.observed["pinky2"])
    runtime = FleetRuntime(env.control, env.transport)
    commands = collect(spy)
    deadline = time.monotonic() + 1.1
    while time.monotonic() < deadline:
        # One sensor tick keeps the active route long enough to inspect keepalive.
        # This replaces the fixture's accelerated pump, never another parallel owner.
        for client in env.clients:
            client.poll(0)
            if client.transport.ready and not client.halted:
                client.driver.heartbeat()
            client.driver.tick()
            for command, phase, details in client.driver.drain_results():
                client.report(command, phase, **details)
        for _ in range(12):
            runtime.poll(0)
        commands.extend(collect(spy))
        if env.control.data["fault"] and any(
            m["type"] == "robot_stop_command" for m in commands
        ):
            break
        time.sleep(0.005)
    save(
        env,
        "nonparticipant_enters_active_route",
        injected_event=event,
        admitted_state_before_followup=admitted,
        original_command=original,
        occupied_route_node=middle,
        observed_commands_after_admission=commands,
        request_id=key,
    )
    assert env.control.observed["pinky2"] == event["payload"]
    assert env.control.clock() - env.control.heard["pinky2"] < 3
    assert env.control.data["fault"], (
        "Known occupancy conflict did not put control in HOLD"
    )
    assert any(m["type"] == "robot_stop_command" for m in commands)
    assert not [
        m
        for m in commands
        if m["type"] == "robot_state_request"
        and m["payload"].get("keep_running_command_id") == original["command_id"]
    ]
