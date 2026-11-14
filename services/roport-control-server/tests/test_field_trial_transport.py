"""The v2 boundary preserves the trial owner's rejection and durable ACK rules."""

from datetime import UTC, datetime
from uuid import uuid4

import pytest
import test_pinky_trial as trial_tests

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.field_trial_transport import (
    FieldTrialTransport,
)
from roport_control_server.adapters.mqtt.motion import MotionReceiver
from roport_control_server.adapters.mqtt.mqtt import Delivery, MqttConfig

profile = trial_tests.profile
control_memory = trial_tests.control_memory


@pytest.fixture
def field_control(control_memory):
    control = control_memory
    raw = control.transport
    adapter = FieldTrialTransport(
        MqttConfig("127.0.0.1", "field-boundary-unit"),
        [wire.topic("pinky1", "event")],
        control.config,
        lambda: control.state,
    )
    adapter.raw = raw
    control.transport = adapter
    control.start()
    return control, raw


def accepted(control):
    command = control.state["active"]
    state = control.state["statuses"][command["target"]]
    result = {key: state[key] for key in wire.PINKY_STATE["properties"] if key in state}
    result.update(node=state["node_id"], arrival_command_id=None)
    return wire.build_message(
        "command_status",
        command["target"],
        control.config["control_id"],
        {
            "boot_version": state["boot_id"],
            "observed_at": datetime.now(UTC).isoformat(),
            "sequence": 1,
            "status": "accepted",
            "result": result,
        },
        command_id=command["command_id"],
    )


@pytest.mark.parametrize(
    "problem",
    [
        "wrong_source_topic",
        "wrong_channel",
        "missing_result_state",
        "unknown_command",
        "invalid_json",
    ],
)
def test_invalid_field_delivery_holds_and_transport_acks_without_crashing(
    field_control, problem
):
    control, raw = field_control
    message = accepted(control)
    destination = wire.topic(message["source"], "event")
    if problem == "wrong_source_topic":
        destination = wire.topic("pinky3", "event")
    elif problem == "wrong_channel":
        destination = wire.topic(message["source"], "motion")
    elif problem == "missing_result_state":
        del message["payload"]["result"]["stopped"]
    elif problem == "unknown_command":
        message["command_id"] = "unknown-command"
    payload = b"{" if problem == "invalid_json" else wire.encode(message)
    incoming = Delivery(destination, payload, False, 1, 1, 0)
    raw.incoming.put(incoming)
    control.poll(0)
    assert control.state["phase"] == "HOLD"
    assert control.state["fault"] == "invalid_trial_event"
    assert control.state["events"] == {}
    assert control.state["completed"] == {}
    assert raw.acked == [incoming]
    assert not any(m["type"] == "event_stored" for _, m in raw.sent)
    assert sum(m["type"] == "line_route_command" for _, m in raw.sent) == 1


def test_valid_field_event_and_duplicate_keep_durable_ack_contract(field_control):
    control, raw = field_control
    message = accepted(control)
    incoming = Delivery(
        wire.topic(message["source"], "event"), wire.encode(message), False, 1, 1, 0
    )
    for _ in range(2):
        raw.incoming.put(incoming)
        control.poll(0)
    assert control.state["phase"] != "HOLD"
    assert list(control.state["events"]) == [message["message_id"]]
    assert len(raw.acked) == 2
    stored = [m for _, m in raw.sent if m["type"] == "event_stored"]
    assert [m["payload"]["acked_message_id"] for m in stored] == [
        message["message_id"]
    ] * 2


def test_bad_field_motion_is_dropped_and_next_sample_is_usable(field_control):
    control, raw = field_control
    receiver = MotionReceiver(control.config, control.transport)
    boot = str(uuid4())
    statuses = {
        **control.state["statuses"],
        "pinky1": {**control.state["statuses"]["pinky1"], "boot_id": boot},
    }
    message = wire.build_message(
        "robot_pose",
        "pinky1",
        control.config["control_id"],
        {
            "boot_version": boot,
            "observed_at": datetime.now(UTC).isoformat(),
            "sequence": 1,
            "map_version": control.config["map_version"],
            "frame_id": "map",
            "pose_source": "tf",
            "pose": {"x": 0.1, "y": 0.2, "yaw": 0.0},
        },
    )
    for robot in ("pinky3", "pinky1"):
        raw.incoming.put(
            Delivery(wire.topic(robot, "motion"), wire.encode(message), False, 1, 0, 0)
        )
    receiver.poll(statuses)
    assert receiver.rejected == 1
    assert receiver.snapshot(statuses)["pinky1"]["pose"]["x"] == 0.1
    assert control.state["phase"] != "HOLD"
