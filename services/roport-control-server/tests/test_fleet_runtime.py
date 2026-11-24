import json
from copy import deepcopy
from datetime import UTC, datetime, timedelta

import pytest
from test_fleet import dispatch, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import MemoryTransport, profile  # noqa: F401

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import Delivery
from roport_control_server.executor.fleet_runtime import FleetRuntime


@pytest.mark.parametrize("kind", ["robot_state", "robot_pose"])
@pytest.mark.parametrize("offset", [-4, 2])
def test_out_of_range_observation_logs_reason_without_refreshing_state(
    fleet, caplog, kind, offset
):
    robot = "pinky1"
    before = deepcopy(fleet.observed[robot])
    p = {
        "boot_version": before["boot_version"],
        "sequence": 2,
        "observed_at": (datetime.now(UTC) + timedelta(seconds=offset)).isoformat(),
    }
    if kind == "robot_state":
        p.update(active_command_id=None, state=before["state"])
    else:
        p.update(
            map_version=fleet.config["map_version"],
            frame_id="map",
            pose_source="amcl",
            pose={"x": 1, "y": 2, "yaw": 0},
        )
    message = wire.build_message(kind, robot, fleet.config["control_id"], p)
    FleetRuntime(fleet, MemoryTransport()).handle(
        Delivery(
            wire.topic(robot, "motion" if kind == "robot_pose" else "event"),
            wire.encode(message),
            False,
            1,
            0 if kind == "robot_pose" else 1,
            0,
        )
    )
    assert fleet.observed[robot] == before and robot not in fleet.poses
    assert "clock_skew_or_delay" in caplog.text
    assert robot in caplog.text and kind in caplog.text
    assert message["message_id"] not in caplog.text


@pytest.mark.parametrize("field", ["observed_at", "boot_version", "sequence"])
@pytest.mark.parametrize("malformation", ["missing", "wrong_type"])
def test_malformed_status_holds_and_broker_acks_without_application_ack(
    fleet, field, malformation
):
    submit(fleet, "malformed")
    command = dispatch(fleet)
    message = wire.build_message(
        "command_status",
        command["target"],
        command["source"],
        {
            "boot_version": command["payload"]["expected_boot_version"],
            "observed_at": datetime.now(UTC).isoformat(),
            "sequence": 2,
            "status": "completed",
            "result": {"stopped": True},
        },
        command_id=command["command_id"],
    )
    if malformation == "missing":
        del message["payload"][field]
    else:
        message["payload"][field] = []
    delivery = Delivery(
        wire.topic(command["target"], "event"),
        json.dumps(message).encode(),
        False,
        1,
        1,
        0,
    )
    transport = MemoryTransport()
    transport.incoming.put(delivery)
    runtime = FleetRuntime(fleet, transport)
    runtime.poll(0)
    runtime.poll(0)
    assert transport.acked == [delivery]
    assert fleet.data["fault"] and not fleet.data["enabled"]
    assert fleet.data["tasks"]["inbound-malformed"]["status"] == "HOLD"
    assert not any(m["type"] == "event_stored" for _, m in transport.sent)
    assert not fleet.data["events"]
