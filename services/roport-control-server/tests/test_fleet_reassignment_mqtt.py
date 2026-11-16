"""Reassign an empty robot during its normal segmented return over real MQTT."""

import json

from test_fleet_concurrent_requests import Trace
from test_fleet_mqtt import fleet_mqtt as fleet_mqtt  # noqa: PLC0414


def test_new_inbound_during_return_reuses_robot_before_parking(fleet_mqtt, monkeypatch):
    env = fleet_mqtt
    control = env.control
    control.assign("pinky2", None)
    trace = Trace(env, monkeypatch)
    previous = env.submit("first-box", "inbound")

    def active_return(data):
        return next(
            (
                command
                for command in data["commands"].values()
                if command["task"] == previous and command["stage"] == "return_route"
            ),
            None,
        )

    def return_is_moving(data):
        command = active_return(data)
        if not command:
            return False
        message = command["message"]
        client = next(c for c in env.clients if c.robot == message["target"])
        return (
            client.journal.data["active"] == message["command_id"]
            and not client.driver.observe()["stopped"]
        )

    trace.until(return_is_moving)
    returning = active_return(control.data)["message"]
    old_task = control.data["tasks"][previous]
    parking = old_task["parking"]
    robot = returning["target"]
    assert returning["payload"]["goal"]["node"] != parking
    requested = env.submit("second-box", "inbound")
    assert control.data["tasks"][requested]["stage"] == "queued"
    assert control.data["tasks"][previous]["status"] == "RUNNING"
    assert returning["command_id"] in control.data["commands"]
    trace.until(
        lambda data: data["tasks"][previous].get("return_reassigned_to") == requested
    )

    handoff = control.snapshot()
    old, new = handoff["tasks"][previous], handoff["tasks"][requested]
    assert old["status"] == "COMPLETE" and old["stage"] == "done"
    assert new["robot"] == robot and new["parking"] == parking
    assert handoff["robots"][robot]["state"]["node"] != parking
    start = next(
        index
        for index, command in enumerate(trace.commands)
        if command["command_id"] == returning["command_id"]
    )
    return_commands = [
        command
        for command in trace.commands[start:]
        if command["request_id"] == previous
    ]
    assert return_commands and all(
        parking not in command["route"] for command in return_commands
    )
    assert handoff["slots"][old["slot"]] == {
        "state": "occupied",
        "package_id": "first-box",
    }

    trace.until(lambda data: data["tasks"][requested]["status"] == "COMPLETE")
    assert control.observed[robot]["state"]["node"] == parking
    assert {
        slot["package_id"]
        for slot in control.data["slots"].values()
        if slot["state"] == "occupied"
    } == {"first-box", "second-box"}
    assert not control.data["commands"] and not control.data["resources"]
    assert not control.data["fault"] and not env.host.data["hold"]
    assert all(
        record["command"]["type"] != "robot_stop_command"
        for client in env.clients
        for record in client.journal.data["records"].values()
    )
    trace.save()
    (env.tmp_path / "reassignment-summary.json").write_text(
        json.dumps(
            {
                "result": "passed",
                "request_received_during_command": returning,
                "handoff": handoff,
                "final_tasks": control.data["tasks"],
                "limits": "Real loopback MQTT and Pinky driver; simulated device I/O and OMX.",
            },
            indent=2,
        )
        + "\n"
    )
