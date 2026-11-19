"""An empty logistics robot can take new work at a completed return boundary."""

from copy import deepcopy

import pytest
from test_fleet import complete, dispatch, finish, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile as profile  # noqa: PLC0414

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.executor.fleet import FleetControl


def start_return(control):
    control.assign("pinky2", None)
    key = submit(control, "stored-box")
    for _ in range(12):
        if control.data["tasks"][key]["stage"] == "return_route":
            break
        command = dispatch(control)
        assert command is not None
        complete(control, command)
    task = control.data["tasks"][key]
    assert task["stage"] == "return_route"
    assert control.data["slots"][task["slot"]]["state"] == "occupied"
    command = dispatch(control)
    assert command["type"] == "line_route_command"
    assert command["payload"]["goal"]["node"] != task["parking"]
    return key, command


def test_returning_robot_reassigned_only_after_segment_completion(fleet):
    previous, returning = start_return(fleet)
    before = deepcopy(fleet.data)
    robot = returning["target"]
    requested = submit(fleet, "next-box")

    assert dispatch(fleet) is None
    assert fleet.data["tasks"][requested]["stage"] == "queued"
    assert fleet.data["tasks"][previous]["status"] == "RUNNING"
    assert (
        fleet.data["commands"][returning["command_id"]]
        == before["commands"][returning["command_id"]]
    )

    complete(fleet, returning)
    incoming = dispatch(fleet)
    old, new = fleet.data["tasks"][previous], fleet.data["tasks"][requested]
    assert incoming["target"] == robot == new["robot"]
    assert incoming["type"] == "line_route_command"
    assert incoming["payload"]["start_node"] == returning["payload"]["goal"]["node"]
    assert incoming["payload"]["goal"]["node"] == "N01"
    assert old["stage"] == "done" and old["status"] == "COMPLETE"
    assert old["return_reassigned_to"] == requested
    assert new["parking"] == old["parking"]
    assert fleet.data["slots"] == before["slots"]
    assert not fleet.data["fault"]
    assert set(fleet.data["resources"].values()) == {requested}


def test_reassignment_uses_fifo_new_request(fleet):
    previous, returning = start_return(fleet)
    first = submit(fleet, "first-next")
    second = submit(fleet, "second-next")
    complete(fleet, returning)
    command = dispatch(fleet)
    assert fleet.data["commands"][command["command_id"]]["task"] == first
    assert fleet.data["tasks"][previous]["return_reassigned_to"] == first
    assert fleet.data["tasks"][second]["stage"] == "queued"


def test_reassignment_waits_for_fresh_state_after_completed_result(fleet):
    previous, returning = start_return(fleet)
    requested = submit(fleet, "next-box")
    robot = returning["target"]
    old_state = deepcopy(fleet.observed[robot])
    event = wire.build_message(
        "command_status",
        robot,
        returning["source"],
        {
            "boot_version": returning["payload"]["expected_boot_version"],
            "observed_at": returning["sent_at"],
            "sequence": 2,
            "status": "completed",
            "result": {
                "node": returning["payload"]["goal"]["node"],
                "mode": "line",
                "stopped": True,
            },
        },
        command_id=returning["command_id"],
    )
    assert fleet.result(event)
    assert dispatch(fleet) is None
    fleet.observe({"source": robot, "payload": old_state})
    assert dispatch(fleet) is None
    assert fleet.data["tasks"][previous]["status"] == "RUNNING"
    assert fleet.data["tasks"][requested]["stage"] == "queued"

    arrival_node = returning["payload"]["goal"]["node"]
    delayed = deepcopy(old_state)
    delayed["sequence"] += 1
    assert delayed["state"]["node"] != arrival_node
    assert delayed["state"]["stopped"] is True
    assert delayed["active_command_id"] is None
    fleet.observe({"source": robot, "payload": delayed})
    assert dispatch(fleet) is None
    assert fleet.observed[robot]["state"]["node"] == arrival_node
    assert fleet.data["last_nodes"][robot] == arrival_node

    state = deepcopy(delayed)
    for change in (
        {"node": arrival_node},
        {
            "node": delayed["state"]["node"],
            "arrival_command_id": returning["command_id"],
        },
        {"node": arrival_node, "stopped": False},
    ):
        state["sequence"] += 1
        state["state"].update(change)
        fleet.observe({"source": robot, "payload": state})
        assert dispatch(fleet) is None

    state["sequence"] += 1
    state["state"].update(stopped=True, heading_rad=1.2)
    fleet.observe({"source": robot, "payload": state})
    command = dispatch(fleet)
    assert command["target"] == robot
    assert command["payload"]["start_node"] == arrival_node
    assert command["payload"]["start_heading_rad"] == 1.2
    assert fleet.data["tasks"][previous]["return_reassigned_to"] == requested


@pytest.mark.parametrize(
    "unready", ["stale", "moving", "fault", "line_enabled", "drive_owner"]
)
def test_unready_returning_robot_is_not_reassigned(fleet, unready):
    previous, returning = start_return(fleet)
    complete(fleet, returning)
    robot = returning["target"]
    if unready == "stale":
        fleet.heard[robot] -= 4
    elif unready == "moving":
        fleet.observed[robot]["state"]["stopped"] = False
    elif unready == "fault":
        fleet.observed[robot]["state"]["fault"] = "local_fault"
    elif unready == "line_enabled":
        fleet.observed[robot]["state"]["line_enabled"] = True
    else:
        fleet.observed[robot]["state"]["drive_owner"] = "line"
    requested = submit(fleet, "next-box")
    assert dispatch(fleet) is None
    assert fleet.data["tasks"][previous]["status"] == "RUNNING"
    assert "return_reassigned_to" not in fleet.data["tasks"][previous]
    assert fleet.data["tasks"][requested]["stage"] == "queued"


def test_loaded_logistics_finishes_transfer_before_new_assignment(fleet):
    fleet.assign("pinky2", None)
    previous = submit(fleet, "loaded-box")
    complete(fleet, dispatch(fleet))
    complete(fleet, dispatch(fleet))
    assert fleet.data["tasks"][previous]["stage"] == "choose_slot"
    requested = submit(fleet, "next-box")
    command = dispatch(fleet)
    assert fleet.data["commands"][command["command_id"]]["task"] == previous
    assert fleet.data["tasks"][previous]["stage"] == "shelf_route"
    assert "return_reassigned_to" not in fleet.data["tasks"][previous]
    assert fleet.data["tasks"][requested]["stage"] == "queued"
    complete(fleet, command)
    transfer = dispatch(fleet)
    assert transfer["type"] == "arm_action_command"
    assert fleet.data["tasks"][requested]["stage"] == "queued"


def test_transfer_departure_is_completed_before_reassignment(fleet):
    fleet.assign("pinky2", None)
    previous = submit(fleet, "stored-box")
    for _ in range(4):
        complete(fleet, dispatch(fleet))
    task = fleet.data["tasks"][previous]
    assert task["stage"] == "return_route" and task["departure"]
    departure = deepcopy(task["departure"])
    requested = submit(fleet, "next-box")
    command = dispatch(fleet)
    assert fleet.data["commands"][command["command_id"]]["task"] == previous
    assert command["payload"]["departure"] == departure
    assert fleet.data["tasks"][requested]["stage"] == "queued"
    assert "return_reassigned_to" not in fleet.data["tasks"][previous]
    complete(fleet, command)
    incoming = dispatch(fleet)
    assert fleet.data["commands"][incoming["command_id"]]["task"] == requested
    assert "departure" not in incoming["payload"]
    assert fleet.data["tasks"][previous]["return_reassigned_to"] == requested


def test_smartcart_is_not_reassigned_to_logistics(fleet):
    fleet.assign("pinky2", None)
    submit(fleet, "cart-box")
    finish(fleet)
    fleet.assign("pinky1", None)
    outbound = submit(fleet, "cart-box", "outbound")
    finish(fleet)
    assert fleet.data["tasks"][outbound]["stage"] == "await_receipt"
    requested = submit(fleet, "next-box")
    assert dispatch(fleet) is None
    assert fleet.data["tasks"][requested]["stage"] == "queued"
    fleet.receipt(outbound)
    commands = finish(fleet)
    assert commands and all(c["target"] == "pinky3" for c in commands)
    assert fleet.data["tasks"][outbound]["status"] == "COMPLETE"
    assert "return_reassigned_to" not in fleet.data["tasks"][outbound]
    assert fleet.data["tasks"][requested]["stage"] == "queued"
    assert fleet.observed["pinky3"]["state"]["node"] in {"H1", "H2"}


def test_without_new_work_return_segments_reach_reserved_parking(fleet):
    previous, returning = start_return(fleet)
    complete(fleet, returning)
    commands = finish(fleet)
    task = fleet.data["tasks"][previous]
    assert commands and all(c["type"] == "line_route_command" for c in commands)
    assert task["stage"] == "done" and task["status"] == "COMPLETE"
    assert "return_reassigned_to" not in task
    assert fleet.observed[task["robot"]]["state"]["node"] == task["parking"]
    assert fleet.data["slots"][task["slot"]] == {
        "state": "occupied",
        "package_id": "stored-box",
    }
    assert not fleet.data["commands"] and not fleet.data["resources"]


def final_return_without_state(control):
    request, command = start_return(control)
    while (
        command["payload"]["goal"]["node"] != control.data["tasks"][request]["parking"]
    ):
        complete(control, command)
        command = dispatch(control)
    robot = command["target"]
    previous = deepcopy(control.observed[robot])
    complete(control, command, observe=False)
    assert control.data["tasks"][request]["status"] == "COMPLETE"
    assert not control.data["commands"]
    return command, previous


def test_final_arrival_after_pinky_restart_requires_explicit_confirmation(fleet):
    command, old = final_return_without_state(fleet)
    robot, node = command["target"], command["payload"]["goal"]["node"]
    fresh = deepcopy(old)
    fresh.update(boot_version="restarted", sequence=1)
    fresh["state"].update(arrival_command_id=None, node=old["state"]["node"])
    fleet.observe({"source": robot, "payload": fresh})
    assert fleet.observed[robot]["boot_version"] == "restarted"
    assert fleet.heard[robot] == fleet.clock()
    assert {node, old["state"]["node"]} <= fleet.occupied("pinky2")
    with pytest.raises(ValueError, match="arrival requires recovery"):
        fleet.ready(robot)
    with pytest.raises(ValueError, match="fresh stopped state"):
        fleet.confirm_arrival_after_restart(robot)

    delayed = deepcopy(old)
    delayed["sequence"] += 1
    delayed["state"].update(node=node, arrival_command_id=command["command_id"])
    fleet.observe({"source": robot, "payload": delayed})
    assert fleet.observed[robot]["boot_version"] == "restarted"

    fresh["sequence"] += 1
    fresh["state"].update(node=node, heading_rad=1.2)
    fleet.observe({"source": robot, "payload": fresh})
    requested = submit(fleet, "after-restart")
    assert dispatch(fleet) is None
    fleet.confirm_arrival_after_restart(robot)
    after = dispatch(fleet)
    assert after["target"] == robot
    assert after["payload"]["start_node"] == node
    assert after["payload"]["start_heading_rad"] == 1.2
    assert fleet.data["commands"][after["command_id"]]["task"] == requested


def test_final_pending_arrival_survives_owner_restart(fleet):
    command, delayed = final_return_without_state(fleet)
    robot, node = command["target"], command["payload"]["goal"]["node"]
    requested = submit(fleet, "after-owner-restart")
    other_states = deepcopy({r: p for r, p in fleet.observed.items() if r != robot})
    path = fleet.store.path
    fleet.store.close()
    journal = Journal(path)
    try:
        restored = FleetControl(fleet.config, journal, clock=fleet.clock)
        assert not restored.data["enabled"]
        for other, state in other_states.items():
            restored.observe({"source": other, "payload": state})
        restored.enable()
        delayed["sequence"] += 1
        restored.observe({"source": robot, "payload": delayed})
        assert dispatch(restored) is None
        assert restored.data["last_nodes"][robot] == node
        assert restored.data["tasks"][requested]["stage"] == "queued"

        delayed["sequence"] += 1
        delayed["state"].update(node=node, arrival_command_id=command["command_id"])
        restored.observe({"source": robot, "payload": delayed})
        after = dispatch(restored)
        assert after["target"] == robot
        assert after["payload"]["start_node"] == node
        assert robot not in restored.data["pending_arrivals"]
    finally:
        journal.close()
