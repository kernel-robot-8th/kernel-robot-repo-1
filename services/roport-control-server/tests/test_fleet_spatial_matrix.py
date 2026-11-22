"""Real layout combinations with synthetic completion, not physical motion evidence."""

from copy import deepcopy
from itertools import pairwise

import pytest
from test_fleet import dispatch, finish, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401

from roport_control_server.executor.fleet import SLOTS


def observe(fleet, robot, **state):
    payload = deepcopy(fleet.observed[robot])
    payload["sequence"] += 1
    payload["state"].update(state)
    fleet.observe({"source": robot, "payload": payload})


@pytest.mark.parametrize("size", ["small", "large"])
@pytest.mark.parametrize("logistics_bay", ["N16", "N17"])
@pytest.mark.parametrize("cart_bay", ["H1", "H2"])
def test_full_cycle_uses_free_bays_and_correct_shelf(
    fleet, size, logistics_bay, cart_bay
):
    observe(fleet, "pinky1", node=logistics_bay)
    observe(fleet, "pinky2", node="N17" if logistics_bay == "N16" else "N16")
    fleet.assign("pinky2", None)
    observe(fleet, "pinky3", node=cart_bay)
    inbound = submit(fleet, "box")
    commands = finish(fleet, size=size)
    task = fleet.data["tasks"][inbound]
    assert task["status"] == "COMPLETE"
    assert task["robot"] == "pinky1" and task["parking"] == logistics_bay
    assert fleet.observed["pinky1"]["state"]["node"] == logistics_bay
    slot = task["slot"]
    assert SLOTS[slot]["size"] == size
    transfer = next(
        m
        for m in commands
        if m["type"] in {"arm_action_command", "shelf_transfer_command"}
    )
    assert transfer["target"] == ("shelf-01" if size == "large" else "omx1")
    outbound = submit(fleet, "box", "outbound")
    commands += finish(fleet)
    assert fleet.data["tasks"][outbound]["stage"] == "await_receipt"
    assert dispatch(fleet) is None
    fleet.receipt(outbound)
    commands += finish(fleet)
    assert fleet.data["tasks"][outbound]["status"] == "COMPLETE"
    assert fleet.observed["pinky3"]["state"]["node"] in {"H1", "H2"}
    assert fleet.data["slots"][slot] == {"state": "empty", "package_id": None}
    assert [
        m["payload"]["to_mode"]
        for m in commands
        if m["type"] == "set_drive_mode_command"
    ] == ["nav2", "line"]
    assert len([m for m in commands if m["type"] == "nav_goal_command"]) == 2
    assert fleet.data["resources"] == {} and fleet.data["commands"] == {}


@pytest.mark.parametrize("slot", list(SLOTS))
def test_outbound_all_six_slots_keep_actual_arm_and_dock(fleet, slot):
    data = deepcopy(fleet.data)
    data["slots"][slot] = {"state": "occupied", "package_id": "stored"}
    fleet.save(data)
    key = submit(fleet, "stored", "outbound")
    commands = finish(fleet)
    assert fleet.data["tasks"][key]["slot"] == slot
    assert commands[0]["payload"]["goal"]["node"] == SLOTS[slot]["node"]
    assert commands[1]["target"] == (
        "shelf-01" if slot.endswith("b") else SLOTS[slot]["arms"][0]
    )
    assert fleet.data["tasks"][key]["stage"] == "await_receipt"
    assert fleet.data["slots"][slot]["state"] == "empty"
    fleet.receipt(key)
    finish(fleet)
    assert fleet.data["tasks"][key]["status"] == "COMPLETE"


@pytest.mark.parametrize(
    ("blocked", "destination"),
    [
        ("N01", "N18"),
        ("N14", None),
        ("N18", "N10"),
        ("N19", "N01"),
        ("N11", "N01"),
        ("N21", "N01"),
        ("N22", "N01"),
        ("N12", "N01"),
    ],
)
@pytest.mark.parametrize("excluded", [False, True])
def test_initial_node_occupancy_is_avoided_or_waited(
    fleet, blocked, destination, excluded
):
    if excluded:
        fleet.assign("pinky2", None)
    observe(fleet, "pinky2", node=blocked)
    key = submit(fleet, "box")
    cmd = dispatch(fleet)
    if destination is None:
        assert cmd is None
        assert fleet.data["tasks"][key]["stage"] in {"queued", "warehouse_route"}
        assert fleet.data["tasks"][key]["reason"]
        assert fleet.data["fault"] is None
    else:
        assert cmd is not None
        path = cmd["payload"]["route"]
        assert blocked not in path
        assert path[0] == "N16"
        assert path[-1] == destination
        assert all(b in fleet.config["nodes"][a] for a, b in pairwise(path))
        assert fleet.data["tasks"][key]["robot"] == "pinky1"


@pytest.mark.parametrize("fault", ["moving", "active", "fault", "heading"])
def test_unavailable_logistics_never_emits_motion(fleet, fault):
    for robot in ("pinky1", "pinky2"):
        if fault == "active":
            fleet.observed[robot]["active_command_id"] = "other-command"
        else:
            change = {
                "moving": {"stopped": False},
                "fault": {"fault": "motor_fault"},
                "heading": {"heading_rad": None},
            }[fault]
            observe(fleet, robot, **change)
    key = submit(fleet, "box")
    assert dispatch(fleet) is None
    assert fleet.data["commands"] == {} and fleet.data["resources"] == {}
    assert fleet.data["tasks"][key]["reason"]


@pytest.mark.parametrize("size", ["small", "large"])
def test_full_compatible_shelf_waits_without_overwriting_stock(fleet, size):
    data = deepcopy(fleet.data)
    for slot, definition in SLOTS.items():
        if definition["size"] == size:
            data["slots"][slot] = {
                "state": "occupied",
                "package_id": "existing-" + slot,
            }
    fleet.save(data)
    stock = deepcopy(fleet.data["slots"])
    key = submit(fleet, "new")
    commands = finish(fleet, size=size)
    assert [m["type"] for m in commands] == [
        "line_route_command",
        "warehouse_load_command",
    ]
    assert fleet.data["tasks"][key]["stage"] == "choose_slot"
    assert fleet.data["slots"] == stock
    assert fleet.data["commands"] == {} and fleet.data["fault"] is None
