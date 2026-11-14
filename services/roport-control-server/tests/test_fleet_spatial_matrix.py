"""Spatial adversarial checks; synthetic completion is not physical motion evidence."""

import json
import time
from copy import deepcopy
from itertools import pairwise
from queue import Empty

import pytest
from test_fleet import complete, dispatch, finish, fleet, profile, submit  # noqa: F401

from roport_control_server.executor.fleet import SLOTS
from roport_control_server.executor.fleet_runtime import FleetRuntime


@pytest.fixture
def c(fleet, tmp_path, request, monkeypatch):  # noqa: F811
    commands = []
    original = fleet.dispatched

    def record(key, message):
        original(key, message)
        commands.append(deepcopy(message))

    monkeypatch.setattr(fleet, "dispatched", record)
    yield fleet
    (tmp_path / "spatial-summary.json").write_text(
        json.dumps(
            {
                "case": request.node.nodeid,
                "scope": "FleetControl + real graph; synthetic device state/completion",
                "commands": commands,
                "snapshot": fleet.snapshot(),
            },
            indent=2,
        )
        + "\n"
    )


def observe(c, robot, **state):
    payload = deepcopy(c.observed[robot])
    payload["sequence"] += 1
    payload["state"].update(state)
    c.observe({"source": robot, "payload": payload})


@pytest.mark.parametrize("size", ["small", "large"])
@pytest.mark.parametrize("logistics_bay", ["N16", "N17"])
@pytest.mark.parametrize("cart_bay", ["H1", "H2"])
def test_full_cycle_uses_free_bays_and_correct_shelf(c, size, logistics_bay, cart_bay):
    observe(c, "pinky1", node=logistics_bay)
    observe(c, "pinky2", node="N17" if logistics_bay == "N16" else "N16")
    c.assign("pinky2", None)
    observe(c, "pinky3", node=cart_bay)
    inbound = submit(c, "box")
    commands = finish(c, size=size)
    task = c.data["tasks"][inbound]
    assert task["status"] == "COMPLETE"
    assert task["robot"] == "pinky1" and task["parking"] == logistics_bay
    assert c.observed["pinky1"]["state"]["node"] == logistics_bay
    slot = task["slot"]
    assert SLOTS[slot]["size"] == size
    transfer = next(
        m
        for m in commands
        if m["type"] in {"arm_action_command", "shelf_transfer_command"}
    )
    assert transfer["target"] == ("shelf-01" if size == "large" else "omx1")
    outbound = submit(c, "box", "outbound")
    commands += finish(c)
    assert c.data["tasks"][outbound]["stage"] == "await_receipt"
    assert dispatch(c) is None
    c.receipt(outbound)
    commands += finish(c)
    assert c.data["tasks"][outbound]["status"] == "COMPLETE"
    assert c.observed["pinky3"]["state"]["node"] in {"H1", "H2"}
    assert c.data["slots"][slot] == {"state": "empty", "package_id": None}
    assert [
        m["payload"]["to_mode"]
        for m in commands
        if m["type"] == "set_drive_mode_command"
    ] == ["nav2", "line"]
    assert len([m for m in commands if m["type"] == "nav_goal_command"]) == 2
    assert c.data["resources"] == {} and c.data["commands"] == {}


@pytest.mark.parametrize("slot", list(SLOTS))
def test_outbound_all_six_slots_keep_actual_arm_and_dock(c, slot):
    data = deepcopy(c.data)
    data["slots"][slot] = {"state": "occupied", "package_id": "stored"}
    c.save(data)
    key = submit(c, "stored", "outbound")
    commands = finish(c)
    assert c.data["tasks"][key]["slot"] == slot
    assert commands[0]["payload"]["goal"]["node"] == SLOTS[slot]["node"]
    assert commands[1]["target"] == (
        "shelf-01" if slot.endswith("b") else SLOTS[slot]["arms"][0]
    )
    assert c.data["tasks"][key]["stage"] == "await_receipt"
    assert c.data["slots"][slot]["state"] == "empty"
    c.receipt(key)
    finish(c)
    assert c.data["tasks"][key]["status"] == "COMPLETE"


@pytest.mark.parametrize(
    "blocked", ["N01", "N14", "N18", "N19", "N11", "N21", "N22", "N12"]
)
@pytest.mark.parametrize("excluded", [False, True])
def test_initial_node_occupancy_is_avoided_or_waited(c, blocked, excluded):
    if excluded:
        c.assign("pinky2", None)
    observe(c, "pinky2", node=blocked)
    key = submit(c, "box")
    cmd = dispatch(c)
    if cmd is None:
        assert c.data["tasks"][key]["stage"] in {"queued", "warehouse_route"}
        assert c.data["tasks"][key]["reason"]
        assert c.data["fault"] is None
    else:
        path = cmd["payload"]["route"]
        assert blocked not in path
        assert path[0] == "N16"
        assert path[-1] == "N01" or blocked in {"N01", "N18"}
        assert all(b in c.config["nodes"][a] for a, b in pairwise(path))
        assert c.data["tasks"][key]["robot"] == "pinky1"


@pytest.mark.parametrize("fault", ["moving", "active", "fault", "heading"])
def test_unavailable_logistics_never_emits_motion(c, fault):
    for robot in ("pinky1", "pinky2"):
        if fault == "active":
            c.observed[robot]["active_command_id"] = "other-command"
        else:
            change = {
                "moving": {"stopped": False},
                "fault": {"fault": "motor_fault"},
                "heading": {"heading_rad": None},
            }[fault]
            observe(c, robot, **change)
    key = submit(c, "box")
    assert dispatch(c) is None
    assert c.data["commands"] == {} and c.data["resources"] == {}
    assert c.data["tasks"][key]["reason"]


@pytest.mark.parametrize("size", ["small", "large"])
def test_full_compatible_shelf_waits_without_overwriting_stock(c, size):
    data = deepcopy(c.data)
    for slot, definition in SLOTS.items():
        if definition["size"] == size:
            data["slots"][slot] = {
                "state": "occupied",
                "package_id": "existing-" + slot,
            }
    c.save(data)
    stock = deepcopy(c.data["slots"])
    key = submit(c, "new")
    commands = finish(c, size=size)
    assert [m["type"] for m in commands] == [
        "line_route_command",
        "warehouse_load_command",
    ]
    assert c.data["tasks"][key]["stage"] == "choose_slot"
    assert c.data["slots"] == stock
    assert c.data["commands"] == {} and c.data["fault"] is None


@pytest.mark.parametrize("mode", ["line", "unknown"])
def test_fresh_unknown_line_position_does_not_become_free_space(c, mode):
    observe(c, "pinky2", node="N18")
    observe(c, "pinky2", node=None, mode=mode, fault="line_nav_unavailable")
    submit(c, "box")
    assert dispatch(c) is None, "unknown physical line position allowed a new route"


def test_excluded_robot_lost_position_keeps_its_last_occupied_node(c):
    observe(c, "pinky2", node="N18")
    c.assign("pinky2", None)
    observe(c, "pinky2", node=None, mode="unknown", fault="line_nav_unavailable")
    assert "N18" in c.occupied("pinky1"), (
        "fresh unknown state erased last known occupancy"
    )


class CaptureTransport:
    ready = True

    def __init__(self):
        self.sent = []

    def publish(self, topic, message):
        self.sent.append(deepcopy(message))

    def receive(self, timeout):
        raise Empty


@pytest.mark.parametrize("intrusion", ["occupied_route", "lost_position"])
def test_active_route_reacts_to_other_robot_new_hazard(c, intrusion):
    submit(c, "box")
    active = dispatch(c)
    assert active and active["target"] == "pinky1"
    route = active["payload"]["route"]
    if intrusion == "occupied_route":
        observe(c, "pinky2", node=route[1])
    else:
        observe(c, "pinky2", node=None, mode="unknown", fault="line_nav_unavailable")
    transport = CaptureTransport()
    runtime = FleetRuntime(c, transport)
    runtime.poll(0)
    runtime.poll(0)
    (c.store.path.parent / "intrusion-wire.json").write_text(
        json.dumps(transport.sent, indent=2)
    )
    assert not c.data["enabled"], (
        "active route remained enabled after new occupancy uncertainty"
    )
    assert any(m["type"] == "robot_stop_command" for m in transport.sent)
    assert not any(m["payload"].get("keep_running_command_id") for m in transport.sent)
    assert c.data["commands"] and c.data["resources"]


def test_reserved_return_bay_new_occupant_blocks_return_only(c):
    submit(c, "box")
    finish(c)
    key = submit(c, "box", "outbound")
    finish(c)
    assert c.data["tasks"][key]["stage"] == "await_receipt"
    bay = c.data["tasks"][key]["parking"]
    observe(c, "pinky2", node=bay)
    c.receipt(key)
    commands = finish(c)
    assert [m["type"] for m in commands] == [
        "nav_goal_command",
        "set_drive_mode_command",
        "line_route_command",
    ]
    assert commands[-1]["payload"]["goal"]["node"] != bay
    assert c.data["tasks"][key]["stage"] == "return_route"
    assert c.data["tasks"][key]["reason"]
    assert c.data["commands"] == {}


def test_nav2_robot_without_line_node_does_not_block_warehouse(c):
    observe(c, "pinky3", node=None, mode="nav2")
    submit(c, "box")
    assert dispatch(c) is not None


@pytest.mark.parametrize(
    "other_node,index,held", [("N16", 2, True), ("N16", 3, False), ("N17", 2, False)]
)
def test_passed_node_is_protected_until_clear_of_remaining_route(
    c, other_node, index, held
):
    submit(c, "box")
    command = dispatch(c)
    payload = deepcopy(c.observed["pinky1"])
    payload.update(
        sequence=payload["sequence"] + 1, active_command_id=command["command_id"]
    )
    payload["state"].update(node=command["payload"]["route"][index], stopped=False)
    c.observe({"source": "pinky1", "payload": payload})
    observe(c, "pinky2", node=other_node)
    transport = CaptureTransport()
    FleetRuntime(c, transport).poll(0)
    assert c.data["enabled"] is not held
    assert c.data["fault"] == ("route occupied: " + other_node if held else None)
    assert any(m["type"] == "robot_stop_command" for m in transport.sent) is held
    assert (
        any(m["payload"].get("keep_running_command_id") for m in transport.sent)
        is not held
    )


def test_fresh_known_position_releases_waiting_request(c):
    observe(c, "pinky2", node=None, mode="unknown", fault="line_nav_unavailable")
    submit(c, "box")
    assert dispatch(c) is None
    assert c.data["enabled"] and c.data["fault"] is None
    observe(c, "pinky2", node="N17", mode="line", fault=None)
    assert dispatch(c)["type"] == "line_route_command"


def test_confirmed_nav2_clears_old_line_occupancy_for_excluded_robot(c):
    observe(c, "pinky2", node="N18")
    c.assign("pinky2", None)
    observe(c, "pinky2", node=None, mode="nav2")
    assert c.data["last_nodes"]["pinky2"] is None
    c.observed.pop("pinky2")
    assert "N18" not in c.occupied("pinky1")


def test_full_shelf_waiter_resumes_after_cart_frees_compatible_slot(c):
    data = deepcopy(c.data)
    for slot, definition in SLOTS.items():
        if definition["size"] == "small":
            data["slots"][slot] = {"state": "occupied", "package_id": "stock-" + slot}
    c.save(data)
    incoming = submit(c, "new")
    finish(c)
    assert c.data["tasks"][incoming]["stage"] == "choose_slot"
    outgoing = submit(c, "stock-1a", "outbound")
    finish(c)
    assert c.data["tasks"][outgoing]["stage"] == "await_receipt"
    assert c.data["tasks"][incoming]["status"] == "COMPLETE"
    assert c.data["slots"]["1a"] == {"state": "occupied", "package_id": "new"}


def test_two_carts_can_wait_for_distinct_customers_without_double_booking(c):
    c.assign("pinky2", "smartcart")
    observe(c, "pinky2", node="H2")
    data = deepcopy(c.data)
    for slot in ("1a", "1c"):
        data["slots"][slot] = {"state": "occupied", "package_id": slot}
    c.save(data)
    keys = [submit(c, slot, "outbound") for slot in ("1a", "1c")]
    finish(c)
    tasks = [c.data["tasks"][key] for key in keys]
    assert all(t["stage"] == "await_receipt" for t in tasks)
    assert {t["robot"] for t in tasks} == {"pinky2", "pinky3"}
    assert {t["parking"] for t in tasks} == {"H1", "H2"}
    for key in reversed(keys):
        c.receipt(key)
        finish(c)
        assert c.data["tasks"][key]["status"] == "COMPLETE"
    assert {c.observed[r]["state"]["node"] for r in ("pinky2", "pinky3")} == {
        "H1",
        "H2",
    }


def test_participant_state_loss_holds_and_sends_stop(c):
    submit(c, "box")
    cmd = dispatch(c)
    c.heard[cmd["target"]] = -100
    transport = CaptureTransport()
    runtime = FleetRuntime(c, transport)
    runtime.last_poll = time.monotonic()
    runtime.poll(0)
    assert c.data["fault"] == "participant state stale"
    assert any(m["type"] == "robot_stop_command" for m in transport.sent)
    assert c.data["commands"] and c.data["resources"]
