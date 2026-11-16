"""Concurrent command boundaries using real dispatch and explicit robot evidence."""

from copy import deepcopy

import pytest
from test_fleet import complete, dispatch, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import MemoryTransport
from test_pinky_trial import profile as profile  # noqa: PLC0414

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import Delivery
from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.fleet_runtime import FleetRuntime


def two_routes(control):
    submit(control, "a")
    submit(control, "b")
    commands = [dispatch(control), dispatch(control)]
    assert all(commands)
    assert {c["target"] for c in commands} == {"pinky1", "pinky2"}
    assert set(control.data["commands"]) == {c["command_id"] for c in commands}
    return commands


def progress(command, node, sequence, *, boot=None):
    return wire.build_message(
        "command_status",
        command["target"],
        command["source"],
        {
            "boot_version": boot or command["payload"]["expected_boot_version"],
            "observed_at": command["sent_at"],
            "sequence": sequence,
            "status": "in_progress",
            "result": {"node": node, "mode": "line", "stopped": False},
        },
        command_id=command["command_id"],
    )


@pytest.mark.parametrize("first_finished", [0, 1])
def test_completion_updates_only_its_command_and_keeps_other_reservations(
    fleet, first_finished
):
    commands = two_routes(fleet)
    finished, other = commands[first_finished], commands[1 - first_finished]
    before = deepcopy(fleet.data)
    finished_task = before["commands"][finished["command_id"]]["task"]
    other_task = before["commands"][other["command_id"]]["task"]
    event = complete(fleet, finished)
    assert set(fleet.data["commands"]) == {other["command_id"]}
    assert (
        fleet.data["commands"][other["command_id"]]
        == before["commands"][other["command_id"]]
    )
    assert fleet.data["tasks"][other_task] == before["tasks"][other_task]
    assert fleet.data["resources"] == {
        key: owner
        for key, owner in before["resources"].items()
        if owner != finished_task
    }
    assert fleet.data["resources"]
    after = deepcopy(fleet.data)
    assert fleet.result(event) and fleet.data == after
    complete(fleet, other)
    assert fleet.data["commands"] == {} and fleet.data["resources"] == {}


def test_hold_stops_every_active_command_without_renewing_any_lease(fleet):
    commands = two_routes(fleet)
    reservations = deepcopy(fleet.data["resources"])
    fleet.hold("operator_stop")
    transport = MemoryTransport()
    runtime = FleetRuntime(fleet, transport)
    runtime.poll(0)
    stops = [m for _, m in transport.sent if m["type"] == "robot_stop_command"]
    assert {m["payload"]["target_command_id"] for m in stops} == {
        c["command_id"] for c in commands
    }
    assert len(stops) == 2 and {m["target"] for m in stops} == {"pinky1", "pinky2"}
    assert fleet.data["resources"] == reservations
    assert all(t["status"] == "HOLD" for t in fleet.data["tasks"].values())
    assert all("stop" in c for c in fleet.data["commands"].values())
    runtime.last_poll = -float("inf")
    runtime.poll(0)
    assert len([m for _, m in transport.sent if m["type"] == "robot_stop_command"]) == 2
    queries = [m for _, m in transport.sent if m["type"] == "robot_state_request"]
    assert queries and all(
        "keep_running_command_id" not in m["payload"] for m in queries
    )
    assert not any(
        m["type"] in wire.COMMANDS and m["type"] != "robot_stop_command"
        for _, m in transport.sent
    )


def test_owner_restart_retains_all_active_commands_and_reservations_on_hold(fleet):
    two_routes(fleet)
    before = deepcopy(fleet.data)
    restarted = FleetControl(fleet.config, fleet.store, clock=lambda: 1)
    assert restarted.data["commands"] == before["commands"]
    assert restarted.data["resources"] == before["resources"]
    assert all(t["status"] == "HOLD" for t in restarted.data["tasks"].values())
    assert (
        not restarted.data["enabled"] and "owner_restarted" in restarted.data["fault"]
    )
    assert restarted.next_action() is None
    with pytest.raises(ValueError, match="reconcile"):
        restarted.enable()


def test_front_ir_progress_retains_incoming_segment_until_next_node(fleet):
    submit(fleet, "box")
    command = dispatch(fleet)
    route = command["payload"]["route"]
    first_edge = fleet.edge(route[0], route[1])
    incoming_edge = fleet.edge(route[1], route[2])
    assert fleet.result(progress(command, route[1], 1))
    assert first_edge in fleet.data["resources"]
    assert "node:" + route[0] in fleet.data["resources"]
    assert fleet.result(progress(command, route[2], 2))
    assert first_edge not in fleet.data["resources"]
    assert "node:" + route[0] in fleet.data["resources"]
    assert incoming_edge in fleet.data["resources"]
    assert "node:" + route[1] in fleet.data["resources"]
    assert fleet.result(progress(command, route[3], 3))
    assert "node:" + route[0] not in fleet.data["resources"]
    assert incoming_edge not in fleet.data["resources"]
    assert "robot:" + command["target"] in fleet.data["resources"]


def test_delayed_progress_is_stored_and_acked_without_regressing_position(fleet):
    submit(fleet, "box")
    command = dispatch(fleet)
    robot, cid = command["target"], command["command_id"]
    route = command["payload"]["route"]
    state = deepcopy(fleet.observed[robot])
    state.update(sequence=2, active_command_id=cid)
    state["state"].update(node=route[3], stopped=False)
    fleet.observe({"source": robot, "payload": state})
    before = deepcopy(fleet.data)

    class StoredAckTransport(MemoryTransport):
        def publish(self, destination, message):
            if message["type"] == "event_stored":
                assert message["payload"]["acked_message_id"] in fleet.data["events"]
            super().publish(destination, message)

    transport = StoredAckTransport()
    runtime = FleetRuntime(fleet, transport)
    delayed = progress(command, route[2], 1)
    runtime.handle(
        Delivery(wire.topic(robot, "event"), wire.encode(delayed), False, 1, 1, 0)
    )
    assert fleet.data["events"][delayed["message_id"]] == delayed
    assert transport.sent[-1][1]["payload"]["acked_message_id"] == delayed["message_id"]
    assert fleet.observed[robot]["state"]["node"] == route[3]
    assert fleet.data["last_nodes"][robot] == route[3]
    assert fleet.data["commands"][cid]["route_index"] == 3
    assert fleet.data["commands"][cid]["sequence"] == 1
    assert fleet.data["resources"] == before["resources"]
    assert fleet.data["tasks"] == before["tasks"]

    newer = progress(command, route[4], 2)
    runtime.handle(
        Delivery(wire.topic(robot, "event"), wire.encode(newer), False, 2, 1, 0)
    )
    assert fleet.observed[robot]["state"]["node"] == route[4]
    assert fleet.data["last_nodes"][robot] == route[4]
    assert fleet.data["commands"][cid]["route_index"] == 4
    assert fleet.data["resources"] != before["resources"]
    assert "node:" + route[2] not in fleet.data["resources"]
    assert "node:" + route[3] not in fleet.data["resources"]
    assert fleet.edge(route[3], route[4]) in fleet.data["resources"]
    assert fleet.data["tasks"] == before["tasks"]
    assert transport.sent[-1][1]["payload"]["acked_message_id"] == newer["message_id"]


@pytest.mark.parametrize("channel", ["command_status", "robot_state"])
def test_stale_sequence_and_changed_boot_cannot_release_route_resources(fleet, channel):
    submit(fleet, "box")
    command = dispatch(fleet)
    route = command["payload"]["route"]
    if channel == "command_status":
        assert fleet.result(progress(command, route[1], 2))
        before = deepcopy(fleet.data)
        assert not fleet.result(progress(command, route[3], 1))
        assert fleet.data == before
        with pytest.raises(ValueError, match="boot"):
            fleet.result(progress(command, route[3], 3, boot="different-boot"))
        assert fleet.data == before
    else:
        state = deepcopy(fleet.observed[command["target"]])
        state.update(sequence=2, active_command_id=command["command_id"])
        state["state"].update(node=route[1], stopped=False)
        fleet.observe({"source": command["target"], "payload": state})
        before = deepcopy(fleet.data)
        state.update(sequence=1)
        state["state"]["node"] = route[3]
        fleet.observe({"source": command["target"], "payload": state})
        assert fleet.data == before
        state.update(sequence=3, boot_version="different-boot")
        fleet.observe({"source": command["target"], "payload": state})
        assert fleet.data["resources"] == before["resources"]
        assert fleet.data["commands"] == before["commands"]
        assert not fleet.data["enabled"] and "restarted" in fleet.data["fault"]


def test_partial_route_completion_does_not_start_omx_before_final_arrival(fleet):
    # A stationary robot blocks the full route; only a safe prefix may be issued.
    fleet.observed["pinky2"]["state"]["node"] = "N01"
    key = submit(fleet, "box")
    partial = dispatch(fleet)
    assert partial["type"] == "line_route_command"
    assert partial["payload"]["goal"]["node"] != "N01"
    complete(fleet, partial)
    assert fleet.data["tasks"][key]["stage"] == "warehouse_route"
    assert "arrival" not in fleet.data["tasks"][key]
    assert dispatch(fleet) is None
    # Explicit new position evidence clears the blocked segment for this boundary test.
    fleet.observed["pinky2"]["state"]["node"] = "N17"
    remaining = dispatch(fleet)
    assert remaining["type"] == "line_route_command"
    assert remaining["payload"]["start_node"] == partial["payload"]["goal"]["node"]
    assert remaining["payload"]["goal"]["node"] == "N01"
    assert fleet.data["tasks"][key]["stage"] == "warehouse_route"
    complete(fleet, remaining)
    assert fleet.data["tasks"][key]["stage"] == "warehouse_load"
    loading = dispatch(fleet)
    assert loading["type"] == "warehouse_load_command" and loading["target"] == "omx3"
    assert loading["payload"]["arrival_command_id"] == remaining["command_id"]


def test_completed_route_waits_for_fresh_state_before_using_new_heading(fleet):
    fleet.observed["pinky2"]["state"]["node"] = "N01"
    key = submit(fleet, "box")
    command = dispatch(fleet)
    robot = command["target"]
    node = command["payload"]["goal"]["node"]
    assert node != "N01"
    old = deepcopy(fleet.observed[robot])
    event = progress(command, node, 1)
    event["payload"]["status"] = "completed"
    event["payload"]["result"].update(stopped=True, heading_rad=1.5725)
    assert fleet.result(event)
    assert fleet.data["last_nodes"][robot] == node
    assert fleet.observed[robot]["state"]["node"] == node
    assert fleet.data["tasks"][key]["stage"] == "warehouse_route"

    fleet.observed["pinky2"]["state"]["node"] = "N17"
    assert dispatch(fleet) is None
    assert "awaiting state" in fleet.data["tasks"][key]["reason"]
    fleet.observe({"source": robot, "payload": old})
    assert dispatch(fleet) is None

    fresh = deepcopy(old)
    fresh.update(sequence=old["sequence"] + 1, active_command_id=command["command_id"])
    fresh["state"].update(node=node, stopped=False, heading_rad=1.5725)
    fleet.observe({"source": robot, "payload": fresh})
    assert dispatch(fleet) is None
    fresh.update(sequence=fresh["sequence"] + 1, active_command_id=None)
    fresh["state"].update(stopped=True, arrival_command_id=command["command_id"])
    fleet.observe({"source": robot, "payload": fresh})
    assert fleet.result(event)  # A replayed completion must not wait again.
    remaining = dispatch(fleet)
    assert remaining["payload"]["start_node"] == node
    assert remaining["payload"]["start_heading_rad"] == 1.5725
    assert remaining["payload"]["goal"]["node"] == "N01"


def test_queue_cannot_block_loaded_robot_leaving_warehouse(fleet):
    first = submit(fleet, "first")
    complete(fleet, dispatch(fleet))
    complete(fleet, dispatch(fleet))
    data = deepcopy(fleet.data)
    fleet.select_slot(data, data["tasks"][first])
    fleet.save(data)
    leader = fleet.data["tasks"][first]
    assert leader["stage"] == "shelf_route"
    assert fleet.observed[leader["robot"]]["state"]["node"] == "N01"

    second = submit(fleet, "second")
    data = deepcopy(fleet.data)
    fleet.allocate(data, data["tasks"][second])
    fleet.save(data)
    follower = fleet.data["tasks"][second]
    assert not fleet.waiting_node_clear("N18", follower)
    assert fleet.waiting_node_clear("N05", follower)
    path = fleet.route_prefix(follower, "N17", "N01")
    assert path[-1] == "N05"

    fleet.observed[leader["robot"]]["state"]["node"] = "N21"
    assert fleet.route_prefix(follower, path[-1], "N01")[-1] == "N01"


@pytest.mark.parametrize(
    "size,dock", [("small", "N21"), ("small", "N22"), ("large", "N11")]
)
@pytest.mark.parametrize("channel", ["command_status", "robot_state"])
def test_follower_starts_on_clearance_before_leader_command_completes(
    fleet, size, dock, channel
):
    first = submit(fleet, "first")
    complete(fleet, dispatch(fleet))
    complete(fleet, dispatch(fleet), size=size)
    if dock == "N22":
        for slot in ("1a", "2a"):
            fleet.data["slots"][slot] = {"state": "occupied", "package_id": slot}
    leader = dispatch(fleet)
    assert leader["payload"]["goal"]["node"] == dock
    second = submit(fleet, "second")
    waiting = dispatch(fleet)
    assert waiting["target"] == "pinky2"
    complete(fleet, waiting)
    assert dispatch(fleet) is None

    assert fleet.result(progress(leader, "N07", 1))
    assert "node:N07" in fleet.data["resources"]
    assert dispatch(fleet) is None
    if channel == "command_status":
        assert fleet.result(progress(leader, "N23", 2))
    else:
        state = deepcopy(fleet.observed[leader["target"]])
        state.update(
            sequence=state["sequence"] + 1, active_command_id=leader["command_id"]
        )
        state["state"].update(node="N23", stopped=False)
        fleet.observe({"source": leader["target"], "payload": state})
    # The long incoming edge remains protected, but its distant start is clear.
    assert "node:N07" not in fleet.data["resources"]
    assert fleet.edge("N07", "N23") in fleet.data["resources"]
    following = dispatch(fleet)
    assert following["target"] == "pinky2"
    assert following["payload"]["goal"]["node"] == "N01"
    assert following["payload"]["route"] == ["N05", "N07", "N18", "N01"]
    assert leader["command_id"] in fleet.data["commands"]
    assert following["command_id"] in fleet.data["commands"]
    assert fleet.data["tasks"][first]["stage"] == "shelf_route"
    assert fleet.data["tasks"][second]["stage"] == "warehouse_route"
    assert fleet.result(progress(following, "N07", 1))
    fleet.check_active()
    assert fleet.data["fault"] is None
    assert leader["command_id"] in fleet.data["commands"]


def test_idle_robot_obstruction_still_allows_a_real_detour(fleet):
    fleet.observed["pinky1"]["state"]["node"] = "N05"
    fleet.observed["pinky2"]["state"]["node"] = "N07"
    task = {"robot": "pinky1", "stage": "warehouse_route", "role": "logistics"}
    assert fleet.route_prefix(task, "N05", "N01") == [
        "N05",
        "N15",
        "N14",
        "N02",
        "N10",
        "N18",
        "N01",
    ]


def test_reserved_edge_still_blocks_the_short_route(fleet):
    fleet.observed["pinky1"]["state"]["node"] = "N05"
    fleet.data["resources"][fleet.edge("N05", "N07")] = "other-request"
    task = {"robot": "pinky1", "stage": "warehouse_route", "role": "logistics"}
    with pytest.raises(ValueError, match="route clearance"):
        fleet.route_prefix(task, "N05", "N01")


@pytest.mark.parametrize(
    "clearance,release", [(None, False), (0.6, False), (0.35, True)]
)
def test_node_release_requires_calibrated_distance_and_keeps_incoming_edge(
    fleet, clearance, release
):
    fleet.config["node_clearance_m"] = clearance
    submit(fleet, "box")
    command = dispatch(fleet)
    assert fleet.result(progress(command, "N18", 1))
    assert ("node:N10" not in fleet.data["resources"]) is release
    assert fleet.edge("N10", "N18") in fleet.data["resources"]
    assert "node:N18" in fleet.data["resources"]
    if release:
        fleet.observed["pinky2"]["state"]["node"] = "N10"
        fleet.check_active()
        assert fleet.data["fault"] is None
    fleet.observed["pinky2"]["state"]["node"] = "N18"
    fleet.check_active()
    assert fleet.data["fault"] == "route occupied: N18"


@pytest.mark.parametrize(
    "update",
    [
        {"node_clearance_m": 0},
        {"node_clearance_m": -1},
        {"node_clearance_m": float("nan")},
        {"node_clearance_m": float("inf")},
        {"node_clearance_m": True},
        {"node_positions": {}},
        {"node_positions": []},
    ],
)
def test_invalid_clearance_calibration_is_rejected_before_store_changes(fleet, update):
    before = deepcopy(fleet.data)
    with pytest.raises(ValueError, match="node_"):
        FleetControl({**fleet.config, **update}, fleet.store)
    assert fleet.data == before
