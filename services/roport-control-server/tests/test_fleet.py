from copy import deepcopy
from uuid import uuid4

import pytest
from test_pinky_trial import (
    SERVICE,
    profile,  # noqa: F401
)

from roport_control_server.adapters.api.maps import load_map
from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.executor.fleet import SLOTS, TARGETS, FleetControl
from roport_control_server.executor.fleet_runtime import expiry


@pytest.fixture
def fleet(profile, tmp_path):  # noqa: F811
    pinky = SERVICE.parent / "roport-pinky"
    layout = load_map(
        pinky / "maps/airport_map.yaml",
        pinky / "src/roport_gz_world/layout/roport_airport.yaml",
    )["layout"]
    goal = {
        k: v
        for k, v in profile["nav_goal"].items()
        if k not in {"goal_version", "map_id"}
    }
    config = {
        **profile,
        "node_positions": {n["id"]: n["position"] for n in layout["nodes"]},
        "node_clearance_m": 0.35,
        "speed_limit_mps": 0.06,
        "goals": {"gate": goal},
        "n12_goal": goal,
        "omx": {
            r: {"model_version": "model-v1", "calibration_version": "cal-v1"}
            for r in TARGETS
            if not r.startswith("pinky")
        },
    }
    journal = Journal(tmp_path / "fleet.json")
    control = FleetControl(config, journal, clock=lambda: 1)
    for robot, node in [
        ("pinky1", "N16"),
        ("pinky2", "N17"),
        ("pinky3", "H1"),
        ("omx1", None),
        ("omx2", None),
        ("omx3", None),
        ("shelf-01", None),
    ]:
        state = {
            "stopped": True,
            "fault": None,
            "ready": True,
            "home_reached": True,
            "holding_state": "empty",
            "holding_package_id": None,
        }
        if node:
            state = {
                "node": node,
                "mode": "line",
                "stopped": True,
                "line_enabled": False,
                "fault": None,
                "heading_rad": 0,
                "arrival_command_id": None,
                "nav2_ready": True,
                "localization_ready": True,
                "drive_owner": "none",
                "map_version": config["map_version"],
                "graph_version": config["graph_version"],
            }
        control.observed[robot] = {
            "boot_version": robot + "-boot",
            "sequence": 1,
            "active_command_id": None,
            "state": state,
        }
        control.heard[robot] = 1
    control.enable()
    yield control
    journal.close()


def submit(c, package, operation="inbound"):
    request = {
        "request_id": operation + "-" + package,
        "operation": operation,
        "package_id": package,
        "goal": None if operation == "inbound" else "gate",
    }
    c.submit(request)
    return request["request_id"]


def dispatch(c):
    action = c.next_action()
    if not action:
        return None
    task, (target, kind, payload) = action
    message = wire.build_message(
        kind,
        c.config["control_id"],
        target,
        payload,
        command_id=str(uuid4()),
        expires_at=expiry(),
    )
    c.dispatched(task, message)
    return message


def complete(c, cmd, *, size="small", broken=None):
    task = c.data["tasks"][c.data["commands"][cmd["command_id"]]["task"]]
    kind, p = cmd["type"], cmd["payload"]
    result = {"stopped": True}
    observation = deepcopy(c.observed[task["robot"]])
    state = observation["state"]
    if kind == "line_route_command":
        result.update(node=p["goal"]["node"], mode="line")
        state.update(result, arrival_command_id=cmd["command_id"])
    elif kind == "set_drive_mode_command":
        result.update(node="N12", mode=p["to_mode"])
        state.update(result)
    elif kind == "nav_goal_command":
        result.update(mode="nav2", pose=p["pose"])
        state.update(node=None, mode="nav2")
    else:
        arms = (
            SLOTS[task["slot"]]["arms"]
            if kind != "warehouse_load_command"
            else ["omx3"]
        )
        result.update(
            package_id=task["request"]["package_id"],
            pinky_id=task["robot"],
            arrival_command_id=task["arrival"],
            placement_confirmed=True,
            release_confirmed=True,
            arms=[
                {
                    "robot_id": arm,
                    "boot_version": arm + "-boot",
                    "stopped": True,
                    "home_reached": True,
                    "holding_state": "empty",
                    "holding_package_id": None,
                }
                for arm in arms
            ],
        )
        if kind == "warehouse_load_command":
            result["package_size"] = size
        else:
            result["slot_id"] = task["slot"]
    if broken:
        result.update(broken)
    event = wire.build_message(
        "command_status",
        cmd["target"],
        cmd["source"],
        {
            "boot_version": p["expected_boot_version"],
            "observed_at": cmd["sent_at"],
            "sequence": 2,
            "status": "completed",
            "result": result,
        },
        command_id=cmd["command_id"],
    )
    c.result(event)
    if cmd["target"] == task["robot"]:
        observation.update(sequence=observation["sequence"] + 1, active_command_id=None)
        c.observe({"source": cmd["target"], "payload": observation})
    return event


def finish(c, *, size="small"):
    commands = []
    for _ in range(20):
        cmd = dispatch(c)
        if not cmd:
            break
        commands.append(cmd)
        complete(c, cmd, size=size)
    return commands


@pytest.mark.parametrize(
    "size,column,dock", [("large", "b", "N11"), ("small", "a", "N21")]
)
def test_inbound_outbound_return(fleet, size, column, dock):
    inbound = submit(fleet, "box")
    commands = finish(fleet, size=size)
    task = fleet.data["tasks"][inbound]
    assert task["status"] == "COMPLETE"
    assert task["slot"].endswith(column)
    assert fleet.data["slots"][task["slot"]]["state"] == "occupied"
    assert any(c["payload"].get("goal", {}).get("node") == dock for c in commands)
    assert any(c["payload"].get("departure") for c in commands)
    outbound = submit(fleet, "box", "outbound")
    commands = finish(fleet)
    assert fleet.data["tasks"][outbound]["stage"] == "await_receipt"
    assert not dispatch(fleet)
    fleet.receipt(outbound)
    commands += finish(fleet)
    assert fleet.data["tasks"][outbound]["status"] == "COMPLETE"
    assert fleet.observed["pinky3"]["state"]["node"] in {"H1", "H2"}
    assert any(c["payload"].get("from_mode") == "nav2" for c in commands)


def test_right_column_and_outbound_actual_slot(fleet):
    data = deepcopy(fleet.data)
    for key in ["1a", "2a"]:
        data["slots"][key] = {"state": "occupied", "package_id": key}
    fleet.save(data)
    inbound = submit(fleet, "right")
    finish(fleet)
    assert fleet.data["tasks"][inbound]["slot"] == "1c"
    outbound = submit(fleet, "right", "outbound")
    commands = finish(fleet)
    assert fleet.data["tasks"][outbound]["slot"] == "1c"
    assert any(
        c["target"] == "omx2" and c["type"] == "arm_action_command" for c in commands
    )


def test_unknown_size_waits_at_warehouse(fleet):
    key = submit(fleet, "unknown")
    finish(fleet, size="unknown")
    assert fleet.data["tasks"][key]["stage"] == "choose_slot"
    assert "unknown" in fleet.data["tasks"][key]["reason"]
    assert all(s["state"] == "empty" for s in fleet.data["slots"].values())


def test_duplicate_request_and_event(fleet):
    key = submit(fleet, "box")
    submit(fleet, "box")
    assert len(fleet.data["tasks"]) == 1
    event = complete(fleet, dispatch(fleet))
    before = deepcopy(fleet.data)
    assert fleet.result(event) and fleet.data == before
    with pytest.raises(ValueError, match="reused"):
        fleet.submit({**fleet.data["tasks"][key]["request"], "package_id": "different"})


def test_invalid_transfer_never_releases(fleet):
    submit(fleet, "box")
    complete(fleet, dispatch(fleet))
    cmd = dispatch(fleet)
    with pytest.raises(ValueError, match="placement"):
        complete(fleet, cmd, broken={"placement_confirmed": False})
    assert (
        next(iter(fleet.data["commands"].values()))["message"] == cmd
        and fleet.data["resources"]
    )


def test_late_success_after_stop(fleet):
    key = submit(fleet, "box")
    cmd = dispatch(fleet)
    fleet.hold("operator_stop")
    complete(fleet, cmd)
    assert fleet.data["tasks"][key]["stage"] == "warehouse_route"
    assert fleet.data["resources"] and not dispatch(fleet)


def test_busy_role_change_and_dynamic_parking(fleet):
    key = submit(fleet, "box")
    cmd = dispatch(fleet)
    with pytest.raises(ValueError, match="current robot"):
        fleet.assign(fleet.data["tasks"][key]["robot"], None)
    complete(fleet, cmd)
    finish(fleet)
    fleet.assign("pinky1", None)
    fleet.observed["pinky1"]["state"]["node"] = "N17"
    fleet.observed["pinky2"]["state"]["node"] = "N16"
    next_key = submit(fleet, "box2")
    finish(fleet)
    assert fleet.data["tasks"][next_key]["robot"] == "pinky2"
    assert fleet.data["tasks"][next_key]["parking"] == "N16"


def test_two_robots_use_disjoint_prefixes_and_undirected_edges(fleet):
    submit(fleet, "a")
    submit(fleet, "b")
    first = dispatch(fleet)
    second = dispatch(fleet)
    assert first["target"] != second["target"]
    assert not set(first["payload"]["route"]) & set(second["payload"]["route"])
    assert second["payload"]["goal"]["node"] != "N01"
    assert not dispatch(fleet)
    assert "edge:N14-N16" in fleet.data["resources"]
    assert "edge:N16-N14" not in fleet.data["resources"]


def test_restart_preserves_reservations(fleet):
    submit(fleet, "a")
    dispatch(fleet)
    old = deepcopy(fleet.data["resources"])
    recovered = FleetControl(fleet.config, fleet.store)
    assert recovered.data["resources"] == old
    assert "owner_restarted" in recovered.data["fault"]
    assert not recovered.next_action()


def test_customer_wait_does_not_block_inbound(fleet):
    submit(fleet, "a")
    finish(fleet)
    outgoing = submit(fleet, "a", "outbound")
    finish(fleet)
    assert fleet.data["tasks"][outgoing]["stage"] == "await_receipt"
    incoming = submit(fleet, "b")
    finish(fleet)
    assert fleet.data["tasks"][incoming]["status"] == "COMPLETE"


def test_unassigned_robot_still_occupies_last_known_bay_after_restart(fleet):
    fleet.observe(
        {
            "source": "pinky1",
            "payload": {
                **fleet.observed["pinky1"],
                "sequence": 2,
            },
        }
    )
    fleet.assign("pinky1", None)
    fleet.heard["pinky1"] = -100
    assert "N16" in fleet.occupied("pinky2")
    restarted = FleetControl(fleet.config, fleet.store, clock=lambda: 1)
    restarted.observed = {k: v for k, v in fleet.observed.items() if k != "pinky1"}
    restarted.heard = {k: v for k, v in fleet.heard.items() if k != "pinky1"}
    assert "N16" in restarted.occupied("pinky2")
    assert not restarted.data["enabled"]


def test_unarmed_omx_waits_without_dispatching(fleet):
    key = submit(fleet, "box")
    complete(fleet, dispatch(fleet))
    fleet.observed["omx3"]["state"]["ready"] = False
    assert dispatch(fleet) is None
    assert fleet.data["tasks"][key]["stage"] == "warehouse_load"
    assert fleet.data["fault"] is None


def test_cooperative_member_restart_holds_and_retains_resources(fleet):
    key = submit(fleet, "large")
    for _ in range(3):
        complete(fleet, dispatch(fleet), size="large")
    cmd = dispatch(fleet)
    assert cmd["target"] == "shelf-01"
    fleet.observe(
        {
            "source": "omx2",
            "payload": {
                **fleet.observed["omx2"],
                "boot_version": "restarted",
                "sequence": 1,
            },
        }
    )
    assert "restarted" in fleet.data["fault"]
    assert fleet.data["tasks"][key]["stage"] == "transfer"
    assert fleet.data["resources"]


def test_restart_between_commands_does_not_auto_resume_changed_robot(fleet):
    key = submit(fleet, "box")
    complete(fleet, dispatch(fleet))
    restarted = FleetControl(fleet.config, fleet.store, clock=lambda: 1)
    assert not restarted.data["enabled"]
    restarted.observe(
        {
            "source": "pinky1",
            "payload": {
                **fleet.observed["pinky1"],
                "boot_version": "restarted",
                "sequence": 1,
            },
        }
    )
    assert "restarted" in restarted.data["fault"]
    assert restarted.data["tasks"][key]["stage"] == "warehouse_load"
    with pytest.raises(ValueError, match="reconcile"):
        restarted.enable()


def test_map_change_does_not_reuse_stored_tasks(fleet):
    restarted = FleetControl({**fleet.config, "map_version": "new-map"}, fleet.store)
    assert "field changed" in restarted.data["fault"]
    assert not restarted.data["enabled"]


def test_absent_slot_can_be_excluded_before_first_start(fleet):
    data = deepcopy(fleet.data)
    data["enabled"] = False
    fleet.save(data)
    fleet.observed.pop("pinky2")
    fleet.heard.pop("pinky2")
    fleet.assign("pinky2", None)
    assert fleet.data["roles"]["pinky2"] is None
    with pytest.raises(ValueError, match="unavailable"):
        fleet.assign("pinky2", "logistics")


def test_disconnect_during_state_query_preserves_active_task(fleet):
    from queue import Empty

    from roport_control_server.executor.fleet_runtime import FleetRuntime

    submit(fleet, "box")
    dispatch(fleet)

    class LostConnection:
        ready = True

        def receive(self, timeout):
            raise Empty

        def publish(self, *args):
            raise ConnectionError("lost between readiness check and publish")

    FleetRuntime(fleet, LostConnection()).poll()
    assert fleet.data["fault"] == "mqtt_delivery_outcome_unknown"
    assert fleet.data["commands"] and fleet.data["resources"]


def test_unknown_heading_waits_instead_of_building_invalid_command(fleet):
    key = submit(fleet, "box")
    fleet.observed["pinky1"]["state"]["heading_rad"] = None
    fleet.observed["pinky2"]["state"]["heading_rad"] = None
    assert dispatch(fleet) is None
    assert "heading unavailable" in fleet.data["tasks"][key]["reason"]
    assert fleet.data["fault"] is None
