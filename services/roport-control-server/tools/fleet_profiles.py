"""Profiles for the controlled, loopback Fleet simulation."""

import math
from copy import deepcopy
from pathlib import Path

from prepare_pinky_trial import prepare
from run_virtual_trial import LAYOUT, MAP, VIRTUAL_SPEED_MPS

from roport_control_server.adapters.api.maps import load_map
from roport_control_server.executor.plans import shortest_route


def profiles(*, map_path=MAP, layout_path=LAYOUT, simulation=True):
    trial = prepare(map_path, layout_path, simulation=simulation)
    speed = VIRTUAL_SPEED_MPS if simulation else 0.06
    for route in trial["routes"].values():
        route["speed_limit_mps"] = speed
    grid = load_map(map_path, layout_path)
    positions = {n["id"]: n["position"] for n in grid["layout"]["nodes"]}
    # The IR bar is 3 cm in front of the body; use the same graph return calibration.
    import yaml

    layout = yaml.safe_load(Path(layout_path).read_text())
    yaw = math.radians(layout["frame"]["yaw_deg"])
    x, y = positions["N12"]
    goal = {
        k: v
        for k, v in trial["nav_goal"].items()
        if k not in {"goal_version", "map_id"}
    }
    back = {
        **goal,
        "goal_id": "N12_RETURN",
        "pose": {
            "x": x + 0.03 * math.cos(yaw),
            "y": y + 0.03 * math.sin(yaw),
            "yaw": math.atan2(math.sin(math.pi + yaw), math.cos(math.pi + yaw)),
        },
        "position_tolerance_m": 0.03,
        "yaw_tolerance_rad": 0.08,
    }
    fleet = {
        **trial,
        "node_positions": positions,
        # Two padded turning footprints + front IR and node-detection allowance.
        "node_clearance_m": 0.35,
        "speed_limit_mps": speed,
        "goals": {"PUBLIC_SPACE_TRIAL": goal},
        "n12_goal": back,
        "omx": {
            r: {"model_version": "sim-model", "calibration_version": "sim-cal"}
            for r in ["omx1", "omx2", "omx3", "shelf-01"]
        },
    }
    if not simulation:
        # Neither simulated clearance nor model names are field calibration.
        fleet.pop("node_clearance_m")
        fleet["omx"] = {
            r: {"model_version": None, "calibration_version": None}
            for r in fleet["omx"]
        }
    robots = {}
    for robot, role, start in [
        ("pinky1", "logistics", "N16"),
        ("pinky2", "logistics", "N17"),
        ("pinky3", "smartcart", "H1"),
    ]:
        p = deepcopy(trial)
        p["robots"][role]["robot_id"] = robot
        if robot == "pinky2":
            route = p["routes"]["to_loading"]
            route.update(
                start_node=start, route=shortest_route(p["nodes"], start, "N01")
            )
        robots[robot] = (role, p)
    return fleet, robots
