"""Generate an editable trial profile from the repository's map and measured layout."""

import argparse
import hashlib
import itertools
import json
import math
import re
from pathlib import Path
from uuid import uuid4

import yaml

from roport_control_server.adapters.api.maps import load_map
from roport_control_server.adapters.mqtt.trial_wire import PROFILE

PATHS = {
    "to_loading": ["N16", "N14", "N02", "N10", "N18", "N01"],
    "to_delivery": ["N01", "N18", "N07", "N23", "N19", "N11"],
    "return_warehouse": ["N11", "N19", "N23", "N07", "N05", "N15", "N14", "N16"],
    "cart_to_delivery": ["H1", "N09", "N24", "N19", "N11"],
    "cart_to_switch": ["N11", "N19", "N24", "N09", "N08", "N12"],
}
SINGLE_PATHS = {
    "to_delivery": ["N16", "N14", "N02", "N10", "N18", "N07", "N23", "N19", "N11"],
    "cart_to_switch": PATHS["cart_to_switch"],
}


def trial_inputs(map_path, layout_path):
    grid = load_map(map_path)
    map_path, layout_path = Path(map_path), Path(layout_path)
    if layout_path.stat().st_size > 64 * 1024:
        raise ValueError("layout YAML exceeds 64 KiB")
    layout = yaml.safe_load(layout_path.read_bytes())

    def number(value, low=-100000, high=100000):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("layout requires finite numbers")
        if not low <= value <= high:
            raise ValueError("layout value out of range")
        return value

    def name(value):
        if not isinstance(value, str) or not re.fullmatch(
            r"[A-Za-z0-9_.-]{1,64}", value
        ):
            raise ValueError("invalid layout identifier")
        return value

    try:
        if layout["units"] != "cm" or layout["version"] != "v14":
            raise ValueError(
                "review trial routes and goal for this layout version/units"
            )
        meta = layout["map"]
        if (
            any(meta[k] != grid[k] for k in ("width", "height", "resolution"))
            or meta["origin"] != grid["origin"][:2]
            or grid["origin"][2] != 0
        ):
            raise ValueError("layout and map grids do not match")
        frame = layout["frame"]
        tx, ty = number(frame["x_m"]), number(frame["y_m"])
        yaw = math.radians(number(frame["yaw_deg"], -180, 180))
        items = layout["nodes"]["items"]
        name(layout["nodes"]["graph_version"])
        if not isinstance(items, list) or not 1 <= len(items) <= 256:
            raise ValueError("layout must have 1..256 nodes")
        nodes = {}
        for item in items:
            node = name(item["id"])
            if node in nodes:
                raise ValueError("duplicate layout node")
            x, y = [number(v) / 100 for v in item["at"]]
            x, y = (
                tx + math.cos(yaw) * x - math.sin(yaw) * y,
                ty + math.sin(yaw) * x + math.cos(yaw) * y,
            )
            if not all(
                grid["origin"][i]
                <= v
                <= grid["origin"][i] + grid[size] * grid["resolution"]
                for i, v, size in ((0, x, "width"), (1, y, "height"))
            ):
                raise ValueError("layout node outside map")
            links = item["links"]
            if not isinstance(links, list) or len(links) > 256:
                raise ValueError("invalid layout links")
            nodes[node] = {"links": [name(link) for link in links]}
        for node, data in nodes.items():
            if any(
                target == node
                or target not in nodes
                or node not in nodes[target]["links"]
                for target in data["links"]
            ):
                raise ValueError(
                    "layout links must be symmetric and reference known nodes"
                )
    except (KeyError, TypeError) as exc:
        raise ValueError("invalid measured layout") from exc
    image = yaml.safe_load(map_path.read_bytes())["image"]
    grid["version"] = hashlib.sha256(
        map_path.read_bytes() + (map_path.parent / image).read_bytes()
    ).hexdigest()[:16]
    return grid, layout, nodes


def prepare(map_path, layout_path, *, simulation=False, single_robot=False):
    grid, layout, nodes = trial_inputs(map_path, layout_path)
    graph_version = layout["nodes"]["graph_version"]
    paths = SINGLE_PATHS if single_robot else PATHS
    # These aliases and route choices belong to this trial, not the map authority.
    routes = {
        name: {
            "graph_version": graph_version,
            "start_node": path[0],
            "start_heading_rad": 0.0 if simulation else None,
            "start_maneuver": "none" if simulation else "REPLACE_ME",
            "route": list(path),
            "goal": {"node_id": path[-1], "location_id": None, "on_arrival": "hold"},
            "speed_limit_mps": 0.06,
        }
        for name, path in paths.items()
    }
    for p in paths.values():
        if any(
            b not in nodes.get(a, {}).get("links", []) for a, b in itertools.pairwise(p)
        ):
            raise ValueError(
                "candidate route does not match current map; edit PATHS first"
            )
    if single_robot:
        # Pinky team confirmed the N16 start pose; control selects the speed cap.
        routes["to_delivery"].update(
            start_heading_rad=-1.5690,
            start_maneuver="none",
            speed_limit_mps=0.06,
        )
    frame = layout["frame"]
    yaw = math.radians(frame["yaw_deg"])
    if single_robot:
        # N11 loading faces east after the driver's arrival turn; validate on site.
        routes["cart_to_switch"].update(
            start_heading_rad=yaw, start_maneuver="none", speed_limit_mps=0.06
        )
    else:
        # N01/N11 report the loading pose BEFORE load_done's second turn.
        headings = {
            "to_loading": -1.5690,
            "to_delivery": yaw,
            "return_warehouse": yaw,
            "cart_to_delivery": math.pi / 2 + yaw,
            "cart_to_switch": yaw,
        }
        for step, route in routes.items():
            route.update(
                start_heading_rad=headings[step],
                start_maneuver="none",
                speed_limit_mps=0.06,
            )
    # Both trials use this v14 candidate; field verification remains mandatory.
    x, y = 4.10, 1.90
    goal_pose = {
        "x": round(frame["x_m"] + math.cos(yaw) * x - math.sin(yaw) * y, 6),
        "y": round(frame["y_m"] + math.sin(yaw) * x + math.cos(yaw) * y, 6),
        "yaw": yaw,
    }
    roles = (("logistics", "pinky1"),)
    if not single_robot:
        roles += (("smartcart", "pinky3"),)
    return {
        "profile": PROFILE,
        "scenario": "single_pinky" if single_robot else "two_pinky",
        "mode": "simulation" if simulation else "hardware",
        "field_verified": False,
        "site_id": "pinky-trial",
        "control_id": "control-trial",
        "package_id": "trial-package-1",
        "graph_version": graph_version,
        "map_id": grid["id"],
        "map_version": grid["version"],
        "nodes": {name: n["links"] for name, n in nodes.items()},
        "routes": routes,
        "robots": {
            role: {"robot_id": robot, "binding_id": str(uuid4())}
            for role, robot in roles
        },
        "command_timeout_s": 300,
        "nav_goal": {
            "goal_id": "PUBLIC_SPACE_TRIAL",
            "goal_version": "trial-goal-v1",
            "map_id": grid["id"],
            "map_version": grid["version"],
            "frame_id": "map",
            "pose": goal_pose,
            "position_tolerance_m": 0.1,
            "yaw_tolerance_rad": 0.15,
        },
    }


def prepare_bundle(output, map_path, layout_path):
    """Generate once, then copy the same immutable identities to both endpoints."""
    output = Path(output)
    profiles = {}
    for scenario in ("single", "two"):
        for mode in ("simulation", "hardware"):
            config = prepare(
                map_path,
                layout_path,
                simulation=mode == "simulation",
                single_robot=scenario == "single",
            )
            identity = f"{scenario}-{mode}-{uuid4().hex[:8]}"
            config.update(site_id=f"trial-{identity}", control_id=f"control-{identity}")
            profiles[f"{scenario}/{mode}"] = config
    output.mkdir(parents=True, exist_ok=False)
    for name, config in profiles.items():
        folder = output / name
        folder.mkdir(parents=True)
        (folder / "trial.json").write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n"
        )
        simulation = config["mode"] == "simulation"
        (folder / "broker.env.example").write_text(
            "# Copy to broker.env and fill it once; use matching broker settings on all PCs.\n"
            f"export ROPORT_MQTT_HOST={'127.0.0.1' if simulation else 'REPLACE_WITH_BROKER_LAN_HOST'}\n"
            "export ROPORT_MQTT_PORT=18883\n"
            "export ROPORT_MQTT_TLS=0\n"
            "# Credentials stay outside this bundle; password authentication requires TLS.\n"
        )
        roles = ", ".join(
            f"{role}={r['robot_id']}" for role, r in config["robots"].items()
        )
        (folder / "CONNECT.md").write_text(
            f"# {name}\n\n"
            f"- 역할: {roles}\n"
            f"- site_id: `{config['site_id']}`\n"
            f"- control_id: `{config['control_id']}`\n"
            "- 이 trial.json을 관제와 해당 핑키에 **그대로 복사**합니다. 각 PC에서 다시 생성하지 않습니다.\n"
            "- journal은 역할별 별도 영속 파일을 사용합니다. 기존 이력은 덮어쓰지 않습니다.\n"
            "- 가상 시험: 같은 PC의 loopback 브로커 + mock_pinky_trial.py를 역할마다 한 번 실행합니다.\n"
            "- 실물 시험: 가상 핑키를 실행하지 않고 공통 LAN 브로커 + pinky_mqtt.trial_runner를 사용합니다.\n"
            "- hardware는 출발 방향/동작·골·독립 watchdog을 현장 검증한 후 field_verified를 true로 바꾸고 --hardware-authorized를 명시합니다.\n"
            "- 물리 ROS 연결은 팀원 driver factory가 담당합니다. 실행 절차는 ZIP 최상단 README의 7절을 따릅니다.\n"
        )
    return profiles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--layout", required=True)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--output")
    destination.add_argument("--bundle-dir")
    parser.add_argument("--simulation", action="store_true")
    parser.add_argument("--single-robot", action="store_true")
    args = parser.parse_args()
    if args.bundle_dir:
        if args.simulation or args.single_robot:
            parser.error("--bundle-dir creates both scenarios and both modes")
        prepare_bundle(args.bundle_dir, args.map, args.layout)
        print(
            "Created single/two × simulation/hardware profiles; hardware needs field checks"
        )
        return
    profile = prepare(
        args.map,
        args.layout,
        simulation=args.simulation,
        single_robot=args.single_robot,
    )
    with Path(args.output).open("x") as f:
        json.dump(profile, f, ensure_ascii=False, indent=2)
    print(
        "Created simulation profile"
        if args.simulation
        else "Created hardware candidates: field_verified=false; verify headings, manoeuvres, goal and watchdog on site"
    )


if __name__ == "__main__":
    main()
