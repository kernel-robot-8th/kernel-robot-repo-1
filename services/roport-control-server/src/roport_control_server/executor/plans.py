"""Bounded sequential plans; shared by customer-request and segment-test inputs."""

import math
import re
from collections import deque
from copy import deepcopy
from itertools import pairwise

from roport_control_server.adapters.mqtt.trial_wire import command

MOTION = {"route", "mode_switch", "navigate"}
KINDS = MOTION | {"gate", "request"}


def stop_nodes(config):
    # Current v14 stops are terminal nodes; compatibility test checks the team layout.
    return {node for node, links in config["nodes"].items() if len(links) == 1}


def switch_payload(config):
    return {
        "node_id": "N12",
        "from_mode": "line",
        "to_mode": "nav2",
        "map_id": config["map_id"],
        "map_version": config["map_version"],
    }


def legacy_plan(config, scope="nav2", smartcart_only=False):
    single = config["scenario"] == "single_pinky"
    cart = "logistics" if single else "smartcart"
    steps = []

    def motion(step, kind, role, payload):
        steps.append(
            {
                "id": step,
                "kind": kind,
                "role": role,
                "label": step,
                "payload": deepcopy(payload),
            }
        )

    def gate(step, role, node, kind="gate"):
        steps.append(
            {
                "id": step,
                "kind": kind,
                "role": role,
                "label": step,
                "payload": {
                    "node_id": node,
                    "package_id": config["package_id"],
                    "quantity": 1,
                },
            }
        )

    if not smartcart_only:
        if not single:
            motion("to_loading", "route", "logistics", config["routes"]["to_loading"])
            gate(
                "WAIT_LOAD",
                "logistics",
                config["routes"]["to_loading"]["goal"]["node_id"],
            )
        motion("to_delivery", "route", "logistics", config["routes"]["to_delivery"])
        gate("WAIT_INBOUND", "logistics", "N11")
        if not single:
            motion(
                "return_warehouse",
                "route",
                "logistics",
                config["routes"]["return_warehouse"],
            )
            gate(
                "WAIT_REQUEST",
                "logistics",
                config["routes"]["return_warehouse"]["goal"]["node_id"],
                "request",
            )
    if not single:
        motion("cart_to_delivery", "route", cart, config["routes"]["cart_to_delivery"])
    gate("WAIT_CART_LOAD", cart, "N11")
    motion("cart_to_switch", "route", cart, config["routes"]["cart_to_switch"])
    if scope == "nav2":
        motion("mode_switch", "mode_switch", cart, switch_payload(config))
        motion("navigate", "navigate", cart, config["nav_goal"])
    return {"version": 1, "name": "기존 주행 시험", "assumptions": [], "steps": steps}


def trial_presets(config, scope="nav2", *, smartcart_only=False):
    """Editable plans share one binding set; selecting a preset never dispatches."""
    if smartcart_only:
        choices = {"smartcart": legacy_plan(config, scope, True)}
    elif config["scenario"] == "single_pinky":
        choices = {"single": legacy_plan(config, scope)}
    else:
        choices = {
            "two": legacy_plan(config, scope),
            "smartcart": legacy_plan(config, scope, True),
        }
        # Bypass the loading bay rather than commanding a stop/turn through N01.
        loading, delivery = (
            config["routes"]["to_loading"],
            config["routes"]["to_delivery"],
        )
        if loading["route"][-2] == delivery["route"][1]:
            route = deepcopy(loading)
            route.update(
                route=loading["route"][:-1] + delivery["route"][2:],
                goal=deepcopy(delivery["goal"]),
            )
            single = {
                **config,
                "scenario": "single_pinky",
                "routes": {
                    "to_delivery": route,
                    "cart_to_switch": config["routes"]["cart_to_switch"],
                },
            }
            choices = {"single": legacy_plan(single, scope), **choices}
    labels = {
        "single": "1대 시험",
        "two": "2대 시험",
        "smartcart": "스마트카트 단독 시험",
    }
    for name, plan in choices.items():
        plan["name"] = labels[name]
        plan["assumptions"] = [
            "비참여 기체는 시험 통로 밖에 정지; OMX 작업은 더미 승인",
            "동일 핑키가 물류·스마트카트 역할 수행"
            if name == "single"
            else "창고 입고 완료 가정"
            if name == "smartcart"
            else "물류 복귀 후 카트 진입",
        ]
    return choices


def validate_plan(config, value, *, allowed_roles=None):
    """No live state or I/O; each robot's position/mode must connect across steps."""
    if not isinstance(value, dict) or set(value) != {
        "version",
        "name",
        "assumptions",
        "steps",
    }:
        raise ValueError("plan fields: version, name, assumptions, steps")
    if type(value["version"]) is not int or value["version"] != 1:
        raise ValueError("unsupported plan version")
    if not isinstance(value["name"], str) or not 1 <= len(value["name"].strip()) <= 80:
        raise ValueError("plan name requires 1..80 characters")
    assumptions = value["assumptions"]
    if (
        not isinstance(assumptions, list)
        or len(assumptions) > 8
        or any(
            not isinstance(s, str) or not 1 <= len(s.strip()) <= 160
            for s in assumptions
        )
    ):
        raise ValueError("assumptions require at most 8 short descriptions")
    steps = value["steps"]
    if not isinstance(steps, list) or not 1 <= len(steps) <= 24:
        raise ValueError("plan requires 1..24 steps")
    roles = set(config["robots"]) if allowed_roles is None else set(allowed_roles)
    seen, positions, modes = set(), {}, {}
    for step in steps:
        if not isinstance(step, dict) or set(step) != {
            "id",
            "kind",
            "role",
            "label",
            "payload",
        }:
            raise ValueError("invalid step fields")
        sid, kind, role, p = (step[k] for k in ("id", "kind", "role", "payload"))
        if (
            not isinstance(sid, str)
            or not re.fullmatch(r"[a-zA-Z0-9_.-]{1,64}", sid)
            or sid in seen
            or sid in {"READY", "HOLD", "COMPLETE", "status"}
        ):
            raise ValueError("step IDs must be unique, non-reserved identifiers")
        if (
            not isinstance(kind, str)
            or kind not in KINDS
            or not isinstance(role, str)
            or role not in roles
        ):
            raise ValueError("unsupported step kind/robot role")
        if (
            not isinstance(step["label"], str)
            or not 1 <= len(step["label"].strip()) <= 80
            or not isinstance(p, dict)
        ):
            raise ValueError("invalid step label/payload")
        seen.add(sid)
        if kind in MOTION:
            command(
                site=config["site_id"],
                control=config["control_id"],
                robot=config["robots"][role]["robot_id"],
                task="validation",
                step=sid,
                kind=kind,
                payload={
                    **p,
                    "binding_id": "validation",
                    "expected_boot_id": "validation",
                },
            )
        if kind == "route":
            path = p["route"]
            if (
                p["graph_version"] != config["graph_version"]
                or any(n not in config["nodes"] for n in path)
                or any(b not in config["nodes"][a] for a, b in pairwise(path))
            ):
                raise ValueError(f"{sid}: disconnected/unknown graph route")
            if path[-1] not in stop_nodes(config):
                raise ValueError(
                    f"{sid}: route must end at a stop-bar node; intersections are via nodes only"
                )
            if any(a == c for a, c in zip(path, path[2:])):
                raise ValueError(f"{sid}: in-route U-turn is unsupported by Pinky")
            if p["start_maneuver"] != "none":
                raise ValueError(
                    f"{sid}: installed Pinky driver supports maneuver none only"
                )
            if not 0.015 <= p["speed_limit_mps"] <= 0.08:
                raise ValueError(f"{sid}: local trial speed must be 0.015..0.08 m/s")
            # A bay arrival has a driver-specific loading turn; it cannot be a through node.
            if any(len(config["nodes"][n]) == 1 for n in path[1:-1]):
                raise ValueError(f"{sid}: split route at a bay before continuing")
            if (
                modes.get(role, "line") != "line"
                or positions.get(role, path[0]) != path[0]
            ):
                raise ValueError(
                    f"{sid}: previous position/mode does not match route start"
                )
            positions[role], modes[role] = path[-1], "line"
        elif kind == "mode_switch":
            if positions.get(role, "N12") != "N12" or modes.get(role, "line") != "line":
                raise ValueError(f"{sid}: switch requires line mode at N12")
            positions[role], modes[role] = "N12", "nav2"
        elif kind == "navigate":
            if modes.get(role, "nav2") != "nav2":
                raise ValueError(f"{sid}: insert mode switch before navigation")
            positions[role], modes[role] = None, "nav2"
        else:
            if (
                set(p) != {"node_id", "package_id", "quantity"}
                or p["node_id"] not in stop_nodes(config)
                or p["package_id"] != config["package_id"]
                or type(p["quantity"]) is not int
                or p["quantity"] != 1
            ):
                raise ValueError(
                    f"{sid}: gate requires known node and configured one-item package"
                )
            if (
                positions.get(role, p["node_id"]) != p["node_id"]
                or modes.get(role, "line") != "line"
            ):
                raise ValueError(f"{sid}: gate must match previous line stop")
            positions[role], modes[role] = p["node_id"], "line"
        if kind in {"mode_switch", "navigate"} and any(
            p[k] != config[k] for k in ("map_id", "map_version")
        ):
            raise ValueError(f"{sid}: map mismatch")
    return deepcopy(value)


def shortest_route(nodes, start, goal, blocked=()):
    """Fewest graph edges, deterministic; physical obstacles still require field checks."""
    if start not in nodes or goal not in nodes or start in blocked or goal in blocked:
        raise ValueError("unknown or occupied start/goal")
    queue = deque([[start]])
    seen = {start, *blocked}
    while queue:
        path = queue.popleft()
        if path[-1] == goal:
            return path
        for nxt in sorted(nodes[path[-1]]):
            if nxt not in seen:
                seen.add(nxt)
                queue.append([*path, nxt])
    raise ValueError("no available graph route")


def route_payload(config, path, heading, speed=0.06):
    return {
        "graph_version": config["graph_version"],
        "start_node": path[0],
        "start_heading_rad": heading,
        "start_maneuver": "none",
        "route": path,
        "goal": {"node_id": path[-1], "location_id": None, "on_arrival": "hold"},
        "speed_limit_mps": speed,
    }


def delivery_plan(config, request, statuses):
    """One-item pickup request -> available robot, graph routes, load gate, optional Nav2.

    Service WSS/inventory are not simulated as implemented: the caller supplies pickup.
    """
    required = {
        "request_id",
        "package_id",
        "quantity",
        "role",
        "pickup",
        "destination",
        "pickup_heading_rad",
        "nav2",
    }
    if not isinstance(request, dict) or set(request) != required:
        raise ValueError("invalid delivery request fields")
    if not isinstance(request["request_id"], str) or not re.fullmatch(
        r"[A-Za-z0-9_.-]{1,64}", request["request_id"]
    ):
        raise ValueError("request_id required")
    if (
        request["package_id"] != config["package_id"]
        or type(request["quantity"]) is not int
        or request["quantity"] != 1
    ):
        raise ValueError("request requires configured package, quantity 1")
    if type(request["nav2"]) is not bool or (
        request["nav2"] and request["destination"] != "N12"
    ):
        raise ValueError("Nav2 request requires N12 destination")
    if type(request["pickup_heading_rad"]) not in (int, float) or not math.isfinite(
        request["pickup_heading_rad"]
    ):
        raise ValueError("pickup departure heading must be confirmed")
    if request["role"] not in {"auto", *config["robots"]}:
        raise ValueError("unknown requested role")
    candidates = []
    for role, robot in config["robots"].items():
        s = statuses.get(robot["robot_id"])
        if request["role"] not in {"auto", role} or not s or s["mode"] != "line":
            continue
        blocked = {
            v["node_id"]
            for k, v in statuses.items()
            if k != robot["robot_id"] and v.get("node_id")
        }
        try:
            a = shortest_route(
                config["nodes"], s["node_id"], request["pickup"], blocked
            )
            b = shortest_route(
                config["nodes"], request["pickup"], request["destination"], blocked
            )
        except ValueError:
            continue
        candidates.append((len(a) + len(b), role, a, b, s["heading_rad"]))
    if not candidates:
        raise ValueError("no ready robot with an unoccupied route")
    _, role, a, b, heading = min(candidates)
    steps = []
    if len(a) > 1:
        steps.append(
            {
                "id": "pickup_route",
                "kind": "route",
                "role": role,
                "label": "물품 위치로 이동",
                "payload": route_payload(config, a, heading),
            }
        )
    steps.append(
        {
            "id": "pickup_load",
            "kind": "gate",
            "role": role,
            "label": "적재 완료 승인 (더미)",
            "payload": {
                "node_id": request["pickup"],
                "package_id": request["package_id"],
                "quantity": 1,
            },
        }
    )
    if len(b) > 1:
        steps.append(
            {
                "id": "delivery_route",
                "kind": "route",
                "role": role,
                "label": "목적지로 이동",
                "payload": route_payload(config, b, request["pickup_heading_rad"]),
            }
        )
    if request["nav2"]:
        steps += [
            {
                "id": "mode_switch",
                "kind": "mode_switch",
                "role": role,
                "label": "Nav2 전환",
                "payload": switch_payload(config),
            },
            {
                "id": "navigate",
                "kind": "navigate",
                "role": role,
                "label": "목표 이동",
                "payload": deepcopy(config["nav_goal"]),
            },
        ]
    return validate_plan(
        config,
        {
            "version": 1,
            "name": f"출고 요청 {request['request_id']}",
            "assumptions": [
                "물품 위치·준비 상태는 시험 입력; OMX 완료는 운영자 더미 승인"
            ],
            "steps": steps,
        },
    )
