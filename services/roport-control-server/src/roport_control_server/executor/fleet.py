"""Request-driven dispatch with robot and remaining-route reservations.

Robot completion, stock placement and customer receipt remain separate facts.
"""

import math
from copy import deepcopy
from time import monotonic

from .fleet_traffic import clear_prefix, edge_key, route_resources, validate_clearance
from .plans import shortest_route

SLOTS = {
    f"{row}{column}": {"node": node, "arms": arms, "size": size}
    for row in (1, 2)
    for column, node, arms, size in (
        ("a", "N21", ["omx1"], "small"),
        ("b", "N11", ["omx1", "omx2"], "large"),
        ("c", "N22", ["omx2"], "small"),
    )
}
PARKING = {"logistics": ("N16", "N17"), "smartcart": ("H1", "H2")}
ROLES = {"pinky1": "logistics", "pinky2": "logistics", "pinky3": "smartcart"}
TARGETS = (*ROLES, "omx1", "omx2", "omx3", "shelf-01")
OMX3_PACKAGE_MARKERS = frozenset({"15", "16", "17", "21", "22"})


class FleetControl:
    def __init__(self, config, store, *, clock=monotonic):
        self.config, self.store, self.clock = config, store, clock
        validate_clearance(
            config["nodes"],
            config.get("node_positions", {}),
            config.get("node_clearance_m"),
        )
        self.observed, self.poses, self.heard = {}, {}, {}
        self.observed_since = {}
        self.awaiting_state = set()
        roles = config.get("initial_roles", ROLES)
        if (
            not isinstance(roles, dict)
            or roles.keys() != ROLES.keys()
            or any(role not in (*PARKING, None) for role in roles.values())
        ):
            raise ValueError("invalid initial fleet roles")
        if not store.data:
            store.save(
                {
                    "roles": roles,
                    "last_nodes": {},
                    "field_versions": {
                        k: config[k] for k in ("graph_version", "map_version")
                    },
                    "slots": {s: {"state": "empty", "package_id": None} for s in SLOTS},
                    "tasks": {},
                    "commands": {},
                    "resources": {},
                    "events": {},
                    "pending_arrivals": {},
                    "enabled": False,
                    "fault": None,
                }
            )
        else:
            data = deepcopy(store.data)
            # Preserve an unfinished command from the former serial dispatcher.
            if "command" in data:
                old = data.pop("command")
                data["commands"] = {old["message"]["command_id"]: old} if old else {}
            data["enabled"] = False
            self.save(data)
            if data.get("field_versions") != {
                k: config[k] for k in ("graph_version", "map_version")
            }:
                self.hold("field changed: reconcile stored tasks and reservations")
            elif data["commands"]:
                self.hold("owner_restarted: reconcile active command")

    @property
    def data(self):
        return self.store.data

    def save(self, data):
        self.store.save(data)

    def hold(self, reason):
        data = deepcopy(self.data)
        data.update(enabled=False, fault=data["fault"] or reason)
        for active in data["commands"].values():
            data["tasks"][active["task"]]["status"] = "HOLD"
        self.save(data)

    def assign(self, robot, role):
        if robot not in ROLES or role not in {*PARKING, None}:
            raise ValueError("unknown robot/role")
        if self.config.get("mode") == "hardware" and robot not in self.config.get(
            "expected_starts", {}
        ):
            raise ValueError("robot outside verified field configuration")
        if any(
            t.get("robot") == robot and t["status"] not in {"COMPLETE", "CANCELED"}
            for t in self.data["tasks"].values()
        ):
            raise ValueError("finish/reconcile current robot task before changing role")
        if not (
            role is None
            and not self.data["enabled"]
            and robot not in self.data["last_nodes"]
            and robot not in self.observed
        ):
            self.ready(robot)
        data = deepcopy(self.data)
        data["roles"][robot] = role
        self.save(data)

    def validate_request(self, request):
        if set(request) - {"marker"} != {
            "request_id",
            "operation",
            "package_id",
            "goal",
        } or request["operation"] not in {"inbound", "outbound"}:
            raise ValueError("request_id, operation, package_id, goal required")
        if any(
            not isinstance(request[k], str) or not request[k].strip()
            for k in ("request_id", "package_id")
        ):
            raise ValueError("nonempty request/package required")
        if (
            request["operation"] == "outbound"
            and request["goal"] not in self.config["goals"]
        ):
            raise ValueError("unknown calibrated destination")
        if request["operation"] == "inbound" and request["goal"] is not None:
            raise ValueError("inbound destination is the shelf")
        previous = self.data["tasks"].get(request["request_id"])
        if previous:
            if previous["request"] != request:
                raise ValueError("request ID reused with different contents")
            return deepcopy(previous)
        if "marker" in request and request["marker"] not in (
            "15",
            "16",
            "17",
            "19",
            "20",
            "21",
            "22",
        ):
            raise ValueError("invalid physical package marker")
        if request["operation"] == "inbound":
            self.validate_warehouse_marker(request)
        if request["operation"] == "outbound":
            for slot in self.data["slots"].values():
                if slot["package_id"] == request["package_id"] and slot.get(
                    "marker"
                ) != request.get("marker"):
                    raise ValueError("shelf package marker differs from request")
        if any(
            t["request"]["package_id"] == request["package_id"]
            and t["status"] not in {"COMPLETE", "CANCELED"}
            for t in self.data["tasks"].values()
        ):
            raise ValueError("package already has an active task")
        if request["operation"] == "inbound" and any(
            s["package_id"] == request["package_id"]
            for s in self.data["slots"].values()
        ):
            raise ValueError("package already stored")
        if request["operation"] == "outbound" and not any(
            s["state"] == "occupied" and s["package_id"] == request["package_id"]
            for s in self.data["slots"].values()
        ):
            raise ValueError("출고할 물품이 선반에 등록되어 있지 않습니다.")
        return None

    @staticmethod
    def validate_warehouse_marker(request):
        marker = request.get("marker", request["package_id"])
        # Legacy trials use numeric package IDs; named virtual packages stay valid.
        if (
            "marker" in request or marker.isdecimal()
        ) and marker not in OMX3_PACKAGE_MARKERS:
            raise ValueError("OMX3 loading supports only markers 15,16,17,21,22")

    def submit(self, request):
        if previous := self.validate_request(request):
            return previous
        data = deepcopy(self.data)
        task = {
            "request": deepcopy(request),
            "stage": "queued",
            "status": "QUEUED",
            "reason": None,
        }
        data["tasks"][request["request_id"]] = task
        self.save(data)
        return deepcopy(task)

    def state_age(self, robot):
        received = self.heard.get(robot, -math.inf)
        return self.clock() - min(received, self.observed_since.get(robot, received))

    def ready(self, robot):
        p = self.observed.get(robot)
        if not p or self.state_age(robot) > 3:
            raise ValueError(f"{robot}: state unavailable/stale")
        arrival = self.data.get("pending_arrivals", {}).get(robot)
        if arrival and arrival.get("recovery_boot"):
            raise ValueError(f"{robot}: arrival requires recovery confirmation")
        if arrival or robot in self.awaiting_state:
            raise ValueError(f"{robot}: awaiting state after completion")
        s = p["state"]
        if not s.get("stopped") or s.get("fault") or p.get("active_command_id"):
            raise ValueError(f"{robot}: not ready")
        if (
            robot in ROLES
            and s.get("mode") == "line"
            and (s.get("line_enabled") or s.get("drive_owner") != "none")
        ):
            raise ValueError(f"{robot}: line controller still active")
        if any(robot in c["participants"] for c in self.data["commands"].values()):
            raise ValueError(f"{robot}: command still active")
        if robot not in ROLES and (
            s.get("ready") is not True
            or (
                robot != "shelf-01"
                and (
                    s.get("home_reached") is not True
                    or s.get("holding_state") != "empty"
                    or s.get("holding_package_id") is not None
                )
            )
        ):
            raise ValueError(f"{robot}: arm not ready/clear")
        return s

    def observe(self, message, *, age=0):
        robot, p = message["source"], message["payload"]
        old = self.observed.get(robot)
        if (
            old
            and old["boot_version"] == p["boot_version"]
            and p["sequence"] <= old["sequence"]
        ):
            return
        if any(
            t.get("robot") == robot
            and t["status"] not in {"COMPLETE", "CANCELED"}
            and t.get("robot_boot") != p["boot_version"]
            for t in self.data["tasks"].values()
        ):
            self.hold(f"{robot}: restarted during task")
        for cmd in self.data["commands"].values():
            if (
                robot in cmd["participant_boots"]
                and p["boot_version"] != cmd["participant_boots"][robot]
            ):
                self.hold(f"{robot}: executor restarted")
        state = p["state"]
        node = state.get("node")
        arrival = self.data.get("pending_arrivals", {}).get(robot)
        if arrival:
            if p["boot_version"] != arrival["boot_version"]:
                if arrival.get("recovery_boot") != p["boot_version"]:
                    data = deepcopy(self.data)
                    data["pending_arrivals"][robot]["recovery_boot"] = p["boot_version"]
                    self.save(data)
            elif (
                arrival.get("recovery_boot")
                or state.get("arrival_command_id") != arrival["command_id"]
                or not self.stopped_at(p, arrival["node"])
            ):
                # A delayed heartbeat must not undo the confirmed arrival.
                return
            else:
                data = deepcopy(self.data)
                del data["pending_arrivals"][robot]
                self.save(data)
        elif old and old["boot_version"] == p["boot_version"]:
            self.awaiting_state.discard(robot)
        received = self.clock()
        self.observed[robot], self.heard[robot] = deepcopy(p), received
        self.observed_since[robot] = received - max(age, 0)
        # Only a known node or a confirmed Nav2 state replaces line occupancy.
        if (
            robot in ROLES
            and (node is not None or p["state"].get("mode") == "nav2")
            and (
                robot not in self.data["last_nodes"]
                or self.data["last_nodes"][robot] != node
            )
        ):
            data = deepcopy(self.data)
            data["last_nodes"][robot] = node
            self.save(data)
        if p["state"].get("fault") and any(
            robot in c["participants"] for c in self.data["commands"].values()
        ):
            self.hold(f"{robot}: {p['state']['fault']}")
        if not self.data["fault"]:
            for cid, active in self.data["commands"].items():
                if active["message"]["target"] == robot and (
                    p.get("active_command_id") == cid
                    or p["state"].get("arrival_command_id") == cid
                ):
                    self.route_progress(cid, node)

    @staticmethod
    def stopped_at(payload, node):
        state = payload["state"]
        return (
            state.get("node") == node
            and state.get("mode") == "line"
            and state.get("stopped") is True
            and payload.get("active_command_id") is None
            and not state.get("fault")
            and not state.get("line_enabled")
            and state.get("drive_owner") == "none"
        )

    def confirm_arrival_after_restart(self, robot, *, command_id, boot_version, note):
        """Acknowledge a restarted robot only after checking it at the arrival node."""
        if robot not in ROLES or not isinstance(note, str) or not note.strip():
            raise ValueError("known Pinky and operator note required")
        arrival = self.data.get("pending_arrivals", {}).get(robot)
        if (
            not arrival
            or not arrival.get("recovery_boot")
            or arrival["command_id"] != command_id
            or arrival["recovery_boot"] != boot_version
        ):
            raise ValueError(f"{robot}: pending restart arrival changed")
        p = self.observed.get(robot)
        if (
            not p
            or p["boot_version"] != boot_version
            or self.state_age(robot) > 3
            or not self.stopped_at(p, arrival["node"])
        ):
            raise ValueError(
                f"{robot}: fresh stopped state at the arrival node required"
            )
        data = deepcopy(self.data)
        del data["pending_arrivals"][robot]
        data.setdefault("arrival_recoveries", {})[command_id] = {
            "robot": robot,
            "node": arrival["node"],
            "boot_version": boot_version,
            "note": note.strip(),
        }
        for task in data["tasks"].values():
            if task.get("robot") == robot and task["status"] not in {
                "COMPLETE",
                "CANCELED",
            }:
                task["robot_boot"] = boot_version
        self.save(data)

    def receipt(self, request_id):
        task = self.data["tasks"].get(request_id)
        if not task or task["stage"] not in {
            "await_receipt",
            "return_nav",
            "switch_line",
            "return_route",
            "done",
        }:
            raise ValueError("task is not awaiting customer receipt")
        if task["stage"] != "await_receipt":
            return
        data = deepcopy(self.data)
        data["tasks"][request_id].update(
            stage="return_nav", status="RUNNING", reason=None
        )
        self.save(data)

    def occupied(self, robot):
        occupied = set()
        for other, role in self.data["roles"].items():
            if other == robot:
                continue
            arrival = self.data.get("pending_arrivals", {}).get(other)
            if arrival:
                occupied.add(arrival["node"])
            p = self.observed.get(other)
            fresh = p and self.state_age(other) <= 3
            state = p["state"] if fresh else {}
            if state.get("node") is not None:
                occupied.add(state["node"])
            elif state.get("mode") == "nav2":
                continue
            elif role is not None:
                raise ValueError(f"{other}: position unknown")
            elif (node := self.data["last_nodes"].get(other)) is not None:
                occupied.add(node)
        return occupied

    def parking(self, role, robot, data=None):
        busy = self.occupied(robot) | {
            t["parking"]
            for t in (data or self.data)["tasks"].values()
            if t.get("robot") != robot
            and t["status"] not in {"COMPLETE", "CANCELED"}
            and "parking" in t
        }
        for node in PARKING[role]:
            if node not in busy:
                return node
        raise ValueError("no unoccupied return bay")

    def allocate(self, data, task):
        request = task["request"]
        role = "logistics" if request["operation"] == "inbound" else "smartcart"
        slot = next(
            (
                key
                for key, s in data["slots"].items()
                if s["state"] == "occupied" and s["package_id"] == request["package_id"]
            ),
            None,
        )
        if role == "smartcart" and slot is None:
            raise ValueError("package not confirmed on shelf")
        candidates = []
        for robot, assigned in data["roles"].items():
            previous = next(
                (
                    t
                    for t in data["tasks"].values()
                    if t.get("robot") == robot
                    and t["status"] not in {"COMPLETE", "CANCELED"}
                ),
                None,
            )
            returning = (
                previous
                and role == "logistics"
                and previous["request"]["operation"] == "inbound"
                and previous["stage"] == "return_route"
                and previous["status"] == "RUNNING"
                and "departure" not in previous
            )
            if assigned != role or (previous and not returning):
                continue
            try:
                state = self.ready(robot)
                if state.get("mode") != "line" or (
                    not returning and state.get("node") not in PARKING[role]
                ):
                    continue
                bay = self.parking(role, robot, data)
                goal = "N01" if role == "logistics" else SLOTS[slot]["node"]
                path = shortest_route(self.config["nodes"], state["node"], goal)
                candidates.append((len(path), robot, bay))
            except ValueError:
                continue
        if not candidates:
            raise ValueError(f"no ready {role} robot with free route/return bay")
        _, robot, bay = min(candidates)
        for previous in data["tasks"].values():
            if previous.get("robot") == robot and previous["status"] not in {
                "COMPLETE",
                "CANCELED",
            }:
                previous.update(
                    stage="done",
                    status="COMPLETE",
                    reason=None,
                    return_reassigned_to=request["request_id"],
                )
        task.update(
            robot=robot,
            robot_boot=self.observed[robot]["boot_version"],
            role=role,
            parking=bay,
            status="RUNNING",
            stage="warehouse_route" if role == "logistics" else "shelf_route",
        )
        if slot:
            task["slot"] = slot
            data["slots"][slot]["state"] = "reserved"

    def select_slot(self, data, task):
        if task.get("size") not in {"small", "large"}:
            raise ValueError("OMX3 package size unknown; hold at N01")
        candidates = []
        for slot, definition in SLOTS.items():
            if (
                slot not in self.config.get("trial_shelf_slots", SLOTS)
                or definition["size"] != task["size"]
                or data["slots"][slot]["state"] != "empty"
            ):
                continue
            try:
                for arm in definition["arms"]:
                    self.ready(arm)
                path = shortest_route(
                    self.config["nodes"],
                    "N01",
                    definition["node"],
                    self.occupied(task["robot"]),
                )
                candidates.append((len(path), slot))
            except ValueError:
                continue
        if not candidates:
            raise ValueError("no compatible free shelf/arm/access route")
        task["slot"] = min(candidates)[1]
        data["slots"][task["slot"]] = {
            "state": "reserved",
            "package_id": task["request"]["package_id"],
        }
        task["stage"] = "shelf_route"

    def next_action(self):
        if not self.data["enabled"] or self.data["fault"]:
            return None
        data = deepcopy(self.data)
        # Finish transfers first; queued work can reuse an empty returning robot.
        pending = sorted(
            data["tasks"].items(),
            key=lambda item: {"queued": 1, "return_route": 2}.get(item[1]["stage"], 0),
        )
        for request_id, task in pending:
            if task["stage"] in {"done", "await_receipt"} or task["status"] == "HOLD":
                continue
            if any(c["task"] == request_id for c in data["commands"].values()):
                continue
            try:
                if task["request"]["operation"] == "inbound" and task["stage"] in {
                    "queued",
                    "warehouse_route",
                    "warehouse_load",
                }:
                    self.validate_warehouse_marker(task["request"])
                if task["stage"] == "queued":
                    self.allocate(data, task)
                if task["stage"] == "choose_slot":
                    self.select_slot(data, task)
                action = self.action(task)
                if set(self.command_resources(task, action)) & set(data["resources"]):
                    raise ValueError("waiting for shared robot/work area")
                task["reason"] = None
                self.save(data)
                return request_id, action
            except ValueError as exc:
                task["reason"] = str(exc)
        if data != self.data:
            self.save(data)
        return None

    @staticmethod
    def route_goal(task):
        return {
            "warehouse_route": "N01",
            "shelf_route": SLOTS[task["slot"]]["node"] if "slot" in task else None,
            "switch_route": "N12",
            "return_route": task["parking"],
        }.get(task["stage"])

    def waiting_node_clear(self, node, task):
        # Waiting must leave other robots a route to their goal and out of a bay.
        for other in self.data["tasks"].values():
            if other.get("robot") in {None, task["robot"]} or other["status"] in {
                "COMPLETE",
                "CANCELED",
            }:
                continue
            if goal := self.route_goal(other):
                start = self.observed[other["robot"]]["state"]["node"]
                try:
                    shortest_route(self.config["nodes"], start, goal, {node})
                except ValueError:
                    return False
            stage = other["stage"]
            if stage in {"warehouse_route", "warehouse_load", "choose_slot"}:
                start, goals = "N01", {s["node"] for s in SLOTS.values()}
            elif stage in {"shelf_route", "transfer"}:
                start = SLOTS[other["slot"]]["node"]
                goals = {other["parking"] if other["role"] == "logistics" else "N12"}
            elif stage in {"navigate", "await_receipt", "return_nav", "switch_line"}:
                start, goals = "N12", {other["parking"]}
            else:
                continue
            if node == start:
                return False
            for goal in goals:
                try:
                    shortest_route(self.config["nodes"], start, goal, {node})
                    break
                except ValueError:
                    pass
            else:
                return False
        return True

    def route_prefix(self, task, start, goal):
        occupied = self.occupied(task["robot"])
        moving = {
            c["message"]["target"]
            for c in self.data["commands"].values()
            if c["message"]["type"] == "line_route_command"
        }
        path = clear_prefix(
            self.config["nodes"],
            start,
            goal,
            occupied=occupied,
            moving_nodes={
                p["state"].get("node")
                for robot, p in self.observed.items()
                if robot in moving
            },
            reserved=self.data["resources"],
        )
        while (
            len(path) > 1
            and path[-1] != goal
            and not self.waiting_node_clear(path[-1], task)
        ):
            path.pop()
        if len(path) < 2:
            raise ValueError("waiting for route clearance")
        if task["stage"] == "return_route" and task["role"] == "logistics":
            # Normal completion at a safe node permits reassignment without STOP/HOLD.
            for index, node in enumerate(path[1:-1], 1):
                if self.waiting_node_clear(node, task):
                    return path[: index + 1]
        return path

    def action(self, task):
        robot, stage = task["robot"], task["stage"]
        state = self.ready(robot)
        boot = self.observed[robot]["boot_version"]
        base = {"expected_boot_version": boot}
        if goal := self.route_goal(task):
            if state["mode"] != "line":
                raise ValueError("line mode required before route")
            if state.get("heading_rad") is None:
                raise ValueError("line heading unavailable")
            if state["node"] == goal:
                raise ValueError("arrival evidence required; already at destination")
            path = self.route_prefix(task, state["node"], goal)
            payload = {
                **base,
                "graph_version": self.config["graph_version"],
                "start_node": path[0],
                "start_heading_rad": state["heading_rad"],
                "start_maneuver": "none",
                "route": path,
                "goal": {"node": path[-1], "on_arrival": "hold"},
                "speed_limit_mps": self.config["speed_limit_mps"],
            }
            if task.get("departure"):
                payload["departure"] = task["departure"]
            return robot, "line_route_command", payload
        if stage in {"switch_nav", "switch_line"}:
            if stage == "switch_nav" and state.get("node") != "N12":
                raise ValueError("mode switch requires N12")
            return (
                robot,
                "set_drive_mode_command",
                {
                    **base,
                    "node": "N12",
                    "from_mode": "line" if stage == "switch_nav" else "nav2",
                    "to_mode": "nav2" if stage == "switch_nav" else "line",
                    "graph_version": self.config["graph_version"],
                    "map_version": self.config["map_version"],
                },
            )
        if stage in {"navigate", "return_nav"}:
            if (
                state["mode"] != "nav2"
                or not state.get("nav2_ready")
                or not state.get("localization_ready")
            ):
                raise ValueError("Nav2/localization not ready")
            goal = (
                self.config["n12_goal"]
                if stage == "return_nav"
                else self.config["goals"][task["request"]["goal"]]
            )
            return (
                robot,
                "nav_goal_command",
                {**base, **goal, "map_version": self.config["map_version"]},
            )
        slot = task.get("slot")
        arms = SLOTS[slot]["arms"] if slot else ["omx3"]
        target = "shelf-01" if len(arms) == 2 else arms[0]
        self.ready(target)
        for arm in arms:
            self.ready(arm)
        expected_node = SLOTS[slot]["node"] if stage == "transfer" else "N01"
        if state.get("node") != expected_node or state.get(
            "arrival_command_id"
        ) != task.get("arrival"):
            raise ValueError("Pinky arrival does not match this transfer")
        p = {
            "expected_boot_version": self.observed[target]["boot_version"],
            "package_id": task["request"].get("marker", task["request"]["package_id"]),
            "arrival_command_id": task["arrival"],
            "expected_pinky_boot_version": boot,
            "model_version": self.config["omx"][target]["model_version"],
            "calibration_version": self.config["omx"][target]["calibration_version"],
        }
        if stage == "warehouse_load":
            return (
                target,
                "warehouse_load_command",
                {
                    **p,
                    "pinky_id": robot,
                    "pickup_location_id": "pickup-01",
                    "load_position": "deck",
                },
            )
        store = task["request"]["operation"] == "inbound"
        if len(arms) == 2:
            return (
                target,
                "shelf_transfer_command",
                {
                    **p,
                    "operation": "store" if store else "retrieve",
                    "slot_id": slot,
                    "pinky_id": robot,
                    "arms": [
                        {
                            "robot_id": arm,
                            "expected_boot_version": self.observed[arm]["boot_version"],
                        }
                        for arm in arms
                    ],
                },
            )
        source = {"kind": "pinky", "robot_id": robot, "position": "deck"}
        dest = {"kind": "shelf", "slot_id": slot}
        return (
            target,
            "arm_action_command",
            {
                **p,
                "operation": "transfer",
                "workcell_id": "shelf-01",
                "source_location": source if store else dest,
                "destination_location": dest if store else source,
            },
        )

    edge = staticmethod(edge_key)

    def command_resources(self, task, action, route_index=0):
        target, kind, payload = action
        resources = {
            "robot:" + r
            for r in {
                task["robot"],
                target,
                *(a["robot_id"] for a in payload.get("arms", [])),
            }
        }
        resources.add("parking:" + task["parking"])
        if task.get("slot"):
            resources.add("slot:" + task["slot"])
        if route := payload.get("route"):
            resources.update(
                route_resources(
                    route,
                    route_index,
                    positions=self.config.get("node_positions"),
                    clearance=self.config.get("node_clearance_m"),
                )
            )
        elif kind in {"set_drive_mode_command", "nav_goal_command"}:
            resources.add("node:N12")
        else:
            # The existing OMX host has one active operation across its arms.
            resources.update(
                {
                    "omx-host",
                    "node:"
                    + (
                        SLOTS[task["slot"]]["node"]
                        if task["stage"] == "transfer"
                        else "N01"
                    ),
                }
            )
        return resources

    def reservations(self, data):
        resources = {}
        for active in data["commands"].values():
            message = active["message"]
            keys = self.command_resources(
                data["tasks"][active["task"]],
                (message["target"], message["type"], message["payload"]),
                active.get("route_index", 0),
            )
            if keys & resources.keys():
                raise ValueError("overlapping command reservations")
            resources.update(dict.fromkeys(keys, active["task"]))
        return resources

    def route_progress(self, command_id, node):
        active = self.data["commands"][command_id]
        route = active["message"]["payload"].get("route", [])
        if node not in route or route.index(node) <= active.get("route_index", 0):
            return
        data = deepcopy(self.data)
        data["commands"][command_id]["route_index"] = route.index(node)
        data["resources"] = self.reservations(data)
        self.save(data)

    def dispatched(self, request_id, message):
        data = deepcopy(self.data)
        task = data["tasks"][request_id]
        participants = {
            task["robot"],
            message["target"],
            *(a["robot_id"] for a in message["payload"].get("arms", [])),
        }
        data["commands"][message["command_id"]] = {
            "task": request_id,
            "message": message,
            "stage": task["stage"],
            "participants": sorted(participants),
            "participant_boots": {
                r: self.observed[r]["boot_version"] for r in participants
            },
            "sequence": -1,
            "route_index": 0,
            "sent_at": self.clock(),
        }
        data["resources"] = self.reservations(data)
        self.save(data)  # Persist exact command before MQTT publish.

    def result(self, message):
        prior = self.data["events"].get(message["message_id"])
        if prior:
            if prior != message:
                raise ValueError("conflicting event ID")
            return True
        active = self.data["commands"].get(message["command_id"])
        if not active:
            return False
        cmd, p = active["message"], message["payload"]
        stop = active.get("stop", {}).get("payload", {})
        # A restarted Pinky may interrupt inherited work only via the recorded STOP.
        stop_interrupt = (
            self.data["fault"]
            and cmd["target"] in ROLES
            and stop.get("expected_boot_version") == p["boot_version"]
            and stop.get("target_command_id") == cmd["command_id"]
            and p["status"] == "interrupted"
            and p.get("reason") == "operator_stop"
            and p.get("result", {}).get("stopped") is True
        )
        if message["source"] != cmd["target"] or (
            p["boot_version"] != cmd["payload"]["expected_boot_version"]
            and not stop_interrupt
        ):
            raise ValueError("command source/boot mismatch")
        if p["sequence"] <= active["sequence"]:
            return False
        data = deepcopy(self.data)
        data["commands"][message["command_id"]]["sequence"] = p["sequence"]
        data["events"][message["message_id"]] = message
        task = data["tasks"][active["task"]]
        if p["status"] in {"failed", "rejected", "interrupted"}:
            task.update(status="HOLD", reason=p.get("reason", p["status"]))
            data.update(fault=data["fault"] or task["reason"], enabled=False)
            for other in data["commands"].values():
                data["tasks"][other["task"]]["status"] = "HOLD"
        elif p["status"] == "completed" and not data["fault"]:
            self.check_completion(cmd, p.get("result", {}), task, active)
            self.advance(data, task, cmd, p["result"])
            del data["commands"][message["command_id"]]
            data["resources"] = self.reservations(data)
        if cmd["type"] == "line_route_command" and not data["fault"]:
            node = p.get("result", {}).get("node")
            route = cmd["payload"]["route"]
            if node in route and route.index(node) >= active.get("route_index", 0):
                # Batched progress can arrive after a newer state heartbeat.
                # Store/ACK it, but never move the observed route position back.
                self.observed[cmd["target"]]["state"]["node"] = node
                data["last_nodes"][cmd["target"]] = node
        pinky_completed = (
            p["status"] == "completed" and not data["fault"] and cmd["target"] in ROLES
        )
        if pinky_completed and cmd["type"] == "line_route_command":
            data.setdefault("pending_arrivals", {})[cmd["target"]] = {
                "boot_version": p["boot_version"],
                "node": p["result"]["node"],
                "command_id": cmd["command_id"],
            }
        self.save(data)
        if pinky_completed and cmd["type"] != "line_route_command":
            self.awaiting_state.add(cmd["target"])
        if p["status"] == "in_progress" and not data["fault"]:
            self.route_progress(message["command_id"], p.get("result", {}).get("node"))
        return True

    def check_completion(self, cmd, result, task, active):
        kind, p = cmd["type"], cmd["payload"]
        if kind == "line_route_command":
            if (
                result.get("node") != p["goal"]["node"]
                or result.get("stopped") is not True
                or result.get("mode") != "line"
            ):
                raise ValueError("route completion mismatch")
        elif kind == "set_drive_mode_command":
            if (
                result.get("mode") != p["to_mode"]
                or result.get("stopped") is not True
                or result.get("node") != "N12"
            ):
                raise ValueError("mode completion mismatch")
        elif kind == "nav_goal_command":
            pose = result.get("pose", {})
            if (
                result.get("stopped") is not True
                or result.get("mode") != "nav2"
                or set(pose) != {"x", "y", "yaw"}
            ):
                raise ValueError("navigation completion lacks stopped pose")
            if (
                math.hypot(pose["x"] - p["pose"]["x"], pose["y"] - p["pose"]["y"])
                > p["position_tolerance_m"]
                or abs(
                    math.atan2(
                        math.sin(pose["yaw"] - p["pose"]["yaw"]),
                        math.cos(pose["yaw"] - p["pose"]["yaw"]),
                    )
                )
                > p["yaw_tolerance_rad"]
            ):
                raise ValueError("navigation goal tolerance mismatch")
        else:
            if (
                result.get("package_id") != p["package_id"]
                or result.get("pinky_id") != task["robot"]
                or result.get("arrival_command_id") != task["arrival"]
                or result.get("placement_confirmed") is not True
                or result.get("release_confirmed") is not True
            ):
                raise ValueError("transfer lacks package placement/release evidence")
            arms = (
                SLOTS[task["slot"]]["arms"] if task["stage"] == "transfer" else ["omx3"]
            )
            reported = {a["robot_id"]: a for a in result.get("arms", [])}
            if set(reported) != set(arms) or any(
                a.get("stopped") is not True
                or a.get("home_reached") is not True
                or a.get("holding_state") != "empty"
                or a.get("holding_package_id") is not None
                or a.get("boot_version") != active["participant_boots"][r]
                for r, a in reported.items()
            ):
                raise ValueError("transfer lacks matching arm return evidence")
            if task["stage"] == "transfer" and result.get("slot_id") != task["slot"]:
                raise ValueError("transfer slot mismatch")

    def advance(self, data, task, cmd, result):
        stage = task["stage"]
        task["reason"] = None
        next_stage = {
            "warehouse_route": "warehouse_load",
            "warehouse_load": "choose_slot",
            "shelf_route": "transfer",
            "switch_route": "switch_nav",
            "switch_nav": "navigate",
            "navigate": "await_receipt",
            "return_nav": "switch_line",
            "switch_line": "return_route",
            "return_route": "done",
        }
        if cmd["type"] == "line_route_command":
            task.pop("departure", None)
            if result["node"] != self.route_goal(task):
                return
            task["arrival"] = cmd["command_id"]
        if stage in {"warehouse_load", "transfer"}:
            task["departure"] = {
                "arrival_command_id": task["arrival"],
                "transfer_command_id": cmd["command_id"],
            }
        if stage == "warehouse_load":
            task["size"] = result.get("package_size", "unknown")
        if stage == "transfer":
            inbound = task["request"]["operation"] == "inbound"
            data["slots"][task["slot"]] = {
                "state": "occupied" if inbound else "empty",
                "package_id": task["request"]["package_id"] if inbound else None,
                **(
                    {"marker": task["request"]["marker"]}
                    if inbound and "marker" in task["request"]
                    else {}
                ),
            }
            task["stage"] = "return_route" if inbound else "switch_route"
        else:
            task["stage"] = next_stage[stage]
        if task["stage"] == "done":
            task["status"] = "COMPLETE"
        elif task["stage"] == "await_receipt":
            task["status"] = "AWAITING_RECEIPT"

    def enable(self):
        if self.data["fault"]:
            raise ValueError("reconcile fault before enabling; no automatic recovery")
        data = deepcopy(self.data)
        data["enabled"] = True
        self.save(data)

    def check_active(self):
        if not self.data["enabled"]:
            return
        for active in self.data["commands"].values():
            if (
                self.clock() - active.get("sent_at", -math.inf)
                > self.config["command_timeout_s"]
            ):
                self.hold("command outcome unknown: timeout")
            elif any(self.state_age(r) > 3 for r in active["participants"]):
                self.hold("participant state stale")
            elif active["message"]["payload"].get("route"):
                robot = active["message"]["target"]
                protected = {
                    key.removeprefix("node:")
                    for key, owner in self.data["resources"].items()
                    if owner == active["task"] and key.startswith("node:")
                }
                try:
                    blocked = protected & self.occupied(robot)
                except ValueError as exc:
                    self.hold(str(exc))
                else:
                    if blocked:
                        self.hold("route occupied: " + ", ".join(sorted(blocked)))

    def snapshot(self):
        return {
            **{
                k: deepcopy(v)
                for k, v in self.data.items()
                if k not in {"events", "host_releases"}
            },
            "robots": deepcopy(self.observed),
            "poses": deepcopy(self.poses),
            "ages": {r: self.state_age(r) for r in self.heard},
            "slot_definitions": SLOTS,
            "goals": list(self.config["goals"]),
        }
