"""One trial owner: warehouse cycle, dummy transfers, smartcart and Nav2.

Run with --config FILE --journal FILE and optional --socket for the web UI.
Nothing moves until an explicit start; the journal is isolated from operational storage.
"""

import argparse
import fcntl
import hashlib
import itertools
import json
import logging
import math
import os
import selectors
import signal
import socket
import sys
from contextlib import ExitStack
from copy import deepcopy
from dataclasses import replace
from datetime import UTC, datetime
from ipaddress import ip_address
from pathlib import Path
from queue import Empty
from time import monotonic
from uuid import uuid4

from roport_control_server.adapters.mqtt import field_wire
from roport_control_server.adapters.mqtt.host_metrics import HostMetricsReceiver
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import decode, timestamp, topic
from roport_control_server.adapters.mqtt.motion import MotionReceiver, motion_topic
from roport_control_server.adapters.mqtt.mqtt import MqttTransport
from roport_control_server.adapters.mqtt.trial_wire import (
    PROFILE,
    check_result,
    command,
    fresh,
    mqtt_config,
    reply,
    validate_trial,
)
from roport_control_server.executor.omx_trial import (
    WORKCELL,
    OmxTrial,
    validate_settings,
)
from roport_control_server.executor.plans import (
    MOTION,
    delivery_plan,
    legacy_plan,
    service_plan,
    shortest_route,
    stop_nodes,
    validate_plan,
)
from roport_control_server.executor.trial_socket import TrialSocket

LOG = logging.getLogger(__name__)

NAV_RETRY_FAULTS = frozenset(
    (
        "nav2_goal_failed",
        "nav2_recovery_limit",
        "nav2_recovery_timeout",
        "gate_dock_marker_missing",
        "gate_dock_marker_lost",
        "gate_dock_timeout",
    )
)

ROUTES = (
    "to_loading",
    "to_delivery",
    "return_warehouse",
    "cart_to_delivery",
    "cart_to_switch",
)


def validate_host_metrics(settings):
    if not isinstance(settings, dict) or set(settings) != {"hosts", "stale_after_s"}:
        raise ValueError("host_metrics requires hosts and stale_after_s")
    HostMetricsReceiver(None, settings["hosts"], settings["stale_after_s"])


def start_host_metrics(config, connection):
    """Optional display-only subscriber; its failure never affects the task link."""
    settings = config.get("host_metrics")
    if settings is None:
        return None, None
    transport = MqttTransport(
        replace(connection, client_id=connection.client_id + "-host"),
        [field_wire.topic(host, "host") for host in settings["hosts"]],
    )
    receiver = HostMetricsReceiver(
        transport, settings["hosts"], settings["stale_after_s"]
    )
    try:
        transport.start()
    except (ConnectionError, OSError):
        print("Host metrics unavailable; task connection is unchanged.", flush=True)
    return receiver, transport


def validate_config(config, *, require_field_verification=True):
    if "host_metrics" in config:
        validate_host_metrics(config["host_metrics"])
    if "omx" in config:
        validate_settings(config["omx"])
        if config["mode"] == "simulation" and config["omx"]["execution_mode"] != "mock":
            raise ValueError("simulation requires a mock OMX executor")
    if config["profile"] != PROFILE or config["mode"] not in {"simulation", "hardware"}:
        raise ValueError("trial profile/mode required")
    scenario = config.get("scenario", "two_pinky")
    if scenario not in {"single_pinky", "two_pinky"}:
        raise ValueError("unknown trial scenario")
    single_robot = scenario == "single_pinky"
    robots = config["robots"]
    roles = {"logistics"} if single_robot else {"logistics", "smartcart"}
    if set(robots) != roles:
        raise ValueError("robot roles do not match trial scenario")
    if len({r["robot_id"] for r in robots.values()}) != len(roles) or len(
        {r["binding_id"] for r in robots.values()}
    ) != len(roles):
        raise ValueError("roles require distinct robot slots and bindings")
    for r in robots.values():
        if r["robot_id"] not in {"pinky1", "pinky2", "pinky3"}:
            raise ValueError("unknown Pinky slot")
        command(
            site=config["site_id"],
            control=config["control_id"],
            robot=r["robot_id"],
            task="validation",
            step="status",
            kind="status",
            payload={"binding_id": r["binding_id"]},
        )
    if not isinstance(config["package_id"], str) or not config["package_id"].strip():
        raise ValueError("trial package_id required")
    nodes = config["nodes"]
    for node, links in nodes.items():
        if any(
            target == node or target not in nodes or node not in nodes[target]
            for target in links
        ):
            raise ValueError("graph must have symmetric known edges")
    route_steps = ("to_delivery", "cart_to_switch") if single_robot else ROUTES
    if set(config["routes"]) != set(route_steps):
        raise ValueError("routes do not match trial scenario")
    for step in route_steps:
        p = config["routes"][step]
        command(
            site=config["site_id"],
            control=config["control_id"],
            robot=robots["logistics"]["robot_id"],
            task="validation",
            step=step,
            kind="route",
            payload={**p, "binding_id": "validation", "expected_boot_id": "validation"},
        )
        if p["graph_version"] != config["graph_version"] or any(
            b not in nodes.get(a, []) for a, b in zip(p["route"], p["route"][1:])
        ):
            raise ValueError("route does not match selected graph")
    routes = config["routes"]
    if single_robot:
        path = routes["to_delivery"]["route"]
        if (
            path[0] != "N16"
            or path[-1] != "N11"
            or [n for n in path if n in {"N18", "N19"}] != ["N18", "N19"]
            or routes["cart_to_switch"]["route"][0] != "N11"
        ):
            raise ValueError("single Pinky requires N16 via N18/N19 to N11")
    else:
        for a, b in itertools.pairwise(ROUTES[:3]):
            if routes[a]["route"][-1] != routes[b]["route"][0]:
                raise ValueError("logistics legs are disconnected")
        if (
            routes["return_warehouse"]["route"][-1] != routes["to_loading"]["route"][0]
            or routes["cart_to_delivery"]["route"][-1]
            != routes["to_delivery"]["route"][-1]
            or routes["cart_to_switch"]["route"][0]
            != routes["cart_to_delivery"]["route"][-1]
        ):
            raise ValueError("trial stops/return mismatch")
    if routes["cart_to_switch"]["route"][-1] != "N12":
        raise ValueError("trial switch requires N12")
    command(
        site=config["site_id"],
        control=config["control_id"],
        robot=robots["logistics" if single_robot else "smartcart"]["robot_id"],
        task="validation",
        step="navigate",
        kind="navigate",
        payload={
            **config["nav_goal"],
            "binding_id": "validation",
            "expected_boot_id": "validation",
        },
    )
    if any(config["nav_goal"][k] != config[k] for k in ("map_id", "map_version")):
        raise ValueError("goal map mismatch")
    if (
        type(config["command_timeout_s"]) not in (int, float)
        or not math.isfinite(config["command_timeout_s"])
        or not 1 <= config["command_timeout_s"] <= 600
    ):
        raise ValueError("command timeout must be 1..600 seconds")
    if (
        require_field_verification
        and config["mode"] == "hardware"
        and config.get("field_verified") is not True
    ):
        raise ValueError(
            "verify starts, manoeuvres, goal and local watchdog before hardware"
        )


class TrialControl:
    def __init__(
        self, config, transport, journal, *, scope="nav2", smartcart_only=False
    ):
        if scope not in {"line-only", "nav2"}:
            raise ValueError("unknown trial scope")
        if journal.data and journal.data.get("scope", "nav2") != scope:
            raise ValueError("journal scope changed; use a new trial journal")
        if smartcart_only and config.get("scenario") != "two_pinky":
            raise ValueError("smartcart-only requires a two_pinky configuration")
        if journal.data and journal.data.get("smartcart_only", False) != smartcart_only:
            raise ValueError("journal role selection changed; use a new trial journal")
        self.scope = scope
        validate_config(config)
        self.config, self.transport, self.journal = deepcopy(config), transport, journal
        self.smartcart_only = smartcart_only
        self.robots = {
            role: robot
            for role, robot in self.config["robots"].items()
            if not smartcart_only or role == "smartcart"
        }
        self.allowed_robots = deepcopy(self.robots)
        self.plan = validate_plan(
            config,
            journal.data.get("plan") or legacy_plan(config, scope, smartcart_only),
            allowed_roles=self.allowed_robots,
        )
        self.robots = self._plan_robots(self.plan)
        if journal.data and journal.data.get("config") != config:
            raise ValueError("journal configuration changed; reconcile old trial first")
        if not journal.data:
            journal.save(
                {
                    "config": config,
                    "plan": self.plan,
                    "plan_index": 0,
                    "custom_plan": False,
                    "plan_revision": str(uuid4()),
                    "plan_requests": {},
                    "scope": scope,
                    "smartcart_only": smartcart_only,
                    "task_id": str(uuid4()),
                    "phase": "READY",
                    "active": None,
                    "completed": {},
                    "events": {},
                    "statuses": {},
                    "boot_ids": {},
                    "transfers": [],
                    "gate": None,
                    "request": None,
                    "fault": None,
                }
            )
        self.pending_status, self.heard = {}, {}
        self.discovery_robots = {}
        self.service_preview = None
        self.motion = None
        self.host_metrics = None
        self.last_poll = 0.0
        self.deadline = None
        self.omx = OmxTrial(self) if "omx" in config else None
        if self.state["phase"] not in {"READY", "COMPLETE", "HOLD"}:
            self.hold("control_restarted")

    @property
    def state(self):
        return self.journal.data

    def _plan_robots(self, plan):
        roles = {s["role"] for s in plan["steps"]}
        return {r: deepcopy(v) for r, v in self.allowed_robots.items() if r in roles}

    def hold(self, reason):
        if self.state["phase"] in {"HOLD", "COMPLETE"}:
            return  # Preserve the first failure or completed result during shutdown.
        state = deepcopy(self.state)
        state.update(phase="HOLD", fault=reason)
        self.journal.save(state)
        # Stop renewing the active robot's lease. Its local controller must stop.

    def retry_navigation(self, execution_id, *, reconciled=False):
        """CLI-only, explicit field reconciliation; never clears other HOLDs."""
        old = self.state["active"]
        if (
            reconciled is not True
            or not self.transport.ready
            or self.state["phase"] != "HOLD"
            or self.state["fault"] not in NAV_RETRY_FAULTS
            or not old
            or old["type"] != "navigate.requested"
            or old["execution_id"] != execution_id
        ):
            raise ValueError("explicit navigation failure reconciliation required")
        failures = [
            e
            for e in self.state["events"].values()
            if e["command_id"] == old["command_id"]
            and e["type"] == "navigate.failed"
            and e["payload"]["error_code"] == self.state["fault"]
        ]
        if not failures:
            raise ValueError("navigation terminal failure not confirmed")
        robot = self._robot(old["step_id"])
        status = self.state["statuses"].get(robot["robot_id"])
        if (
            not status
            or monotonic() - self.heard.get(robot["robot_id"], -math.inf) > 3
            or not -0.5
            <= (datetime.now(UTC) - timestamp(status["observed_at"])).total_seconds()
            <= 3
            or status["fault"] != self.state["fault"]
            or not status["stopped"]
            or status["line_enabled"]
            or status["mode"] != "nav2"
            or not status["nav2_ready"]
            or not status["localization_ready"]
            or status["drive_owner"] != "nav2"
            or status["boot_id"] != old["payload"]["expected_boot_id"]
            or any(status[k] != old["payload"][k] for k in ("map_id", "map_version"))
        ):
            raise ValueError(
                "navigation retry requires fresh stopped state in same boot/map"
            )
        cmd = command(
            site=self.config["site_id"],
            control=self.config["control_id"],
            robot=robot["robot_id"],
            task=old["task_id"],
            step=old["step_id"],
            kind="navigate",
            payload=deepcopy(old["payload"]),
        )
        state = deepcopy(self.state)
        state.setdefault("navigation_retries", []).append(
            {
                "failed_execution_id": old["execution_id"],
                "failed_command": deepcopy(old),
                "fault": self.state["fault"],
                "execution_id": cmd["execution_id"],
                "goal_id": old["payload"]["goal_id"],
                "reconciled_at": datetime.now(UTC).isoformat(),
            }
        )
        state.update(phase=old["step_id"], fault=None, active=cmd)
        self.journal.save(state)  # Persist the new attempt before publication.
        self.deadline = monotonic() + self.config["command_timeout_s"]
        self.transport.publish(
            topic(self.config["site_id"], "robot", robot["robot_id"], "cmd"), cmd
        )

    def _step(self, step=None):
        if step is None:
            return self.plan["steps"][self.state.get("plan_index", 0)]
        return next(s for s in self.plan["steps"] if s["id"] == step)

    def _robot(self, step):
        return self.robots[self._step(step)["role"]]

    def saved_plans(self):
        path = self.journal.path.parent / "scenario-plans.json"
        if not path.exists():
            return []
        with path.open() as file:
            if path.stat().st_size > 262144:
                raise ValueError("scenario library too large")
            data = json.load(file)
        return data.get("plans", [])

    def save_plan(self, plan):
        # Journal provides the same exclusive lock, fsync and atomic replacement as runs.
        library = Journal(self.journal.path.parent / "scenario-plans.json")
        try:
            plans = library.data.get("plans", [])
            plans = [p for p in plans if p["name"] != plan["name"]]
            if len(plans) >= 16:
                raise ValueError("scenario library limit: 16 plans")
            library.save({"plans": [*plans, plan]})
        finally:
            library.close()

    def validate_plan(self, plan):
        plan = validate_plan(self.config, plan, allowed_roles=self.allowed_robots)
        if self.scope == "line-only" and any(
            s["kind"] in {"mode_switch", "navigate"} for s in plan["steps"]
        ):
            raise ValueError(
                "line-only owner cannot execute Nav2; restart with --scope nav2"
            )
        return plan

    def configure(
        self, plan, field_confirmed=False, request=None, *, service_request=None
    ):
        if self.state["phase"] not in {"READY", "COMPLETE"}:
            raise ValueError(
                "plan is frozen after start or HOLD; reconcile before restarting"
            )
        plan = self.validate_plan(plan)
        if self.state.get("service_request") and self.state["phase"] != "COMPLETE":
            raise ValueError(
                "service request already applied; finish or reconcile before replacing"
            )
        if self.config["mode"] == "hardware" and field_confirmed is not True:
            raise ValueError(
                "confirm this plan's starts, headings, workspace and watchdog"
            )
        state = deepcopy(self.state)
        if request:
            previous = state.setdefault("plan_requests", {}).get(request["request_id"])
            if previous is not None:
                if previous != request or self.state.get("plan") != plan:
                    raise ValueError("conflicting request_id")
                return
            if len(state["plan_requests"]) >= 32:
                raise ValueError("request history full; use a new journal")
            state["plan_requests"][request["request_id"]] = deepcopy(request)
        if state["phase"] == "COMPLETE":
            for robot in self.robots.values():
                self._status(robot)
            archive = Journal(
                self.journal.path.parent / ("run-" + state["task_id"] + ".json")
            )
            try:
                archive.save(state)
            finally:
                archive.close()
            state.update(
                task_id=str(uuid4()),
                phase="READY",
                active=None,
                completed={},
                events={},
                transfers=[],
                gate=None,
                request=None,
                fault=None,
            )
        state.update(
            plan=plan,
            plan_index=0,
            custom_plan=True,
            plan_revision=str(uuid4()),
            field_confirmation={
                "confirmed": field_confirmed is True,
                "at": datetime.now(UTC).isoformat(),
            },
            service_request=deepcopy(service_request),
        )
        if service_request:
            records = state.setdefault("service_requests", {})
            if len(records) >= 32:
                raise ValueError(
                    "service request history full; reconcile before new journal"
                )
            records[service_request["request_id"]] = {
                "request": deepcopy(service_request),
                "task_id": state["task_id"],
            }
        self.journal.save(state)
        self.plan = plan
        self.robots = self._plan_robots(plan)
        self.pending_status.clear()
        self.last_poll = 0.0
        self.discovery_robots = {}
        self.service_preview = None

    def service_receipt(self, request):
        if not isinstance(request, dict) or not isinstance(
            request.get("request_id"), str
        ):
            raise TypeError("invalid service request")
        record = self.state.get("service_requests", {}).get(request["request_id"])
        if record is None:
            return None
        if record["request"] != request:
            raise ValueError("conflicting service request_id")
        return {
            "task_id": record["task_id"],
            "phase": self.state["phase"]
            if record["task_id"] == self.state["task_id"]
            else "COMPLETE",
        }

    def plan_service(self, request):
        # Validate intent before enabling read-only status discovery of another role.
        operation = request.get("operation") if isinstance(request, dict) else None
        if operation not in ("inbound", "outbound"):
            raise ValueError("unsupported service operation")
        role = (
            "logistics"
            if operation == "inbound" or "smartcart" not in self.allowed_robots
            else "smartcart"
        )
        if role not in self.allowed_robots:
            raise ValueError("service role unavailable")
        self.discovery_robots = {**self.robots, role: self.allowed_robots[role]}
        try:
            statuses = {
                r["robot_id"]: self._status(r) for r in self.discovery_robots.values()
            }
        except ValueError as exc:
            if str(exc) == "robot status unavailable/stale":
                return None
            raise
        config = {**self.config, "robots": self.allowed_robots}
        plan = service_plan(config, request, statuses, self.scope, self.allowed_robots)
        first = plan["steps"][0]
        if (
            first["kind"] == "route"
            and statuses[self.allowed_robots[first["role"]]["robot_id"]]["node_id"]
            != first["payload"]["start_node"]
        ):
            raise ValueError(
                "서비스 시작 장소에 기체가 없습니다. 고급 시험에서 위치를 확인하세요."
            )
        return plan

    def preview_service(self, request):
        if self.state["phase"] not in {"READY", "COMPLETE"}:
            raise ValueError("service busy; current task is frozen")
        if self.service_receipt(request):
            raise ValueError("이미 접수한 요청입니다. 현재 작업 결과를 확인하세요.")
        plan = self.plan_service(request)
        if plan is None:
            self.service_preview = None
            return {"pending": True}
        self.service_preview = {
            "preview_id": str(uuid4()),
            "request": deepcopy(request),
            "plan": plan,
        }
        return deepcopy(self.service_preview)

    def submit_service(self, request, preview_id, field_confirmed):
        receipt = self.service_receipt(request)
        if receipt:
            return receipt
        preview = self.service_preview
        if (
            not preview
            or preview_id != preview["preview_id"]
            or request != preview["request"]
        ):
            raise ValueError("서비스 계획을 다시 확인하세요.")
        if self.plan_service(request) != preview["plan"]:
            self.service_preview = None
            raise ValueError("기체 상태가 변경되었습니다. 계획을 다시 확인하세요.")
        self.configure(preview["plan"], field_confirmed, service_request=request)
        return self.service_receipt(request)

    def plan_delivery(self, request):
        statuses = {r["robot_id"]: self._status(r) for r in self.robots.values()}
        config = {**self.config, "robots": self.robots}
        return delivery_plan(config, request, statuses)

    def find_route(self, start, goal):
        if goal not in stop_nodes(self.config):
            raise ValueError(
                "goal must be a stop-bar node; intersections are via nodes only"
            )
        return shortest_route(self.config["nodes"], start, goal)

    def _status(self, robot):
        device = robot["robot_id"]
        status = self.state["statuses"].get(device)
        if not status or monotonic() - self.heard.get(device, -math.inf) > 3:
            raise ValueError("robot status unavailable/stale")
        if (
            status["fault"] is not None
            or not status["stopped"]
            or status["line_enabled"]
        ):
            raise ValueError("robot is not safely waiting")
        return status

    def _step_status(self, step):
        robot = self._robot(step["id"])
        status = self._status(robot)
        kind, p = step["kind"], step["payload"]
        if kind == "route":
            if status["node_id"] != p["start_node"] or status["mode"] != "line":
                raise ValueError("unexpected route start")
            angle = status["heading_rad"] - p["start_heading_rad"]
            if abs(math.atan2(math.sin(angle), math.cos(angle))) > 0.35:
                raise ValueError("route start heading mismatch")
            for other in self.robots.values():
                if other == robot:
                    continue
                node = self._status(other)["node_id"]
                if node not in self.config["nodes"]:
                    raise ValueError("other trial robot position unknown")
                if node in p["route"]:
                    raise ValueError("route occupied by another trial robot")
        elif kind in {"gate", "request", "mode_switch"}:
            if status["node_id"] != p["node_id"] or status["mode"] != "line":
                raise ValueError("step requires its configured line stop")
        elif (
            status["mode"] != "nav2"
            or not status["nav2_ready"]
            or not status["localization_ready"]
            or status["drive_owner"] != "nav2"
        ):
            raise ValueError("nav2_not_ready")
        return status

    def _send(self, step):
        definition = self._step(step)
        robot = self._robot(step)
        status = self._step_status(definition)
        kind, payload = definition["kind"], definition["payload"]
        cmd = command(
            site=self.config["site_id"],
            control=self.config["control_id"],
            robot=robot["robot_id"],
            task=self.state["task_id"],
            step=step,
            kind=kind,
            payload={
                **payload,
                "binding_id": robot["binding_id"],
                "expected_boot_id": status["boot_id"],
            },
        )
        state = deepcopy(self.state)
        state.update(phase=step, active=cmd, gate=None)
        self.journal.save(state)
        self.deadline = monotonic() + self.config["command_timeout_s"]
        self.transport.publish(
            topic(self.config["site_id"], "robot", robot["robot_id"], "cmd"), cmd
        )

    def start_error(self):
        if self.state["phase"] != "READY":
            return "trial already started"
        if not self.transport.ready:
            return "MQTT subscriptions not ready"
        try:
            for role in self.robots:
                first = next((s for s in self.plan["steps"] if s["role"] == role), None)
                if first is not None:
                    self._step_status(first)
        except ValueError as exc:
            return str(exc)
        return None

    def _enter_step(self, index, arrival=None):
        state = deepcopy(self.state)
        state.update(plan_index=index, active=None, gate=None)
        if index == len(self.plan["steps"]):
            state["phase"] = "COMPLETE"
            self.journal.save(state)
            return
        step = self.plan["steps"][index]
        self._step_status(step)
        state["phase"] = step["id"]
        if step["kind"] == "gate":
            state["gate"] = {
                **step["payload"],
                "task_id": state["task_id"],
                "step_id": step["id"],
                "stop_session_id": str(uuid4()),
                "arrival_execution_id": arrival,
                "robot_id": self._robot(step["id"])["robot_id"],
            }
        self.journal.save(state)
        if step["kind"] in MOTION:
            self._send(step["id"])

    def start(self):
        error = self.start_error()
        if error:
            raise ValueError(error)
        self._enter_step(0)

    def _gate_error(self, kind):
        if self.state["phase"] in {"COMPLETE", "HOLD", "READY"}:
            raise ValueError("no approval pending")
        step = self._step()
        if (
            self.state["phase"] != step["id"]
            or step["kind"] != kind
            or (kind == "gate" and not self.state["gate"])
        ):
            raise ValueError("approval does not match current step")
        for robot in self.robots.values():
            self._status(robot)
        self._step_status(step)
        # Check the next departure BEFORE consuming an operator approval.
        index = self.state.get("plan_index", 0) + 1
        if index < len(self.plan["steps"]):
            self._step_status(self.plan["steps"][index])

    def dummy(self, token):
        if not self.state["gate"] or self.state["gate"]["stop_session_id"] != token:
            raise ValueError("dummy does not match current stop session")
        if self.omx and self.omx.manual_error():
            raise ValueError(self.omx.manual_error())
        return self.complete_transfer(token, "ui")

    def complete_transfer(self, token, source, event=None):
        if any(t["stop_session_id"] == token for t in self.state["transfers"]):
            return False
        gate = self.state["gate"]
        if not gate or token != gate["stop_session_id"]:
            raise ValueError("dummy does not match current stop session")
        self._gate_error("gate")
        state = deepcopy(self.state)
        execution_mode = (
            self.config["omx"]["execution_mode"]
            if self.omx and gate.get("omx_operation")
            else ("mock" if self.config["mode"] == "simulation" else "hardware")
        )
        state["transfers"].append(
            {
                **gate,
                "simulated": execution_mode == "mock",
                "source": "trial-operator" if source == "ui" else "omx-mqtt",
                "completion_source": source,
                "execution_mode": execution_mode,
                "evidence": "operator_confirmation"
                if source == "ui"
                else event["payload"]["evidence"],
                "placement_confirmed": None,
                "event_id": event["message_id"] if event else None,
                "observed_at": datetime.now(UTC).isoformat(),
            }
        )
        self.journal.save(state)
        self._enter_step(state.get("plan_index", 0) + 1, gate["stop_session_id"])
        return True

    def request_error(self):
        if not self.transport.ready:
            return "MQTT subscriptions not ready"
        try:
            self._gate_error("request")
        except (ValueError, IndexError) as exc:
            return str(exc)
        return None

    def request(self, items):
        error = self.request_error()
        if error:
            raise ValueError(error)
        p = self._step()["payload"]
        if (
            items != [{"package_id": p["package_id"], "quantity": 1}]
            or type(items[0]["quantity"]) is not int
        ):
            raise ValueError(
                "trial accepts its one stored package after logistics return"
            )
        state = deepcopy(self.state)
        state["request"] = {"request_id": str(uuid4()), "items": items}
        self.journal.save(state)
        self._enter_step(state.get("plan_index", 0) + 1)

    def _ack(self, event):
        ack = reply(
            event,
            "event.ack",
            {"acked_message_id": event["message_id"], "status": "stored"},
        )
        self.transport.publish(
            topic(self.config["site_id"], "robot", event["source"], "ack"), ack
        )

    def handle(self, delivery):
        event = decode(delivery.payload)
        if self.omx and event["source"] == WORKCELL:
            self.omx.handle(delivery, event)
            return
        validate_trial(event)
        if event["type"].split(".")[1] not in {
            "accepted",
            "progress",
            "completed",
            "failed",
            "rejected",
        }:
            raise ValueError("invalid event channel/type")
        robot = next(
            (
                r
                for r in self.allowed_robots.values()
                if r["robot_id"] == event["source"]
            ),
            None,
        )
        if (
            delivery.retained
            or delivery.qos != 1
            or robot is None
            or delivery.topic
            != topic(self.config["site_id"], "robot", event["source"], "event")
            or event["target"] != self.config["control_id"]
            or event["site_id"] != self.config["site_id"]
            or event["payload"].get("binding_id") != robot["binding_id"]
        ):
            raise ValueError("untrusted/unrelated trial event")
        if event["task_id"] != self.state["task_id"]:
            archive = self.journal.path.parent / ("run-" + event["task_id"] + ".json")
            if archive.exists() and archive.stat().st_size <= 4 * 1024 * 1024:
                old = json.loads(archive.read_text())
                if old.get("events", {}).get(event["message_id"]) == event:
                    self._ack(
                        event
                    )  # Release old robot outbox only for durably stored events.
            return
        if (
            robot not in self.robots.values()
            and robot not in self.discovery_robots.values()
        ):
            return  # Late replies from a deselected robot cannot affect this plan.
        if event["message_id"] in self.state["events"]:
            if self.state["events"][event["message_id"]] != event:
                raise ValueError("conflicting event ID")
            self._ack(event)
            return
        p, device = event["payload"], event["source"]
        if event["type"] == "status.completed":
            cmd = self.pending_status.pop(event["command_id"], None)
            if cmd is None or timestamp(cmd["expires_at"]) <= datetime.now(UTC):
                return
            check_result(cmd, event)
            fresh(event)
            old_boot = self.state["boot_ids"].get(device)
            if old_boot and old_boot != p["boot_id"]:
                self.hold("robot_restarted")
                return
            if any(
                p[k] != self.config[k]
                for k in ("graph_version", "map_id", "map_version")
            ):
                self.hold("robot_version_mismatch")
                return
            current = self.state["statuses"].get(device)
            if current and timestamp(p["observed_at"]) < timestamp(
                current["observed_at"]
            ):
                return
            state = deepcopy(self.state)
            state["statuses"][device], state["boot_ids"][device] = p, p["boot_id"]
            self.journal.save(state)
            self.heard[device] = monotonic()
            retry_pending = (
                self.state.get("active")
                and any(
                    r["execution_id"] == self.state["active"]["execution_id"]
                    for r in self.state.get("navigation_retries", [])
                )
                and not any(
                    e["command_id"] == self.state["active"]["command_id"]
                    and e["type"] == "navigate.accepted"
                    for e in self.state["events"].values()
                )
            )
            if p["fault"] is not None and not (
                retry_pending and p["fault"] in NAV_RETRY_FAULTS
            ):
                self.hold(
                    p["fault"]
                    if self.state.get("active")
                    and self.state["active"]["type"] == "navigate.requested"
                    and p["fault"] in NAV_RETRY_FAULTS
                    else "robot_fault"
                )
            return
        cmd = self.state["active"]
        if not cmd or event["command_id"] != cmd["command_id"]:
            return
        if any(
            p[k] != self.config[k] for k in ("graph_version", "map_id", "map_version")
        ):
            raise ValueError("robot version changed during action")
        check_result(cmd, event)
        fresh(event)
        previous = [
            e
            for e in self.state["events"].values()
            if e["command_id"] == cmd["command_id"]
        ]
        revision = max((e["payload"]["phase_revision"] for e in previous), default=0)
        if p["phase_revision"] <= revision:
            raise ValueError("conflicting or reordered phase revision")
        if event["type"] == "route.progress":
            previous_seq = max(
                (e["payload"].get("seq", -1) for e in previous), default=-1
            )
            if p["seq"] < previous_seq:
                raise ValueError("route progress moved backwards")
        state = deepcopy(self.state)
        state["events"][event["message_id"]] = event
        state["statuses"][device] = p
        self.journal.save(state)
        self.heard[device] = monotonic()
        self._ack(event)  # Application ACK only after fsync; separate from PUBACK.
        phase = event["type"].split(".")[1]
        retry = next(
            (
                retry
                for retry in self.state.get("navigation_retries", [])
                if retry["execution_id"] == cmd["execution_id"]
            ),
            None,
        )
        if (
            self.state["phase"] != "HOLD"
            and phase == "rejected"
            and p["error_code"] == "recovery_required"
            and retry
            and retry.get("failed_command")
            and retry.get("fault")
        ):
            # The robot did not start this attempt; keep the confirmed failure retryable.
            state = deepcopy(self.state)
            state.update(
                phase="HOLD",
                fault=retry["fault"],
                active=retry["failed_command"],
            )
            self.deadline = None
            self.journal.save(state)
            return
        if phase in {"failed", "rejected"}:
            self.hold(p["error_code"])
        elif phase == "completed" and self.state["phase"] != "HOLD":
            step = cmd["step_id"]
            state = deepcopy(self.state)
            state["completed"][step] = event
            state["active"] = None
            self.deadline = None
            self.journal.save(state)
            try:
                self._enter_step(state.get("plan_index", 0) + 1, cmd["execution_id"])
            except ValueError:
                self.hold("next_step_not_ready")

    def poll(self, timeout=0.1):
        if self.motion is not None:
            self.motion.poll(self.state["statuses"])
        if self.host_metrics is not None:
            # Display only: never feeds gate, departure, HOLD or timeouts.
            try:
                self.host_metrics.poll()
            except Exception as exc:  # noqa: BLE001 - display-only; isolate from control loop
                LOG.warning("host metrics poll failed: %s", type(exc).__name__)
        try:
            if not self.transport.ready:
                self.hold("mqtt_disconnected")
                return
            if self.omx:
                try:
                    self.omx.poll()
                except ValueError:
                    self.hold("omx_gate_not_ready")
            now = monotonic()
            if self.deadline and now > self.deadline:
                self.hold("execution_timeout_unknown")
                self.deadline = None
            if self.state["phase"] not in {"READY", "COMPLETE", "HOLD"} and any(
                now - self.heard.get(r["robot_id"], -math.inf) > 3
                for r in self.robots.values()
            ):
                self.hold("robot_status_timeout")
            retry_wait = (
                self.state["phase"] == "HOLD"
                and self.state.get("active")
                and self.state["active"]["type"] == "navigate.requested"
                and self.state["fault"] in NAV_RETRY_FAULTS
            )
            if now - self.last_poll >= 1 and (
                self.state["phase"] != "HOLD" or retry_wait
            ):
                # A fresh response can arrive after the next 1 Hz request.
                utc_now = datetime.now(UTC)
                self.pending_status = {
                    key: cmd
                    for key, cmd in self.pending_status.items()
                    if timestamp(cmd["expires_at"]) > utc_now
                }
                for robot in {**self.robots, **self.discovery_robots}.values():
                    cmd = command(
                        site=self.config["site_id"],
                        control=self.config["control_id"],
                        robot=robot["robot_id"],
                        task=self.state["task_id"],
                        step="status",
                        kind="status",
                        payload={"binding_id": robot["binding_id"]},
                    )
                    self.pending_status[cmd["command_id"]] = cmd
                    self.transport.publish(
                        topic(
                            self.config["site_id"], "robot", robot["robot_id"], "cmd"
                        ),
                        cmd,
                    )
                self.last_poll = now
            try:
                delivery = self.transport.receive(timeout)
            except Empty:
                return
            try:
                self.handle(delivery)
            except ValueError:
                # A malformed/current event cannot unlock any following route.
                self.hold("invalid_trial_event")
            self.transport.ack(delivery)
        except (ConnectionError, TimeoutError):
            self.hold("transport_outcome_unknown")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--journal")
    parser.add_argument(
        "--check", action="store_true", help="static config check; no MQTT or journal"
    )
    parser.add_argument("--hardware-authorized", action="store_true")
    parser.add_argument(
        "--smartcart-only",
        action="store_true",
        help="two_pinky config: only smartcart H1 to N11, dummy load, N12 and optional Nav2",
    )
    parser.add_argument(
        "--scope",
        choices=("line-only", "nav2"),
        default="nav2",
        help="line-only completes stopped at N12; nav2 also verifies the public goal",
    )
    parser.add_argument(
        "--socket", help="Unix socket for the trial web UI; disables stdin"
    )
    args = parser.parse_args()
    with open(args.config) as file:
        config = json.load(file)
    validate_config(config, require_field_verification=not args.check)
    if args.smartcart_only and config.get("scenario") != "two_pinky":
        parser.error("--smartcart-only requires a two_pinky configuration")
    connection = mqtt_config(config["control_id"])
    if config["mode"] == "simulation" and connection.host not in {
        "localhost",
        "127.0.0.1",
        "::1",
    }:
        parser.error("simulation profile is loopback-only")
    if args.check:
        print(
            f"Static config valid: {config['scenario']} / {args.scope}; "
            f"field_verified={config.get('field_verified')}; "
            f"smartcart_only={args.smartcart_only}; "
            "broker, driver and physical readiness NOT checked"
        )
        return
    if not args.journal:
        parser.error("--journal required for execution")
    if config["mode"] == "hardware" and not args.hardware_authorized:
        parser.error("hardware trial requires field checks and --hardware-authorized")
    journal = Journal(args.journal)
    from roport_control_server.adapters.mqtt.field_trial_transport import (
        FieldTrialTransport,
    )

    transport = FieldTrialTransport(
        connection,
        [
            topic(config["site_id"], "robot", r["robot_id"], "event")
            for role, r in config["robots"].items()
            if not args.smartcart_only or role == "smartcart"
        ]
        + (
            [topic(config["site_id"], "workcell", WORKCELL, "event")]
            if "omx" in config
            else []
        ),
        config,
        lambda: journal.data,
    )
    control = TrialControl(
        config, transport, journal, scope=args.scope, smartcart_only=args.smartcart_only
    )
    selector = selectors.DefaultSelector()
    if not args.socket:
        selector.register(sys.stdin, selectors.EVENT_READ)
    web = motion_transport = host_transport = None
    owner = ExitStack()
    started = False

    def terminate(_signal, _frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, terminate)
    try:
        # Different journals must not let CLI and web owners evict each other in MQTT.
        lock_dir = (
            Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
            / "roport/trial-owners"
        )
        lock_dir.mkdir(parents=True, exist_ok=True)
        addresses = {connection.host}
        for *_, address in socket.getaddrinfo(
            connection.host, connection.port, type=socket.SOCK_STREAM
        ):
            ip = ip_address(address[0])
            ip = getattr(ip, "ipv4_mapped", None) or ip
            addresses.add(str(ip))
            if ip.is_loopback:
                addresses.update({"localhost", "127.0.0.1", "::1"})
        # Lock each resolved endpoint so aliases cannot create a second owner.
        for address in sorted(addresses):
            identity = hashlib.sha256(
                json.dumps([address, connection.port, config["control_id"]]).encode()
            ).hexdigest()
            lock = owner.enter_context((lock_dir / (identity + ".lock")).open("a"))
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.socket:
            web = TrialSocket(args.socket, control)
        transport.start()
        started = True
        motion_transport = FieldTrialTransport(
            replace(connection, client_id=connection.client_id + "-motion"),
            [
                motion_topic(config["site_id"], r["robot_id"])
                for r in config["robots"].values()
            ],
            config,
            lambda: journal.data,
        )
        control.motion = MotionReceiver(config, motion_transport)
        try:
            motion_transport.start()
        except ConnectionError:
            print(
                "Display motion unavailable; task connection is unchanged.", flush=True
            )
        control.host_metrics, host_transport = start_host_metrics(config, connection)
        if web:
            print(
                "Trial web owner ready; waiting for explicit operator commands.",
                flush=True,
            )
            while True:
                control.poll(0.05)
                web.poll()
        print(
            "Inputs: start | dummy <stop_session_id> | request <package_id> | retry <failed_execution_id> reconciled | status | stop",
            flush=True,
        )
        previous = None
        while True:
            control.poll()
            visible = (
                control.state["phase"],
                control.state["fault"],
                control.state["gate"],
            )
            if visible != previous:
                print(
                    json.dumps(
                        {"phase": visible[0], "fault": visible[1], "gate": visible[2]},
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                previous = deepcopy(visible)
            if selector.select(0):
                line = sys.stdin.readline()
                if not line:
                    break
                fields = line.strip().split()
                try:
                    if fields == ["start"]:
                        control.start()
                    elif len(fields) == 2 and fields[0] == "dummy":
                        control.dummy(fields[1])
                    elif len(fields) == 2 and fields[0] == "request":
                        control.request([{"package_id": fields[1], "quantity": 1}])
                    elif (
                        len(fields) == 3
                        and fields[0] == "retry"
                        and fields[2] == "reconciled"
                    ):
                        control.retry_navigation(fields[1], reconciled=True)
                    elif fields == ["status"]:
                        print(json.dumps(control.state, ensure_ascii=False), flush=True)
                    elif fields == ["stop"]:
                        break
                    else:
                        print("Unknown input", flush=True)
                except ValueError as exc:
                    print(f"Not advanced: {exc}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if started and control.state["phase"] != "COMPLETE":
                control.hold("operator_stopped")
        finally:
            transport.stop()
            if motion_transport:
                motion_transport.stop()
            if host_transport:
                host_transport.stop()
            if web:
                web.close()
            owner.close()
            journal.close()
            selector.close()
            signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
