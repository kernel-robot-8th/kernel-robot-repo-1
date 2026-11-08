"""Bounded local RPC, serviced only by the MQTT trial's single owner loop."""

import json
import os
import selectors
import socket
import time
from copy import deepcopy
from pathlib import Path
from uuid import uuid4

from roport_control_server.executor.plans import stop_nodes, trial_presets


class TrialSocket:
    def __init__(self, path, control):
        self.path = Path(path)
        self.control = control
        self.session_id = str(uuid4())
        self.selector = selectors.DefaultSelector()
        self.clients = {}
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Never replace a socket another owner may still be using.
        try:
            self.listener.bind(str(self.path))
        except BaseException:
            self.listener.close()
            self.selector.close()
            raise
        os.chmod(self.path, 0o600)
        self.listener.listen(8)
        self.listener.setblocking(False)
        self.selector.register(self.listener, selectors.EVENT_READ)

    def snapshot(self):
        c = self.control
        state = c.state
        now = time.monotonic()
        config = {
            key: deepcopy(c.config[key])
            for key in (
                "scenario",
                "mode",
                "package_id",
                "robots",
                "routes",
                "nodes",
                "graph_version",
                "map_id",
                "map_version",
                "nav_goal",
            )
        }
        config["robots"] = deepcopy(c.robots)
        if c.state.get("custom_plan"):
            config["routes"] = {
                s["id"]: deepcopy(s["payload"])
                for s in c.plan["steps"]
                if s["kind"] == "route"
            }
        elif c.smartcart_only:
            config["routes"] = {
                step: route
                for step, route in config["routes"].items()
                if step.startswith("cart_")
            }
        return {
            "session_id": self.session_id,
            "plan": deepcopy(c.plan),
            "plan_revision": c.state.get("plan_revision"),
            "custom_plan": c.state.get("custom_plan", False),
            "plan_index": c.state.get("plan_index", 0),
            "catalogue": {
                "robots": c.allowed_robots,
                "routes": c.config["routes"],
                "nav_goal": c.config["nav_goal"],
                "nodes": c.config["nodes"],
                "stop_nodes": sorted(stop_nodes(c.config)),
                "presets": trial_presets(
                    c.config, c.scope, smartcart_only=c.smartcart_only
                ),
                "saved": [p["name"] for p in c.saved_plans()],
            },
            "scope": c.scope,
            "smartcart_only": c.smartcart_only,
            "start_error": c.start_error(),
            "request_error": c.request_error(),
            "config": config,
            "state": {
                key: deepcopy(state[key])
                for key in (
                    "task_id",
                    "phase",
                    "fault",
                    "gate",
                    "active",
                    "statuses",
                    "completed",
                    "transfers",
                    "request",
                )
            },
            "ages": {
                r["robot_id"]: max(0, now - c.heard[r["robot_id"]])
                if r["robot_id"] in c.heard
                else None
                for r in c.robots.values()
            },
            "mqtt_ready": c.transport.ready,
            "events": [
                {k: e[k] for k in ("message_id", "type", "source", "sent_at")}
                for e in list(state["events"].values())[-12:]
            ],
        }

    def dispatch(self, request):
        if request == {"action": "snapshot"}:
            return self.snapshot()
        required = {"action", "session_id", "task_id", "phase", "expires_at"}
        if not isinstance(request, dict) or request.get("action") not in {
            "start",
            "dummy",
            "request",
            "stop",
            "validate_plan",
            "configure",
            "submit_delivery",
            "delivery_plan",
            "route_plan",
            "save_plan",
            "load_plan",
        }:
            raise ValueError("unknown trial action")
        if request["action"] == "load_plan":
            required.add("name")
        elif request["action"] in {"validate_plan", "save_plan"}:
            required.add("plan")
        elif request["action"] == "submit_delivery":
            required.update({"request", "plan", "field_confirmed"})
        elif request["action"] == "configure":
            required.update({"plan", "field_confirmed"})
        elif request["action"] == "delivery_plan":
            required.add("request")
        elif request["action"] == "route_plan":
            required.update({"start_node", "goal_node"})
        elif request["action"] == "dummy":
            required.add("stop_session_id")
        elif request["action"] == "request":
            required.add("items")
        if set(request) != required:
            raise ValueError("invalid trial command fields")
        expiry = request["expires_at"]
        if (
            type(expiry) not in (int, float)
            or not time.time() < expiry <= time.time() + 2
        ):
            raise ValueError("expired trial command; refresh status")
        state = self.control.state
        if (
            request["session_id"] != self.session_id
            or request["task_id"] != state["task_id"]
            or (request["action"] != "stop" and request["phase"] != state["phase"])
        ):
            raise ValueError("trial changed; refresh status before acting")
        action = request["action"]
        if action in {
            "validate_plan",
            "save_plan",
            "load_plan",
            "configure",
            "submit_delivery",
            "delivery_plan",
            "route_plan",
        }:
            if state["phase"] not in {"READY", "COMPLETE"}:
                raise ValueError("plan is frozen while running or held")
            if action == "load_plan":
                plans = [
                    p
                    for p in self.control.saved_plans()
                    if p["name"] == request["name"]
                ]
                if not plans:
                    raise ValueError("saved plan not found")
                return {"plan": self.control.validate_plan(plans[0])}
            if action == "route_plan":
                return {
                    "route": self.control.find_route(
                        request["start_node"], request["goal_node"]
                    )
                }
            if action == "delivery_plan":
                return {"plan": self.control.plan_delivery(request["request"])}
            plan = self.control.validate_plan(request["plan"])
            if action == "submit_delivery":
                if self.control.plan_delivery(request["request"]) != plan:
                    raise ValueError(
                        "robot state changed; regenerate and review request plan"
                    )
                self.control.configure(
                    plan, request["field_confirmed"], request["request"]
                )
                self.session_id = str(uuid4())
                return self.snapshot()
            if action == "validate_plan":
                return {"plan": plan, "validated": True}
            if action == "save_plan":
                self.control.save_plan(plan)
            else:
                self.control.configure(plan, request["field_confirmed"])
                self.session_id = str(uuid4())
            return self.snapshot()
        try:
            if request["action"] == "start":
                self.control.start()
            elif request["action"] == "dummy":
                self.control.dummy(request["stop_session_id"])
            elif request["action"] == "request":
                self.control.request(request["items"])
            else:
                if state["phase"] == "COMPLETE":
                    raise ValueError("trial is already complete")
                self.control.hold("operator_stopped")
        except (ConnectionError, TimeoutError) as exc:
            self.control.hold("transport_outcome_unknown")
            raise ValueError(
                "transport_outcome_unknown; inspect before retrying"
            ) from exc
        return self.snapshot()

    def _drop(self, client):
        self.selector.unregister(client)
        self.clients.pop(client)
        client.close()

    def poll(self):
        for key, _ in self.selector.select(0):
            client = key.fileobj
            if client is self.listener:
                client, _ = self.listener.accept()
                client.setblocking(False)
                if len(self.clients) >= 8:
                    client.close()
                    continue
                self.clients[client] = time.monotonic()
                self.selector.register(client, selectors.EVENT_READ)
                continue
            try:
                raw = client.recv(32769)
            except (ConnectionResetError, BlockingIOError):
                self._drop(client)
                continue
            try:
                if not raw or len(raw) > 32768:
                    raise ValueError("invalid local request size")
                result = {"ok": True, "snapshot": self.dispatch(json.loads(raw))}
            except (ValueError, TypeError, RecursionError) as exc:
                result = {"ok": False, "error": str(exc)}
            except OSError:
                self._drop(client)
                raise  # A failed journal write must stop the owner, not just the UI.
            try:
                raw = json.dumps(result, ensure_ascii=False, allow_nan=False).encode()
                if len(raw) > 65536:
                    raw = b'{"ok":false,"error":"trial snapshot exceeds local message limit"}'
                client.send(raw)
            except OSError:
                pass  # Never block the MQTT/lease loop on a disconnected viewer.
            finally:
                self._drop(client)
        for client, accepted in list(self.clients.items()):
            if time.monotonic() - accepted > 1:
                self._drop(client)

    def close(self):
        for client in list(self.clients):
            self._drop(client)
        self.selector.close()
        self.listener.close()
        self.path.unlink(missing_ok=True)
