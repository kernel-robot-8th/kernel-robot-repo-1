"""Trial UI projection and local IPC; never imports MQTT or owns robot execution."""

import itertools
import json
import secrets
import socket
import time
from copy import deepcopy
from threading import Lock
from uuid import uuid4

from .display import ROBOTS

PHASES = {
    "READY": "출발 대기",
    "to_loading": "창고 상차 구역으로 이동",
    "WAIT_LOAD": "OMX 창고 상차 완료 승인 대기",
    "return_warehouse": "물류 핑키 창고 복귀",
    "WAIT_REQUEST": "시험 물품 요청 대기",
    "cart_to_delivery": "스마트카트 인도장으로 이동",
    "to_delivery": "인도장으로 이동",
    "WAIT_INBOUND": "OMX 입고 완료 승인 대기",
    "WAIT_CART_LOAD": "OMX 카트 적재 완료 승인 대기",
    "cart_to_switch": "N12로 이동",
    "mode_switch": "Nav2 전환 확인",
    "navigate": "공용공간 목표로 이동",
    "COMPLETE": "시험 완료",
    "HOLD": "시험 보류",
}


class TrialView:
    def __init__(self, socket_path, token, grid):
        if not token or len(token) < 24:
            raise ValueError("ROPORT_TRIAL_TOKEN requires at least 24 characters")
        if "layout" not in grid:
            raise ValueError("trial UI requires a measured map and layout")
        self.socket_path, self.token = str(socket_path), token
        self.session = str(uuid4())
        self.revisions = itertools.count(1)
        self.last = None
        self._snapshot_lock = Lock()
        self._next_snapshot = 0.0
        self._available = False
        self.map_id, self.map_version = grid["id"], grid["version"]
        self.graph_version = grid["layout"]["graph_version"]
        self.nodes = {n["id"]: n["position"] for n in grid["layout"]["nodes"]}
        self.graph = {n["id"]: sorted(n["links"]) for n in grid["layout"]["nodes"]}

    def authorized(self, token):
        return secrets.compare_digest(self.token.encode(), token.encode())

    def rpc(self, request):
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
            connection.settimeout(1)
            connection.connect(self.socket_path)
            connection.sendall(json.dumps(request, allow_nan=False).encode())
            data = connection.recv(65537)
        if len(data) > 65536:
            raise ValueError("local response too large")
        result = json.loads(data)
        if not result["ok"]:
            raise ValueError(result["error"])
        return result["snapshot"]

    def compatible(self, config):
        return (
            config["scenario"] in {"single_pinky", "two_pinky"}
            and all(
                r["robot_id"] in {"pinky1", "pinky2", "pinky3"}
                for r in config["robots"].values()
            )
            and config["map_id"] == self.map_id
            and config["map_version"] == self.map_version
            and config["graph_version"] == self.graph_version
            and {k: sorted(v) for k, v in config["nodes"].items()} == self.graph
        )

    def command(self, request, *, secure_transport=False):
        if not isinstance(request, dict) or "expires_at" in request:
            raise ValueError("invalid trial request")
        with self._snapshot_lock:
            try:
                snapshot = self.rpc({"action": "snapshot"})
                if snapshot["config"]["mode"] == "hardware" and not secure_transport:
                    raise PermissionError(
                        "실물 조작은 로컬 주소 또는 HTTPS에서만 가능합니다."
                    )
                if (
                    not self.compatible(snapshot["config"])
                    and request.get("action") != "stop"
                ):
                    raise ValueError("trial map/graph mismatch; commands disabled")
                return self.rpc({**request, "expires_at": time.time() + 0.8})
            finally:
                self._next_snapshot = 0.0

    def bootstrap(self):
        result = {
            "robots": ROBOTS,
            "trial": True,
            "trial_nodes": self.nodes,
            "trial_routes": [],
        }
        try:
            snapshot = self.rpc({"action": "snapshot"})
        except OSError:
            return (
                result  # Keep the original map/tabs usable while the owner is offline.
            )
        if not self.compatible(snapshot["config"]):
            raise ValueError(
                "trial requires matching map/graph and supported Pinky profile"
            )
        result["trial_routes"] = [
            r["route"] for r in snapshot["config"]["routes"].values()
        ]
        return result

    def snapshot(self):
        with self._snapshot_lock:
            if time.monotonic() >= self._next_snapshot:
                try:
                    data = self.rpc({"action": "snapshot"})
                    self.last = (data, time.monotonic())
                    self._available = True
                except (OSError, ValueError, KeyError):
                    self._available = False
                # Share reads and failed probes across HTTP/WS viewers in this worker.
                self._next_snapshot = time.monotonic() + 0.2
            available = self._available
            data, received = deepcopy(self.last) if self.last else (None, None)
        error = None if available else "시험 실행부 미연결 · 조작 비활성"
        rows, tasks, trial = [], [], None
        if data:
            state, config = data["state"], data["config"]
            matches = self.compatible(config)
            if not matches:
                error = "시험 지도/노드 설정 불일치 · 출발/승인 비활성"
            phase = state["phase"]
            scope = data.get("scope", "nav2")
            label = (
                "N12 주행 완료 · Nav2 제외"
                if phase == "COMPLETE" and scope == "line-only"
                else PHASES.get(phase, phase)
            )
            for role, device in config["robots"].items():
                robot = device["robot_id"]
                status = state["statuses"].get(robot, {})
                age = data["ages"][robot]
                if age is not None:
                    age += time.monotonic() - received
                stale = (
                    not available or not data["mqtt_ready"] or age is None or age > 3
                )
                pose, source = None, "위치 미수신"
                node = status.get("node_id")
                if matches and status.get("mode") == "line" and node in self.nodes:
                    pose = [*self.nodes[node], 0, status["heading_rad"]]
                    source = "마지막 확인 노드 기준 · 연속 위치 아님"
                completed = state["completed"].get("navigate")
                if (
                    matches
                    and phase == "COMPLETE"
                    and completed
                    and completed["source"] == robot
                ):
                    p = completed["payload"]["pose"]
                    pose, source = [p["x"], p["y"], 0, p["yaw"]], "Nav2 완료 응답 위치"
                rows.append(
                    {
                        "robot_id": robot,
                        "role": role,
                        "pose": pose,
                        "pose_source": source,
                        "joints": {},
                        "velocity": None,
                        "age": age,
                        "stale": stale,
                        "mode": f"{status.get('mode', '미수신')} · {node or '노드 미확인'}",
                        "status": status,
                    }
                )
            fresh = (
                available
                and matches
                and data["mqtt_ready"]
                and all(not r["stale"] for r in rows)
            )
            safe_wait = fresh and all(
                r["status"].get("stopped")
                and not r["status"].get("line_enabled")
                and r["status"].get("fault") is None
                for r in rows
            )
            single = config["scenario"] == "single_pinky"
            trial = {
                "session_id": data["session_id"],
                "task_id": state["task_id"],
                "phase": phase,
                "label": label,
                "scope": scope,
                "scenario": config["scenario"],
                "package_id": config["package_id"],
                "request": state["request"],
                "active_robot": (state["active"] or {}).get("target")
                or (state["gate"] or {}).get("robot_id")
                or (
                    state["completed"].get("navigate")
                    or state["completed"].get("cart_to_switch")
                    or {}
                ).get("source"),
                "mode": config["mode"],
                "mqtt_ready": data["mqtt_ready"] and available,
                "gate": state["gate"],
                "fault": state["fault"],
                "events": data["events"],
                "available": available,
                "can_start": safe_wait and phase == "READY" and not data["start_error"],
                "start_error": data["start_error"] if phase == "READY" else None,
                "can_dummy": safe_wait
                and phase in {"WAIT_LOAD", "WAIT_INBOUND", "WAIT_CART_LOAD"},
                "can_request": safe_wait and not single and phase == "WAIT_REQUEST",
                "can_stop": available and phase not in {"COMPLETE", "HOLD"},
                "route": (state["active"] or {}).get("payload", {}).get("route", []),
                "routes": [r["route"] for r in config["routes"].values()]
                if matches
                else [],
            }
            tasks = [
                {
                    "task_id": state["task_id"],
                    "robot": ", ".join(
                        r["robot_id"] for r in config["robots"].values()
                    ),
                    "stage": label,
                    "progress": round(
                        len(state["completed"])
                        / (len(config["routes"]) + (0 if scope == "line-only" else 2))
                        * 100
                    ),
                    "status": "연결 끊김" if not available else state["fault"] or phase,
                }
            ]
        by_id = {row["robot_id"]: row for row in rows}
        rows = [
            by_id.get(
                robot["id"],
                {
                    "robot_id": robot["id"],
                    "pose": None,
                    "joints": {},
                    "velocity": None,
                    "age": None,
                    "stale": True,
                    "status": {},
                    "mode": "시험 미배정"
                    if robot["kind"] == "pinky"
                    else "상태 미연결",
                },
            )
            for robot in ROBOTS
        ]
        return {
            "session_id": self.session,
            "revision": next(self.revisions),
            "robots": rows,
            "tasks": tasks,
            "demo": False,
            "trial": trial,
            "source": "MQTT 시험 실행부" if available else "실행부 연결 끊김",
            "task_source": "MQTT 시험 시나리오 · 실제 수신 응답 기준",
            "errors": [e for e in (error, trial and trial["fault"]) if e],
            "message": "노드 기준 표시 · 연속 좌표 미수신 · 중단은 보류/lease 갱신 중단",
        }
