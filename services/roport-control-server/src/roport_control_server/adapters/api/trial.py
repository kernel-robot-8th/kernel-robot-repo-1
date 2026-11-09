"""Trial UI projection and local IPC; never imports MQTT or owns robot execution."""

import itertools
import json
import math
import secrets
import socket
import time
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from threading import Lock
from uuid import uuid4

from .display import ROBOTS

PHASES = {
    "READY": "출발 대기",
    "to_loading": "창고 상차장으로 이동",
    "WAIT_LOAD": "OMX 창고 상차 완료 승인 대기",
    "to_delivery": "인도장으로 이동",
    "WAIT_INBOUND": "OMX 입고 완료 승인 대기",
    "return_warehouse": "물류 핑키 창고 복귀",
    "WAIT_REQUEST": "물품 요청 대기",
    "cart_to_delivery": "스마트카트 인도장 진입",
    "WAIT_CART_LOAD": "OMX 카트 적재 완료 승인 대기",
    "cart_to_switch": "N12로 이동",
    "mode_switch": "Nav2 전환 확인",
    "navigate": "공용공간 목표로 이동",
    "COMPLETE": "시험 완료",
    "HOLD": "시험 보류",
}


class TrialView:
    def __init__(
        self,
        socket_path,
        token,
        grid,
        *,
        virtual_reset_path=None,
    ):
        if not token or len(token) < 24:
            raise ValueError("ROPORT_TRIAL_TOKEN requires at least 24 characters")
        if "layout" not in grid:
            raise ValueError("trial UI requires a measured map and layout")
        self.socket_path, self.token = str(socket_path), token
        self.session = str(uuid4())
        self.revisions = itertools.count(1)
        self.last = None
        self.last_positions = {}
        self.reset_path = Path(virtual_reset_path) if virtual_reset_path else None
        self._snapshot_lock = Lock()
        self._next_snapshot = 0.0
        self._available = False
        self.grid = grid
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
            and config["map_id"] == self.map_id
            and config["map_version"] == self.map_version
            and config["graph_version"] == self.graph_version
            and {k: sorted(v) for k, v in config["nodes"].items()} == self.graph
        )

    def command(self, request, *, secure_transport=False):
        if not isinstance(request, dict) or "expires_at" in request:
            raise ValueError("invalid trial request")
        plan = request.get("plan")
        if isinstance(plan, dict) and isinstance(plan.get("steps"), list):
            for step in plan["steps"]:
                if isinstance(step, dict) and step.get("kind") == "navigate":
                    payload = step.get("payload")
                    pose = payload.get("pose") if isinstance(payload, dict) else None
                    if not isinstance(pose, dict):
                        raise ValueError("Nav2 goal requires a pose object")
                    for i, axis, size in ((0, "x", "width"), (1, "y", "height")):
                        value = pose.get(axis)
                        if (
                            type(value) not in (int, float)
                            or not math.isfinite(value)
                            or not self.grid["origin"][i]
                            <= value
                            < self.grid["origin"][i]
                            + self.grid[size] * self.grid["resolution"]
                        ):
                            raise ValueError("Nav2 goal outside the configured map")
        with self._snapshot_lock:
            try:
                snapshot = self.rpc({"action": "snapshot"})
                if snapshot["config"]["mode"] == "hardware" and not secure_transport:
                    raise PermissionError(
                        "실물 조작은 로컬 주소 또는 HTTPS에서만 가능합니다."
                    )
                if request.get("action") == "reset":
                    if (
                        not self.reset_path
                        or snapshot["config"]["mode"] != "simulation"
                    ):
                        raise ValueError(
                            "가상 핑키 실행기로 시작한 시험만 초기화할 수 있습니다."
                        )
                    if set(request) != {"action", "session_id", "task_id", "phase"} or (
                        request["session_id"] != snapshot["session_id"]
                        or request["task_id"] != snapshot["state"]["task_id"]
                    ):
                        raise ValueError(
                            "시험이 변경되었습니다. 최신 화면에서 다시 조작하세요."
                        )
                    try:
                        with self.reset_path.open("x") as file:
                            json.dump(request, file)
                    except FileExistsError:
                        pass  # The supervisor consumes one reset per run, including retries.
                    return {"reset_pending": True}
                if self.reset_path and self.reset_path.exists():
                    raise ValueError("가상 시험 초기화 중입니다. 새 연결을 기다리세요.")
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
            custom = data.get("custom_plan", False)
            plan = data.get("plan")
            steps = plan["steps"] if plan else []
            index = data.get("plan_index", 0)
            current = steps[index] if index < len(steps) else None
            label = (
                "N12 주행 완료 · Nav2 제외"
                if phase == "COMPLETE" and scope == "line-only" and not custom
                else PHASES.get(phase, phase)
            )
            if custom and current and phase == current["id"]:
                label = current["label"]
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
                pose_age, pose_stale, pose_live, pose_key = None, True, False, None
                identity = (
                    device["binding_id"],
                    status.get("boot_id"),
                    config["map_version"],
                )
                previous = self.last_positions.get(robot)
                if previous and (previous["identity"] != identity or not matches):
                    self.last_positions.pop(robot)
                    previous = None
                node = status.get("node_id")
                if matches and status.get("mode") == "line" and node in self.nodes:
                    pose = [*self.nodes[node], 0, status["heading_rad"]]
                    source = "마지막 확인 노드 기준 · 연속 위치 아님"
                completed = next(
                    (
                        event
                        for event in reversed(list(state["completed"].values()))
                        if event["type"] == "navigate.completed"
                        and event["source"] == robot
                    ),
                    None,
                )
                if (
                    matches
                    and phase == "COMPLETE"
                    and completed
                    and completed["source"] == robot
                ):
                    p = completed["payload"]["pose"]
                    pose, source = [p["x"], p["y"], 0, p["yaw"]], "Nav2 완료 응답 위치"
                    pose_age = max(
                        0,
                        (
                            datetime.now(UTC)
                            - datetime.fromisoformat(
                                completed["payload"]["observed_at"]
                            )
                        ).total_seconds(),
                    )
                motion = data.get("motion", {}).get(robot) if matches else None
                if motion and not all(
                    self.grid["origin"][i]
                    <= motion["pose"][axis]
                    <= self.grid["origin"][i]
                    + self.grid[size] * self.grid["resolution"]
                    for i, axis, size in ((0, "x", "width"), (1, "y", "height"))
                ):
                    motion = None
                    if pose_age is None:
                        pose = None
                if (
                    motion
                    and pose_age is not None
                    and (
                        datetime.fromisoformat(motion["observed_at"])
                        < datetime.fromisoformat(completed["payload"]["observed_at"])
                    )
                ):
                    motion = None
                if motion:
                    p = motion["pose"]
                    pose = [p["x"], p["y"], p["z"], p["yaw"]]
                    source = {
                        "tf": "TF 지도 위치",
                        "line_estimate": "라인 주행 추정 위치",
                        "simulation": "가상 센서 위치",
                    }.get(motion["pose_source"], motion["pose_source"])
                    pose_age = motion["age"] + time.monotonic() - received
                    pose_stale = not available or motion["stale"] or pose_age > 1
                    pose_live = not pose_stale
                    pose_key = f"{motion['boot_id']}:{motion['sequence']}"
                if pose is not None:
                    if pose_age is None:
                        # A node is a confirmed landmark, not a continuous observation.
                        same = (
                            previous
                            and previous["pose"] == pose
                            and previous["source"] == source
                        )
                        pose_age = (
                            (time.monotonic() - previous["at"] + previous["age"])
                            if same
                            else (age or 0)
                        )
                    self.last_positions[robot] = {
                        "identity": identity,
                        "pose": pose,
                        "source": source,
                        "age": pose_age,
                        "at": time.monotonic(),
                        "key": pose_key,
                    }
                elif previous and matches:
                    pose, source = previous["pose"], previous["source"]
                    pose_age = previous["age"] + time.monotonic() - previous["at"]
                    pose_key = previous["key"]
                if pose_stale and pose is not None:
                    source += " · 마지막 확인 위치"
                rows.append(
                    {
                        "robot_id": robot,
                        "role": role,
                        "pose": pose,
                        "pose_source": source,
                        "pose_age": pose_age,
                        "pose_stale": pose_stale,
                        "pose_live": pose_live,
                        "pose_key": pose_key,
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
            trial = {
                "reset_available": bool(self.reset_path)
                and config["mode"] == "simulation",
                "can_reset": available
                and config["mode"] == "simulation"
                and bool(self.reset_path)
                and not self.reset_path.exists(),
                "reset_pending": bool(self.reset_path and self.reset_path.exists()),
                "session_id": data["session_id"],
                "plan": plan,
                "plan_revision": data.get("plan_revision"),
                "custom_plan": custom,
                "plan_index": index,
                "current_kind": current["kind"] if current else None,
                "catalogue": {
                    **data.get("catalogue", {}),
                    "node_positions": self.nodes,
                },
                "can_configure": available
                and matches
                and phase in {"READY", "COMPLETE"},
                "task_id": state["task_id"],
                "phase": phase,
                "label": label,
                "scope": scope,
                "scenario": config["scenario"],
                "smartcart_only": data.get("smartcart_only", False),
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
                and current is not None
                and current["kind"] == "gate"
                and phase == current["id"],
                "can_request": safe_wait
                and current is not None
                and current["kind"] == "request"
                and phase == current["id"]
                and not data["request_error"],
                "request_error": data["request_error"]
                if current is not None and current["kind"] == "request"
                else None,
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
                    "progress": round((index / len(steps) if steps else 0) * 100),
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
            "task_source": f"{len(config['robots'])}대 시나리오 · 실제 수신 응답 기준"
            if data
            else "시험 설정 응답 대기",
            "errors": [e for e in (error, trial and trial["fault"]) if e],
            "message": "위치 출처·관측 경과 확인 · 지연 시 마지막 위치 유지 · 중단은 보류/lease 갱신 중단",
        }
