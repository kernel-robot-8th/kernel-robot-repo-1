"""Projection of the owner's shared state; the browser never dispatches MQTT."""

import time
from datetime import UTC, datetime

from .display import ROBOTS
from .trial import TrialView


class FleetView(TrialView):
    def _check_map(self, data):
        if (
            data["map_version"] != self.map_version
            or data["graph_version"] != self.graph_version
        ):
            raise ValueError("fleet/map mismatch")

    def bootstrap(self):
        self._check_map(self.rpc({"action": "snapshot"}))
        return {"fleet": True, "trial": False}

    def command(self, request, *, secure_transport=False):
        data = self.rpc({"action": "snapshot"})
        if data["mode"] == "hardware" and not secure_transport:
            raise PermissionError("실물 조작은 로컬 주소 또는 HTTPS에서만 가능합니다.")
        if not isinstance(request, dict) or "expires_at" in request:
            raise ValueError("invalid fleet request")
        if request.get("action") != "stop":
            self._check_map(data)
        return self.rpc({**request, "expires_at": time.time() + 0.8})

    def snapshot(self):
        try:
            data = self.rpc({"action": "snapshot"})
            self.last = data
            available = True
        except (OSError, ValueError):
            data = self.last
            available = False
        rows, tasks = [], []
        for robot in ROBOTS:
            rid = robot["id"]
            report = (data or {}).get("robots", {}).get(rid, {})
            state = report.get("state", {})
            age = (data or {}).get("ages", {}).get(rid)
            sample = (data or {}).get("poses", {}).get(rid)
            pose = None
            source = "위치 미수신"
            pose_age = None
            live = bool(sample and sample["pose"] is not None)
            last_node = (data or {}).get("last_nodes", {}).get(rid)
            current_node = state.get("node")
            fallback_node = None
            if (
                not live
                and current_node not in self.nodes
                and state.get("mode") != "nav2"
                and (data or {}).get("map_version") == self.map_version
            ):
                fallback_node = last_node
            if sample and sample["pose"]:
                p = sample["pose"]
                pose = [p["x"], p["y"], 0, p["yaw"]]
                source = sample["pose_source"]
                pose_age = (
                    datetime.now(UTC) - datetime.fromisoformat(sample["observed_at"])
                ).total_seconds()
            elif current_node in self.nodes:
                pose = [*self.nodes[current_node], 0, state.get("heading_rad", 0)]
                source = "노드 상태"
            elif fallback_node in self.nodes:
                pose = [*self.nodes[fallback_node], 0, 0]
                source = f"마지막 확인 노드 {fallback_node} · 방향 미확인"
            stale = not available or age is None or age > 3
            rows.append(
                {
                    "robot_id": rid,
                    "role": (data or {}).get("roles", {}).get(rid),
                    "pose": pose,
                    "pose_source": source,
                    "pose_age": pose_age,
                    "pose_key": f"{report.get('boot_version')}:{sample and sample['sequence']}",
                    "pose_stale": stale
                    or (not live and current_node not in self.nodes)
                    or (pose_age is not None and pose_age > 1),
                    "pose_live": live,
                    "joints": {},
                    "velocity": None,
                    "age": age,
                    "stale": stale,
                    "status": {
                        **state,
                        "node_id": state.get("node"),
                        "boot_id": report.get("boot_version"),
                    },
                    "mode": " · ".join(
                        str(v) for v in (state.get("mode"), state.get("node")) if v
                    )
                    or (
                        "작업 중"
                        if report.get("active_command_id")
                        else "준비 완료"
                        if state.get("ready")
                        else "상태 대기"
                    ),
                }
            )
        if data:
            tasks = [
                {
                    "task_id": key,
                    "robot": t.get("robot", "미배정"),
                    "stage": t["stage"],
                    "progress": 100 if t["status"] == "COMPLETE" else 0,
                    "status": t.get("reason") or t["status"],
                }
                for key, t in data["tasks"].items()
            ]
        return {
            "session_id": self.session,
            "revision": next(self.revisions),
            "robots": rows,
            "tasks": tasks,
            "demo": False,
            "fleet": {**(data or {}), "available": available},
            "source": "공유 관제 실행부" if available else "실행부 연결 끊김",
            "task_source": "요청별 배차 · 6칸 선반 · 공유 통행 예약",
            "errors": [(data or {}).get("fault")] if (data or {}).get("fault") else [],
            "message": "작업 결과·물품 보관·고객 수령을 각각 확인합니다.",
        }
