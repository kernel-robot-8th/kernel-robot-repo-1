"""Executor that runs shelf-01 COOPERATIVE tasks through the local OMX control panel (v3.8+).

The panel owns the arms, cameras and safety stop. This adapter only calls its HTTP API on this PC:
a route table picks the trained model for each (operation, slot) because each slot needs its own
policy; tasks without a route are rejected before anything moves. A start needs the panel's
armed switch (turned on by a person on the panel screen) and is sent with "source": "mqtt".
Completion facts come from /api/status last_result of the started run; unknown values stay None.
From panel 3.9 a shelf alignment may follow the evaluation in a separate job: the run is judged
only when last_result is final, and when alignment ran, arm return comes from its rest check.
"""

import json
import re
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from .cooperative import ARMS, OPERATIONS, SLOTS, Rejected

ARM_SIDES = {"left": "omx1", "right": "omx2"}
STOPS = {
    "중지": "PANEL_STOPPED",
    "즉시 중단": "PANEL_ABORTED",
    "비상 정지": "PANEL_EMERGENCY_STOP",
    "강제 종료": "PANEL_KILLED",
}
RESULT_GRACE_S = (
    20.0  # run gone from the job without a matching last_result → unknown result
)
MODELS_TTL_S = 30.0


def _by_side(raw) -> dict[str, dict]:
    """Panel arm entries ({"OMX 1": {"side": "left", ...}}) keyed by omx1/omx2."""
    arms = {}
    for arm in (raw if isinstance(raw, dict) else {}).values():
        name = ARM_SIDES.get((arm or {}).get("side"))
        if name in ARMS:
            arms[name] = arm
    return arms


def _hold_code(result: dict) -> str:
    """stage "hold" stays until a person acts: emergency stop, or release found no path back."""
    align = result.get("align") if isinstance(result.get("align"), dict) else {}
    if "비상 정지" in (align.get("stopped_by"), result.get("stopped_by")):
        return "PANEL_EMERGENCY_STOP"
    if align.get("released") is False:
        return "PANEL_ALIGN_RELEASE_FAILED"
    return "PANEL_HOLD"


def _align_error(align: dict) -> str | None:
    if align.get("ran") is not True:
        return "PANEL_ALIGN_SKIPPED"
    if align.get("stopped_by"):
        if align["stopped_by"] == "비상 정지":
            return "PANEL_EMERGENCY_STOP"
        return "PANEL_ALIGN_STOPPED"
    if align.get("ok") is not True:
        return "PANEL_ALIGN_FAILED"
    return None


@dataclass(frozen=True)
class Route:
    model_path: str  # as listed by the panel's model list (GPU laptop path)
    task: str  # panel task sentence; "{box}" is replaced with the box marker
    episode_s: float
    reset_s: float


def load_routes(path: Path) -> dict[tuple[str, str], Route]:
    """{"routes": [{"operation", "slot_id", "model_path", "task", "episode_s", "reset_s"}]}"""
    routes = {}
    for item in json.loads(Path(path).read_text())["routes"]:
        key = (item["operation"], item["slot_id"])
        if key[0] not in OPERATIONS.values() or key[1] not in SLOTS:
            raise ValueError(f"unknown operation/slot in routes: {key}")
        if key in routes:
            raise ValueError(f"duplicate route: {key}")
        route = Route(
            str(item["model_path"]),
            str(item["task"]),
            float(item["episode_s"]),
            float(item["reset_s"]),
        )
        if not route.model_path or not route.task:
            raise ValueError(f"route {key} needs model_path and task")
        routes[key] = route
    return routes


class PanelExecutor:
    """One evaluation episode per task; the run is matched by the repo_id the panel returns."""

    def __init__(
        self,
        routes: dict[tuple[str, str], Route],
        *,
        box_marker: int | None,
        base_url: str = "http://127.0.0.1:8865",
        timeout: float = 5.0,
        start_timeout: float = 300.0,
    ):
        self.routes, self.box_marker = routes, box_marker
        self.base_url = base_url.rstrip("/")
        self.timeout, self.start_timeout = timeout, start_timeout
        self.repo_id: str | None = None
        self._missing_since: float | None = None
        self._models: tuple[float, set[str]] = (0.0, set())

    def _call(self, method: str, path: str, body: dict | None = None, timeout=None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(
            self.base_url + path,
            data=data,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout or self.timeout) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                message = json.loads(exc.read()).get("error") or str(exc)
            except ValueError:
                message = str(exc)
            raise RuntimeError(f"panel {path}: {message}") from exc

    def readiness(self) -> dict:
        status = self._call("GET", "/api/status")
        armed = status.get("armed")  # absent before panel 3.8 → None → not armed
        return {
            "armed": armed if isinstance(armed, bool) else None,
            "busy": bool((status.get("job") or {}).get("running")),
            "error": None,
            "arms": {},
        }

    def check(self, task: dict) -> None:
        """Refuse before acceptance: no route, no box marker or the model is not trained yet."""
        route = self.routes.get((task["operation"], task["slot_id"]))
        if route is None:
            raise Rejected("UNSUPPORTED_SLOT")
        if self.box_marker is None:
            raise Rejected("BOX_MARKER_UNKNOWN")
        try:
            models = self._model_paths()
        except Exception as exc:
            raise Rejected("EXECUTOR_ERROR") from exc
        if route.model_path not in models:
            raise Rejected("MODEL_NOT_READY")

    def _model_paths(self) -> set[str]:
        fetched, paths = self._models
        if time.monotonic() - fetched > MODELS_TTL_S:
            runs = self._call("GET", "/api/train/runs", timeout=15).get("runs", [])
            paths = {r.get("model_path") for r in runs if r.get("model_path")}
            self._models = (time.monotonic(), paths)
        return paths

    def start(self, task: dict) -> None:
        route = self.routes[(task["operation"], task["slot_id"])]
        body = {
            "source": "mqtt",
            "model_path": route.model_path,
            "task": route.task.replace("{box}", str(self.box_marker)),
            "episodes": 1,
            "episode_s": route.episode_s,
            "reset_s": route.reset_s,
            "target_marker": self.box_marker,
        }
        reply = self._call("POST", "/api/eval/start", body, timeout=self.start_timeout)
        if not isinstance(reply.get("repo_id"), str) or not reply["repo_id"]:
            raise RuntimeError("panel did not return the run's repo_id")
        self.repo_id, self._missing_since = reply["repo_id"], None

    def observe(self) -> dict:
        if self.repo_id is None:
            return {"state": "idle"}
        status = self._call("GET", "/api/status")
        job = status.get("job") or {}
        info = job.get("info") or {}
        if job.get("running") and info.get("repo_id") == self.repo_id:
            self._missing_since = None
            phase = info.get("phase")
            return {
                "state": "running",
                "step": phase if isinstance(phase, str) else None,
            }
        result = status.get("last_result")
        if isinstance(result, dict) and result.get("repo_id") == self.repo_id:
            self._missing_since = None
            if result.get("stage") == "hold":  # waits for a person; report it now
                return {"state": "failed", "error": _hold_code(result), "arms": {}}
            if result.get("final", True) is not True:  # absent before panel 3.9
                running = (
                    job.get("running") and info.get("eval_repo_id") == self.repo_id
                )
                step = info.get("phase") if running else result.get("stage")
                return {
                    "state": "running",
                    "step": step if isinstance(step, str) else None,
                }
            return self._result(result)
        # Between the job ending and last_result appearing; give up only after a grace period.
        now = time.monotonic()
        self._missing_since = self._missing_since or now
        if now - self._missing_since < RESULT_GRACE_S:
            return {"state": "running", "step": "finishing"}
        return {"state": "failed", "error": "RESULT_UNAVAILABLE", "arms": {}}

    @staticmethod
    def _result(result: dict) -> dict:
        exit_ok = result.get("exit_ok") is True
        safe = True if exit_ok else (False if result.get("exit_ok") is False else None)
        align = result.get("align")
        align = align if isinstance(align, dict) else None
        if align is not None:
            # Alignment ends at the evaluation start pose with grippers closed: at_home does not
            # apply, so return is the rest check; gripper_open is still read at evaluation end.
            rest = align.get("rest") if isinstance(align.get("rest"), dict) else {}
            rest_arms = _by_side(rest.get("arms"))
        arms = {}
        for name, arm in _by_side(result.get("arms")).items():
            opened, home = arm.get("gripper_open"), arm.get("at_home")
            if align is not None:
                far = (rest_arms.get(name) or {}).get("far_joints")
                known = rest.get("check") in {"ok", "far"} and isinstance(far, list)
                home = rest.get("check") == "ok" and not far if known else None
            arms[name] = {
                "connected": True if arm.get("joints") else None,
                "grasping": (not opened) if isinstance(opened, bool) else None,
                "returned": home if isinstance(home, bool) else None,
                "safe": safe,
            }
        if exit_ok and align is None:
            return {"state": "finished", "step": "home", "arms": arms}
        if exit_ok:
            error = _align_error(align)
            if error is None:
                return {
                    "state": "finished",
                    "step": "rest",
                    "arms": arms,
                    "evidence": "shelf_align",
                }
            return {"state": "failed", "error": error, "arms": arms}
        err = result.get("err") or {}
        code = err.get("code") or err.get("name")
        error = (
            "PANEL_" + re.sub(r"[^A-Za-z0-9]+", "_", str(code)).strip("_").upper()
            if code
            else STOPS.get(result.get("stopped_by"), "PANEL_EVAL_FAILED")
        )
        return {"state": "failed", "error": error, "arms": arms}
