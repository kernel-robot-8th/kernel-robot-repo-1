"""Executor that runs shelf-01 COOPERATIVE tasks through the local OMX control panel (v3.14+).

The panel owns the arms, cameras and safety stop. This adapter only calls its HTTP API on this PC:
the panel runs the robot loop and sends each observation to the GPU AI inference server over UDP
(wire RPI1, panel 3.13+), so the Host reports ready only while the panel says that link is up.
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
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from .cooperative import ARMS, OPERATIONS, SLOTS, Rejected
from .mqtt_link.messages import check_expiry

ARM_SIDES = {"left": "omx1", "right": "omx2"}
STOPS = {
    "중지": "PANEL_STOPPED",
    "즉시 중단": "PANEL_ABORTED",
    "비상 정지": "PANEL_EMERGENCY_STOP",
    "강제 종료": "PANEL_KILLED",
}
# Stops a person makes on the panel that end with the arms set down; emergency stop and kill do not.
OPERATOR_STOPS = {"중지", "즉시 중단"}
RESULT_GRACE_S = (
    20.0  # run gone from the job without a matching last_result → unknown result
)
MODELS_TTL_S = 30.0
# Released gripper (2026-11-10 test agreement, model 22 data): at or below 52 + 5 %. An empty gripper
# closes to about 48-51; a box edge can also stop it near 50, which only the arm position tells apart
# (a held box keeps the arm away from its start pose). Open (61-67) or stopped on a box (58-60) is above.
GRIP_RELEASED_MAX = 54.6
ARMS_TIMEOUT_S = 15.0
# /api/arms (3.11), end_on_return (3.12), AI inference over UDP only (3.13) and the "shelf_align"
# start field with alignment after a control-tower evaluation (3.14): an older panel may run every
# episode to episode_s, reach the AI server over HTTP (SSH tunnel), or skip the alignment.
PANEL_MIN_VERSION = (3, 14, 0)
# Unloading needs the camera-only Pinky check after the evaluation (panel 3.15: "pinky_check").
PANEL_MIN_VERSION_UNLOAD = (3, 15, 0)
# The panel PINGs the AI server about every 5 s (up to ~15 s on a slow GPU tunnel) and stamps
# gpu_link.infer.udp.at with its clock (time.time(); same PC as the Host). Older than this → stale.
AI_LINK_FRESH_S = 20.0
OUTPUT_FILTERS = {None, "약", "중", "강"}
# Panel 3.10 enables its async engine for remote inference only when no other run setting is sent,
# so a route with a filter states the async settings itself.
EVAL_OPTIONS = {
    "async_mode",
    "actions_per_chunk",
    "chunk_size_threshold",
    "aggregate_fn",
}


def _by_side(raw) -> dict[str, dict]:
    """Panel arm entries ({"OMX 1": {"side": "left", ...}}) keyed by omx1/omx2."""
    arms = {}
    for arm in (raw if isinstance(raw, dict) else {}).values():
        name = ARM_SIDES.get((arm or {}).get("side"))
        if name in ARMS:
            arms[name] = arm
    return arms


def _version(raw) -> tuple[int, ...] | None:
    try:
        return tuple(int(part) for part in str(raw).split("."))
    except ValueError:
        return None


def _grasping(arm: dict) -> bool | None:
    """Gripper above GRIP_RELEASED_MAX → still grasping (or open); unknown value → None."""
    grip = arm.get("gripper")
    if not isinstance(grip, (int, float)):
        grip = (arm.get("joints") or {}).get("gripper")
    if isinstance(grip, bool) or not isinstance(grip, (int, float)):
        return None
    return grip > GRIP_RELEASED_MAX


def _live_flags(arm: dict) -> dict:
    """Panel /api/arms entry → Host arm flags (see PanelExecutor._arms_now)."""
    home = arm.get("at_home")
    connected = True if arm.get("joints") else None
    return {
        "connected": connected,
        "grasping": _grasping(arm),
        "returned": home if isinstance(home, bool) else None,
        "safe": connected,  # read while no robot program drives the arms
    }


def _ai_link_error(status: dict, now: float) -> str | None:
    """None only when the panel's last UDP check of the AI inference server passed recently.

    ok None (panel starting, or the GPU server has not allowed this PC yet) is not ready: the panel
    itself refuses a start only on ok False, so the Host must not count on that refusal.
    """
    link = (status.get("gpu_link") or {}).get("infer")
    if not isinstance(link, dict):
        return "AI_LINK_UNKNOWN: panel reports no AI inference link (control PC role needed)"
    if link.get("transport") != "udp":
        return f"AI_LINK_NOT_UDP: {link.get('transport')}"
    udp = link.get("udp") if isinstance(link.get("udp"), dict) else {}
    if udp.get("ok") is not True:
        stage = udp.get("stage") or ("starting" if udp.get("ok") is None else "failed")
        return f"AI_LINK_DOWN: {stage} {udp.get('error') or ''}".strip()[:200]
    at = udp.get("at")
    if isinstance(at, bool) or not isinstance(at, (int, float)):
        return "AI_LINK_STALE: no check time"
    if not 0 <= now - at < AI_LINK_FRESH_S:
        return f"AI_LINK_STALE: last check {now - at:.0f} s ago"
    return None


def _hold_code(result: dict) -> str:
    """stage "hold" stays until a person acts: emergency stop, or release found no path back."""
    align = result.get("align") if isinstance(result.get("align"), dict) else {}
    if "비상 정지" in (align.get("stopped_by"), result.get("stopped_by")):
        return "PANEL_EMERGENCY_STOP"
    if align.get("released") is False:
        return "PANEL_ALIGN_RELEASE_FAILED"
    return "PANEL_HOLD"


# Shelf loading mission (agreed 2026-11-16); the box alignment is part of it:
#   loaded and aligned                            → MISSION_COMPLETE (evidence "shelf_align")
#   on the shelf, not aligned in time, arms home   → MISSION_COMPLETE_NOT_ALIGNED
#   not loaded, evaluation failed, arms not home  → MISSION_FAILED: arms sent back to the start pose,
#                                                   reported without HOLD once they are home
#   shelf unreadable, emergency stop, unknown, …  → MISSION_HOLD: a person checks first
NOT_ALIGNED = "shelf_load_not_aligned"
# Failures that wait for a person; any other failure first sends the arms home (panel /api/home/return).
HOLD_ERRORS = {
    "PANEL_EMERGENCY_STOP",
    "PANEL_KILLED",
    "PANEL_HOLD",
    "PANEL_ALIGN_RELEASE_FAILED",
    "PANEL_SHELF_UNREADABLE",
    "PANEL_ALIGN_SKIPPED",
    "PANEL_ALIGN_NOT_RUN",
    "PANEL_PINKY_UNREADABLE",
    "PANEL_SLOT_UNREADABLE",
    "PANEL_PINKY_CHECK_NOT_RUN",
    "PANEL_PINKY_CHECK_CONFLICT",
    "PANEL_NOT_ARMED_FOR_HOME",
    "RESULT_UNAVAILABLE",
}
HOME_RETURN_TIMEOUT_S = 90.0


# Unloading mission (shelf → waiting Pinky; completion rule agreed 2026-11-17): judged only by the
# cameras after the evaluation, no time limit. The panel (pinky_check) reports
#   slot_empty — the commanded shelf slot holds no box any more (shelf-facing camera, front_2)
#   on_pinky   — the tray reference marker (ArUco 18) is hidden by 1/3 or more (top camera, front)
# Both true → MISSION_COMPLETE (evidence "pinky_load"). Box still in the slot, or slot empty but the
# tray marker visible (dropped) → MISSION_FAILED, arms sent home. Anything unread → MISSION_HOLD.
PINKY_LOAD = "pinky_load"


def _slot_empty(pinky: dict) -> bool | None:
    """The commanded slot is empty; older pickers only give the shelf-wide on_shelf."""
    if "slot_empty" in pinky:
        value = pinky.get("slot_empty")
    else:
        on_shelf = pinky.get("on_shelf")
        value = None if not isinstance(on_shelf, bool) else not on_shelf
    return value if isinstance(value, bool) else None


def _pinky_error(pinky) -> str | None:
    if not isinstance(pinky, dict):  # panel older than the check, or it never reported
        return "PANEL_PINKY_CHECK_NOT_RUN"
    on_pinky, slot_empty = pinky.get("on_pinky"), _slot_empty(pinky)
    if slot_empty is False:
        # the tray marker covered while the box still sits in its slot: a camera is wrong
        return (
            "PANEL_PINKY_CHECK_CONFLICT"
            if on_pinky is True
            else "PANEL_BOX_STILL_ON_SHELF"
        )
    if slot_empty is None:
        return "PANEL_SLOT_UNREADABLE"
    if on_pinky is True:
        return None
    if on_pinky is False:
        return "PANEL_BOX_DROPPED"
    return "PANEL_PINKY_UNREADABLE"


def _home_and_released(arms: dict) -> bool:
    """The Host's completion test: every arm connected, home, safe and not grasping."""
    want = {"connected": True, "returned": True, "safe": True, "grasping": False}
    return all(
        all((arms.get(name) or {}).get(k) is v for k, v in want.items())
        for name in ARMS
    )


def _align_error(align: dict, slot: str | None) -> str | None:
    if (
        align.get("on_shelf") is False
    ):  # the aligner looked and the box is not on the shelf
        return "PANEL_BOX_NOT_ON_SHELF"
    if align.get("ran") is not True:
        return "PANEL_ALIGN_SKIPPED"
    # The panel aligns at the slot of its own config (one slot per PC), not the commanded one.
    if slot is not None and str(align.get("slot") or "").lower() != slot.lower():
        return "PANEL_ALIGN_WRONG_SLOT"
    if align.get("stopped_by"):
        if align["stopped_by"] == "비상 정지":
            return "PANEL_EMERGENCY_STOP"
        return "PANEL_ALIGN_STOPPED"
    if align.get("released") is False:  # the release after a stop found no way back
        return "PANEL_ALIGN_RELEASE_FAILED"
    if (
        align.get("ok") is not True
    ):  # time limit, pushing blocked, hand height not calibrated, …
        return "PANEL_ALIGN_FAILED"
    return None


@dataclass(frozen=True)
class Route:
    model_path: str  # as listed by the panel's model list (GPU laptop path)
    task: str  # panel task sentence; "{box}" is replaced with the box marker
    episode_s: float
    reset_s: float
    # panel low-pass on joint targets: 약 / 중 / 강, None = off
    output_filter: str | None = None
    # also smooth the gripper (lightly); off keeps grasp timing exact
    filter_gripper: bool = False
    # extra panel eval fields, limited to EVAL_OPTIONS
    eval_options: dict | None = None


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
            item.get("output_filter") or None,
            bool(item.get("filter_gripper", False)),
            dict(item.get("eval_options") or {}) or None,
        )
        if not route.model_path or not route.task:
            raise ValueError(f"route {key} needs model_path and task")
        if route.output_filter not in OUTPUT_FILTERS:
            raise ValueError(f"route {key} output_filter must be one of 약/중/강")
        if set(route.eval_options or {}) - EVAL_OPTIONS:
            raise ValueError(
                f"route {key} eval_options allows only {sorted(EVAL_OPTIONS)}"
            )
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
        self.slot: str | None = None
        self.operation: str | None = None
        self._missing_since: float | None = None
        self._recovery: dict | None = None  # a failed mission's arms on their way home
        self._models: tuple[float, set[str]] = (float("-inf"), set())

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
        busy = bool((status.get("job") or {}).get("running"))
        version = _version(status.get("version"))
        unload = any(op == "OUTBOUND" for op, _ in self.routes)
        minimum = PANEL_MIN_VERSION_UNLOAD if unload else PANEL_MIN_VERSION
        if version is None or version < minimum:
            need = ".".join(map(str, minimum))
            error = f"PANEL_VERSION: {status.get('version')} (needs {need}+)"
        else:
            error = _ai_link_error(status, time.time())
        return {
            "armed": armed if isinstance(armed, bool) else None,
            "busy": busy,
            "error": error,
            "arms": {} if busy else self._arms_now(),
        }

    def _arms_now(self) -> dict:
        """Live arm flags from panel /api/arms (3.11+), checked against every routed model's home.

        Grippers count as released at or below GRIP_RELEASED_MAX (the safe stop leaves them fully
        closed on nothing).
        Any doubt (older panel, read error, unknown home) leaves the flag None: never ready by guess.
        """
        merged: dict[str, dict] = {}
        for model in sorted({r.model_path for r in self.routes.values()}):
            try:
                seen = self._call(
                    "GET",
                    "/api/arms?model=" + urllib.parse.quote(model),
                    timeout=ARMS_TIMEOUT_S,
                )
            except Exception:  # noqa: BLE001 - panel before 3.11 or unreachable: unknown
                return {}
            for name, arm in _by_side(seen.get("arms")).items():
                flags = _live_flags(arm)
                prev = merged.get(name)
                if prev is not None and prev["returned"] is not flags["returned"]:
                    # at one model's home but not another's: not returned for every route
                    both = (prev["returned"], flags["returned"])
                    flags["returned"] = False if False in both else None
                merged[name] = flags
        return merged

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
            # panel 3.12+: end the episode once the arms are back at the start pose with grippers
            # released, so the policy does not start the task again before episode_s runs out
            "end_on_return": True,
        }
        if task["operation"] == "OUTBOUND":
            # Unloading ends with the cameras checking the box on the Pinky tray, never with a
            # shelf alignment (the panel refuses both at once).
            body |= {"shelf_align": False, "pinky_check": True}
        else:
            # The shelf loading mission ends with the box aligned on the shelf: ask the panel
            # (3.14+) to align after the evaluation even when the model is not in its own list.
            body["shelf_align"] = True
        body |= route.eval_options or {}
        if route.output_filter:
            body |= {
                "output_filter": route.output_filter,
                "filter_gripper": route.filter_gripper,
            }
        check_expiry(task)
        reply = self._call("POST", "/api/eval/start", body, timeout=self.start_timeout)
        if not isinstance(reply.get("repo_id"), str) or not reply["repo_id"]:
            raise RuntimeError("panel did not return the run's repo_id")
        self.repo_id, self._missing_since = reply["repo_id"], None
        self.slot, self._recovery = task["slot_id"], None
        self.operation = task["operation"]

    def observe(self) -> dict:
        if self.repo_id is None:
            return {"state": "idle"}
        status = self._call("GET", "/api/status")
        if self._recovery is not None:
            return self._recovering(status)
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
            observation = self._result(result, self.slot, self.operation)
            if observation["state"] == "finished":
                # Evaluation snapshots predate alignment; every completion (aligned or not)
                # also needs the final motor pose: arms home with grippers released.
                live = self._arms_now()
                if self.operation == "OUTBOUND":
                    # judged after the camera check, with the arms at rest: the motors now
                    # (an evaluation that ended on an error left "safe" False in its snapshot)
                    observation["arms"] = {
                        name: dict(live.get(name) or {}) for name in ARMS
                    }
                for name, flags in observation["arms"].items():
                    for flag in ("connected", "returned", "safe"):
                        if flags.get(flag) is True:
                            flags[flag] = live.get(name, {}).get(flag)
                    if flags.get("grasping") is False:
                        flags["grasping"] = live.get(name, {}).get("grasping")
                if not _home_and_released(observation["arms"]):
                    return self._recover("ARM_NOT_RETURNED", status)
                return observation
            if observation.get("operator_stop") or observation["error"] in HOLD_ERRORS:
                return observation
            return self._recover(observation["error"], status)
        # Between the job ending and last_result appearing; give up only after a grace period.
        now = time.monotonic()
        self._missing_since = self._missing_since or now
        if now - self._missing_since < RESULT_GRACE_S:
            return {"state": "running", "step": "finishing"}
        return {"state": "failed", "error": "RESULT_UNAVAILABLE", "arms": {}}

    def _recover(self, error: str, status: dict) -> dict:
        """Mission failed: send both arms back to the start pose, then report it without HOLD."""
        live = self._arms_now()
        if _home_and_released(live):
            return {"state": "failed", "error": error, "arms": live, "recovered": True}
        if status.get("armed") is not True:
            # The panel turned armed off (a stop or an anomaly) or a person did: the Host never
            # moves the arms then, even though the panel's home move does not check armed.
            detail = f"{error}: armed off, arms not moved"
            return {
                "state": "failed",
                "error": "PANEL_NOT_ARMED_FOR_HOME",
                "arms": live,
                "detail": detail,
            }
        try:
            # the run's start pose, grippers opened (panel 3.14: [홈 위치로 이동])
            # "source": "mqtt" lets the panel refuse it while armed is off (its button stays free)
            self._call("POST", "/api/home/return", {"source": "mqtt"})
        except Exception as exc:  # noqa: BLE001 - no way home: a person takes over (HOLD)
            detail = f"home return refused: {exc}"[:200]
            return {"state": "failed", "error": error, "arms": live, "detail": detail}
        self._recovery = {"error": error, "since": time.monotonic()}
        return {"state": "running", "step": "home_return"}

    def _recovering(self, status: dict) -> dict:
        error, since = self._recovery["error"], self._recovery["since"]
        if (status.get("job") or {}).get("running"):
            if time.monotonic() - since < HOME_RETURN_TIMEOUT_S:
                return {"state": "running", "step": "home_return"}
            self._recovery = None
            detail = "home return did not finish"
            return {"state": "failed", "error": error, "arms": {}, "detail": detail}
        self._recovery = None
        live = self._arms_now()  # only the motors tell whether the arms are home now
        recovered = _home_and_released(live)
        return {"state": "failed", "error": error, "arms": live, "recovered": recovered}

    @staticmethod
    def _result(
        result: dict, slot: str | None = None, operation: str | None = None
    ) -> dict:
        exit_ok = result.get("exit_ok") is True
        safe = True if exit_ok else (False if result.get("exit_ok") is False else None)
        align = result.get("align")
        align = align if isinstance(align, dict) else None
        # Arms moved again only when an alignment ran; then its rest check replaces the eval
        # snapshot (necessary, not sufficient: observe() also checks the final live home pose).
        use_rest = align is not None and align.get("ran") is True
        if use_rest:
            rest = align.get("rest") if isinstance(align.get("rest"), dict) else {}
            rest_arms = _by_side(rest.get("arms"))
        arms = {}
        for name, arm in _by_side(result.get("arms")).items():
            home = arm.get("at_home")
            if use_rest:
                far = (rest_arms.get(name) or {}).get("far_joints")
                known = rest.get("check") in {"ok", "far"} and isinstance(far, list)
                home = rest.get("check") == "ok" and not far if known else None
            arms[name] = {
                "connected": True if arm.get("joints") else None,
                "grasping": _grasping(arm),
                "returned": home if isinstance(home, bool) else None,
                "safe": safe,
            }
        stopped_by = result.get("stopped_by")
        if operation == "OUTBOUND" and stopped_by not in STOPS:
            # finished, timed out or ended on an error: the cameras decide
            error = _pinky_error(result.get("pinky"))
            if error is None:
                return {
                    "state": "finished",
                    "step": "pinky",
                    "arms": arms,
                    "evidence": PINKY_LOAD,
                }
            return {"state": "failed", "error": error, "arms": arms}
        if exit_ok:
            if align is None:  # no aligner on this PC: nobody looked at the shelf
                return {"state": "failed", "error": "PANEL_ALIGN_NOT_RUN", "arms": arms}
            error = _align_error(align, slot)
            if error is None:
                return {
                    "state": "finished",
                    "step": "rest",
                    "arms": arms,
                    "evidence": "shelf_align",
                }
            if error in {"PANEL_ALIGN_SKIPPED", "PANEL_ALIGN_FAILED"}:
                if align.get("on_shelf") is True:
                    # Completed only if the arms are home with grippers released (observe checks).
                    return {
                        "state": "finished",
                        "step": "rest",
                        "arms": arms,
                        "evidence": NOT_ALIGNED,
                        "align_error": error,
                    }
                if (
                    error == "PANEL_ALIGN_FAILED"
                ):  # the cameras could not tell where the box is
                    error = "PANEL_SHELF_UNREADABLE"
            stopped = align.get("stopped_by") in OPERATOR_STOPS
            return {
                "state": "failed",
                "error": error,
                "arms": arms,
                "operator_stop": stopped and align.get("released") is True,
            }
        if stopped_by in OPERATOR_STOPS:
            # The stop is the cause; errors raised while shutting down are side effects.
            return {
                "state": "failed",
                "error": STOPS[stopped_by],
                "arms": arms,
                "operator_stop": True,
            }
        err = result.get("err") or {}
        code = err.get("code") or err.get("name")
        error = (
            "PANEL_" + re.sub(r"[^A-Za-z0-9]+", "_", str(code)).strip("_").upper()
            if code
            else STOPS.get(stopped_by, "PANEL_EVAL_FAILED")
        )
        return {"state": "failed", "error": error, "arms": arms}
