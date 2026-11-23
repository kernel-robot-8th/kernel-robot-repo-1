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
import logging
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from .cooperative import ARMS, OPERATIONS, SLOT_ARMS, SLOTS, Rejected, StartRefused
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
# Unloading needs the camera-only Pinky check after the evaluation (panel 3.15: "pinky_check"),
# read at the commanded slot (3.17: "slot") and against this mission's own empty-tray photo taken
# right before it (3.20: tray_reference_state, pre_mission_match). Without that photo a changed
# tray proves nothing (2026-11-21 review: tray_changed alone let the Pinky leave).
PANEL_MIN_VERSION_UNLOAD = (3, 20, 0)
# 3.17.0: the start takes the commanded "slot" ("2B") and "pinky_id", and reports
# last_result.placement (the box seen in its slot) right after the evaluation, before the alignment.
PANEL_COMMAND_SLOT = (3, 17, 0)
# 3.18.0: "end_on_place" ends the loading evaluation once the box sits in its slot and both hands are
# clear of it; the aligner then takes the arms where they are (hand-over). Older panels ignore it.
PANEL_END_ON_PLACE = (3, 18, 0)
# 3.20.0: the placement carries the evidence for an early COMPLETE (control agreement 2026-11-18):
# the hands' state at that moment and the camera's own capture time (not the panel's read time),
# marked observed_clock "capture" (a "control_step" time or no mark never counts).
PANEL_PLACEMENT_EVIDENCE = (3, 20, 0)
# An early COMPLETE uses a capture at most this old (seconds, same PC clock).
PLACEMENT_FRESH_S = 3.0
# package_id is the box's ArUco marker number as a string (agreed with control, same rule as OMX3's
# omx3/target.py): ASCII digits, no leading zero. Package markers 15-22 except 18, the Pinky tray
# marker since docs/aruco/markers.yaml v1.6. A route may narrow it to the boxes its model learned.
PACKAGE_MARKERS = frozenset({15, 16, 17, 19, 20, 21, 22})
_MARKER_NUMBER = re.compile(r"[1-9]?[0-9]")


def package_marker(package_id) -> int:
    """The box marker named by the command's package_id; anything else is refused unmoved."""
    if not isinstance(package_id, str) or not _MARKER_NUMBER.fullmatch(package_id):
        raise Rejected("INVALID_TARGET")
    marker = int(package_id)
    if marker not in PACKAGE_MARKERS:
        raise Rejected("INVALID_TARGET")
    return marker


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


def _handover_hold(result: dict) -> bool:
    """The aligner took the arms over (hand-over) and then found no safe way home: it left them
    where they are with the torque on (real_runner result handover_align "hold", exit 2). Panel
    3.21.0 still closes that as an ordinary alignment, so nobody may send the arms home now."""
    align = result.get("align")
    return isinstance(align, dict) and align.get("handover_align") == "hold"


def _hold_code(result: dict) -> str:
    """stage "hold" stays until a person acts: emergency stop, or release found no path back."""
    align = result.get("align") if isinstance(result.get("align"), dict) else {}
    if "비상 정지" in (align.get("stopped_by"), result.get("stopped_by")):
        return "PANEL_EMERGENCY_STOP"
    if _handover_hold(result):
        return "PANEL_HANDOVER_HOLD"
    if align.get("released") is False:
        return "PANEL_ALIGN_RELEASE_FAILED"
    return "PANEL_HOLD"


# Shelf loading mission (agreed 2026-11-16); the box alignment is part of it:
#   loaded and aligned                            → MISSION_COMPLETE (evidence "shelf_align")
#   on the shelf, not aligned in time, arms home   → MISSION_COMPLETE_NOT_ALIGNED
#     (also when the aligner could not read the shelf but the evaluation's placement judge read
#      this box's own marker in 2B with the hands clear: align_error PANEL_SHELF_UNREADABLE,
#      2026-11-22)
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
    "PANEL_PINKY_NO_REFERENCE",
    "PANEL_SLOT_UNREADABLE",
    "PANEL_PINKY_CHECK_NOT_RUN",
    "PANEL_PINKY_CHECK_CONFLICT",
    "PANEL_NOT_ARMED_FOR_HOME",
    "PANEL_HANDOVER_FAILED",
    "PANEL_HANDOVER_STOPPED",
    "PANEL_HANDOVER_HOLD",
    "RESULT_UNAVAILABLE",
}
HOME_RETURN_TIMEOUT_S = 90.0
# The panel takes episodes of 3-600 s. A control-tower evaluation ends on its own (end_on_return:
# the arms are back; end_on_place: the box is placed and the hands are clear); the limit only
# bounds a run that does neither. The default must leave room inside the control tower's task
# limit (command_timeout_s 300 s) for the alignment (up to 60 s) and the return home, or a late but
# good result arrives after the control tower has already held (#134 review). A route may set
# 3-600 s explicitly (longer only with a matching control limit).
EPISODE_MIN_S, EPISODE_MAX_S = 3.0, 600.0
EPISODE_DEFAULT_S = 180.0


# Unloading mission (shelf → waiting Pinky; completion rule agreed 2026-11-17): judged only by the
# cameras after the evaluation, no time limit. The panel (pinky_check) reports
#   slot_empty — the commanded shelf slot holds no box any more (shelf-facing camera, front_2);
#                only this slot's own reading counts, never the shelf-wide on_shelf
#   on_pinky   — the box is on the Pinky tray (top camera, front): the box top marker is read inside
#                the tray area, or else the tray differs from the empty-tray photo the panel took
#                right before this mission (tray_changed >= 0.25; tray_reference_state "ok" and
#                pre_mission_match true). No marker is stuck on the tray: the unload policy was
#                trained on a bare tray and does not start when one is there.
# Both true → MISSION_COMPLETE (evidence "pinky_load"). Box still in the slot, or slot empty but the
# tray unchanged (dropped) → MISSION_FAILED, arms sent home. Anything unread or not confirmed
# (another slot, no pre-mission photo) → MISSION_HOLD.
PINKY_LOAD = "pinky_load"


def _released_at_placement(placement: dict) -> bool:
    """Both hands let the box go when it was seen in its slot (control agreement 2026-11-18).

    Hand-over judge (source "eval_judge"): both fingertips behind the box front and the shelf
    edge by 3 cm (panel tips_clear, from the joints). Otherwise the evaluation ended on its
    return: both arms at the model's home with grippers at or below GRIP_RELEASED_MAX.
    """
    if placement.get("source") == "eval_judge":
        return placement.get("tips_clear") is True
    end = placement.get("eval_end")
    if not isinstance(end, dict) or end.get("all_at_home") is not True:
        return False
    arms = end.get("arms") if isinstance(end.get("arms"), dict) else {}
    grips = [(arm or {}).get("gripper") for arm in arms.values()]
    return len(grips) == len(ARMS) and all(
        isinstance(g, (int, float)) and g <= GRIP_RELEASED_MAX for g in grips
    )


class PanelRefused(RuntimeError):
    """The panel answered with an HTTP error; reply keeps its JSON body."""

    def __init__(self, message: str, reply: dict):
        super().__init__(message)
        self.reply = reply


def _slot_empty(pinky: dict, slot: str | None) -> bool | None:
    """The commanded slot's own reading; None when it is missing or about another slot.

    The shelf-wide on_shelf is never used in its place (2026-11-21 review): it says nothing about
    the commanded slot.
    """
    value = pinky.get("slot_empty")
    if not isinstance(value, bool):
        return None
    if slot is None or str(pinky.get("slot") or "").lower() != slot.lower():
        return None
    return value


def _pinky_error(pinky, slot: str | None) -> str | None:
    if not isinstance(pinky, dict):  # panel older than the check, or it never reported
        return "PANEL_PINKY_CHECK_NOT_RUN"
    on_pinky, slot_empty = pinky.get("on_pinky"), _slot_empty(pinky, slot)
    if slot_empty is False:
        # a box seen on the tray while it still sits in its slot: a camera is wrong
        return (
            "PANEL_PINKY_CHECK_CONFLICT"
            if on_pinky is True
            else "PANEL_BOX_STILL_ON_SHELF"
        )
    if slot_empty is None:
        return "PANEL_SLOT_UNREADABLE"
    # panel 3.20: the tray is compared only with this mission's own photo taken right before the
    # unloading (never another Pinky's or a shared one); unless the panel confirms it, the tray
    # result is not trusted (a missing field holds too)
    if (
        pinky.get("tray_reference_state") != "ok"
        or pinky.get("pre_mission_match") is not True
    ):
        return "PANEL_PINKY_NO_REFERENCE"
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


# The evaluation's placement judge (panel handover_judge.py, 1117e) watches only shelf 2B: its top
# camera area and shelf edge are those of 2B (handover_fk.json "top_roi", "front_y_2b").
JUDGE_SLOTS = frozenset({"2B"})


def _judge_saw_box(result: dict, box: int) -> bool:
    """The judge itself saw this mission's box marker sitting still in 2B when it said "placed".

    The panel (3.18+) gives the judge only the commanded marker (place_judge.json box_ids
    [box]); the judge's "box" is that marker as the top camera read it (id, flat top, shelf-height
    size inside the 2B area, still for 0.5 s) and is copied unchanged into
    last_result.handover.judge.box. Unlike placement.box (the number the evaluation was started
    with), this is an observation (#134 review, 2026-11-22).
    """
    handover = result.get("handover")
    if not isinstance(handover, dict):
        return False
    if handover.get("mode") != "handover" or handover.get("end_reason") != "placed":
        return False
    judge = handover.get("judge")
    seen = judge.get("box") if isinstance(judge, dict) else None
    return (
        isinstance(seen, dict)
        and type(seen.get("id")) is int
        and seen["id"] == box
        and seen.get("flat") is True
        and not seen.get("rejected")
    )


def _judge_placed(result: dict, slot: str | None, box: int | None) -> bool:
    """The evaluation's own placement judge (panel 3.18+, hand-over) saw this mission's box in the
    commanded slot with both hands clear of it.

    Evidence of loading when the aligner could not read the shelf afterwards (camera moved, no
    frame; TK 2026-11-22). Required (#134 review): the slot is one the judge watches (2B), the
    judge's own marker reading is this mission's box (_judge_saw_box), and the hand-over ended on
    "placed" (the judge says so only with the fingertips behind the box front and the shelf edge).
    The panel keeps the judge's report at the top of last_result.placement (tips_clear) or, once
    the aligner wrote its own shelf read, under placement["first"] (source/on_shelf/times only).
    """
    placement = result.get("placement")
    if not isinstance(placement, dict) or slot is None or box is None:
        return False
    if slot.upper() not in JUDGE_SLOTS or not _judge_saw_box(result, box):
        return False
    if placement.get("source") == "eval_judge":
        judged_box = placement.get("box")
        return (
            str(placement.get("slot") or "").upper() == slot.upper()
            and type(judged_box) is int
            and judged_box == box
            and placement.get("on_shelf") is True
            and placement.get("tips_clear") is True
        )
    first = placement.get("first")
    return (
        isinstance(first, dict)
        and first.get("source") == "eval_judge"
        and first.get("on_shelf") is True
    )


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
    # box markers the model learned; None = any package marker
    markers: frozenset[int] | None = None
    # the field profile's names for this model and the arms' calibration (v2 commands carry
    # model_version / calibration_version); None = not declared, v2 refuses the route
    model_version: str | None = None
    calibration_version: str | None = None


def _route_key(task: dict) -> tuple[str, str, str]:
    return (task["operation"], task["slot_id"], task["manipulation_mode"])


def load_routes(path: Path) -> dict[tuple[str, str, str], Route]:
    """{"routes": [{"operation", "slot_id", "model_path", "task", "reset_s"}]}

    Optional per route: "episode_s" (absent or null = EPISODE_DEFAULT_S, inside the control tower's task limit), "manipulation_mode" (the slot's own, COOPERATIVE for column B and DEDICATED
    for A/C), "markers" (box markers the model learned), output_filter, filter_gripper, eval_options,
    "model_version" and "calibration_version" (v2: the exact pair control may command; a top-level
    value is every route's default).
    """
    routes = {}
    doc = json.loads(Path(path).read_text())
    for item in doc["routes"]:
        operation, slot = item["operation"], item["slot_id"]
        if operation not in OPERATIONS.values() or slot not in SLOTS:
            raise ValueError(f"unknown operation/slot in routes: {(operation, slot)}")
        mode = item.get("manipulation_mode") or SLOT_ARMS[slot[1]][0]
        if mode != SLOT_ARMS[slot[1]][0]:
            raise ValueError(
                f"route {(operation, slot)}: slot {slot} is {SLOT_ARMS[slot[1]][0]}"
            )
        key = (operation, slot, mode)
        markers = item.get("markers")
        if markers is not None and (
            not isinstance(markers, list)
            or not markers
            or any(type(m) is not int or m not in PACKAGE_MARKERS for m in markers)
        ):
            raise ValueError(
                f"route {key} markers must be package markers {sorted(PACKAGE_MARKERS)}"
            )
        if key in routes:
            raise ValueError(f"duplicate route: {key}")
        episode_s = item.get("episode_s")
        episode_s = EPISODE_DEFAULT_S if episode_s is None else float(episode_s)
        if not EPISODE_MIN_S <= episode_s <= EPISODE_MAX_S:
            raise ValueError(
                f"route {key} episode_s must be {EPISODE_MIN_S:g}-{EPISODE_MAX_S:g} s "
                f"(leave it out for {EPISODE_DEFAULT_S:g} s)"
            )
        route = Route(
            str(item["model_path"]),
            str(item["task"]),
            episode_s,
            float(item["reset_s"]),
            item.get("output_filter") or None,
            bool(item.get("filter_gripper", False)),
            dict(item.get("eval_options") or {}) or None,
            frozenset(markers) if markers is not None else None,
            item.get("model_version", doc.get("model_version")),
            item.get("calibration_version", doc.get("calibration_version")),
        )
        for name in ("model_version", "calibration_version"):
            value = getattr(route, name)
            if value is not None and (not isinstance(value, str) or not value.strip()):
                raise ValueError(f"route {key} {name} must be a non-empty string")
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
        routes: dict[tuple[str, str, str], Route],
        *,
        box_marker: int | None = None,  # unused: the command's package_id names the box
        base_url: str = "http://127.0.0.1:8865",
        timeout: float = 5.0,
        start_timeout: float = 300.0,
        report_placement: bool = False,
        end_on_place: bool = False,
        place_shadow: bool = False,
    ):
        self._arms_read: dict | None = None  # bus read behind the last arm flags
        self.routes, self.box_marker = routes, box_marker
        self.box: int | None = None  # the running task's box marker
        # Loading in two reports: COMPLETE once the box is in its slot (Pinky may leave), then the
        # alignment result as a progress event. Needs a control tower that accepts "shelf_placed".
        self.report_placement = report_placement
        self.end_on_place = end_on_place
        # panel 3.18: run the placement judge during loading and only record it (no hand-over)
        self.place_shadow = place_shadow
        self._version: tuple[int, ...] | None = None  # from the last readiness check
        self._placed = False
        self.base_url = base_url.rstrip("/")
        self.timeout, self.start_timeout = timeout, start_timeout
        self.repo_id: str | None = None
        self.slot: str | None = None
        self.operation: str | None = None
        self._missing_since: float | None = None
        self._recovery: dict | None = None  # a failed mission's arms on their way home
        self._models: tuple[float, set[str]] = (float("-inf"), set())
        self._models_refresh: threading.Thread | None = None

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
                reply = json.loads(exc.read())
            except ValueError:
                reply = {}
            reply = reply if isinstance(reply, dict) else {}
            message = reply.get("error") or str(exc)
            raise PanelRefused(f"panel {path}: {message}", reply) from exc

    def readiness(self) -> dict:
        if time.monotonic() - self._models[0] > MODELS_TTL_S:
            self._refresh_models()  # kept warm off the MQTT loop (the sampler calls this)
        status = self._call("GET", "/api/status")
        armed = status.get("armed")  # absent before panel 3.8 → None → not armed
        busy = bool((status.get("job") or {}).get("running"))
        version = _version(status.get("version"))
        self._version = version
        unload = any(key[0] == "OUTBOUND" for key in self.routes)
        minimum = PANEL_MIN_VERSION_UNLOAD if unload else PANEL_MIN_VERSION
        if version is None or version < minimum:
            need = ".".join(map(str, minimum))
            error = f"PANEL_VERSION: {status.get('version')} (needs {need}+)"
        else:
            error = _ai_link_error(status, time.time())
        self._arms_read = None
        arms = {} if busy else self._arms_now()
        return {
            "armed": armed if isinstance(armed, bool) else None,
            "busy": busy,
            "error": error,
            "arms": arms,
            # the bus read behind those flags (panel "at" + joints per arm); v2 compares two of
            # them to see the arms standing still after a stop
            "arms_read": self._arms_read,
        }

    def _arms_now(self) -> dict:
        """Live arm flags from panel /api/arms (3.11+), checked against every routed model's home.

        Grippers count as released at or below GRIP_RELEASED_MAX (the safe stop leaves them fully
        closed on nothing).
        Any doubt (older panel, read error, unknown home) leaves the flag None: never ready by guess.
        """
        merged: dict[str, dict] = {}
        read = None
        for model in sorted({r.model_path for r in self.routes.values()}):
            try:
                seen = self._call(
                    "GET",
                    "/api/arms?model=" + urllib.parse.quote(model),
                    timeout=ARMS_TIMEOUT_S,
                )
            except Exception:  # noqa: BLE001 - panel before 3.11 or unreachable: unknown
                return {}
            if seen.get("source") == "bus" and isinstance(seen.get("at"), str):
                joints = {
                    name: dict(arm["joints"])
                    for name, arm in _by_side(seen.get("arms")).items()
                    if isinstance(arm.get("joints"), dict)
                }
                # one cached bus read serves every model; a different "at" is a different read
                read = (
                    {"at": seen["at"], "joints": joints}
                    if read is None or read["at"] == seen["at"]
                    else None
                )
            else:
                read = None
            for name, arm in _by_side(seen.get("arms")).items():
                flags = _live_flags(arm)
                prev = merged.get(name)
                if prev is not None and prev["returned"] is not flags["returned"]:
                    # at one model's home but not another's: not returned for every route
                    both = (prev["returned"], flags["returned"])
                    flags["returned"] = False if False in both else None
                merged[name] = flags
        self._arms_read = read
        return merged

    def check(self, task: dict) -> None:
        """Refuse before acceptance: no route, no box marker or the model is not trained yet."""
        route = self.routes.get(_route_key(task))
        if route is None:
            raise Rejected("UNSUPPORTED_SLOT")
        if task["manipulation_mode"] != "COOPERATIVE":
            # A/C columns (one arm): panel 3.21 has no single-arm control-tower evaluation and
            # would run its default arm set, both arms with a cooperative model (2026-11-21
            # review). Refused before acceptance even when a route is configured.
            raise Rejected("UNSUPPORTED_SLOT")
        marker = package_marker(task["package_id"])  # INVALID_TARGET
        if route.markers is not None and marker not in route.markers:
            raise Rejected(
                "MODEL_NOT_READY"
            )  # this slot's model never learned that box
        try:
            models = self._model_paths()
        except Exception as exc:
            raise Rejected("EXECUTOR_ERROR") from exc
        if route.model_path not in models:
            raise Rejected("MODEL_NOT_READY")

    def versions(self, task: dict) -> dict | None:
        """The model/calibration pair this task's route was set up with (v2 compares it with the
        command's); None when the route declares none."""
        route = self.routes.get(_route_key(task))
        if route is None or not route.model_version or not route.calibration_version:
            return None
        return {
            "model_version": route.model_version,
            "calibration_version": route.calibration_version,
        }

    def _model_paths(self) -> set[str]:
        """The panel's trained models. Only the very first call waits for the panel (about 1 s);
        later an expired list is refreshed in the background and the known list answers now, so a
        transfer's checks never stall the Host's MQTT loop (2026-11-23 23:41 field run)."""
        fetched, paths = self._models
        if fetched == float("-inf"):
            return self._fetch_models()
        if time.monotonic() - fetched > MODELS_TTL_S:
            self._refresh_models()
        return paths

    def _fetch_models(self) -> set[str]:
        runs = self._call("GET", "/api/train/runs", timeout=15).get("runs", [])
        paths = {r.get("model_path") for r in runs if r.get("model_path")}
        self._models = (time.monotonic(), paths)
        return paths

    def _refresh_models(self) -> None:
        if self._models_refresh is not None and self._models_refresh.is_alive():
            return

        def run() -> None:
            try:
                self._fetch_models()
            except Exception as exc:  # noqa: BLE001 - the known list stays until a refresh works
                logging.getLogger(__name__).warning(
                    "panel model list refresh failed: %s", exc
                )

        self._models_refresh = threading.Thread(
            target=run, name="panel-models", daemon=True
        )
        self._models_refresh.start()

    def start(self, task: dict) -> None:
        if (
            task["manipulation_mode"] != "COOPERATIVE"
        ):  # never reached past check(); no arm move
            raise StartRefused(
                "UNSUPPORTED_SLOT", "one-arm slots are not run by the panel yet"
            )
        route = self.routes[_route_key(task)]
        box = package_marker(task["package_id"])
        body = {
            "source": "mqtt",
            "model_path": route.model_path,
            "task": route.task.replace("{box}", str(box)),
            "episodes": 1,
            "episode_s": route.episode_s,
            "reset_s": route.reset_s,
            "target_marker": box,
            # panel 3.12+: end the episode once the arms are back at the start pose with grippers
            # released, so the policy does not start the task again before episode_s runs out
            "end_on_return": True,
        }
        if task["operation"] == "OUTBOUND":
            # Unloading ends with the cameras checking the box on the Pinky tray, never with a
            # shelf alignment (the panel refuses both at once).
            body |= {"shelf_align": False, "pinky_check": True}
            # The box number is for the Pinky check only: the unloading model was trained without the
            # target circle the panel would otherwise draw on the top camera (panel option, 3.17+).
            body["draw_target"] = False
        else:
            # The shelf loading mission ends with the box aligned on the shelf: ask the panel
            # (3.14+) to align after the evaluation even when the model is not in its own list.
            body["shelf_align"] = True
            # panel 3.14+: when the episode ended on its detected return with no stop or error,
            # skip the slow end-of-run safe stop (home move, wrist, gripper close; ~14 s) and just
            # release torque, because the aligner takes the arms right after. Any other end stays
            # a full stop. Unloading has no aligner after it, so it keeps the full safe stop.
            body["fast_release"] = True
        version = self._version or ()
        if version >= PANEL_COMMAND_SLOT or task["operation"] == "OUTBOUND":
            # the panel aligns and checks at the commanded slot and that Pinky's tray reference
            # (unloading always: it needs panel 3.20 for the pre-mission tray photo)
            body |= {"slot": task["slot_id"].upper(), "pinky_id": task.get("pinky_id")}
        if task["operation"] == "INBOUND" and version >= PANEL_END_ON_PLACE:
            if self.end_on_place:
                body["end_on_place"] = True
            elif self.place_shadow:
                body["place_shadow"] = True
        body |= route.eval_options or {}
        if route.output_filter:
            body |= {
                "output_filter": route.output_filter,
                "filter_gripper": route.filter_gripper,
            }
        check_expiry(task)
        try:
            reply = self._call(
                "POST", "/api/eval/start", body, timeout=self.start_timeout
            )
        except PanelRefused as exc:
            check = exc.reply.get("tray_check")
            if exc.reply.get("refused") == "tray_check" and isinstance(check, dict):
                # panel 3.20: the Pinky tray right before unloading was not empty or could not be
                # read; nothing moved, so the scene is checked without holding the Host
                state = check.get("tray_check")
                code = (
                    "PANEL_TRAY_NOT_EMPTY"
                    if state == "not_empty"
                    else "PANEL_TRAY_CHECK_FAILED"
                )
                why = check.get("not_empty_why") or state
                raise StartRefused(code, f"tray check: {why}", hold=False) from exc
            raise
        if not isinstance(reply.get("repo_id"), str) or not reply["repo_id"]:
            raise RuntimeError("panel did not return the run's repo_id")
        self.repo_id, self._missing_since = reply["repo_id"], None
        self.slot, self._recovery = task["slot_id"], None
        self.operation, self.box = task["operation"], box
        self._placed = False

    def stop(self) -> None:
        """Stop the running evaluation the way the panel's [중지] does (v2 robot_stop_command).

        The panel turns armed off and lets the run land the arms (safe stop); observe() then
        reports the run as failed with operator_stop once it ended. A person re-arms the panel.
        """
        self._call("POST", "/api/stop", {})

    def _placement(self, result) -> dict | None:
        """The box seen in the commanded slot while the alignment still runs (panel 3.17+)."""
        if (
            not self.report_placement
            or self.operation != "INBOUND"
            or self._placed
            or not isinstance(result, dict)
            or result.get("repo_id") != self.repo_id
            or result.get("final")
            is not False  # finished at once: one report as before
            or result.get("stage") == "hold"
            or _handover_hold(result)
        ):
            return None
        placement = result.get("placement")
        if not isinstance(placement, dict) or placement.get("on_shelf") is not True:
            return None
        if str(placement.get("slot") or "").lower() != str(self.slot or "").lower():
            return None  # seen at another slot: the final result decides
        if (self._version or ()) < PANEL_PLACEMENT_EVIDENCE:
            return None  # no gripper evidence or capture time yet: one report as before
        if not _released_at_placement(placement):
            return None  # the hands may still hold the box; a later reading may show it
        captured = placement.get("observed_ts")
        if (
            not isinstance(captured, (int, float))
            or not 0 <= time.time() - captured <= PLACEMENT_FRESH_S
        ):
            return None  # too old (or unknown) to let the Pinky leave on it
        if placement.get("observed_clock") != "capture":
            # panel 3.20 falls back to the control step's time ("control_step") when the camera
            # stamp is unknown, and older readers do not say: only a real capture time counts
            # (2026-11-21 review)
            return None
        self._placed = True
        keys = (
            "slot",
            "box",
            "source",
            "observed_at",
            "observed_ts",
            "observed_clock",
            "reported_at",
            "tips_clear",
            "tip_y",
            "y_out",
            "gripper",
            "eval_end",
        )
        return {
            "state": "placed",
            "grasp_released": True,
            "placement": {k: placement.get(k) for k in keys if k in placement},
        }

    def observe(self) -> dict:
        if self.repo_id is None:
            return {"state": "idle"}
        status = self._call("GET", "/api/status")
        if self._recovery is not None:
            return self._recovering(status)
        placed = self._placement(status.get("last_result"))
        if placed is not None:
            return placed
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
            # waits for a person; report it now and never send the arms home from here
            if result.get("stage") == "hold" or _handover_hold(result):
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
            observation = self._result(result, self.slot, self.operation, self.box)
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
        result: dict,
        slot: str | None = None,
        operation: str | None = None,
        box: int | None = None,
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
            grasping = _grasping(arm)
            if use_rest:
                far = (rest_arms.get(name) or {}).get("far_joints")
                known = rest.get("check") in {"ok", "far"} and isinstance(far, list)
                home = rest.get("check") == "ok" and not far if known else None
                # the grippers after the alignment too: with a hand-over (panel 3.18) the eval
                # snapshot is the moment of release, grippers opened wide (11/21: 60 > 54.6)
                after = _grasping(rest_arms.get(name) or {})
                if after is not None:  # an older rest check has no gripper value
                    grasping = after
            arms[name] = {
                "connected": True if arm.get("joints") else None,
                "grasping": grasping,
                "returned": home if isinstance(home, bool) else None,
                "safe": safe,
            }
        stopped_by = result.get("stopped_by")
        if operation == "OUTBOUND" and stopped_by not in STOPS:
            # finished, timed out or ended on an error: the cameras decide
            error = _pinky_error(result.get("pinky"), slot)
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
            handover = result.get("handover")
            handover = handover if isinstance(handover, dict) else {}
            if (
                error == "PANEL_ALIGN_SKIPPED"
                and handover.get("state") in {"failed", "stopped"}
                and handover.get("fallback") == "home"
            ):
                # panel 3.18 hand-over: the aligner never took the arms, the panel brought them
                # home instead; the box was seen placed at the judge moment (no shelf read later)
                error = "PANEL_HANDOVER_" + handover["state"].upper()
                placement = result.get("placement")
                placed = (
                    isinstance(placement, dict) and placement.get("on_shelf") is True
                )
                if handover.get("fallback_ok") is True and placed:
                    return {
                        "state": "finished",
                        "step": "rest",
                        "arms": arms,
                        "evidence": NOT_ALIGNED,
                        "align_error": error,
                    }
                return {"state": "failed", "error": error, "arms": arms}
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
                if align.get("on_shelf") is None and _judge_placed(result, slot, box):
                    # TK 2026-11-22: the aligner could not read the shelf (camera moved, no
                    # frame), but this mission's placement judge read the box's own marker in 2B
                    # with the hands clear: loaded, not aligned. Completed (Pinky may leave) only if the
                    # arms are home with grippers released (observe checks); "not on the shelf"
                    # (on_shelf false) still fails.
                    return {
                        "state": "finished",
                        "step": "rest",
                        "arms": arms,
                        "evidence": NOT_ALIGNED,
                        "align_error": error,
                    }
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
