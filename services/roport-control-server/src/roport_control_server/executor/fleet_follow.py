"""Customer follow sessions; registration and permission never imply motion."""

from copy import deepcopy
from datetime import UTC, datetime

from ..contracts.service_wire import validate_follow_request


def supported_modes(control, robot):
    if (
        robot not in control.data["roles"]
        or control.data["roles"][robot] != "smartcart"
    ):
        return []
    hardware = control.config["mode"] == "hardware"
    if hardware and (
        control.config.get("field_verified") is not True
        or control.config.get("verified_follow_robot") != robot
    ):
        return []
    return control.config.get("follow_modes", {}).get(
        robot, [] if hardware else ["marker"]
    )


def registration_ready(control, task, *, target=False):
    follow = task.get("follow", {})
    registration = follow.get("registration", {})
    observed = control.observed.get(task["robot"], {})
    try:
        expires = datetime.fromisoformat(registration["expires_at"])
        valid = expires.tzinfo and expires > datetime.now(UTC)
    except (KeyError, TypeError, ValueError):
        valid = False
    if (
        not valid
        or follow.get("robot_boot_version") != observed.get("boot_version")
        or (target and not registration.get("target_ref"))
    ):
        raise ValueError("registration_expired: capture and select again")
    return registration


def allowed_actions(control, task):
    follow = task.get("follow")
    robot = task.get("robot")
    if (
        not follow
        or follow["mode"] == "none"
        or not control.data["enabled"]
        or control.data["fault"]
        or task["status"] == "HOLD"
        or control.state_age(robot) > 3
    ):
        return []
    stage = task["stage"]
    if stage in {"follow_start", "following"}:
        return ["pause", "return"]
    if stage not in {"follow_register", "follow_pending", "follow_paused"}:
        return []
    try:
        control.ready(robot)
    except ValueError:
        return []
    actions = ["return"]
    if follow["mode"] not in supported_modes(control, robot):
        return actions
    if follow["mode"] == "appearance":
        actions.append("capture")
        if follow.get("registration"):
            actions.append("release")
            try:
                registration = registration_ready(control, task)
                if registration.get("frame_ref") and stage == "follow_register":
                    actions.append("select")
            except ValueError:
                pass
    if stage == "follow_paused":
        try:
            if follow["mode"] == "appearance":
                registration_ready(control, task, target=True)
            actions.append("resume")
        except ValueError:
            pass
    return actions


def projection(control, task):
    robot = task.get("robot")
    result = {
        "package_id": task["request"]["package_id"],
        "robot": robot,
        "stage": task["stage"],
        "status": "HOLD" if control.data["fault"] else task["status"],
        "supported_follow_modes": supported_modes(control, robot),
        "robot_observed_at": control.observed.get(robot, {}).get("observed_at"),
    }
    if task.get("follow"):
        follow = deepcopy(task["follow"])
        follow.pop("robot_boot_version", None)
        follow.pop("pending", None)
        follow.pop("return_requested", None)
        first_following = follow.pop("first_following_at", None)
        alert_after = control.config.get("boarding_alert_after_s")
        robot_state = control.observed.get(robot, {}).get("state", {})
        observed_follow = robot_state.get("follow", {})
        if (
            alert_after is not None
            and first_following
            and task["stage"] == "following"
            and result["status"] == "RUNNING"
            and control.data["enabled"]
            and control.state_age(robot) <= 3
            and robot_state.get("fault") is None
            and observed_follow.get("active") is True
            and observed_follow.get("follow_mode") == follow["mode"]
            and observed_follow.get("follow_session_id") == follow["session_id"]
            and (
                datetime.now(UTC) - datetime.fromisoformat(first_following)
            ).total_seconds()
            >= alert_after
        ):
            follow["boarding_alert"] = True
        follow.update(
            reason=task.get("reason") or control.data["fault"],
            allowed_actions=allowed_actions(control, task),
            observed_at=result["robot_observed_at"],
        )
        result["follow"] = follow
    return result


def request(control, payload, *, sent_at=None):
    """Persist the first outcome, including refusals, before acknowledging execution."""
    validate_follow_request(payload)
    data = deepcopy(control.data)
    requests = data.setdefault("follow_requests", {})
    previous = requests.get(payload["request_id"])
    if previous:
        if previous["payload"] != payload:
            raise ValueError("follow request ID reused with different contents")
        return
    matches = [
        t
        for t in data["tasks"].values()
        if t.get("follow", {}).get("session_id") == payload["receipt_id"]
        and t.get("robot") == payload["robot_id"]
    ]
    if len(matches) != 1:
        raise ValueError("unknown follow session")
    task = matches[0]
    follow, operation = task["follow"], payload["operation"]
    outcome = {
        "request_id": payload["request_id"],
        "status": "accepted",
        "reason": None,
    }
    try:
        if sent_at is not None:
            age = (datetime.now(UTC) - datetime.fromisoformat(sent_at)).total_seconds()
            if age > 10 or age < -1:
                raise ValueError("follow_request_expired")
        if payload["expected_revision"] != follow["revision"]:
            raise ValueError("stale_follow_revision")
        if operation not in allowed_actions(control, task):
            raise ValueError("follow_action_not_allowed")
        if operation in {"capture", "select", "release"}:
            if operation == "select":
                registration = registration_ready(control, task)
                if payload["frame_ref"] != registration.get("frame_ref"):
                    raise ValueError("stale_frame")
                if "candidate_id" in payload and payload["candidate_id"] not in {
                    c["candidate_id"] for c in registration.get("candidates", [])
                }:
                    raise ValueError("unknown_candidate")
            follow["pending"] = {
                k: payload[k]
                for k in ("operation", "frame_ref", "candidate_id", "box")
                if k in payload
            }
            task.update(stage="follow_registering", reason=None)
        else:
            control.apply_follow_control(data, task, operation)
    except ValueError as exc:
        outcome.update(status="rejected", reason=str(exc))
    follow["last_request"] = outcome
    requests[payload["request_id"]] = {"payload": deepcopy(payload), "outcome": outcome}
    control.save(data)
