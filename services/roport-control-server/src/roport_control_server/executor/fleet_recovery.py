"""Explicit operator reconciliation; never fabricates a robot completion."""

from copy import deepcopy
from datetime import UTC, datetime

from .fleet import PARKING, ROLES


def reconcile(control, command_id, outcome, note):
    if (
        not isinstance(command_id, str)
        or not command_id
        or not isinstance(outcome, str)
        or outcome not in {"completed", "not_started", "removed"}
        or not isinstance(note, str)
        or not note.strip()
    ):
        raise ValueError("recovery outcome and operator note required")
    old = control.data.get("recoveries", {}).get(command_id)
    if old:
        if old["outcome"] != outcome or old["note"] != note:
            raise ValueError("command already reconciled differently")
        return
    active = control.data["commands"].get(command_id)
    if not control.data["fault"] or control.data["enabled"] or not active:
        raise ValueError("held command required")
    events = [
        e for e in control.data["events"].values() if e.get("command_id") == command_id
    ]
    terminal = [
        e
        for e in events
        if e["payload"]["status"] in {"completed", "rejected", "failed", "interrupted"}
    ]
    stop = active.get("stop")
    stop_event = next(
        (
            e
            for e in control.data["events"].values()
            if stop
            and e.get("command_id") == stop["command_id"]
            and e["payload"]["status"] == "completed"
            and e["payload"].get("result", {}).get("stopped") is True
        ),
        None,
    )
    if stop and stop_event is None:
        raise ValueError("requested stop is not confirmed")
    if terminal:
        event = max(terminal, key=lambda e: e["payload"]["sequence"])
    elif outcome == "removed" and stop_event:
        event = (
            stop_event  # Explicit removal can end unknown work after a confirmed stop.
        )
    else:
        raise ValueError("command outcome still unknown; reconcile device first")
    for robot in active["participants"]:
        p = control.observed.get(robot, {})
        s = p.get("state", {})
        if (
            control.clock() - control.heard.get(robot, -float("inf")) > 3
            or s.get("stopped") is not True
            or p.get("active_command_id")
            or s.get("fault")
        ):
            raise ValueError(robot + ": fresh stopped idle state required")
        if robot in ROLES:
            if s.get("mode") == "line" and (
                s.get("line_enabled") or s.get("drive_owner") != "none"
            ):
                raise ValueError(robot + ": line ownership not released")
        elif not (
            s.get("ready") is True
            and (
                robot == "shelf-01"
                or (
                    s.get("home_reached") is True
                    and s.get("holding_state") == "empty"
                    and s.get("holding_package_id") is None
                )
            )
        ):
            raise ValueError(robot + ": arm not returned/empty")
    data = deepcopy(control.data)
    task = data["tasks"][active["task"]]
    p = event["payload"]
    if outcome == "completed":
        if p["status"] != "completed":
            raise ValueError("matching completion evidence required")
        control.check_completion(active["message"], p.get("result", {}), task, active)
        result = p["result"]
        current = control.observed[task["robot"]]["state"]
        if active["message"]["target"] == task["robot"]:
            proof = current
            if active["message"]["type"] == "nav_goal_command":
                pose = control.poses.get(task["robot"], {})
                if (
                    pose.get("boot_version")
                    != control.observed[task["robot"]]["boot_version"]
                    or not pose.get("pose")
                    or not 0
                    <= (
                        datetime.now(UTC) - datetime.fromisoformat(pose["observed_at"])
                    ).total_seconds()
                    <= 3
                ):
                    raise ValueError("fresh navigation pose required")
                proof = {**current, "pose": pose["pose"]}
            control.check_completion(active["message"], proof, task, active)
        task["status"] = "RUNNING"
        control.advance(data, task, active["message"], result)
    elif outcome == "not_started":
        if p["status"] != "rejected":
            raise ValueError("only a device-rejected command can be retried")
        task.update(status="RUNNING", reason=None)
    else:
        current = control.observed[task["robot"]]["state"]
        role = data["roles"][task["robot"]]
        if current.get("mode") != "line" or current.get("node") not in PARKING[role]:
            raise ValueError("return the cleared robot to a parking node first")
        # Operator confirms removal of this package; never edits unrelated shelf stock.
        for slot in data["slots"].values():
            if slot["package_id"] == task["request"]["package_id"]:
                slot.update(state="empty", package_id=None)
        task.update(stage="done", status="CANCELED", reason=note)
    task["robot_boot"] = control.observed[task["robot"]]["boot_version"]
    del data["commands"][command_id]
    data["resources"] = control.reservations(data)
    data.setdefault("recoveries", {})[command_id] = {
        "outcome": outcome,
        "note": note,
        "event_id": event["message_id"],
        "fault": data["fault"],
    }
    control.save(data)


def release(control, note):
    if not isinstance(note, str) or not note.strip():
        raise ValueError("operator note required")
    if not control.data["fault"]:
        return
    if control.data["commands"] or any(
        t["status"] == "HOLD" for t in control.data["tasks"].values()
    ):
        raise ValueError("reconcile every held command first")
    if control.data["field_versions"] != {
        k: control.config[k] for k in ("map_version", "graph_version")
    }:
        raise ValueError("changed field requires a separate migration")
    for robot, role in control.data["roles"].items():
        if role is not None:
            control.ready(robot)
    data = deepcopy(control.data)
    data.setdefault("recovery_releases", []).append(
        {"fault": data["fault"], "note": note}
    )
    data.update(fault=None, enabled=False)
    control.save(data)  # A separate enable action starts work.
