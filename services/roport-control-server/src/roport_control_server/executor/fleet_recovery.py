"""Explicit operator reconciliation; never fabricates a robot completion."""

import math
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from uuid import uuid4

from roport_control_server.adapters.mqtt import field_wire as wire

from .fleet import PARKING, ROLES


def stop_current(stop, boot, now):
    return (
        stop["payload"]["expected_boot_version"] == boot
        and (now - wire.timestamp(stop["sent_at"])).total_seconds() < 3
        and now < wire.timestamp(stop["expires_at"])
    )


def new_stop(control_id, active, boot, *, reason="control_hold"):
    return wire.build_message(
        "robot_stop_command",
        control_id,
        active["target"],
        {
            "expected_boot_version": boot,
            "target_command_id": active["command_id"],
            "reason": reason,
        },
        command_id=str(uuid4()),
        expires_at=(datetime.now(UTC) + timedelta(seconds=3)).isoformat(),
    )


def recover_stop(control, command_id, stop_id, boot_version, note):
    if (
        not all(
            isinstance(value, str) and value.strip()
            for value in (command_id, stop_id, boot_version, note)
        )
        or not control.data["fault"]
        or control.data["enabled"]
    ):
        raise ValueError("held stop and operator note required")
    data = deepcopy(control.data)
    active = data["commands"].get(command_id, {})
    stop = active.get("stop")
    if not stop or stop["command_id"] != stop_id:
        raise ValueError("pending stop changed")
    if stop_current(stop, boot_version, datetime.now(UTC)):
        raise ValueError("current stop can still complete")
    robot = stop["target"]
    p = control.observed.get(robot, {})
    state = p.get("state", {})
    if p.get("boot_version") != boot_version or control.state_age(robot) > 3:
        raise ValueError("fresh state for the requested boot required")

    ongoing = p.get("active_command_id")
    if ongoing == command_id and (
        boot_version == stop["payload"]["expected_boot_version"]
        or (
            robot in ROLES
            and state.get("fault") == "recovery_required"
            and state.get("stopped") is True
            and not state.get("line_enabled")
            and state.get("drive_owner") == "none"
        )
    ):
        active["stop"] = new_stop(
            control.config["control_id"], active["message"], boot_version
        )
        active.pop("stop_published", None)
        active.pop("manual_stop", None)
    elif (
        ongoing is None
        and state.get("stopped") is True
        and not state.get("line_enabled")
        and (robot not in ROLES or state.get("drive_owner") == "none")
    ):
        active["manual_stop"] = {"boot_version": boot_version, "note": note.strip()}
    else:
        raise ValueError("matching active command or stopped idle state required")
    data.setdefault("stop_recoveries", {})[stop_id] = {
        "boot_version": boot_version,
        "note": note.strip(),
        "stop_id": active["stop"]["command_id"],
    }
    control.save(data)


def request_host_release(control, request_id, hold_command_id, operator, confirmed):
    if not isinstance(request_id, str) or not request_id:
        raise ValueError("release request ID required")
    data = deepcopy(control.data)
    records = data.setdefault("host_releases", {})
    if request_id in records:
        old = records[request_id]["command"]["payload"]
        if (old["hold_command_id"], old["operator"], old["confirmed"]) != (
            hold_command_id,
            operator,
            confirmed,
        ):
            raise ValueError("release request ID conflict")
        return
    observed = control.observed.get("shelf-01", {})
    failed = data["commands"].get(hold_command_id, {}).get("message", {})
    if (
        not data["fault"]
        or data["enabled"]
        or not observed.get("state", {}).get("fault")
        or control.state_age("shelf-01") > 3
        or failed.get("target") != "shelf-01"
        or confirmed != {"arms": True, "box": True, "pinky": True}
    ):
        raise ValueError("current shelf HOLD and all operator confirmations required")
    if request_id in data["commands"] or request_id == hold_command_id:
        raise ValueError("release requires a new command ID")
    cmd = wire.build_message(
        "hold_release_command",
        control.config["control_id"],
        "shelf-01",
        {
            "expected_boot_version": observed["boot_version"],
            "hold_command_id": hold_command_id,
            "operator": operator,
            "confirmed": confirmed,
        },
        command_id=request_id,
        expires_at=(datetime.now(UTC) + timedelta(seconds=5)).isoformat(),
    )
    records[request_id] = {"command": cmd, "status": "pending"}
    control.save(data)


def _n16_evidence(control, active, task, result, confirmed_boot):
    robot = active["message"]["target"]
    if (
        active["message"]["type"] != "line_route_command"
        or result["status"] != "failed"
        or result.get("reason") != "line_lost"
        or task["stage"] != "warehouse_route"
        or task["request"]["operation"] != "inbound"
        or task["robot"] != robot
        or task.get("slot")
        or task.get("departure")
    ):
        raise ValueError(
            "상차 전 입고 경로에서 line_lost로 실패한 명령만 복구할 수 있습니다."
        )
    report = control.observed[robot]
    state = report["state"]
    seen = wire.timestamp(report.get("observed_at"))
    if (
        report["boot_version"] != confirmed_boot
        or seen <= wire.timestamp(result["observed_at"])
        or not -1 <= (datetime.now(UTC) - seen).total_seconds() <= 3
        or state.get("mode") != "line"
        or state.get("node") != "N16"
        or report.get("active_command_id", True) is not None
        or state.get("fault", True) is not None
        or state.get("line_enabled") is not False
        or any(
            state.get(k) != control.config[k]
            or control.data["field_versions"].get(k) != control.config[k]
            for k in ("map_version", "graph_version")
        )
    ):
        raise ValueError("현재 기동에서 실패 이후의 최신 N16 정차 상태가 필요합니다.")
    positions = control.config.get("node_positions", {})
    if not all(n in positions for n in ("N14", "N16")):
        raise ValueError("N16·N14 지도 좌표가 필요합니다.")
    x, y = positions["N16"]
    dx, dy = positions["N14"][0] - x, positions["N14"][1] - y
    heading = state.get("heading_rad")
    if (
        not (dx or dy)
        or type(heading) not in (int, float)
        or not math.isfinite(heading)
        or abs(math.remainder(heading - math.atan2(dy, dx), math.tau)) > 0.35
    ):
        raise ValueError("N16에서 N14를 향한 기체 방향을 확인하세요.")
    if "N16" in control.occupied(robot) or any(
        control.data["resources"].get(key) not in (None, active["task"])
        for key in ("node:N16", "parking:N16")
    ):
        raise ValueError("N16을 다른 기체가 점유하고 있습니다.")
    return {
        "confirmed_boot": confirmed_boot,
        "node": "N16",
        "heading_rad": heading,
        "observed_at": report["observed_at"],
    }


def reconcile(control, command_id, outcome, note, *, confirmed_boot=None):
    if (
        not isinstance(command_id, str)
        or not command_id
        or not isinstance(outcome, str)
        or outcome not in {"completed", "not_started", "removed", "repositioned_n16"}
        or (outcome == "repositioned_n16" and not confirmed_boot)
        or not isinstance(note, str)
        or not note.strip()
    ):
        raise ValueError("recovery outcome and operator note required")
    old = control.data.get("recoveries", {}).get(command_id)
    if old:
        if (
            old["outcome"] != outcome
            or old["note"] != note
            or old.get("confirmed_boot") != confirmed_boot
        ):
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
    ended_before_stop = (
        bool(terminal)
        and active["message"]["target"] in ROLES
        and any(
            e.get("command_id") == stop["command_id"]
            and e["payload"]["status"] == "rejected"
            and e["payload"].get("reason") == "stop_target_mismatch"
            for e in control.data["events"].values()
        )
        if stop
        else False
    )
    manual_stop = active.get("manual_stop")
    operator_stopped = (
        manual_stop
        and outcome == "removed"
        and (
            control.observed.get(stop["target"], {}).get("boot_version")
            == manual_stop["boot_version"]
        )
    )
    if stop and stop_event is None and not ended_before_stop and not operator_stopped:
        raise ValueError("requested stop is not confirmed")
    if terminal:
        event = max(terminal, key=lambda e: e["payload"]["sequence"])
    elif outcome == "removed" and (stop_event or operator_stopped):
        event = stop_event
    else:
        raise ValueError("command outcome still unknown; reconcile device first")
    for robot in active["participants"]:
        p = control.observed.get(robot, {})
        s = p.get("state", {})
        if (
            control.state_age(robot) > 3
            or s.get("stopped") is not True
            or p.get("active_command_id")
            or s.get("fault")
        ):
            if outcome == "repositioned_n16":
                raise ValueError(
                    f"{robot}의 최신 정지·유휴 상태와 기체 오류 해제를 확인하세요."
                )
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
    p = event["payload"] if event else {}
    evidence = {}
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
    elif outcome == "repositioned_n16":
        if not event:
            raise ValueError("line loss evidence required")
        evidence = _n16_evidence(control, active, task, p, confirmed_boot)
        task.update(status="RUNNING", reason=None)
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
        "event_id": event["message_id"] if event else None,
        "fault": data["fault"],
        **evidence,
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
