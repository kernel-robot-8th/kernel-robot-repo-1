from copy import deepcopy
from datetime import UTC, datetime

import pytest
from test_fleet import complete, dispatch, finish, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.executor.fleet_recovery import reconcile, release


def held(fleet):
    submit(fleet, "recover")
    cmd = dispatch(fleet)
    fleet.hold("lost reply")
    return cmd


def terminal(fleet, cmd, status):
    message = wire.build_message(
        "command_status",
        cmd["target"],
        cmd["source"],
        {
            "boot_version": cmd["payload"]["expected_boot_version"],
            "observed_at": cmd["sent_at"],
            "sequence": 4,
            "status": status,
            "reason": "test",
            "result": {},
        },
        command_id=cmd["command_id"],
    )
    fleet.result(message)


def test_late_completion_is_reconciled_once_and_requires_separate_enable(fleet):
    cmd = held(fleet)
    complete(fleet, cmd)
    reconcile(fleet, cmd["command_id"], "completed", "checked robot at goal")
    saved = deepcopy(fleet.data)
    reconcile(fleet, cmd["command_id"], "completed", "checked robot at goal")
    assert fleet.data == saved
    assert not fleet.data["commands"] and fleet.data["fault"] == "lost reply"
    assert fleet.data["tasks"]["inbound-recover"]["stage"] == "warehouse_load"
    release(fleet, "all devices checked")
    assert fleet.data["fault"] is None and not fleet.data["enabled"]
    assert fleet.next_action() is None
    fleet.enable()
    assert fleet.next_action()[1][1] == "warehouse_load_command"


def test_unknown_outcome_never_clears_reservations(fleet):
    cmd = held(fleet)
    before = deepcopy(fleet.data)
    with pytest.raises(ValueError, match="unknown"):
        reconcile(fleet, cmd["command_id"], "removed", "checked")
    with pytest.raises(ValueError, match="every"):
        release(fleet, "checked")
    assert fleet.data == before


@pytest.mark.parametrize("issue", ["stale", "moving", "active", "fault", "wrong_node"])
def test_completion_cannot_override_current_device_state(fleet, issue):
    cmd = held(fleet)
    complete(fleet, cmd)
    p = fleet.observed[cmd["target"]]
    if issue == "stale":
        fleet.heard[cmd["target"]] = -10
    elif issue == "moving":
        p["state"]["stopped"] = False
    elif issue == "active":
        p["active_command_id"] = "other"
    elif issue == "fault":
        p["state"]["fault"] = "driver_hold"
    else:
        p["state"]["node"] = "N16"
    before = deepcopy(fleet.data)
    with pytest.raises(ValueError):
        reconcile(fleet, cmd["command_id"], "completed", "checked")
    assert fleet.data == before


def test_device_rejection_may_retry_but_failure_may_not(fleet):
    cmd = held(fleet)
    terminal(fleet, cmd, "failed")
    with pytest.raises(ValueError, match="device-rejected"):
        reconcile(fleet, cmd["command_id"], "not_started", "checked")
    terminal(fleet, cmd, "rejected")
    # Same sequence is ignored, so failure cannot be overwritten by a late rejection.
    with pytest.raises(ValueError, match="device-rejected"):
        reconcile(fleet, cmd["command_id"], "not_started", "checked")


def test_removed_package_cancels_task_without_claiming_customer_receipt(fleet):
    cmd = held(fleet)
    terminal(fleet, cmd, "interrupted")
    reconcile(
        fleet, cmd["command_id"], "removed", "package removed, robot parked at N16"
    )
    assert fleet.data["tasks"]["inbound-recover"]["status"] == "CANCELED"
    assert not fleet.data["commands"] and not fleet.data["resources"]
    release(fleet, "checked")
    submit(fleet, "another")
    fleet.enable()
    assert fleet.next_action()


def test_no_recovery_if_requested_stop_has_not_finished(fleet):
    cmd = held(fleet)
    complete(fleet, cmd)
    data = deepcopy(fleet.data)
    data["commands"][cmd["command_id"]]["stop"] = {"command_id": "stop-1"}
    fleet.save(data)
    with pytest.raises(ValueError, match="stop"):
        reconcile(fleet, cmd["command_id"], "completed", "checked")


def test_shelf_controller_uses_controller_state_and_each_arm_evidence(fleet):
    submit(fleet, "large-recovery")
    for _ in range(20):
        cmd = dispatch(fleet)
        assert cmd is not None
        if cmd["target"] == "shelf-01":
            break
        complete(fleet, cmd, size="large")
    else:
        pytest.fail("cooperative transfer was not dispatched")
    fleet.observed["shelf-01"]["state"] = {
        "ready": True,
        "stopped": True,
        "fault": None,
        "arms": [],
        "docking": None,
    }
    fleet.hold("operator review")
    complete(fleet, cmd, size="large")
    fleet.observed["omx2"]["state"]["home_reached"] = False
    with pytest.raises(ValueError, match="returned/empty"):
        reconcile(fleet, cmd["command_id"], "completed", "checked")
    fleet.observed["omx2"]["state"]["home_reached"] = True
    reconcile(fleet, cmd["command_id"], "completed", "checked")
    assert fleet.data["tasks"]["inbound-large-recovery"]["stage"] == "return_route"


def test_nav_completion_recovery_requires_fresh_matching_current_pose(fleet):
    submit(fleet, "nav-recovery")
    finish(fleet)
    submit(fleet, "nav-recovery", "outbound")
    for _ in range(20):
        cmd = dispatch(fleet)
        assert cmd is not None
        if cmd["type"] == "nav_goal_command":
            break
        complete(fleet, cmd)
    else:
        pytest.fail("navigation goal was not dispatched")
    fleet.hold("operator review")
    complete(fleet, cmd)
    with pytest.raises(ValueError, match="fresh navigation pose"):
        reconcile(fleet, cmd["command_id"], "completed", "checked")
    fleet.poses[cmd["target"]] = {
        "boot_version": fleet.observed[cmd["target"]]["boot_version"],
        "observed_at": datetime.now(UTC).isoformat(),
        "pose": {**cmd["payload"]["pose"], "x": 100},
    }
    with pytest.raises(ValueError, match="tolerance"):
        reconcile(fleet, cmd["command_id"], "completed", "checked")
    fleet.poses[cmd["target"]]["pose"] = cmd["payload"]["pose"]
    reconcile(fleet, cmd["command_id"], "completed", "checked")
    assert fleet.data["tasks"]["outbound-nav-recovery"]["stage"] == "await_receipt"


def test_unknown_pre_restart_command_can_only_end_after_current_boot_stop_and_removal(
    fleet,
):
    from roport_control_server.executor.fleet_runtime import FleetRuntime

    cmd = held(fleet)
    robot = cmd["target"]
    fleet.observed[robot]["boot_version"] = "replacement-boot"
    stop = wire.build_message(
        "robot_stop_command",
        cmd["source"],
        robot,
        {
            "expected_boot_version": "replacement-boot",
            "target_command_id": cmd["command_id"],
            "reason": "operator_request",
        },
        command_id="replacement-stop",
        expires_at=cmd["expires_at"],
    )
    data = deepcopy(fleet.data)
    data["commands"][cmd["command_id"]]["stop"] = stop
    fleet.save(data)
    event = wire.build_message(
        "command_status",
        robot,
        cmd["source"],
        {
            "boot_version": "replacement-boot",
            "observed_at": stop["sent_at"],
            "sequence": 1,
            "status": "completed",
            "result": {"stopped": True},
        },
        command_id=stop["command_id"],
    )
    FleetRuntime(fleet, None).stored_result(event)
    for outcome in ("completed", "not_started"):
        with pytest.raises(ValueError, match="unknown"):
            reconcile(fleet, cmd["command_id"], outcome, "checked")
    reconcile(
        fleet, cmd["command_id"], "removed", "package removed, restarted robot parked"
    )
    assert fleet.data["tasks"]["inbound-recover"]["status"] == "CANCELED"
    assert (
        fleet.data["recoveries"][cmd["command_id"]]["event_id"] == event["message_id"]
    )
    release(fleet, "checked")
    assert not fleet.data["enabled"]


@pytest.mark.parametrize(
    "command_id,outcome", [([], "removed"), ("id", []), ("", "completed")]
)
def test_malformed_operator_fields_are_rejected_without_state_changes(
    fleet, command_id, outcome
):
    before = deepcopy(fleet.data)
    with pytest.raises(ValueError):
        reconcile(fleet, command_id, outcome, "checked")
    assert fleet.data == before


@pytest.mark.parametrize("issue", [None, "moving", "active", "stale"])
def test_terminal_then_stop_rejection_requires_fresh_idle(fleet, issue):
    cmd = held(fleet)
    complete(fleet, cmd)
    data = deepcopy(fleet.data)
    stop = wire.build_message(
        "robot_stop_command",
        cmd["source"],
        cmd["target"],
        {
            "expected_boot_version": cmd["payload"]["expected_boot_version"],
            "target_command_id": cmd["command_id"],
            "reason": "control_hold",
        },
        command_id="late-stop",
        expires_at=cmd["expires_at"],
    )
    data["commands"][cmd["command_id"]]["stop"] = stop
    fleet.save(data)
    message = wire.build_message(
        "command_status",
        cmd["target"],
        cmd["source"],
        {
            "boot_version": cmd["payload"]["expected_boot_version"],
            "observed_at": cmd["sent_at"],
            "sequence": 99,
            "status": "rejected",
            "reason": "stop_target_mismatch",
            "result": {},
        },
        command_id=stop["command_id"],
    )
    from roport_control_server.executor.fleet_runtime import FleetRuntime

    FleetRuntime(fleet, None).stored_result(message)
    if issue == "moving":
        fleet.observed[cmd["target"]]["state"]["stopped"] = False
    if issue == "active":
        fleet.observed[cmd["target"]]["active_command_id"] = "another"
    if issue == "stale":
        fleet.heard[cmd["target"]] = -10
    if issue:
        with pytest.raises(ValueError, match="fresh stopped idle"):
            reconcile(fleet, cmd["command_id"], "completed", "checked")
    else:
        reconcile(fleet, cmd["command_id"], "completed", "checked")
        assert not fleet.data["commands"]
