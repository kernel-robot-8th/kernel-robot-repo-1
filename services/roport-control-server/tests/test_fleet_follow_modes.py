"""Customer intent, robot registration, and operator motion permission are separate."""

from copy import deepcopy
from datetime import UTC, datetime, timedelta
from uuid import uuid4

import pytest
from test_fleet import dispatch
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.executor.fleet import FleetControl


def expiry(seconds=60):
    return (datetime.now(UTC) + timedelta(seconds=seconds)).isoformat()


@pytest.fixture
def waiting(fleet):
    fleet.config["follow_modes"] = {"pinky3": ["marker", "appearance"]}
    task = {
        "request": {
            "request_id": "outbound",
            "operation": "outbound",
            "package_id": "box",
            "goal": "gate",
        },
        "stage": "await_receipt",
        "status": "AWAITING_RECEIPT",
        "reason": None,
        "robot": "pinky3",
        "robot_boot": "pinky3-boot",
        "parking": "H1",
    }
    data = deepcopy(fleet.data)
    data["tasks"]["outbound"] = task
    fleet.save(data)
    fleet.observed["pinky3"]["state"].update(mode="nav2", node=None)
    fleet.observed["pinky3"]["observed_at"] = expiry(0)
    return fleet


def request(fleet, operation, **extra):
    return {
        "receipt_id": "receipt",
        "order_id": "order",
        "reservation_id": "reservation",
        "robot_id": "pinky3",
        "request_id": str(uuid4()),
        "operation": operation,
        "expected_revision": fleet.data["tasks"]["outbound"]["follow"]["revision"],
        **extra,
    }


def result(fleet, command, status, result=None, *, sequence=1):
    event = wire.build_message(
        "command_status",
        "pinky3",
        fleet.config["control_id"],
        {
            "boot_version": "pinky3-boot",
            "sequence": sequence,
            "status": status,
            "observed_at": expiry(0),
            **({"result": result} if result else {}),
        },
        command_id=command["command_id"],
    )
    fleet.result(event)
    return event


def fresh(fleet):
    p = deepcopy(fleet.observed["pinky3"])
    p.update(sequence=p["sequence"] + 1, observed_at=expiry(0), active_command_id=None)
    fleet.observe({"source": "pinky3", "payload": p})


def registered(fleet):
    fleet.receipt("outbound", follow_mode="appearance", session_id="receipt")
    capture = request(fleet, "capture")
    fleet.follow_request(capture)
    cmd = dispatch(fleet)
    result(
        fleet,
        cmd,
        "completed",
        {
            "stopped": True,
            "follow_session_id": "receipt",
            "frame_ref": "frame-1",
            "image_ref": "image-1",
            "expires_at": expiry(),
            "perception_boot_id": "perception-1",
            "candidates": [{"candidate_id": "person-1", "box": [0.1, 0.1, 0.8, 0.9]}],
        },
    )
    fresh(fleet)
    select = request(fleet, "select", frame_ref="frame-1", candidate_id="person-1")
    fleet.follow_request(select)
    cmd = dispatch(fleet)
    result(
        fleet,
        cmd,
        "completed",
        {
            "stopped": True,
            "follow_session_id": "receipt",
            "target_ref": "target-1",
            "expires_at": expiry(),
            "perception_boot_id": "perception-1",
        },
    )
    fresh(fleet)
    return capture, select


def test_appearance_receipt_waits_without_return_or_motion_and_replays_once(waiting):
    c = waiting
    c.receipt("outbound", follow_mode="appearance", session_id="receipt")
    before = deepcopy(c.data)
    c.receipt("outbound", follow_mode="appearance", session_id="receipt")
    assert c.data == before
    assert c.data["tasks"]["outbound"]["stage"] == "follow_register"
    assert c.next_action() is None
    with pytest.raises(ValueError):
        c.follow_control("outbound", "approve")


def test_registration_dedup_approval_and_actual_active_evidence(waiting):
    c = waiting
    capture, select = registered(c)
    task = c.data["tasks"]["outbound"]
    assert task["stage"] == "follow_pending"
    assert task["follow"]["registration"]["target_ref"] == "target-1"
    assert c.next_action() is None
    before = deepcopy(c.data)
    c.follow_request(capture)
    c.follow_request(select)
    assert c.data == before
    with pytest.raises(ValueError, match="reused"):
        c.follow_request({**select, "candidate_id": "another"})
    c.follow_control("outbound", "approve")
    cmd = dispatch(c)
    assert (
        cmd["payload"]["target_ref"] == "target-1" and "card_id" not in cmd["payload"]
    )
    result(c, cmd, "accepted")
    assert c.data["tasks"]["outbound"]["stage"] == "follow_start"
    result(c, cmd, "in_progress", {"follow_active": False}, sequence=2)
    assert c.data["tasks"]["outbound"]["stage"] == "follow_start"
    with pytest.raises(ValueError, match="activation evidence"):
        result(c, cmd, "in_progress", {"follow_active": True}, sequence=3)
    proof = {
        **c.observed["pinky3"]["state"],
        "drive_owner": "nav2",
        "follow_active": True,
        "follow": {
            "active": True,
            "follow_mode": "appearance",
            "follow_session_id": "receipt",
            "target_ref": "target-1",
            "perception_boot_id": "perception-1",
        },
    }
    result(c, cmd, "in_progress", proof, sequence=3)
    assert c.data["tasks"]["outbound"]["stage"] == "following"
    c.follow_request(request(c, "return"))
    assert c.data["tasks"]["outbound"]["stage"] == "follow_stopping"
    assert c.next_action() is None and c.data["commands"]


@pytest.mark.parametrize(
    "reason", ["stale_revision", "expired", "future", "hold", "stale_robot"]
)
def test_delayed_or_unsafe_actions_never_dispatch(waiting, reason):
    c = waiting
    c.receipt("outbound", follow_mode="appearance", session_id="receipt")
    p = request(c, "capture")
    sent_at = expiry(0)
    if reason == "stale_revision":
        p["expected_revision"] += 1
    elif reason == "expired":
        sent_at = expiry(-11)
    elif reason == "future":
        sent_at = expiry(2)
    elif reason == "hold":
        c.hold("operator_stopped")
    else:
        c.heard["pinky3"] = -4
    c.follow_request(p, sent_at=sent_at)
    assert c.data["tasks"]["outbound"]["follow"]["last_request"]["status"] == "rejected"
    assert c.next_action() is None
    before = deepcopy(c.data)
    c.follow_request(p, sent_at=expiry(0))
    assert c.data == before


def test_expired_target_or_owner_restart_cannot_start(waiting):
    c = waiting
    registered(c)
    data = deepcopy(c.data)
    data["tasks"]["outbound"]["follow"]["registration"]["expires_at"] = expiry(-1)
    c.save(data)
    with pytest.raises(ValueError, match="registration_expired"):
        c.follow_control("outbound", "approve")
    restored = FleetControl(c.config, c.store, clock=c.clock)
    assert restored.next_action() is None
    assert restored.data["tasks"]["outbound"]["stage"] == "follow_pending"
    assert not restored.data["enabled"]


def test_selection_expiring_before_dispatch_can_be_recaptured(waiting):
    c = waiting
    registered(c)
    data = deepcopy(c.data)
    task = data["tasks"]["outbound"]
    task.update(stage="follow_registering")
    task["follow"]["pending"] = {
        "operation": "select",
        "frame_ref": "frame-1",
        "candidate_id": "person-1",
    }
    task["follow"]["registration"]["expires_at"] = expiry(-1)
    c.save(data)
    assert c.next_action() is None
    task = c.data["tasks"]["outbound"]
    assert task["stage"] == "follow_register"
    assert task["follow"]["last_request"]["status"] == "failed"
    assert "registration" not in task["follow"] and "pending" not in task["follow"]
    assert set(c.service_projection(task)["follow"]["allowed_actions"]) == {
        "capture",
        "return",
    }


def test_refused_return_does_not_leave_an_automatic_return_intent(waiting):
    c = waiting
    registered(c)
    c.follow_control("outbound", "approve")
    c.observed["pinky3"]["state"]["stopped"] = False
    c.follow_request(request(c, "return"))
    follow = c.data["tasks"]["outbound"]["follow"]
    assert follow["last_request"]["status"] == "rejected"
    assert "return_requested" not in follow
    assert c.data["tasks"]["outbound"]["stage"] == "follow_start"


def test_standing_customer_does_not_make_an_unowned_follower_ready(waiting):
    c = waiting
    c.observed["pinky3"]["state"]["follow"] = {"active": True, "follow_mode": "marker"}
    with pytest.raises(ValueError, match="controller still active"):
        c.ready("pinky3")


def test_marker_wire_compatibility_and_none_return(waiting):
    c = waiting
    c.receipt("outbound", 42, session_id="receipt")
    c.follow_control("outbound", "approve")
    cmd = dispatch(c)
    assert cmd["payload"]["card_id"] == 42
    assert "follow_mode" not in cmd["payload"]


def test_disabled_appearance_and_invalid_card_fail_before_receipt_changes(waiting):
    c = waiting
    c.config.pop("follow_modes")
    before = deepcopy(c.data)
    with pytest.raises(ValueError, match="unavailable"):
        c.receipt("outbound", follow_mode="appearance")
    with pytest.raises(ValueError, match="requires marker"):
        c.receipt("outbound", 42, follow_mode="appearance")
    assert c.data == before
    c.receipt("outbound", follow_mode="none", session_id="receipt")
    assert c.data["tasks"]["outbound"]["stage"] == "return_nav"
    assert c.service_projection(c.data["tasks"]["outbound"])["follow"]["mode"] == "none"
