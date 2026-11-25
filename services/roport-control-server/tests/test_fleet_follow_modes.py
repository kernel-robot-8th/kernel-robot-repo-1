"""Customer intent, robot registration, and operator motion permission are separate."""

from copy import deepcopy
from datetime import UTC, datetime, timedelta
from uuid import uuid4

import pytest
from test_fleet import dispatch
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.executor import fleet_follow
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


def test_registration_dedup_approval_and_actual_active_evidence(waiting, monkeypatch):
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
    task = c.data["tasks"]["outbound"]
    assert task["stage"] == "following"
    started = datetime.fromisoformat(task["follow"]["first_following_at"])

    class Clock(datetime):
        elapsed = 19

        @classmethod
        def now(cls, tz=None):
            return started + timedelta(seconds=cls.elapsed)

    monkeypatch.setattr(fleet_follow, "datetime", Clock)
    c.config["boarding_alert_after_s"] = 20
    assert "boarding_alert" not in c.service_projection(task)["follow"]
    Clock.elapsed = 20
    assert "boarding_alert" not in c.service_projection(task)["follow"]
    robot_state = c.observed["pinky3"]["state"]
    for change in ({"active": False}, {"follow_session_id": "another-receipt"}):
        robot_state["follow"] = {**proof["follow"], **change}
        assert "boarding_alert" not in c.service_projection(task)["follow"]
    robot_state["follow"] = proof["follow"]
    projection = c.service_projection(task)
    assert projection["follow"]["boarding_alert"] is True
    assert "first_following_at" not in projection["follow"]
    assert projection["follow"]["revision"] == task["follow"]["revision"]
    assert (
        "boarding_alert"
        not in c.service_projection({**task, "stage": "follow_paused"})["follow"]
    )
    for elapsed in (21, 30):
        Clock.elapsed = elapsed
        assert c.service_projection(task)["follow"]["boarding_alert"] is True
    with monkeypatch.context() as stale:
        stale.setattr(c, "state_age", lambda robot: 4)
        assert "boarding_alert" not in c.service_projection(task)["follow"]
    del c.config["boarding_alert_after_s"]
    assert "boarding_alert" not in c.service_projection(task)["follow"]
    c.config["boarding_alert_after_s"] = 20
    c.follow_request(request(c, "return"))
    assert c.data["tasks"]["outbound"]["stage"] == "follow_stopping"
    assert (
        "boarding_alert"
        not in c.service_projection(c.data["tasks"]["outbound"])["follow"]
    )
    assert c.next_action() is None and c.data["commands"]


@pytest.mark.parametrize("legacy", [False, True])
def test_marker_goal_and_activation_share_the_receipt_session(waiting, legacy):
    c = waiting
    c.receipt("outbound", follow_card=35, session_id="receipt")
    if legacy:
        data = deepcopy(c.data)
        del data["tasks"]["outbound"]["follow"]
        c.save(data)
    c.follow_control("outbound", "approve")
    cmd = dispatch(c)
    assert cmd["type"] == "follow_target_command"
    assert cmd["payload"]["card_id"] == 35
    result(c, cmd, "accepted")
    proof = {
        **c.observed["pinky3"]["state"],
        "drive_owner": "nav2",
        "follow_active": True,
        "follow": {
            "active": True,
            "follow_mode": "marker",
        },
    }
    if legacy:
        assert "follow_session_id" not in cmd["payload"]
    else:
        assert cmd["payload"]["follow_mode"] == "marker"
        assert cmd["payload"]["follow_session_id"] == "receipt"
        proof["follow"]["follow_session_id"] = "other-receipt"
        with pytest.raises(ValueError, match="follow activation evidence"):
            result(c, cmd, "in_progress", proof, sequence=2)
        assert c.data["tasks"]["outbound"]["stage"] == "follow_start"
        proof["follow"]["follow_session_id"] = "receipt"
    result(c, cmd, "in_progress", proof, sequence=2)
    assert c.data["tasks"]["outbound"]["stage"] == "following"


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


@pytest.mark.parametrize("modes", [None, [], ["appearance"]])
def test_hardware_marker_requires_explicit_mode(waiting, modes):
    c = waiting
    c.config.pop("follow_modes")
    c.config.update(
        mode="hardware", field_verified=True, verified_follow_robot="pinky3"
    )
    if modes is not None:
        c.config["follow_modes"] = {"pinky3": modes}
    task = c.data["tasks"]["outbound"]
    assert c.service_projection(task)["supported_follow_modes"] == (modes or [])
    before = deepcopy(c.data)
    with pytest.raises(ValueError, match="unavailable"):
        c.receipt("outbound", 42, session_id="receipt")
    assert c.data == before
    c.receipt("outbound", follow_mode="none", session_id="receipt")
    assert c.data["tasks"]["outbound"]["stage"] == "return_nav"


def test_existing_hardware_session_can_return_with_mode_disabled(waiting):
    c = waiting
    c.config.update(
        mode="hardware",
        field_verified=True,
        verified_follow_robot="pinky3",
        follow_modes={"pinky3": ["marker"]},
    )
    c.receipt("outbound", 42, session_id="receipt")
    task = c.data["tasks"]["outbound"]
    assert c.service_projection(task)["supported_follow_modes"] == ["marker"]
    c.config["follow_modes"].clear()
    with pytest.raises(ValueError, match="허용한 추종 모드"):
        c.follow_control("outbound", "approve")
    assert c.next_action() is None
    c.follow_control("outbound", "return")
    assert c.data["tasks"]["outbound"]["stage"] == "return_nav"


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
