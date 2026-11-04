from copy import deepcopy
from datetime import UTC, datetime

import pytest
from test_dedicated import NOW, command

from roport_control_server.domain.dedicated_cycle import DedicatedCycle


def stamp(offset=0):
    return datetime.fromtimestamp(NOW + offset, UTC).isoformat()


def begin(cycle, cmd, *, empty=False):
    cycle.begin(
        cmd,
        p7_cart_id=cmd["payload"]["pinky_id"],
        cart_empty=empty,
        host_boot_id="host-boot",
        pinky_boot_id="pinky-boot",
        docking_execution_id="dock",
        evidence_seq=0,
        observed_at=stamp(),
        now=NOW,
    )


def complete(cmd):
    p = cmd["payload"]
    return {
        **{
            k: cmd[k]
            for k in ("site_id", "task_id", "step_id", "command_id", "execution_id")
        },
        "type": "shelf.execute.completed",
        "source": "shelf-01",
        "target": cmd["source"],
        "boot_id": "host-boot",
        "observed_at": stamp(),
        "payload": {
            **deepcopy(p),
            "state": "SAFE_RETURNED",
            "active_arm_safe": True,
            "inactive_arm_safe": True,
            "arms": {
                a: {"captured_at": stamp(), "safe": True} for a in ("omx1", "omx2")
            },
            "location": "slot" if p["operation"] == "INBOUND" else "pinky",
        },
    }


def evidence(cmd, *, exit=False):
    return {
        "pinky_id": cmd["payload"]["pinky_id"],
        "docking_execution_id": "dock",
        "pinky_boot_id": "pinky-boot",
        "evidence_seq": 2 if exit else 1,
        "source_observed_at": stamp(),
        "stopped": True,
        "docked": not exit,
        "p7_clear": True,
    }


def cycle():
    return DedicatedCycle(max_evidence_age=2, clock_skew=0.1)


def test_full_cycle_requires_return_departure_then_p7_exit():
    c = cycle()
    inbound, outbound = command(), command("OUTBOUND")
    begin(c, inbound)
    with pytest.raises(ValueError, match="departure_not_ready"):
        c.approve_departure(evidence(inbound), now=NOW)
    with pytest.raises(ValueError, match="outbound_requires"):
        begin(c, outbound, empty=True)
    c.completed(complete(inbound), now=NOW)
    with pytest.raises(ValueError, match="departure_not_approved"):
        c.p7_cleared(evidence(inbound), now=NOW)
    c.approve_departure(evidence(inbound), now=NOW)
    with pytest.raises(ValueError, match="outbound_requires"):
        begin(c, outbound, empty=True)
    c.p7_cleared(evidence(inbound, exit=True), now=NOW)
    with pytest.raises(ValueError, match="empty_pinky3"):
        begin(c, outbound, empty=False)
    begin(c, outbound, empty=True)
    c.completed(complete(outbound), now=NOW)
    assert c.approve_departure(evidence(outbound), now=NOW)
    c.p7_cleared(evidence(outbound, exit=True), now=NOW)
    assert c.state == "DELIVERY_READY"  # Still not customer receipt.


@pytest.mark.parametrize(
    "field,value",
    [
        ("execution_id", "other"),
        ("boot_id", "restarted"),
        ("source", "omx1"),
    ],
)
def test_wrong_results_do_not_release_pinky(field, value):
    c, cmd = cycle(), command()
    begin(c, cmd)
    result = complete(cmd)
    result[field] = value
    with pytest.raises(ValueError):
        c.completed(result, now=NOW)
    assert c.state == "INBOUND_ACTIVE"


@pytest.mark.parametrize(
    "field,value",
    [
        ("inactive_arm_safe", False),
        ("active_arm_safe", 1),
        ("package_id", "other"),
        ("manipulation_mode", "COOPERATIVE"),
        ("location", "UNKNOWN"),
    ],
)
def test_incomplete_or_mismatched_assignment_does_not_release_pinky(field, value):
    c, cmd = cycle(), command()
    begin(c, cmd)
    result = complete(cmd)
    result["payload"][field] = value
    with pytest.raises(ValueError):
        c.completed(result, now=NOW)


def test_stale_arm_observation_is_not_freshened_by_result_timestamp():
    c, cmd = cycle(), command()
    begin(c, cmd)
    result = complete(cmd)
    result["payload"]["arms"]["omx2"]["captured_at"] = stamp(-3)
    c.completed(result, now=NOW)  # The completion fact is recorded...
    with pytest.raises(ValueError, match="stale_or_future"):  # ...departure is not.
        c.approve_departure(evidence(cmd), now=NOW)


def test_pinky_exit_must_match_current_docking_and_be_fresh():
    c, cmd = cycle(), command()
    begin(c, cmd)
    c.completed(complete(cmd), now=NOW)
    c.approve_departure(evidence(cmd), now=NOW)
    with pytest.raises(ValueError, match="pinky_boot_or_docking"):
        c.p7_cleared({**evidence(cmd), "docking_execution_id": "other"}, now=NOW)
    with pytest.raises(ValueError, match="stale_or_future"):
        c.p7_cleared(evidence(cmd), now=NOW + 3)


@pytest.mark.parametrize(
    "field,value",
    [
        ("resource_token", "other"),
        ("permit_id", "other"),
        ("workcell_id", "other"),
        ("robot_ids", ["omx2"]),
        ("expected_package_ids", ["other"]),
    ],
)
def test_result_keeps_full_resource_and_assignment_binding(field, value):
    c, cmd = cycle(), command()
    begin(c, cmd)
    result = complete(cmd)
    result["payload"][field] = value
    with pytest.raises(ValueError, match="result_assignment_mismatch"):
        c.completed(result, now=NOW)
    assert c.state == "INBOUND_ACTIVE"


def test_p7_exit_rejects_duplicate_or_previous_boot_evidence():
    c, cmd = cycle(), command()
    begin(c, cmd)
    c.completed(complete(cmd), now=NOW)
    with pytest.raises(ValueError, match="new_pinky_evidence"):
        c.approve_departure({**evidence(cmd), "evidence_seq": 0}, now=NOW)
    c.approve_departure(evidence(cmd), now=NOW)
    with pytest.raises(ValueError, match="new_pinky_evidence"):
        c.p7_cleared({**evidence(cmd, exit=True), "evidence_seq": 1}, now=NOW)
    with pytest.raises(ValueError, match="pinky_boot_or_docking"):
        c.p7_cleared(
            {**evidence(cmd, exit=True), "pinky_boot_id": "restarted"}, now=NOW
        )
    assert c.state == "INBOUND_WAIT_P7_CLEAR"


def test_departure_rechecks_each_arm_age_not_only_latest_result_time():
    c, cmd = cycle(), command()
    begin(c, cmd)
    result = complete(cmd)
    result["payload"]["arms"]["omx2"]["captured_at"] = stamp(-1.9)
    c.completed(result, now=NOW)
    with pytest.raises(ValueError, match="stale_or_future"):
        c.approve_departure(
            {**evidence(cmd), "source_observed_at": stamp(0.2)}, now=NOW + 0.2
        )


def at(event, offset):
    """Same Host snapshot observed later (observed_at and both arms)."""
    event = deepcopy(event)
    event["observed_at"] = stamp(offset)
    for arm in event["payload"]["arms"].values():
        arm["captured_at"] = stamp(offset)
    return event


def test_stale_arm_safety_is_refreshed_without_rewriting_completion():
    c, cmd = cycle(), command()
    begin(c, cmd)
    done = complete(cmd)
    c.completed(done, now=NOW)
    late = {**evidence(cmd), "source_observed_at": stamp(3)}
    with pytest.raises(ValueError, match="stale_or_future"):
        c.approve_departure(late, now=NOW + 3)
    c.refresh_arm_safety(at(done, 3), now=NOW + 3)
    assert c.result == done  # Completion fact is kept, only safety evidence moves.
    assert c.approve_departure(late, now=NOW + 3)


def test_late_completion_is_recorded_but_blocks_departure_until_refresh():
    c, cmd = cycle(), command()
    begin(c, cmd)
    done = complete(cmd)
    c.completed(done, now=NOW + 3)  # older than max_evidence_age: still a real fact
    assert c.result == done
    assert c.state.endswith("_READY_TO_DEPART")
    late = {**evidence(cmd), "source_observed_at": stamp(3)}
    with pytest.raises(ValueError, match="stale_or_future"):
        c.approve_departure(late, now=NOW + 3)
    c.refresh_arm_safety(at(done, 3), now=NOW + 3)
    assert c.approve_departure(late, now=NOW + 3)


def test_late_completion_still_checks_binding_and_safety():
    c, cmd = cycle(), command()
    begin(c, cmd)
    other = {**complete(cmd), "execution_id": "other"}
    with pytest.raises(ValueError, match="result_binding_mismatch"):
        c.completed(other, now=NOW + 3)
    unsafe = complete(cmd)
    unsafe["payload"]["inactive_arm_safe"] = False
    with pytest.raises(ValueError, match="both_arm_safety_required"):
        c.completed(unsafe, now=NOW + 3)
    assert c.result is None


@pytest.mark.parametrize(
    "field,value,reason",
    [
        ("execution_id", "other", "result_binding_mismatch"),
        ("boot_id", "restarted", "host_restarted_reconcile"),
        ("type", "shelf.execute.held", "physical_result_not_complete"),
    ],
)
def test_arm_refresh_is_bound_to_execution_and_host_boot(field, value, reason):
    c, cmd = cycle(), command()
    begin(c, cmd)
    c.completed(complete(cmd), now=NOW)
    with pytest.raises(ValueError, match=reason):
        c.refresh_arm_safety({**at(complete(cmd), 1), field: value}, now=NOW + 1)


def test_arm_refresh_rejects_unsafe_regressed_or_early_use():
    c, cmd = cycle(), command()
    begin(c, cmd)
    with pytest.raises(ValueError, match="departure_not_ready"):
        c.refresh_arm_safety(complete(cmd), now=NOW)
    c.completed(at(complete(cmd), 1), now=NOW + 1)
    unsafe = at(complete(cmd), 1.5)
    unsafe["payload"]["arms"]["omx2"]["safe"] = False
    with pytest.raises(ValueError, match="both_arm_safety"):
        c.refresh_arm_safety(unsafe, now=NOW + 1.5)
    older = at(complete(cmd), 1.5)
    older["payload"]["arms"]["omx1"]["captured_at"] = stamp(0.5)
    with pytest.raises(ValueError, match="arm_observation_regressed"):
        c.refresh_arm_safety(older, now=NOW + 1.5)


def test_expired_command_cannot_begin_but_late_result_is_kept():
    expired = command()
    expired["expires_at"] = stamp(-1)
    with pytest.raises(ValueError, match="expired_command"):
        begin(cycle(), expired)
    c, cmd = cycle(), command()
    begin(c, cmd)
    c.completed(at(complete(cmd), 31), now=NOW + 31)  # expires_at is NOW + 30.
    assert c.state == "INBOUND_READY_TO_DEPART"


@pytest.mark.parametrize(
    "change",
    [
        lambda p: p.update(package_id=None, expected_package_ids=[None]),
        lambda p: p.update(inactive_arm_id="omx1"),
        lambda p: p.update(assigned_arm_id="omx3", robot_ids=["omx3"]),
        lambda p: p.update(robot_ids=["omx2"]),
        lambda p: p.update(slot_id=""),
        lambda p: p.pop("permit_id"),
        lambda p: p.update(manipulation_mode="COOPERATIVE"),
        lambda p: p.update(pinky_id="pinky3"),
    ],
)
def test_begin_revalidates_assignment_payload(change):
    c, cmd = cycle(), command()
    change(cmd["payload"])
    with pytest.raises(ValueError):
        begin(c, cmd)
    assert c.state == "EMPTY"


@pytest.mark.parametrize("payload", [None, [], "x"])
def test_begin_rejects_non_object_payload_with_value_error(payload):
    with pytest.raises(ValueError, match="invalid_command"):
        cycle().begin(
            {**command(), "payload": payload},
            p7_cart_id="pinky1",
            cart_empty=False,
            host_boot_id="host-boot",
            pinky_boot_id="pinky-boot",
            docking_execution_id="dock",
            evidence_seq=0,
            observed_at=stamp(),
            now=NOW,
        )
