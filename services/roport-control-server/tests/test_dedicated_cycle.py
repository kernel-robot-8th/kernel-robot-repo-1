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
        ("observed_at", stamp(-3)),
        ("source", "omx1"),
    ],
)
def test_wrong_or_stale_results_do_not_release_pinky(field, value):
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
    with pytest.raises(ValueError, match="stale_or_future"):
        c.completed(result, now=NOW)


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
