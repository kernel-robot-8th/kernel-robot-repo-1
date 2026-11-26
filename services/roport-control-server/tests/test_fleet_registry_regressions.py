"""Independent owner/registry regressions in each fixture's isolated DB schema."""

from copy import deepcopy
from datetime import UTC, datetime
from types import SimpleNamespace

import psycopg
import pytest
from registry_helpers import mutation, registration
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import MemoryTransport, profile  # noqa: F401

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    migrate_service,
)
from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import Delivery
from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.fleet_registry import FleetRegistry
from roport_control_server.executor.fleet_runtime import FleetRuntime

ACTOR = "uid:1000"
CLEAR = {"empty_load": True, "removed_from_lane": True}


@pytest.fixture
def owner(registry_db, fleet):
    registry, dsn = registry_db
    migrate_service(dsn)
    store = OperationalStore(dsn)
    control = FleetControl(deepcopy(fleet.config), store, clock=lambda: 1)
    FleetRegistry(control)
    try:
        yield SimpleNamespace(
            control=control,
            store=store,
            registry=registry,
            dsn=dsn,
            reports=deepcopy(fleet.observed),
        )
    finally:
        store.close()


def observe(owner, robot="pinky1", *, boot=None):
    control = owner.control
    payload = deepcopy(owner.reports[robot])
    payload.update(
        boot_version=boot or payload["boot_version"],
        sequence=control.observed.get(robot, {}).get("sequence", 0) + 1,
        observed_at=datetime.now(UTC).isoformat(),
    )
    control.observe({"source": robot, "payload": payload})
    return payload


def register(owner, index=1, **changes):
    observe(owner, f"pinky{index}")
    return owner.control.registry.change(
        f"pinky{index}", "registration", registration(index, **changes), actor=ACTOR
    )


def excluded(owner):
    first = register(owner)
    return owner.control.registry.change(
        "pinky1", "exclude", mutation(first, excluded=True), actor=ACTOR
    )


@pytest.mark.parametrize("operation", ["registration", "unregister"])
def test_registry_and_fleet_roll_back_together_on_snapshot_write_failure(
    owner, operation
):
    control = owner.control
    if operation == "unregister":
        request = mutation(excluded(owner))
    else:
        observe(owner)
        request = registration()
    before = deepcopy(control.data)
    observations = deepcopy(control.observed)
    metadata = owner.registry.snapshot()
    history = owner.registry.history("pinky1")
    # This trigger exists only in registry_db's UUID schema and fails after registry SQL.
    with psycopg.connect(owner.dsn) as conn:
        conn.execute("""
            CREATE FUNCTION reject_test_snapshot() RETURNS trigger LANGUAGE plpgsql AS $$
            BEGIN RAISE EXCEPTION 'injected snapshot failure'; END $$;
            CREATE TRIGGER reject_test_snapshot BEFORE UPDATE ON control_operational_state
            FOR EACH ROW EXECUTE FUNCTION reject_test_snapshot();
        """)
    with pytest.raises(ValueError, match="registry_unavailable"):
        control.registry.change(
            "pinky1",
            operation,
            request,
            actor=ACTOR,
            **({"confirmed": CLEAR} if operation == "unregister" else {}),
        )
    assert control.data == before
    assert control.observed == observations
    assert owner.registry.snapshot() == metadata
    assert owner.registry.history("pinky1") == history
    with psycopg.connect(owner.dsn) as conn:
        assert (
            conn.execute("SELECT data FROM control_operational_state").fetchone()[0]
            == before
        )


@pytest.mark.parametrize(
    "blocker",
    [
        "stale",
        "moving",
        "line_active",
        "device_command",
        "participant",
        "task",
        "robot_resource",
        "task_resource",
        "pending_arrival",
    ],
)
def test_unregistration_cannot_erase_active_or_uncertain_robot(owner, blocker):
    control = owner.control
    previous = excluded(owner)
    payload = control.observed["pinky1"]
    if blocker == "stale":
        control.heard["pinky1"] = -10
    elif blocker == "moving":
        payload["state"]["stopped"] = False
    elif blocker == "line_active":
        payload["state"]["line_enabled"] = True
    elif blocker == "device_command":
        payload["active_command_id"] = "motion-running"
    else:
        data = deepcopy(control.data)
        if blocker == "participant":
            data["commands"]["shared-load"] = {"participants": ["pinky1", "omx3"]}
        elif blocker == "task":
            data["tasks"]["returning"] = {"robot": "pinky1", "status": "RUNNING"}
        elif blocker == "robot_resource":
            data["resources"]["robot:pinky1"] = "unresolved-owner"
        elif blocker == "task_resource":
            data["tasks"]["past-task"] = {"robot": "pinky1", "status": "COMPLETE"}
            data["resources"]["node:N16"] = "past-task"
        else:
            data["pending_arrivals"]["pinky1"] = {"command_id": "arrival-pending"}
        control.save(data)
    before = deepcopy(control.data)
    with pytest.raises(ValueError):
        control.registry.change(
            "pinky1", "unregister", mutation(previous), actor=ACTOR, confirmed=CLEAR
        )
    assert control.data == before
    row = owner.registry.snapshot()["robots"][0]
    assert row["registered"] and row["revision"] == previous["revision"]


@pytest.mark.parametrize("missing", ["empty_load", "removed_from_lane"])
def test_operator_confirmation_does_not_replace_live_stopped_evidence(owner, missing):
    control = owner.control
    previous = excluded(owner)
    before = deepcopy(control.data)
    with pytest.raises(ValueError, match="confirm_empty_load_and_removed_from_lane"):
        control.registry.change(
            "pinky1",
            "unregister",
            mutation(previous),
            actor=ACTOR,
            confirmed={**CLEAR, missing: False},
        )
    assert control.data == before


def test_replacement_drops_old_boot_pose_arrival_and_preserves_exact_replay(owner):
    control = owner.control
    previous = excluded(owner)
    old = deepcopy(control.observed["pinky1"])
    old["state"]["arrival_command_id"] = "old-arrival"
    control.observed["pinky1"] = old
    control.poses["pinky1"] = {"boot_version": old["boot_version"], "pose": {"x": 1}}
    request = mutation(previous)
    result = control.registry.change(
        "pinky1", "unregister", request, actor=ACTOR, confirmed=CLEAR
    )
    assert "pinky1" not in control.observed
    assert "pinky1" not in control.poses
    assert "pinky1" not in control.data["last_nodes"]
    assert control.data["roles"]["pinky1"] is None
    assert (
        control.registry.change(
            "pinky1", "unregister", request, actor=ACTOR, confirmed=CLEAR
        )
        == result
    )
    assert len(owner.registry.history("pinky1")) == 3
    observe(owner)
    assert "pinky1" not in control.observed
    replacement = registration(
        device_id="sim-replacement", expected_revision=result["revision"]
    )
    with pytest.raises(ValueError):
        control.registry.change("pinky1", "registration", replacement, actor=ACTOR)
    observe(owner, boot="replacement-boot")
    rebound = control.registry.change(
        "pinky1", "registration", replacement, actor=ACTOR
    )
    assert rebound["binding_id"] != previous["binding_id"]
    assert control.observed["pinky1"]["state"]["arrival_command_id"] is None
    old["sequence"] += 100
    old["observed_at"] = datetime.now(UTC).isoformat()
    control.observe({"source": "pinky1", "payload": old})
    assert control.observed["pinky1"]["boot_version"] == "replacement-boot"
    pose = wire.build_message(
        "robot_pose",
        "pinky1",
        control.config["control_id"],
        {
            "boot_version": old["boot_version"],
            "sequence": 999,
            "observed_at": datetime.now(UTC).isoformat(),
            "map_version": control.config["map_version"],
            "frame_id": "map",
            "pose_source": "amcl",
            "pose": {"x": 1, "y": 2, "yaw": 0},
        },
    )
    FleetRuntime(control, MemoryTransport()).handle(
        Delivery(wire.topic("pinky1", "motion"), wire.encode(pose), False, 1, 0, 0)
    )
    assert "pinky1" not in control.poses


def test_fixed_slots_keep_registration_metadata_distinct_from_identity_proof(owner):
    control = owner.control
    for index in range(1, 4):
        register(owner, index)
    with pytest.raises(ValueError, match="unknown_pinky_slot"):
        control.registry.change("pinky4", "registration", registration(4), actor=ACTOR)
    rows = [r for r in control.registry.snapshot()["robots"] if r["kind"] == "pinky"]
    assert len(rows) == 3 and all(r["registered"] for r in rows)
    for row in rows:
        assert row["display"]["config_match"] == "unknown"
        assert "config_unverified" in row["display"]["reasons"]
        assert row["file_config"] is None
        assert row["observation"]["runtime_config"] is None
        assert row["observation"]["loaded"] is None


def test_legacy_static_roles_do_not_require_registry_opt_in(fleet):
    assert fleet.registry is None
    assert "registry_id" not in fleet.data
    fleet.assign("pinky1", None)
    fleet.assign("pinky1", "smartcart")
    assert fleet.data["roles"]["pinky1"] == "smartcart"


def test_role_change_is_explicit_after_exclusion_and_updates_registry_with_fleet(owner):
    control = owner.control
    previous = excluded(owner)
    before = deepcopy(control.data)
    with pytest.raises(ValueError, match="reinclude_robot_before_assigning_role"):
        control.assign("pinky1", "smartcart", actor=ACTOR)
    assert control.data == before
    control.registry.change(
        "pinky1", "exclude", mutation(previous, excluded=False), actor=ACTOR
    )
    control.assign("pinky1", "smartcart", actor=ACTOR)
    row = control.registry.snapshot()["robots"][0]
    assert row["desired_config"]["role"] == "smartcart"
    assert row["fleet_role"] == control.data["roles"]["pinky1"] == "smartcart"
    assert not row["excluded"]
