from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from uuid import uuid4

import pytest
from registry_helpers import mutation, registration

from roport_control_server.adapters.db.migrate import migrate
from roport_control_server.adapters.db.registry import RegistryStore
from roport_control_server.contracts.robots import (
    DesiredConfig,
    RegistryError,
    RegistryRequest,
    ReleaseEvidence,
)


def test_three_slots_and_omx_mapping(registry_db):
    store, _ = registry_db
    snapshot = store.snapshot()
    assert snapshot["revision"] == 0
    assert [r["robot_id"] for r in snapshot["robots"]] == [
        "pinky1",
        "pinky2",
        "pinky3",
        "omx1",
        "omx2",
        "omx3",
    ]
    assert [(r["host_id"], r["arm_id"]) for r in snapshot["robots"][3:]] == [
        ("shelf-01", "omx1"),
        ("shelf-01", "omx2"),
        ("omx3", "omx3"),
    ]
    for row in snapshot["robots"]:
        assert row["observation"]["battery_percent"] is None
        assert row["observation"]["loaded"] is None
        assert row["display"]["ready"] is False
    for index in range(1, 4):
        store.change(
            f"pinky{index}", "registration", registration(index), development=True
        )
    assert sum(r["registered"] for r in store.snapshot()["robots"]) == 3
    with pytest.raises(RegistryError, match="unknown_pinky_slot"):
        store.change("pinky4", "registration", registration(4), development=True)


def test_reconnect_history_config_and_rebinding(registry_db):
    store, dsn = registry_db
    data = registration()
    first = store.change("pinky1", "registration", data, development=True)
    assert RegistryStore(dsn).snapshot() == store.snapshot()
    config = {**data["config"], "ros_domain_id": 20}
    changed = store.change(
        "pinky1", "config", mutation(first, config=config), development=True
    )
    row = store.snapshot()["robots"][0]
    assert row["desired_config"]["ros_domain_id"] == 20
    assert row["desired_config_revision"] == 2
    assert row["file_config"] is None and row["observation"]["runtime_config"] is None
    assert row["display"]["config_match"] == "unknown"
    with pytest.raises(RegistryError, match="exclude_first"):
        store.change("pinky1", "unregister", mutation(changed), development=True)
    excluded = store.change(
        "pinky1", "exclude", mutation(changed, excluded=True), development=True
    )
    ended = store.change("pinky1", "unregister", mutation(excluded), development=True)
    new = store.change(
        "pinky1",
        "registration",
        registration(expected_revision=ended["revision"]),
        development=True,
    )
    assert new["binding_id"] != first["binding_id"]
    with pytest.raises(RegistryError, match="binding_conflict"):
        store.change(
            "pinky1",
            "exclude",
            mutation(new, binding_id=first["binding_id"], excluded=True),
            development=True,
        )
    assert store.snapshot()["robots"][0]["observation"]["pose"] is None
    with store.connection() as conn:
        assert (
            conn.execute("SELECT count(*) AS n FROM control_robot_bindings").fetchone()[
                "n"
            ]
            == 2
        )
    assert len(store.history("pinky1")) == 5


def test_retries_are_exact_and_atomic(registry_db):
    store, _ = registry_db
    data = registration()
    with ThreadPoolExecutor(max_workers=4) as pool:
        responses = list(
            pool.map(
                lambda _: store.change(
                    "pinky1", "registration", data, development=True
                ),
                range(8),
            )
        )
    assert all(r == responses[0] for r in responses)
    assert store.snapshot()["revision"] == 1 and len(store.history("pinky1")) == 1
    with pytest.raises(RegistryError, match="request_id_reused"):
        store.change(
            "pinky1",
            "registration",
            {**data, "device_id": "sim-other"},
            development=True,
        )


@pytest.mark.parametrize("collision", ["device_id", "ros_domain_id", "ip"])
def test_concurrent_cross_slot_duplicates(registry_db, collision):
    store, _ = registry_db
    one, two = registration(1), registration(2)
    if collision == "device_id":
        two[collision] = one[collision]
    else:
        two["config"][collision] = one["config"][collision]

    def run(pair):
        try:
            return store.change(*pair, development=True)
        except RegistryError as error:
            return error.code

    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(
            pool.map(
                run, [("pinky1", "registration", one), ("pinky2", "registration", two)]
            )
        )
    assert sum(isinstance(r, dict) for r in results) == 1
    assert "duplicate_device_domain_or_address" in results
    assert store.snapshot()["revision"] == 1


def test_two_editors_cannot_overwrite(registry_db):
    store, _ = registry_db
    result = store.change("pinky1", "registration", registration(), development=True)
    store.change("pinky1", "exclude", mutation(result, excluded=True), development=True)
    with pytest.raises(RegistryError, match="revision_conflict"):
        store.change(
            "pinky1",
            "config",
            mutation(result, config=registration()["config"]),
            development=True,
        )


def test_failure_rolls_back_binding_revision_and_receipt(registry_db):
    store, _ = registry_db
    with store.connection() as conn:
        conn.execute(
            "ALTER TABLE control_registry_changes ADD CONSTRAINT reject_write CHECK (false)"
        )
    with pytest.raises(RegistryError, match="registry_unavailable"):
        store.change("pinky1", "registration", registration(), development=True)
    assert store.snapshot()["revision"] == 0
    assert not store.snapshot()["robots"][0]["registered"]
    assert not store.history("pinky1")


def test_migration_is_explicit_idempotent_and_mode_locked(registry_db):
    store, dsn = registry_db
    store.change("pinky1", "registration", registration(), development=True)
    before = store.snapshot()
    migrate(dsn, development=True)
    assert store.snapshot() == before
    with pytest.raises(ValueError, match="mode differs"):
        migrate(dsn)


def test_production_and_real_devices_are_blocked(registry_db):
    store, _ = registry_db
    with pytest.raises(RegistryError, match="operating_writes_disabled"):
        store.change("pinky1", "registration", registration())
    with pytest.raises(RegistryError, match="synthetic_device_required"):
        store.change(
            "pinky1",
            "registration",
            registration(device_id="real-device"),
            development=True,
        )
    with store.connection() as conn:
        conn.execute("UPDATE control_registry_meta SET development=false")
    with pytest.raises(RegistryError, match="development_database_required"):
        store.change("pinky1", "registration", registration(), development=True)


@pytest.mark.parametrize(
    "field,bad",
    [
        ("ip", "x;shutdown"),
        ("ip", 123),
        ("role", "anything"),
        ("ros_domain_id", True),
        ("ros_domain_id", 102),
        ("ros_domain_id", -1),
        ("calibration", "pinky_ab\n"),
    ],
)
def test_config_boundary(field, bad):
    with pytest.raises(RegistryError):
        DesiredConfig.parse({**registration()["config"], field: bad})


@pytest.mark.parametrize(
    "change",
    [
        {"request_id": "bad"},
        {"expected_revision": True},
        {"expected_revision": -1},
        {"device_id": "a\n"},
        {"verified": True},
        {"credential": "REPLACE_ME"},
    ],
)
def test_request_boundary(change):
    with pytest.raises(RegistryError):
        RegistryRequest.parse("registration", {**registration(), **change})


def test_unknown_or_unsafe_release_is_never_clear():
    assert len(ReleaseEvidence().reasons(True)) == 7
    clear = ReleaseEvidence(True, True, False, False, False, False, False)
    assert clear.reasons(True) == []
    assert clear.reasons(False) == ["exclude_first"]
    for field in (
        "online",
        "stopped",
        "active_task",
        "pending_command",
        "occupied_resource",
        "loaded",
        "maintenance",
    ):
        assert replace(clear, **{field: None}).reasons(True)
    assert replace(clear, loaded=True).reasons(True) == ["loaded"]


def test_history_limit_and_invalid_mutation(registry_db):
    store, _ = registry_db
    with pytest.raises(RegistryError):
        store.history("pinky1", limit=100000)
    with pytest.raises(RegistryError):
        RegistryRequest.parse(
            "unregister",
            {"request_id": str(uuid4()), "expected_revision": 1, "binding_id": "bad"},
        )
