"""Owner registration persistence and operator boundary on an isolated DB schema."""

import os
import socket
import time
from copy import deepcopy

import pytest
from registry_helpers import mutation, registration
from test_fleet import fleet, profile  # noqa: F401

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    migrate_service,
)
from roport_control_server.contracts.robots import RegistryError
from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.fleet_registry import FleetRegistry
from roport_control_server.executor.fleet_socket import FleetSocket


@pytest.fixture
def registered_owner(registry_db, fleet):  # noqa: F811
    registry, dsn = registry_db
    migrate_service(dsn)
    store = OperationalStore(dsn)
    try:
        control = FleetControl(deepcopy(fleet.config), store, clock=fleet.clock)
        control.observed = deepcopy(fleet.observed)
        control.heard = deepcopy(fleet.heard)
        FleetRegistry(control)
        yield control, registry, dsn
    finally:
        store.close()


def test_roles_registration_config_exclusion_are_shared(registered_owner):
    control, registry, _ = registered_owner
    assert control.data["roles"] == dict.fromkeys(("pinky1", "pinky2", "pinky3"))
    first = control.registry.change(
        "pinky1", "registration", registration(), actor="uid:123"
    )
    assert control.data["roles"]["pinky1"] == "logistics"
    request = mutation(first, config={**registration()["config"], "role": "smartcart"})
    changed = control.registry.change("pinky1", "config", request, actor="uid:123")
    assert control.data["roles"]["pinky1"] == "smartcart"
    assert registry.snapshot()["robots"][0]["desired_config"]["role"] == "smartcart"
    assert (
        control.registry.change("pinky1", "config", request, actor="uid:456") == changed
    )
    assert registry.snapshot()["revision"] == 2
    control.assign("pinky1", None, actor="uid:123")
    assert control.data["roles"]["pinky1"] is None
    with pytest.raises(RegistryError, match="reinclude_robot"):
        control.assign("pinky1", "logistics", actor="uid:123")
    row = registry.snapshot()["robots"][0]
    control.registry.change(
        "pinky1", "exclude", mutation(row, excluded=False), actor="uid:123"
    )
    assert control.data["roles"]["pinky1"] == "smartcart"
    with registry.connection() as conn:
        assert {
            r["actor"]
            for r in conn.execute(
                "SELECT actor FROM control_registry_changes"
            ).fetchall()
        } == {"uid:123"}


def test_legacy_development_writer_stays_sim_only(registered_owner):
    control, registry, _ = registered_owner
    with pytest.raises(RegistryError, match="operating_writes_disabled"):
        registry.change("pinky1", "registration", registration())
    with pytest.raises(RegistryError, match="synthetic_device_required"):
        control.registry.change(
            "pinky1",
            "registration",
            registration(device_id="pinky-d660"),
            actor="uid:123",
        )


def test_operating_registration_and_confirmed_release_use_owner_evidence(
    registered_owner,
):
    control, registry, _ = registered_owner
    with registry.connection() as conn:
        conn.execute("UPDATE control_registry_meta SET development=false")
    control.config.update(mode="hardware", expected_starts={"pinky1": "N16"})
    first = control.registry.change(
        "pinky1", "registration", registration(device_id="pinky-d660"), actor="uid:123"
    )
    excluded = control.registry.change(
        "pinky1", "exclude", mutation(first, excluded=True), actor="uid:123"
    )
    request = mutation(excluded)
    confirmed = {"empty_load": True, "removed_from_lane": True}
    result = control.registry.change(
        "pinky1", "unregister", request, actor="uid:123", confirmed=confirmed
    )
    assert not registry.snapshot()["robots"][0]["registered"]
    assert (
        control.registry.change(
            "pinky1", "unregister", request, actor="uid:123", confirmed=confirmed
        )
        == result
    )
    with registry.connection() as conn:
        row = conn.execute(
            "SELECT payload, actor FROM control_registry_changes WHERE action='unregister'"
        ).fetchone()
    assert row["payload"]["confirmed"] == confirmed
    assert row["actor"] == "uid:123"


def test_registry_socket_requires_peer_and_records_actual_uid(
    registered_owner, tmp_path
):
    control, registry, _ = registered_owner
    channel = FleetSocket(tmp_path / "registry.sock", control)
    try:
        request = {
            "action": "registry",
            "robot": "pinky1",
            "operation": "registration",
            "request": registration(),
            "session_id": channel.session_id,
            "expires_at": time.time() + 1,
        }
        with pytest.raises(ValueError, match="peer credentials"):
            channel.dispatch(request)
        peer, remote = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        with peer, remote:
            # A real Unix peer, not a JSON actor supplied by the browser.
            response = channel.dispatch(request, client=peer)
        assert response["registry_result"]["robot_id"] == "pinky1"
        assert response["roles"]["pinky1"] == "logistics"
        assert response["registry"]["robots"][0]["fleet_role"] == "logistics"
        assert registry.history("pinky1")[0]["actor"] == f"uid:{os.getuid()}"
    finally:
        channel.close()


def test_registry_restart_preserves_existing_work_hold(registered_owner):
    control, _, dsn = registered_owner
    control.registry.change("pinky1", "registration", registration(), actor="uid:123")
    state = deepcopy(control.data)
    state["tasks"]["pending"] = {
        "robot": "pinky1",
        "status": "HOLD",
        "stage": "loading_route",
    }
    state["fault"] = "operator_stop"
    control.save(state)
    control.store.close()
    store = OperationalStore(dsn)
    try:
        restarted = FleetControl(control.config, store)
        FleetRegistry(restarted)
        assert restarted.data["tasks"] == state["tasks"]
        assert restarted.data["fault"] == "operator_stop"
        assert not restarted.data["enabled"]
        assert restarted.data["roles"]["pinky1"] == "logistics"
    finally:
        store.close()


def test_first_adoption_rejects_static_unfinished_work(registry_db, fleet):  # noqa: F811
    _, dsn = registry_db
    migrate_service(dsn)
    store = OperationalStore(dsn)
    try:
        control = FleetControl(fleet.config, store)
        state = deepcopy(control.data)
        state["tasks"]["pending"] = {
            "robot": "pinky1",
            "status": "RUNNING",
            "stage": "loading_route",
        }
        control.save(state)
        with pytest.raises(RegistryError, match="finish_work"):
            FleetRegistry(control)
        assert control.data == state and control.registry is None
    finally:
        store.close()
