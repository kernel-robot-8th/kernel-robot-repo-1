"""The service inbox and the same fleet executor share stable request identities."""

from copy import deepcopy
from datetime import UTC, datetime

import pytest
from test_fleet import finish
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401
from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import example, request

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    ServiceTaskStore,
)
from roport_control_server.executor.fleet_runtime import service_tick as tick


def service_tick(control, store):
    return tick(control, store, now=datetime(2030, 1, 1, 2, tzinfo=UTC))


@pytest.fixture
def operational_fleet(fleet, dsn):
    store = OperationalStore(dsn)
    store.save(fleet.data)
    fleet.store = store
    try:
        yield fleet
    finally:
        store.close()


def queued(store, *, packages=("a", "b"), operation="prepare_order", goal="gate"):
    message = request()
    message["payload"].update(
        operation=operation,
        pickup_point_id=goal,
        packages=[{"package_id": package, "quantity": 1} for package in packages],
    )
    return store.accept(message)


def test_multi_package_replay_keeps_one_child_per_physical_package(fleet, dsn):
    store = ServiceTaskStore(dsn)
    task = queued(store)
    service_tick(fleet, store)
    service_tick(fleet, store)
    assert set(fleet.data["tasks"]) == {task["task_id"] + ":a", task["task_id"] + ":b"}
    assert all(t["stage"] == "queued" for t in fleet.data["tasks"].values())
    # Acceptance and queueing must not claim that a robot has started moving.
    assert store.pending_tasks()[0]["state"] == "QUEUED"
    finish(fleet)
    service_tick(fleet, store)
    assert not store.pending_tasks()
    assert store.outbox()[-1]["type"] == "task.completed"


def test_empty_manifest_does_not_create_motion_work(fleet, dsn):
    store = ServiceTaskStore(dsn)
    queued(store, packages=())
    service_tick(fleet, store)
    assert not fleet.data["tasks"]
    assert store.pending_tasks()[0]["state"] == "WAITING_MANIFEST"
    assert fleet.data["enabled"]


def test_unknown_goal_does_not_crash_or_disable_owner(fleet, dsn):
    store = ServiceTaskStore(dsn)
    queued(store, operation="customer_service", goal="not-calibrated")
    service_tick(fleet, store)
    assert store.pending_tasks()[0]["state"] == "HOLD"
    assert not fleet.data["tasks"]
    assert fleet.data["enabled"] and fleet.data["fault"] is None
    service_tick(fleet, store)
    assert len(store.outbox()) == 2  # one acceptance + one hold, not repeated events


def test_missing_shelf_stock_holds_service_request_without_creating_robot_work(
    fleet, dsn
):
    store = ServiceTaskStore(dsn)
    queued(store, packages=("missing",), operation="customer_service")
    before = deepcopy(fleet.data)
    service_tick(fleet, store)
    saved = store.pending_tasks()[0]
    assert saved["state"] == "HOLD"
    assert saved["progress"] == {
        "reason": "출고할 물품이 선반에 등록되어 있지 않습니다."
    }
    assert fleet.data == before
    service_tick(fleet, store)
    assert len(store.outbox()) == 2


@pytest.mark.parametrize("packages", [("a", "missing"), ("missing", "a")])
def test_unavailable_package_holds_whole_service_request_before_creating_children(
    fleet, dsn, packages
):
    store = ServiceTaskStore(dsn)
    data = deepcopy(fleet.data)
    data["slots"]["1a"] = {"state": "occupied", "package_id": "a"}
    fleet.save(data)
    queued(store, packages=packages, operation="customer_service")
    before = deepcopy(fleet.data)
    service_tick(fleet, store)
    assert store.pending_tasks()[0]["state"] == "HOLD"
    assert store.pending_tasks()[0]["progress"] == {
        "reason": "출고할 물품이 선반에 등록되어 있지 않습니다."
    }
    assert fleet.data == before
    assert fleet.data["tasks"] == {} and fleet.data["commands"] == {}
    assert fleet.next_action() is None


def test_service_metadata_is_not_lost_or_reinterpreted_as_dispatch_time(fleet, dsn):
    store = ServiceTaskStore(dsn)
    msg = request()
    msg["payload"].update(
        operation="prepare_order", pickup_point_id="gate", order_revision=7
    )
    task = store.accept(msg)
    service_tick(fleet, store)
    saved = store.pending_tasks()[0]["request"]
    assert saved["order_revision"] == 7
    assert datetime.fromisoformat(saved["pickup_at"]) == datetime.fromisoformat(
        msg["payload"]["pickup_at"]
    )
    assert saved == task["request"]


def test_recorded_receipt_releases_only_matching_waiting_child_once(
    operational_fleet, dsn
):
    fleet = operational_fleet
    store = ServiceTaskStore(dsn)
    data = deepcopy(fleet.data)
    data["slots"]["1a"] = {"state": "occupied", "package_id": "a"}
    fleet.save(data)
    task = queued(store, packages=("a",), operation="customer_service")
    service_tick(fleet, store)
    finish(fleet)
    child_id = task["task_id"] + ":a"
    assert fleet.data["tasks"][child_id]["stage"] == "await_receipt"
    service_tick(fleet, store)
    assert fleet.data["tasks"][child_id]["stage"] == "await_receipt"
    store.accept_receipt(
        {
            **example("receipt"),
            "site_id": task["site_id"],
            "task_id": task["task_id"],
            "payload": {
                "receipt_id": "receipt-test",
                "package_ids": ["a"],
                "reservation_id": task["reservation_id"],
                "order_id": task["order_id"],
            },
        }
    )
    service_tick(fleet, store)
    assert fleet.data["tasks"][child_id]["stage"] == "return_nav"
    before = deepcopy(fleet.data)
    service_tick(fleet, store)
    assert fleet.data == before
    finish(fleet)
    service_tick(fleet, store)
    assert not store.pending_tasks()


@pytest.mark.parametrize("quantity", [0, 2, True])
def test_package_id_already_denotes_one_physical_package(dsn, quantity):
    message = request()
    message["payload"]["packages"][0]["quantity"] = quantity
    with pytest.raises(ValueError, match="manifest"):
        ServiceTaskStore(dsn).accept(message)


@pytest.mark.parametrize("pickup_at", [None, "tomorrow", "2030-01-01T10:00:00"])
def test_service_timestamp_requires_explicit_instant(dsn, pickup_at):
    message = request()
    message["payload"]["pickup_at"] = pickup_at
    with pytest.raises(ValueError, match="pickup_at"):
        ServiceTaskStore(dsn).accept(message)


def test_partial_receipt_only_releases_its_package(operational_fleet, dsn):
    fleet = operational_fleet
    store = ServiceTaskStore(dsn)
    data = deepcopy(fleet.data)
    for slot, package in [("1a", "a"), ("2a", "b")]:
        data["slots"][slot] = {"state": "occupied", "package_id": package}
    fleet.save(data)
    task = queued(store, operation="customer_service")
    service_tick(fleet, store)
    data = deepcopy(fleet.data)
    for slot in ("1a", "2a"):
        data["slots"][slot] = {"state": "empty", "package_id": None}
    for child in data["tasks"].values():
        child.update(stage="await_receipt", status="AWAITING_RECEIPT")
    fleet.save(data)
    receipt = {
        **example("receipt"),
        "site_id": task["site_id"],
        "task_id": task["task_id"],
        "payload": {
            "receipt_id": "receipt-a",
            "package_ids": ["a"],
            "reservation_id": task["reservation_id"],
            "order_id": task["order_id"],
        },
    }
    store.accept_receipt(receipt)
    service_tick(fleet, store)
    assert fleet.data["tasks"][task["task_id"] + ":a"]["stage"] == "return_nav"
    assert fleet.data["tasks"][task["task_id"] + ":b"]["stage"] == "await_receipt"
    receipt["message_id"] = "receipt-message-b"
    receipt["payload"]["receipt_id"] = "receipt-b"
    receipt["payload"]["package_ids"] = ["b"]
    store.accept_receipt(receipt)
    service_tick(fleet, store)
    assert fleet.data["tasks"][task["task_id"] + ":b"]["stage"] == "return_nav"
    assert len(store.receipts()) == 2


def test_cancelled_package_does_not_close_other_in_progress_deliveries(
    operational_fleet, dsn
):
    fleet = operational_fleet
    store = ServiceTaskStore(dsn)
    data = deepcopy(fleet.data)
    for slot, package in [("1a", "a"), ("2a", "b")]:
        data["slots"][slot] = {"state": "occupied", "package_id": package}
    fleet.save(data)
    task = queued(store, operation="customer_service")
    service_tick(fleet, store)
    data = deepcopy(fleet.data)
    data["tasks"][task["task_id"] + ":a"].update(status="CANCELED", stage="done")
    data["tasks"][task["task_id"] + ":b"].update(
        status="AWAITING_RECEIPT", stage="await_receipt"
    )
    fleet.save(data)
    service_tick(fleet, store)
    assert store.pending_tasks()[0]["state"] == "RUNNING"
    data = deepcopy(fleet.data)
    data["tasks"][task["task_id"] + ":b"].update(status="COMPLETE", stage="done")
    fleet.save(data)
    service_tick(fleet, store)
    assert not store.pending_tasks()
    assert store.outbox()[-1]["type"] == "task.failed"


def test_business_package_identity_maps_to_marker_and_survives_shelf(fleet):
    from test_fleet import complete, dispatch

    request = {
        "request_id": "marker-in",
        "operation": "inbound",
        "package_id": "business-package",
        "goal": None,
        "marker": "15",
    }
    fleet.submit(request)
    sent = []
    for _ in range(100):
        command = dispatch(fleet)
        if command is None:
            break
        for reserved in fleet.data["slots"].values():
            if reserved["state"] == "reserved":
                assert reserved["package_id"] == "business-package"
        if "package_id" in command["payload"]:
            sent.append(command["payload"]["package_id"])
        complete(fleet, command)
    assert sent == ["15", "15"]
    slot = fleet.data["tasks"]["marker-in"]["slot"]
    assert fleet.data["slots"][slot] == {
        "state": "occupied",
        "package_id": "business-package",
        "marker": "15",
    }
    outbound = {
        **request,
        "request_id": "marker-out",
        "operation": "outbound",
        "goal": "gate",
    }
    with pytest.raises(ValueError, match="marker differs"):
        fleet.submit({**outbound, "marker": "16"})
    fleet.submit(outbound)
    finish(fleet)
    assert fleet.data["slots"][slot] == {"state": "empty", "package_id": None}
    assert fleet.data["tasks"]["marker-out"]["stage"] == "await_receipt"


@pytest.mark.parametrize("failed", [False, True])
def test_service_worker_exit_holds_owner_and_closes_database(
    fleet, dsn, tmp_path, monkeypatch, failed
):
    import json
    import time

    from test_fleet_runtime import MemoryTransport

    from roport_control_server.adapters.wss.service import ServiceLink
    from roport_control_server.executor import fleet_runtime as module

    config = tmp_path / "fleet.json"
    config.write_text(json.dumps(fleet.config))
    transport = MemoryTransport()
    transport.start = lambda: None
    transport.stop = lambda: None
    monkeypatch.setattr(module, "MqttTransport", lambda *_: transport)
    monkeypatch.setenv("ROPORT_DATABASE_URL", dsn)
    monkeypatch.setenv(
        "ROPORT_SERVICE_WSS_URL", "ws://127.0.0.1/ws/v1/sites/test/control"
    )
    monkeypatch.setenv("ROPORT_SERVICE_TOKEN", "test-only")
    monkeypatch.setenv("ROPORT_SERVICE_SITE", "test")
    monkeypatch.setenv("ROPORT_MQTT_HOST", "127.0.0.1")

    async def ended(*_):
        if failed:
            raise RuntimeError("test storage failed")

    monkeypatch.setattr(ServiceLink, "run", ended)
    monkeypatch.setattr(module.FleetRuntime, "poll", lambda *_: time.sleep(0.01))
    monkeypatch.setattr(
        "sys.argv",
        ["fleet", "--config", str(config), "--socket", str(tmp_path / "owner.sock")],
    )
    with pytest.raises(
        RuntimeError,
        match="test storage failed" if failed else "service link owner ended",
    ):
        module.main()
    reopened = OperationalStore(dsn)
    try:
        assert reopened.data["fault"] == "owner_stopped"
        assert not reopened.data["enabled"]
    finally:
        reopened.close()
