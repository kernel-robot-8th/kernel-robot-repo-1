from copy import deepcopy
from datetime import UTC, datetime

import pytest
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401
from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import request

from roport_control_server.adapters.db.service_tasks import ServiceTaskStore
from roport_control_server.contracts.service_policy import dispatch_at
from roport_control_server.executor.fleet_runtime import service_tick


def test_lead_time_is_separate_from_customer_arrival_time():
    p = request()["payload"]
    assert dispatch_at(p, {}) == datetime(2030, 1, 1, 0, 55, tzinfo=UTC)
    assert dispatch_at(p, {"dispatch_lead_s": 60}) == datetime(
        2030, 1, 1, 0, 59, tzinfo=UTC
    )
    assert p["pickup_at"] == "2030-01-01T01:00:00Z"


@pytest.mark.parametrize("bad", [-1, float("nan"), float("inf"), True, "60"])
def test_invalid_lead_time(bad):
    with pytest.raises(ValueError):
        dispatch_at(request()["payload"], {"dispatch_lead_s": bad})


def test_manifest_update_is_atomic_with_dispatch_claim(dsn):
    store = ServiceTaskStore(dsn)
    original = request()
    original["payload"]["packages"] = []
    task = store.accept(original)
    update = request()
    update["message_id"] = "manifest-1"
    update["payload"]["order_revision"] = 1
    assert store.accept(update)["task_id"] == task["task_id"]
    assert store.claim(task["task_id"], 0) is None
    assert store.claim(task["task_id"], 1)["request"] == update["payload"]
    changed = deepcopy(update)
    changed["message_id"] = "manifest-2"
    changed["payload"]["order_revision"] = 2
    changed["payload"]["packages"] = [{"package_id": "replacement", "quantity": 1}]
    rejected = store.accept(changed)
    assert rejected["request"] == update["payload"]
    assert (
        rejected["progress"]["manifest_change"]["reason"] == "dispatch_already_locked"
    )
    count = len(store.outbox())
    store.accept(changed)
    assert len(store.outbox()) == count  # refusal is also stored before transport ACK


def test_future_request_waits_without_claiming_and_starts_at_lead_time(fleet, dsn):
    store = ServiceTaskStore(dsn)
    message = request()
    message["payload"]["pickup_point_id"] = "gate"
    data = deepcopy(fleet.data)
    data["slots"]["1a"] = {"state": "occupied", "package_id": "package-1"}
    fleet.save(data)
    store.accept(message)
    service_tick(fleet, store, now=datetime(2030, 1, 1, 0, 54, 59, tzinfo=UTC))
    assert not fleet.data["tasks"] and not store.pending_tasks()[0]["dispatch_locked"]
    service_tick(fleet, store, now=datetime(2030, 1, 1, 0, 55, tzinfo=UTC))
    assert len(fleet.data["tasks"]) == 1 and store.pending_tasks()[0]["dispatch_locked"]


def test_old_manifest_refusal_is_durable_and_keeps_live_progress(dsn):
    store = ServiceTaskStore(dsn)
    current = request()
    current["payload"]["order_revision"] = 2
    task = store.accept(current)
    progress = {
        "packages": [
            {"package_id": "package-1", "robot": "pinky3", "stage": "await_receipt"}
        ]
    }
    store.update(task["task_id"], "RUNNING", progress)
    stale = request()
    stale["message_id"] = "stale-manifest"
    result = store.accept(stale)
    assert result["request"] == current["payload"]
    assert result["progress"]["packages"] == progress["packages"]
    assert result["progress"]["manifest_change"]["reason"] == "order_revision_not_newer"
    count = len(store.outbox())
    store.accept(stale)
    assert len(store.outbox()) == count
    refreshed = store.update(task["task_id"], "RUNNING", progress)
    assert (
        refreshed["progress"]["manifest_change"]
        == result["progress"]["manifest_change"]
    )
    assert len(store.outbox()) == count


def test_corrected_manifest_can_leave_pre_dispatch_hold(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    store.update(task["task_id"], "HOLD", {"reason": "package missing"})
    revised = request()
    revised["message_id"] = "corrected"
    revised["payload"]["order_revision"] = 1
    revised["payload"]["packages"] = [{"package_id": "corrected-box", "quantity": 1}]
    assert store.accept(revised)["state"] == "QUEUED"
