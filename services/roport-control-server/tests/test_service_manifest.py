from copy import deepcopy

from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import request

from roport_control_server.adapters.db.service_tasks import ServiceTaskStore


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
