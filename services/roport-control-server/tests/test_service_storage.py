"""Failure and replay checks for the candidate's durable boundary."""

import asyncio
import json
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

import psycopg
import pytest
from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import example, request

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    ServiceTaskStore,
    migrate_service,
)
from roport_control_server.adapters.wss.service import ServiceLink


def test_parallel_replay_creates_one_task_and_one_event(dsn):
    store = ServiceTaskStore(dsn)
    with ThreadPoolExecutor(max_workers=4) as pool:
        tasks = list(pool.map(lambda _: store.accept(request()), range(4)))
    assert len({task["task_id"] for task in tasks}) == 1
    assert len(store.pending_tasks()) == len(store.outbox()) == 1


def test_transaction_rolls_back_request_and_state_when_outbox_write_fails(
    dsn, monkeypatch
):
    store = ServiceTaskStore(dsn)

    def fail(*args):
        raise RuntimeError("injected outbox failure")

    with monkeypatch.context() as patch:
        patch.setattr(store, "_event", fail)
        with pytest.raises(RuntimeError, match="injected"):
            store.accept(request())
    assert store.pending_tasks() == store.outbox() == []
    task = store.accept(request())
    with monkeypatch.context() as patch:
        patch.setattr(store, "_event", fail)
        with pytest.raises(RuntimeError, match="injected"):
            store.update(task["task_id"], "RUNNING")
    assert store.pending_tasks()[0]["state"] == "QUEUED"
    assert len(store.outbox()) == 1


def test_ack_cannot_clear_another_sites_outbox(dsn):
    store = ServiceTaskStore(dsn)
    store.accept(request())
    message = store.outbox()[0]
    store.acknowledge(message["message_id"], "other")
    assert store.outbox() == [message]
    store.acknowledge(message["message_id"], "test")
    store.acknowledge(message["message_id"], "test")
    assert store.outbox() == []


def test_early_receipt_replay_never_becomes_departure_permission(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    receipt = example("receipt")
    receipt["task_id"] = task["task_id"]
    assert store.accept_receipt(receipt)["arrival_verified"] is False
    operational = OperationalStore(dsn)
    try:
        operational.save(
            {
                "tasks": {
                    task["task_id"] + ":package-1": {
                        "stage": "await_receipt",
                        "status": "AWAITING_RECEIPT",
                    }
                }
            }
        )
        assert store.accept_receipt(receipt)["arrival_verified"] is False
        assert store.receipts() == []
        conflicting = deepcopy(receipt)
        conflicting["payload"]["receipt_id"] = "new-receipt"
        with pytest.raises(ValueError, match="message_id reused"):
            store.accept_receipt(conflicting)
        conflicting["message_id"] = "new-receipt-message"
        assert store.accept_receipt(conflicting)["arrival_verified"] is True
        operational.save({"tasks": {}})
        assert store.accept_receipt(conflicting)["arrival_verified"] is True
        conflicting["message_id"] = "another-transmission"
        assert store.accept_receipt(conflicting)["arrival_verified"] is True
    finally:
        operational.close()
    assert len(store.receipts()) == 1


def test_receipt_message_id_cannot_alias_a_request(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    receipt = example("receipt")
    receipt.update(task_id=task["task_id"], message_id=request()["message_id"])
    with pytest.raises(ValueError, match="message_id reused"):
        store.accept_receipt(receipt)
    assert not store.receipts()


def test_migration_repeated_and_legacy_receipt_has_no_arrival_evidence(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    receipt = example("receipt")
    receipt["task_id"] = task["task_id"]
    store.accept_receipt(receipt)
    with psycopg.connect(dsn) as conn:
        conn.execute(
            "ALTER TABLE control_service_receipts DROP COLUMN arrival_verified"
        )
        conn.execute("ALTER TABLE control_service_tasks DROP COLUMN dispatch_locked")
    migrate_service(dsn)
    migrate_service(dsn)
    assert store.accept(request())["task_id"] == task["task_id"]
    assert store.accept_receipt(receipt)["arrival_verified"] is False
    assert len(store.outbox()) == 1


@pytest.mark.parametrize(
    "kind, state",
    [
        ("accepted", "QUEUED"),
        ("progress", "RUNNING"),
        ("completed", "COMPLETED"),
        ("failed", "FAILED"),
        ("rejected", "REJECTED"),
    ],
)
def test_outbound_events_match_shared_examples(dsn, kind, state):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    if state == "COMPLETED":
        store.update(task["task_id"], "RUNNING")
    if state != "QUEUED":
        store.update(task["task_id"], state)
    event = store.outbox()[-1]
    expected = example(kind)
    expected.update({key: event[key] for key in ("message_id", "sent_at", "task_id")})
    assert event == expected


@pytest.mark.parametrize(
    "change",
    [
        lambda m: m.update(site_id="other"),
        lambda m: m.update(source="control-01", target="service-01"),
        lambda m: m.update(schema_version="2.0"),
        lambda m: m.pop("message_id"),
        lambda m: m.update(sent_at="not-a-date"),
        lambda m: m["payload"].update(extra="not-supported"),
    ],
)
def test_invalid_frames_do_not_commit_or_ack(dsn, change):
    store = ServiceTaskStore(dsn)
    incoming = request()
    change(incoming)
    sent = []
    stop = asyncio.Event()

    class Socket:
        async def recv(self):
            stop.set()
            return json.dumps(incoming)

        async def send(self, message):
            sent.append(message)

    async def scenario():
        link = ServiceLink(
            "ws://localhost/ws/v1/sites/test/control", "test-only", "test", store
        )
        await link.exchange(Socket(), stop)

    asyncio.run(scenario())
    assert not sent and not store.pending_tasks() and not store.outbox()


def test_failed_commit_sends_no_ack(dsn, monkeypatch):
    store = ServiceTaskStore(dsn)
    sent = []

    class Socket:
        async def recv(self):
            return json.dumps(request())

        async def send(self, message):
            sent.append(message)

    def fail(*args):
        raise RuntimeError("injected commit failure")

    async def scenario():
        link = ServiceLink(
            "ws://localhost/ws/v1/sites/test/control", "test-only", "test", store
        )
        with pytest.raises(RuntimeError, match="injected"):
            await link.exchange(Socket(), asyncio.Event())

    monkeypatch.setattr(store, "_event", fail)
    asyncio.run(scenario())
    assert not sent and not store.pending_tasks()


@pytest.mark.parametrize(
    "change",
    [
        lambda m: m["payload"].update(status="REJECTED"),
        lambda m: m["payload"].update(message_id=None),
        lambda m: m.update(target="wrong-control"),
    ],
)
def test_malformed_ack_cannot_clear_outbox(dsn, change):
    store = ServiceTaskStore(dsn)
    store.accept(request())
    pending = store.outbox()
    ack = example("ack-event")
    ack["payload"]["message_id"] = pending[0]["message_id"]
    change(ack)
    stop = asyncio.Event()

    class Socket:
        async def recv(self):
            stop.set()
            return json.dumps(ack)

        async def send(self, message):
            assert json.loads(message)["type"] != "message.ack"

    async def scenario():
        link = ServiceLink(
            "ws://localhost/ws/v1/sites/test/control", "test-only", "test", store
        )
        await link.exchange(Socket(), stop)

    asyncio.run(scenario())
    assert store.outbox() == pending


def test_stored_receipt_replay_survives_later_manifest_change(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    receipt = example("receipt")
    receipt["task_id"] = task["task_id"]
    assert store.accept_receipt(receipt)["arrival_verified"] is False
    changed = request()
    changed["message_id"] = "changed-manifest"
    changed["payload"].update(
        order_revision=1, packages=[{"package_id": "replacement", "quantity": 1}]
    )
    store.accept(changed)
    assert store.accept_receipt(receipt)["arrival_verified"] is False
    receipt["message_id"] = "receipt-retransmission"
    assert store.accept_receipt(receipt)["arrival_verified"] is False
    assert store.receipts() == []
