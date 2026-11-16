"""Durable transport tests against an isolated schema, never a field database."""

import asyncio
import json
import os
from copy import deepcopy
from uuid import uuid4

import psycopg
import pytest
from psycopg.conninfo import make_conninfo
from websockets.asyncio.server import serve

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    ServiceTaskStore,
    migrate_service,
)
from roport_control_server.adapters.wss.service import ServiceLink


@pytest.fixture
def dsn():
    base = os.environ.get("ROPORT_TEST_DATABASE_URL")
    if not base:
        if os.environ.get("REQUIRE_REGISTRY_TESTS") == "1":
            pytest.fail("ROPORT_TEST_DATABASE_URL required")
        pytest.skip("isolated PostgreSQL not configured")
    schema = "service_test_" + uuid4().hex
    with psycopg.connect(base, autocommit=True) as conn:
        conn.execute(f'CREATE SCHEMA "{schema}"')
    url = make_conninfo(base, options=f"-csearch_path={schema}")
    migrate_service(url)
    try:
        yield url
    finally:
        with psycopg.connect(base, autocommit=True) as conn:
            conn.execute(f'DROP SCHEMA "{schema}" CASCADE')


def request():
    return {
        "schema_version": "1.0",
        "message_id": "request-1",
        "type": "task.requested",
        "site_id": "test",
        "source": "service-01",
        "target": "control-01",
        "sent_at": "2030-01-01T00:00:00Z",
        "payload": {
            "order_id": "order-1",
            "reservation_id": "reservation-1",
            "operation": "customer_service",
            "packages": [{"package_id": "package-1", "quantity": 1}],
            "pickup_point_id": "G1",
            "pickup_at": "2030-01-01T01:00:00Z",
            "order_revision": 0,
        },
    }


def test_replay_conflicts_restart_and_terminal(dsn):
    store = ServiceTaskStore(dsn)
    msg = request()
    task = store.accept(msg)
    assert ServiceTaskStore(dsn).accept(msg)["task_id"] == task["task_id"]
    assert len(store.outbox()) == 1
    bad = deepcopy(msg)
    bad["payload"]["pickup_point_id"] = "G2"
    with pytest.raises(ValueError, match="message_id reused"):
        store.accept(bad)
    second = deepcopy(msg)
    second["message_id"] = "request-2"
    second["payload"]["reservation_id"] = "reservation-2"
    with pytest.raises(ValueError, match="active task"):
        store.accept(second)
    store.update(task["task_id"], "RUNNING", {"stage": "to_delivery"})
    store.update(task["task_id"], "COMPLETED")
    store.update(task["task_id"], "COMPLETED")
    assert len(store.outbox()) == 3
    with pytest.raises(ValueError, match="terminal"):
        store.update(task["task_id"], "RUNNING")
    store.acknowledge(store.outbox()[0]["message_id"])
    assert len(ServiceTaskStore(dsn).outbox()) == 2
    assert store.pending_tasks() == []


def test_manifest_wait_and_receipt_binding(dsn):
    store = ServiceTaskStore(dsn)
    msg = request()
    msg["payload"]["packages"] = []
    task = store.accept(msg)
    assert task["state"] == "WAITING_MANIFEST"
    with pytest.raises(ValueError, match="manifest"):
        store.update(task["task_id"], "RUNNING")
    second = request()
    second["message_id"] = "request-2"
    second["payload"].update(order_id="order-2", reservation_id="reservation-2")
    task = store.accept(second)
    receipt = {
        "type": "handover.receipt_recorded",
        "site_id": "test",
        "task_id": task["task_id"],
        "payload": {
            "receipt_id": "receipt-1",
            "reservation_id": "wrong",
            "order_id": "order-2",
            "package_ids": ["package-1"],
        },
    }
    with pytest.raises(ValueError, match="match"):
        store.accept_receipt(receipt)
    receipt["payload"]["reservation_id"] = "reservation-2"
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
        assert store.accept_receipt(receipt)["arrival_verified"] is True
        assert store.accept_receipt(receipt)["arrival_verified"] is True
    finally:
        operational.close()
    assert len(store.receipts()) == 1


def test_operational_snapshot_single_owner_and_restart(dsn):
    first = OperationalStore(dsn)
    try:
        with pytest.raises(RuntimeError, match="another"):
            OperationalStore(dsn)
        data = {"resources": {"corridor": "pinky1"}, "tasks": ["a"]}
        first.save(data)
        data["tasks"].append("outside")
        assert first.data["tasks"] == ["a"]
    finally:
        first.close()
    second = OperationalStore(dsn)
    try:
        assert second.data["resources"] == {"corridor": "pinky1"}
    finally:
        second.close()


def test_real_websocket_reconnect_ack_after_commit(dsn):
    async def scenario():
        stop = asyncio.Event()
        store = ServiceTaskStore(dsn)
        connections = 0
        task_id = None

        async def server(ws):
            nonlocal connections, task_id
            connections += 1
            assert ws.request.headers["Authorization"] == "Bearer test-only-token"
            await ws.send(json.dumps(request()))
            while True:
                event = json.loads(await ws.recv())
                if event["type"] == "message.ack":
                    # ACK means another DB connection sees the committed request.
                    tasks = ServiceTaskStore(dsn).pending_tasks()
                    assert len(tasks) == 1
                    task_id = tasks[0]["task_id"]
                    if connections == 1:
                        await ws.close()
                        return
                if event["type"] == "task.accepted" and connections > 1:
                    await ws.send(
                        json.dumps(
                            {
                                "type": "message.ack",
                                "site_id": "test",
                                "payload": {"message_id": event["message_id"]},
                            }
                        )
                    )
                    for _ in range(50):
                        if not store.outbox():
                            stop.set()
                            return
                        await asyncio.sleep(0.01)

        async with serve(server, "127.0.0.1", 0) as host:
            port = host.sockets[0].getsockname()[1]
            link = ServiceLink(
                f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                "test-only-token",
                "test",
                store,
            )
            await asyncio.wait_for(link.run(stop), 8)
        assert connections == 2
        assert task_id
        assert len(store.pending_tasks()) == 1
        assert not store.outbox()

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "url",
    [
        "ws://example.com/ws/v1/sites/test/control",
        "wss://user:pass@example.com/ws/v1/sites/test/control",
        "wss://example.com/ws/v1/sites/other/control",
    ],
)
def test_reject_insecure_or_mismatched_endpoint(url):
    with pytest.raises(ValueError):
        ServiceLink(url, "test-token", "test", None)
