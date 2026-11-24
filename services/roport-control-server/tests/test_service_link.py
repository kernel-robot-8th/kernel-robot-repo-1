"""Durable transport tests against an isolated schema, never a field database."""

import asyncio
import json
import os
from copy import deepcopy
from pathlib import Path
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


EXAMPLES = json.loads(
    (
        Path(__file__).resolve().parents[3] / "interfaces/wss/candidate/examples.json"
    ).read_text()
)


def example(label):
    return deepcopy(next(row["message"] for row in EXAMPLES if row["label"] == label))


def request():
    return example("request")


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
    store.acknowledge(store.outbox()[0]["message_id"], "test")
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
    receipt = example("receipt")
    receipt["task_id"] = task["task_id"]
    receipt["payload"].update(reservation_id="wrong", order_id="order-2")
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
                    ack = example("ack-event")
                    ack["payload"]["message_id"] = event["message_id"]
                    await ws.send(json.dumps(ack))
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


def test_replaced_connection_does_not_reconnect(dsn):
    async def scenario():
        connections = 0

        async def server(ws):
            nonlocal connections
            connections += 1
            await ws.close(code=4001, reason="replaced")

        async with serve(server, "127.0.0.1", 0) as host:
            port = host.sockets[0].getsockname()[1]
            link = ServiceLink(
                f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                "test-only-token",
                "test",
                ServiceTaskStore(dsn),
            )
            await asyncio.wait_for(link.run(asyncio.Event()), 4)
        assert connections == 1

    asyncio.run(scenario())
