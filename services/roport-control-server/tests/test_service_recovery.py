"""Review regressions: refused frames, database interruption and dispatch ordering."""

import asyncio
import json
from copy import deepcopy

import psycopg
import pytest
from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import example, request
from websockets.asyncio.server import serve

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    ServiceTaskStore,
    migrate_service,
)
from roport_control_server.adapters.wss.service import ServiceLink
from roport_control_server.contracts.service_wire import ServiceStorageError


def test_repeated_migration_keeps_predispatch_hold_editable_and_claim_locked(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    store.update(task["task_id"], "HOLD", {"reason": "missing stock"})
    migrate_service(dsn)
    migrate_service(dsn)
    changed = request()
    changed["message_id"] = "revised"
    changed["payload"]["order_revision"] = 1
    changed["payload"]["packages"] = [{"package_id": "replacement", "quantity": 1}]
    assert store.accept(changed)["state"] == "QUEUED"
    assert store.claim(task["task_id"], 1)["dispatch_locked"] is True
    migrate_service(dsn)
    assert store.pending_tasks()[0]["dispatch_locked"] is True


def test_running_then_hold_locks_manifest_without_explicit_claim(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    assert store.update(task["task_id"], "RUNNING")["dispatch_locked"] is True
    store.update(task["task_id"], "HOLD")
    changed = request()
    changed["message_id"] = "revised"
    changed["payload"]["order_revision"] = 1
    saved = store.accept(changed)
    assert saved["state"] == "HOLD" and saved["request"] == task["request"]
    assert saved["progress"]["manifest_change"]["reason"] == "dispatch_already_locked"


def test_legacy_child_evidence_locks_hold_without_matching_sql_wildcards(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    store.update(task["task_id"], "HOLD")
    owner = OperationalStore(dsn)
    try:
        owner.save({"tasks": {task["task_id"] + ":package-1": {}}})
    finally:
        owner.close()
    migrate_service(dsn)
    assert store.pending_tasks()[0]["dispatch_locked"] is True


def test_outbox_sequence_ignores_transaction_start_times_and_limits_batch(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    for i in range(105):
        store.update(task["task_id"], "RUNNING", {"sample": i})
    with psycopg.connect(dsn) as conn:
        conn.execute(
            "UPDATE control_service_outbox SET created_at=now() - ((message->'payload'->>'task_revision')::int * interval '1 second')"
        )
    batch = store.outbox()
    assert [m["payload"]["task_revision"] for m in batch] == list(range(100))
    for m in batch:
        store.acknowledge(m["message_id"], "test")
    assert [m["payload"]["task_revision"] for m in store.outbox()] == list(
        range(100, 106)
    )
    migrate_service(dsn)
    store.update(task["task_id"], "HOLD")
    assert [m["payload"]["task_revision"] for m in store.outbox()] == list(
        range(100, 107)
    )


def test_rejected_and_malformed_frames_do_not_block_receipt_ack_or_next_request(dsn):
    async def scenario():
        store = ServiceTaskStore(dsn)
        original = store.accept(request())
        rejected = request()
        rejected["message_id"] = "refused"
        rejected["payload"]["reservation_id"] = "another-reservation"
        receipt = example("receipt")
        receipt["task_id"] = original["task_id"]
        next_request = request()
        next_request["message_id"] = "next"
        next_request["payload"].update(
            order_id="next-order", reservation_id="next-reservation"
        )
        pending = store.outbox()[0]
        ack = example("ack-event")
        ack["payload"]["message_id"] = pending["message_id"]
        stop, connections = asyncio.Event(), []

        async def server(ws):
            connections.append(ws)
            for frame in [
                "{bad json",
                json.dumps({"type": "invalid"}),
                *map(json.dumps, [rejected, receipt, ack, next_request]),
            ]:
                await ws.send(frame)
            ids = set()
            while len(ids) < 3:
                message = json.loads(await ws.recv())
                if message["type"] == "message.ack":
                    ids.add(message["payload"]["message_id"])
            assert ids == {"refused", receipt["message_id"], "next"}
            stop.set()

        async with serve(server, "127.0.0.1", 0) as host:
            port = host.sockets[0].getsockname()[1]
            await asyncio.wait_for(
                ServiceLink(
                    f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                    "test-token",
                    "test",
                    store,
                ).run(stop),
                6,
            )
        assert len(connections) == 1
        assert pending not in store.outbox()
        assert len(store.pending_tasks()) == 2
        with psycopg.connect(dsn) as conn:
            assert conn.execute(
                "SELECT reason FROM control_service_quarantine"
            ).fetchone() == ("active_order",)
            assert conn.execute(
                "SELECT arrival_verified FROM control_service_receipts"
            ).fetchone() == (False,)
        store.update(original["task_id"], "COMPLETED")
        assert (
            store.receive(deepcopy(rejected)) == "active_order"
        )  # Retry cannot resurrect refused work.
        with psycopg.connect(dsn) as conn:
            assert (
                conn.execute(
                    "SELECT count(*) FROM control_service_quarantine"
                ).fetchone()[0]
                == 1
            )

    asyncio.run(scenario())


@pytest.mark.parametrize("point", ["outbox", "accept", "acknowledge"])
def test_database_failure_reconnects_without_ack_and_recovers(dsn, monkeypatch, point):
    async def scenario():
        store = ServiceTaskStore(dsn)
        stop, failed, connections = asyncio.Event(), False, []
        original_connect = psycopg.connect
        target = getattr(store, point)

        def once(*args):
            nonlocal failed
            if not failed:
                failed = True
                with monkeypatch.context() as patch:
                    patch.setattr(
                        psycopg,
                        "connect",
                        lambda *_a, **_k: (_ for _ in ()).throw(
                            psycopg.OperationalError("test-only DB outage")
                        ),
                    )
                    return target(*args)
            return target(*args)

        monkeypatch.setattr(store, point, once)

        async def server(ws):
            connections.append(ws)
            await ws.send(json.dumps(request()))
            try:
                while True:
                    try:
                        event = json.loads(await asyncio.wait_for(ws.recv(), 0.1))
                    except TimeoutError:
                        event = {}
                    if event.get("type") == "task.accepted":
                        ack = example("ack-event")
                        ack["payload"]["message_id"] = event["message_id"]
                        await ws.send(json.dumps(ack))
                    if len(connections) >= 2 and failed and not store.outbox():
                        assert len(store.pending_tasks()) == 1
                        stop.set()
                        return
            except __import__("websockets").exceptions.ConnectionClosed:
                pass

        async with serve(server, "127.0.0.1", 0) as host:
            port = host.sockets[0].getsockname()[1]
            await asyncio.wait_for(
                ServiceLink(
                    f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                    "test-token",
                    "test",
                    store,
                ).run(stop),
                8,
            )
        assert psycopg.connect is original_connect
        assert failed and len(connections) >= 2

    asyncio.run(scenario())


def test_service_storage_error_keeps_driver_exception_out_of_transport(
    dsn, monkeypatch
):
    monkeypatch.setattr(
        psycopg,
        "connect",
        lambda *_a, **_k: (_ for _ in ()).throw(
            psycopg.OperationalError("private-dsn")
        ),
    )
    with pytest.raises(
        ServiceStorageError, match="service storage transaction failed"
    ) as exc:
        ServiceTaskStore(dsn).outbox()
    assert "private-dsn" not in str(exc.value)


def test_commit_disconnect_rolls_back_without_quarantine_or_acknowledged_work(
    dsn, monkeypatch
):
    store = ServiceTaskStore(dsn)
    event = store._event

    def disconnect(conn, task, kind):
        event(conn, task, kind)
        conn.close()

    with monkeypatch.context() as patch:
        patch.setattr(store, "_event", disconnect)
        with pytest.raises(ServiceStorageError):
            store.receive(request())
    assert store.pending_tasks() == store.outbox() == []
    with psycopg.connect(dsn) as conn:
        assert (
            conn.execute("SELECT count(*) FROM control_service_quarantine").fetchone()[
                0
            ]
            == 0
        )
    assert store.receive(request()) is None
    assert len(store.pending_tasks()) == len(store.outbox()) == 1


def test_legacy_outbox_is_backfilled_in_revision_order_once(dsn):
    store = ServiceTaskStore(dsn)
    task = store.accept(request())
    store.update(task["task_id"], "RUNNING")
    store.update(task["task_id"], "HOLD")
    with psycopg.connect(dsn) as conn:
        # Only this fixture's isolated schema, to emulate the pre-sequence table.
        conn.execute("ALTER TABLE control_service_outbox DROP COLUMN seq")
        conn.execute(
            "UPDATE control_service_outbox SET created_at=now() - ((message->'payload'->>'task_revision')::int * interval '1 second')"
        )
    migrate_service(dsn)
    store.update(task["task_id"], "RUNNING")
    migrate_service(dsn)
    store.update(task["task_id"], "COMPLETED")
    assert [m["payload"]["task_revision"] for m in store.outbox()] == list(range(5))
