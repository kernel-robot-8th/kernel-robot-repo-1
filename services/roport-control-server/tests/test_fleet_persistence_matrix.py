"""PostgreSQL/WSS fault matrix. Stage checkpoints are explicit, not physical-motion evidence."""

import asyncio
import json
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import UTC, datetime
from time import monotonic
from types import SimpleNamespace

import psycopg
import pytest
from psycopg.types.json import Jsonb
from test_fleet import complete, dispatch, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import MemoryTransport
from test_pinky_trial import profile as profile  # noqa: PLC0414
from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import request
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    ServiceTaskStore,
    migrate_service,
)
from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.wss.service import ServiceLink
from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.fleet_runtime import FleetRuntime, service_tick


@pytest.fixture
def operational(fleet, dsn):
    with_store = OperationalStore(dsn)
    with_store.save(fleet.data)
    fleet.store = with_store
    try:
        yield fleet, ServiceTaskStore(dsn)
    finally:
        with_store.close()


def incoming(order="order-1", packages=("a",), *, operation="prepare_order"):
    message = request()
    message["message_id"] = "message-" + order
    message["payload"].update(
        order_id=order,
        reservation_id="reservation-" + order,
        packages=[{"package_id": p, "quantity": 1} for p in packages],
        operation=operation,
        pickup_point_id="gate",
    )
    return message


def customer_task(
    control, store, order="order-1", packages=("a",), stage="await_receipt"
):
    data = deepcopy(control.data)
    for package in packages:
        slot = next(
            key for key, value in data["slots"].items() if value["state"] == "empty"
        )
        data["slots"][slot] = {"state": "occupied", "package_id": package}
    control.save(data)
    task = store.accept(incoming(order, packages, operation="customer_service"))
    service_tick(control, store)
    # Explicit boundary fixture; the full-motion E2E lives in service_fleet_e2e.py.
    data = deepcopy(control.data)
    for package in packages:
        data["tasks"][task["task_id"] + ":" + package].update(
            stage=stage,
            status="AWAITING_RECEIPT" if stage == "await_receipt" else "QUEUED",
        )
    if stage == "await_receipt":
        for slot in data["slots"].values():
            if slot["package_id"] in packages:
                slot.update(state="empty", package_id=None)
    control.save(data)
    return task


def receipt(task, packages=("a",), identifier="receipt-1"):
    return {
        "message_id": "message-" + identifier,
        "type": "handover.receipt_recorded",
        "site_id": task["site_id"],
        "task_id": task["task_id"],
        "payload": {
            "receipt_id": identifier,
            "reservation_id": task["reservation_id"],
            "order_id": task["order_id"],
            "package_ids": list(packages),
        },
    }


def child(control, task, package="a"):
    return control.data["tasks"][task["task_id"] + ":" + package]


def fail_write(dsn, table, operation="UPDATE", condition="true"):
    # Each dsn is a disposable UUID schema; never installed in a shared/field DB.
    with psycopg.connect(dsn) as conn:
        conn.execute(
            f"CREATE FUNCTION matrix_reject() RETURNS trigger LANGUAGE plpgsql AS $$ BEGIN IF {condition} THEN RAISE EXCEPTION 'matrix injected storage failure'; END IF; RETURN NEW; END $$"
        )
        conn.execute(
            f"CREATE TRIGGER matrix_reject BEFORE {operation} ON {table} FOR EACH ROW EXECUTE FUNCTION matrix_reject()"
        )


def counts(dsn):
    with psycopg.connect(dsn) as conn:
        return {
            table: conn.execute("SELECT count(*) FROM " + table).fetchone()[0]
            for table in (
                "control_service_tasks",
                "control_service_inbox",
                "control_service_outbox",
            )
        }


def test_concurrent_identical_requests_commit_one_task_and_acceptance(dsn):
    message = incoming()
    with ThreadPoolExecutor(max_workers=8) as workers:
        tasks = list(
            workers.map(lambda _: ServiceTaskStore(dsn).accept(message), range(8))
        )
    assert len({task["task_id"] for task in tasks}) == 1
    assert set(counts(dsn).values()) == {1}


@pytest.mark.parametrize(
    "field,value",
    [
        ("order_id", "different-order"),
        ("reservation_id", "different-reservation"),
        ("packages", [{"package_id": "different-package", "quantity": 1}]),
    ],
)
def test_reused_message_id_different_body_does_not_mutate_committed_task(
    dsn, field, value
):
    store, message = ServiceTaskStore(dsn), incoming()
    accepted = store.accept(message)
    message["payload"][field] = value
    with pytest.raises(ValueError, match="message_id reused"):
        store.accept(message)
    assert store.pending_tasks()[0]["request"] == accepted["request"]
    assert set(counts(dsn).values()) == {1}


def test_same_reservation_new_message_id_still_has_one_child(operational):
    control, store = operational
    message = incoming()
    task = store.accept(message)
    message["message_id"] = "different-message-same-reservation"
    assert store.accept(message)["task_id"] == task["task_id"]
    service_tick(control, store)
    once = store.outbox()
    service_tick(control, store)
    assert len(control.data["tasks"]) == 1
    assert [event["type"] for event in once] == ["task.accepted", "task.progress"]
    assert store.outbox() == once


def test_two_orders_same_package_cannot_create_two_physical_tasks(operational):
    control, store = operational
    a = store.accept(incoming("first"))
    b = store.accept(incoming("second"))
    service_tick(control, store)
    assert len(control.data["tasks"]) == 1
    assert set(control.data["tasks"]) == {a["task_id"] + ":a"}
    states = {task["task_id"]: task["state"] for task in store.pending_tasks()}
    assert states[a["task_id"]] == "QUEUED" and states[b["task_id"]] == "HOLD"
    assert control.data["fault"] is None and control.data["commands"] == {}


def test_unrelated_orders_and_receipt_do_not_change_other_child(operational):
    control, store = operational
    a = customer_task(control, store, "first", ("a",))
    b = customer_task(control, store, "second", ("b",))
    before = deepcopy(child(control, b, "b"))
    store.accept_receipt(receipt(a))
    service_tick(control, store)
    assert child(control, a)["stage"] == "return_nav"
    assert child(control, b, "b") == before


def test_partial_duplicate_receipt_preserves_unconfirmed_packages(operational):
    control, store = operational
    task = customer_task(control, store, packages=("a", "b"))
    first = receipt(task)
    store.accept_receipt(first)
    store.accept_receipt(first)
    service_tick(control, store)
    assert child(control, task)["stage"] == "return_nav"
    assert child(control, task, "b")["stage"] == "await_receipt"
    before = deepcopy(control.data)
    service_tick(control, store)
    assert control.data == before and len(store.receipts()) == 1
    store.accept_receipt(receipt(task, ("b",), "receipt-b"))
    service_tick(control, store)
    assert child(control, task, "b")["stage"] == "return_nav"


@pytest.mark.parametrize(
    "field,value",
    [
        ("task_id", "other-task"),
        ("site_id", "other-site"),
        ("order_id", "other-order"),
        ("reservation_id", "other-reservation"),
        ("package_ids", ["other-package"]),
    ],
)
def test_receipt_identity_mismatch_preserves_tasks_and_receipt_store(
    operational, field, value
):
    control, store = operational
    task = customer_task(control, store)
    message = receipt(task)
    target = message if field in {"task_id", "site_id"} else message["payload"]
    target[field] = value
    before = deepcopy(control.data)
    with pytest.raises(ValueError, match="match"):
        store.accept_receipt(message)
    assert control.data == before and not store.receipts()


def test_receipt_predating_customer_arrival_cannot_later_authorize_return(operational):
    control, store = operational
    task = customer_task(control, store, stage="queued")
    try:
        store.accept_receipt(receipt(task))
    except ValueError:
        return  # Rejecting an impossible early receipt is a valid boundary behavior.
    service_tick(control, store)
    assert child(control, task)["stage"] == "queued"
    data = deepcopy(control.data)
    data["tasks"][task["task_id"] + ":a"].update(
        stage="await_receipt", status="AWAITING_RECEIPT"
    )
    control.save(data)
    service_tick(control, store)
    assert child(control, task)["stage"] == "await_receipt", (
        "Receipt stored before delivery silently authorized return when arrival later occurred"
    )


def test_early_receipt_verdict_survives_arrival_replay_and_new_receipt(operational):
    control, store = operational
    task = customer_task(control, store, stage="queued")
    message = receipt(task)
    original = store.accept_receipt(message)
    assert original["arrival_verified"] is False
    data = deepcopy(control.data)
    data["tasks"][task["task_id"] + ":a"].update(
        stage="await_receipt", status="AWAITING_RECEIPT"
    )
    control.save(data)
    assert store.accept_receipt(message) == original
    service_tick(control, store)
    assert not store.receipts()
    assert child(control, task)["stage"] == "await_receipt"
    fresh = receipt(task, identifier="receipt-after-arrival")
    verified = store.accept_receipt(fresh)
    assert verified["arrival_verified"] is True
    service_tick(control, store)
    assert child(control, task)["stage"] == "return_nav"
    assert store.accept_receipt(fresh) == verified
    assert store.accept_receipt(message) == original


def test_receipt_group_with_one_unarrived_package_never_partially_authorizes(
    operational,
):
    control, store = operational
    task = customer_task(control, store, packages=("a", "b"))
    data = deepcopy(control.data)
    data["tasks"][task["task_id"] + ":b"].update(stage="navigate", status="RUNNING")
    control.save(data)
    message = receipt(task, packages=("a", "b"))
    assert store.accept_receipt(message)["arrival_verified"] is False
    service_tick(control, store)
    assert child(control, task, "a")["stage"] == "await_receipt"
    assert child(control, task, "b")["stage"] == "navigate"
    data = deepcopy(control.data)
    data["tasks"][task["task_id"] + ":b"].update(
        stage="await_receipt", status="AWAITING_RECEIPT"
    )
    control.save(data)
    assert store.accept_receipt(message)["arrival_verified"] is False
    service_tick(control, store)
    assert all(child(control, task, p)["stage"] == "await_receipt" for p in ("a", "b"))
    assert (
        store.accept_receipt(
            receipt(task, packages=("a", "b"), identifier="both-arrived")
        )["arrival_verified"]
        is True
    )
    service_tick(control, store)
    assert all(child(control, task, p)["stage"] == "return_nav" for p in ("a", "b"))


def test_legacy_receipts_migrate_unverified_without_deletion_or_later_revival(
    operational, dsn
):
    control, store = operational
    task = customer_task(control, store)
    message = receipt(task)
    with psycopg.connect(dsn) as conn:
        conn.execute(
            "ALTER TABLE control_service_receipts DROP COLUMN arrival_verified"
        )
        conn.execute(
            "INSERT INTO control_service_receipts(task_id,receipt_id,message) VALUES (%s,%s,%s)",
            (task["task_id"], message["payload"]["receipt_id"], Jsonb(message)),
        )
    migrate_service(dsn)
    migrate_service(dsn)
    original = store.accept_receipt(message)
    assert original["message"] == message and original["arrival_verified"] is False
    assert not store.receipts()
    service_tick(control, store)
    assert child(control, task)["stage"] == "await_receipt"
    assert (
        store.accept_receipt(receipt(task, identifier="fresh-confirmation"))[
            "arrival_verified"
        ]
        is True
    )
    service_tick(control, store)
    assert child(control, task)["stage"] == "return_nav"


@pytest.mark.parametrize("stage", ["queued", "await_receipt"])
def test_receipt_db_failure_never_sends_websocket_ack(operational, dsn, stage):
    control, store = operational
    task = customer_task(control, store, stage=stage)
    for event in store.outbox():
        store.acknowledge(event["message_id"])
    fail_write(dsn, "control_service_receipts", operation="INSERT")

    async def scenario():
        received = []

        async def peer(socket):
            await socket.send(json.dumps(receipt(task)))
            try:
                received.append(json.loads(await asyncio.wait_for(socket.recv(), 0.3)))
            except (TimeoutError, ConnectionClosed):
                pass

        async with serve(peer, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            link = ServiceLink(
                f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                "test-token",
                "test",
                store,
            )
            with pytest.raises(psycopg.Error, match="matrix injected"):
                await asyncio.wait_for(link.run(asyncio.Event()), 3)
        assert not received
        with psycopg.connect(dsn) as conn:
            assert (
                conn.execute(
                    "SELECT count(*) FROM control_service_receipts"
                ).fetchone()[0]
                == 0
            )
        assert child(control, task)["stage"] == stage

    asyncio.run(scenario())


@pytest.mark.parametrize("stage", ["queued", "await_receipt"])
def test_receipt_ack_loss_preserves_first_arrival_verdict_after_reconnect(
    operational, dsn, stage
):
    control, store = operational
    task = customer_task(control, store, stage=stage)
    for event in store.outbox():
        store.acknowledge(event["message_id"])
    expected = stage == "await_receipt"

    async def scenario():
        stop, connections, errors, verdicts = asyncio.Event(), [], [], []

        async def peer(socket):
            connections.append(len(connections) + 1)
            try:
                await socket.send(json.dumps(receipt(task)))
                event = json.loads(await asyncio.wait_for(socket.recv(), 2))
                assert event["type"] == "message.ack"
                assert event["payload"]["message_id"] == receipt(task)["message_id"]
                with psycopg.connect(dsn) as conn:
                    rows = conn.execute(
                        "SELECT arrival_verified FROM control_service_receipts"
                    ).fetchall()
                assert rows == [
                    (expected,)
                ]  # Another connection sees commit before ACK.
                verdicts.append(rows[0][0])
                if len(connections) == 1:
                    data = deepcopy(control.data)
                    data["tasks"][task["task_id"] + ":a"].update(
                        stage="return_nav" if expected else "await_receipt",
                        status="RUNNING" if expected else "AWAITING_RECEIPT",
                    )
                    control.save(data)
                    await (
                        socket.close()
                    )  # Service loses ACK and resends on its next session.
                else:
                    stop.set()
            except Exception as exc:  # noqa: BLE001 - surface peer assertions
                errors.append(exc)
                stop.set()

        async with serve(peer, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            link = ServiceLink(
                f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                "test-token",
                "test",
                store,
            )
            await asyncio.wait_for(link.run(stop), 8)
        assert not errors, errors
        assert len(connections) == 2 and verdicts == [expected, expected]
        service_tick(control, store)
        assert child(control, task)["stage"] == (
            "return_nav" if expected else "await_receipt"
        )

    asyncio.run(scenario())


def test_old_completed_task_receipt_does_not_release_new_order(operational):
    control, store = operational
    old = customer_task(control, store, "old")
    data = deepcopy(control.data)
    data["tasks"][old["task_id"] + ":a"].update(stage="done", status="COMPLETE")
    control.save(data)
    service_tick(control, store)
    current = customer_task(control, store, "new")
    store.accept_receipt(receipt(old))
    service_tick(control, store)
    assert child(control, current)["stage"] == "await_receipt"
    assert child(control, old)["stage"] == "done"


@pytest.mark.parametrize("phase", ["idle", "active", "await_receipt"])
def test_restart_preserves_work_but_never_automatically_dispatches(
    operational, dsn, phase
):
    control, store = operational
    if phase == "active":
        submit(control, "box")
        dispatch(control)
    elif phase == "await_receipt":
        customer_task(control, store)
    before = deepcopy(control.data)
    control.store.close()
    reopened = OperationalStore(dsn)
    try:
        restarted = FleetControl(control.config, reopened, clock=lambda: 1)
        assert not restarted.data["enabled"] and restarted.next_action() is None
        assert restarted.data["roles"] == before["roles"]
        assert restarted.data["slots"] == before["slots"]
        if phase == "active":
            assert restarted.data["commands"] == before["commands"]
            assert restarted.data["resources"] == before["resources"]
            assert next(iter(restarted.data["tasks"].values()))["status"] == "HOLD"
        else:
            assert restarted.data["tasks"] == before["tasks"]
    finally:
        reopened.close()


@pytest.mark.parametrize("phase", ["idle", "active"])
def test_legacy_single_command_checkpoint_preserves_records_without_motion(
    operational, dsn, phase
):
    control, _ = operational
    key = submit(control, "box")
    command = dispatch(control)
    if phase == "idle":
        complete(control, command)
    else:
        control.result(
            wire.build_message(
                "command_status",
                command["target"],
                command["source"],
                {
                    "boot_version": command["payload"]["expected_boot_version"],
                    "observed_at": command["sent_at"],
                    "sequence": 1,
                    "status": "accepted",
                    "result": {},
                },
                command_id=command["command_id"],
            )
        )
    submit(control, "queued-box")
    legacy = deepcopy(control.data)
    old = legacy.pop("commands").get(command["command_id"])
    if old:
        old.pop("route_index")
        old.pop("sent_at")
        legacy["resources"]["field"] = key
    legacy["command"] = old
    control.save(legacy)
    control.store.close()
    reopened = OperationalStore(dsn)
    try:
        restarted = FleetControl(control.config, reopened, clock=lambda: 1)
        expected = deepcopy(legacy["tasks"])
        if phase == "active":
            expected[key]["status"] = "HOLD"
            assert "owner_restarted" in restarted.data["fault"]
        else:
            assert restarted.data["fault"] is None
        assert restarted.data["tasks"] == expected
        assert restarted.data["events"] == legacy["events"] and legacy["events"]
        assert restarted.data["resources"] == legacy["resources"]
        assert restarted.data["commands"] == (
            {command["command_id"]: old} if old else {}
        )
        assert "command" not in restarted.data
        assert not restarted.data["enabled"] and restarted.next_action() is None
        with psycopg.connect(dsn) as conn:
            persisted = conn.execute(
                "SELECT data FROM control_operational_state"
            ).fetchone()[0]
        assert persisted == restarted.data
        transport = MemoryTransport()
        FleetRuntime(restarted, transport).poll(0)
        assert not any(
            message["type"] in wire.COMMANDS and message["type"] != "robot_stop_command"
            for _, message in transport.sent
        )
        assert all(
            "keep_running_command_id" not in message["payload"]
            for _, message in transport.sent
        )
        assert restarted.data["tasks"] == expected
        assert restarted.data["resources"] == legacy["resources"]
    finally:
        reopened.close()


def test_second_operational_owner_is_rejected_then_lock_is_released(dsn):
    first = OperationalStore(dsn)
    try:
        with pytest.raises(RuntimeError, match="another"):
            OperationalStore(dsn)
    finally:
        first.close()
    second = OperationalStore(dsn)
    second.close()


@pytest.mark.parametrize("boundary", ["allocate", "dispatch"])
def test_database_save_failure_prevents_new_robot_command_publish(
    operational, dsn, boundary
):
    control, _ = operational
    submit(control, "box")
    condition = (
        "true" if boundary == "allocate" else "NEW.data->'commands' <> '{}'::jsonb"
    )
    fail_write(dsn, "control_operational_state", condition=condition)
    transport = MemoryTransport()
    runtime = FleetRuntime(control, transport)
    runtime.last_poll = monotonic()
    with pytest.raises(psycopg.Error, match="matrix injected"):
        runtime.poll(0)
    assert not any(message["type"] in wire.COMMANDS for _, message in transport.sent)
    assert control.data["commands"] == {}
    with psycopg.connect(dsn) as conn:
        assert (
            conn.execute(
                "SELECT data->'commands' FROM control_operational_state"
            ).fetchone()[0]
            == {}
        )


def test_completion_db_failure_prevents_application_and_mqtt_ack(operational, dsn):
    control, _ = operational
    submit(control, "box")
    command = dispatch(control)
    event = wire.build_message(
        "command_status",
        command["target"],
        command["source"],
        {
            "boot_version": command["payload"]["expected_boot_version"],
            "observed_at": datetime.now(UTC).isoformat(),
            "sequence": 1,
            "status": "completed",
            "result": {
                "node": command["payload"]["goal"]["node"],
                "stopped": True,
                "mode": "line",
            },
        },
        command_id=command["command_id"],
    )
    packet = SimpleNamespace(
        payload=wire.encode(event),
        topic=wire.topic(event["source"], "event"),
        retained=False,
        qos=1,
    )
    transport = MemoryTransport()
    transport.incoming.put(packet)
    runtime = FleetRuntime(control, transport)
    runtime.last_poll = monotonic()
    before = deepcopy(control.data)
    fail_write(dsn, "control_operational_state")
    with pytest.raises(psycopg.Error, match="matrix injected"):
        runtime.poll(0)
    assert control.data == before
    assert not transport.acked
    assert not any(message["type"] == "event_stored" for _, message in transport.sent)


@pytest.mark.parametrize("table", ["control_service_tasks", "control_service_outbox"])
def test_real_websocket_request_save_failure_does_not_ack_or_partially_commit(
    dsn, table
):
    fail_write(dsn, table, operation="INSERT")

    async def scenario():
        received = []
        stop = asyncio.Event()

        async def peer(socket):
            await socket.send(json.dumps(incoming()))
            try:
                received.append(json.loads(await asyncio.wait_for(socket.recv(), 0.3)))
            except TimeoutError:
                pass

        async with serve(peer, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            link = ServiceLink(
                f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                "test-token",
                "test",
                ServiceTaskStore(dsn),
            )
            with pytest.raises(psycopg.Error, match="matrix injected"):
                await asyncio.wait_for(link.run(stop), 3)
        assert received == []
        assert set(counts(dsn).values()) == {0}

    asyncio.run(scenario())


@pytest.mark.parametrize("lost", ["request_ack", "task_ack"])
def test_service_peer_restart_and_ack_loss_replay_original_committed_event(dsn, lost):
    async def scenario():
        store, stop = ServiceTaskStore(dsn), asyncio.Event()
        connections, ids, errors = [], [], []

        async def peer(socket):
            connections.append(len(connections) + 1)
            number = connections[-1]
            try:
                await socket.send(json.dumps(incoming()))
                while True:
                    event = json.loads(await socket.recv())
                    if event["type"] == "message.ack":
                        assert len(ServiceTaskStore(dsn).pending_tasks()) == 1
                        if lost == "request_ack" and number == 1:
                            await socket.close()
                            return
                    if event["type"] == "task.accepted":
                        ids.append(event["message_id"])
                        if lost == "task_ack" and number == 1:
                            await socket.close()
                            return
                        await socket.send(
                            json.dumps(
                                {
                                    "site_id": "test",
                                    "type": "message.ack",
                                    "payload": {"message_id": event["message_id"]},
                                }
                            )
                        )
                        for _ in range(100):
                            if not store.outbox():
                                stop.set()
                                return
                            await asyncio.sleep(0.01)
            except Exception as exc:  # noqa: BLE001 - propagate peer assertion failures
                errors.append(exc)
                stop.set()

        async with serve(peer, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            link = ServiceLink(
                f"ws://127.0.0.1:{port}/ws/v1/sites/test/control",
                "test-token",
                "test",
                store,
            )
            await asyncio.wait_for(link.run(stop), 8)
        assert not errors, errors
        assert len(connections) == 2 and len(set(ids)) == 1
        assert len(store.pending_tasks()) == 1 and not store.outbox()
        assert set(counts(dsn).values()) == {1}

    asyncio.run(scenario())


def test_unrelated_robot_restart_does_not_corrupt_active_task(operational):
    control, _ = operational
    key = submit(control, "box")
    command = dispatch(control)
    assert command["target"] == "pinky1"
    before = deepcopy(control.data)
    state = deepcopy(control.observed["pinky2"])
    state.update(boot_version="unrelated-new-boot", sequence=1)
    control.observe({"source": "pinky2", "payload": state})
    assert control.data["tasks"][key] == before["tasks"][key]
    assert control.data["commands"] == before["commands"]
    assert control.data["resources"] == before["resources"]
    assert control.data["enabled"] and control.data["fault"] is None
