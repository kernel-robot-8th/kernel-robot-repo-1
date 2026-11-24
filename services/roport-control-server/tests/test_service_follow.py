"""Durable service inbox and Fleet session replay share one request identity."""

from copy import deepcopy
from datetime import UTC, datetime
from uuid import uuid4

import pytest
from test_fleet import finish
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_fleet_service import operational_fleet as operational_fleet  # noqa: PLC0414
from test_fleet_service import queued, service_tick
from test_pinky_trial import profile  # noqa: F401
from test_service_link import dsn as dsn  # noqa: PLC0414
from test_service_link import example

from roport_control_server.adapters.db.service_tasks import ServiceTaskStore


@pytest.fixture
def receipt(operational_fleet, dsn):
    fleet = operational_fleet
    fleet.config["follow_modes"] = {"pinky3": ["marker", "appearance"]}
    store = ServiceTaskStore(dsn)
    data = deepcopy(fleet.data)
    data["slots"]["1a"] = {"state": "occupied", "package_id": "box"}
    fleet.save(data)
    task = queued(store, packages=("box",), operation="customer_service")
    service_tick(fleet, store)
    finish(fleet)
    msg = {
        **example("receipt"),
        "message_id": "receipt-message",
        "task_id": task["task_id"],
        "payload": {
            "receipt_id": "receipt",
            "reservation_id": task["reservation_id"],
            "order_id": task["order_id"],
            "package_ids": ["box"],
            "follow_mode": "appearance",
        },
    }
    assert store.receive(msg) is None
    service_tick(fleet, store)
    return fleet, store, task


def action(fleet, task):
    child = fleet.data["tasks"][task["task_id"] + ":box"]
    return {
        **example("receipt"),
        "message_id": str(uuid4()),
        "type": "follow.control_requested",
        "sent_at": datetime.now(UTC).isoformat(),
        "task_id": task["task_id"],
        "payload": {
            "receipt_id": "receipt",
            "reservation_id": task["reservation_id"],
            "order_id": task["order_id"],
            "robot_id": child["robot"],
            "request_id": str(uuid4()),
            "expected_revision": child["follow"]["revision"],
            "operation": "capture",
        },
    }


def test_receipt_intent_projection_and_crash_window_replay(receipt):
    fleet, store, task = receipt
    child_id = task["task_id"] + ":box"
    assert fleet.data["tasks"][child_id]["stage"] == "follow_register"
    projected = store.pending_tasks()[0]["progress"]["packages"][0]
    assert projected["follow"]["mode"] == "appearance"
    assert "capture" in projected["follow"]["allowed_actions"]
    assert fleet.next_action() is None
    message = action(fleet, task)
    assert store.receive(message) is None
    fleet.follow_request(message["payload"], sent_at=message["sent_at"])
    # Crash between the snapshot commit and queue acknowledgement: replay is inert.
    before = deepcopy(fleet.data)
    service_tick(fleet, store)
    assert fleet.data == before
    assert store.follow_requests() == []
    assert fleet.data["tasks"][child_id]["stage"] == "follow_registering"
    assert store.receive(message) is None
    service_tick(fleet, store)
    assert fleet.data == before
    changed = deepcopy(message)
    changed["message_id"] = str(uuid4())
    changed["payload"]["operation"] = "return"
    assert store.receive(changed) is not None
    assert fleet.data == before


@pytest.mark.parametrize(
    "field,value",
    [
        ("robot_id", "pinky1"),
        ("receipt_id", "someone-else"),
        ("order_id", "another-order"),
    ],
)
def test_cross_receipt_requests_are_quarantined(receipt, field, value):
    fleet, store, task = receipt
    message = action(fleet, task)
    message["payload"][field] = value
    assert store.receive(message) is not None
    assert store.follow_requests() == []
    assert fleet.next_action() is None
