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
