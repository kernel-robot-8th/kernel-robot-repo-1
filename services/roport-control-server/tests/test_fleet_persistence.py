"""Fleet commit boundaries against disposable PostgreSQL schemas."""

from copy import deepcopy
from datetime import UTC, datetime
from time import monotonic
from types import SimpleNamespace

import psycopg
import pytest
from test_fleet import dispatch, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_fleet_service import operational_fleet as operational_fleet  # noqa: PLC0414
from test_pinky_trial import MemoryTransport
from test_pinky_trial import profile as profile  # noqa: PLC0414
from test_service_link import dsn as dsn  # noqa: PLC0414

from roport_control_server.adapters.db.service_tasks import OperationalStore
from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.fleet_runtime import FleetRuntime


def fail_save(dsn, condition="true"):
    # This fixture owns a temporary UUID schema, never a shared or field database.
    with psycopg.connect(dsn) as conn:
        conn.execute(
            f"CREATE FUNCTION reject_save() RETURNS trigger LANGUAGE plpgsql AS $$ BEGIN IF {condition} THEN RAISE EXCEPTION 'matrix injected storage failure'; END IF; RETURN NEW; END $$"
        )
        conn.execute(
            "CREATE TRIGGER reject_save BEFORE UPDATE ON control_operational_state FOR EACH ROW EXECUTE FUNCTION reject_save()"
        )


@pytest.mark.parametrize("active", [False, True])
def test_database_restart_retains_work_but_requires_operator_start(
    operational_fleet, dsn, active
):
    control = operational_fleet
    submit(control, "box")
    if active:
        dispatch(control)
    before = deepcopy(control.data)
    control.store.close()
    reopened = OperationalStore(dsn)
    try:
        restarted = FleetControl(control.config, reopened, clock=lambda: 1)
        assert not restarted.data["enabled"] and restarted.next_action() is None
        for key in ("roles", "slots", "commands", "resources"):
            assert restarted.data[key] == before[key]
        if active:
            assert next(iter(restarted.data["tasks"].values()))["status"] == "HOLD"
        else:
            assert restarted.data["tasks"] == before["tasks"]
    finally:
        reopened.close()


@pytest.mark.parametrize("boundary", ["allocate", "dispatch"])
def test_database_save_failure_prevents_new_robot_command_publish(
    operational_fleet, dsn, boundary
):
    control = operational_fleet
    submit(control, "box")
    condition = (
        "true" if boundary == "allocate" else "NEW.data->'commands' <> '{}'::jsonb"
    )
    fail_save(dsn, condition)
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


def test_completion_db_failure_prevents_application_and_mqtt_ack(
    operational_fleet, dsn
):
    control = operational_fleet
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
    fail_save(dsn)
    with pytest.raises(psycopg.Error, match="matrix injected"):
        runtime.poll(0)
    assert control.data == before
    assert not transport.acked
    assert not any(message["type"] == "event_stored" for _, message in transport.sent)
