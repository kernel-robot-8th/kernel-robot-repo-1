"""Business requests and PR45 events must never create a second physical action."""

# ruff: noqa: F811
import time
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

import pytest
from test_pinky_trial import control_memory, profile, single_profile  # noqa: F401

from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import encode, topic
from roport_control_server.executor.omx_trial import OmxTrial
from roport_control_server.executor.pinky_trial import TrialControl
from roport_control_server.executor.trial_socket import TrialSocket


def intent(c, operation="outbound"):
    return {
        "request_id": str(uuid4()),
        "operation": operation,
        "package_id": c.config["package_id"],
        "quantity": 1,
        "destination": "shelf" if operation == "inbound" else "handover",
        "source_confirmed": True,
    }


def test_service_roles_and_server_only_plan(control_memory):
    c = control_memory
    inbound = c.preview_service(intent(c, "inbound"))
    assert [s["id"] for s in inbound["plan"]["steps"]] == [
        "to_loading",
        "WAIT_LOAD",
        "to_delivery",
        "WAIT_INBOUND",
        "return_warehouse",
    ]
    assert {s["role"] for s in inbound["plan"]["steps"]} == {"logistics"}
    c.configure(inbound["plan"])
    # A prior logistics-only plan must not hide the smartcart from service planning.
    outbound = c.preview_service(intent(c))
    assert {s["role"] for s in outbound["plan"]["steps"]} == {"smartcart"}
    assert (
        next(s for s in outbound["plan"]["steps"] if s["kind"] == "gate")["payload"][
            "omx_operation"
        ]
        == "OUTBOUND"
    )
    assert c.transport.sent == []
    assert "route" not in outbound["request"]


@pytest.mark.parametrize(
    "change",
    [
        {"quantity": True},
        {"quantity": 2},
        {"source_confirmed": False},
        {"destination": "N12"},
        {"pickup": "N11"},
        {"operation": "cancel"},
    ],
)
def test_bad_service_intent_never_changes_plan(control_memory, change):
    c = control_memory
    before = deepcopy(c.state)
    with pytest.raises(ValueError):
        c.preview_service({**intent(c), **change})
    assert c.state == before and c.transport.sent == []


def test_changed_state_and_changed_request_are_fenced(control_memory):
    c = control_memory
    request = intent(c)
    preview = c.preview_service(request)
    state = deepcopy(c.state)
    state["statuses"]["pinky3"]["node_id"] = "N11"
    c.journal.save(state)
    with pytest.raises(ValueError, match="変更|변경"):
        c.submit_service(request, preview["preview_id"], False)
    assert c.state["phase"] == "READY" and c.transport.sent == []


def test_service_replay_survives_session_and_restart(control_memory, tmp_path):
    c = control_memory
    request = intent(c)
    preview = c.preview_service(request)
    socket = TrialSocket(tmp_path / "s", c)
    body = {
        "action": "service_submit",
        "session_id": socket.session_id,
        "task_id": c.state["task_id"],
        "phase": "READY",
        "expires_at": time.time() + 1,
        "request": request,
        "preview_id": preview["preview_id"],
        "field_confirmed": False,
    }
    try:
        first = socket.dispatch(body)["service_receipt"]
        assert socket.dispatch(body)["service_receipt"] == first
        assert c.transport.sent == []
        with pytest.raises(ValueError, match="conflicting"):
            c.service_receipt({**request, "quantity": 2})
        restored = TrialControl(c.config, c.transport, c.journal)
        assert restored.service_receipt(request) == first
        with pytest.raises(ValueError, match="already applied"):
            restored.configure(preview["plan"])
    finally:
        socket.close()


def test_status_discovery_does_not_enable_motion(control_memory):
    c = control_memory
    c.configure(c.preview_service(intent(c, "inbound"))["plan"])
    c.heard.pop("pinky3")
    assert c.preview_service(intent(c)) == {"pending": True}
    c.poll(0)
    assert any(cmd["target"] == "pinky3" for cmd in c.pending_status.values())
    assert all(cmd["type"] == "status.requested" for _, cmd in c.transport.sent)


@pytest.fixture
def omx_pair(control_memory, monkeypatch, tmp_path):
    c = control_memory
    omx = Path(__file__).resolve().parents[2] / "roport-omx"
    monkeypatch.syspath_prepend(str(omx / "src"))
    monkeypatch.syspath_prepend(str(omx / "scripts"))
    from cooperative_mock import MockExecutor
    from roport_omx.cooperative import CooperativeHost

    c.config["omx"] = {
        "binding_id": "test-omx",
        "slot_id": "1a",
        "model_version": "sim-model",
        "calibration_version": "sim-cal",
        "execution_mode": "mock",
    }
    state = deepcopy(c.state)
    state["config"] = deepcopy(c.config)
    state["statuses"]["pinky1"]["node_id"] = "N11"
    state["statuses"]["pinky1"]["heading_rad"] = c.config["routes"]["return_warehouse"][
        "start_heading_rad"
    ]
    c.journal.save(state)
    c.omx = OmxTrial(c)
    gate = {
        "id": "transfer",
        "kind": "gate",
        "role": "logistics",
        "label": "입고",
        "payload": {
            "node_id": "N11",
            "package_id": c.config["package_id"],
            "quantity": 1,
            "omx_operation": "INBOUND",
        },
    }
    route = {
        "id": "return",
        "kind": "route",
        "role": "logistics",
        "label": "복귀",
        "payload": c.config["routes"]["return_warehouse"],
    }
    c.configure(
        {"version": 1, "name": "OMX 시험", "assumptions": [], "steps": [gate, route]}
    )
    c.start()
    journal = Journal(tmp_path / "host.json")
    executor = MockExecutor(0)
    host = CooperativeHost(
        journal,
        executor,
        site=c.config["site_id"],
        control=c.config["control_id"],
        binding="test-omx",
    )
    yield c, host, executor
    journal.close()


def deliver(c, event, **kw):
    message = SimpleNamespace(
        topic=topic(c.config["site_id"], "workcell", "shelf-01", "event"),
        payload=encode(event),
        qos=1,
        retained=False,
    )
    for k, v in kw.items():
        setattr(message, k, v)
    c.handle(message)


def dispatch_omx(c, host):
    c.omx.poll()
    status = c.transport.sent[-1][1]
    for e in host.handle(status, channel="cmd"):
        deliver(c, e)
    c.omx.poll()
    command = c.transport.sent[-1][1]
    for e in host.handle(command, channel="cmd"):
        deliver(c, e)
    return command


def test_mqtt_complete_advances_once_and_keeps_evidence(omx_pair):
    c, host, executor = omx_pair
    token = c.state["gate"]["stop_session_id"]
    command = dispatch_omx(c, host)
    with pytest.raises(ValueError, match="전송됨"):
        c.dummy(token)
    event = host.poll()[0]
    deliver(c, event)
    deliver(c, event)
    assert c.state["phase"] == "return"
    assert sum(cmd["type"] == "route.requested" for _, cmd in c.transport.sent) == 1
    assert len(c.state["transfers"]) == len(executor.tasks) == 1
    transfer = c.state["transfers"][0]
    assert (
        transfer["completion_source"] == "mqtt" and transfer["execution_mode"] == "mock"
    )
    assert (
        transfer["placement_confirmed"] is None
        and transfer["evidence"] == "arm_return_only"
    )
    assert command["payload"]["dock_session_id"] == token
    with pytest.raises(ValueError):
        c.dummy(token)


def test_manual_first_prevents_late_status_from_starting_omx(omx_pair):
    c, host, executor = omx_pair
    c.omx.poll()
    request = c.transport.sent[-1][1]
    c.dummy(c.state["gate"]["stop_session_id"])
    for e in host.handle(request, channel="cmd"):
        deliver(c, e)
    c.omx.poll()
    assert not executor.tasks
    assert c.state["transfers"][0]["completion_source"] == "ui"
    assert sum(cmd["type"] == "route.requested" for _, cmd in c.transport.sent) == 1


@pytest.mark.parametrize(
    "fault", ["boot", "binding", "session", "qos", "retained", "stale", "arms", "ids"]
)
def test_invalid_omx_completion_never_departs(omx_pair, fault):
    c, host, _ = omx_pair
    dispatch_omx(c, host)
    event = host.poll()[0]
    kw = {}
    if fault in {"boot", "binding", "session"}:
        key = {
            "boot": "boot_id",
            "binding": "binding_id",
            "session": "dock_session_id",
        }[fault]
        event["payload"][key] = "unrelated"
    elif fault in {"qos", "retained"}:
        kw[fault] = 0 if fault == "qos" else True
    elif fault == "stale":
        event["payload"]["observed_at"] = (
            datetime.now(UTC) - timedelta(seconds=15)
        ).isoformat()
    elif fault == "arms":
        event["payload"]["arms"]["omx2"]["safe"] = False
    else:
        event["execution_id"] = "wrong"
    try:
        deliver(c, event, **kw)
    except ValueError:
        pass
    assert not c.state["transfers"]
    assert not any(cmd["type"] == "route.requested" for _, cmd in c.transport.sent)


def test_unsaved_omx_event_is_never_acked(omx_pair, monkeypatch):
    c, host, _ = omx_pair
    dispatch_omx(c, host)
    event = host.poll()[0]
    before = len(c.transport.sent)

    def fail(_):
        raise ValueError("journal full")

    monkeypatch.setattr(c.journal, "save", fail)
    with pytest.raises(OSError):
        deliver(c, event)
    assert len(c.transport.sent) == before
    assert not c.state["transfers"]


def test_restart_after_dispatch_holds_without_resend(omx_pair):
    c, host, _ = omx_pair
    dispatch_omx(c, host)
    restored = TrialControl(c.config, c.transport, c.journal)
    assert restored.state["phase"] == "HOLD"
    before = len(c.transport.sent)
    restored.omx.poll()
    assert len(c.transport.sent) == before


@pytest.mark.parametrize("delay", ["immediate", "stale", "next_task"])
def test_ui_after_safe_status_wins_over_delayed_completion(omx_pair, delay):
    c, host, _ = omx_pair
    dispatch_omx(c, host)
    event = host.poll()[0]  # Physical completion happened, but its event is delayed.
    time.sleep(0.002)  # Host uses millisecond observation timestamps.
    c.omx.next_poll = 0
    c.omx.poll()
    for status in host.handle(c.transport.sent[-1][1], channel="cmd"):
        deliver(c, status)
    assert c.omx.manual_error() is None
    c.dummy(c.state["gate"]["stop_session_id"])
    if delay == "stale":
        event["payload"]["observed_at"] = (
            datetime.now(UTC) - timedelta(seconds=15)
        ).isoformat()
    if delay == "next_task":
        state = deepcopy(c.state)
        state.update(phase="COMPLETE", active=None, gate=None)
        c.journal.save(state)
        c.configure(c.plan)
    deliver(c, event)
    if delay == "next_task":
        assert c.state["phase"] == "READY" and not c.state["transfers"]
    else:
        assert c.state["phase"] == "return"
        assert len(c.state["transfers"]) == 1
        assert c.state["transfers"][0]["completion_source"] == "ui"
    assert sum(cmd["type"] == "route.requested" for _, cmd in c.transport.sent) == 1


def test_host_restart_status_prevents_old_completion_departure(omx_pair):
    c, host, _ = omx_pair
    dispatch_omx(c, host)
    completed = host.poll()[0]
    c.omx.next_poll = 0
    c.omx.poll()
    status = host.handle(c.transport.sent[-1][1], channel="cmd")[0]
    status["payload"]["boot_id"] = "new-boot"
    deliver(c, status)
    assert c.state["fault"] == "omx_restarted"
    deliver(c, completed)
    assert not c.state["transfers"]
    assert not any(cmd["type"] == "route.requested" for _, cmd in c.transport.sent)


def test_reordered_final_cannot_override_newer_progress(omx_pair):
    c, host, _ = omx_pair
    dispatch_omx(c, host)
    completed = host.poll()[0]
    progress = deepcopy(completed)
    progress.update(message_id=str(uuid4()), type="shelf_inbound.progress")
    progress["payload"]["phase_revision"] += 1
    deliver(c, progress)
    deliver(c, completed)
    assert not c.state["transfers"]
    assert not any(cmd["type"] == "route.requested" for _, cmd in c.transport.sent)
