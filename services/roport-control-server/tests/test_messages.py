import copy
import json
from datetime import UTC, datetime, timedelta

import pytest

from roport_control_server.communication.messages import (
    MAX_MESSAGE_BYTES,
    CommandHistory,
    decode,
    encode,
    topic,
)
from roport_control_server.examples.mqtt_demo import MockExecutor, make_command


@pytest.mark.parametrize(
    "raw",
    [
        b"null",
        b"[]",
        b"{",
        b"\xff",
        b'{"a":1,"a":2}',
        b"{" + b'"a":' + b"[" * 2000,
        b" " * (MAX_MESSAGE_BYTES + 1),
    ],
)
def test_bad_wire_envelopes(raw):
    with pytest.raises(ValueError):
        decode(raw)


@pytest.mark.parametrize(
    "key,value",
    [
        ("schema_version", "2.0"),
        ("command_id", ""),
        ("target", "pinky1/#"),
        ("payload", []),
        ("sent_at", "2026-10-30T10:00:00"),
        ("expires_at", None),
    ],
)
def test_bad_headers(key, value):
    command = make_command("pinky1")
    command[key] = value
    with pytest.raises(ValueError):
        decode(json.dumps(command).encode())


@pytest.mark.parametrize("number", ["NaN", "Infinity", "1e400"])
def test_nonfinite_payload(number):
    command = make_command("pinky1")
    raw = json.dumps(command).replace('"test-v1"', number).encode()
    with pytest.raises(ValueError):
        decode(raw)


def test_duplicate_keeps_original_events_and_conflicts_do_not_execute():
    executor = MockExecutor("pinky1")
    command = make_command("pinky1")
    first = executor.handle(decode(encode(command)))
    assert executor.handle(command) == first
    changed = copy.deepcopy(command)
    changed["execution_id"] = "other-attempt"
    assert (
        executor.handle(changed)[0]["payload"]["error_code"] == "idempotency_conflict"
    )
    assert executor.execution_count == 1


def test_full_history_preserves_dedupe():
    history = CommandHistory(1)
    command = make_command("pinky1")
    history.remember(command, [{"result": "original"}])
    with pytest.raises(ValueError, match="command_history_full"):
        history.lookup(make_command("pinky1"))
    assert history.lookup(command) == [{"result": "original"}]


def test_expired_retained_and_wrong_destination_never_execute():
    executor = MockExecutor("pinky1")
    command = make_command("pinky1")
    assert (
        executor.handle(command, retained=True)[0]["payload"]["error_code"]
        == "retained_command"
    )
    command["sent_at"] = (datetime.now(UTC) - timedelta(seconds=60)).isoformat()
    command["expires_at"] = (datetime.now(UTC) - timedelta(seconds=30)).isoformat()
    assert executor.handle(command)[0]["payload"]["error_code"] == "expired_command"
    command["target"] = "pinky2"
    with pytest.raises(ValueError, match="routing_mismatch"):
        executor.handle(command)
    assert executor.execution_count == 0


def test_one_arm_return_is_not_joint_completion():
    executor = MockExecutor("shelf-01", returned_arms={"omx1"})
    events = executor.handle(make_command("shelf-01"))
    assert [event["phase"] for event in events] == ["accepted", "progress"]
    assert events[-1]["payload"]["departure_ready"] is False
    with pytest.raises(ValueError, match="joint workcell"):
        topic("test", "robot", "omx1", "cmd")


def test_missing_expiry_on_unknown_command_is_rejected_not_typeerror():
    command = make_command("pinky1")
    command["type"] = "route.cancel"
    del command["expires_at"]
    executor = MockExecutor("pinky1")
    assert executor.handle(decode(encode(command)))[0]["phase"] == "rejected"
    assert executor.execution_count == 0
    assert executor.handle(make_command("pinky1"))[-1]["phase"] == "completed"


def test_expired_duplicate_returns_original_result(monkeypatch):
    from roport_control_server.communication import messages

    executor = MockExecutor("pinky1")
    command = make_command("pinky1")
    original = executor.handle(command)

    class Later(datetime):
        @classmethod
        def now(cls, tz=None):
            return datetime.fromisoformat(command["expires_at"]) + timedelta(seconds=1)

    monkeypatch.setattr(messages, "datetime", Later)
    assert executor.handle(command) == original
    assert executor.handle(command, retained=True)[0]["phase"] == "rejected"
    changed = {**command, "execution_id": "different"}
    assert (
        executor.handle(changed)[0]["payload"]["error_code"] == "idempotency_conflict"
    )
    assert executor.execution_count == 1
