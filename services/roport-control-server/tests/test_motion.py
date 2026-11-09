"""Display telemetry cannot complete a task or replace its boot/binding fences."""

import json
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from types import SimpleNamespace
from uuid import uuid4

import pytest
from test_pinky_trial import MemoryTransport, single_profile  # noqa: F401

from roport_control_server.adapters.mqtt.motion import (
    MotionReceiver,
    decode_motion,
    encode_motion,
    motion_message,
    motion_topic,
)


def sample(profile, boot, *, sequence=1, **changes):
    robot = profile["robots"]["logistics"]
    observation = {
        "observed_at": datetime.now(UTC).isoformat(),
        "map_id": profile["map_id"],
        "map_version": profile["map_version"],
        "frame_id": "map",
        "pose_source": "tf",
        "pose": {"x": 0.1, "y": 0.2, "z": 0, "yaw": 0.3},
        "velocity": None,
        "joints": {},
    }
    observation.update(changes)
    return motion_message(
        profile["site_id"],
        robot["robot_id"],
        robot["binding_id"],
        boot,
        sequence,
        observation,
    )


def delivery(message, **changes):
    data = {
        "topic": motion_topic(message["site_id"], message["source"]),
        "payload": json.dumps(message).encode(),
        "qos": 0,
        "retained": False,
    }
    data.update(changes)
    return SimpleNamespace(**data)


@pytest.fixture
def receiver(request):
    receiver = MotionReceiver(
        request.getfixturevalue("single_profile"), MemoryTransport()
    )
    statuses = {"pinky1": {"boot_id": str(uuid4())}}
    return receiver, statuses


def test_wire_matches_existing_db_motion_without_importing_db(receiver):
    receiver, statuses = receiver
    m = sample(receiver.config, statuses["pinky1"]["boot_id"])
    destination = motion_topic(m["site_id"], "pinky1")
    assert decode_motion(delivery(m)) == m
    # The DB adapter is held in a separate PR; verify compatibility when present.
    import importlib.util

    if importlib.util.find_spec("roport_control_server.adapters.mqtt.state"):
        from roport_control_server.adapters.mqtt.state import encode_state

        assert json.loads(encode_motion(destination, m)) == json.loads(
            encode_state(destination, m)
        )
    receiver.consume(delivery(m), statuses)
    assert receiver.snapshot(statuses)["pinky1"]["pose"]["x"] == 0.1


@pytest.mark.parametrize(
    "change",
    [
        {"binding_id": str(uuid4())},
        {"boot_id": str(uuid4())},
        {"observed_at": (datetime.now(UTC) - timedelta(seconds=10)).isoformat()},
        {"observed_at": (datetime.now(UTC) + timedelta(seconds=10)).isoformat()},
    ],
)
def test_wrong_identity_and_expired_observation_cannot_replace_pose(receiver, change):
    receiver, statuses = receiver
    m = sample(receiver.config, statuses["pinky1"]["boot_id"])
    receiver.consume(delivery(m), statuses)
    invalid = deepcopy(m)
    invalid["payload"].update(sequence=2, **change)
    with pytest.raises(ValueError):
        receiver.consume(delivery(invalid), statuses)
    assert receiver.snapshot(statuses)["pinky1"]["sequence"] == 1


@pytest.mark.parametrize(
    "change", [{"map_version": "wrong"}, {"frame_id": "odom"}, {"pose": None}]
)
def test_invalid_localization_holds_last_pose_with_original_age(receiver, change):
    receiver, statuses = receiver
    boot = statuses["pinky1"]["boot_id"]
    receiver.consume(delivery(sample(receiver.config, boot)), statuses)
    previous = receiver.samples["pinky1"]["received"]
    receiver.consume(
        delivery(sample(receiver.config, boot, sequence=2, **change)), statuses
    )
    row = receiver.snapshot(statuses)["pinky1"]
    assert row["stale"] and row["pose"]["x"] == 0.1
    assert receiver.samples["pinky1"]["received"] == previous
    receiver.consume(delivery(sample(receiver.config, boot, sequence=3)), statuses)
    assert not receiver.snapshot(statuses)["pinky1"]["stale"]


def test_sequence_boot_disconnect_and_age_fences(receiver):
    receiver, statuses = receiver
    boot = statuses["pinky1"]["boot_id"]
    m = sample(receiver.config, boot)
    receiver.consume(delivery(m), statuses)
    with pytest.raises(ValueError, match="out-of-order"):
        receiver.consume(delivery(m), statuses)
    receiver.samples["pinky1"]["received"] -= 2
    assert receiver.snapshot(statuses)["pinky1"]["stale"]
    receiver.consume(delivery(sample(receiver.config, boot, sequence=2)), statuses)
    receiver.transport.ready = False
    assert receiver.snapshot(statuses)["pinky1"]["stale"]
    statuses["pinky1"]["boot_id"] = str(uuid4())
    assert receiver.snapshot(statuses) == {}
    with pytest.raises(ValueError, match="identity"):
        receiver.consume(delivery(m), statuses)


@pytest.mark.parametrize(
    "changes",
    [{"qos": 1}, {"retained": True}, {"payload": b"{"}, {"payload": b"x" * 16385}],
)
def test_wrong_delivery_policy_or_bad_json(receiver, changes):
    receiver, statuses = receiver
    m = sample(receiver.config, statuses["pinky1"]["boot_id"])
    with pytest.raises(ValueError):
        decode_motion(delivery(m, **changes))


def test_motion_is_not_a_trial_event(receiver):
    from roport_control_server.adapters.mqtt.trial_wire import validate_trial

    receiver, statuses = receiver
    m = sample(receiver.config, statuses["pinky1"]["boot_id"])
    with pytest.raises(ValueError):
        validate_trial(m)


def test_nonfinite_and_duplicate_json_keys_are_rejected(receiver):
    receiver, statuses = receiver
    m = sample(receiver.config, statuses["pinky1"]["boot_id"])
    m["payload"]["pose"]["x"] = float("nan")
    with pytest.raises(ValueError):
        decode_motion(delivery(m))
    m = sample(receiver.config, statuses["pinky1"]["boot_id"])
    raw = json.dumps(m).replace('"x": 0.1', '"x": 0.1, "x": 0.2')
    with pytest.raises(ValueError):
        decode_motion(delivery(m, payload=raw.encode()))


def test_publisher_is_optional_rate_limited_and_never_breaks_task_client(
    tmp_path, request
):
    from mock_pinky_trial import FakePinky

    from roport_control_server.adapters.mqtt.mqtt import MqttConfig
    from roport_control_server.adapters.mqtt.pinky_trial import PinkyTrialClient

    profile = request.getfixturevalue("single_profile")
    fake = FakePinky(profile, "logistics")
    robot = profile["robots"]["logistics"]
    client = PinkyTrialClient(
        MqttConfig("127.0.0.1", "motion-test"),
        site=profile["site_id"],
        control=profile["control_id"],
        robot="pinky1",
        binding_id=robot["binding_id"],
        journal_path=tmp_path / "client.json",
        observe=fake.observe,
        validate_action=fake.validate_action,
        dispatch=fake.dispatch,
        safe_stop=fake.stop,
    )
    try:
        client.publish_motion()
        assert client.motion_sequence == 0
        observation = sample(profile, client.boot)["payload"]
        client.observe_motion = lambda: observation
        before = deepcopy(client.journal.data)
        client.publish_motion()  # Offline display transport is optional.
        assert client.motion_sequence == 1 and client.motion_errors == 1
        client.publish_motion()
        assert client.motion_sequence == 1
        assert client.journal.data == before and client.halted is None
    finally:
        client.close()
