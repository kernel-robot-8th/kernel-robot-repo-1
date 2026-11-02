from dataclasses import replace
from datetime import UTC, datetime, timedelta

import pytest

from roport_control_server.spatial_state import RobotSample, StateCache


def sample(**changes):
    return replace(
        RobotSample(
            "pinky1",
            "boot-1",
            1,
            datetime.now(UTC).isoformat(),
            "test",
            pose=(1, 2, 0, 1.5),
        ),
        **changes,
    )


def test_display_values_preserve_plain_strings_and_named_joints():
    value = sample(mode="<b>주행</b>", joints=(("l_wheel_joint", 2.0),))
    assert value.to_dict()["mode"] == "<b>주행</b>"
    assert value.to_dict()["joints"] == {"l_wheel_joint": 2.0}


@pytest.mark.parametrize(
    "changes",
    [
        {"robot_id": "unknown"},
        {"sequence": True},
        {"sequence": -1},
        {"pose": (float("nan"), 0, 0, 0)},
        {"pose": (1, 2, 3)},
        {"observed_at": "2026-01-01"},
        {"frame_id": "odom"},
        {"joints": (("joint1", 1), ("joint1", 2))},
        {"velocity": float("inf")},
    ],
)
def test_invalid_sensor_fields_rejected(changes):
    with pytest.raises(ValueError):
        sample(**changes)


def test_age_sequence_map_and_reconnect_do_not_turn_old_data_current():
    now = datetime.now(UTC)
    cache = StateCache("test")
    value = sample(observed_at=now.isoformat())
    assert not cache.apply(value, now=10, wall_time=now)
    cache.begin_connection()
    assert cache.apply(value, now=10, wall_time=now)
    assert not cache.apply(value, now=11, wall_time=now)
    assert not cache.apply(
        replace(value, map_id="other", sequence=2), now=11, wall_time=now
    )
    assert not cache.apply(replace(value, session_id="boot-2"), now=11, wall_time=now)
    assert cache.snapshot(now=11)[0]["stale"] is False
    assert cache.snapshot(now=12)[0]["stale"] is True
    cache.disconnect()
    assert cache.snapshot(now=11)[0]["stale"] is True
    cache.begin_connection()
    assert cache.snapshot(now=11)[0]["stale"] is True
    assert cache.apply(
        replace(value, session_id="boot-2", sequence=0), now=11, wall_time=now
    )
    assert not cache.apply(replace(value, sequence=5), now=11, wall_time=now)


def test_packet_age_counts_toward_freshness():
    now = datetime.now(UTC)
    cache = StateCache("test")
    cache.begin_connection()
    assert cache.apply(
        sample(observed_at=(now - timedelta(seconds=1.5)).isoformat()),
        now=10,
        wall_time=now,
    )
    assert cache.snapshot(now=10.6)[0]["stale"]
    assert not cache.apply(
        sample(sequence=2, observed_at=(now - timedelta(seconds=3)).isoformat()),
        now=11,
        wall_time=now,
    )
    assert not cache.apply(
        sample(sequence=2, observed_at=(now + timedelta(seconds=3)).isoformat()),
        now=11,
        wall_time=now,
    )
