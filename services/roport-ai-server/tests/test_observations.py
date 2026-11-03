"""RPV1 source/loopback checks with synthetic bytes; no camera or GPU."""

import math
import socket
import zlib
from dataclasses import asdict, replace
from uuid import UUID

import pytest

from observations import ObservationImageResolver, validate_images
from udp_frame_protocol import CHUNK_BYTES, FrameInfo, pack_header

NOW_NS = 1_793_664_000_000_000_000
SESSION = UUID("00000000-0000-0000-0000-000000000001")
PEER = ("127.0.0.1", 12345)
INFO = FrameInfo("host1", "side", "rgb", SESSION, 1, NOW_NS, "group1")


def packets(info=INFO, data=b"synthetic-jpeg" * 300):
    count = math.ceil(len(data) / CHUNK_BYTES)
    return [
        pack_header(info, i, count, len(data), zlib.crc32(data))
        + data[i * CHUNK_BYTES : (i + 1) * CHUNK_BYTES]
        for i in range(count)
    ]


def observation(info=INFO):
    entry = asdict(info)
    entry.pop("codec")
    entry["session"] = str(info.session)
    return {"captured_at": "2026-11-03T00:00:00+00:00", "images": {"side": entry}}


def resolver(**overrides):
    settings = {
        "timeout": 0.1,
        "max_inflight": 2,
        "max_bytes": 100_000,
        "max_frames": 2,
        "max_age_ns": 1_000_000_000,
        "max_timestamp_skew_ns": 1_000_000,
        "clock_ns": lambda: NOW_NS,
    }
    return ObservationImageResolver(
        {INFO.key: (PEER, SESSION)}, **(settings | overrides)
    )


def feed(target, info=INFO, data=b"synthetic-jpeg" * 300, peer=PEER):
    return [target.feed(packet, peer) for packet in packets(info, data)]


def test_reversed_fragments_resolve_only_exact_manifest():
    target = resolver()
    for packet in reversed(packets()):
        target.feed(packet, PEER)
    assert target(observation()) == {"side": b"synthetic-jpeg" * 300}
    for field, value in {
        "frame_id": 2,
        "source": "other",
        "stream": "other",
        "session": str(UUID(int=2)),
        "captured_ns": NOW_NS + 1,
        "observation_group": "group2",
    }.items():
        changed = observation()
        changed["images"]["side"][field] = value
        with pytest.raises(ValueError):
            target(changed)


def test_loss_checksum_timeout_and_untrusted_peer_never_resolve(monkeypatch):
    target = resolver()
    for packet in packets()[:-1]:
        target.feed(packet, PEER)
    with pytest.raises(ValueError, match="unavailable"):
        target(observation())
    target.receiver.expire(float("inf"))
    assert target.receiver.reserved_bytes == 0
    assert target.feed(packets()[-1], PEER) is False
    with pytest.raises(ValueError):
        target(observation())

    corrupt = resolver()
    parts = packets()
    parts[-1] = parts[-1][:-1] + bytes([parts[-1][-1] ^ 1])
    for part in parts:
        corrupt.feed(part, PEER)
    assert corrupt.receiver.stats["checksum"] == 1
    for peer in (("127.0.0.2", PEER[1]), ("127.0.0.1", PEER[1] + 1)):
        assert not any(feed(corrupt, peer=peer))
    assert corrupt.receiver.stats["untrusted_peer"] > 0
    assert not any(feed(corrupt, replace(INFO, session=UUID(int=2))))
    assert corrupt.receiver.stats["unknown_stream_session"] > 0


def test_missing_and_mixed_cameras_and_bad_headers_rejected():
    target = resolver()
    feed(target)
    bad = observation()
    bad["images"] = {}
    with pytest.raises(ValueError):
        target(bad)
    bad = observation()
    bad["images"]["side"]["raw"] = "not allowed"
    with pytest.raises(ValueError):
        target(bad)
    bad = observation()
    bad["images"]["side"]["frame_id"] = True
    with pytest.raises(ValueError):
        target(bad)
    other = observation(replace(INFO, camera="top", observation_group="group2"))
    bad = observation()
    bad["images"]["top"] = other["images"]["side"]
    with pytest.raises(ValueError):
        validate_images(bad["images"])


def test_expiry_skew_and_capacity_do_not_fall_back():
    clock = [NOW_NS]
    target = resolver(max_frames=1, clock_ns=lambda: clock[0])
    assert any(feed(target))
    assert any(feed(target, replace(INFO, frame_id=2)))
    with pytest.raises(ValueError, match="unavailable"):
        target(observation())
    assert target(observation(replace(INFO, frame_id=2)))
    clock[0] += 1_000_000_001
    with pytest.raises(ValueError):
        target(observation(replace(INFO, frame_id=2)))
    assert target._bytes == 0
    assert not any(feed(resolver(), replace(INFO, captured_ns=NOW_NS - 1_000_000_001)))
    assert not any(feed(resolver(), replace(INFO, captured_ns=NOW_NS + 1_000_001)))
    target = resolver()
    skewed = replace(INFO, captured_ns=NOW_NS - 1_000_001)
    assert any(feed(target, skewed))
    with pytest.raises(ValueError, match="skew"):
        target(observation(skewed))


@pytest.mark.parametrize(
    "setting",
    [
        "max_frames",
        "max_bytes",
        "max_inflight",
        "max_age_ns",
        "max_timestamp_skew_ns",
        "timeout",
    ],
)
def test_limits_are_explicit_and_positive(setting):
    with pytest.raises(ValueError):
        resolver(**{setting: 0})
    with pytest.raises(ValueError):
        resolver(**{setting: True})


def test_loopback_manifest_binding_without_stale_image_reuse():
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
    ):
        sender.bind(("127.0.0.1", 0))
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        target = ObservationImageResolver(
            {INFO.key: (sender.getsockname(), SESSION)},
            timeout=0.1,
            max_inflight=1,
            max_bytes=100_000,
            max_frames=1,
            max_age_ns=1_000_000_000,
            max_timestamp_skew_ns=1_000_000,
            clock_ns=lambda: NOW_NS,
        )
        for packet in reversed(packets()):
            sender.sendto(packet, receiver.getsockname())
            data, peer = receiver.recvfrom(1201)
            target.feed(data, peer)
        assert target(observation())["side"] == b"synthetic-jpeg" * 300
        missing = replace(INFO, frame_id=2)
        with pytest.raises(ValueError, match="unavailable"):
            target(observation(missing))
