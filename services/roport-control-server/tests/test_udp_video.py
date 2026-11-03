import hashlib
import os
import shutil
import socket
import subprocess
import sys
from dataclasses import replace
from itertools import pairwise
from pathlib import Path
from uuid import UUID, uuid4

import pytest

from roport_control_server.adapters.udp import udp_frame_protocol as proto
from roport_control_server.adapters.udp import udp_frame_receiver as receiving
from roport_control_server.adapters.udp import udp_frame_sender as sending

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
PEER = ("127.0.0.1", 5302)


@pytest.fixture
def info():
    return proto.FrameInfo("omx1", "cam1", "video", uuid4(), 1, 100, "pick-1")


def receiver(info, **limits):
    return receiving.FrameReceiver({info.key: (PEER, info.session)}, **limits)


def test_reordered_duplicate_fragments_reassemble_exact_bytes_once(info):
    frame = bytes(range(256)) * 32
    packets = list(sending.fragment_frame(frame, info))
    assert max(map(len, packets)) <= proto.DATAGRAM_BYTES
    sink = receiver(info)
    assert sink.feed(packets[-1], PEER) is None
    assert sink.feed(packets[-1], PEER) is None
    results = [sink.feed(packet, PEER) for packet in reversed(packets[:-1])]
    assert [result for result in results if result] == [(info, frame)]
    assert sink.reserved_bytes == 0 and sink.stats["duplicate"] == 1
    assert all(sink.feed(packet, PEER) is None for packet in packets)
    assert sink.stats["completed"] == 1


@pytest.mark.parametrize("limits", [{}, {"timeout": 0.5}])
def test_loss_deadline_is_not_extended_by_duplicate_or_late_restart(
    info, monkeypatch, limits
):
    clock = [10.0]
    monkeypatch.setattr(receiving.time, "monotonic", lambda: clock[0])
    sink = receiver(info, **limits)
    packets = list(sending.fragment_frame(b"x" * 3000, info))
    sink.feed(packets[0], PEER)
    clock[0] += sink.timeout * 0.8
    sink.feed(packets[0], PEER)
    clock[0] += sink.timeout * 0.21
    sink.expire()
    assert sink.reserved_bytes == 0 and sink.stats["timeout"] == 1
    assert all(sink.feed(packet, PEER) is None for packet in packets)
    assert sink.stats["late"] == len(packets)


def test_new_frame_supersedes_incomplete_old_frame(info):
    sink = receiver(info)
    old = list(sending.fragment_frame(b"x" * 3000, info))
    sink.feed(old[0], PEER)
    newer = replace(info, frame_id=2)
    assert sink.feed(next(sending.fragment_frame(b"new", newer)), PEER) == (
        newer,
        b"new",
    )
    assert sink.feed(old[1], PEER) is None
    assert sink.stats["superseded"] == 1 and sink.reserved_bytes == 0


def test_nine_camera_streams_interleave_without_cross_contamination(info):
    infos = [
        replace(
            info,
            source=f"omx{i // 2 + 1}" if i < 6 else f"pinky{i - 5}",
            camera=f"cam{i % 2 + 1}",
        )
        for i in range(9)
    ]
    sink = receiving.FrameReceiver(
        {entry.key: (PEER, entry.session) for entry in infos}
    )
    packets = [
        list(sending.fragment_frame(bytes([i]) * 3000, entry))
        for i, entry in enumerate(infos)
    ]
    results = []
    for fragments in zip(*packets, strict=True):
        for packet in fragments:
            if result := sink.feed(packet, PEER):
                results.append(result)
    assert results == [(entry, bytes([i]) * 3000) for i, entry in enumerate(infos)]
    assert sink.reserved_bytes == 0


def test_peer_and_session_must_be_configured_explicitly(info):
    sink = receiver(info)
    packet = next(sending.fragment_frame(b"data", info))
    assert sink.feed(packet, ("127.0.0.2", 5302)) is None
    restarted = replace(info, session=uuid4(), frame_id=0)
    fresh_packet = next(sending.fragment_frame(b"data", restarted))
    assert sink.feed(fresh_packet, PEER) is None
    fresh_sink = receiver(restarted)
    assert fresh_sink.feed(fresh_packet, PEER) == (restarted, b"data")
    assert fresh_sink.feed(packet, PEER) is None
    assert sink.reserved_bytes == 0


@pytest.mark.parametrize("limits", [{"max_bytes": 2999}, {"max_inflight": 1}])
def test_capacity_limits_do_not_allocate_from_untrusted_size(info, limits):
    other = replace(info, camera="cam2")
    sink = receiving.FrameReceiver(
        {i.key: (PEER, i.session) for i in (info, other)}, **limits
    )
    sink.feed(next(sending.fragment_frame(b"a" * 3000, info)), PEER)
    sink.feed(next(sending.fragment_frame(b"b" * 3000, other)), PEER)
    assert sink.reserved_bytes <= sink.max_bytes
    assert len(sink._pending) <= sink.max_inflight
    assert sink.stats["capacity"] >= 1


@pytest.mark.parametrize(
    "mutation", ["version", "count", "size", "index", "id", "short", "oversize"]
)
def test_malformed_header_rejected_before_allocation(info, mutation):
    packet = next(sending.fragment_frame(b"data", info))
    fields = list(proto._HEADER.unpack_from(packet))
    if mutation == "version":
        fields[0] = b"BAD!"
    elif mutation == "count":
        fields[5] = 65535
    elif mutation == "size":
        fields[6] = 2**32 - 1
    elif mutation == "index":
        fields[4] = 2
    elif mutation == "id":
        fields[8] = b"bad/#"
    packet = proto._HEADER.pack(*fields) + packet[proto._HEADER.size :]
    if mutation == "short":
        packet = packet[:-1]
    elif mutation == "oversize":
        packet += b"x" * 1200
    sink = receiver(info)
    assert sink.feed(packet, PEER) is None
    assert sink.stats["malformed"] == 1 and sink.reserved_bytes == 0


def test_conflicting_duplicate_and_corrupt_payload_drop_whole_frame(info):
    packets = list(sending.fragment_frame(b"x" * 3000, info))
    sink = receiver(info)
    sink.feed(packets[0], PEER)
    sink.feed(packets[0][:-1] + b"y", PEER)
    assert sink.stats["conflict"] == 1 and sink.reserved_bytes == 0
    sink = receiver(info)
    packets[-1] = packets[-1][:-1] + b"y"
    assert all(sink.feed(packet, PEER) is None for packet in packets)
    assert sink.stats["checksum"] == 1 and sink.reserved_bytes == 0


@pytest.mark.parametrize("size", [0, proto.MAX_FRAME_BYTES + 1])
def test_sender_size_limits(info, size):
    with pytest.raises(ValueError, match="encoded frame"):
        list(sending.fragment_frame(b"x" * size, info))


@pytest.mark.parametrize("size", [500_000, proto.MAX_FRAME_BYTES])
def test_default_sender_and_receiver_complete_large_paced_frames(
    info, monkeypatch, size
):
    clock = [0.0]
    monkeypatch.setattr(sending.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        sending.time, "sleep", lambda delay: clock.__setitem__(0, clock[0] + delay)
    )
    sink = receiver(info)
    results = []

    class Socket:
        def sendto(self, packet, destination):
            if result := sink.feed(packet, PEER):
                results.append(result)
            return len(packet)

    frame = b"x" * size
    sending.FrameSender(Socket(), PEER).send(frame, info)
    assert results == [(info, frame)]
    assert 0.5 < sink.last_reassembly_seconds < sink.timeout
    assert sink.stats["timeout"] == 0 and sink.reserved_bytes == 0


def test_sender_paces_packets_and_frames(info, monkeypatch):
    clock = [0.0]
    times = []
    monkeypatch.setattr(sending.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        sending.time, "sleep", lambda delay: clock.__setitem__(0, clock[0] + delay)
    )

    class Socket:
        def sendto(self, packet, destination):
            times.append((clock[0], len(packet)))
            return len(packet)

    sender = sending.FrameSender(Socket(), PEER, fps=2, bitrate=80_000)
    sender.send(b"x" * 3000, info)
    sender.send(b"y" * 100, replace(info, frame_id=2))
    for (before, size), (after, _) in pairwise(times):
        assert after - before >= ((size + 28) * 8 / 80_000) - 1e-9
    assert times[-1][0] >= 0.5


def test_real_loopback_socket_roundtrip(info):
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx,
    ):
        tx.bind(("127.0.0.1", 0))
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1)
        sink = receiving.FrameReceiver({info.key: (tx.getsockname(), info.session)})
        sender = sending.FrameSender(tx, rx.getsockname())
        frame = b"encoded-frame-fixture" * 100
        sender.send(frame, info)
        result = None
        while result is None:
            packet, peer = rx.recvfrom(proto.DATAGRAM_BYTES + 1)
            result = sink.feed(packet, peer)
        assert result == (info, frame)


def test_wire_format_is_pinned_to_rpv1():
    # Literal values pin the OMX↔AI wire contract (README "시험 프로토콜 v1").
    assert proto.MAGIC == b"RPV1"
    assert proto.DATAGRAM_BYTES == 1200
    assert proto.MAX_FRAME_BYTES == 1024 * 1024
    assert proto._HEADER.format == "!4s16sQQHHII16s16s16s16s8s"
    assert proto._HEADER.size == 120
    assert proto.CHUNK_BYTES == 1080


def test_fragment_bytes_match_pre_split_udp_video_golden():
    # Expected digest was computed with the original communication/udp_video.py
    # at origin/main f25942c, not with this code, so a wire change fails here.
    info = proto.FrameInfo(
        "omx1",
        "cam1",
        "video",
        UUID("00112233-4455-6677-8899-aabbccddeeff"),
        7,
        123456789,
        "pick-1",
        "jpeg",
    )
    frame = bytes(range(256)) * 8
    packets = list(sending.fragment_frame(frame, info))
    assert [len(p) for p in packets] == [1200, 1088]
    assert packets[0][: proto._HEADER.size].hex() == (
        "5250563100112233445566778899aabbccddeeff0000000000000007"
        "00000000075bcd1500000002000008009f5edd586f6d783100000000"
        "000000000000000063616d3100000000000000000000000076696465"
        "6f00000000000000000000007069636b2d3100000000000000000000"
        "6a70656700000000"
    )
    assert (
        hashlib.sha256(b"".join(packets)).hexdigest()
        == "41d038c3e034d7fce648d203e3bff6868879fd0a72cf5578a73e8392eacd0abb"
    )
    parsed = [proto.parse_datagram(p) for p in packets]
    assert all(p[0] == info for p in parsed)
    assert b"".join(p[-1] for p in parsed) == frame


PORTABLE_SENDER = """
import sys
from uuid import uuid4
from udp.udp_frame_protocol import DATAGRAM_BYTES, FrameInfo, parse_datagram
from udp.udp_frame_sender import fragment_frame
info = FrameInfo("omx1", "cam1", "video", uuid4(), 1, 100, "pick-1")
frame = bytes(range(256)) * 10
parsed = [parse_datagram(p) for p in fragment_frame(frame, info)]
assert all(p[0] == info for p in parsed)
assert b"".join(p[-1] for p in parsed) == frame
assert "udp.udp_frame_receiver" not in sys.modules
assert DATAGRAM_BYTES == 1200
"""

PORTABLE_RECEIVER = """
import sys
import zlib
from uuid import uuid4
from udp.udp_frame_protocol import CHUNK_BYTES, DATAGRAM_BYTES, FrameInfo, pack_header
from udp.udp_frame_receiver import FrameReceiver
info = FrameInfo("omx1", "cam1", "video", uuid4(), 1, 100, "pick-1")
frame = bytes(range(256)) * 10
count = -(-len(frame) // CHUNK_BYTES)
peer = ("127.0.0.1", 5302)
sink = FrameReceiver({info.key: (peer, info.session)})
results = [
    sink.feed(
        pack_header(info, i, count, len(frame), zlib.crc32(frame))
        + frame[i * CHUNK_BYTES : (i + 1) * CHUNK_BYTES],
        peer,
    )
    for i in range(count)
]
assert results[-1] == (info, frame)
assert "udp.udp_frame_sender" not in sys.modules
assert DATAGRAM_BYTES == 1200
"""


@pytest.mark.parametrize(
    "module, script",
    [(sending, PORTABLE_SENDER), (receiving, PORTABLE_RECEIVER)],
    ids=["omx-protocol+sender", "ai-protocol+receiver"],
)
def test_portable_pair_without_project_or_third_party_imports(tmp_path, module, script):
    folder = tmp_path / "udp"
    folder.mkdir()
    for path in (proto.__file__, module.__file__):
        shutil.copy(path, folder)
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    subprocess.run(
        [sys.executable, "-S", "-c", script],
        cwd=tmp_path,
        env=environment,
        check=True,
        timeout=5,
    )


def test_demo_sender_receiver_are_separate_processes(tmp_path):
    import base64
    import json
    import select

    image = tmp_path / "frame.png"
    image.write_bytes(
        base64.b64decode(
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+j7X8AAAAASUVORK5CYII="
        )
    )
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as a,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as b,
    ):
        a.bind(("127.0.0.1", 0))
        b.bind(("127.0.0.1", 0))
        ports = (a.getsockname()[1], b.getsockname()[1])
    command = [sys.executable, str(SCRIPTS / "udp_video_demo.py")]
    options = [
        "--session",
        str(uuid4()),
        "--port",
        str(ports[0]),
        "--source-port",
        str(ports[1]),
    ]
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    with subprocess.Popen(
        [*command, "receive", *options],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=environment,
    ) as process:
        try:
            assert select.select([process.stdout], [], [], 5)[0]
            assert "mock AI" in process.stdout.readline()
            subprocess.run(
                [
                    *command,
                    "send",
                    *options,
                    "--file",
                    str(image),
                    "--codec",
                    "png",
                    "--frames",
                    "1",
                ],
                env=environment,
                capture_output=True,
                text=True,
                check=True,
                timeout=5,
            )
            assert select.select([process.stdout], [], [], 5)[0]
            event = json.loads(process.stdout.readline())
            assert (
                event["bytes"] == image.stat().st_size
                and event["stats"]["completed"] == 1
            )
        finally:
            process.terminate()
            process.wait(timeout=5)


def test_reassembly_time_and_counts_use_receiver_clock(info, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(receiving.time, "monotonic", lambda: clock[0])
    sink = receiver(info)
    packets = list(sending.fragment_frame(b"x" * 2000, info))
    sink.feed(packets[0], PEER)
    clock[0] += 0.1
    assert sink.feed(packets[1], PEER) == (info, b"x" * 2000)
    assert sink.last_reassembly_seconds == pytest.approx(0.1)
    assert sink.stats["received_datagrams"] == 2
    assert sink.stats["completed_bytes"] == 2000
