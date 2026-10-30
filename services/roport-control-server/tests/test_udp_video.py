import os
import shutil
import socket
import subprocess
import sys
from dataclasses import replace
from itertools import pairwise
from uuid import uuid4

import pytest

from roport_control_server.communication import udp_video as video

PEER = ("127.0.0.1", 5302)


@pytest.fixture
def info():
    return video.FrameInfo("omx1", "cam1", "video", uuid4(), 1, 100, "pick-1")


def receiver(info, **limits):
    return video.FrameReceiver({info.key: (PEER, info.session)}, **limits)


def test_reordered_duplicate_fragments_reassemble_exact_bytes_once(info):
    frame = bytes(range(256)) * 32
    packets = list(video.fragment_frame(frame, info))
    assert max(map(len, packets)) <= video.DATAGRAM_BYTES
    sink = receiver(info)
    assert sink.feed(packets[-1], PEER) is None
    assert sink.feed(packets[-1], PEER) is None
    results = [sink.feed(packet, PEER) for packet in reversed(packets[:-1])]
    assert [result for result in results if result] == [(info, frame)]
    assert sink.reserved_bytes == 0 and sink.stats["duplicate"] == 1
    assert all(sink.feed(packet, PEER) is None for packet in packets)
    assert sink.stats["completed"] == 1


def test_loss_deadline_is_not_extended_by_duplicate_or_late_restart(info, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(video.time, "monotonic", lambda: clock[0])
    sink = receiver(info, timeout=0.5)
    packets = list(video.fragment_frame(b"x" * 3000, info))
    sink.feed(packets[0], PEER)
    clock[0] += 0.4
    sink.feed(packets[0], PEER)
    clock[0] += 0.11
    sink.expire()
    assert sink.reserved_bytes == 0 and sink.stats["timeout"] == 1
    assert all(sink.feed(packet, PEER) is None for packet in packets)
    assert sink.stats["late"] == len(packets)


def test_new_frame_supersedes_incomplete_old_frame(info):
    sink = receiver(info)
    old = list(video.fragment_frame(b"x" * 3000, info))
    sink.feed(old[0], PEER)
    newer = replace(info, frame_id=2)
    assert sink.feed(next(video.fragment_frame(b"new", newer)), PEER) == (newer, b"new")
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
    sink = video.FrameReceiver({entry.key: (PEER, entry.session) for entry in infos})
    packets = [
        list(video.fragment_frame(bytes([i]) * 3000, entry))
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
    packet = next(video.fragment_frame(b"data", info))
    assert sink.feed(packet, ("127.0.0.2", 5302)) is None
    restarted = replace(info, session=uuid4(), frame_id=0)
    fresh_packet = next(video.fragment_frame(b"data", restarted))
    assert sink.feed(fresh_packet, PEER) is None
    fresh_sink = receiver(restarted)
    assert fresh_sink.feed(fresh_packet, PEER) == (restarted, b"data")
    assert fresh_sink.feed(packet, PEER) is None
    assert sink.reserved_bytes == 0


@pytest.mark.parametrize("limits", [{"max_bytes": 2999}, {"max_inflight": 1}])
def test_capacity_limits_do_not_allocate_from_untrusted_size(info, limits):
    other = replace(info, camera="cam2")
    sink = video.FrameReceiver(
        {i.key: (PEER, i.session) for i in (info, other)}, **limits
    )
    sink.feed(next(video.fragment_frame(b"a" * 3000, info)), PEER)
    sink.feed(next(video.fragment_frame(b"b" * 3000, other)), PEER)
    assert sink.reserved_bytes <= sink.max_bytes
    assert len(sink._pending) <= sink.max_inflight
    assert sink.stats["capacity"] >= 1


@pytest.mark.parametrize(
    "mutation", ["version", "count", "size", "index", "id", "short", "oversize"]
)
def test_malformed_header_rejected_before_allocation(info, mutation):
    packet = next(video.fragment_frame(b"data", info))
    fields = list(video._HEADER.unpack_from(packet))
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
    packet = video._HEADER.pack(*fields) + packet[video._HEADER.size :]
    if mutation == "short":
        packet = packet[:-1]
    elif mutation == "oversize":
        packet += b"x" * 1200
    sink = receiver(info)
    assert sink.feed(packet, PEER) is None
    assert sink.stats["malformed"] == 1 and sink.reserved_bytes == 0


def test_conflicting_duplicate_and_corrupt_payload_drop_whole_frame(info):
    packets = list(video.fragment_frame(b"x" * 3000, info))
    sink = receiver(info)
    sink.feed(packets[0], PEER)
    sink.feed(packets[0][:-1] + b"y", PEER)
    assert sink.stats["conflict"] == 1 and sink.reserved_bytes == 0
    sink = receiver(info)
    packets[-1] = packets[-1][:-1] + b"y"
    assert all(sink.feed(packet, PEER) is None for packet in packets)
    assert sink.stats["checksum"] == 1 and sink.reserved_bytes == 0


@pytest.mark.parametrize("size", [0, video.MAX_FRAME_BYTES + 1])
def test_sender_size_limits(info, size):
    with pytest.raises(ValueError, match="encoded frame"):
        list(video.fragment_frame(b"x" * size, info))


def test_maximum_frame_is_accepted(info):
    sink = receiver(info)
    frame = b"x" * video.MAX_FRAME_BYTES
    result = None
    for packet in video.fragment_frame(frame, info):
        result = sink.feed(packet, PEER)
    assert result == (info, frame)


def test_sender_paces_packets_and_frames(info, monkeypatch):
    clock = [0.0]
    times = []
    monkeypatch.setattr(video.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        video.time, "sleep", lambda delay: clock.__setitem__(0, clock[0] + delay)
    )

    class Socket:
        def sendto(self, packet, destination):
            times.append((clock[0], len(packet)))
            return len(packet)

    sender = video.FrameSender(Socket(), PEER, fps=2, bitrate=80_000)
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
        sink = video.FrameReceiver({info.key: (tx.getsockname(), info.session)})
        sender = video.FrameSender(tx, rx.getsockname())
        frame = b"encoded-frame-fixture" * 100
        sender.send(frame, info)
        result = None
        while result is None:
            packet, peer = rx.recvfrom(video.DATAGRAM_BYTES + 1)
            result = sink.feed(packet, peer)
        assert result == (info, frame)


def test_portable_without_project_or_third_party_imports(tmp_path):
    shutil.copy(video.__file__, tmp_path / "udp_video.py")
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    subprocess.run(
        [
            sys.executable,
            "-S",
            "-c",
            "import udp_video; assert udp_video.DATAGRAM_BYTES == 1200",
        ],
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
    command = [sys.executable, "-m", "roport_control_server.examples.udp_video_demo"]
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
    monkeypatch.setattr(video.time, "monotonic", lambda: clock[0])
    sink = receiver(info)
    packets = list(video.fragment_frame(b"x" * 2000, info))
    sink.feed(packets[0], PEER)
    clock[0] += 0.1
    assert sink.feed(packets[1], PEER) == (info, b"x" * 2000)
    assert sink.last_reassembly_seconds == pytest.approx(0.1)
    assert sink.stats["received_datagrams"] == 2
    assert sink.stats["completed_bytes"] == 2000
