"""Bounded, lossy UDP transport for independent encoded video frames (wire v1)."""

import math
import re
import socket
import struct
import time
import zlib
from collections import Counter
from dataclasses import dataclass, field
from itertools import chain
from uuid import UUID

DATAGRAM_BYTES = 1200
MAX_FRAME_BYTES = 1024 * 1024
_HEADER = struct.Struct("!4s16sQQHHII16s16s16s16s8s")
CHUNK_BYTES = DATAGRAM_BYTES - _HEADER.size
_ID = re.compile(r"[A-Za-z0-9_.-]{1,16}\Z")
Stream = tuple[str, str, str]
Peer = tuple[str, int]


@dataclass(frozen=True)
class FrameInfo:
    source: str
    camera: str
    stream: str
    session: UUID
    frame_id: int
    captured_ns: int
    observation_group: str
    codec: str = "jpeg"

    def __post_init__(self):
        for value in (self.source, self.camera, self.stream, self.observation_group):
            if not isinstance(value, str) or not _ID.fullmatch(value):
                raise ValueError("frame identifiers require 1..16 ASCII characters")
        if not isinstance(self.session, UUID) or self.codec not in ("jpeg", "png"):
            raise ValueError("session UUID and independent JPEG/PNG frames required")
        for value in (self.frame_id, self.captured_ns):
            if type(value) is not int or not 0 <= value < 2**64:
                raise ValueError("frame_id and captured_ns require uint64")

    @property
    def key(self) -> Stream:
        return self.source, self.camera, self.stream


def fragment_frame(frame: bytes, info: FrameInfo):
    """Yield datagrams; capture/encoding and frame numbering belong to the caller."""
    if not isinstance(frame, bytes) or not 1 <= len(frame) <= MAX_FRAME_BYTES:
        raise ValueError("encoded frame must be 1..1048576 bytes")
    count = math.ceil(len(frame) / CHUNK_BYTES)
    checksum = zlib.crc32(frame)
    for index in range(count):
        header = _HEADER.pack(
            b"RPV1",
            info.session.bytes,
            info.frame_id,
            info.captured_ns,
            index,
            count,
            len(frame),
            checksum,
            info.source.encode(),
            info.camera.encode(),
            info.stream.encode(),
            info.observation_group.encode(),
            info.codec.encode(),
        )
        yield header + frame[index * CHUNK_BYTES : (index + 1) * CHUNK_BYTES]


def _parse(packet: bytes):
    if not _HEADER.size < len(packet) <= DATAGRAM_BYTES:
        raise ValueError("invalid datagram size")
    magic, session, frame_id, captured, index, count, size, crc, *names = (
        _HEADER.unpack_from(packet)
    )
    if magic != b"RPV1" or not 1 <= size <= MAX_FRAME_BYTES:
        raise ValueError("invalid version/frame size")
    if count != math.ceil(size / CHUNK_BYTES) or not 0 <= index < count:
        raise ValueError("invalid fragment count/index")
    chunk = packet[_HEADER.size :]
    if len(chunk) != min(CHUNK_BYTES, size - index * CHUNK_BYTES):
        raise ValueError("invalid fragment length")
    source, camera, stream, group, codec = (
        n.rstrip(b"\0").decode("ascii") for n in names
    )
    info = FrameInfo(
        source, camera, stream, UUID(bytes=session), frame_id, captured, group, codec
    )
    return info, index, count, size, crc, chunk


@dataclass
class _Pending:
    info: FrameInfo
    count: int
    size: int
    crc: int
    deadline: float
    started_at: float
    chunks: dict[int, bytes] = field(default_factory=dict)


class FrameReceiver:
    """Single consumer; one pending frame per allowlisted stream, no background I/O."""

    def __init__(
        self,
        streams: dict[Stream, tuple[Peer, UUID]],
        *,
        timeout: float = 0.5,
        max_inflight: int = 16,
        max_bytes: int = 8 * MAX_FRAME_BYTES,
    ):
        if not streams or len(streams) > 32:
            raise ValueError("configure 1..32 known streams")
        if (
            not math.isfinite(timeout)
            or timeout <= 0
            or max_inflight < 1
            or max_bytes < 1
        ):
            raise ValueError("positive finite receive limits required")
        for key, (peer, session) in streams.items():
            FrameInfo(*key, session, 0, 0, "validation")
            socket.inet_pton(socket.AF_INET, peer[0])
            if not 1 <= peer[1] <= 65535:
                raise ValueError("peer requires an IPv4 address and source port")
        self.streams = streams.copy()
        self.timeout = timeout
        self.max_inflight = max_inflight
        self.max_bytes = max_bytes
        self.stats: Counter = Counter()
        self._pending: dict[Stream, _Pending] = {}
        self._latest: dict[Stream, int] = {}
        self.reserved_bytes = 0
        self.last_reassembly_seconds: float | None = None

    def _drop(self, key: Stream, reason: str):
        self.reserved_bytes -= self._pending.pop(key).size
        self.stats[reason] += 1

    def expire(self, now: float | None = None):
        now = time.monotonic() if now is None else now
        for key, pending in list(self._pending.items()):
            if now >= pending.deadline:
                self._drop(key, "timeout")

    def feed(self, packet: bytes, peer: Peer) -> tuple[FrameInfo, bytes] | None:
        now = time.monotonic()
        self.stats["received_datagrams"] += 1
        self.expire(now)
        if peer not in {entry[0] for entry in self.streams.values()}:
            self.stats["untrusted_peer"] += 1
            return None
        try:
            info, index, count, size, crc, chunk = _parse(packet)
        except (ValueError, UnicodeError, struct.error):
            self.stats["malformed"] += 1
            return None
        key = info.key
        if self.streams.get(key) != (peer, info.session):
            self.stats["unknown_stream_session"] += 1
            return None
        pending = self._pending.get(key)
        latest = self._latest.get(key, -1)
        if info.frame_id < latest or (info.frame_id == latest and pending is None):
            self.stats["late"] += 1
            return None
        if info.frame_id > latest:
            self._latest[key] = info.frame_id
            if pending:
                self._drop(key, "superseded")
            if (
                len(self._pending) >= self.max_inflight
                or self.reserved_bytes + size > self.max_bytes
            ):
                self.stats["capacity"] += 1
                return None
            pending = _Pending(info, count, size, crc, now + self.timeout, now)
            self._pending[key] = pending
            self.reserved_bytes += size
        if (pending.info, pending.count, pending.size, pending.crc) != (
            info,
            count,
            size,
            crc,
        ):
            self._drop(key, "conflict")
            return None
        if index in pending.chunks:
            if pending.chunks[index] != chunk:
                self._drop(key, "conflict")
            else:
                self.stats["duplicate"] += 1
            return None
        pending.chunks[index] = chunk
        if len(pending.chunks) != count:
            return None
        frame = b"".join(pending.chunks[i] for i in range(count))
        if zlib.crc32(frame) != crc:
            self._drop(key, "checksum")
            return None
        self.last_reassembly_seconds = now - pending.started_at
        self.stats["completed_bytes"] += size
        self._drop(key, "completed")
        return info, frame


class FrameSender:
    """One camera per sender; pace wire bytes (including IP/UDP overhead) and FPS."""

    def __init__(
        self,
        sock: socket.socket,
        destination: Peer,
        *,
        fps: float = 10,
        bitrate: int = 8_000_000,
    ):
        if (
            not math.isfinite(fps)
            or fps <= 0
            or type(bitrate) is not int
            or bitrate <= 0
        ):
            raise ValueError("positive finite FPS/bitrate required")
        self.sock = sock
        self.destination = destination
        self.fps = fps
        self.bitrate = bitrate
        self._next_frame = 0.0
        self._next_packet = 0.0

    def send(self, frame: bytes, info: FrameInfo) -> int:
        packets = fragment_frame(frame, info)
        # Validate before waiting or sending any fragment.
        first = next(packets)
        time.sleep(max(0, self._next_frame - time.monotonic()))
        self._next_frame = time.monotonic() + 1 / self.fps
        total = 0
        for packet in chain((first,), packets):
            time.sleep(max(0, self._next_packet - time.monotonic()))
            sent_at = time.monotonic()
            total += self.sock.sendto(packet, self.destination)
            self._next_packet = sent_at + (len(packet) + 28) * 8 / self.bitrate
        return total
