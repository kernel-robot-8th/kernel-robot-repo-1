# Source: RO:PORT control adapters/udp/ (RPV1); keep wire compatibility.
"""UDP frame receiver (AI side). Imports only udp_frame_protocol."""

import math
import socket
import struct
import time
import zlib
from collections import Counter
from dataclasses import dataclass, field
from uuid import UUID

from udp_frame_protocol import FrameInfo, Peer, Stream, parse_datagram


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
        timeout: float,
        max_inflight: int,
        max_bytes: int,
    ):
        if not streams or len(streams) > 32:
            raise ValueError("configure 1..32 known streams")
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
            or type(max_inflight) is not int
            or max_inflight < 1
            or type(max_bytes) is not int
            or max_bytes < 1
        ):
            raise ValueError("positive finite receive limits required")
        for key, (peer, session) in streams.items():
            FrameInfo(*key, session, 0, 0, "validation")
            socket.inet_pton(socket.AF_INET, peer[0])
            if type(peer[1]) is not int or not 1 <= peer[1] <= 65535:
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
            info, index, count, size, crc, chunk = parse_datagram(packet)
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
