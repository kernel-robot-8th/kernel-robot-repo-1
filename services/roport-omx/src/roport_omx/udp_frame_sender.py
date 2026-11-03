# RPV1 source: RO:PORT control adapters/udp; portable stdlib implementation.
"""UDP frame sender (OMX camera host side). Imports only udp_frame_protocol."""

import math
import socket
import time
import zlib
from itertools import chain

from .udp_frame_protocol import (
    CHUNK_BYTES,
    MAX_FRAME_BYTES,
    FrameInfo,
    Peer,
    pack_header,
)


def fragment_frame(frame: bytes, info: FrameInfo):
    """Yield datagrams; capture/encoding and frame numbering belong to the caller."""
    if not isinstance(frame, bytes) or not 1 <= len(frame) <= MAX_FRAME_BYTES:
        raise ValueError("encoded frame must be 1..1048576 bytes")
    count = math.ceil(len(frame) / CHUNK_BYTES)
    checksum = zlib.crc32(frame)
    for index in range(count):
        header = pack_header(info, index, count, len(frame), checksum)
        yield header + frame[index * CHUNK_BYTES : (index + 1) * CHUNK_BYTES]


class FrameSender:
    """One camera per sender; pace wire bytes (including IP/UDP overhead) and FPS."""

    def __init__(
        self,
        sock: socket.socket,
        destination: Peer,
        *,
        fps: float,
        bitrate: int,
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
