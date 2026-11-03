"""UDP frame wire v1: constants, FrameInfo and header pack/parse (stdlib only)."""

import math
import re
import struct
from dataclasses import dataclass
from uuid import UUID

DATAGRAM_BYTES = 1200
MAX_FRAME_BYTES = 1024 * 1024
MAGIC = b"RPV1"  # wire version; incompatible header changes bump it
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


def pack_header(info: FrameInfo, index: int, count: int, size: int, crc: int) -> bytes:
    return _HEADER.pack(
        MAGIC,
        info.session.bytes,
        info.frame_id,
        info.captured_ns,
        index,
        count,
        size,
        crc,
        info.source.encode(),
        info.camera.encode(),
        info.stream.encode(),
        info.observation_group.encode(),
        info.codec.encode(),
    )


def parse_datagram(packet: bytes):
    if not _HEADER.size < len(packet) <= DATAGRAM_BYTES:
        raise ValueError("invalid datagram size")
    magic, session, frame_id, captured, index, count, size, crc, *names = (
        _HEADER.unpack_from(packet)
    )
    if magic != MAGIC or not 1 <= size <= MAX_FRAME_BYTES:
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
