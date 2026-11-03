"""Resolve only complete RPV1 JPEG frames bound to the current observation."""

import time
from collections import OrderedDict
from datetime import UTC, datetime
from uuid import UUID

from udp_frame_protocol import FrameInfo
from udp_frame_receiver import FrameReceiver

IMAGE_FIELDS = {
    "source",
    "camera",
    "stream",
    "session",
    "frame_id",
    "captured_ns",
    "observation_group",
}


class ImageNotReady(ValueError):
    """The valid current observation has no complete eligible image yet."""


def validate_images(images: dict) -> dict[str, FrameInfo]:
    if not isinstance(images, dict) or not images:
        raise ValueError("complete image manifest required")
    result = {}
    for camera, entry in images.items():
        if not isinstance(entry, dict) or set(entry) != IMAGE_FIELDS:
            raise ValueError("exact image header metadata required")
        try:
            info = FrameInfo(**(entry | {"session": UUID(entry["session"])}))
        except (TypeError, ValueError, AttributeError) as exc:
            raise ValueError("invalid image header metadata") from exc
        if camera != info.camera:
            raise ValueError("camera key differs from header")
        result[camera] = info
    if len({(i.session, i.observation_group) for i in result.values()}) != 1:
        raise ValueError("images must share session and observation group")
    return result


def _captured_ns(observation):
    try:
        captured = datetime.fromisoformat(observation["captured_at"])
        if captured.utcoffset() is None:
            raise ValueError("capture timestamp requires offset")
        elapsed = captured - datetime(1970, 1, 1, tzinfo=UTC)
        return (
            elapsed.days * 86400 + elapsed.seconds
        ) * 10**9 + elapsed.microseconds * 1000
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("observation captured_at required") from exc


class ObservationImageResolver:
    """Single consumer; explicit limits, peer/session allowlist and no stale fallback.

    Feed datagrams from the composition's socket loop before calling the resolver.
    max_bytes bounds both incomplete reassembly and the completed frame cache.
    """

    def __init__(
        self,
        streams,
        *,
        timeout,
        max_inflight,
        max_bytes,
        max_frames,
        max_age_ns,
        max_timestamp_skew_ns,
        clock_ns=time.time_ns,
    ):
        for value in (max_frames, max_age_ns, max_timestamp_skew_ns):
            if type(value) is not int or value <= 0:
                raise ValueError("explicit positive integer image limits required")
        cameras = [key[1] for key in streams]
        if len(set(cameras)) != len(cameras):
            raise ValueError("one allowlisted stream per camera required")
        self.receiver = FrameReceiver(
            streams, timeout=timeout, max_inflight=max_inflight, max_bytes=max_bytes
        )
        self.cameras = set(cameras)
        self.max_frames = max_frames
        self.max_bytes = max_bytes
        self.max_age_ns = max_age_ns
        self.max_timestamp_skew_ns = max_timestamp_skew_ns
        self.clock_ns = clock_ns
        self._frames = OrderedDict()
        self._bytes = 0

    def _remove(self, info):
        self._bytes -= len(self._frames.pop(info))

    def _expire(self, now):
        for info in list(self._frames):
            if now - info.captured_ns > self.max_age_ns:
                self._remove(info)

    def feed(self, packet: bytes, peer) -> bool:
        now = self.clock_ns()
        self._expire(now)
        completed = self.receiver.feed(packet, peer)
        if completed is None:
            return False
        info, frame = completed
        if (
            info.codec != "jpeg"
            or now - info.captured_ns > self.max_age_ns
            or info.captured_ns - now > self.max_timestamp_skew_ns
            or len(frame) > self.max_bytes
        ):
            return False
        while self._frames and (
            len(self._frames) >= self.max_frames
            or self._bytes + len(frame) > self.max_bytes
        ):
            self._remove(next(iter(self._frames)))
        self._frames[info] = frame
        self._bytes += len(frame)
        return True

    def __call__(self, observation: dict) -> dict[str, bytes]:
        if not isinstance(observation, dict):
            raise TypeError("observation object required")
        headers = validate_images(observation.get("images"))
        if set(headers) != self.cameras:
            raise ValueError("complete configured camera set required")
        captured = _captured_ns(observation)
        now = self.clock_ns()
        self._expire(now)
        if captured > now or now - captured > self.max_age_ns:
            raise ValueError("stale or future arm observation")
        result = {}
        for camera, info in headers.items():
            if self.receiver.streams.get(info.key, (None, None))[1] != info.session:
                raise ValueError("unregistered image stream/session")
            if abs(info.captured_ns - captured) > self.max_timestamp_skew_ns:
                raise ValueError("arm/image timestamp skew exceeded")
            if info not in self._frames:
                raise ImageNotReady("current complete image unavailable")
            result[camera] = self._frames[info]
        return result
