"""Bind independently encoded camera frames to one measured observation."""

from copy import deepcopy
from uuid import UUID

from .udp_frame_protocol import FrameInfo

IMAGE_FIELDS = frozenset(
    (
        "source",
        "camera",
        "stream",
        "session",
        "frame_id",
        "captured_ns",
        "observation_group",
    )
)


def validate_images(images):
    """Validate the exact JSON header manifest; never accept raw image payloads."""
    if not isinstance(images, dict) or not images:
        raise ValueError("images must be a nonempty camera manifest")
    group = None
    for camera, header in images.items():
        if not isinstance(header, dict) or set(header) != IMAGE_FIELDS:
            raise ValueError("image manifest requires exact frame header fields")
        if camera != header["camera"] or not isinstance(header["session"], str):
            raise ValueError("camera key/session mismatch")
        info = FrameInfo(**{**header, "session": UUID(header["session"])})
        binding = (info.source, info.session, info.observation_group)
        if group is not None and group != binding:
            raise ValueError(
                "camera frames must share source/session/observation group"
            )
        group = binding
    return deepcopy(images)


class ObservationImageProducer:
    """Send a complete set of JPEG bytes and return its header-only manifest."""

    def __init__(self, senders, *, source, stream, session):
        if not isinstance(senders, dict) or not senders:
            raise ValueError("explicit camera senders required")
        self.senders = dict(senders)
        self.source, self.stream, self.session = source, stream, UUID(str(session))

    def send(self, frames, *, frame_id, captured_ns, observation_group):
        if not isinstance(frames, dict) or set(frames) != set(self.senders):
            raise ValueError("complete current camera set required")
        infos = {
            camera: FrameInfo(
                self.source,
                camera,
                self.stream,
                self.session,
                frame_id,
                captured_ns,
                observation_group,
            )
            for camera in frames
        }
        # Validate all frames before sending any camera.
        from .udp_frame_sender import fragment_frame

        for camera, frame in frames.items():
            next(fragment_frame(frame, infos[camera]))
        for camera, frame in frames.items():
            self.senders[camera].send(frame, infos[camera])
        return validate_images(
            {
                camera: {
                    **{field: getattr(info, field) for field in IMAGE_FIELDS},
                    "session": str(info.session),
                }
                for camera, info in infos.items()
            }
        )
