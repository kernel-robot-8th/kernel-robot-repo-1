"""Compare current map_server replies with the validated local trinary map."""

from hashlib import sha256
import math
from pathlib import Path
import time

from roport_pinky.trial_nav2 import map_reference

from .goals import _read_yaml


def reference_from_file(map_yaml):
    path = Path(map_yaml)
    image = path.parent / _read_yaml(path)['image']
    version = sha256(path.read_bytes() + image.read_bytes()).hexdigest()[:16]
    return map_reference(path, {'map_id': path.stem, 'map_version': version})


def matches_reference(grid, reference):
    cfg, width, height, cells = reference
    info, origin = grid.info, grid.info.origin
    q = origin.orientation
    expected_yaw = cfg['origin'][2]
    # Quaternions q and -q represent the same orientation. Compare their dot
    # product, including x/y, to reject non-planar or malformed map origins.
    quaternion = (q.x, q.y, q.z, q.w)
    dot = q.z * math.sin(expected_yaw / 2) + q.w * math.cos(expected_yaw / 2)
    numbers = (info.resolution, origin.position.x, origin.position.y,
               origin.position.z, *quaternion)
    return (all(math.isfinite(v) for v in numbers)
            and grid.header.frame_id == 'map'
            and (info.width, info.height) == (width, height)
            and abs(info.resolution - cfg['resolution']) < 1e-7
            and abs(origin.position.x - cfg['origin'][0]) < 1e-6
            and abs(origin.position.y - cfg['origin'][1]) < 1e-6
            and abs(origin.position.z) < 1e-6
            and abs(sum(v * v for v in quaternion) - 1.) < 1e-6
            and abs(abs(dot) - 1.) < 1e-10
            and list(grid.data) == cells)


class LiveMapCheck:
    """One asynchronous GetMap at a time, with a two-second proof lifetime.

    A map's header stamp is its load time, so freshness uses service replies.
    Missing, timed-out or mismatched replies revoke readiness during a goal too.
    """

    def __init__(self, client, request_factory, reference, clock=time.monotonic):
        self.client, self.request_factory = client, request_factory
        self.reference, self.clock = reference, clock
        self.pending = None
        self.last_request = -math.inf
        self.checked_at = None
        self.error = 'map_server/map: current map not verified'

    def recheck(self):
        if self.pending is not None:
            self.pending.cancel()
        self.pending = None
        self.last_request = -math.inf
        self.checked_at = None

    def poll(self):
        now = self.clock()
        if not self.client.service_is_ready():
            self.checked_at = None
            self.error = 'map_server/map: service unavailable'
            if self.pending is not None:
                self.pending.cancel()
                self.pending = None
            return
        if self.pending is not None:
            if self.pending.done():
                try:
                    valid = matches_reference(self.pending.result().map, self.reference)
                    self.checked_at = now if valid else None
                    self.error = 'map_server/map: running map differs from --map'
                except Exception:
                    self.checked_at = None
                    self.error = 'map_server/map: response failed'
                self.pending = None
            elif now - self.last_request >= 2.:
                self.pending.cancel()
                self.pending = None
                self.checked_at = None
                self.error = 'map_server/map: response timed out'
            else:
                return
        if now - self.last_request >= 1. and self.client.service_is_ready():
            self.last_request = now
            try:
                self.pending = self.client.call_async(self.request_factory())
            except Exception:
                self.checked_at = None
                self.error = 'map_server/map: request failed'

    def failures(self):
        if self.checked_at is None:
            return [self.error]
        if self.clock() - self.checked_at > 2.:
            return ['map_server/map: verification expired']
        return []
