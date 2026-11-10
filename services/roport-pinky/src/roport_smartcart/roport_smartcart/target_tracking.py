"""Image-space target selection for the PC preview; no robot commands."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class Detection:
    track_id: int
    box: tuple[float, float, float, float]
    confidence: float
    class_id: int

    def __post_init__(self):
        if (type(self.track_id) is not int or self.track_id < 0
                or type(self.class_id) is not int or self.class_id < 0
                or len(self.box) != 4 or not all(isfinite(v) for v in self.box)
                or self.box[0] >= self.box[2] or self.box[1] >= self.box[3]
                or not isfinite(self.confidence) or not 0 <= self.confidence <= 1):
            raise ValueError('invalid tracked detection')


class TargetSelection:
    """Latch loss until explicit selection; a tracker ID is not an identity."""

    def __init__(self, max_age=0.5, max_center_shift=0.75):
        if (not isfinite(max_age) or max_age <= 0
                or not isfinite(max_center_shift) or max_center_shift <= 0):
            raise ValueError('invalid selection limits')
        self.max_age = max_age
        self.max_center_shift = max_center_shift
        self.target = None
        self.reason = 'SELECT TARGET'
        self.last_stamp = None

    def clear(self, reason='SELECT TARGET'):
        self.target = None
        self.last_stamp = None
        self.reason = reason

    def fresh(self, stamp, now):
        return (isfinite(stamp) and isfinite(now)
                and 0 <= now - stamp <= self.max_age)

    def select(self, detections, x, y, stamp, now):
        self.clear()
        if not self.fresh(stamp, now):
            self.clear('STALE VIDEO - RESELECT')
            return
        hits = [d for d in detections if d.box[0] <= x <= d.box[2]
                and d.box[1] <= y <= d.box[3]]
        if len(hits) != 1:
            self.reason = 'AMBIGUOUS / EMPTY CLICK'
            return
        self.target = hits[0]
        self.last_stamp = stamp
        self.reason = 'TRACKING - PREVIEW ONLY'

    def update(self, detections, stamp, now):
        if not self.fresh(stamp, now):
            self.clear('STALE VIDEO - RESELECT')
            return
        if self.target is None:
            return
        if stamp < self.last_stamp:
            self.clear('OUT OF ORDER VIDEO - RESELECT')
            return
        if stamp == self.last_stamp:
            return
        matches = [d for d in detections if d.track_id == self.target.track_id]
        if len(matches) != 1 or matches[0].class_id != self.target.class_id:
            self.clear('TARGET LOST - RESELECT')
            return
        old, new = self.target.box, matches[0].box
        width, height = old[2] - old[0], old[3] - old[1]
        dx = abs((new[0] + new[2] - old[0] - old[2]) / 2) / width
        dy = abs((new[1] + new[3] - old[1] - old[3]) / 2) / height
        overlaps = [d for d in detections if d.track_id != self.target.track_id
                    and min(d.box[2], new[2]) > max(d.box[0], new[0])
                    and min(d.box[3], new[3]) > max(d.box[1], new[1])]
        if max(dx, dy) > self.max_center_shift or overlaps:
            self.clear('AMBIGUOUS TARGET - RESELECT')
            return
        # shortcut: ID + spatial continuity cannot detect a smooth identity swap.
        # Add and field-validate appearance Re-ID before autonomous following.
        self.target = matches[0]
        self.last_stamp = stamp
