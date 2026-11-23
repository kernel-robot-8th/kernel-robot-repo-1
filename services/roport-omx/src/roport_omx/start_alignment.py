"""Plan a bounded joint-space start alignment, without checking scene collisions."""

import json

from .act_support import Rejected, finite, vector


def _within(measured, reference, tolerance):
    return all(
        abs(actual - expected) <= limit
        for actual, expected, limit in zip(measured, reference, tolerance, strict=True)
    )


class StartAlignment:
    def __init__(self, config, *, target_tolerance=None):
        if not isinstance(config, dict) or set(config) != {
            "start",
            "target",
            "start_tolerance",
            "tracking_tolerance",
            "max_rate",
            "timeout_s",
            "settled_samples",
        }:
            raise Rejected("explicit start alignment configuration required")
        self.start = vector(config["start"])
        self.target = vector(config["target"])
        self.start_tolerance = vector(config["start_tolerance"])
        self.tracking_tolerance = vector(config["tracking_tolerance"])
        self.max_rate = vector(config["max_rate"])
        self.target_tolerance = vector(
            self.start_tolerance if target_tolerance is None else target_tolerance
        )
        if any(
            limit <= 0
            for limit in (
                *self.start_tolerance,
                *self.tracking_tolerance,
                *self.max_rate,
                *self.target_tolerance,
            )
        ):
            raise Rejected("positive alignment tolerances and joint rates required")
        if any(
            a > b
            for a, b in zip(self.target_tolerance, self.tracking_tolerance, strict=True)
        ):
            raise Rejected(
                "target tolerance must not exceed alignment tracking tolerance"
            )
        self.timeout_s = config["timeout_s"]
        self.settled_samples = config["settled_samples"]
        if (
            not finite(self.timeout_s)
            or self.timeout_s <= 0
            or type(self.settled_samples) is not int
            or self.settled_samples < 1
        ):
            raise Rejected("positive alignment timeout and sample count required")
        self.done = False
        self._started_at = None
        self._progress = 0
        self._settle_streak = 0

    def begin(self, measured, now):
        if self._started_at is not None or not finite(now):
            raise Rejected("invalid alignment start time or alignment already started")
        measured = vector(measured)
        if not _within(measured, self.start, self.start_tolerance):
            raise Rejected("measured pose differs from the registered alignment start")
        delta = tuple(
            target - actual
            for actual, target in zip(measured, self.target, strict=True)
        )
        duration = max(
            abs(step) / rate for step, rate in zip(delta, self.max_rate, strict=True)
        )
        if not finite(duration) or duration >= self.timeout_s:
            raise Rejected(
                "alignment timeout is too short for the configured joint rates"
            )
        self._initial = self._previous_command = measured
        self._tracking_reference = measured
        self._previous_time = self._started_at = now
        self._delta = delta
        self._duration = duration

    def step(self, measured, now):
        if self.done:
            return None
        if self._started_at is None or not finite(now) or now < self._previous_time:
            raise Rejected("start alignment requires a monotonic clock")
        measured = vector(measured)
        if now - self._started_at >= self.timeout_s:
            delta = [a - b for a, b in zip(measured, self.target, strict=True)]
            raise Rejected(
                "start alignment timed out before settling: "
                + json.dumps(
                    {
                        "measured": measured,
                        "target": self.target,
                        "delta_to_target": delta,
                        "outside_target_tolerance": [
                            i
                            for i, (d, t) in enumerate(
                                zip(delta, self.target_tolerance, strict=True)
                            )
                            if abs(d) > t
                        ],
                        "elapsed_s": now - self._started_at,
                    }
                )
            )
        if not _within(measured, self._previous_command, self.tracking_tolerance):
            raise Rejected(
                "measured pose exceeded the start alignment tracking limit: "
                + json.dumps(
                    {
                        "measured": measured,
                        "target": self._previous_command,
                        "delta": [
                            a - b
                            for a, b in zip(
                                measured, self._previous_command, strict=True
                            )
                        ],
                        "elapsed_s": now - self._started_at,
                    }
                )
            )

        elapsed = now - self._previous_time
        self._tracking_reference = self._previous_command
        if elapsed == 0:
            return list(self._previous_command)
        if self._previous_command == self.target:
            if _within(measured, self.target, self.target_tolerance):
                self._settle_streak += 1
            else:
                self._settle_streak = 0
        self._previous_time = now
        if self._settle_streak >= self.settled_samples:
            self.done = True
            return None

        self._progress = (
            min(
                1,
                (now - self._started_at) / self._duration,
                self._progress + elapsed / self._duration,
            )
            if self._duration
            else 1
        )
        self._previous_command = (
            self.target
            if self._progress == 1
            else tuple(
                initial + self._progress * delta
                for initial, delta in zip(self._initial, self._delta, strict=True)
            )
        )
        return list(self._previous_command)

    def check_write(self, measured, target):
        """Check the last planned point against the newest motor reading before writing."""
        if (
            self._started_at is None
            or self.done
            or vector(target) != self._previous_command
        ):
            raise Rejected("motor command differs from the planned alignment target")
        measured = vector(measured)
        if not _within(measured, self._tracking_reference, self.tracking_tolerance):
            delta = [
                a - b for a, b in zip(measured, self._tracking_reference, strict=True)
            ]
            raise Rejected(
                "alignment tracking limit exceeded before motor write: "
                + json.dumps(
                    {
                        "measured": measured,
                        "reference": self._tracking_reference,
                        "delta": delta,
                        "tolerance": self.tracking_tolerance,
                        "outside_tracking_tolerance": [
                            i
                            for i, (d, t) in enumerate(
                                zip(delta, self.tracking_tolerance, strict=True)
                            )
                            if abs(d) > t
                        ],
                    }
                )
            )

    def check_hold(self, measured):
        if not self.done or not _within(
            vector(measured), self.target, self.target_tolerance
        ):
            raise Rejected("measured pose drifted from the settled alignment target")
