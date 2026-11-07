"""ROS-free velocity ownership building block, not a driver watchdog replacement.

Distances/speeds use metres and radians; timestamps use the injected monotonic
clock. The caller must independently verify standstill and Nav2 cancellation.
There is no ROS publisher here and no claim of verified hardware stopping.
"""

import math
import time
from enum import Enum


class Source(str, Enum):
    NONE = 'NONE'
    LINE = 'LINE'
    NAV = 'NAV'


class VelocityGate:
    def __init__(self, *, max_linear, max_angular, command_timeout,
                 heartbeat_timeout, sensor_timeout, clock=time.monotonic):
        values = (max_linear, max_angular, command_timeout,
                  heartbeat_timeout, sensor_timeout)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('limits and timeouts must be finite and positive')
        self.max_linear, self.max_angular = max_linear, max_angular
        self.command_timeout = command_timeout
        self.heartbeat_timeout, self.sensor_timeout = heartbeat_timeout, sensor_timeout
        self.clock = clock
        self.source = Source.NONE
        self.generation = 0
        self.fault = None
        self.armed = False
        self._pending = Source.NONE
        self._cancel_required = False
        self._last_time = None
        self._heartbeat = self._sensor = None
        self._command = None
        self._sequence = -1

    def _trip(self, reason):
        self.fault = reason
        self.armed = False
        self._command = None

    def _now(self):
        now = self.clock()
        if not math.isfinite(now) or (
                self._last_time is not None and now < self._last_time):
            self._trip('invalid clock')
            return None
        self._last_time = now
        return now

    def _fresh(self, now):
        return (self._heartbeat is not None and self._sensor is not None
                and 0 <= now - self._heartbeat < self.heartbeat_timeout
                and 0 <= now - self._sensor < self.sensor_timeout)

    def _check(self, now):
        if now is None:
            return False
        if self.armed and not self._fresh(now):
            self._trip('stale heartbeat or sensor')
        return self.armed and self.fault is None

    def request_source(self, source):
        """Always revoke output, including for invalid requests; return epoch."""
        self._now()
        self._cancel_required = self._cancel_required or self.source == Source.NAV
        self.source = Source.NONE
        self.armed = False
        self._command = None
        self._sequence = -1
        self._heartbeat = self._sensor = None
        self.generation += 1
        try:
            self._pending = Source(source)
        except ValueError:
            self._pending = Source.NONE
            raise ValueError('unknown velocity source') from None
        return self.generation

    def confirm_source(self, generation, *, stopped, nav_cancelled):
        """Enable a requested owner only after external transition evidence."""
        if self._now() is None or generation != self.generation:
            return False
        if not stopped or (self._cancel_required and not nav_cancelled):
            return False
        self.source = self._pending
        self._cancel_required = False
        return self.source != Source.NONE

    def _observe(self, attribute, observed_at):
        now = self._now()
        self._check(now)
        if now is None:
            return False
        stamp = now if observed_at is None else observed_at
        previous = getattr(self, attribute)
        if (not math.isfinite(stamp) or stamp > now
                or (previous is not None and stamp <= previous)):
            return False
        setattr(self, attribute, stamp)
        return True

    def heartbeat(self, *, observed_at=None):
        return self._observe('_heartbeat', observed_at)

    def sensor_update(self, *, observed_at=None):
        return self._observe('_sensor', observed_at)

    def arm(self):
        """Explicitly clear a fault only with fresh observations and an owner."""
        now = self._now()
        if now is None or self.source == Source.NONE or not self._fresh(now):
            return False
        self.fault = None
        self.armed = True
        self._command = None
        return True

    def disarm(self):
        self.armed = False
        self._command = None

    def submit(self, source, generation, sequence, linear, angular, *,
               issued_at=None):
        """Accept strictly increasing command sequences in the current epoch."""
        now = self._now()
        if not self._check(now):
            return False
        if (source != self.source or source == Source.NONE
                or generation != self.generation
                or type(sequence) is not int or sequence <= self._sequence):
            return False
        if (not math.isfinite(linear) or not math.isfinite(angular)
                or abs(linear) > self.max_linear
                or abs(angular) > self.max_angular):
            self._trip('invalid velocity')
            return False
        stamp = now if issued_at is None else issued_at
        if not math.isfinite(stamp) or not 0 <= now - stamp < self.command_timeout:
            return False
        self._sequence = sequence
        self._command = (stamp, linear, angular)
        return True

    def output(self):
        now = self._now()
        if not self._check(now) or self._command is None:
            return (0.0, 0.0)
        stamp, linear, angular = self._command
        if now - stamp >= self.command_timeout:
            self._command = None
            return (0.0, 0.0)
        return (linear, angular)
