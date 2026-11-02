"""Display-only measured state. This is not a robot command or completion contract."""

import math
import time
from dataclasses import asdict, dataclass
from datetime import UTC, datetime

ROBOT_IDS = ("pinky1", "pinky2", "pinky3", "omx1", "omx2", "omx3")


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def text(value, maximum=128):
    return isinstance(value, str) and 0 < len(value) <= maximum


@dataclass(frozen=True)
class RobotSample:
    robot_id: str
    session_id: str
    sequence: int
    observed_at: str
    map_id: str
    frame_id: str = "map"
    pose: tuple[float, float, float, float] | None = (
        None  # x, y, z, yaw; metres/radians
    )
    joints: tuple[tuple[str, float], ...] = ()
    mode: str = "미수신"
    velocity: float | None = None

    def __post_init__(self):
        if self.robot_id not in ROBOT_IDS or not text(self.session_id):
            raise ValueError("unknown robot or invalid session")
        if type(self.sequence) is not int or not 0 <= self.sequence <= 2**53 - 1:
            raise ValueError("sequence must be a nonnegative safe integer")
        if not text(self.map_id) or self.frame_id != "map" or not text(self.mode):
            raise ValueError("map id, map frame and short mode are required")
        try:
            timestamp = datetime.fromisoformat(self.observed_at)
        except (ValueError, TypeError) as exc:
            raise ValueError("observed_at must be an ISO timestamp") from exc
        if timestamp.tzinfo is None:
            raise ValueError("observed_at requires timezone")
        if self.pose is not None and (
            not isinstance(self.pose, tuple)
            or len(self.pose) != 4
            or not all(finite(v) for v in self.pose)
            or any(abs(v) > 10000 for v in self.pose[:3])
            or abs(self.pose[3]) > 1000
        ):
            raise ValueError("pose requires finite metres and yaw in radians")
        if not isinstance(self.joints, tuple) or len(self.joints) > 16:
            raise ValueError("at most 16 named joints required")
        names = set()
        for pair in self.joints:
            if not isinstance(pair, tuple) or len(pair) != 2:
                raise ValueError("joint requires name and radians")
            name, angle = pair
            if not text(name) or name in names or not finite(angle) or abs(angle) > 1e6:
                raise ValueError("unique named finite joint values required")
            names.add(name)
        if self.velocity is not None and (
            not finite(self.velocity) or abs(self.velocity) > 100
        ):
            raise ValueError("invalid velocity")

    def to_dict(self):
        result = asdict(self)
        result["joints"] = dict(self.joints)
        return result


class StateCache:
    """GUI-thread-owned latest values. Reconnect explicitly resets accepted sessions."""

    def __init__(self, map_id, stale_seconds=2.0):
        if not text(map_id) or not finite(stale_seconds) or stale_seconds <= 0:
            raise ValueError("valid map and positive finite freshness limit required")
        self.map_id = map_id
        self.stale_seconds = stale_seconds
        self.samples = {}
        self.received = {}
        self.sessions = {}
        self.connected = False

    def begin_connection(self):
        # Preserve last positions, but require a fresh sample before declaring them current.
        self.connected = True
        self.sessions.clear()
        self.received.clear()

    def disconnect(self):
        self.connected = False

    def apply(self, sample: RobotSample, *, now=None, wall_time=None):
        if not isinstance(sample, RobotSample) or sample.map_id != self.map_id:
            return False
        wall_time = datetime.now(UTC) if wall_time is None else wall_time
        age = (wall_time - datetime.fromisoformat(sample.observed_at)).total_seconds()
        if not self.connected or age < -2 or age > self.stale_seconds:
            return False
        accepted_session = self.sessions.get(sample.robot_id)
        if accepted_session is not None and accepted_session != sample.session_id:
            return False
        previous = self.samples.get(sample.robot_id)
        if (
            sample.robot_id in self.received
            and previous is not None
            and sample.sequence <= previous.sequence
        ):
            return False
        self.sessions[sample.robot_id] = sample.session_id
        self.samples[sample.robot_id] = sample
        # Sensor age counts towards freshness; a delayed packet must not get a new full TTL.
        self.received[sample.robot_id] = (
            time.monotonic() if now is None else now
        ) - max(0, age)
        return True

    def snapshot(self, *, now=None):
        now = time.monotonic() if now is None else now
        result = []
        for robot_id in ROBOT_IDS:
            sample = self.samples.get(robot_id)
            received = self.received.get(robot_id)
            age = max(0, now - received) if received is not None else None
            row = (
                sample.to_dict()
                if sample
                else {
                    "robot_id": robot_id,
                    "pose": None,
                    "joints": {},
                    "mode": "미수신",
                }
            )
            row.update(
                age=age,
                stale=not self.connected or age is None or age >= self.stale_seconds,
            )
            result.append(row)
        return result
