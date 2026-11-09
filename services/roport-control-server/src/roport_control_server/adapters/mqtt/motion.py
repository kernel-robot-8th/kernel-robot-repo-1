"""Display-only motion samples, compatible with roport-state-dev-v1; no DB/task ACK."""

import json
from copy import deepcopy
from datetime import UTC, datetime
from queue import Empty
from time import monotonic
from uuid import UUID, uuid4

from .messages import _invalid_number, _unique_object, timestamp
from .trial_wire import ID, POSE, _check, enum, obj

PROFILE = "roport-state-dev-v1"
MAX_AGE = 1.0
DISTANCE = {"type": "number", "minimum": -1000, "maximum": 1000}
SPEED = {"type": "number", "minimum": -20, "maximum": 20}
MOTION = obj(
    binding_id=ID,
    boot_id=ID,
    sequence={"type": "integer", "minimum": 0, "maximum": 2**53 - 1},
    observed_at={"type": "string"},
    map_id=ID,
    map_version=ID,
    frame_id=enum("map", "odom"),
    pose_source=ID,
    pose={
        **obj(x=DISTANCE, y=DISTANCE, z=DISTANCE, yaw=POSE["properties"]["yaw"]),
        "type": ["object", "null"],
    },
    velocity={
        **obj(linear_x=SPEED, linear_y=SPEED, angular_z=SPEED),
        "type": ["object", "null"],
    },
    joints=obj(),
)
ENVELOPE = obj(
    schema_version=enum("1.1"),
    profile=enum(PROFILE),
    message_id=ID,
    type=enum("motion"),
    site_id={"type": "string", "pattern": r"^[A-Za-z0-9_-]{1,32}$"},
    source=enum("pinky1", "pinky2", "pinky3"),
    target=enum("control"),
    sent_at={"type": "string"},
    payload=MOTION,
)


def motion_topic(site, robot):
    return f"roport/v1/{site}/robot/{robot}/motion"


def encode_motion(destination, message):
    _check(message, ENVELOPE)
    if destination != motion_topic(message["site_id"], message["source"]):
        raise ValueError("motion topic mismatch")
    p = message["payload"]
    for key in ("binding_id", "boot_id"):
        if str(UUID(p[key])) != p[key]:
            raise ValueError("motion identity requires UUID")
    timestamp(message["sent_at"])
    timestamp(p["observed_at"])
    raw = json.dumps(message, allow_nan=False, separators=(",", ":")).encode()
    if len(raw) > 16384:
        raise ValueError("motion sample too large")
    return raw


def decode_motion(delivery):
    if delivery.qos != 0 or delivery.retained or len(delivery.payload) > 16384:
        raise ValueError("motion requires bounded QoS0/non-retained samples")
    try:
        message = json.loads(
            delivery.payload,
            object_pairs_hook=_unique_object,
            parse_constant=_invalid_number,
        )
        encode_motion(delivery.topic, message)
        return message
    except (TypeError, UnicodeError, RecursionError, AttributeError) as exc:
        raise ValueError("invalid motion sample") from exc


def motion_message(site, robot, binding, boot, sequence, observation):
    return {
        "schema_version": "1.1",
        "profile": PROFILE,
        "message_id": str(uuid4()),
        "type": "motion",
        "site_id": site,
        "source": robot,
        "target": "control",
        "sent_at": datetime.now(UTC).isoformat(),
        "payload": {
            **observation,
            "binding_id": binding,
            "boot_id": boot,
            "sequence": sequence,
        },
    }


class MotionReceiver:
    """Separate bounded transport: display loss never consumes task/event capacity."""

    def __init__(self, config, transport):
        self.config, self.transport = config, transport
        self.bindings = {
            r["robot_id"]: r["binding_id"] for r in config["robots"].values()
        }
        self.samples, self.sequences = {}, {}
        self.rejected = 0

    def consume(self, delivery, statuses):
        m = decode_motion(delivery)
        robot, p = m["source"], m["payload"]
        if (
            m["site_id"] != self.config["site_id"]
            or self.bindings.get(robot) != p["binding_id"]
            or statuses.get(robot, {}).get("boot_id") != p["boot_id"]
        ):
            raise ValueError("unconfirmed motion identity")
        now = datetime.now(UTC)
        age = (now - timestamp(p["observed_at"])).total_seconds()
        if (
            not -0.5 <= age <= MAX_AGE
            or not -0.5 <= (now - timestamp(m["sent_at"])).total_seconds() <= MAX_AGE
        ):
            raise ValueError("expired motion sample")
        previous = self.sequences.get(robot)
        if (
            previous
            and previous[0] == p["boot_id"]
            and (
                p["sequence"] <= previous[1]
                or timestamp(p["observed_at"]) < previous[2]
            )
        ):
            raise ValueError("out-of-order motion sample")
        self.sequences[robot] = (
            p["boot_id"],
            p["sequence"],
            timestamp(p["observed_at"]),
        )
        valid = (
            p["frame_id"] == "map"
            and p["map_id"] == self.config["map_id"]
            and p["map_version"] == self.config["map_version"]
            and p["pose"] is not None
        )
        if valid:
            self.samples[robot] = {
                "payload": p,
                "received": monotonic(),
                "age_at_receive": max(0, age),
                "valid": True,
            }
        elif robot in self.samples:
            self.samples[robot]["valid"] = False

    def poll(self, statuses):
        for _ in range(32):
            try:
                delivery = self.transport.receive(0)
            except (Empty, ConnectionError):
                break
            try:
                self.consume(delivery, statuses)
            except ValueError:
                self.rejected += 1

    def snapshot(self, statuses):
        result = {}
        for robot, sample in self.samples.items():
            p = sample["payload"]
            if statuses.get(robot, {}).get("boot_id") != p["boot_id"]:
                continue
            age = sample["age_at_receive"] + monotonic() - sample["received"]
            result[robot] = {
                **deepcopy(p),
                "age": age,
                "stale": age > MAX_AGE
                or not sample["valid"]
                or not self.transport.ready,
            }
        return result
