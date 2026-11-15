"""Single-site MQTT contract. This file is portable with the OMX endpoint kit."""

import json
import math
import re
from datetime import UTC, datetime
from uuid import uuid4

SCHEMA_VERSION = "2.0"
PROFILE = "roport-control-v2"
MAX_MESSAGE_BYTES = 65536
ROBOTS = {"pinky1", "pinky2", "pinky3", "omx1", "omx2", "omx3"}
COMMANDS = {
    "line_route_command",
    "set_drive_mode_command",
    "nav_goal_command",
    "shelf_transfer_command",
    "arm_action_command",
    "warehouse_load_command",
    "robot_stop_command",
}
TYPES = COMMANDS | {
    "robot_state_request",
    "robot_state",
    "command_status",
    "robot_pose",
    "event_stored",
}
ID = {"type": "string", "pattern": r"[A-Za-z0-9_.:-]{1,128}"}
NULL_ID = {**ID, "nullable": True}
BOOL = {"type": "boolean"}
NUMBER = {"type": "number", "minimum": -10000, "maximum": 10000}
SEQ = {"type": "integer", "minimum": 0, "maximum": 2**53 - 1}


def choice(*values):
    return {"enum": values}


def obj(required=None, **properties):
    return {
        "type": "object",
        "properties": properties,
        "required": list(properties) if required is None else required,
    }


def array(item, minimum=1, maximum=64):
    return {"type": "array", "items": item, "minimum": minimum, "maximum": maximum}


POSE = obj(
    x=NUMBER, y=NUMBER, yaw={"type": "number", "minimum": -math.pi, "maximum": math.pi}
)
ARM = {
    "stopped": BOOL,
    "ready": BOOL,
    "home_reached": BOOL,
    "holding_state": choice("empty", "holding", "unknown"),
    "holding_package_id": NULL_ID,
    "fault": NULL_ID,
}
PINKY_STATE = obj(
    node=NULL_ID,
    arrival_command_id=NULL_ID,
    mode=choice("line", "nav2", "unknown"),
    stopped=BOOL,
    line_enabled=BOOL,
    nav2_ready=BOOL,
    localization_ready=BOOL,
    drive_owner=choice("none", "line", "nav2"),
    graph_version=ID,
    map_version=ID,
    heading_rad={**POSE["properties"]["yaw"], "nullable": True},
    fault=NULL_ID,
)
ARM_STATE = obj(**ARM)
DOCKING = obj(
    pinky_id=choice("pinky1", "pinky2", "pinky3"),
    pinky_boot_version=ID,
    arrival_command_id=NULL_ID,
    observed_at={"type": "string"},
    alignment=choice("confirmed", "unconfirmed", "unknown"),
)
WORKCELL_STATE = obj(
    stopped=BOOL,
    ready=BOOL,
    fault=NULL_ID,
    arms=array(
        obj(
            robot_id=choice("omx1", "omx2"),
            boot_version=ID,
            active_command_id=NULL_ID,
            **ARM,
        ),
        maximum=2,
    ),
    docking={**DOCKING, "nullable": True},
)
FENCE = {"expected_boot_version": ID}
REPORT = {"boot_version": ID, "observed_at": {"type": "string"}, "sequence": SEQ}
LOCATION = obj(
    required=["kind"],
    kind=choice("pinky", "shelf"),
    robot_id=ID,
    position=ID,
    slot_id=ID,
)
TRANSFER = {
    "package_id": ID,
    "arrival_command_id": ID,
    "expected_pinky_boot_version": ID,
}
SCHEMAS = {
    "robot_state_request": obj(
        required=[], expected_boot_version=ID, keep_running_command_id=ID
    ),
    "line_route_command": obj(
        required=[
            *FENCE,
            "graph_version",
            "start_node",
            "start_heading_rad",
            "start_maneuver",
            "route",
            "goal",
            "speed_limit_mps",
        ],
        **FENCE,
        graph_version=ID,
        start_node=ID,
        start_heading_rad=POSE["properties"]["yaw"],
        start_maneuver=choice("none"),
        route=array(ID, 2),
        goal=obj(node=ID, on_arrival=choice("hold")),
        speed_limit_mps={"type": "number", "minimum": 0.015, "maximum": 0.08},
        departure=obj(arrival_command_id=ID, transfer_command_id=ID),
    ),
    "set_drive_mode_command": obj(
        **FENCE,
        node=choice("N12"),
        from_mode=choice("line", "nav2"),
        to_mode=choice("line", "nav2"),
        graph_version=ID,
        map_version=ID,
    ),
    "nav_goal_command": obj(
        required=[
            *FENCE,
            "map_version",
            "frame_id",
            "pose",
            "position_tolerance_m",
            "yaw_tolerance_rad",
        ],
        **FENCE,
        map_version=ID,
        frame_id=choice("map"),
        pose=POSE,
        goal_id=ID,
        position_tolerance_m={"type": "number", "minimum": 0.01, "maximum": 0.5},
        yaw_tolerance_rad={"type": "number", "minimum": 0.01, "maximum": 0.5},
    ),
    "shelf_transfer_command": obj(
        **FENCE,
        **TRANSFER,
        operation=choice("store", "retrieve"),
        slot_id=ID,
        pinky_id=choice("pinky1", "pinky2", "pinky3"),
        arms=array(obj(robot_id=choice("omx1", "omx2"), **FENCE), 2, 2),
        model_version=ID,
        calibration_version=ID,
    ),
    "arm_action_command": obj(
        required=[*FENCE, "operation", "workcell_id", "calibration_version"],
        **FENCE,
        operation=choice("transfer", "return_home"),
        workcell_id=choice("shelf-01"),
        calibration_version=ID,
        **TRANSFER,
        source_location=LOCATION,
        destination_location=LOCATION,
        model_version=ID,
    ),
    "warehouse_load_command": obj(
        **FENCE,
        **TRANSFER,
        pickup_location_id=ID,
        pinky_id=choice("pinky1", "pinky2", "pinky3"),
        load_position=ID,
        model_version=ID,
        calibration_version=ID,
    ),
    "robot_stop_command": obj(**FENCE, target_command_id=ID, reason=ID),
    "robot_state": obj(**REPORT, active_command_id=NULL_ID, state={"type": "object"}),
    "command_status": obj(
        required=[*REPORT, "status"],
        **REPORT,
        status=choice(
            "accepted", "in_progress", "completed", "rejected", "failed", "interrupted"
        ),
        reason=ID,
        result={"type": "object"},
    ),
    "robot_pose": obj(
        **REPORT,
        map_version=ID,
        frame_id=choice("map", "odom"),
        pose_source=ID,
        pose={**POSE, "nullable": True},
    ),
    "event_stored": obj(acked_message_id=ID),
}


def timestamp(value):
    if not isinstance(value, str):
        raise ValueError("timestamp must be text")  # noqa: TRY004 - invalid wire data uses ValueError
    try:
        result = datetime.fromisoformat(value)
    except ValueError as exc:
        raise ValueError("invalid timestamp") from exc
    if result.tzinfo is None:
        raise ValueError("timestamp requires offset")
    return result


def _check(value, schema, path="payload"):
    if value is None and schema.get("nullable"):
        return
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError(f"{path}: invalid choice")
    kind = schema.get("type")
    if kind is None:
        return
    expected = {
        "string": str,
        "boolean": bool,
        "object": dict,
        "array": list,
        "integer": int,
    }
    if kind == "number":
        valid = type(value) in (int, float) and math.isfinite(value)
    else:
        valid = type(value) is expected[kind]
    if not valid:
        raise ValueError(f"{path}: wrong type")
    if kind in {"number", "integer"} and not schema.get(
        "minimum", -math.inf
    ) <= value <= schema.get("maximum", math.inf):
        raise ValueError(f"{path}: outside range")
    if (
        kind == "string"
        and "pattern" in schema
        and not re.fullmatch(schema["pattern"], value)
    ):
        raise ValueError(f"{path}: invalid identifier")
    if kind == "object" and "properties" in schema:
        if (
            not set(schema["required"]) <= value.keys()
            or not value.keys() <= schema["properties"].keys()
        ):
            raise ValueError(f"{path}: missing/unexpected fields")
        for key, item in value.items():
            _check(item, schema["properties"][key], f"{path}.{key}")
    if kind == "array":
        if not schema["minimum"] <= len(value) <= schema["maximum"]:
            raise ValueError(f"{path}: invalid length")
        for item in value:
            _check(item, schema["items"], path)


def topic(target, channel):
    if target not in ROBOTS | {"shelf-01"} or channel not in {
        "cmd",
        "event",
        "ack",
        "motion",
    }:
        raise ValueError("unknown topic endpoint/channel")
    if channel == "motion" and not target.startswith("pinky"):
        raise ValueError("motion is Pinky only")
    return (
        f"roport/{'workcell' if target == 'shelf-01' else 'robot'}/{target}/{channel}"
    )


def validate(message):
    if (
        not isinstance(message, dict)
        or message.get("schema_version") != SCHEMA_VERSION
        or message.get("profile") != PROFILE
    ):
        raise ValueError("unsupported field MQTT contract")
    kind = message.get("type")
    if kind not in TYPES:
        raise ValueError("unsupported field message type")
    required = {
        "schema_version",
        "profile",
        "message_id",
        "type",
        "source",
        "target",
        "sent_at",
        "payload",
    }
    if kind in COMMANDS | {"command_status"}:
        required.add("command_id")
    keep = kind == "robot_state_request" and bool(message.get("payload"))
    if kind in COMMANDS or keep:
        required.add("expires_at")
    if message.keys() != required:
        raise ValueError("missing/unexpected envelope fields")
    for key in ("message_id", "source", "target") + (
        ("command_id",) if "command_id" in required else ()
    ):
        _check(message[key], ID, key)
    sent = timestamp(message["sent_at"])
    if "expires_at" in required and timestamp(message["expires_at"]) <= sent:
        raise ValueError("expiry must follow sent_at")
    p = message["payload"]
    _check(p, SCHEMAS[kind])
    if "observed_at" in p:
        timestamp(p["observed_at"])
    endpoint = (
        message["source"]
        if kind in {"robot_state", "robot_pose", "command_status"}
        else message["target"]
    )
    if endpoint not in ROBOTS | {"shelf-01"}:
        raise ValueError("unknown endpoint")
    if (
        kind == "robot_state_request"
        and keep
        and set(p) != {"expected_boot_version", "keep_running_command_id"}
    ):
        raise ValueError("keep-running requires boot and command")
    if kind == "robot_state":
        _check(
            p["state"],
            PINKY_STATE
            if endpoint.startswith("pinky")
            else WORKCELL_STATE
            if endpoint == "shelf-01"
            else ARM_STATE,
        )
        dock = p["state"].get("docking")
        if dock:
            timestamp(dock["observed_at"])
    if kind == "line_route_command" and (
        not endpoint.startswith("pinky")
        or p["route"][0] != p["start_node"]
        or p["route"][-1] != p["goal"]["node"]
    ):
        raise ValueError("route endpoints mismatch")
    if kind == "set_drive_mode_command" and (
        not endpoint.startswith("pinky") or p["from_mode"] == p["to_mode"]
    ):
        raise ValueError("invalid mode transition")
    if kind in {"nav_goal_command", "robot_pose"} and not endpoint.startswith("pinky"):
        raise ValueError("navigation and motion require Pinky")
    if kind == "shelf_transfer_command" and (
        endpoint != "shelf-01" or {a["robot_id"] for a in p["arms"]} != {"omx1", "omx2"}
    ):
        raise ValueError("cooperative task requires both arms")
    if kind == "arm_action_command":
        needed = set(FENCE) | {"operation", "workcell_id", "calibration_version"}
        if p["operation"] == "transfer":
            needed |= set(TRANSFER) | {
                "source_location",
                "destination_location",
                "model_version",
            }
        if endpoint not in {"omx1", "omx2"} or set(p) != needed:
            raise ValueError("invalid individual arm fields")
        for key in ("source_location", "destination_location"):
            if key in p:
                location = p[key]
                fields = (
                    {"kind", "robot_id", "position"}
                    if location["kind"] == "pinky"
                    else {"kind", "slot_id"}
                )
                if set(location) != fields:
                    raise ValueError("invalid location")
    if kind == "warehouse_load_command" and endpoint != "omx3":
        raise ValueError("warehouse load requires OMX3")
    if kind == "command_status":
        if p["status"] in {"failed", "rejected", "interrupted"} and "reason" not in p:
            raise ValueError("failure requires reason")
        if p["status"] == "completed" and "result" not in p:
            raise ValueError("completion requires observed result")
    raw = json.dumps(message, allow_nan=False, separators=(",", ":")).encode()
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")


def encode(message):
    validate(message)
    return json.dumps(message, allow_nan=False, separators=(",", ":")).encode()


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def decode(raw):
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    try:
        message = json.loads(raw, object_pairs_hook=_unique)
        validate(message)
        return message
    except (TypeError, UnicodeError, RecursionError) as exc:
        raise ValueError("invalid field message") from exc


def build_message(type, source, target, payload, command_id=None, expires_at=None):
    message = {
        "schema_version": SCHEMA_VERSION,
        "profile": PROFILE,
        "message_id": str(uuid4()),
        "type": type,
        "source": source,
        "target": target,
        "sent_at": datetime.now(UTC).isoformat(),
        "payload": payload,
    }
    if command_id is not None:
        message["command_id"] = command_id
    if expires_at is not None:
        message["expires_at"] = expires_at
    validate(message)
    return message


def json_schema():
    """Export the same shape rules for kits/docs; validate() also enforces semantics."""

    def convert(schema):
        row = {k: v for k, v in schema.items() if k != "nullable"}
        if row.get("type") == "object" and "properties" in row:
            row["properties"] = {k: convert(v) for k, v in row["properties"].items()}
            row["additionalProperties"] = False
        if row.get("type") == "array":
            row["items"] = convert(row["items"])
            row["minItems"], row["maxItems"] = row.pop("minimum"), row.pop("maximum")
        if "pattern" in row:
            row["pattern"] = "^" + row["pattern"] + "$"
        if schema.get("nullable"):
            return {"anyOf": [row, {"type": "null"}]}
        return row

    variants = []
    for kind, payload in SCHEMAS.items():
        p = convert(payload)
        if kind == "robot_state":
            p["properties"]["state"] = {
                "oneOf": [convert(s) for s in (PINKY_STATE, ARM_STATE, WORKCELL_STATE)]
            }
        props = {k: convert(ID) for k in ("message_id", "source", "target")}
        props.update(
            schema_version={"const": SCHEMA_VERSION},
            profile={"const": PROFILE},
            type={"const": kind},
            sent_at={"type": "string", "format": "date-time"},
            payload=p,
        )
        if kind in COMMANDS | {"command_status"}:
            props["command_id"] = convert(ID)
        if kind in COMMANDS:
            props["expires_at"] = {"type": "string", "format": "date-time"}
        variant = {
            "type": "object",
            "required": list(props),
            "properties": props,
            "additionalProperties": False,
        }
        if kind == "robot_state_request":
            props["expires_at"] = {"type": "string", "format": "date-time"}
            variant["oneOf"] = [
                {
                    "properties": {"payload": {"maxProperties": 0}},
                    "not": {"required": ["expires_at"]},
                },
                {
                    "required": ["expires_at"],
                    "properties": {
                        "payload": {
                            "required": [
                                "expected_boot_version",
                                "keep_running_command_id",
                            ]
                        }
                    },
                },
            ]
        variants.append(variant)
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": PROFILE,
        "oneOf": variants,
    }
