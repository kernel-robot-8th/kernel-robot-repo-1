"""Explicit trial extension of the comm envelope; all fields live in payload.

SCHEMAS is also exported in the handoff kit. The validator deliberately supports
only the JSON Schema keywords used here, not arbitrary third-party schemas.
"""

import math
import os
import re
from datetime import UTC, datetime, timedelta
from uuid import uuid4

from .messages import encode, timestamp, validate
from .mqtt import MqttConfig

PROFILE = "roport-pinky-driving-trial-v1"
ID = {"type": "string", "pattern": r"^[A-Za-z0-9_.:-]{1,128}$"}
BOOL = {"type": "boolean"}
UINT = {"type": "integer", "minimum": 0, "maximum": 1000000}
NUMBER = {"type": "number", "minimum": -10000, "maximum": 10000}


def enum(*values):
    return {"enum": list(values)}


def obj(**properties):
    return {
        "type": "object",
        "properties": properties,
        "required": list(properties),
        "additionalProperties": False,
    }


POSE = obj(
    x=NUMBER, y=NUMBER, yaw={"type": "number", "minimum": -math.pi, "maximum": math.pi}
)
BINDING = {"binding_id": ID}
FENCE = {**BINDING, "expected_boot_id": ID}
STATE = {
    "node_id": {"type": ["string", "null"], "pattern": ID["pattern"]},
    "mode": enum("line", "nav2", "unknown"),
    "stopped": BOOL,
    "line_enabled": BOOL,
    "nav2_ready": BOOL,
    "localization_ready": BOOL,
    "drive_owner": enum("none", "line", "nav2"),
    "graph_version": ID,
    "map_id": ID,
    "map_version": ID,
    "heading_rad": POSE["properties"]["yaw"],
    "fault": {"type": ["string", "null"], "pattern": ID["pattern"]},
}
REPORT = {
    **BINDING,
    "boot_id": ID,
    "observed_at": {"type": "string"},
    "phase_revision": {"type": "integer", "minimum": 1, "maximum": 1000000},
    **STATE,
}
ROUTE_RESULT = {
    "seq": UINT,
    "total": UINT,
    "prev_node": STATE["node_id"],
    "next_node": STATE["node_id"],
}
SCHEMAS = {
    "status.requested": obj(**BINDING),
    "status.completed": obj(**REPORT),
    "route.requested": obj(
        **FENCE,
        graph_version=ID,
        start_node=ID,
        start_heading_rad=POSE["properties"]["yaw"],
        start_maneuver=enum("none", "turn_back", "back_out"),
        route={"type": "array", "items": ID, "minItems": 2, "maxItems": 64},
        goal=obj(node_id=ID, location_id=STATE["node_id"], on_arrival=enum("hold")),
        speed_limit_mps={"type": "number", "minimum": 0.01, "maximum": 0.2},
    ),
    "mode_switch.requested": obj(
        **FENCE,
        node_id=enum("N12"),
        from_mode=enum("line"),
        to_mode=enum("nav2"),
        map_id=ID,
        map_version=ID,
    ),
    "navigate.requested": obj(
        **FENCE,
        goal_id=ID,
        goal_version=ID,
        map_id=ID,
        map_version=ID,
        frame_id=enum("map"),
        pose=POSE,
        position_tolerance_m={"type": "number", "minimum": 0.01, "maximum": 0.5},
        yaw_tolerance_rad={"type": "number", "minimum": 0.01, "maximum": 0.5},
    ),
    "event.ack": obj(acked_message_id=ID, status=enum("stored")),
}
for operation in ("route", "mode_switch", "navigate"):
    SCHEMAS[f"{operation}.accepted"] = obj(**REPORT)
    for phase in ("failed", "rejected"):
        SCHEMAS[f"{operation}.{phase}"] = obj(**REPORT, error_code=ID)
SCHEMAS["route.progress"] = obj(**REPORT, **ROUTE_RESULT)
SCHEMAS["route.completed"] = obj(**REPORT, **ROUTE_RESULT)
SCHEMAS["mode_switch.completed"] = obj(**REPORT)
SCHEMAS["navigate.completed"] = obj(
    **REPORT,
    goal_id=ID,
    goal_version=ID,
    frame_id=enum("map"),
    pose=POSE,
    nav2_result=enum("SUCCEEDED"),
)


def _check(value, schema, path="payload"):
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError(f"{path}: invalid choice")
    kind = schema.get("type")
    if kind is None:
        return
    kinds = kind if isinstance(kind, list) else [kind]
    matches = {
        "object": type(value) is dict,
        "array": type(value) is list,
        "string": type(value) is str,
        "integer": type(value) is int,
        "number": type(value) in (int, float),
        "boolean": type(value) is bool,
        "null": value is None,
    }
    if not any(matches[k] for k in kinds):
        raise ValueError(f"{path}: wrong type")
    if value is None:
        return
    if type(value) in (int, float) and (
        not math.isfinite(value)
        or not schema.get("minimum", -math.inf)
        <= value
        <= schema.get("maximum", math.inf)
    ):
        raise ValueError(f"{path}: number outside range")
    if (
        type(value) is str
        and "pattern" in schema
        and not re.fullmatch(schema["pattern"], value)
    ):
        raise ValueError(f"{path}: invalid identifier")
    if type(value) is dict:
        if set(value) != set(schema["properties"]):
            raise ValueError(f"{path}: missing or unexpected fields")
        for key, item in value.items():
            _check(item, schema["properties"][key], f"{path}.{key}")
    if type(value) is list:
        if not schema["minItems"] <= len(value) <= schema["maxItems"]:
            raise ValueError(f"{path}: invalid length")
        for item in value:
            _check(item, schema["items"], path)


def validate_trial(message):
    validate(message)
    command = message["type"].endswith(".requested")
    keys = {
        "schema_version",
        "profile",
        "message_id",
        "type",
        "site_id",
        "source",
        "target",
        "task_id",
        "step_id",
        "command_id",
        "execution_id",
        "sent_at",
        "payload",
    }
    if command:
        keys.add("expires_at")
    if set(message) != keys or message.get("profile") != PROFILE:
        raise ValueError("invalid trial profile/envelope")
    schema = SCHEMAS.get(message["type"])
    if schema is None:
        raise ValueError("unsupported trial type")
    _check(message["payload"], schema)
    payload = message["payload"]
    if "observed_at" in payload:
        timestamp(payload["observed_at"])
    if message["type"] == "route.requested":
        if (
            payload["route"][0] != payload["start_node"]
            or payload["route"][-1] != payload["goal"]["node_id"]
        ):
            raise ValueError("route endpoints mismatch")
        if any(a == b for a, b in zip(payload["route"], payload["route"][1:])):
            raise ValueError("repeated adjacent route node")
    encode(message)


def fresh(message, maximum_age=3):
    now = datetime.now(UTC)
    for field in (
        message["sent_at"],
        message["payload"].get("observed_at", message["sent_at"]),
    ):
        if not -0.5 <= (now - timestamp(field)).total_seconds() <= maximum_age:
            raise ValueError("stale or future trial message")


def command(*, site, control, robot, task, step, kind, payload):
    now = datetime.now(UTC)
    result = {
        "schema_version": "1.0",
        "profile": PROFILE,
        "message_id": str(uuid4()),
        "type": f"{kind}.requested",
        "site_id": site,
        "source": control,
        "target": robot,
        "task_id": task,
        "step_id": step,
        "command_id": str(uuid4()),
        "execution_id": str(uuid4()),
        "sent_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=10)).isoformat(),
        "payload": payload,
    }
    validate_trial(result)
    return result


def reply(request, kind, payload):
    result = {
        k: request[k]
        for k in (
            "schema_version",
            "profile",
            "site_id",
            "task_id",
            "step_id",
            "command_id",
            "execution_id",
        )
    }
    result.update(
        message_id=str(uuid4()),
        type=kind,
        source=request["target"],
        target=request["source"],
        sent_at=datetime.now(UTC).isoformat(),
        payload=payload,
    )
    validate_trial(result)
    return result


def mqtt_config(client_id):
    return MqttConfig(
        host=os.environ.get("ROPORT_MQTT_HOST", "127.0.0.1"),
        client_id=client_id,
        port=int(os.environ.get("ROPORT_MQTT_PORT", "1883")),
        tls=os.environ.get("ROPORT_MQTT_TLS") == "1",
        ca_file=os.environ.get("ROPORT_MQTT_CA_FILE"),
        username=os.environ.get("ROPORT_MQTT_USERNAME"),
        password=os.environ.get("ROPORT_MQTT_PASSWORD"),
    )


def check_result(request, event):
    """Additional semantic checks shared by both ends; never infer physical success."""
    p, result = request["payload"], event["payload"]
    for key in ("site_id", "task_id", "step_id", "command_id", "execution_id"):
        if event[key] != request[key]:
            raise ValueError("event correlation mismatch")
    if (event["source"], event["target"]) != (request["target"], request["source"]):
        raise ValueError("event identity mismatch")
    kind = request["type"].split(".")[0]
    if (
        not event["type"].startswith(kind + ".")
        or result["binding_id"] != p["binding_id"]
    ):
        raise ValueError("event operation/binding mismatch")
    if "expected_boot_id" in p and result["boot_id"] != p["expected_boot_id"]:
        raise ValueError("robot restarted")
    phase = event["type"].split(".")[1]
    if phase in {"completed", "progress"} and result["fault"] is not None:
        raise ValueError("faulted result")
    if kind == "route" and phase in {"completed", "progress"}:
        route, index = p["route"], result["seq"]
        if (
            result["graph_version"] != p["graph_version"]
            or result["total"] != len(route)
            or index >= len(route)
            or result["node_id"] != route[index]
            or result["prev_node"] != (route[index - 1] if index else None)
            or result["next_node"]
            != (route[index + 1] if index + 1 < len(route) else None)
        ):
            raise ValueError("route progress mismatch")
        if phase == "completed" and (
            index != len(route) - 1
            or not result["stopped"]
            or result["line_enabled"]
            or result["mode"] != "line"
            or result["drive_owner"] not in {"none", "line"}
        ):
            raise ValueError("route completion requires goal stop and disabled line")
    if kind in {"mode_switch", "navigate"} and phase == "completed":
        if (
            result["mode"] != "nav2"
            or not result["stopped"]
            or result["line_enabled"]
            or not result["nav2_ready"]
            or not result["localization_ready"]
            or result["drive_owner"] != "nav2"
            or any(result[k] != p[k] for k in ("map_id", "map_version"))
        ):
            raise ValueError("Nav2 handoff/result not ready")
        if kind == "mode_switch" and result["node_id"] != "N12":
            raise ValueError("mode switch requires N12")
        if kind == "navigate":
            if any(result[k] != p[k] for k in ("goal_id", "goal_version", "frame_id")):
                raise ValueError("navigation goal mismatch")
            a, b = result["pose"], p["pose"]
            yaw_error = abs(
                math.atan2(math.sin(a["yaw"] - b["yaw"]), math.cos(a["yaw"] - b["yaw"]))
            )
            if (
                math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > p["position_tolerance_m"]
                or yaw_error > p["yaw_tolerance_rad"]
            ):
                raise ValueError("navigation pose outside goal tolerance")
