"""Inbound envelope for the local WSS candidate; not the published S1 proposal."""

import re
from datetime import datetime


class ServiceStorageError(RuntimeError):
    """A service-store transaction failed; retry delivery without acknowledging it."""


def validate_service_message(message, kind):
    fields = {
        "schema_version",
        "message_id",
        "type",
        "site_id",
        "source",
        "target",
        "sent_at",
        "payload",
    }
    if kind in {"handover.receipt_recorded", "follow.control_requested"}:
        fields.add("task_id")
    if not isinstance(message, dict) or set(message) != fields:
        raise ValueError("invalid service envelope fields")
    if (
        message["schema_version"] != "1.0"
        or message["type"] != kind
        or (message["source"], message["target"]) != ("service-01", "control-01")
    ):
        raise ValueError("invalid service version, type or direction")
    for key in fields & {"message_id", "site_id", "task_id"}:
        if (
            not isinstance(message[key], str)
            or not message[key]
            or len(message[key]) > 128
        ):
            raise ValueError(f"invalid {key}")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", message["site_id"]):
        raise ValueError("invalid site_id")
    try:
        if datetime.fromisoformat(message["sent_at"]).tzinfo is None:
            raise ValueError("timezone required")
    except (TypeError, ValueError) as exc:
        raise ValueError("sent_at must be an ISO timestamp with timezone") from exc
    payload = message["payload"]
    required = {
        "task.requested": {
            "order_id",
            "reservation_id",
            "operation",
            "packages",
            "pickup_point_id",
            "pickup_at",
            "order_revision",
        },
        "handover.receipt_recorded": {
            "receipt_id",
            "reservation_id",
            "order_id",
            "package_ids",
        },
        "message.ack": {"message_id"},
        "follow.control_requested": {
            "receipt_id",
            "reservation_id",
            "order_id",
            "robot_id",
            "request_id",
            "expected_revision",
            "operation",
        },
    }
    if isinstance(payload, dict):
        optional = {
            "handover.receipt_recorded": {"follow_card", "follow_mode"},
            "follow.control_requested": {"frame_ref", "candidate_id", "box"},
        }.get(kind, set())
        if kind in required:
            required[kind] |= payload.keys() & optional
    if (
        kind not in required
        or not isinstance(payload, dict)
        or set(payload) != required[kind]
    ):
        raise ValueError("invalid service payload fields")
    if kind == "message.ack" and (
        not isinstance(payload["message_id"], str) or not payload["message_id"]
    ):
        raise ValueError("ACK message_id required")
    if kind == "handover.receipt_recorded":
        validate_follow_mode(payload.get("follow_mode"), payload.get("follow_card"))
    if kind == "follow.control_requested":
        validate_follow_request(payload)


def validate_follow_mode(mode, card):
    mode = mode if mode is not None else "marker" if card is not None else "none"
    if mode not in {"none", "marker", "appearance"}:
        raise ValueError("invalid follow mode")
    if mode == "marker":
        if type(card) is not int or not 35 <= card <= 49:
            raise ValueError("invalid follow card")
    elif card is not None:
        raise ValueError("follow card requires marker mode")
    return mode


def validate_follow_request(payload):
    for field in ("receipt_id", "reservation_id", "order_id", "robot_id", "request_id"):
        if (
            not isinstance(payload.get(field), str)
            or not 1 <= len(payload[field]) <= 128
        ):
            raise ValueError(f"invalid {field}")
    if (
        type(payload.get("expected_revision")) is not int
        or payload["expected_revision"] < 1
    ):
        raise ValueError("invalid follow revision")
    operation = payload.get("operation")
    if operation not in {"capture", "select", "release", "pause", "resume", "return"}:
        raise ValueError("invalid follow operation")
    selected = payload.keys() & {"frame_ref", "candidate_id", "box"}
    if operation != "select":
        if selected:
            raise ValueError("selection fields require select")
        return
    if selected not in ({"frame_ref", "candidate_id"}, {"frame_ref", "box"}):
        raise ValueError("select requires frame and one candidate or box")
    for field in selected & {"frame_ref", "candidate_id"}:
        if not isinstance(payload[field], str) or not 1 <= len(payload[field]) <= 128:
            raise ValueError(f"invalid {field}")
    if "box" in selected:
        box = payload["box"]
        if (
            not isinstance(box, list)
            or len(box) != 4
            or any(type(n) not in {int, float} or not 0 <= n <= 1 for n in box)
            or not box[0] < box[2]
            or not box[1] < box[3]
        ):
            raise ValueError("invalid normalized selection box")
