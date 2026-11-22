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
    if kind == "handover.receipt_recorded":
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
    }
    if (
        kind == "handover.receipt_recorded"
        and isinstance(payload, dict)
        and "follow_card" in payload
    ):
        required[kind].add("follow_card")
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
