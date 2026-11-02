"""MQTT task envelope v1.0. Device-specific payload checks belong to the executor."""

import json
import re
from datetime import UTC, datetime

MAX_MESSAGE_BYTES = 64 * 1024
_ID = re.compile(r"[A-Za-z0-9_.:-]{1,128}\Z")
_HEADERS = (
    "message_id",
    "type",
    "site_id",
    "source",
    "target",
    "task_id",
    "step_id",
    "command_id",
    "execution_id",
)


def topic(site: str, kind: str, device: str, channel: str) -> str:
    if not all(isinstance(v, str) and _ID.fullmatch(v) for v in (site, device)):
        raise ValueError("invalid topic identifier")
    if kind not in {"robot", "workcell"} or channel not in {"cmd", "event"}:
        raise ValueError("only task cmd/event channels are implemented")
    if kind == "robot" and device not in {"pinky1", "pinky2", "pinky3", "omx3"}:
        raise ValueError("OMX1/2 commands must use the joint workcell")
    if kind == "workcell" and device != "shelf-01":
        raise ValueError("unknown workcell")
    return f"roport/v1/{site}/{kind}/{device}/{channel}"


def timestamp(value: str) -> datetime:
    try:
        result = datetime.fromisoformat(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("timestamp must be an ISO 8601 string") from exc
    if result.tzinfo is None:
        raise ValueError("timestamp requires a UTC offset")
    return result


def validate(message: dict) -> None:
    if not isinstance(message, dict) or message.get("schema_version") != "1.0":
        raise ValueError("unsupported schema_version")
    for key in _HEADERS:
        value = message.get(key)
        if not isinstance(value, str) or not _ID.fullmatch(value):
            raise ValueError(f"invalid {key}")
    if not isinstance(message.get("payload"), dict):
        raise TypeError("payload must be an object")
    sent = timestamp(message.get("sent_at"))
    if (
        message["type"].endswith(".requested") or message["type"] == "depart.permitted"
    ) and timestamp(message.get("expires_at")) <= sent:
        raise ValueError("expires_at must follow sent_at")


def encode(message: dict) -> bytes:
    validate(message)
    raw = json.dumps(
        message, ensure_ascii=False, allow_nan=False, separators=(",", ":")
    ).encode()
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    return raw


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _invalid_number(value):
    raise ValueError("non-finite JSON number")


def decode(raw: bytes) -> dict:
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    try:
        message = json.loads(
            raw, object_pairs_hook=_unique_object, parse_constant=_invalid_number
        )
        validate(message)
        # Also rejects finite-looking JSON numbers that overflow to infinity.
        encode(message)
        return message
    except (TypeError, UnicodeError, RecursionError) as exc:
        raise ValueError("invalid JSON envelope") from exc


def check_command(
    message: dict, *, site: str, target: str, source: str, retained: bool
) -> None:
    validate(message)
    if (message["site_id"], message["target"], message["source"]) != (
        site,
        target,
        source,
    ):
        raise ValueError("routing_mismatch")
    if retained:
        raise ValueError("retained_command")
    if timestamp(message.get("expires_at")) <= timestamp(message["sent_at"]):
        raise ValueError("expires_at must follow sent_at")


def check_expiry(message: dict) -> None:
    """Only new execution expires; a recorded result remains queryable."""
    if timestamp(message.get("expires_at")) <= datetime.now(UTC):
        raise ValueError("expired_command")


class CommandHistory:
    """Single-consumer command/result cache, independent of any transport."""

    def __init__(self, capacity: int = 1024):
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.capacity = capacity
        # shortcut: memory only; add durable execution records before real hardware use.
        self._records: dict[str, tuple[str, list[dict]]] = {}

    @staticmethod
    def _fingerprint(message: dict) -> str:
        return json.dumps(
            {k: v for k, v in message.items() if k not in {"message_id", "sent_at"}},
            sort_keys=True,
            allow_nan=False,
        )

    def lookup(self, message: dict) -> list[dict] | None:
        previous = self._records.get(message["command_id"])
        if previous is not None:
            if previous[0] != self._fingerprint(message):
                raise ValueError("idempotency_conflict")
            return previous[1]
        if len(self._records) >= self.capacity:
            # Evicting old commands would allow a delayed duplicate to execute again.
            raise ValueError("command_history_full")
        return None

    def remember(self, message: dict, results: list[dict]) -> None:
        if self.lookup(message) is not None:
            raise ValueError("command already recorded")
        self._records[message["command_id"]] = (self._fingerprint(message), results)
