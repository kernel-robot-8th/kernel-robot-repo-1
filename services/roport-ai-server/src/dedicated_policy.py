"""Injected dedicated-arm policy candidates; no model loading or robot actuation."""

import json
import math
import time
from copy import deepcopy
from datetime import UTC, datetime

from observations import ImageNotReady, validate_images

SCHEMA_VERSION = "omx.dedicated/1"
MAX_DATAGRAM_BYTES = 60_000
BINDINGS = (
    "execution_id",
    "operation",
    "assigned_arm_id",
    "phase",
    "phase_revision",
    "policy_id",
    "model_version",
    "policy_bundle_version",
    "feature_schema_version",
    "calibration_version",
    "request_id",
    "observation_seq",
    "captured_at",
    "expires_at",
)
REGISTRY_FIELDS = (
    "operation",
    "assigned_arm_id",
    "phase",
    "model_version",
    "policy_bundle_version",
    "feature_schema_version",
    "calibration_version",
)
CONTEXT_FIELDS = ("execution_id", "phase_revision", "policy_id", *REGISTRY_FIELDS)
PHASES = {"GRASPING", "TRANSFERRING", "RELEASING", "RETURNING"}


class DatagramError(ValueError):
    """Malformed or oversized JSON datagram."""


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise DatagramError("duplicate JSON key")
        result[key] = value
    return result


def _invalid_constant(value):
    raise DatagramError("nonfinite JSON number")


def decode_datagram(data: bytes) -> dict:
    """Decode one complete datagram; fragments and duplicate keys are rejected."""
    if not isinstance(data, bytes) or not 0 < len(data) <= MAX_DATAGRAM_BYTES:
        raise DatagramError("invalid datagram size or type")
    try:
        value = json.loads(
            data.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_invalid_constant,
        )
        # JSON exponent overflow (1e999) also must not produce infinity.
        json.dumps(value, allow_nan=False)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise DatagramError("invalid finite JSON object") from exc
    if not isinstance(value, dict):
        raise DatagramError("datagram must contain an object")
    return value


def encode_datagram(message: dict) -> bytes:
    if not isinstance(message, dict):
        raise DatagramError("datagram must contain an object")
    try:
        data = json.dumps(message, allow_nan=False, separators=(",", ":")).encode()
    except (ValueError, TypeError, RecursionError, UnicodeError) as exc:
        raise DatagramError("message is not finite JSON") from exc
    if len(data) > MAX_DATAGRAM_BYTES:
        raise DatagramError("datagram exceeds size limit")
    return data


def _integer(value):
    return type(value) is int and value >= 0


def _number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _vector(value):
    return isinstance(value, list) and len(value) == 6 and all(map(_number, value))


def _features(value):
    return (
        isinstance(value, list)
        and len(value) == 6
        and all(isinstance(name, str) and name.strip() for name in value)
        and len(set(value)) == 6
    )


def _timestamp(value):
    if not isinstance(value, str) or "T" not in value:
        raise ValueError("timestamp must be ISO8601 with offset")
    parsed = datetime.fromisoformat(value)
    if parsed.utcoffset() is None:
        raise ValueError("timestamp requires UTC offset")
    return parsed.timestamp()


def _error(request, code):
    response = {
        "schema_version": SCHEMA_VERSION,
        "type": "inference.error",
        "code": code,
    }
    if isinstance(request, dict):
        response.update(
            {
                key: request[key]
                for key in BINDINGS
                if key in request and type(request[key]) in (str, int)
            }
        )
    return response


class PolicyRouter:
    """Serial caller API. Providers implement reset() and infer(observation).

    Metadata is keyed by policy_id and includes REGISTRY_FIELDS, six ordered
    feature_names and use_degrees. The provider owns image preprocessing.
    """

    def __init__(self, policies: dict, metadata: dict, *, max_sessions: int = 1024):
        if type(max_sessions) is not int or max_sessions < 1:
            raise ValueError("max_sessions must be a positive integer")
        self._max_sessions = max_sessions
        self.policies = dict(policies)
        self.metadata = deepcopy(metadata)
        for policy_id, entry in self.metadata.items():
            if (
                not isinstance(policy_id, str)
                or not policy_id.strip()
                or not isinstance(entry, dict)
                or any(
                    not isinstance(entry.get(key), str) or not entry[key].strip()
                    for key in REGISTRY_FIELDS
                )
                or entry["operation"] not in {"INBOUND", "OUTBOUND"}
                or entry["phase"] not in PHASES
                or not _features(entry.get("feature_names"))
                or type(entry.get("use_degrees")) is not bool
            ):
                raise ValueError("invalid policy metadata")
        self._active = {}
        self._retired = {}
        self._sessions = set()

    def handle_datagram(self, data: bytes, *, now: float) -> bytes:
        try:
            request = decode_datagram(data)
        except DatagramError:
            return encode_datagram(_error(None, "invalid_datagram"))
        response = self.handle(request, now=now)
        try:
            return encode_datagram(response)
        except DatagramError:
            return encode_datagram(_error(None, "response_too_large"))

    def handle(self, request: dict, *, now: float) -> dict:
        started = time.monotonic()
        try:
            # Direct callers obey the same finite JSON and size boundary as UDP.
            request = decode_datagram(encode_datagram(request))
        except DatagramError:
            return _error(request, "invalid_request")

        def error(code):
            return _error(request, code)

        if (
            request.get("schema_version") != SCHEMA_VERSION
            or request.get("type") not in ("policy.reset", "policy.infer")
            or any(key not in request for key in BINDINGS)
            or any(
                not isinstance(request[key], str) or not request[key].strip()
                for key in BINDINGS
                if key not in {"phase_revision", "observation_seq"}
            )
            or not _integer(request["phase_revision"])
            or not _integer(request["observation_seq"])
            or request["operation"] not in {"INBOUND", "OUTBOUND"}
            or request["phase"] not in PHASES
            or not _number(now)
        ):
            return error("invalid_request")
        try:
            captured = _timestamp(request["captured_at"])
            expires = _timestamp(request["expires_at"])
            # Compare capture time at ISO8601 wire precision; keep raw now for deadlines.
            wire_now = datetime.fromtimestamp(now, UTC).timestamp()
        except (ValueError, OverflowError, OSError):
            return error("invalid_timestamp")
        if captured > wire_now or now >= expires or captured >= expires:
            return error(
                "expired_observation"
                if now >= expires or captured >= expires
                else "future_observation"
            )
        policy_id = request["policy_id"]
        provider = self.policies.get(policy_id)
        metadata = self.metadata.get(policy_id)
        if (
            provider is None
            or metadata is None
            or not callable(getattr(provider, "reset", None))
            or not callable(getattr(provider, "infer", None))
        ):
            return error("model_not_configured")
        if any(request[key] != metadata[key] for key in REGISTRY_FIELDS):
            return error("metadata_mismatch")
        arm = request["assigned_arm_id"]
        execution = request["execution_id"]
        context = tuple(request[key] for key in CONTEXT_FIELDS)
        state = self._active.get(arm)
        if execution in self._retired.get(arm, set()):
            return error("stale_execution")
        response = {"schema_version": SCHEMA_VERSION}
        response.update({key: request[key] for key in BINDINGS})
        if request["type"] == "policy.reset":
            response["type"] = "policy.reset.ack"
            try:
                encode_datagram(response)
            except DatagramError:
                return error("response_too_large")
            if state is not None and state["execution"] == execution:
                if request["phase_revision"] < state["revision"]:
                    return error("stale_reset")
                if request["phase_revision"] == state["revision"]:
                    if context != state["context"]:
                        return error("reset_conflict")
                    if not state["ready"]:
                        return error("reset_required")
                    if now + time.monotonic() - started >= expires:
                        return error("deadline_exceeded")
                    return response
            if (arm, execution) not in self._sessions:
                if len(self._sessions) >= self._max_sessions:
                    return error("session_capacity_exceeded")
                # Tombstones are never evicted: an old execution must not reactivate.
                self._sessions.add((arm, execution))
            if state is not None and state["execution"] != execution:
                self._retired.setdefault(arm, set()).add(state["execution"])
            state = {
                "execution": execution,
                "revision": request["phase_revision"],
                "context": context,
                "observation_seq": request["observation_seq"] - 1,
                "action_seq": 0,
                "captured_at": captured,
                "ready": False,
            }
            self._active[arm] = state
            try:
                provider.reset()
            except Exception:  # noqa: BLE001 - provider failures must stay wire errors.
                return error("reset_failed")
            if now + time.monotonic() - started >= expires:
                return error("deadline_exceeded")
            state["ready"] = True
            return response
        if state is None or not state["ready"] or state["context"] != context:
            return error("reset_required")
        observation = request.get("observation")
        if (
            not isinstance(observation, dict)
            or not _vector(observation.get("positions"))
            or observation.get("feature_names") != metadata["feature_names"]
            or type(observation.get("use_degrees")) is not bool
            or observation["use_degrees"] != metadata["use_degrees"]
        ):
            return error("invalid_observation")
        if "cameras" in metadata:
            try:
                images = validate_images(observation.get("images"))
                if (
                    set(images) != set(metadata["cameras"])
                    or observation.get("captured_at") != request["captured_at"]
                ):
                    return error("invalid_observation")
            except (TypeError, ValueError):
                return error("invalid_observation")
        if request["observation_seq"] <= state["observation_seq"]:
            return error("stale_observation")
        if captured < state["captured_at"]:
            return error("stale_observation")
        state["observation_seq"] = request["observation_seq"]
        state["captured_at"] = captured
        try:
            actions = provider.infer(deepcopy(observation))
        except ImageNotReady:
            state["ready"] = False
            return error("input_not_ready")
        except Exception:  # noqa: BLE001 - provider failures must stay wire errors.
            state["ready"] = False
            return error("inference_failed")
        if now + time.monotonic() - started >= expires:
            state["ready"] = False
            return error("deadline_exceeded")
        if isinstance(actions, list) and len(actions) > MAX_DATAGRAM_BYTES // 14:
            state["ready"] = False
            return error("response_too_large")
        if (
            not isinstance(actions, list)
            or not actions
            or not all(map(_vector, actions))
        ):
            state["ready"] = False
            return error("invalid_actions")
        state["action_seq"] += 1
        response = {
            **response,
            "type": "action.candidate",
            "action_seq": state["action_seq"],
            "actions": deepcopy(actions),
        }
        try:
            encode_datagram(response)
        except DatagramError:
            state["ready"] = False
            return error("response_too_large")
        if now + time.monotonic() - started >= expires:
            state["ready"] = False
            return error("deadline_exceeded")
        return response
