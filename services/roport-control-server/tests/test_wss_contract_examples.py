"""interfaces/wss: published examples satisfy the AsyncAPI schemas (S1 contract).

This guards the contract files only. The codec in adapters/wss and the Spring side must
reuse the same examples and the request_sha256 vector; they are separate work (CTL-1,
SVC-1).
"""

import hashlib
import json
import re
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

INTERFACES = Path(__file__).resolve().parents[3] / "interfaces/wss"
SPEC = yaml.safe_load((INTERFACES / "roport-wss.yaml").read_text())
ROWS = json.loads((INTERFACES / "examples.json").read_text())
S1_TYPES = {
    "session.opened",
    "message.ack",
    "task.requested",
    "task.rejected",
    "command.conflict",
}
DIRECTION = {
    "session.opened": ("service", "control"),
    "task.requested": ("service", "control"),
    "task.rejected": ("control", "service"),
    "command.conflict": ("control", "service"),
}
MAX_FRAME_BYTES = 64 * 1024


def check(schema, value, path="$"):
    """Validate the JSON Schema subset used by roport-wss.yaml. Returns error strings."""
    if "$ref" in schema:
        node = SPEC
        for part in schema["$ref"].removeprefix("#/").split("/"):
            node = node[part]
        return check(node, value, path)
    errors = []
    for sub in schema.get("allOf", []):
        errors += check(sub, value, path)
    kind = schema.get("type")
    kinds = {
        "object": dict,
        "string": str,
        "integer": int,
        "array": list,
    }
    if kind and (not isinstance(value, kinds[kind]) or isinstance(value, bool)):
        return errors + [f"{path}: expected {kind}"]
    if "const" in schema and value != schema["const"]:
        errors.append(f"{path}: expected const {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        errors.append(f"{path}: {value!r} not in enum")
    if isinstance(value, str):
        if "pattern" in schema and not re.fullmatch(schema["pattern"], value):
            errors.append(f"{path}: pattern mismatch")
        if len(value) > schema.get("maxLength", 10**9):
            errors.append(f"{path}: too long")
    is_int = isinstance(value, int) and not isinstance(value, bool)
    if is_int and value < schema.get("minimum", value):
        errors.append(f"{path}: below minimum")
    if isinstance(value, list):
        if len(value) < schema.get("minItems", 0):
            errors.append(f"{path}: too few items")
        if "items" in schema:
            for i, item in enumerate(value):
                errors += check(schema["items"], item, f"{path}[{i}]")
    if isinstance(value, dict):
        props = schema.get("properties", {})
        for name in schema.get("required", []):
            if name not in value:
                errors.append(f"{path}.{name}: required")
        if schema.get("additionalProperties") is False:
            for name in value.keys() - props.keys():
                errors.append(f"{path}.{name}: not allowed")
        for name, sub in props.items():
            if name in value:
                errors += check(sub, value[name], f"{path}.{name}")
    return errors


def validate(message):
    """Whole-frame check: envelope + per-type schema + cross-field rules."""
    type_ = message.get("type")
    spec = SPEC["components"]["messages"].get(type_)
    if spec is None:
        return [f"unknown type {type_!r}"]
    errors = check(spec["payload"], message)
    allowed = set(spec["x-envelope-fields"])
    errors += [f"$.{k}: not allowed for {type_}" for k in message.keys() - allowed]
    if type_ in DIRECTION:
        want = DIRECTION[type_]
        if (message.get("source"), message.get("target")) != want:
            errors.append("direction mismatch")
    payload = message.get("payload", {})
    if type_ == "message.ack":
        rejected = payload.get("status") == "REJECTED"
        if rejected != ("error_code" in payload):
            errors.append("$.payload.error_code: required if and only if REJECTED")
    if "expires_at" in message and message["expires_at"] <= message.get("sent_at", ""):
        errors.append("expires_at must be after sent_at")
    if type_ == "task.requested":
        ids = [p.get("package_id") for p in payload.get("packages", [])]
        if len(ids) != len(set(ids)):
            errors.append("duplicate package_id")
    if len(json.dumps(message, ensure_ascii=False).encode()) > MAX_FRAME_BYTES:
        errors.append("frame over 64KiB")
    return errors


def request_sha256(message):
    """Business hash of a request: only these fields, sorted keys, no whitespace."""
    p = message["payload"]
    fields = {
        "site_id": message["site_id"],
        "type": message["type"],
        "command_id": message["command_id"],
        "pickup_id": p["pickup_id"],
        "order_id": p["order_id"],
        "order_revision": p["order_revision"],
        "pickup_location_id": p["pickup_location_id"],
        "packages": sorted(p["packages"], key=lambda x: x["package_id"]),
    }
    text = json.dumps(fields, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(text.encode()).hexdigest()


def message_of(type_, index=0):
    return deepcopy(
        [r["message"] for r in ROWS if r["message"]["type"] == type_][index]
    )


def test_every_s1_type_has_a_valid_example():
    assert {r["message"]["type"] for r in ROWS} == S1_TYPES
    for row in ROWS:
        assert validate(row["message"]) == [], row["label"]


def test_spec_declares_exactly_the_s1_types():
    assert SPEC["asyncapi"] == "3.0.0"
    assert set(SPEC["components"]["messages"]) == S1_TYPES
    assert SPEC["info"]["version"] == "1.0"


def test_each_example_is_a_single_small_json_frame():
    for row in ROWS:
        frame = json.dumps(row["message"], ensure_ascii=False)
        assert json.loads(frame) == row["message"]
        assert len(frame.encode()) <= MAX_FRAME_BYTES


@pytest.mark.parametrize(
    "type_, mutate",
    [
        ("task.requested", lambda m: m.update(schema_version="1.1")),
        ("task.requested", lambda m: m.pop("expires_at")),
        ("task.requested", lambda m: m.pop("command_id")),
        ("task.requested", lambda m: m.update(site_id="site.a")),
        ("task.requested", lambda m: m.update(site_id="a/b")),
        ("task.requested", lambda m: m.update(message_id="bad id")),
        ("task.requested", lambda m: m.update(extra="x")),
        ("task.requested", lambda m: m.update(sent_at="2026-11-14T03:00:00+09:00")),
        ("task.requested", lambda m: m.update(expires_at=m["sent_at"])),
        ("task.requested", lambda m: m.update(source="control", target="service")),
        ("task.requested", lambda m: m["payload"].update(packages=[])),
        ("task.requested", lambda m: m["payload"].update(order_revision=-1)),
        ("task.requested", lambda m: m["payload"].update(extra="x")),
        (
            "task.requested",
            lambda m: m["payload"]["packages"].append(
                deepcopy(m["payload"]["packages"][0])
            ),
        ),
        (
            "task.rejected",
            lambda m: m["payload"].update(reason_code="IDEMPOTENCY_CONFLICT"),
        ),
        ("task.rejected", lambda m: m.pop("in_reply_to_message_id")),
        ("task.rejected", lambda m: m.update(target="control")),
        ("command.conflict", lambda m: m["payload"].pop("original_message_id")),
        ("message.ack", lambda m: m["payload"].update(status="REJECTED")),
        ("message.ack", lambda m: m["payload"].update(error_code="INVALID_PAYLOAD")),
        ("message.ack", lambda m: m["payload"].update(detail="x" * 257)),
        ("session.opened", lambda m: m["payload"].update(connection_generation=0)),
        ("session.opened", lambda m: m.update(command_id="cmd-1")),
        ("session.unknown", lambda m: m.update(type="session.unknown")),
    ],
)
def test_invalid_frames_are_rejected(type_, mutate):
    message = message_of("session.opened" if type_ == "session.unknown" else type_)
    mutate(message)
    assert validate(message)


def test_ack_examples_cover_stored_and_rejected():
    statuses = {
        r["message"]["payload"]["status"]
        for r in ROWS
        if r["message"]["type"] == "message.ack"
    }
    assert statuses == {"STORED", "REJECTED"}


def test_responses_point_at_the_request_they_answer():
    request = message_of("task.requested")
    rejected = message_of("task.rejected")
    assert rejected["command_id"] == request["command_id"]
    assert rejected["in_reply_to_message_id"] == request["message_id"]
    # A conflict answers the second request that reused the command_id; the original
    # request keeps its own outcome.
    conflict = message_of("command.conflict")
    assert conflict["command_id"] == request["command_id"]
    assert conflict["payload"]["original_message_id"] == request["message_id"]
    assert (
        conflict["in_reply_to_message_id"]
        == conflict["payload"]["conflicting_message_id"]
    )
    assert conflict["payload"]["conflicting_message_id"] != request["message_id"]


def test_request_sha256_vector_ignores_transport_metadata():
    request = message_of("task.requested")
    vector = json.loads((INTERFACES / "examples.json").read_text())
    row = next(r for r in vector if "request_sha256" in r)
    expected = row["request_sha256"]
    # The canonical text is what other languages must reproduce byte for byte.
    assert hashlib.sha256(row["canonical"].encode()).hexdigest() == expected
    assert request_sha256(request) == expected
    for field, value in [
        ("message_id", "other-id"),
        ("sent_at", "2026-11-14T03:00:09.000Z"),
        ("expires_at", "2026-11-14T03:10:00.000Z"),
    ]:
        changed = deepcopy(request)
        changed[field] = value
        assert request_sha256(changed) == expected
    reordered = deepcopy(request)
    reordered["payload"]["packages"].reverse()
    assert request_sha256(reordered) == expected
    for mutate in (
        lambda m: m["payload"].update(
            order_revision=m["payload"]["order_revision"] + 1
        ),
        lambda m: m.update(command_id="cmd-other"),
        lambda m: m["payload"]["packages"][0].update(package_revision=99),
    ):
        changed = deepcopy(request)
        mutate(changed)
        assert request_sha256(changed) != expected


def test_examples_contain_no_secret_like_keys():
    text = json.dumps(ROWS).lower()
    for word in ("token", "password", "secret", "authorization", "bearer"):
        assert word not in text
