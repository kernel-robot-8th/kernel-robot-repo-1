"""The published examples and AsyncAPI payloads describe the actual native codec."""

import json
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from roport_control_server.adapters.mqtt import field_wire as wire

INTERFACES = Path(__file__).resolve().parents[3] / "interfaces/mqtt"


def test_all_twelve_types_have_valid_documented_examples():
    rows = json.loads((INTERFACES / "examples.json").read_text())
    assert {r["message"]["type"] for r in rows} == wire.TYPES
    for row in rows:
        m = row["message"]
        assert wire.decode(wire.encode(m)) == m
        endpoint = (
            m["source"]
            if m["type"] in {"robot_state", "robot_pose", "command_status"}
            else m["target"]
        )
        assert row["topic"] == wire.topic(endpoint, row["topic"].rsplit("/", 1)[-1])
        assert row["retain"] is False
        assert row["qos"] == (0 if m["type"] == "robot_pose" else 1)


def test_asyncapi_matches_runtime_schema():
    spec = yaml.safe_load((INTERFACES / "roport-mqtt.yaml").read_text())
    assert spec["asyncapi"] == "3.0.0"
    for variant in wire.json_schema()["oneOf"]:
        name = variant["properties"]["type"]["const"]
        # JSON roundtrip normalizes tuple enum declarations to arrays.
        assert spec["components"]["messages"][name]["payload"] == json.loads(
            json.dumps(variant)
        )


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True, "0.03"])
def test_invalid_route_speed_never_reaches_driver(value):
    rows = json.loads((INTERFACES / "examples.json").read_text())
    message = deepcopy(
        next(r["message"] for r in rows if r["message"]["type"] == "line_route_command")
    )
    message["payload"]["speed_limit_mps"] = value
    with pytest.raises(ValueError):
        wire.validate(message)


def test_json_duplicate_keys_rejected():
    with pytest.raises(ValueError, match="duplicate"):
        wire.decode(b'{"type":"robot_state","type":"command_status"}')
