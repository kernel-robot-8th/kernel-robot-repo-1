"""The published examples and AsyncAPI payloads describe the actual native codec."""

import json
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from roport_control_server.adapters.mqtt import field_wire as wire

INTERFACES = Path(__file__).resolve().parents[3] / "interfaces/mqtt"


def test_all_thirteen_types_have_valid_documented_examples():
    rows = json.loads((INTERFACES / "examples.json").read_text())
    assert {r["message"]["type"] for r in rows} == wire.TYPES
    for row in rows:
        m = row["message"]
        assert wire.decode(wire.encode(m)) == m
        endpoint = (
            m["source"]
            if m["type"]
            in {"robot_state", "robot_pose", "command_status", "host_metrics"}
            else m["target"]
        )
        assert row["topic"] == wire.topic(endpoint, row["topic"].rsplit("/", 1)[-1])
        assert row["retain"] is False
        assert row["qos"] == (0 if m["type"] in {"robot_pose", "host_metrics"} else 1)


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


def host_sample(source="pinky1", **payload):
    return wire.build_message(
        "host_metrics",
        source,
        "control-01",
        {
            "boot_version": "3f1c2d4e-0000-4000-8000-000000000001",
            "sequence": 1,
            "observed_at": "2026-11-14T10:00:00+09:00",
            "cpu_count": 4,
            "load_1m": 0.5,
            "load_5m": 0.25,
            "load_15m": 0.0,
            "mem_total_kb": 4000000,
            "mem_available_kb": 3000000,
            "cpu_temp_c": 51.2,
            **payload,
        },
    )


def test_host_topic_covers_hosts_only():
    assert wire.topic("pinky1", "host") == "roport/robot/pinky1/host"
    assert wire.topic("omx3", "host") == "roport/robot/omx3/host"
    assert wire.topic("shelf-01", "host") == "roport/workcell/shelf-01/host"
    for endpoint in ("omx1", "omx2", "control-01"):
        with pytest.raises(ValueError):
            wire.topic(endpoint, "host")


def test_host_metrics_envelope_and_payload_rules():
    for host in ("pinky1", "pinky2", "pinky3", "omx3", "shelf-01"):
        wire.validate(host_sample(host))
    unknown = host_sample(
        **{k: None for k in ("cpu_count", "load_1m", "mem_total_kb", "cpu_temp_c")}
    )
    assert unknown["payload"]["cpu_count"] is None  # null is never coerced to 0
    message = host_sample()
    for bad in (
        {**message, "source": "omx1"},
        {**message, "command_id": "cmd-1"},
        {**message, "expires_at": "2026-11-14T10:00:05+09:00"},
        {**message, "payload": {**message["payload"], "sequence": 0}},
        {**message, "payload": {**message["payload"], "load_1m": -1}},
        {**message, "payload": {**message["payload"], "cpu_count": 2.5}},
        {**message, "payload": {**message["payload"], "mem_available_kb": 4000001}},
        {**message, "payload": {**message["payload"], "cpu_percent": 3}},
    ):
        with pytest.raises(ValueError):
            wire.validate(bad)


def test_host_metrics_never_uses_qos1_task_publish():
    from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport

    transport = MqttTransport(
        MqttConfig("127.0.0.1", "unit-host"), [wire.topic("pinky1", "host")]
    )
    transport._ready.set()  # no broker: rejection happens before any network I/O
    with pytest.raises(ValueError):
        transport.publish(wire.topic("pinky1", "host"), host_sample())
