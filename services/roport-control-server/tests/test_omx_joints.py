"""Exercise the read-only shelf-arm feed with synthetic observations."""

import json
from contextlib import ExitStack
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient
from fleet_web_fixture import SERVICE, owner
from mqtt_helpers import wait_until

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport
from roport_control_server.adapters.mqtt.omx_joints import OmxJointReceiver
from roport_control_server.executor.fleet_runtime import FleetRuntime


def sample(arm="omx1", boot="boot-1", seq=1, kind="mission", when=None):
    return wire.build_message(
        "omx_joint_state",
        arm,
        "control-test",
        {
            "boot_version": boot,
            "sequence": seq,
            "observed_at": (when or datetime.now(UTC)).isoformat(),
            "sample_kind": kind,
            "joints": dict.fromkeys(wire.JOINT_NAMES, 0.0),
        },
    )


def deliver(receiver, msg, **metadata):
    packet = {
        "qos": 0,
        "retain": False,
        "mid": 1,
        "topic": wire.topic(msg["source"], "motion"),
        "payload": wire.encode(msg),
        **metadata,
    }
    receiver._on_message(Mock(), None, SimpleNamespace(**packet))


def test_latest_measured_pose_is_per_arm_and_per_authoritative_boot():
    states = {"omx1": {"boot_version": "boot-1"}, "omx2": {"boot_version": "boot-2"}}
    clock = [50]
    rx = OmxJointReceiver(
        MqttConfig("localhost", "unit"), "control-test", states, lambda: clock[0]
    )
    t = datetime.now(UTC) - timedelta(seconds=0.5)
    original = sample(seq=4, when=t)
    deliver(rx, original)
    deliver(rx, sample("omx2", "boot-2", kind="idle", when=t))
    for dropped in (
        sample(seq=4),
        sample(seq=3),
        sample(seq=5, when=t),
        sample(boot="unknown", seq=5),
        sample(seq=5, when=t + timedelta(seconds=10)),
        sample(seq=5, when=t - timedelta(seconds=3)),
        {**sample(seq=5), "target": "other"},
    ):
        deliver(rx, dropped)
    for bad_packet in (
        {"retain": True},
        {"qos": 1},
        {"topic": wire.topic("omx2", "motion")},
        {"payload": b"{}"},
    ):
        deliver(rx, sample(seq=5), **bad_packet)
    assert rx.snapshot()["omx1"]["sequence"] == 4
    assert rx.snapshot()["omx1"]["status"] == "live"
    assert rx.snapshot()["omx2"]["status"] == "idle"
    clock[0] += 3
    assert rx.snapshot()["omx1"]["status"] == "stale"
    assert rx.snapshot()["omx1"]["joints"] == original["payload"]["joints"]
    assert rx.snapshot()["omx2"]["status"] == "idle"
    clock[0] += 18
    assert rx.snapshot()["omx2"]["status"] == "stale"
    states["omx1"] = {"boot_version": "boot-new"}
    deliver(rx, sample(boot="boot-1", seq=900))
    assert "omx1" not in rx.snapshot()
    deliver(rx, sample(boot="boot-new", seq=0))
    assert rx.snapshot()["omx1"]["sequence"] == 0


def test_wire_requires_six_numeric_angles_and_shelf_source():
    m = sample()
    p = m["payload"]
    for joints in (
        {"joint1": 0},
        {**p["joints"], "gripper_joint_2": 0},
        *(
            {**p["joints"], "joint3": v}
            for v in (True, "0", None, float("inf"), float("nan"))
        ),
    ):
        with pytest.raises(ValueError):
            wire.encode({**m, "payload": {**p, "joints": joints}})
    for source in ("pinky1", "shelf-01", "omx3"):
        with pytest.raises(ValueError):
            wire.encode({**m, "source": source})
    t = MqttTransport(MqttConfig("localhost", "unit-host"), [wire.topic("omx1", "cmd")])
    with pytest.raises(ValueError):
        t.publish(wire.topic("omx1", "motion"), m)


def test_joint_frame_on_command_transport_cannot_change_operational_state():
    control, transport = Mock(), Mock()
    runtime = FleetRuntime(control, transport)
    runtime.handle(
        SimpleNamespace(
            payload=wire.encode(sample()),
            topic=wire.topic("omx1", "event"),
            qos=0,
            retained=False,
        )
    )
    assert not control.mock_calls and not transport.mock_calls


def test_omx_2a_sample_from_host_sdk_through_mqtt_to_websocket(broker, monkeypatch):
    monkeypatch.syspath_prepend(str(SERVICE.parent / "roport-omx/src"))
    from roport_omx.mqtt_link import field_wire
    from roport_omx.mqtt_link.mqtt import (
        MqttConfig as HostConfig,
    )
    from roport_omx.mqtt_link.mqtt import (
        MqttTransport as Host,
    )

    capture = json.loads((SERVICE / "docs/examples/omx-2a-sample.json").read_text())
    recorded = capture["wire"]
    arm = recorded["source"]
    with ExitStack() as stack:
        folder = Path(stack.enter_context(TemporaryDirectory()))
        env = stack.enter_context(owner(folder))
        before = deepcopy(env.control.data)
        target = env.control.config["control_id"]
        rx = OmxJointReceiver(
            MqttConfig("127.0.0.1", "test-control", broker[0]),
            target,
            env.control.observed,
        )
        env.channel.joints = rx
        rx.start()
        stack.callback(rx.stop)
        tx = Host(
            HostConfig("127.0.0.1", "test-host", broker[0]),
            [field_wire.topic(arm, "cmd")],
        )
        tx.start()
        stack.callback(tx.stop)
        client = stack.enter_context(TestClient(env.app))
        ws = stack.enter_context(
            client.websocket_connect(
                "/api/state?mode=omx_joints", headers={"origin": "http://testserver"}
            )
        )
        assert ws.receive_json() == {"available": True, "arms": {}}
        payload = {
            **recorded["payload"],
            "boot_version": env.control.observed[arm]["boot_version"],
            "observed_at": datetime.now(UTC).isoformat(),
        }
        tx.publish_motion(
            capture["topic"],
            field_wire.build_message(recorded["type"], arm, target, payload),
        )
        wait_until(lambda: arm in rx.snapshot())
        for _ in range(10):
            snapshot = ws.receive_json()["arms"]
            if arm in snapshot:
                break
        assert set(snapshot) == {"omx1"}
        assert snapshot[arm]["joints"] == recorded["payload"]["joints"]
        assert snapshot[arm]["status"] == "live"
        assert env.control.data == before
