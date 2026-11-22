"""Deployment snapshots and passive checks; no ROS or physical devices started."""

import hashlib
import json
import subprocess
import sys
import threading
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Empty

import pytest

SERVICE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SERVICE / "tools"))
from check_field import StateCheck, listen, settings
from fleet_profiles import profiles
from prepare_field import prepare_field

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.mqtt import Delivery, MqttConfig, MqttTransport
from roport_control_server.adapters.mqtt.trial_runner import select_robot


def sample(config, *, sequence=1, **changes):
    state = {
        "node": "N16",
        "arrival_command_id": None,
        "mode": "line",
        "stopped": True,
        "line_enabled": False,
        "nav2_ready": True,
        "localization_ready": True,
        "drive_owner": "none",
        "graph_version": config["graph_version"],
        "map_version": config["map_version"],
        "heading_rad": -1.569,
        "fault": None,
    }
    state.update(changes)
    return wire.build_message(
        "robot_state",
        "pinky1",
        config["control_id"],
        {
            "boot_version": "test-boot",
            "observed_at": datetime.now(UTC).isoformat(),
            "sequence": sequence,
            "active_command_id": None,
            "state": state,
        },
    )


def delivery(message, **changes):
    return Delivery(
        **(
            {
                "topic": wire.topic(message["source"], "event"),
                "payload": wire.encode(message),
                "retained": False,
                "mid": 1,
                "qos": 1,
                "generation": 0,
            }
            | changes
        )
    )


def test_snapshot_matches_sources_and_never_certifies_hardware(tmp_path):
    root = tmp_path / "handoff"
    fleet = prepare_field(root)
    assert fleet["mode"] == "hardware" and not fleet["field_verified"]
    assert fleet["speed_limit_mps"] == 0.06 and "node_clearance_m" not in fleet
    assert all(
        v == {"model_version": None, "calibration_version": None}
        for v in fleet["omx"].values()
    )
    manifest = json.loads((root / "manifest.json").read_text())
    for entry in manifest["files"]:
        assert (
            hashlib.sha256((root / entry["path"]).read_bytes()).hexdigest()
            == entry["sha256"]
        )
    for robot, role in (
        ("pinky1", "logistics"),
        ("pinky2", "logistics"),
        ("pinky3", "smartcart"),
    ):
        profile = json.loads((root / robot / "trial.json").read_text())
        assert all(
            profile[k] == fleet[k]
            for k in ("control_id", "map_version", "graph_version", "nodes")
        )
        with pytest.raises(ValueError, match="field_verified"):
            select_robot(profile, role, hardware_authorized=True)
        profile["field_verified"] = True
        assert (
            select_robot(profile, role, hardware_authorized=True)["robot_id"] == robot
        )
    subprocess.run(
        [sys.executable, "-c", "import pinky_mqtt.trial_runner"], cwd=root, check=True
    )
    with pytest.raises(FileExistsError):
        prepare_field(root)
    assert profiles()[0]["speed_limit_mps"] == 0.08
    assert profiles()[0]["omx"]["omx1"]["model_version"] == "sim-model"


def test_settings_detects_cross_pc_map_and_slot_mismatch(tmp_path):
    root = tmp_path / "handoff"
    fleet = prepare_field(root)
    _, issues = settings(root, ["pinky1", "omx3"])
    assert any("field_verified" in p for p in issues)
    assert any("model_version" in p for p in issues)
    assert any("n12_goal" in p for p in issues)
    fleet["n12_goal"] = profiles()[0]["n12_goal"]
    fleet["field_verified"] = True
    fleet["omx"]["omx3"] = {
        "model_version": "measured-model",
        "calibration_version": "measured-cal",
    }
    (root / "fleet.json").write_text(json.dumps(fleet))
    path = root / "pinky1/trial.json"
    p = json.loads(path.read_text())
    p["field_verified"] = True
    path.write_text(json.dumps(p))
    assert settings(root, ["pinky1", "omx3"])[1] == []
    fleet["n12_goal"]["map_version"] = "old-goal-map"
    (root / "fleet.json").write_text(json.dumps(fleet))
    assert any(
        "goal N12_RETURN map_version mismatch" in issue
        for issue in settings(root, ["pinky1"])[1]
    )
    p["map_version"] = "other-map"
    p["robots"]["logistics"]["robot_id"] = "pinky2"
    path.write_text(json.dumps(p))
    problems = settings(root, ["pinky1"])[1]
    assert any("map_version" in p for p in problems)
    assert any("slot mismatch" in p for p in problems)


def test_report_requires_fresh_increasing_single_boot_states():
    config, _ = profiles()
    config["expected_starts"] = {"pinky1": "N16"}
    check = StateCheck(config, ["pinky1"], ["pinky1"])
    first = sample(config)
    check.receive(delivery(first))
    check.receive(delivery(first))
    assert not check.report()["ok"]
    check.receive(delivery(sample(config, sequence=2)))
    assert check.report()["ok"]
    assert not check.report(datetime.now(UTC) + timedelta(seconds=4))["ok"]
    changed = sample(config, sequence=3)
    changed["payload"]["boot_version"] = "another-owner"
    check.receive(delivery(changed))
    assert not check.report()["ok"]


@pytest.mark.parametrize(
    "change", [{"qos": 0}, {"retained": True}, {"topic": wire.topic("pinky2", "event")}]
)
def test_invalid_delivery_never_counts_as_prepared(change):
    config, _ = profiles()
    config["expected_starts"] = {"pinky1": "N16"}
    check = StateCheck(config, ["pinky1"])
    with pytest.raises(ValueError):
        check.receive(delivery(sample(config), **change))
    assert not check.report()["ok"]


@pytest.mark.parametrize(
    "change",
    [
        {"map_version": "old-map"},
        {"stopped": False},
        {"line_enabled": True},
        {"nav2_ready": False},
        {"node": None},
    ],
)
def test_report_explains_unready_state(change):
    config, _ = profiles()
    config["expected_starts"] = {"pinky1": "N16"}
    check = StateCheck(config, ["pinky1"], ["pinky1"])
    for sequence in (1, 2):
        check.receive(delivery(sample(config, sequence=sequence, **change)))
    assert not check.report()["ok"]


def test_passive_mqtt_sends_no_command_ack_or_keepalive(broker):
    config, _ = profiles()
    config["expected_starts"] = {"pinky1": "N16"}
    sender = MqttTransport(
        MqttConfig("127.0.0.1", "test-field-source", broker[0]),
        [wire.topic("pinky1", c) for c in ("cmd", "ack")],
    )
    stop = threading.Event()
    errors = []

    def publish():
        try:
            sequence = 0
            while not stop.wait(0.08):
                sequence += 1
                sender.publish(
                    wire.topic("pinky1", "event"), sample(config, sequence=sequence)
                )
        except Exception as exc:  # noqa: BLE001 - propagate thread failures to the test
            errors.append(exc)

    sender.start()
    worker = threading.Thread(target=publish)
    worker.start()
    try:
        report = listen(
            config,
            ["pinky1"],
            MqttConfig("127.0.0.1", "test-field-check", broker[0]),
            0.6,
            nav2=["pinky1"],
        )
        assert report["ok"], report
        with pytest.raises(Empty):
            sender.receive(0.1)
    finally:
        stop.set()
        worker.join(2)
        sender.stop()
    assert not errors


@pytest.mark.parametrize("arm_ids,ok", [(["omx1"], False), (["omx1", "omx2"], True)])
def test_shelf_readiness_requires_both_arms(arm_ids, ok):
    config, _ = profiles()
    check = StateCheck(config, ["shelf-01"])
    arms = [
        {
            "robot_id": robot,
            "boot_version": "boot",
            "active_command_id": None,
            "stopped": True,
            "ready": True,
            "home_reached": True,
            "holding_state": "empty",
            "holding_package_id": None,
            "fault": None,
        }
        for robot in arm_ids
    ]
    for sequence in (1, 2):
        p = sample(config, sequence=sequence)["payload"]
        p["state"] = {
            "stopped": True,
            "ready": True,
            "fault": None,
            "arms": arms,
            "docking": None,
        }
        msg = wire.build_message("robot_state", "shelf-01", config["control_id"], p)
        check.receive(delivery(msg))
    assert check.report()["ok"] is ok


@pytest.mark.parametrize(
    "change", ["copied", "renamed", "missing_id", "heading", "tolerance"]
)
def test_delivery_pose_cannot_be_used_for_n12_return(tmp_path, change):
    root = tmp_path / "bundle"
    fleet = prepare_field(root)
    goal = profiles()[0]["n12_goal"]
    if change in {"copied", "renamed"}:
        goal = dict(fleet["goals"]["PUBLIC_SPACE_TRIAL"])
        if change == "renamed":
            goal["goal_id"] = "N12_RETURN"
    elif change == "missing_id":
        del goal["goal_id"]
    elif change == "heading":
        goal["pose"]["yaw"] = 0.0
    else:
        goal["position_tolerance_m"] = 0.5
    fleet["n12_goal"] = goal
    (root / "fleet.json").write_text(json.dumps(fleet))
    assert any("N12_RETURN identity" in issue for issue in settings(root, [])[1])


@pytest.mark.parametrize("node,ready", [("N17", True), ("N16", False)])
def test_second_logistics_robot_uses_individual_start(tmp_path, node, ready):
    root = tmp_path / "bundle"
    prepare_field(root)
    config, _ = settings(root, ["pinky2"])
    check = StateCheck(config, ["pinky2"])
    for seq in (1, 2):
        msg = sample(config, sequence=seq, node=node)
        msg["source"] = "pinky2"
        check.receive(delivery(msg))
    assert check.report()["ok"] is ready


def test_old_first_sample_expires_until_replaced():
    config, _ = profiles()
    config["expected_starts"] = {"pinky1": "N16"}
    check = StateCheck(config, ["pinky1"])
    msg = sample(config)
    msg["payload"]["observed_at"] = (datetime.now(UTC) - timedelta(hours=2)).isoformat()
    check.receive(delivery(msg))
    check.receive(delivery(sample(config, sequence=2)))
    assert not check.report()["ok"]
    check.receive(delivery(sample(config, sequence=3)))
    assert check.report()["ok"]
