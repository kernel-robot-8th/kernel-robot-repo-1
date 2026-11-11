import hashlib
import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from zipfile import ZipFile

import pytest
import yaml

from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import topic
from roport_control_server.adapters.mqtt.mqtt import MqttConfig, MqttTransport
from roport_control_server.executor.pinky_trial import TrialControl

SERVICE = Path(__file__).resolve().parents[1]


def tool(name):
    spec = importlib.util.spec_from_file_location(
        name, SERVICE / "tools" / f"{name}.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def handoff(tmp_path):
    scenarios = tmp_path / "scenarios"
    pinky = SERVICE.parent / "roport-pinky"
    prepare = tool("prepare_pinky_trial")
    profiles = prepare.prepare_bundle(
        scenarios,
        pinky / "maps/airport_map.yaml",
        pinky / "src/roport_gz_world/layout/roport_airport.yaml",
    )
    archive = tmp_path / "handoff.zip"
    tool("build_pinky_mqtt_kit").build(archive, scenarios_dir=scenarios)
    kit = tmp_path / "kit"
    with ZipFile(archive) as source:
        source.extractall(kit)
    return scenarios, profiles, kit


def test_bundle_keeps_four_distinct_identities_and_does_not_overwrite(handoff):
    scenarios, profiles, kit = handoff
    assert set(profiles) == {
        f"{s}/{m}" for s in ("single", "two") for m in ("simulation", "hardware")
    }
    assert len({p["site_id"] for p in profiles.values()}) == 4
    assert len({p["control_id"] for p in profiles.values()}) == 4
    bindings = [
        r["binding_id"] for p in profiles.values() for r in p["robots"].values()
    ]
    assert len(bindings) == len(set(bindings)) == 6
    for name, p in profiles.items():
        assert json.loads((kit / "scenarios" / name / "trial.json").read_text()) == p
        assert (
            "ROPORT_MQTT_PORT=18883"
            in (kit / "scenarios" / name / "broker.env.example").read_text()
        )
        assert p["nav_goal"]["goal_id"] == "PUBLIC_SPACE_TRIAL"
        if p["mode"] == "hardware":
            assert p["field_verified"] is False
            for step, route in p["routes"].items():
                if p["scenario"] == "single_pinky" and step == "to_delivery":
                    assert route["start_heading_rad"] == -1.5690
                    assert route["start_maneuver"] == "none"
                    assert route["speed_limit_mps"] == 0.06
                elif p["scenario"] == "single_pinky":
                    assert route["start_maneuver"] == "none"
                    assert route["speed_limit_mps"] == 0.06
                    assert abs(route["start_heading_rad"] - 0.001745329) < 1e-8
                else:
                    assert isinstance(route["start_heading_rad"], float)
                    assert route["start_maneuver"] == "none"
                    assert route["speed_limit_mps"] == 0.06
        assert p["command_timeout_s"] == 300
    original = (scenarios / "single/simulation/trial.json").read_bytes()
    pinky = SERVICE.parent / "roport-pinky"
    with pytest.raises(FileExistsError):
        tool("prepare_pinky_trial").prepare_bundle(
            scenarios,
            pinky / "maps/airport_map.yaml",
            pinky / "src/roport_gz_world/layout/roport_airport.yaml",
        )
    assert (scenarios / "single/simulation/trial.json").read_bytes() == original


def test_handoff_manifest_matches_exported_files_and_map(handoff):
    _, profiles, kit = handoff
    manifest = json.loads((kit / "manifest.json").read_text())
    for name, checksum in manifest["files"].items():
        assert hashlib.sha256((kit / name).read_bytes()).hexdigest() == checksum
    assert not (kit / "pinky_mqtt/state.py").exists()
    pinky = SERVICE.parent / "roport-pinky"
    expected = hashlib.sha256(
        (pinky / "maps/airport_map.yaml").read_bytes()
        + (pinky / "maps/airport_map.pgm").read_bytes()
    ).hexdigest()[:16]
    assert all(p["map_version"] == expected for p in profiles.values())


def test_hardware_static_check_has_no_journal_or_broker_side_effects(handoff, tmp_path):
    scenarios, _, _ = handoff
    config = scenarios / "two/hardware/trial.json"
    before = config.read_bytes()
    journal = tmp_path / "never-created.json"
    env = {**os.environ, "ROPORT_MQTT_HOST": "192.0.2.1", "ROPORT_MQTT_PORT": "18883"}
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "roport_control_server.executor.pinky_trial",
            "--config",
            str(config),
            "--journal",
            str(journal),
            "--scope",
            "line-only",
            "--check",
        ],
        env=env,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "field_verified=False" in result.stdout
    assert "NOT checked" in result.stdout
    assert not journal.exists() and config.read_bytes() == before


@pytest.mark.parametrize(
    "change",
    [
        lambda v: v.update(units="m"),
        lambda v: v.update(version="v15"),
        lambda v: v["map"].update(width=1),
        lambda v: v["frame"].update(x_m=float("nan")),
        lambda v: v["nodes"]["items"].append(v["nodes"]["items"][0]),
        lambda v: v["nodes"]["items"][0].update(links=["UNKNOWN"]),
        lambda v: v["nodes"]["items"][0].update(at=[100000, 100000]),
    ],
)
def test_config_generator_rejects_mismatched_layout_before_writing(tmp_path, change):
    pinky = SERVICE.parent / "roport-pinky"
    value = yaml.safe_load(
        (pinky / "src/roport_gz_world/layout/roport_airport.yaml").read_text()
    )
    change(value)
    layout = tmp_path / "layout.yaml"
    layout.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError):
        tool("prepare_pinky_trial").prepare_bundle(
            tmp_path / "profiles", pinky / "maps/airport_map.yaml", layout
        )
    assert not (tmp_path / "profiles").exists()


def runner(kit, config, role, journal, *extra):
    return [
        sys.executable,
        "-m",
        "pinky_mqtt.trial_runner",
        "--config",
        str(config),
        "--role",
        role,
        "--journal",
        str(journal),
        *extra,
    ]


@pytest.mark.parametrize(
    "case",
    ["hardware", "hardware_unverified", "wrong_role", "remote_simulation", "check"],
)
def test_runner_rejects_before_driver_import_or_network(handoff, tmp_path, case):
    _, _, kit = handoff
    mode = "hardware" if case.startswith("hardware") else "simulation"
    profile = kit / "scenarios/single" / mode / "trial.json"
    env = {
        **os.environ,
        "PYTHONPATH": str(kit),
        "ROPORT_MQTT_HOST": "not-a-loopback-host"
        if case == "remote_simulation"
        else "127.0.0.1",
    }
    extra = ["--driver", "must_not_import:create_driver"]
    if case == "hardware_unverified":
        extra.append("--hardware-authorized")
    if case == "check":
        extra.append("--check")
    result = subprocess.run(
        runner(
            kit,
            profile,
            "smartcart" if case == "wrong_role" else "logistics",
            tmp_path / "journal.json",
            *extra,
        ),
        env=env,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    assert result.returncode == (0 if case == "check" else 2), result.stderr
    assert "ModuleNotFoundError" not in result.stderr
    assert not (tmp_path / "journal.json").exists()


@pytest.mark.parametrize("scenario", ["single", "two", "smartcart"])
def test_published_runner_with_current_control_over_mqtt(
    handoff, broker, tmp_path, scenario
):
    _, profiles, kit = handoff
    smartcart_only = scenario == "smartcart"
    config_scenario = "two" if smartcart_only else scenario
    profile = profiles[f"{config_scenario}/simulation"]
    config_path = kit / "scenarios" / config_scenario / "simulation/trial.json"
    connection = MqttConfig("127.0.0.1", profile["control_id"], port=broker[0])
    transport = MqttTransport(
        connection,
        [
            topic(profile["site_id"], "robot", r["robot_id"], "event")
            for r in profile["robots"].values()
        ],
    )
    journal = Journal(tmp_path / "control.json")
    control = TrialControl(profile, transport, journal, smartcart_only=smartcart_only)
    processes, logs = [], []
    env = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join((str(kit), str(SERVICE / "tests"))),
        "ROPORT_MQTT_HOST": "127.0.0.1",
        "ROPORT_MQTT_PORT": str(broker[0]),
    }
    phases = []
    try:
        for role in control.robots:
            log = (tmp_path / f"{role}.log").open("w+")
            logs.append(log)
            processes.append(
                subprocess.Popen(
                    runner(
                        kit,
                        config_path,
                        role,
                        tmp_path / f"{role}.json",
                        "--driver",
                        "trial_driver:create_driver",
                    ),
                    env=env,
                    stdout=log,
                    stderr=log,
                )
            )
        transport.start()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            control.poll(0.02)
            phase = control.state["phase"]
            assert all(p.poll() is None for p in processes)
            assert phase != "HOLD", control.state["fault"]
            if not phases or phases[-1] != phase:
                phases.append(phase)
            if phase == "READY" and len(control.state["statuses"]) == len(processes):
                control.start()
            elif phase in {"WAIT_LOAD", "WAIT_INBOUND", "WAIT_CART_LOAD"}:
                control.dummy(control.state["gate"]["stop_session_id"])
            elif phase == "WAIT_REQUEST":
                control.request([{"package_id": profile["package_id"], "quantity": 1}])
            elif phase == "COMPLETE":
                robot_states = [
                    json.loads((tmp_path / f"{role}.json").read_text())
                    for role in control.robots
                ]
                if all(not s["active"] and not s["outbox"] for s in robot_states):
                    break
        assert phases[-1] == "COMPLETE"
        assert len(control.state["completed"]) == (7 if scenario == "two" else 4)
        assert len(control.state["transfers"]) == (
            1 if smartcart_only else 2 if scenario == "single" else 3
        )
        assert all(not s["outbox"] and not s["active"] for s in robot_states)
        assert ("WAIT_REQUEST" in phases) == (scenario == "two")
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                p.wait(timeout=5)
        for log in logs:
            log.close()
        transport.stop()
        journal.close()
