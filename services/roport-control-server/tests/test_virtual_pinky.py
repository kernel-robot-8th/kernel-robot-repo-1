"""Shared hardware driver through virtual I/O and real loopback MQTT (no ROS)."""

import json
import sys
import time
from copy import deepcopy
from pathlib import Path

import pytest
from virtual_pinky import VirtualPinky

from roport_control_server.adapters.mqtt.field_pinky import FieldPinkyClient
from roport_control_server.adapters.mqtt.field_trial_transport import (
    FieldTrialTransport,
)
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.messages import topic
from roport_control_server.adapters.mqtt.motion import MotionReceiver, motion_topic
from roport_control_server.adapters.mqtt.mqtt import MqttConfig
from roport_control_server.executor.pinky_trial import TrialControl
from roport_control_server.executor.plans import trial_presets

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from run_virtual_trial import simulation_profile


@pytest.mark.parametrize("scenario", ["single", "two", "smartcart"])
@pytest.mark.parametrize("common_profile", [False, True])
def test_shared_driver_mqtt_full_scenario(broker, tmp_path, scenario, common_profile):
    profile, _ = simulation_profile("two" if common_profile else scenario)
    port, _, _ = broker
    roles = {
        r: v
        for r, v in profile["robots"].items()
        if (
            scenario == "two"
            or r == ("smartcart" if scenario == "smartcart" else "logistics")
        )
    }
    journal = Journal(tmp_path / "control.json")
    transport = FieldTrialTransport(
        MqttConfig("127.0.0.1", "virtual-owner", port=port),
        [
            topic(profile["site_id"], "robot", r["robot_id"], "event")
            for r in profile["robots"].values()
        ],
        profile,
        lambda: journal.data,
    )
    control = TrialControl(
        profile,
        transport,
        journal,
        smartcart_only=not common_profile and scenario == "smartcart",
    )
    if common_profile:
        control.configure(trial_presets(profile)[scenario])
    motion_transport = FieldTrialTransport(
        MqttConfig("127.0.0.1", "virtual-poses", port=port),
        [motion_topic(profile["site_id"], r["robot_id"]) for r in roles.values()],
        profile,
        lambda: journal.data,
    )
    control.motion = MotionReceiver(profile, motion_transport)
    positions = {"line": set(), "nav2": set()}
    drivers, clients = [], []
    try:
        for role, robot in roles.items():
            driver = VirtualPinky(profile, role, threaded=False)
            drivers.append(driver)
            client = FieldPinkyClient(
                MqttConfig("127.0.0.1", role, port=port),
                profile,
                role,
                tmp_path / f"{role}.json",
                driver,
            )
            clients.append(client)
            client.start()
        transport.start()
        motion_transport.start()
        deadline = time.monotonic() + 60
        while control.state["phase"] != "COMPLETE" and time.monotonic() < deadline:
            for client, driver in zip(clients, drivers, strict=True):
                client.poll(0)
                if client.transport.ready and not client.halted:
                    driver.heartbeat()
                # Accelerated physics only in this unit test; launcher always runs at 1x.
                driver.tick()
                for cmd, phase, details in driver.drain_results():
                    client.report(cmd, phase, **details)
                client.publish_motion()
            if any(driver.mode == "nav2" for driver in drivers):
                time.sleep(
                    0.001
                )  # Let real 5 Hz telemetry observe accelerated Nav2 travel.
            control.poll(0)
            for robot, row in control.motion.snapshot(
                control.state["statuses"]
            ).items():
                mode = control.state["statuses"][robot]["mode"]
                positions[mode].add((row["pose"]["x"], row["pose"]["y"]))
            assert control.state["phase"] != "HOLD", control.state["fault"]
            if control.state["phase"] == "READY" and not control.start_error():
                control.start()
            if control.state["gate"]:
                control.dummy(control.state["gate"]["stop_session_id"])
            if control.state["phase"] == "WAIT_REQUEST" and not control.request_error():
                control.request([{"package_id": profile["package_id"], "quantity": 1}])
        assert control.state["phase"] == "COMPLETE", control.state["phase"]
        assert all(len(points) > 1 for points in positions.values()), positions
        assert "navigate" in control.state["completed"]
        # Preserve the old 100 s lower bound, scaled from the 0.03 m/s preset.
        assert all(driver.sim.t > 100 * 0.03 / 0.08 for driver in drivers)
        assert all(driver.ex.cfg.base_speed == 0.08 for driver in drivers)
        for client in clients:
            routes = [
                row["command"]
                for row in client.journal.data["records"].values()
                if row["command"]["type"] == "line_route_command"
            ]
            assert routes
            assert {m["payload"]["speed_limit_mps"] for m in routes} == {0.08}
        assert (
            control.state["completed"]["navigate"]["payload"]["nav2_result"]
            == "SUCCEEDED"
        )
    finally:
        for client in clients:
            client.close()
        for driver in drivers:
            driver.close()
        transport.stop()
        motion_transport.stop()
        journal.close()


def route_command(profile):
    return {
        "type": "route.requested",
        "payload": profile["routes"]["to_delivery"],
        "command_id": "test",
        "execution_id": "test",
    }


def test_real_route_logic_requires_time_rotation_and_load_done():
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    try:
        cmd = route_command(profile)
        driver.heartbeat()
        driver.validate_action(cmd)
        driver.dispatch(cmd)
        results = []
        while not results or results[-1][1] != "completed":
            driver.heartbeat()
            driver.tick()
            results += driver.drain_results()
            assert driver.sim.t < 300
        assert 60 < driver.sim.t < 120
        assert driver.ex.state == "loading"
        assert abs(driver.observe()["heading_rad"] - 0.0017453) < 0.1
        assert [r[2]["seq"] for r in results] == list(range(1, 9))
        cmd["payload"] = profile["routes"]["cart_to_switch"]
        driver.validate_action(cmd)
        driver.dispatch(cmd)
        assert driver.last["cmd"] == "load_done"
        assert driver.logic.pending is not None
    finally:
        driver.close()


def test_lease_loss_stops_motion_and_latches():
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    try:
        driver.heartbeat()
        driver.dispatch(route_command(profile))
        driver.tick()
        driver.lease_at = time.monotonic() - 2
        driver.tick()
        before = (driver.sim.x, driver.sim.y)
        driver.heartbeat()
        driver.tick()
        assert (driver.sim.x, driver.sim.y) == before
        assert driver.sim.cmd == (0, 0)
        assert driver.observe()["fault"] == "drive_lease_expired"
        assert driver.drain_results()[0][1] == "failed"
    finally:
        driver.close()


def test_missing_tape_fails_instead_of_fake_arrival():
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", fault="line-loss", threaded=False)
    try:
        driver.heartbeat()
        driver.dispatch(route_command(profile))
        result = []
        for _ in range(500):
            driver.heartbeat()
            driver.tick()
            result += driver.drain_results()
            if any(phase in {"failed", "completed"} for _, phase, _ in result):
                break
        assert result[-1][1] == "failed"
        assert driver.logic.node is None
        assert driver.sim.cmd == (0, 0)
    finally:
        driver.close()


def test_nav2_off_does_not_claim_readiness():
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", nav2=False, threaded=False)
    try:
        driver.heartbeat()
        assert not driver.observe()["nav2_ready"]
        cmd = {
            "type": "mode_switch.requested",
            "payload": {k: profile[k] for k in ("map_id", "map_version")},
        }
        with pytest.raises(ValueError, match="nav2_not_ready"):
            driver.validate_action(cmd)
    finally:
        driver.close()


def test_virtual_nav2_return_reacquires_n12_before_line_route():
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    try:
        driver.sim.place("N12", "W")
        driver.nav.pose = driver.map_pose()
        driver.mode = "nav2"
        driver.heartbeat()
        driver.tick()
        command = {
            "type": "mode_switch.requested",
            "command_id": "return-line",
            "execution_id": "return-line",
            "payload": {
                "node_id": "N12",
                "from_mode": "nav2",
                "to_mode": "line",
                **{k: profile[k] for k in ("map_id", "map_version")},
            },
        }
        driver.nav.goal = dict(driver.nav.pose)
        with pytest.raises(ValueError, match="N12_physical_alignment_required"):
            driver.validate_action(command)
        driver.nav.cancel()
        driver.validate_action(command)
        driver.dispatch(command)
        results = []
        deadline = time.monotonic() + 2
        while not results and time.monotonic() < deadline:
            driver.heartbeat()
            driver.tick()
            results.extend(driver.drain_results())
            time.sleep(0.02)
        assert results[-1][1] == "completed"
        assert driver.observe()["node_id"] == "N12"
        assert driver.mode == "line"
        route = route_command(profile)
        route["payload"] = {
            **route["payload"],
            "start_node": "N12",
            "start_heading_rad": driver.observe()["heading_rad"],
            "route": ["N12", "N08"],
            "goal": {"node_id": "N08", "location_id": None, "on_arrival": "hold"},
        }
        driver.validate_action(route)
        driver.dispatch(route)
        results = []
        for _ in range(2000):
            driver.heartbeat()
            driver.tick()
            results.extend(driver.drain_results())
            if results and results[-1][1] in {"completed", "failed"}:
                break
        assert results[-1][1] == "completed", results
        assert driver.observe()["node_id"] == "N08"
    finally:
        driver.close()


def test_record_preserves_settings_but_isolates_identity(tmp_path):
    profile, _ = simulation_profile("two")
    profile.update(mode="hardware", field_verified=True)
    for route in profile["routes"].values():
        route["speed_limit_mps"] = 0.04
    record = tmp_path / "real.json"
    data = {
        "config": profile,
        "phase": "COMPLETE",
        "events": {},
        "completed": {},
        "scope": "line-only",
        "smartcart_only": True,
        "plan": {"name": "recorded segment", "steps": []},
    }
    record.write_text(json.dumps(data))
    original = record.read_bytes()
    virtual, baseline = simulation_profile("two", record)
    assert virtual["routes"] == profile["routes"]
    driver = VirtualPinky(virtual, "logistics", threaded=False)
    try:
        assert driver.ex.cfg.base_speed == driver.nav.speed_mps == 0.04
    finally:
        driver.close()
    assert virtual["robots"]["smartcart"]["robot_id"] == "pinky3"
    assert virtual["robots"] != profile["robots"]
    assert virtual["site_id"] != profile["site_id"]
    assert virtual["mode"] == "simulation" and not virtual["field_verified"]
    assert record.read_bytes() == original
    assert baseline["phase"] == "COMPLETE"
    assert baseline["recorded_plan"] == data["plan"]
    assert baseline["replay_scope"] == "configuration_only"
    assert baseline["source_code_verified"] is False
    old = deepcopy(data)
    old["config"]["graph_version"] = "line-v13-20261103"
    record.write_text(json.dumps(old))
    with pytest.raises(ValueError, match="graph_version mismatch"):
        simulation_profile("two", record)


def test_hardware_profile_cannot_start_virtual_driver():
    profile, _ = simulation_profile("single")
    profile["mode"] = "hardware"
    with pytest.raises(ValueError, match="simulation"):
        VirtualPinky(profile, "logistics")


def test_stale_odometry_cannot_report_stopped_or_complete():
    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", fault="odom-stale", threaded=False)
    try:
        driver.heartbeat()
        driver.dispatch(route_command(profile))
        driver.motion_time = 5
        driver.logic.odom_t = time.time() - 2
        driver.tick()
        assert driver.observe()["stopped"] is False
        assert not any(phase == "completed" for _, phase, _ in driver.drain_results())
    finally:
        driver.close()


@pytest.mark.parametrize(
    "termination,scenario",
    [("SIGINT", "single"), ("SIGTERM", "two"), ("SIGHUP", "smartcart")],
)
def test_launcher_isolated_environment_and_cleanup(termination, scenario):
    import os
    import signal
    import subprocess

    binary = os.environ.get("MOSQUITTO_BIN")
    if not binary:
        pytest.skip("launcher smoke requires MOSQUITTO_BIN")
    service = Path(__file__).resolve().parents[1]
    # Use a short path: pytest tmp_path may exceed the AF_UNIX path limit.
    import tempfile

    with tempfile.TemporaryDirectory(prefix="rp-vp-") as directory:
        root = Path(directory) / "run"
        env = {
            **os.environ,
            "ROPORT_MQTT_HOST": "192.0.2.1",
            "ROPORT_MQTT_PORT": "1",
            "ROPORT_MQTT_USERNAME": "must-not-inherit",
            "ROPORT_MQTT_PASSWORD": "test-only",
        }
        proc = subprocess.Popen(
            [
                sys.executable,
                "tools/run_virtual_trial.py",
                "--output",
                str(root),
                "--scenario",
                scenario,
            ],
            cwd=service,
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            from mqtt_helpers import wait_until
            from run_virtual_trial import fetch

            def ready():
                if proc.poll() is not None:
                    raise AssertionError("launcher exited")
                if not (root / "run.json").exists():
                    return False
                try:
                    info = json.loads((root / "run.json").read_text())
                    return fetch(info["url"].removesuffix("/ui/"))["can_start"]
                except (OSError, TypeError, json.JSONDecodeError):
                    return False

            wait_until(ready, timeout=20)
            info = json.loads((root / "run.json").read_text())
            assert info["simulation"] is True
            assert (root / "ui-key.txt").stat().st_mode & 0o077 == 0
            assert "127.0.0.1" in (root / "mosquitto.conf").read_text()
            import urllib.error
            import urllib.request

            url = info["url"].removesuffix("/ui/")
            old = fetch(url)
            key = (root / "ui-key.txt").read_text().strip()
            request = urllib.request.Request(
                url + "/api/trial/commands",
                json.dumps(
                    {
                        "action": "reset",
                        **{k: old[k] for k in ("session_id", "task_id", "phase")},
                    }
                ).encode(),
                headers={
                    "Content-Type": "application/json",
                    "Origin": url,
                    "x-roport-trial-token": key,
                },
            )
            with urllib.request.urlopen(request, timeout=3) as response:
                assert json.load(response)["accepted"]

            def reset_ready():
                try:
                    trial = fetch(url)
                    return (
                        trial["session_id"] != old["session_id"] and trial["can_start"]
                    )
                except (OSError, TypeError):
                    return False

            wait_until(reset_ready, timeout=30)
            new_info = json.loads((root / "run.json").read_text())
            archive = Path(new_info["previous_run_dir"])
            assert new_info["url"] == info["url"] and new_info["reset_count"] == 1
            assert (root / "ui-key.txt").read_text().strip() == key
            old_config = json.loads((archive / "trial.json").read_text())
            new_config = json.loads((root / "trial.json").read_text())
            assert old_config["site_id"] != new_config["site_id"]
            for role in old_config["robots"]:
                assert (
                    old_config["robots"][role]["binding_id"]
                    != new_config["robots"][role]["binding_id"]
                )
            assert (
                json.loads((archive / "outcome.json").read_text())["result"] == "reset"
            )
            assert (archive / "control.json").exists()
            for pid in info["pids"].values():
                with pytest.raises(ProcessLookupError):
                    os.kill(pid, 0)
            info = new_info
            with pytest.raises(urllib.error.HTTPError) as stale:
                urllib.request.urlopen(request, timeout=3)
            assert stale.value.code == 409
            assert not (root / "reset.request").exists()
        finally:
            proc.send_signal(getattr(signal, termination))
            proc.wait(timeout=20)
        assert proc.returncode == 0
        assert (root / "control.json").exists()
        for pid in info["pids"].values():
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)


def test_display_pose_is_continuous_stamped_and_separate_from_control():
    from datetime import UTC, datetime

    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", threaded=False)
    try:
        first = driver.observe_motion()
        assert first["pose_source"] == "simulation"
        driver.heartbeat()
        driver.dispatch(route_command(profile))
        node = driver.observe()["node_id"]
        for _ in range(40):
            driver.heartbeat()
            driver.tick()
        moving = driver.observe_motion()
        assert driver.observe()["node_id"] == node
        assert moving["pose"] != first["pose"]
        assert moving["observed_at"] != first["observed_at"]
        assert "pose" not in driver.observe()  # Task-v1 state is unchanged.
        driver.safe_stop("test_stop")
        driver.tick()
        stopped = driver.observe_motion()
        driver.tick()
        again = driver.observe_motion()
        assert stopped["pose"] == again["pose"]
        assert datetime.fromisoformat(again["observed_at"]) <= datetime.now(UTC)
        driver.nav.observed_at -= 10
        driver.mode = "nav2"
        assert driver.observe_motion()["pose"] is None
    finally:
        driver.close()


def test_line_display_estimate_uses_measured_frame_when_localization_is_off():
    import math

    profile, _ = simulation_profile("single")
    driver = VirtualPinky(profile, "logistics", nav2=False, threaded=False)
    try:
        assert driver.observe_motion()["pose"] is None
        driver.heartbeat()
        driver.dispatch(route_command(profile))
        for _ in range(20):
            driver.heartbeat()
            driver.tick()
        estimate = driver.logic.st["pose_est"]
        motion = driver.observe_motion()
        angle = math.radians(driver.frame["yaw_deg"])
        assert motion["pose_source"] == "line_estimate"
        assert motion["pose"]["x"] == pytest.approx(
            driver.frame["x_m"]
            + math.cos(angle) * estimate["x_cm"] / 100
            - math.sin(angle) * estimate["y_cm"] / 100
        )
        assert motion["pose"]["y"] == pytest.approx(
            driver.frame["y_m"]
            + math.sin(angle) * estimate["x_cm"] / 100
            + math.cos(angle) * estimate["y_cm"] / 100
        )
        driver.logic.st_t -= 2
        assert driver.observe_motion()["pose"] is None
    finally:
        driver.close()
