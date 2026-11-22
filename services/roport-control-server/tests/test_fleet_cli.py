import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import confirm_fleet_arrival
from fleet_profiles import profiles

from roport_control_server.adapters.mqtt.mqtt import MqttConfig
from roport_control_server.executor import fleet_runtime


@pytest.mark.parametrize(
    "mode,host", [("hardware", "127.0.0.1"), ("simulation", "example.com")]
)
def test_owner_rejects_unsupported_environment_before_opening_journal(
    tmp_path, monkeypatch, capsys, mode, host
):
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"mode": mode, "control_id": "control-test"}))
    journal = tmp_path / "journal.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "owner",
            "--config",
            str(config),
            "--journal",
            str(journal),
            "--socket",
            "unused",
        ],
    )
    monkeypatch.setattr(
        fleet_runtime, "mqtt_config", lambda _: MqttConfig(host, "unit")
    )
    with pytest.raises(SystemExit) as exc:
        fleet_runtime.main()
    assert exc.value.code == 2
    assert "simulation" in capsys.readouterr().err
    assert not journal.exists()


@pytest.mark.parametrize("error", [KeyboardInterrupt, KeyError, TypeError])
def test_owner_exit_persists_hold_and_closes_resources(
    fleet, tmp_path, monkeypatch, capsys, error
):
    config = tmp_path / "config.json"
    config.write_text(json.dumps({**fleet.config, "mode": "simulation"}))
    journal = fleet.store.path
    fleet.store.close()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "owner",
            "--config",
            str(config),
            "--journal",
            str(journal),
            "--socket",
            "unused",
        ],
    )
    monkeypatch.setattr(
        fleet_runtime, "mqtt_config", lambda _: MqttConfig("127.0.0.1", "unit")
    )
    closed = []
    monkeypatch.setattr(
        fleet_runtime,
        "MqttTransport",
        lambda *_: SimpleNamespace(
            start=lambda: None, stop=lambda: closed.append("mqtt")
        ),
    )
    monkeypatch.setattr(
        fleet_runtime,
        "FleetSocket",
        lambda *_: SimpleNamespace(
            poll=lambda: None, close=lambda: closed.append("socket")
        ),
    )

    def fail(_):
        raise error("injected runtime failure")

    monkeypatch.setattr(fleet_runtime.FleetRuntime, "poll", fail)
    if error is KeyboardInterrupt:
        fleet_runtime.main()
    else:
        with pytest.raises(error):
            fleet_runtime.main()
    data = json.loads(journal.read_text())
    assert data["fault"] == "owner_stopped" and not data["enabled"]
    assert closed == ["socket", "mqtt"]
    assert "local socket" in capsys.readouterr().out


@pytest.mark.parametrize("simulation", [True, False])
def test_return_calibration_is_simulation_only(simulation):
    config, robots = profiles(simulation=simulation)
    assert bool(config.get("node_clearance_m")) is simulation
    assert ("n12_goal" in config) is simulation
    assert bool(config["omx"]["omx1"]["calibration_version"]) is simulation
    assert len(robots) == 3


@pytest.mark.parametrize("error", [None, OSError, ValueError])
def test_arrival_cli_binds_current_session_and_reports_failure(
    monkeypatch, capsys, error
):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "confirm",
            "--socket",
            "local.sock",
            "--robot",
            "pinky1",
            "--command-id",
            "arrival-1",
            "--boot-version",
            "boot-2",
            "--note",
            "checked stopped robot",
        ],
    )
    calls = []

    def rpc(path, message):
        assert path == "local.sock"
        calls.append(message)
        if len(calls) == 1:
            return {"session_id": "current-owner"}
        if error:
            raise error("not confirmed")
        return {}

    monkeypatch.setattr(confirm_fleet_arrival, "rpc", rpc)
    if error:
        with pytest.raises(SystemExit) as exc:
            confirm_fleet_arrival.main()
        assert exc.value.code == 1
        assert "Arrival confirmation failed" in capsys.readouterr().err
    else:
        confirm_fleet_arrival.main()
        assert "Arrival confirmed" in capsys.readouterr().out
    assert calls[0] == {"action": "snapshot"}
    expires = calls[1].pop("expires_at")
    assert time.time() < expires <= time.time() + 1
    assert calls[1] == {
        "action": "confirm_arrival",
        "session_id": "current-owner",
        "robot": "pinky1",
        "command_id": "arrival-1",
        "boot_version": "boot-2",
        "note": "checked stopped robot",
    }


@pytest.mark.parametrize("verified,authorized", [(False, True), (True, False)])
def test_field_owner_requires_both_confirmations(
    tmp_path, monkeypatch, capsys, verified, authorized
):
    config = tmp_path / "fleet.json"
    config.write_text(json.dumps({"mode": "hardware", "field_verified": verified}))
    argv = ["owner", "--config", str(config), "--socket", "unused"]
    if authorized:
        argv.append("--hardware-authorized")
    monkeypatch.setattr(sys, "argv", argv)
    with pytest.raises(SystemExit):
        fleet_runtime.main()
    assert "field verification and explicit authorization" in capsys.readouterr().err
