"""Control-generated routes against the unchanged Pinky driver and 2D line simulator."""

import importlib.util
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from time import monotonic

import pytest
from test_pinky_trial import MemoryTransport, delivery, prepare_module

from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.trial_wire import reply
from roport_control_server.executor.pinky_trial import TrialControl, validate_config

PINKY = Path(__file__).resolve().parents[2] / "roport-pinky"


@pytest.fixture
def rig_module(monkeypatch):
    for package in ("roport_pinky", "roport_line_nav", "roport_pinky_line"):
        monkeypatch.syspath_prepend(str(PINKY / "src" / package))
    spec = importlib.util.spec_from_file_location(
        "pinky_driver_rig", PINKY / "src/roport_pinky/test/test_trial_driver.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("single", [True, False])
def test_control_routes_complete_with_team_line_driver(rig_module, tmp_path, single):
    profile = prepare_module.prepare(
        PINKY / "maps/airport_map.yaml",
        PINKY / "src/roport_gz_world/layout/roport_airport.yaml",
        single_robot=single,
    )
    validate_config(profile, require_field_verification=False)
    assert profile["field_verified"] is False
    with pytest.raises(ValueError, match="verify starts"):
        validate_config(profile)
    profile["mode"] = "simulation"  # No hardware authorization/network/ROS import.
    rigs = {"logistics": rig_module.Rig()}
    if not single:
        rigs["smartcart"] = rig_module.Rig("H1", "N")
    journal = Journal(tmp_path / "control.json")
    control = TrialControl(profile, MemoryTransport(), journal, scope="line-only")

    def observe(role):
        return {
            **rigs[role].logic.observe(),
            "observed_at": datetime.now(UTC).isoformat(),
            "binding_id": profile["robots"][role]["binding_id"],
            "boot_id": f"test-{role}",
        }

    def refresh():
        state = deepcopy(control.state)
        for role, robot in profile["robots"].items():
            state["statuses"][robot["robot_id"]] = observe(role)
            control.heard[robot["robot_id"]] = monotonic()
        journal.save(state)

    try:
        refresh()
        control.start()
        durations = {}
        while control.state["phase"] != "COMPLETE":
            cmd = control.state["active"]
            role = next(
                r
                for r, robot in profile["robots"].items()
                if robot["robot_id"] == cmd["target"]
            )
            rig = rigs[role]
            rig.logic.validate_action(cmd)
            rig.results.clear()
            before = rig.sim.t
            rig.logic.dispatch(cmd)
            assert rig.run(until=rig_module.finished)
            durations[cmd["step_id"]] = round(rig.sim.t - before, 2)
            assert durations[cmd["step_id"]] < profile["command_timeout_s"]
            _, phase, details = rig.results[-1]
            assert phase == "completed", (cmd["step_id"], details)
            refresh()
            control.handle(
                delivery(
                    reply(
                        cmd,
                        "route.completed",
                        {
                            **observe(role),
                            **details,
                            "phase_revision": 1,
                        },
                    ),
                    "event",
                )
            )
            while control.state["gate"]:
                control.dummy(control.state["gate"]["stop_session_id"])
            if control.state["phase"] == "WAIT_REQUEST":
                assert rigs["logistics"].logic.observe()["node_id"] == "N16"
                assert rigs["smartcart"].logic.observe()["node_id"] == "H1"
                control.request([{"package_id": profile["package_id"], "quantity": 1}])
        assert set(control.state["completed"]) == set(profile["routes"])
        assert len(control.state["transfers"]) == (2 if single else 3)
        print(f"{profile['scenario']} simulated seconds per route: {durations}")
    finally:
        journal.close()
