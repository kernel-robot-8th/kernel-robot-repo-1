from copy import deepcopy
from datetime import UTC, datetime, timedelta

import pytest
from test_fleet import dispatch, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import MemoryTransport, profile  # noqa: F401

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.journal import Journal
from roport_control_server.adapters.mqtt.mqtt import Delivery
from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.fleet_recovery import reconcile, recover_stop
from roport_control_server.executor.fleet_runtime import FleetRuntime


def start_held(fleet):
    submit(fleet, "held-stop")
    command = dispatch(fleet)
    fleet.hold("operator_stop")
    return command


def age_stop(fleet, command_id):
    data = deepcopy(fleet.data)
    stop = data["commands"][command_id]["stop"]
    now = datetime.now(UTC)
    stop["sent_at"] = (now - timedelta(seconds=10)).isoformat()
    stop["expires_at"] = (now - timedelta(seconds=5)).isoformat()
    fleet.save(data)


@pytest.mark.parametrize("restart", [False, True])
def test_stop_retries_same_envelope_after_failure_before_publish(fleet, restart):
    command = start_held(fleet)
    cid = command["command_id"]

    class OfflineBeforeSend(MemoryTransport):
        def publish(self, destination, message):
            assert message["type"] == "robot_stop_command"
            self.ready = False
            raise ConnectionError("disconnected before send")

    FleetRuntime(fleet, OfflineBeforeSend()).poll(0)
    saved = deepcopy(fleet.data["commands"][cid]["stop"])
    assert not fleet.data["commands"][cid].get("stop_published")
    reopened = None
    if restart:
        path = fleet.store.path
        fleet.store.close()
        reopened = Journal(path)
        fleet = FleetControl(fleet.config, reopened, clock=fleet.clock)
    try:
        transport = MemoryTransport()
        runtime = FleetRuntime(fleet, transport)
        runtime.poll(0)
        runtime.poll(0)
        assert [m for _, m in transport.sent if m["type"] == "robot_stop_command"] == [
            saved
        ]
        assert fleet.data["commands"][cid]["stop_published"] is True
        assert fleet.data["fault"] == "operator_stop"
        assert not fleet.data["enabled"]
    finally:
        if reopened:
            reopened.close()


def test_expired_stop_requires_operator_to_reissue_for_current_command(fleet):
    cmd = start_held(fleet)
    cid, robot = cmd["command_id"], cmd["target"]
    transport = MemoryTransport()
    runtime = FleetRuntime(fleet, transport)
    runtime.poll(0)
    age_stop(fleet, cid)
    stop_id = fleet.data["commands"][cid]["stop"]["command_id"]
    state = fleet.observed[robot]
    state["active_command_id"] = cid
    runtime.poll(0)
    assert sum(m["type"] == "robot_stop_command" for _, m in transport.sent) == 1
    with pytest.raises(ValueError, match="pending stop changed"):
        recover_stop(fleet, cid, "wrong-stop", state["boot_version"], "checked")
    recover_stop(fleet, cid, stop_id, state["boot_version"], "checked")
    new = fleet.data["commands"][cid]["stop"]
    assert new["command_id"] != stop_id
    assert new["payload"]["target_command_id"] == cid
    runtime.poll(0)
    assert [m for _, m in transport.sent if m["type"] == "robot_stop_command"][
        -1
    ] == new
    assert not fleet.data["enabled"]
    with pytest.raises(ValueError, match="requested stop is not confirmed"):
        reconcile(fleet, cid, "removed", "checked goods")


def test_new_boot_does_not_receive_old_stop_and_requires_separate_removal(fleet):
    cmd = start_held(fleet)
    cid, robot = cmd["command_id"], cmd["target"]
    state = deepcopy(fleet.observed[robot])
    state["boot_version"] = "new-boot"
    fleet.observe({"source": robot, "payload": state})
    transport = MemoryTransport()
    FleetRuntime(fleet, transport).poll(0)
    assert not any(m["type"] == "robot_stop_command" for _, m in transport.sent)
    stop_id = fleet.data["commands"][cid]["stop"]["command_id"]
    fleet.heard[robot] -= 4
    with pytest.raises(ValueError, match="fresh state"):
        recover_stop(fleet, cid, stop_id, "new-boot", "checked")
    state["sequence"] += 1
    fleet.observe({"source": robot, "payload": state})
    recover_stop(fleet, cid, stop_id, "new-boot", "verified parked and stopped")
    assert cid in fleet.data["commands"] and not fleet.data["enabled"]
    with pytest.raises(ValueError, match="requested stop is not confirmed"):
        reconcile(fleet, cid, "completed", "checked")
    reconcile(fleet, cid, "removed", "checked goods")
    assert fleet.data["recoveries"][cid]["event_id"] is None
    assert fleet.data["fault"] == "operator_stop"
    assert not fleet.data["enabled"]


def test_original_observation_expires_before_receive_age(fleet):
    cmd = start_held(fleet)
    robot = cmd["target"]
    clock = [fleet.clock()]
    fleet.clock = lambda: clock[0]
    state = deepcopy(fleet.observed[robot])
    state["sequence"] += 1
    state["observed_at"] = (datetime.now(UTC) - timedelta(seconds=2.9)).isoformat()
    message = wire.build_message(
        "robot_state", robot, fleet.config["control_id"], state
    )
    FleetRuntime(fleet, MemoryTransport()).handle(
        Delivery(wire.topic(robot, "event"), wire.encode(message), False, 1, 1, 0)
    )
    assert fleet.heard[robot] == clock[0]
    assert 2.8 < fleet.state_age(robot) <= 3
    clock[0] += 0.3
    with pytest.raises(ValueError, match="unavailable/stale"):
        fleet.ready(robot)

    # Same timestamp budget applies to recovery, even when no STOP is required.
    from test_fleet import complete

    complete(fleet, cmd, observe=False)
    with pytest.raises(ValueError, match="fresh stopped idle"):
        reconcile(fleet, cmd["command_id"], "completed", "checked")
