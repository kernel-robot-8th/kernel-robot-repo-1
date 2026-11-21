import json
import socket
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest
from test_fleet import dispatch, submit
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_fleet_reassignment import final_return_without_state
from test_pinky_trial import profile  # noqa: F401

from roport_control_server.executor.fleet_socket import FleetSocket


def request(owner, **payload):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as peer:
        peer.settimeout(1)
        peer.connect(str(owner.path))
        peer.sendall(json.dumps(payload).encode())
        owner.poll()
        owner.poll()
        return json.loads(peer.recv(65536))


@pytest.mark.parametrize("expiry", [None, 0])
def test_stop_does_not_expire_but_still_requires_current_owner_session(fleet, expiry):
    with TemporaryDirectory(prefix="rp-fleet-") as directory:
        owner = FleetSocket(Path(directory) / "s", fleet)
        try:
            payload = {
                "action": "stop",
                "session_id": owner.session_id,
                "expires_at": expiry,
            }
            for invalid in (
                {**payload, "session_id": "previous-owner"},
                {**payload, "action": "enable"},
                {**payload, "unexpected": True},
                {key: value for key, value in payload.items() if key != "expires_at"},
            ):
                assert not request(owner, **invalid)["ok"]
                assert fleet.data["fault"] is None
            assert request(owner, **payload)["ok"]
            assert fleet.data["fault"] == "operator_stop" and not fleet.data["enabled"]
        finally:
            owner.close()


def test_restarted_arrival_confirmation_is_bound_to_incident_and_preserves_hold(fleet):
    command, state = final_return_without_state(fleet)
    robot, node = command["target"], command["payload"]["goal"]["node"]
    fleet.hold("operator_stop")
    queued = submit(fleet, "next-inbound")
    state.update(boot_version="new-boot", sequence=1)
    state["state"].update(node=node, arrival_command_id=None, stopped=False)
    fleet.observe({"source": robot, "payload": state})

    with TemporaryDirectory(prefix="rp-fleet-") as directory:
        owner = FleetSocket(Path(directory) / "s", fleet)
        try:
            snapshot = request(owner, action="snapshot")["snapshot"]
            assert snapshot["pending_arrivals"][robot]["recovery_boot"] == "new-boot"

            def operate(action, **fields):
                return request(
                    owner,
                    **{
                        "action": action,
                        "session_id": snapshot["session_id"],
                        "expires_at": time.time() + 1,
                        **fields,
                    },
                )

            fields = {
                "robot": robot,
                "command_id": command["command_id"],
                "boot_version": "new-boot",
                "note": "checked stopped robot at arrival",
            }
            assert not operate("confirm_arrival", **fields)["ok"]
            state["sequence"] += 1
            state["state"].update(stopped=True, heading_rad=1.2)
            fleet.observe({"source": robot, "payload": state})
            for bad in (
                {"session_id": "old-owner"},
                {"expires_at": time.time() - 1},
                {"boot_version": "different-boot"},
                {"command_id": "different-arrival"},
                {"note": ""},
            ):
                assert not operate("confirm_arrival", **(fields | bad))["ok"]
            # On receipt the state was fresh, but its original observation has expired.
            fleet.observe(
                {
                    "source": robot,
                    "payload": {**state, "sequence": state["sequence"] + 1},
                },
                age=2.9,
            )
            fleet.clock = lambda: 1.2
            assert not operate("confirm_arrival", **fields)["ok"]
            state["sequence"] += 2
            fleet.observe({"source": robot, "payload": state})
            assert not operate("release_hold", note="checked")["ok"]
            assert robot in fleet.data["pending_arrivals"]

            assert operate("confirm_arrival", **fields)["ok"]
            assert robot not in fleet.data["pending_arrivals"]
            assert fleet.data["arrival_recoveries"][command["command_id"]] == {
                "robot": robot,
                "node": node,
                "boot_version": "new-boot",
                "note": fields["note"],
            }
            assert fleet.data["fault"] == "operator_stop"
            assert dispatch(fleet) is None
            assert operate("release_hold", note="checked fleet")["ok"]
            assert dispatch(fleet) is None
            assert operate("enable")["ok"]
            after = dispatch(fleet)
            assert after["payload"]["start_node"] == node
            assert after["payload"]["start_heading_rad"] == 1.2
            assert fleet.data["commands"][after["command_id"]]["task"] == queued
        finally:
            owner.close()
