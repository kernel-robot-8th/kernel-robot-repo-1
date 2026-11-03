"""Explicit source/mock cycle through control producer, Host and loopback AI UDP.

No motor adapter, learned model, physical hold, contact simulation or customer
receipt is implemented by this demonstration. Its poses and timeouts are fixtures.
"""

import argparse
import json
import socket
import sys
import tempfile
from copy import deepcopy
from pathlib import Path
from threading import Thread

REPO = Path(__file__).resolve().parents[3]
for module in ("roport-omx", "roport-control-server", "roport-ai-server"):
    sys.path.insert(0, str(REPO / "services" / module / "src"))

from dedicated_demo import MockClock, mock_envelopes, mock_observe, synthetic_profile
from dedicated_policy import PolicyRouter, decode_datagram, encode_datagram
from dedicated_udp import serve_once
from roport_control_server.adapters.mqtt.messages import decode, encode
from roport_control_server.domain.dedicated_cycle import DedicatedCycle
from roport_control_server.executor.dedicated import PermitRelay, make_dedicated_command

from roport_omx.dedicated import PHASES, VERDICTS, DedicatedHost, Profile, Rejected


class ScriptedFixture:
    """Return the observed fixture position only; this is not a learned ACT model."""

    def reset(self):
        pass

    def infer(self, observation):
        return [observation["positions"]]


def make_router(profile):
    policies, metadata = {}, {}
    for operation, phases in profile["policies"].items():
        for phase, policy in phases.items():
            policy_id = policy["policy_id"]
            policies[policy_id] = ScriptedFixture()
            metadata[policy_id] = {
                "operation": operation,
                "assigned_arm_id": profile["roles"][operation],
                "phase": phase,
                "model_version": policy["model_version"],
                **{
                    k: deepcopy(profile[k])
                    for k in (
                        "policy_bundle_version",
                        "feature_schema_version",
                        "calibration_version",
                        "feature_names",
                        "use_degrees",
                    )
                },
            }
    return PolicyRouter(policies, metadata)


def _assert_outbound_refused(cycle, profile, clock, stage):
    """Control must refuse OUTBOUND until INBOUND is stored and P7 is clear."""
    assignment = mock_envelopes(clock, "OUTBOUND")[0]["payload"]
    command = make_dedicated_command(
        assignment,
        site_id=profile["site_id"],
        source=profile["control_id"],
        task_id="mock-task",
        step_id="mock-early-OUTBOUND",
        command_id="mock-early-" + stage,
        execution_id="mock-early-exec-" + stage,
        now=clock.wall(),
        expires_at=clock.wall() + 60,
        roles=profile["roles"],
    )
    try:
        cycle.begin(
            command,
            p7_cart_id="pinky3",
            cart_empty=True,
            host_boot_id="mock-host-boot",
            pinky_boot_id="mock-pinky-boot",
            docking_execution_id="mock-early-docking",
            evidence_seq=0,
            observed_at=clock.stamp(),
            now=clock.wall(),
        )
    except ValueError as exc:
        assert str(exc) == "outbound_requires_p7_clear_and_empty_pinky3", exc
        return
    raise AssertionError("OUTBOUND begin accepted " + stage)


def exchange(router, request, clock):
    """Exercise actual UDP framing on ephemeral loopback sockets only."""
    errors = []
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client,
    ):
        server.bind(("127.0.0.1", 0))
        client.bind(("127.0.0.1", 0))
        server.settimeout(2)
        client.settimeout(2)

        def respond():
            try:
                serve_once(
                    server,
                    router,
                    allowed_peer=client.getsockname(),
                    clock=clock.wall,
                )
            except (OSError, ValueError) as exc:
                errors.append(exc)

        worker = Thread(target=respond)
        worker.start()
        try:
            client.sendto(encode_datagram(request), server.getsockname())
            raw, peer = client.recvfrom(60_001)
            if peer != server.getsockname():
                raise ValueError("unexpected_policy_peer")
        finally:
            worker.join(timeout=3)
        if errors:
            raise errors[0]
        return decode_datagram(raw)


def run_cycle(*, negative_checks=False, replay_prior=False):
    """negative_checks: assert control rejects out-of-order OUTBOUND begins.

    replay_prior: replay INBOUND's last confirmation into OUTBOUND's first WAIT and
    stop there; the Host must reject it and HOLD (state is returned, not DELIVERY_READY).
    """
    profile, clock = synthetic_profile(), MockClock()
    prior_confirmation = None
    router = make_router(profile)
    cycle = DedicatedCycle(max_evidence_age=2, clock_skew=0.1)
    trace, seq = [], 1
    with tempfile.TemporaryDirectory() as directory:
        host = DedicatedHost(
            Profile.from_dict(profile),
            Path(directory) / "mock-journal.sqlite",
            host_boot_id="mock-host-boot",
            wall_clock=clock.wall,
            monotonic_clock=clock.monotonic,
        )
        try:
            for operation in ("INBOUND", "OUTBOUND"):
                assignment = mock_envelopes(clock, operation)[0]["payload"]
                command = make_dedicated_command(
                    assignment,
                    site_id=profile["site_id"],
                    source=profile["control_id"],
                    task_id="mock-task",
                    step_id="mock-" + operation,
                    command_id="mock-command-" + operation,
                    execution_id="mock-execution-" + operation,
                    now=clock.wall(),
                    expires_at=clock.wall() + 60,
                    roles=profile["roles"],
                )
                docking = "mock-docking-" + operation
                cycle.begin(
                    command,
                    p7_cart_id=assignment["pinky_id"],
                    cart_empty=operation == "OUTBOUND",
                    host_boot_id="mock-host-boot",
                    pinky_boot_id="mock-pinky-boot",
                    docking_execution_id=docking,
                    evidence_seq=0,
                    observed_at=clock.stamp(),
                    now=clock.wall(),
                )
                evidence = {
                    "pinky_id": assignment["pinky_id"],
                    "pinky_boot_id": "mock-pinky-boot",
                    "docking_execution_id": docking,
                    "evidence_seq": 1,
                    "source_observed_at": clock.stamp(),
                    "docked": True,
                    "stopped": True,
                }
                relay = PermitRelay(
                    command,
                    control_boot_id="mock-control-boot",
                    host_boot_id="mock-host-boot",
                    pinky_boot_id="mock-pinky-boot",
                    docking_execution_id=docking,
                    max_source_age=5,
                    permit_ttl=2,
                    clock_skew=0.1,
                )
                mock_observe(host, clock, seq)
                host.update_permit(
                    decode(encode(relay.relay(evidence, now=clock.wall())))
                )
                host.accept_command(decode(encode(command)))
                trace.append(operation + ":accepted")
                for index, phase in enumerate(PHASES):
                    host.acknowledge_reset(
                        exchange(router, host.reset_request(), clock)
                    )
                    host.validate_candidate(
                        exchange(router, host.infer_request(), clock)
                    )
                    target = host.next_action()
                    assert target == (0, 0, 0, 0, 0, 50)
                    host.phase_boundary()
                    # WAIT also resets AI state before accepting physical confirmation.
                    host.acknowledge_reset(
                        exchange(router, host.reset_request(), clock)
                    )
                    clock.advance(0.02)
                    seq += 1
                    mock_observe(host, clock, seq)
                    confirmation = {
                        "execution_id": command["execution_id"],
                        "phase": phase,
                        "phase_revision": host.phase_revision,
                        "verdict_kind": VERDICTS[index],
                        "confirmer": "mock-supervisor",  # fixture, not a real person
                        "positive": True,
                        "observation_seq": seq,
                        "captured_at": clock.stamp(),
                    }
                    if index < len(PHASES) - 1:
                        confirmation["next_policy_id"] = profile["policies"][operation][
                            PHASES[index + 1]
                        ]["policy_id"]
                    if replay_prior and prior_confirmation and index == 0:
                        try:
                            host.confirm(prior_confirmation)
                        except Rejected:
                            assert host.state == "HOLD"
                            trace.append(operation + ":prior-confirmation-rejected")
                            return {
                                "source_mock_only": True,
                                "physical_motion": False,
                                "state": host.state,
                                "trace": trace,
                            }
                        raise AssertionError("prior execution confirmation accepted")
                    host.confirm(confirmation)
                    prior_confirmation = confirmation
                    trace.append(operation + ":" + host.state)
                event = host.result_event()
                cycle.completed(decode(encode(event)), now=clock.wall())
                assert host.status()["departure_allowed"] is False
                if negative_checks and operation == "INBOUND":
                    _assert_outbound_refused(cycle, profile, clock, "after-completed")
                cycle.approve_departure(
                    {**evidence, "source_observed_at": clock.stamp()}, now=clock.wall()
                )
                trace.append(operation + ":control-departure-ready")
                if negative_checks and operation == "INBOUND":
                    _assert_outbound_refused(cycle, profile, clock, "before-p7-clear")
                cycle.p7_cleared(
                    {
                        **evidence,
                        "source_observed_at": clock.stamp(),
                        "evidence_seq": 2,
                        "p7_clear": True,
                        "docked": False,
                    },
                    now=clock.wall(),
                )
                trace.append(operation + ":P7-clear")
                clock.advance()
                seq += 1
        finally:
            host.close()
    return {
        "source_mock_only": True,
        "physical_motion": False,
        "transport": "loopback UDP; MQTT codec in-process",
        "state": cycle.state,
        "trace": trace,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mock", action="store_true", required=True)
    parser.parse_args()
    print(json.dumps(run_cycle(), indent=2))


if __name__ == "__main__":
    main()
