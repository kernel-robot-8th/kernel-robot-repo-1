"""Synthetic source/mock safety contracts. Never opens a device."""

import sqlite3
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from dedicated_demo import (
    MockClock,
    mock_envelopes,
    mock_observe,
    run_mock,
    synthetic_profile,
)

from roport_omx.dedicated import PHASES, DedicatedHost, Profile, Rejected


class DedicatedTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "journal.sqlite"
        self.clock = MockClock()
        self.host = self.create_host()
        self.command, self.permit = mock_envelopes(self.clock)
        mock_observe(self.host, self.clock, 1)
        self.host.update_permit(self.permit)

    def create_host(self, profile=None):
        host = DedicatedHost(
            Profile.from_dict(profile or synthetic_profile()),
            self.path,
            host_boot_id="mock-host-boot",
            wall_clock=self.clock.wall,
            monotonic_clock=self.clock.monotonic,
        )
        self.addCleanup(host.close)
        return host

    def start(self):
        self.host.accept_command(self.command)

    def reset(self):
        request = self.host.reset_request()
        self.host.acknowledge_reset({**request, "type": "policy.reset.ack"})
        return request

    def candidate(self, actions=None):
        request = self.host.infer_request()
        return {
            **request,
            "type": "action.candidate",
            "action_seq": 0,
            "actions": actions or [[0, 0, 0, 0, 0, 50]],
        }

    def evidence(self):
        if self.host.state.startswith("WAIT_") and not self.host.reset_acknowledged:
            self.reset()
        self.clock.advance()
        seq = self.host.observations["omx1"]["seq"] + 1
        mock_observe(self.host, self.clock, seq)
        return {
            "execution_id": self.command["execution_id"],
            "phase": self.host.phase,
            "phase_revision": self.host.phase_revision,
            "positive": True,
            "observation_seq": seq,
            "captured_at": self.clock.stamp(),
            "next_policy_id": self.host.profile["policies"]["INBOUND"]["TRANSFERRING"][
                "policy_id"
            ],
        }

    def assert_held(self, action):
        with self.assertRaises(Rejected):
            action()
        self.assertEqual(self.host.state, "HOLD")
        self.assertFalse(self.host.queue)
        self.assertFalse(self.host.status()["departure_allowed"])

    def test_normal_both_operations(self):
        for operation in ("INBOUND", "OUTBOUND"):
            result = run_mock(operation)
            self.assertEqual(result["state"], "SAFE_RETURNED")
            self.assertEqual(result["phase_revision"], 8)
            self.assertFalse(result["departure_allowed"])

    def test_durable_before_accept_and_duplicate_conflict(self):
        self.start()
        with sqlite3.connect(self.path) as journal:
            self.assertEqual(
                journal.execute("SELECT state FROM executions").fetchone()[0],
                "GRASPING",
            )
        self.assertTrue(self.host.accept_command(deepcopy(self.command))["duplicate"])
        changed = deepcopy(self.command)
        changed["payload"]["order_id"] = "other"
        self.assert_held(lambda: self.host.accept_command(changed))

    def test_transport_retry_dedupes_and_returns_stored_reason(self):
        self.start()
        self.host.hold("mock operator stop")
        retry = deepcopy(self.command)
        self.clock.advance()
        retry.update(message_id="new-message-id", sent_at=self.clock.stamp())
        result = self.host.accept_command(retry)
        self.assertTrue(result["duplicate"])
        self.assertEqual(result["reason"], "mock operator stop")
        self.assertEqual(result["result"]["payload"]["reason"], "mock operator stop")

    def test_duplicate_lookup_cannot_bypass_route_validation(self):
        self.start()
        retry = deepcopy(self.command)
        retry["source"] = "untrusted"
        with self.assertRaises(Rejected):
            self.host.accept_command(retry)

    def test_wait_reset_required_before_confirmation(self):
        self.start()
        self.host.phase_boundary()
        evidence = {
            "execution_id": self.command["execution_id"],
            "phase": self.host.phase,
            "phase_revision": self.host.phase_revision,
            "positive": True,
            "observation_seq": 2,
            "captured_at": self.clock.stamp(),
        }
        self.assert_held(lambda: self.host.confirm(evidence))

    def test_journal_restart_does_not_reexecute(self):
        self.start()
        recovered = self.create_host()
        self.assertEqual(recovered.state, "RECONCILING")
        duplicate = recovered.accept_command(self.command)
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(duplicate["state"], "RECONCILING")
        with self.assertRaises(Rejected):
            recovered.reset_request()

    def test_initial_missing_permit_rejected(self):
        self.host.permit = None
        with self.assertRaisesRegex(Rejected, "initial permit"):
            self.start()
        self.assertEqual(self.host.state, "READY")

    def test_missing_profile_values_and_nonfinite_config(self):
        for key in (
            "timing",
            "max_step",
            "joint_bounds",
            "joint_directions",
            "safe_positions",
            "site_id",
            "control_id",
        ):
            with self.subTest(key=key):
                profile = synthetic_profile()
                del profile[key]
                with self.assertRaises(Rejected):
                    Profile.from_dict(profile)
        profile = synthetic_profile()
        profile["timing"]["clock_skew"] = float("nan")
        with self.assertRaises(Rejected):
            Profile.from_dict(profile)

    def test_wrong_mode_multi_package_roles_and_versions(self):
        changes = [
            {"manipulation_mode": None},
            {"operation": "COOPERATIVE"},
            {"robot_ids": ["omx1", "omx2"]},
            {"assigned_arm_id": "omx2"},
            {"expected_package_ids": ["mock-package", "another"]},
            {"pinky_id": "pinky3"},
            {"calibration_version": "other"},
        ]
        for change in changes:
            with self.subTest(change=change):
                command = deepcopy(self.command)
                command["payload"].update(change)
                with self.assertRaises(Rejected):
                    self.host.accept_command(command)
        self.assertEqual(self.host.state, "READY")

    def test_first_packet_controller_route_is_profile_bound(self):
        for key, value in (
            ("site_id", "other"),
            ("source", "rogue"),
            ("target", "host"),
        ):
            for message in (self.command, self.permit):
                altered = deepcopy(message)
                altered[key] = value
                with self.assertRaises(Rejected):
                    if altered["type"] == "workcell.permit":
                        self.host.update_permit(altered)
                    else:
                        self.host.accept_command(altered)

    def test_retained_rejected_even_duplicate(self):
        self.start()
        self.assert_held(lambda: self.host.accept_command(self.command, retained=True))

    def test_duplicate_permit_never_extends_monotonic_lease(self):
        self.start()
        deadline = self.host.permit_deadline
        self.clock.advance(0.5)
        duplicate = deepcopy(self.permit)
        duplicate["sent_at"] = self.clock.stamp()
        duplicate["expires_at"] = self.clock.stamp(10)
        self.assertFalse(self.host.update_permit(duplicate))
        self.assertEqual(self.host.permit_deadline, deadline)
        self.clock.advance(1.6)
        self.assertEqual(self.host.tick(), "HOLD")

    def test_renewed_permit_cannot_clear_hold(self):
        self.start()
        self.host.hold("operator stop")
        self.clock.advance()
        renewed = deepcopy(self.permit)
        renewed["payload"].update(
            evidence_seq=2,
            source_observed_at=self.clock.stamp(),
            expires_at=self.clock.stamp(5),
        )
        renewed.update(sent_at=self.clock.stamp(), expires_at=self.clock.stamp(5))
        self.assertTrue(self.host.update_permit(renewed))
        self.assertEqual(self.host.state, "HOLD")
        self.assert_held(lambda: self.host.next_action())

    def test_revocation_and_real_boolean_requirement(self):
        self.start()
        altered = deepcopy(self.permit)
        altered["payload"]["allowed"] = 1
        self.assert_held(lambda: self.host.update_permit(altered))

    def test_boot_change_holds(self):
        self.start()
        altered = deepcopy(self.permit)
        altered["payload"].update(evidence_seq=2, pinky_boot_id="reboot")
        self.assert_held(lambda: self.host.update_permit(altered))

    def test_future_and_out_of_order_permit_hold(self):
        self.start()
        altered = deepcopy(self.permit)
        altered["payload"].update(evidence_seq=0)
        self.assert_held(lambda: self.host.update_permit(altered))

    def test_future_permit_rejected(self):
        self.start()
        altered = deepcopy(self.permit)
        altered["payload"].update(
            evidence_seq=2, source_observed_at=self.clock.stamp(1)
        )
        self.assert_held(lambda: self.host.update_permit(altered))

    def test_no_source_sequence_progress_cannot_renew(self):
        self.start()
        altered = deepcopy(self.permit)
        altered["payload"].update(evidence_seq=2)
        self.assert_held(lambda: self.host.update_permit(altered))

    def test_wall_rollback_does_not_bypass_local_lease(self):
        self.start()
        self.clock.elapsed += 3
        self.clock.now -= 1
        self.assertEqual(self.host.tick(), "HOLD")

    def test_unsafe_inactive_arm_and_out_of_limit_measured_position_hold(self):
        self.start()
        self.clock.advance()
        self.assert_held(
            lambda: self.host.observe(
                "omx2",
                [2, 0, 0, 0, 0, 50],
                observation_seq=2,
                captured_at=self.clock.stamp(),
            )
        )

    def test_reset_required_before_inference(self):
        self.start()
        self.assert_held(lambda: self.host.infer_request())

    def test_stale_reset_ack_cannot_unlock_phase(self):
        self.start()
        old = self.reset()
        self.host.phase_boundary()
        self.host.confirm(self.evidence())
        self.host.reset_request()
        self.assert_held(
            lambda: self.host.acknowledge_reset({**old, "type": "policy.reset.ack"})
        )

    def test_stale_wrong_phase_and_nan_candidates(self):
        for key, value in (
            ("observation_seq", -1),
            ("phase_revision", 0),
            ("operation", "OUTBOUND"),
            ("model_version", "other"),
            ("actions", [[float("nan")] * 6]),
        ):
            with self.subTest(key=key):
                directory = tempfile.TemporaryDirectory()
                try:
                    host = DedicatedHost(
                        Profile(synthetic_profile()),
                        Path(directory.name) / "j.sqlite",
                        host_boot_id="mock-host-boot",
                        wall_clock=self.clock.wall,
                        monotonic_clock=self.clock.monotonic,
                    )
                    mock_observe(host, self.clock, 1)
                    host.update_permit(self.permit)
                    host.accept_command(self.command)
                    reset = host.reset_request()
                    host.acknowledge_reset({**reset, "type": "policy.reset.ack"})
                    request = host.infer_request()
                    reply = {
                        **request,
                        "type": "action.candidate",
                        "action_seq": 0,
                        "actions": [[0, 0, 0, 0, 0, 50]],
                        key: value,
                    }
                    with self.assertRaises(Rejected):
                        host.validate_candidate(reply)
                    self.assertEqual(host.state, "HOLD")
                finally:
                    host.close()
                    directory.cleanup()

    def test_action_dimensions_and_step_limits(self):
        self.start()
        self.reset()
        reply = self.candidate([[6, 0, 0, 0, 0, 50]])
        self.assert_held(lambda: self.host.validate_candidate(reply))

    def test_action_count_never_completes_phase_and_targets_are_not_measured(self):
        self.start()
        self.reset()
        candidate = self.candidate([[1, 0, 0, 0, 0, 50]])
        self.host.validate_candidate(candidate)
        self.assertEqual(self.host.next_action()[0], 1)
        self.assertIsNone(self.host.next_action())
        self.assertEqual(self.host.state, "GRASPING")
        self.assertEqual(self.host.observations["omx1"]["positions"][0], 0)

    def test_inference_timeout_holds(self):
        self.start()
        self.reset()
        self.host.infer_request()
        self.clock.advance(1.01)
        self.assertEqual(self.host.tick(), "HOLD")
        self.assertIn("timeout", self.host.reason)

    def test_wait_delayed_confirmation_clears_old_queue_and_advances_revision(self):
        self.start()
        self.reset()
        self.host.validate_candidate(self.candidate())
        self.host.phase_boundary()
        self.assertFalse(self.host.queue)
        self.clock.advance(0.5)
        evidence = self.evidence()
        self.assertEqual(self.host.confirm(evidence), "TRANSFERRING")
        self.assertEqual(self.host.phase_revision, 3)
        self.assertFalse(self.host.reset_acknowledged)
        self.assertEqual(
            self.host.observations["omx1"]["positions"], (0, 0, 0, 0, 0, 50)
        )

    def test_old_confirmation_and_negative_confirmation_hold(self):
        self.start()
        self.host.phase_boundary()
        evidence = self.evidence()
        evidence["positive"] = False
        self.assert_held(lambda: self.host.confirm(evidence))

    def test_confirmation_requires_new_observation(self):
        self.start()
        self.host.phase_boundary()
        evidence = {
            "execution_id": self.command["execution_id"],
            "phase": self.host.phase,
            "phase_revision": 1,
            "positive": True,
            "observation_seq": 1,
            "captured_at": self.clock.stamp(),
        }
        self.assert_held(lambda: self.host.confirm(evidence))

    def test_wait_timeout_holds_under_continuing_fresh_permit_and_arms(self):
        self.start()
        self.host.phase_boundary()
        for seq in (2, 3, 4):
            self.clock.advance(1)
            permit = deepcopy(self.permit)
            permit["payload"].update(
                evidence_seq=seq,
                source_observed_at=self.clock.stamp(),
                expires_at=self.clock.stamp(5),
            )
            permit.update(sent_at=self.clock.stamp(), expires_at=self.clock.stamp(5))
            self.host.update_permit(permit)
            mock_observe(self.host, self.clock, seq)
        self.assertEqual(self.host.tick(), "HOLD")
        self.assertIn("confirmation timeout", self.host.reason)

    def test_next_policy_entry_envelope_checked(self):
        self.start()
        self.host.profile["policies"]["INBOUND"]["TRANSFERRING"]["entry_bounds"][0] = [
            1,
            2,
        ]
        self.host.phase_boundary()
        evidence = self.evidence()
        self.assert_held(lambda: self.host.confirm(evidence))

    def test_queue_local_deadline_survives_wall_clock_rollback(self):
        self.start()
        self.reset()
        self.host.validate_candidate(self.candidate())
        self.clock.elapsed += 1.01
        self.clock.now -= 0.05
        self.assert_held(lambda: self.host.next_action())

    def test_nonobject_candidate_holds(self):
        self.start()
        self.reset()
        self.host.infer_request()
        self.assert_held(lambda: self.host.validate_candidate([]))

    def test_wrong_action_dimension_holds(self):
        self.start()
        self.reset()
        self.assert_held(
            lambda: self.host.validate_candidate(self.candidate([[0] * 5]))
        )

    def test_completed_execution_can_accept_next_dedicated_operation(self):
        self.start()
        seq = 1
        for index, phase in enumerate(PHASES):
            self.reset()
            self.host.phase_boundary()
            self.reset()
            self.clock.advance()
            seq += 1
            mock_observe(self.host, self.clock, seq)
            evidence = {
                "execution_id": self.command["execution_id"],
                "phase": phase,
                "phase_revision": self.host.phase_revision,
                "positive": True,
                "observation_seq": seq,
                "captured_at": self.clock.stamp(),
            }
            if index < 3:
                evidence["next_policy_id"] = self.host.profile["policies"]["INBOUND"][
                    PHASES[index + 1]
                ]["policy_id"]
            self.host.confirm(evidence)
        retry = deepcopy(self.command)
        retry["message_id"] = "retry"
        event = self.host.result_event()
        self.assertEqual(event["type"], "shelf.execute.completed")
        self.assertEqual(event["boot_id"], "mock-host-boot")
        self.assertEqual(event["observed_at"], self.clock.stamp())
        self.clock.advance(0.01)
        self.assertEqual(self.host.result_event(), event)
        self.assertTrue(event["payload"]["active_arm_safe"])
        self.assertTrue(event["payload"]["inactive_arm_safe"])
        self.assertEqual(event["payload"]["location"], "slot")
        for field in ("command_id", "execution_id", "task_id", "step_id"):
            self.assertEqual(event[field], self.command[field])
        for field, value in self.command["payload"].items():
            self.assertEqual(event["payload"][field], value)
        self.assertEqual(self.host.accept_command(retry)["result"], event)
        recovered = self.create_host()
        self.assertEqual(recovered.accept_command(retry)["result"], event)
        command, permit = mock_envelopes(self.clock, "OUTBOUND")
        permit["payload"]["docking_execution_id"] = "new-dock"
        self.host.update_permit(permit)
        self.assertEqual(self.host.accept_command(command)["state"], "GRASPING")

    def test_queue_expiry_before_each_target(self):
        self.start()
        self.reset()
        self.host.validate_candidate(self.candidate())
        self.clock.advance(1.01)
        self.assert_held(lambda: self.host.next_action())


if __name__ == "__main__":
    unittest.main()
