"""Actual r8 Host/executor adapters; virtual device ports never touch motors."""

import sys
import tempfile
import time
import unittest
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from pathlib import Path
from threading import current_thread, main_thread
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts")]

from field_mock import virtual_alignment, virtual_executors
from test_omx3_runner import FakeRobot, ScriptPolicy, box_until, make

from roport_omx.field_executors import (
    IndependentArmExecutor,
    PanelFieldExecutor,
    WarehouseExecutor,
)
from roport_omx.field_host import FieldOmxHost
from roport_omx.mqtt_link.field_wire import build_message
from roport_omx.mqtt_link.journal import Journal


def stationary_sample():
    """Explicit fake motor observation for instantaneous device fixtures."""
    return {"observed_at": datetime.now(UTC).isoformat(), "stopped": True}


def wait_until(read, predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = read()
        if predicate(result):
            return result
        time.sleep(0.005)
    raise AssertionError(f"condition not reached: {result}")


def expiry():
    return (datetime.now(UTC) + timedelta(seconds=10)).isoformat()


class FieldHostTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.journal = Journal(Path(self.temp.name) / "journal.json")
        self.executors = virtual_executors(0)
        self.host = FieldOmxHost(self.journal, self.executors)
        self.host.alignment = lambda command: virtual_alignment(self.host, command)
        self.observe("N11")

    def tearDown(self):
        self.journal.close()
        self.temp.cleanup()

    def observe(self, node, *, boot_version="pinky-boot", **changes):
        state = {
            "node": node,
            "arrival_command_id": "arrival-1",
            "mode": "line",
            "stopped": True,
            "line_enabled": False,
            "nav2_ready": False,
            "localization_ready": False,
            "drive_owner": "none",
            "graph_version": "v14",
            "map_version": "map-v1",
            "heading_rad": 0,
            "fault": None,
        }
        state.update(changes)
        self.host.observe_pinky(
            build_message(
                "robot_state",
                "pinky1",
                "control-01",
                {
                    "boot_version": boot_version,
                    "observed_at": datetime.now(UTC).isoformat(),
                    "sequence": self.host.pinkies.get("pinky1", {}).get("sequence", 0)
                    + 1,
                    "active_command_id": None,
                    "state": state,
                },
            )
        )

    def command(self, target="shelf-01", **changes):
        common = {
            "expected_boot_version": self.host.boot_version,
            "package_id": "package-17",
            "expected_pinky_boot_version": "pinky-boot",
            "arrival_command_id": "arrival-1",
            "model_version": "sim-model",
            "calibration_version": "sim-cal",
        }
        if target == "shelf-01":
            kind = "shelf_transfer_command"
            p = {
                **common,
                "operation": "store",
                "slot_id": "1b",
                "pinky_id": "pinky1",
                "arms": [
                    {"robot_id": a, "expected_boot_version": self.host.boot_version}
                    for a in ("omx1", "omx2")
                ],
            }
        elif target == "omx3":
            kind = "warehouse_load_command"
            p = {
                **common,
                "pickup_location_id": "pickup-01",
                "pinky_id": "pinky1",
                "load_position": "deck",
            }
        else:
            kind = "arm_action_command"
            p = {
                **common,
                "operation": "transfer",
                "workcell_id": "shelf-01",
                "source_location": {
                    "kind": "pinky",
                    "robot_id": "pinky1",
                    "position": "deck",
                },
                "destination_location": {
                    "kind": "shelf",
                    "slot_id": "1a" if target == "omx1" else "1c",
                },
            }
        p.update(changes)
        return build_message(
            kind, "control-01", target, p, command_id="command-1", expires_at=expiry()
        )

    def test_copied_mqtt_contract_matches_control_source(self):
        common = (
            ROOT.parent
            / "roport-control-server/src/roport_control_server/adapters/mqtt"
        )
        copied = ROOT / "src/roport_omx/mqtt_link"
        for name in ("field_wire.py", "mqtt.py"):
            self.assertEqual((common / name).read_bytes(), (copied / name).read_bytes())

    def test_wrong_versions_cannot_start_any_executor(self):
        for target, node in (
            ("shelf-01", "N11"),
            ("omx1", "N21"),
            ("omx2", "N22"),
            ("omx3", "N01"),
        ):
            self.observe(node)
            cmd = self.command(target, calibration_version="old-cal")
            cmd["command_id"] = target
            self.assertEqual(
                self.host.handle(cmd)[0]["payload"]["reason"],
                "executor_version_mismatch",
            )
            self.assertFalse(self.executors[target].tasks)

    def test_expiry_during_prepare_or_acceptance_never_starts_device(self):
        for stage in ("readiness", "check", "accepted_save"):
            with self.subTest(stage=stage):
                command = self.command()
                command["command_id"] = stage
                executor = self.executors["shelf-01"]
                clock = datetime.now(UTC)

                def delay():
                    nonlocal clock
                    clock += timedelta(seconds=20)

                ready, save = executor.readiness, self.journal.save

                def delayed_ready():
                    delay()
                    return ready()

                def delayed_save(data):
                    save(data)
                    if data["commands"][stage].get("status") == "accepted":
                        delay()

                with (
                    patch("roport_omx.field_host.datetime") as clock_type,
                    patch.object(
                        executor,
                        "readiness",
                        delayed_ready if stage == "readiness" else ready,
                    ),
                    patch.object(
                        executor,
                        "check",
                        lambda task: delay() if stage == "check" else None,
                        create=True,
                    ),
                    patch.object(
                        self.journal,
                        "save",
                        delayed_save if stage == "accepted_save" else save,
                    ),
                ):
                    clock_type.now.side_effect = lambda tz: clock
                    clock_type.fromisoformat = datetime.fromisoformat
                    events = self.host.handle(command)
                self.assertEqual(events[-1]["payload"]["reason"], "expired_command")
                self.assertFalse(executor.tasks)
                self.assertIsNone(self.host.data["active"])
                self.assertIsNone(self.host.data["hold"])

    def test_cooperative_completion_and_durable_ack(self):
        command = self.command()
        accepted = self.host.handle(command)[0]
        done = self.host.poll()[0]
        self.assertEqual(done["payload"]["status"], "completed")
        self.assertEqual(done["payload"]["result"]["slot_id"], "1b")
        self.assertTrue(done["payload"]["result"]["placement_confirmed"])
        self.assertEqual(len(self.executors["shelf-01"].tasks), 1)
        self.assertEqual(self.host.handle(command), [done])
        self.assertIn(done, self.journal.data["outbox"])
        ack = build_message(
            "event_stored",
            "control-01",
            "shelf-01",
            {"acked_message_id": done["message_id"]},
        )
        self.host.handle(ack)
        self.assertNotIn(done, self.host.pending())
        self.assertIn(accepted, self.host.pending())

    def test_arrival_is_consumed_across_ack_and_restart(self):
        first = self.command()
        self.host.handle(first)
        done = self.host.poll()[0]
        self.host.handle(
            build_message(
                "event_stored",
                "control-01",
                "shelf-01",
                {"acked_message_id": done["message_id"]},
            )
        )
        self.host = FieldOmxHost(self.journal, self.executors)
        self.host.alignment = lambda cmd: virtual_alignment(self.host, cmd)
        self.observe("N11")
        self.assertEqual(self.host.handle(first), [done])
        second = self.command()
        second["command_id"] = "command-2"
        self.assertEqual(
            self.host.handle(second)[0]["payload"]["reason"], "arrival_already_used"
        )
        self.assertEqual(len(self.executors["shelf-01"].tasks), 1)
        self.observe("N11", arrival_command_id="arrival-2")
        third = self.command(arrival_command_id="arrival-2")
        third["command_id"] = "command-3"
        self.assertEqual(self.host.handle(third)[0]["payload"]["status"], "accepted")
        self.assertEqual(self.host.poll()[0]["payload"]["status"], "completed")

    def test_failed_start_still_consumes_arrival_after_reconciliation(self):
        with patch.object(
            self.executors["shelf-01"], "start", side_effect=RuntimeError
        ):
            self.host.handle(self.command())
        self.host.data["hold"] = (
            None  # simulated operator reconciliation, not a wire API
        )
        self.host.data.pop("recovery_command_id", None)
        second = self.command()
        second["command_id"] = "retry"
        self.assertEqual(
            self.host.handle(second)[0]["payload"]["reason"], "arrival_already_used"
        )

    def test_rejected_precheck_does_not_consume_arrival(self):
        bad = self.command(calibration_version="old")
        bad["command_id"] = "bad-version"
        self.assertEqual(self.host.handle(bad)[0]["payload"]["status"], "rejected")
        self.assertEqual(
            self.host.handle(self.command())[0]["payload"]["status"], "accepted"
        )

    def test_old_journal_outcome_also_consumes_arrival(self):
        self.host.handle(self.command())
        self.host.poll()
        self.host.data["commands"]["command-1"].pop("arrival_consumed")
        self.host._save()
        self.host = FieldOmxHost(self.journal, self.executors)
        self.host.alignment = lambda cmd: virtual_alignment(self.host, cmd)
        self.observe("N11")
        second = self.command()
        second["command_id"] = "new-owner"
        self.assertEqual(
            self.host.handle(second)[0]["payload"]["reason"], "arrival_already_used"
        )

    def test_return_home_reports_unsupported_operation(self):
        for home in (False, True):
            cmd = build_message(
                "arm_action_command",
                "control-01",
                "omx1",
                {
                    "expected_boot_version": self.host.boot_version,
                    "operation": "return_home",
                    "workcell_id": "shelf-01",
                    "calibration_version": "sim-cal",
                },
                command_id=f"home-{home}",
                expires_at=expiry(),
            )
            self.executors["omx1"].unreturned = set() if home else {"omx1"}
            self.assertEqual(
                self.host.handle(cmd)[0]["payload"]["reason"], "unsupported_operation"
            )
            self.assertFalse(self.executors["omx1"].tasks)

    def test_state_identifies_participating_targets_only(self):
        self.host.handle(self.command())
        for target in ("shelf-01", "omx1", "omx2"):
            self.assertEqual(
                self.host.state(target)["payload"]["active_command_id"], "command-1"
            )
        other = self.host.state("omx3")["payload"]
        self.assertIsNone(other["active_command_id"])
        self.assertFalse(other["state"]["ready"])
        self.assertEqual(other["state"]["fault"], "host_busy")
        self.host.poll()
        self.observe("N01", arrival_command_id="arrival-2")
        cmd = self.command("omx3", arrival_command_id="arrival-2")
        cmd["command_id"] = "warehouse"
        self.host.handle(cmd)
        shelf = self.host.state("shelf-01")["payload"]
        self.assertIsNone(shelf["active_command_id"])
        self.assertTrue(
            all(a["active_command_id"] is None for a in shelf["state"]["arms"])
        )
        self.assertEqual(shelf["state"]["fault"], "host_busy")

    def test_old_or_future_command_refused_but_duplicate_remains_queryable(self):
        for age in (120, -2):
            cmd = self.command()
            cmd["command_id"] = f"aged-{age}"
            cmd["sent_at"] = (datetime.now(UTC) - timedelta(seconds=age)).isoformat()
            self.assertEqual(
                self.host.handle(cmd)[0]["payload"]["reason"], "stale_message"
            )
        cmd = self.command()
        accepted = self.host.handle(cmd)[0]
        with patch("roport_omx.field_host._fresh", return_value=False):
            self.assertEqual(self.host.handle(cmd), [accepted])
        self.assertEqual(len(self.executors["shelf-01"].tasks), 1)

    def test_independent_columns_and_nodes(self):
        for arm, node in (("omx1", "N21"), ("omx2", "N22")):
            self.observe(node, arrival_command_id=f"arrival-{node}")
            command = self.command(arm, arrival_command_id=f"arrival-{node}")
            command["command_id"] = arm
            self.assertEqual(
                self.host.handle(command)[0]["payload"]["status"], "accepted"
            )
            done = self.host.poll()[0]
            self.assertEqual(done["payload"]["status"], "completed")
            self.assertEqual(
                [a["robot_id"] for a in done["payload"]["result"]["arms"]], [arm]
            )

    def test_cooperative_independent_resource_conflict(self):
        self.executors["shelf-01"].run_s = 20
        self.host.handle(self.command())
        cmd = self.command("omx1")
        cmd["command_id"] = "other"
        self.assertEqual(
            self.host.handle(cmd)[0]["payload"]["reason"], "workcell_held_or_busy"
        )
        self.assertFalse(self.executors["omx1"].tasks)

    def test_wrong_column_and_wrong_arrival_cannot_move(self):
        for changes in (
            {"slot_id": "1a"},
            {"arrival_command_id": "old"},
            {"expected_pinky_boot_version": "old"},
        ):
            cmd = self.command(**changes)
            cmd["command_id"] = str(len(self.host.data["commands"]))
            self.assertEqual(self.host.handle(cmd)[0]["payload"]["status"], "rejected")
        self.assertFalse(self.executors["shelf-01"].tasks)

    def test_duplicate_body_conflict_does_not_repeat_action(self):
        cmd = self.command()
        self.host.handle(cmd)
        changed = deepcopy(cmd)
        changed["payload"]["slot_id"] = "2b"
        with self.assertRaisesRegex(ValueError, "immutable"):
            self.host.handle(changed)
        self.assertEqual(len(self.executors["shelf-01"].tasks), 1)

    def test_missing_alignment_is_not_invented(self):
        self.host.alignment = lambda cmd: None
        event = self.host.handle(self.command())[0]
        self.assertEqual(event["payload"]["reason"], "docking_alignment_unconfirmed")
        self.assertFalse(self.executors["shelf-01"].tasks)

    def test_pinky_departure_during_action_holds(self):
        self.host.handle(self.command())
        self.observe("N11", stopped=False)
        done = self.host.poll()[0]
        self.assertEqual(done["payload"]["status"], "failed")
        self.assertTrue(self.host.data["hold"])
        self.assertTrue(self.executors["shelf-01"].aborted)

    def test_arm_return_only_is_not_placement(self):
        self.executors["shelf-01"].placement = False
        self.host.handle(self.command())
        self.assertEqual(
            self.host.poll()[0]["payload"]["reason"], "placement_or_release_unconfirmed"
        )
        self.assertTrue(self.host.data["hold"])

    def test_one_arm_not_returned_and_executor_failure_hold(self):
        self.host.handle(self.command())
        self.executors["shelf-01"].unreturned.add("omx2")
        self.assertEqual(
            self.host.poll()[0]["payload"]["reason"], "participating_arm_not_returned"
        )

    def test_pinky_reboot_cannot_be_undone_by_delayed_old_boot(self):
        self.host.handle(self.command())
        self.observe("N11", boot_version="pinky-new-boot")
        self.observe("N11")
        self.assertEqual(self.host.pinkies["pinky1"]["boot_version"], "pinky-new-boot")
        self.assertEqual(self.host.poll()[0]["payload"]["status"], "failed")
        self.assertTrue(self.executors["shelf-01"].aborted)

    def test_reboot_preserves_unknown_ownership(self):
        self.host.handle(self.command())
        restored = FieldOmxHost(self.journal, self.executors)
        self.assertEqual(restored.data["hold"], "host_restarted_outcome_unknown")
        self.assertNotEqual(restored.boot_version, self.host.boot_version)
        self.assertTrue(restored.pending())

    def test_watchdog_read_only_queries_and_replays_do_not_renew(self):
        command = self.command()
        self.host.handle(command)
        self.host.deadline = 0
        self.host.handle(
            build_message("robot_state_request", "control-01", "shelf-01", {})
        )
        self.assertEqual(self.host.deadline, 0)
        keep = build_message(
            "robot_state_request",
            "control-01",
            "shelf-01",
            {
                "expected_boot_version": self.host.boot_version,
                "keep_running_command_id": command["command_id"],
            },
            expires_at=expiry(),
        )
        self.host.handle(keep)
        self.assertGreater(self.host.deadline, time.monotonic())
        self.host.deadline = 0
        self.host.handle(keep)
        self.assertEqual(
            self.host.poll()[0]["payload"]["reason"], "control_watchdog_expired"
        )

    def test_stop_one_arm_interrupts_whole_cooperative_command(self):
        original = self.command()
        self.host.handle(original)
        stop = build_message(
            "robot_stop_command",
            "control-01",
            "omx1",
            {
                "expected_boot_version": self.host.boot_version,
                "target_command_id": original["command_id"],
                "reason": "operator_request",
            },
            command_id="stop-1",
            expires_at=expiry(),
        )
        self.host.handle(stop)
        result = self.host.poll()
        self.assertEqual(
            [e["payload"]["status"] for e in result], ["interrupted", "completed"]
        )
        self.assertEqual(result[0]["command_id"], original["command_id"])
        self.assertEqual(result[1]["command_id"], stop["command_id"])

    def test_stop_without_observed_stopping_terminates_both_commands_as_failed(self):
        original = self.command()
        self.host.handle(original)
        executor = self.executors["shelf-01"]
        executor.abort = lambda: None
        executor.run_s = 20
        stop = build_message(
            "robot_stop_command",
            "control-01",
            "shelf-01",
            {
                "expected_boot_version": self.host.boot_version,
                "target_command_id": original["command_id"],
                "reason": "operator_request",
            },
            command_id="stop-1",
            expires_at=expiry(),
        )
        self.host.handle(stop)
        self.host.stop_deadline = 0.0001
        events = self.host.poll()
        self.assertEqual([e["payload"]["status"] for e in events], ["failed", "failed"])
        self.assertTrue(self.host.data["hold"])

    def stop_command(self, name="stop-1", target="shelf-01"):
        return build_message(
            "robot_stop_command",
            "control-01",
            target,
            {
                "expected_boot_version": self.host.boot_version,
                "target_command_id": "command-1",
                "reason": "operator_request",
            },
            command_id=name,
            expires_at=expiry(),
        )

    def test_local_reconciliation_preserves_history_and_never_resumes_motion(self):
        self.host.handle(self.command())
        with self.assertRaisesRegex(ValueError, "pending device stop"):
            self.host.release_hold("checked")
        self.host.handle(self.stop_command())
        self.host.poll()
        saved = deepcopy(self.host.data)
        self.executors["shelf-01"].unreturned.add("omx2")
        with self.assertRaisesRegex(ValueError, "returned"):
            self.host.release_hold("checked")
        self.assertEqual(self.host.data, saved)
        self.executors["shelf-01"].unreturned.clear()
        self.host.release_hold("operator checked both arms and removed the package")
        self.assertIsNone(self.host.data["hold"])
        self.assertEqual(self.host.data["commands"], saved["commands"])
        self.assertEqual(self.host.data["outbox"], saved["outbox"])
        self.assertEqual(len(self.executors["shelf-01"].tasks), 1)
        self.assertTrue(self.host.data["recoveries"])

    def test_second_stop_keeps_first_stop_until_confirmation(self):
        self.host.handle(self.command())
        first = self.stop_command()
        self.host.handle(first)
        self.assertEqual(self.host.handle(first)[0]["payload"]["status"], "accepted")
        second = self.host.handle(self.stop_command("stop-2"))[0]
        self.assertEqual(second["payload"]["reason"], "stop_already_pending")
        self.assertEqual(self.host.data["stop"]["command_id"], "stop-1")
        events = self.host.poll()
        self.assertEqual(events[-1]["command_id"], "stop-1")
        self.assertEqual(events[-1]["payload"]["status"], "completed")

    def test_restart_aborts_without_resuming_and_accepts_stop_for_old_command(self):
        self.host.handle(self.command())
        executor = self.executors["shelf-01"]
        self.host = FieldOmxHost(self.journal, self.executors)
        self.assertTrue(executor.aborted)
        self.assertEqual(len(executor.tasks), 1)
        self.assertEqual(self.host.poll(), [])
        self.assertEqual(self.host.data["commands"]["command-1"]["status"], "failed")
        self.assertEqual(
            self.host.state("shelf-01")["payload"]["active_command_id"], "command-1"
        )
        self.assertEqual(
            self.host.handle(self.stop_command())[0]["payload"]["status"], "accepted"
        )
        result = self.host.poll()
        self.assertEqual([e["payload"]["status"] for e in result], ["completed"])
        self.assertIsNone(self.host.data.get("recovery_command_id"))
        self.assertTrue(self.host.data["hold"])

    def test_failed_abort_keeps_command_available_for_stop_retry(self):
        self.host.handle(self.command())
        executor = self.executors["shelf-01"]
        with patch.object(
            executor, "abort", side_effect=RuntimeError("device offline")
        ):
            self.host.deadline = 0
            self.assertEqual(self.host.poll()[0]["payload"]["status"], "failed")
        self.assertEqual(
            self.host.handle(self.stop_command())[0]["payload"]["status"], "accepted"
        )
        self.assertEqual(self.host.poll()[-1]["payload"]["status"], "completed")

    def test_restart_during_stop_finishes_old_stop_and_allows_new_stop(self):
        self.host.handle(self.command())
        self.host.handle(self.stop_command())
        self.host = FieldOmxHost(self.journal, self.executors)
        self.assertNotIn("stop", self.host.data)
        self.assertEqual(self.host.data["commands"]["stop-1"]["status"], "failed")
        self.assertEqual(
            self.host.handle(self.stop_command("stop-2"))[0]["payload"]["status"],
            "accepted",
        )
        self.assertEqual(self.host.poll()[-1]["command_id"], "stop-2")

    def test_busy_panel_reports_unknown_pose_without_inventing_an_arm_fault(self):
        self.host.handle(self.command())
        panel = self.executors["shelf-01"]
        panel.readiness = lambda: {
            "armed": True,
            "busy": True,
            "error": None,
            "arms": {},
        }
        for target in ("omx1", "omx2"):
            state = self.host.state(target)["payload"]["state"]
            self.assertIsNone(state["fault"])
            self.assertFalse(state["ready"])
            self.assertFalse(state["stopped"])
            self.assertFalse(state["home_reached"])
            self.assertEqual(state["holding_state"], "unknown")
        panel.readiness = lambda: {
            "armed": True,
            "busy": True,
            "error": None,
            "arms": {"omx1": {"connected": False}},
        }
        self.assertEqual(
            self.host.state("omx1")["payload"]["state"]["fault"],
            "arm_observation_unavailable",
        )
        panel.readiness = lambda: {
            "armed": True,
            "busy": True,
            "error": "panel_read_failed",
            "arms": {},
        }
        self.assertEqual(
            self.host.state("omx2")["payload"]["state"]["fault"], "panel_read_failed"
        )

    def test_panel_completion_proof_must_match_actual_visit_and_be_fresh(self):
        panel = self.executors["shelf-01"]
        panel.check = lambda task, **kwargs: None
        panel.prepare = panel.readiness
        wrong = {
            "command_id": "command-1",
            "package_id": "wrong-package",
            "slot_id": "1b",
            "arrival_command_id": "arrival-1",
            "observed_at": datetime.now(UTC).isoformat(),
            "placement_confirmed": True,
            "release_confirmed": True,
        }
        adapter = PanelFieldExecutor(
            panel,
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=stationary_sample,
            completion=lambda task: wrong,
            stop=panel.abort,
        )
        self.addCleanup(adapter.close)
        self.host.executors["shelf-01"] = adapter
        wait_until(adapter.readiness, lambda r: r.get("armed"))
        self.host.handle(self.command())
        wait_until(adapter.observe, lambda r: r["state"] == "finished")
        valid = {**wrong, "package_id": "package-17"}
        changes = [
            {key: "another-visit"}
            for key in ("command_id", "package_id", "slot_id", "arrival_command_id")
        ] + [
            {"observed_at": (datetime.now(UTC) + timedelta(seconds=offset)).isoformat()}
            for offset in (-10, 10)
        ]
        for change in changes:
            with self.subTest(change=change):
                wrong.clear()
                wrong.update(valid | change)
                self.assertNotIn("placement_confirmed", adapter.observe())
        wrong.clear()
        wrong.update(valid)
        self.assertTrue(adapter.observe()["placement_confirmed"])

    def test_real_adapters_require_fresh_post_abort_stop_evidence(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        sample = stationary_sample()
        options = dict(
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=lambda: sample,
        )
        adapters = [
            WarehouseExecutor(runner, marker_ids={"package-17": 17}, **options),
            IndependentArmExecutor(
                "omx1",
                {("INBOUND", "1a"): runner},
                marker_ids={"package-17": 17},
                **options,
            ),
            PanelFieldExecutor(
                self.executors["shelf-01"],
                stop=self.executors["shelf-01"].abort,
                **options,
            ),
        ]
        for adapter in adapters:
            with self.subTest(adapter=type(adapter).__name__):
                sample.clear()
                sample.update(stationary_sample())
                adapter.abort()
                if isinstance(adapter, PanelFieldExecutor):
                    self.addCleanup(adapter.close)
                    adapter.stop_future.result(timeout=2)
                self.assertNotEqual(adapter.observe().get("state"), "stopped")
                for bad in (
                    {},
                    {"observed_at": "invalid", "stopped": True},
                    {
                        "observed_at": (
                            datetime.now(UTC) + timedelta(seconds=10)
                        ).isoformat(),
                        "stopped": True,
                    },
                    {
                        "observed_at": (
                            datetime.now(UTC) - timedelta(seconds=10)
                        ).isoformat(),
                        "stopped": True,
                    },
                    {"observed_at": datetime.now(UTC).isoformat(), "stopped": False},
                ):
                    sample.clear()
                    sample.update(bad)
                    self.assertNotEqual(adapter.observe().get("state"), "stopped")
                sample.clear()
                sample.update(stationary_sample())
                self.assertEqual(adapter.observe().get("state"), "stopped")
        self.assertGreater(robot.held, 0)

    def test_home_pose_without_stop_sensor_is_not_ready(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        self.host.executors["omx3"] = WarehouseExecutor(
            runner,
            marker_ids={"package-17": 17},
            model_version="sim-model",
            calibration_version="sim-cal",
        )
        self.observe("N01")
        state = self.host.state("omx3")["payload"]["state"]
        self.assertTrue(state["home_reached"])
        self.assertFalse(state["stopped"])
        self.assertFalse(state["ready"])
        self.assertEqual(
            self.host.handle(self.command("omx3"))[0]["payload"]["reason"],
            "arm_not_safely_home",
        )
        self.assertEqual(robot.sent, 0)

    def test_real_runner_stop_confirmation_reaches_host(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        sample = stationary_sample()
        adapter = WarehouseExecutor(
            runner,
            marker_ids={"package-17": 17},
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=lambda: sample,
        )
        self.host.executors["omx3"] = adapter
        self.observe("N01")
        self.host.handle(self.command("omx3"))
        adapter.host.wait(2)
        self.host.handle(self.stop_command(target="omx3"))
        self.assertEqual(self.host.poll(), [])
        sample.update(stationary_sample())
        self.assertEqual(
            [e["payload"]["status"] for e in self.host.poll()],
            ["interrupted", "completed"],
        )

    def test_stop_observation_error_keeps_recoverable_stop_target(self):
        self.host.handle(self.command())
        self.host.handle(self.stop_command())
        with patch.object(
            self.executors["shelf-01"], "observe", side_effect=OSError("sensor lost")
        ):
            self.host.poll()
        self.assertNotIn("stop", self.host.data)
        self.assertEqual(self.host.data["commands"]["stop-1"]["status"], "failed")
        self.assertEqual(
            self.host.handle(self.stop_command("stop-2"))[0]["payload"]["status"],
            "accepted",
        )

    def test_existing_omx3_host_runner_adapter_completes(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        adapter = WarehouseExecutor(
            runner,
            marker_ids={"package-17": 17},
            package_sizes={"package-17": "large"},
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=stationary_sample,
        )
        self.host.executors["omx3"] = adapter
        self.observe("N01")
        self.host.handle(self.command("omx3"))
        adapter.host.wait(2)
        done = self.host.poll()[0]
        self.assertEqual(done["payload"]["status"], "completed", done)
        self.assertEqual(done["payload"]["result"]["package_size"], "large")
        self.assertGreater(robot.sent, 0)
        self.assertEqual(
            [a["robot_id"] for a in done["payload"]["result"]["arms"]], ["omx3"]
        )

    def test_expiry_inside_runner_preparation_does_not_write_motor_targets(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        adapter = WarehouseExecutor(
            runner,
            marker_ids={"package-17": 17},
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=stationary_sample,
        )
        self.host.executors["omx3"] = adapter
        self.observe("N01")
        clock = datetime.now(UTC)
        precheck = runner._precheck

        def delayed_precheck(marker):
            nonlocal clock
            result = precheck(marker)
            if current_thread() is not main_thread():
                clock += timedelta(seconds=20)
            return result

        with (
            patch("roport_omx.field_executors.datetime") as clock_type,
            patch.object(runner, "_precheck", delayed_precheck),
            patch.object(
                adapter,
                "stop_status",
                lambda: {"observed_at": clock.isoformat(), "stopped": True},
            ),
        ):
            clock_type.now.side_effect = lambda tz: clock
            clock_type.fromisoformat = datetime.fromisoformat
            self.host.handle(self.command("omx3"))
            adapter.host.wait(2)
        self.assertEqual(robot.sent, 0)
        self.assertEqual(self.host.poll()[0]["payload"]["status"], "failed")

    def test_panel_preparation_expiry_never_starts_evaluation(self):
        panel = self.executors["shelf-01"]
        clock = datetime.now(UTC)

        def delayed_check(task, *, cached=False):
            nonlocal clock
            if not cached:
                clock += timedelta(seconds=20)

        panel.check = delayed_check
        panel.prepare = panel.readiness
        adapter = PanelFieldExecutor(
            panel,
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=stationary_sample,
            stop=panel.abort,
        )
        self.addCleanup(adapter.close)
        self.host.executors["shelf-01"] = adapter
        wait_until(adapter.readiness, lambda r: r.get("armed"))
        adapter.stop_status = lambda: {
            "observed_at": clock.isoformat(),
            "stopped": True,
        }
        with patch("roport_omx.field_executors.datetime") as clock_type:
            clock_type.now.side_effect = lambda tz: clock
            clock_type.fromisoformat = datetime.fromisoformat
            self.host.handle(self.command())
            with self.assertRaisesRegex(ValueError, "expired_command"):
                adapter.start_future.result(timeout=2)
        self.assertFalse(panel.tasks)

    def test_independent_actual_runner_adapter_uses_one_arm(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        adapter = IndependentArmExecutor(
            "omx1",
            {("INBOUND", "1a"): runner},
            marker_ids={"package-17": 17},
            model_version="sim-model",
            calibration_version="sim-cal",
            stop_status=stationary_sample,
        )
        self.host.executors["omx1"] = adapter
        self.observe("N21")
        self.host.handle(self.command("omx1"))
        adapter.current.host.wait(2)
        result = self.host.poll()[0]
        self.assertEqual(result["payload"]["status"], "completed", result)
        self.assertGreater(robot.sent, 0)
        self.assertEqual(
            [a["robot_id"] for a in result["payload"]["result"]["arms"]], ["omx1"]
        )

    def test_panel_return_without_sensor_proof_stays_unconfirmed(self):
        from test_panel_executor import ROUTES, FakePanel

        from roport_omx.panel_executor import PanelExecutor

        panel = FakePanel()
        try:
            adapter = PanelFieldExecutor(
                PanelExecutor(ROUTES, box_marker=16, base_url=panel.url),
                stop=lambda: None,
                model_version="sim-model",
                calibration_version="sim-cal",
                stop_status=stationary_sample,
            )
            self.host.executors["shelf-01"] = adapter
            command = self.command(slot_id="2b")
            wait_until(adapter.readiness, lambda r: r.get("armed"))
            self.host.handle(command)
            adapter.start_future.result(timeout=2)
            panel.finish()
            result = wait_until(
                self.host.poll, lambda r: r and r[-1]["payload"]["status"] == "failed"
            )[-1]
            self.assertEqual(
                result["payload"]["reason"], "placement_or_release_unconfirmed"
            )
        finally:
            adapter.close()
            panel.server.shutdown()
            panel.server.server_close()


if __name__ == "__main__":
    unittest.main()
