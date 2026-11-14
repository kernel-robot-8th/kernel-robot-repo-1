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

    def test_independent_columns_and_nodes(self):
        for arm, node in (("omx1", "N21"), ("omx2", "N22")):
            self.observe(node)
            command = self.command(arm)
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

    def test_panel_completion_proof_must_match_actual_visit_and_be_fresh(self):
        panel = self.executors["shelf-01"]
        panel.check = lambda task: None
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
            completion=lambda task: wrong,
            stop=panel.abort,
        )
        self.host.executors["shelf-01"] = adapter
        self.host.handle(self.command())
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

    def test_existing_omx3_host_runner_adapter_completes(self):
        robot, policy = FakeRobot(), ScriptPolicy(away=4)
        runner, _ = make(robot, policy, box_until(robot, 4))
        adapter = WarehouseExecutor(
            runner,
            marker_ids={"package-17": 17},
            package_sizes={"package-17": "large"},
            model_version="sim-model",
            calibration_version="sim-cal",
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

        def delayed_check(task):
            nonlocal clock
            clock += timedelta(seconds=20)

        panel.check = delayed_check
        adapter = PanelFieldExecutor(
            panel,
            model_version="sim-model",
            calibration_version="sim-cal",
            stop=panel.abort,
        )
        self.host.executors["shelf-01"] = adapter
        with patch("roport_omx.field_executors.datetime") as clock_type:
            clock_type.now.side_effect = lambda tz: clock
            clock_type.fromisoformat = datetime.fromisoformat
            self.host.handle(self.command())
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
            )
            self.host.executors["shelf-01"] = adapter
            command = self.command(slot_id="2b")
            self.host.handle(command)
            panel.finish()
            result = self.host.poll()[0]
            self.assertEqual(
                result["payload"]["reason"], "placement_or_release_unconfirmed"
            )
        finally:
            panel.server.shutdown()
            panel.server.server_close()


if __name__ == "__main__":
    unittest.main()
