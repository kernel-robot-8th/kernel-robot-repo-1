"""shelf-01 Host on the field contract v2 (roport-control-v2) with the simulated executor.

No broker, panel or device. Every message the Host sends is checked by field_wire.validate (the
same contract module as control's), and completions against what Fleet checks
(services/roport-control-server executor/fleet.py check_completion on task/control-integration-test).
"""

import importlib.util
import sys
import tempfile
import time
import unittest
from datetime import UTC, datetime, timedelta
from pathlib import Path
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.cooperative_v2 import FieldHost
from roport_omx.mqtt_link import field_wire
from roport_omx.mqtt_link.journal import Journal

spec = importlib.util.spec_from_file_location(
    "cooperative_mock", ROOT / "scripts/cooperative_mock.py"
)
mock = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mock)

CONTROL = "control-01"
EVIDENCE = {"INBOUND": "shelf_align", "OUTBOUND": "pinky_load"}


def command(kind, target, payload, *, expires_s=5.0, sent_delay_s=0.0, command_id=None):
    now = datetime.now(UTC) - timedelta(seconds=sent_delay_s)
    return field_wire.build_message(
        kind,
        CONTROL,
        target,
        payload,
        command_id=command_id or str(uuid4()),
        expires_at=(now + timedelta(seconds=sent_delay_s + expires_s)).isoformat(),
    ) | {"sent_at": now.isoformat()}


def pinky_state(
    *, node="N11", arrival="cmd-route-001", boot="boot-pinky1", stopped=True, seq=1
):
    return field_wire.build_message(
        "robot_state",
        "pinky1",
        CONTROL,
        {
            "boot_version": boot,
            "observed_at": datetime.now(UTC).isoformat(),
            "sequence": seq,
            "active_command_id": None,
            "state": {
                "node": node,
                "arrival_command_id": arrival,
                "mode": "line",
                "stopped": stopped,
                "line_enabled": False,
                "nav2_ready": False,
                "localization_ready": False,
                "drive_owner": "none",
                "graph_version": "line-test",
                "map_version": "map-test",
                "heading_rad": 0.0,
                "fault": None,
            },
        },
    )


class FieldHostTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "shelf-01-v2.json"
        self.journal = Journal(self.path)
        self.executor = mock.MockExecutor(0.2, evidence=EVIDENCE)
        self.host = FieldHost(self.journal, self.executor, control=CONTROL)
        self.host.observe_pinky(pinky_state())

    def tearDown(self):
        self.journal.close()
        self.dir.cleanup()

    def transfer(self, operation="store", **change):
        boot = self.host.boot_version
        payload = {
            "expected_boot_version": boot,
            "operation": operation,
            "package_id": "16",
            "slot_id": "2b",
            "pinky_id": "pinky1",
            "arrival_command_id": "cmd-route-001",
            "arms": [
                {"robot_id": "omx1", "expected_boot_version": boot},
                {"robot_id": "omx2", "expected_boot_version": boot},
            ],
            "model_version": "real_act_rs1107x129_60k_1107",
            "calibration_version": "model-num-22-cal",
            "expected_pinky_boot_version": "boot-pinky1",
        } | change.pop("payload", {})
        return command("shelf_transfer_command", "shelf-01", payload, **change)

    def send(self, message, endpoint=None):
        events = self.host.handle(
            message, endpoint=endpoint or message["target"], channel="cmd"
        )
        for event in events:
            field_wire.validate(event)
        return events

    def run_to_end(self, timeout=3.0):
        events, deadline = [], time.monotonic() + timeout
        while time.monotonic() < deadline:
            new = self.host.poll()
            for event in new:
                field_wire.validate(event)
            events += new
            if any(
                e["payload"]["status"] in {"completed", "failed", "interrupted"}
                for e in new
            ):
                return events
            time.sleep(0.02)
        return events

    def statuses(self, events):
        return [e["payload"]["status"] for e in events]

    def test_state_answers_for_the_workcell_and_each_arm(self):
        request = field_wire.build_message(
            "robot_state_request", CONTROL, "shelf-01", {}
        )
        (state,) = self.send(request)
        self.assertEqual(state["type"], "robot_state")
        self.assertEqual(state["source"], "shelf-01")
        s = state["payload"]["state"]
        self.assertTrue(s["ready"] and s["stopped"])
        self.assertIsNone(s["fault"])
        self.assertEqual([a["robot_id"] for a in s["arms"]], ["omx1", "omx2"])
        self.assertEqual(s["docking"]["pinky_id"], "pinky1")
        self.assertEqual(s["docking"]["alignment"], "unknown")  # no workcell sensor
        for arm in ("omx1", "omx2"):
            request = field_wire.build_message("robot_state_request", CONTROL, arm, {})
            (state,) = self.send(request)
            self.assertEqual(state["source"], arm)
            a = state["payload"]["state"]
            # what Fleet's ready() needs for a participating arm
            self.assertEqual(
                (a["ready"], a["stopped"], a["home_reached"], a["holding_state"]),
                (True, True, True, "empty"),
            )
            self.assertEqual(state["payload"]["boot_version"], self.host.boot_version)

    def test_store_completes_with_the_result_fleet_checks(self):
        events = self.send(self.transfer())
        self.assertEqual(self.statuses(events), ["accepted"])
        self.assertEqual(self.executor.tasks[-1]["operation"], "INBOUND")
        self.assertEqual(self.executor.tasks[-1]["manipulation_mode"], "COOPERATIVE")
        events = self.run_to_end()
        done = events[-1]
        self.assertEqual(done["payload"]["status"], "completed")
        r = done["payload"]["result"]
        self.assertEqual(
            (r["package_id"], r["slot_id"], r["pinky_id"], r["arrival_command_id"]),
            ("16", "2b", "pinky1", "cmd-route-001"),
        )
        self.assertIs(r["placement_confirmed"], True)
        self.assertIs(r["release_confirmed"], True)
        self.assertEqual({a["robot_id"] for a in r["arms"]}, {"omx1", "omx2"})
        for a in r["arms"]:
            self.assertEqual(
                (
                    a["stopped"],
                    a["home_reached"],
                    a["holding_state"],
                    a["holding_package_id"],
                ),
                (True, True, "empty", None),
            )
            self.assertEqual(a["boot_version"], self.host.boot_version)
        sequences = [
            e["payload"]["sequence"]
            for e in [*self.host._data["commands"][done["command_id"]]["events"]]
        ]
        self.assertEqual(sequences, sorted(sequences))
        self.assertIsNone(self.host.active)
        self.assertIsNone(self.host.hold)

    def test_retrieve_completes(self):
        self.send(self.transfer("retrieve"))
        self.assertEqual(self.executor.tasks[-1]["operation"], "OUTBOUND")
        self.assertEqual(self.run_to_end()[-1]["payload"]["status"], "completed")

    def test_no_placement_evidence_never_completes(self):
        self.executor.evidence = {}
        self.send(self.transfer())
        done = self.run_to_end()[-1]
        self.assertEqual(done["payload"]["status"], "failed")
        self.assertEqual(done["payload"]["reason"], "placement_or_release_unconfirmed")
        self.assertEqual(self.host.hold, done["command_id"])

    def test_arm_not_home_fails_and_holds(self):
        (event,) = self.send(self.transfer(command_id="cmd-home-first"))
        self.assertEqual(event["payload"]["status"], "accepted")
        self.executor.unreturned = {"omx2"}  # omx2 does not come back after the run
        done = self.run_to_end()[-1]
        self.assertEqual(done["payload"]["status"], "failed")
        self.assertEqual(self.host.hold, done["command_id"])

    def test_refusals(self):
        boot = self.host.boot_version
        cases = [
            (
                "boot_version_mismatch",
                self.transfer(payload={"expected_boot_version": "old"}),
            ),
            (
                "participating_arms_mismatch",
                self.transfer(
                    payload={
                        "arms": [
                            {"robot_id": "omx1", "expected_boot_version": boot},
                            {"robot_id": "omx2", "expected_boot_version": "old"},
                        ]
                    }
                ),
            ),
            ("cooperative_slot_mismatch", self.transfer(payload={"slot_id": "2a"})),
            ("stale_message", self.transfer(sent_delay_s=4)),
            (
                "pinky_boot_mismatch",
                self.transfer(payload={"expected_pinky_boot_version": "x"}),
            ),
            (
                "pinky_visit_not_confirmed",
                self.transfer(payload={"arrival_command_id": "cmd-route-other"}),
            ),
        ]
        for reason, message in cases:
            with self.subTest(reason=reason):
                (event,) = self.send(message)
                self.assertEqual(event["payload"]["status"], "rejected")
                self.assertEqual(event["payload"]["reason"], reason)
                self.assertEqual(event["command_id"], message["command_id"])
        self.executor.unreturned = {"omx1"}  # an arm away from home before the start
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "arm_not_safely_home")
        self.assertEqual(self.executor.tasks, [])

    def test_expired_command_is_rejected(self):
        message = self.transfer(expires_s=0.05)
        time.sleep(0.1)
        (event,) = self.send(message)
        self.assertEqual(event["payload"]["reason"], "expired_command")

    def test_pinky_not_at_the_shelf_is_rejected(self):
        self.host.observe_pinky(pinky_state(node="N19", seq=2))
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "pinky_visit_not_confirmed")
        self.host.pinkies.clear()
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "pinky_observation_stale")

    def test_bench_without_pinky(self):
        self.host.require_pinky = False
        self.host.pinkies.clear()
        self.assertEqual(self.statuses(self.send(self.transfer())), ["accepted"])

    def test_one_arm_and_busy(self):
        boot = self.host.boot_version
        arm = command(
            "arm_action_command",
            "omx2",
            {
                "expected_boot_version": boot,
                "operation": "return_home",
                "workcell_id": "shelf-01",
                "calibration_version": "c",
            },
        )
        (event,) = self.send(arm)
        self.assertEqual(event["source"], "omx2")
        self.assertEqual(event["payload"]["reason"], "unsupported_one_arm_command")
        self.send(self.transfer())
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "resource_busy")

    def test_resend_returns_the_same_events_and_conflict_is_dropped(self):
        message = self.transfer()
        first = self.send(message)
        self.assertEqual(self.send(message), first)  # QoS1 duplicate: no second start
        self.assertEqual(len(self.executor.tasks), 1)
        changed = message | {"payload": message["payload"] | {"package_id": "17"}}
        with self.assertRaises(ValueError):
            self.send(changed)  # never answered under the original command_id

    def test_stop_interrupts_the_task_and_completes_the_stop(self):
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        stop = command(
            "robot_stop_command",
            "shelf-01",
            {
                "expected_boot_version": self.host.boot_version,
                "target_command_id": task["command_id"],
                "reason": "control_hold",
            },
        )
        self.assertEqual(self.statuses(self.send(stop)), ["accepted"])
        self.assertIsNone(
            self.send(stop)[1:] or None
        )  # resend: same accepted event only
        events = self.run_to_end()
        ended = {
            e["command_id"]: e["payload"]
            for e in events
            if e["payload"]["status"] != "in_progress"
        }
        self.assertEqual(ended[task["command_id"]]["status"], "interrupted")
        self.assertEqual(ended[task["command_id"]]["reason"], "control_hold")
        self.assertIs(ended[task["command_id"]]["result"]["stopped"], True)
        self.assertEqual(
            ended[stop["command_id"]],
            ended[stop["command_id"]]
            | {"status": "completed", "result": {"stopped": True}},
        )
        self.assertIsNone(self.host.active)
        self.assertIsNone(self.host.hold)  # the run landed with the arms set down

    def test_stop_for_an_ended_command_completes_when_idle(self):
        task = self.transfer()
        self.send(task)
        self.run_to_end()
        stop = command(
            "robot_stop_command",
            "shelf-01",
            {
                "expected_boot_version": self.host.boot_version,
                "target_command_id": task["command_id"],
                "reason": "operator_request",
            },
        )
        self.send(stop)
        (done,) = self.host.poll()
        self.assertEqual(
            (done["command_id"], done["payload"]["status"]),
            (stop["command_id"], "completed"),
        )

    def test_pinky_leaving_stops_the_arms(self):
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        self.host.observe_pinky(pinky_state(stopped=False, seq=2))
        done = self.run_to_end()[-1]
        self.assertEqual(done["payload"]["status"], "failed")
        self.assertEqual(done["payload"]["reason"], "pinky_left")
        self.assertEqual(self.host.hold, task["command_id"])

    def test_ack_clears_the_outbox(self):
        (event,) = self.send(self.transfer())
        ack = field_wire.build_message(
            "event_stored",
            CONTROL,
            "shelf-01",
            {"acked_message_id": event["message_id"]},
        )
        self.host.handle(ack, endpoint="shelf-01", channel="ack")
        self.assertNotIn(event, self.host.pending())

    def test_restart_during_a_task_holds(self):
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        self.journal.close()
        self.journal = Journal(self.path)
        host = FieldHost(self.journal, mock.MockExecutor(), control=CONTROL)
        self.assertEqual(host.hold, task["command_id"])
        self.assertIsNone(host.active)
        self.assertNotEqual(host.boot_version, self.host.boot_version)
        self.assertEqual(host.hold_info()["error_code"], "result_unknown")

    def test_v1_journal_is_refused(self):
        self.journal.save({"identity": {"profile": "roport-omx-task-v1"}})
        self.journal.close()
        self.journal = Journal(self.path)
        with self.assertRaises(ValueError):
            FieldHost(self.journal, mock.MockExecutor(), control=CONTROL)


if __name__ == "__main__":
    unittest.main()
