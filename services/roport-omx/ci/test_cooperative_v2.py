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
VERSIONS = {
    "model_version": "real_act_rs1107x129_60k_1107",
    "calibration_version": "model-num-22-cal",
}


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
    *,
    node="N11",
    arrival="cmd-route-001",
    boot="boot-pinky1",
    stopped=True,
    seq=1,
    age_s=0.0,
    target=CONTROL,
):
    return field_wire.build_message(
        "robot_state",
        "pinky1",
        target,
        {
            "boot_version": boot,
            "observed_at": (datetime.now(UTC) - timedelta(seconds=age_s)).isoformat(),
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
        self.executor = mock.MockExecutor(0.2, evidence=EVIDENCE, versions=VERSIONS)
        self.docking = "confirmed"  # what the simulated docking sensor reports
        self.host = FieldHost(
            self.journal, self.executor, control=CONTROL, alignment=self.dock
        )
        self.host.observe_pinky(pinky_state())

    def dock(self, command, pinky):
        # a workcell docking sensor (field_host.py contract): identity from the observation
        return {
            "pinky_id": pinky["pinky_id"],
            "pinky_boot_version": pinky["boot_version"],
            "arrival_command_id": pinky["state"]["arrival_command_id"],
            "observed_at": datetime.now(UTC).isoformat(),
            "alignment": self.docking,
        }

    def keep_running(self, command_id, *, sent_delay_s=0.0, boot=None):
        # what Fleet's runtime sends about once a second for an active command
        sent = datetime.now(UTC) - timedelta(seconds=sent_delay_s)
        return field_wire.build_message(
            "robot_state_request",
            CONTROL,
            "shelf-01",
            {
                "expected_boot_version": boot or self.host.boot_version,
                "keep_running_command_id": command_id,
            },
            expires_at=(sent + timedelta(seconds=3 + sent_delay_s)).isoformat(),
        ) | {"sent_at": sent.isoformat()}

    def stop_command(self, target, reason="control_hold"):
        return command(
            "robot_stop_command",
            "shelf-01",
            {
                "expected_boot_version": self.host.boot_version,
                "target_command_id": target,
                "reason": reason,
            },
        )

    def release(self, hold, **confirmed):
        return command(
            "hold_release_command",
            "shelf-01",
            {
                "expected_boot_version": self.host.boot_version,
                "hold_command_id": hold,
                "operator": "operator-01",
                "confirmed": {"arms": True, "box": True, "pinky": True} | confirmed,
            },
        )

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
        # contract: a confirmed stop still holds the workcell until a person checks it
        self.assertEqual(self.host.hold, task["command_id"])
        self.assertEqual(self.host.hold_info()["error_code"], "control_hold")
        self.assertFalse(self.host.hold_info()["local_release_only"])

    def test_stop_for_an_ended_command_is_rejected(self):
        task = self.transfer()
        self.send(task)
        self.run_to_end()
        (event,) = self.send(self.stop_command(task["command_id"], "operator_request"))
        self.assertEqual(
            (event["payload"]["status"], event["payload"]["reason"]),
            ("rejected", "target_command_not_active"),
        )

    def run_stop(self, executor, timeout_s=0.3):
        """Start a long task on `executor`, stop it from control, poll until the stop ends."""
        self.executor = executor
        self.host.executor = executor
        self.host.stop_timeout_s = timeout_s
        self.visit = getattr(self, "visit", 0) + 1  # one transfer per Pinky arrival
        arrival = f"cmd-route-stop-{self.visit}"
        self.host.observe_pinky(pinky_state(arrival=arrival, seq=100 + self.visit))
        task = self.transfer(payload={"arrival_command_id": arrival})
        self.assertEqual(self.statuses(self.send(task)), ["accepted"])
        stop = self.stop_command(task["command_id"])
        self.send(stop)
        events, deadline = [], time.monotonic() + 3
        while time.monotonic() < deadline and not any(
            e["command_id"] == stop["command_id"]
            and e["payload"]["status"] != "accepted"
            for e in events
        ):
            events += self.host.poll()
            time.sleep(0.02)
        for event in events:
            field_wire.validate(event)
        ended = {
            e["command_id"]: e["payload"]
            for e in events
            if e["payload"]["status"] != "in_progress"
        }
        return ended[task["command_id"]], ended[stop["command_id"]]

    def test_stop_is_confirmed_only_by_arms_seen_standing_still(self):
        # #135 review (twice): the run having ended (busy false) with connected arms is no proof
        # the arms stopped; they may still slow down or move to a safe pose
        def sample(**override):
            class Executor(mock.MockExecutor):
                def readiness(self):
                    ready = super().readiness()
                    if not self.stopped:
                        return ready
                    if "safe" in override:
                        for arm in ready["arms"].values():
                            arm["safe"] = override["safe"]
                    return ready | {
                        k: v for k, v in override.items() if k in {"arms_read", "error"}
                    }

                def observe(self):
                    if self.stopped and "observe" in override:
                        return override["observe"]
                    return super().observe()

            return Executor(30, evidence=EVIDENCE, versions=VERSIONS)

        unconfirmed = {
            # the reviewer's sample: not busy, connected, safe false, no stop observation
            "safe false, no stop observation": sample(safe=False, arms_read=None),
            "connected and safe, no bus read": sample(arms_read=None),
            "panel unreachable after RESULT_UNAVAILABLE": sample(
                error="panel unreachable",
                observe={"state": "failed", "error": "RESULT_UNAVAILABLE", "arms": {}},
            ),
        }
        moving = mock.MockExecutor(30, evidence=EVIDENCE, versions=VERSIONS)
        moving.drift = 2.0  # each bus read finds the joints 2 units further
        unconfirmed["arms still moving between reads"] = moving
        for name, executor in unconfirmed.items():
            with self.subTest(case=name):
                self.host.clear_hold()
                original, stop = self.run_stop(executor)
                self.assertEqual(
                    (original["status"], original["reason"]),
                    ("failed", "physical_stop_not_confirmed"),
                )
                self.assertEqual(
                    (stop["status"], stop["reason"]),
                    ("failed", "physical_stop_not_confirmed"),
                )
                info = self.host.hold_info()
                self.assertEqual(info["error_code"], "physical_stop_not_confirmed")
                self.assertTrue(info["local_release_only"])
                (event,) = self.send(self.release(self.host.hold))
                self.assertEqual(
                    event["payload"]["reason"], "local_emergency_release_required"
                )
        # two bus reads after the stop with the joints unchanged: stopped
        self.host.clear_hold()
        original, stop = self.run_stop(
            mock.MockExecutor(30, evidence=EVIDENCE, versions=VERSIONS), timeout_s=5
        )
        self.assertEqual(
            (original["status"], original["result"]["stopped"]), ("interrupted", True)
        )
        self.assertEqual(
            (stop["status"], stop["result"]), ("completed", {"stopped": True})
        )

    def test_one_bus_read_or_a_read_from_before_the_stop_is_not_enough(self):
        self.host._stop_wall = time.time()
        reads = iter(
            [
                {"at": "r1", "joints": {a: {"j": 1.0} for a in ("omx1", "omx2")}},
                {"at": "r1", "joints": {a: {"j": 1.0} for a in ("omx1", "omx2")}},
                {"at": "r2", "joints": {a: {"j": 1.4} for a in ("omx1", "omx2")}},
            ]
        )
        arms = {
            a: {"connected": True, "safe": True, "returned": True, "grasping": False}
            for a in ("omx1", "omx2")
        }
        self.host._sample = lambda: (
            {"busy": False, "error": None, "arms": arms, "arms_read": next(reads)},
            time.time(),
        )
        self.assertIsNone(self.host._stop_confirmed())  # first read
        self.assertIsNone(self.host._stop_confirmed())  # the same cached read again
        self.assertIsNotNone(self.host._stop_confirmed())  # a second read, within 1.0
        self.host._stop_reads = []
        self.host._sample = lambda: (
            {
                "busy": False,
                "error": None,
                "arms": arms,
                "arms_read": {"at": "r3", "joints": {}},
            },
            self.host._stop_wall - 1,  # sampled before the stop request
        )
        self.assertIsNone(self.host._stop_confirmed())

    def test_keep_running_renewals_keep_the_task_and_their_absence_stops_it(self):
        self.host.watchdog_s = 0.3
        self.executor.run_s = 0.8
        task = self.transfer()
        self.send(task)
        events, deadline = [], time.monotonic() + 3
        while time.monotonic() < deadline:
            self.send(self.keep_running(task["command_id"]))
            new = self.host.poll()
            events += new
            if any(e["payload"]["status"] in {"completed", "failed"} for e in new):
                break
            time.sleep(0.1)
        self.assertEqual(events[-1]["payload"]["status"], "completed")
        # renewals that must not count: old, for another command, another boot, a duplicate
        self.executor = mock.MockExecutor(30, evidence=EVIDENCE, versions=VERSIONS)
        self.host.executor = self.executor
        task = self.transfer(payload={"arrival_command_id": "cmd-route-002"})
        self.host.observe_pinky(pinky_state(arrival="cmd-route-002", seq=2))
        self.send(task)
        duplicate = self.keep_running(task["command_id"])
        self.send(duplicate)
        ignored = [
            self.keep_running(task["command_id"], sent_delay_s=2),
            self.keep_running("cmd-other"),
            self.keep_running(task["command_id"], boot="boot-old"),
        ]
        events, deadline = [], time.monotonic() + 2
        while time.monotonic() < deadline and not events:
            for message in ignored:
                self.send(message)
            self.send(duplicate)
            self.host.observe_pinky(
                pinky_state(arrival="cmd-route-002", seq=int(time.monotonic() * 100))
            )
            events += [
                e for e in self.host.poll() if e["payload"]["status"] == "failed"
            ]
            time.sleep(0.05)
        (failed,) = events
        self.assertEqual(failed["payload"]["reason"], "control_watchdog_expired")
        self.assertTrue(self.executor.stopped)  # the arms were told to stop
        self.assertEqual(self.host.hold, task["command_id"])

    def test_watchdog_runs_without_the_broker(self):
        # local_safety() needs no delivery: the reconnect loop calls poll() only
        self.host.watchdog_s = 0.2
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        time.sleep(0.3)
        (failed,) = self.host.local_safety()
        self.assertEqual(failed["payload"]["reason"], "control_watchdog_expired")
        self.assertIn(failed, self.host.pending())  # sent after the reconnect
        self.assertTrue(self.executor.stopped)

    def test_pinky_state_lost_during_the_task_stops_the_arms(self):
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        self.host.pinkies.clear()
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["reason"], "pinky_observation_stale")
        self.assertTrue(self.executor.stopped)

    def test_pinky_observation_must_be_fresh_current_and_for_this_control(self):
        # #135 review: a state observed 35 s ago but received now proved the visit
        self.host.pinkies.clear()
        self.host.observe_pinky(pinky_state(age_s=35))
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "pinky_observation_stale")
        with self.assertRaises(ValueError):
            self.host.observe_pinky(pinky_state(target="control-02", seq=2))
        # a newer boot retires the old one: its late states are ignored
        self.host.observe_pinky(pinky_state(boot="boot-pinky1-new", seq=1))
        self.host.observe_pinky(pinky_state(boot="boot-pinky1", seq=99))
        self.assertEqual(self.host.pinkies["pinky1"]["boot_version"], "boot-pinky1-new")
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "pinky_boot_mismatch")

    def test_docking_needs_the_sensor_confirmation(self):
        for docking in ("unknown", "misaligned"):
            with self.subTest(docking=docking):
                self.docking = docking
                (event,) = self.send(self.transfer())
                self.assertEqual(
                    event["payload"]["reason"], "docking_alignment_unconfirmed"
                )
        self.host.alignment = None  # no sensor connected (the panel Host today)
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "docking_alignment_unconfirmed")
        self.assertEqual(self.executor.tasks, [])
        self.host.require_docking = (
            False  # simulated runs only (CLI refuses it for panel)
        )
        self.assertEqual(self.statuses(self.send(self.transfer())), ["accepted"])

    def test_docking_lost_during_the_task_stops_the_arms(self):
        self.executor.run_s = 30
        self.send(self.transfer())
        self.docking = "unknown"
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["reason"], "docking_alignment_lost")

    def test_model_and_calibration_must_match_the_route(self):
        # #135 review: a local route alone accepted any model/calibration control asked for
        cases = [
            {"model_version": "other-model"},
            {"calibration_version": "other-cal"},
        ]
        for change in cases:
            with self.subTest(change=change):
                (event,) = self.send(self.transfer(payload=change))
                self.assertEqual(
                    event["payload"]["reason"], "executor_version_mismatch"
                )
        self.executor.model_versions = None  # the route declares no pair
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "executor_version_mismatch")
        self.assertEqual(self.executor.tasks, [])

    def test_one_transfer_per_pinky_arrival(self):
        self.send(self.transfer())
        self.run_to_end()
        (event,) = self.send(self.transfer())
        self.assertEqual(event["payload"]["reason"], "arrival_already_used")
        # a refusal before acceptance does not use the arrival up
        message = self.transfer(payload={"arrival_command_id": "cmd-route-002"})
        self.host.observe_pinky(pinky_state(arrival="cmd-route-002", seq=2))
        self.executor.unreturned = {"omx1"}
        (event,) = self.send(message)
        self.assertEqual(event["payload"]["reason"], "arm_not_safely_home")
        self.executor.unreturned = set()
        self.assertEqual(
            self.statuses(
                self.send(
                    self.transfer(payload={"arrival_command_id": "cmd-route-002"})
                )
            ),
            ["accepted"],
        )

    def test_hold_release_command(self):
        self.executor.evidence = {}
        task = self.transfer()
        self.send(task)
        self.run_to_end()
        hold = self.host.hold
        self.assertEqual(hold, task["command_id"])
        refusals = [
            ("operator_checks_required", self.release(hold, box=False)),
            ("hold_changed", self.release("cmd-other")),
            (
                "stale_message",
                self.release(hold)
                | {"sent_at": (datetime.now(UTC) - timedelta(seconds=4)).isoformat()},
            ),
        ]
        for reason, message in refusals:
            with self.subTest(reason=reason):
                (event,) = self.send(message)
                self.assertEqual(event["payload"]["reason"], reason)
                self.assertIn(
                    event, self.host.pending()
                )  # stored: control gets the answer
                self.assertEqual(self.host.hold, hold)
        old_boot = self.release(hold)
        old_boot["payload"]["expected_boot_version"] = "boot-old"
        (event,) = self.send(old_boot)
        self.assertEqual(event["payload"]["reason"], "boot_version_mismatch")
        self.executor.unreturned = {"omx2"}
        (event,) = self.send(self.release(hold))
        self.assertEqual(event["payload"]["reason"], "arm_not_safely_home")
        self.executor.unreturned = set()
        message = self.release(hold)
        (event,) = self.send(message)
        self.assertEqual(event["payload"]["status"], "completed")
        self.assertEqual(event["payload"]["result"], {"released_hold": hold})
        self.assertIsNone(self.host.hold)
        self.assertEqual(
            self.send(message), [event]
        )  # a resend returns the stored result

    def test_emergency_stop_hold_is_released_only_at_the_shelf(self):
        class Emergency(mock.MockExecutor):
            def observe(self):
                seen = super().observe()
                if seen["state"] == "finished":
                    return {
                        "state": "failed",
                        "error": "PANEL_EMERGENCY_STOP",
                        "arms": seen["arms"],
                    }
                return seen

        self.executor = Emergency(0.2, evidence=EVIDENCE, versions=VERSIONS)
        self.host.executor = self.executor
        self.send(self.transfer())
        done = self.run_to_end()[-1]
        self.assertEqual(done["payload"]["reason"], "panel_emergency_stop")
        (event,) = self.send(self.release(self.host.hold))
        self.assertEqual(event["payload"]["reason"], "local_emergency_release_required")
        self.host.clear_hold()  # the panel's [HOLD 해제] at the shelf
        self.assertIsNone(self.host.hold)

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
        info = host.hold_info()
        self.assertEqual(info["error_code"], "host_restarted_outcome_unknown")
        self.assertTrue(info["local_release_only"])
        # control learns the outcome is unknown (kept in the outbox until stored)
        (failed,) = [e for e in host.pending() if e["payload"]["status"] == "failed"]
        field_wire.validate(failed)
        self.assertEqual(failed["command_id"], task["command_id"])
        self.assertEqual(failed["payload"]["reason"], "host_restarted_outcome_unknown")

    def test_v1_journal_is_refused(self):
        self.journal.save({"identity": {"profile": "roport-omx-task-v1"}})
        self.journal.close()
        self.journal = Journal(self.path)
        with self.assertRaises(ValueError):
            FieldHost(self.journal, mock.MockExecutor(), control=CONTROL)


if __name__ == "__main__":
    unittest.main()
