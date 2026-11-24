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
from unittest.mock import patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx import cooperative_v2
from roport_omx.cooperative import StartRefused
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
        self.host = FieldHost(self.journal, self.executor, control=CONTROL)
        self.host.observe_pinky(pinky_state())

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
            (
                "stale_message",
                self.transfer(sent_delay_s=9),
            ),  # older than FRESH_S (8 s)
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
        # one-arm transfers are run now; return_home stays reserved (spec: unsupported_operation)
        self.assertEqual(event["payload"]["reason"], "unsupported_operation")
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

    def test_only_later_bus_reads_after_the_stop_count(self):
        # #135 review: the panel's read time (arms_read.at), not when the Host collected the
        # readiness sample, decides whether a bus read was taken after the stop request
        stop = time.time()
        self.host._stop_wall = stop
        arms = {
            a: {"connected": True, "safe": True, "returned": True, "grasping": False}
            for a in ("omx1", "omx2")
        }

        def read(offset_s, value=1.0, at=None):
            stamp = at or datetime.fromtimestamp(stop + offset_s).isoformat()
            return {"at": stamp, "joints": {a: {"j": value} for a in ("omx1", "omx2")}}

        def confirm(*reads):
            self.host._stop_reads = []
            seen = None
            for r in reads:
                self.host._sample = lambda r=r: (
                    {"busy": False, "error": None, "arms": arms, "arms_read": r},
                    time.time(),  # every readiness sample itself is fresh
                )
                seen = self.host._stop_confirmed()
            return seen

        cases = {
            # two different cached reads from 2 minutes before the stop, in fresh samples
            "two reads from before the stop": (read(-120), read(-110)),
            "one before, one after": (read(-5), read(10)),
            "a single read": (read(10),),
            "the same read twice": (read(10), read(10)),
            "a late, older read after a good one": (read(10), read(5)),
            "unreadable read time": (read(10), read(0, at="later")),
            "no read time": (read(10), {"joints": read(20)["joints"]}),
            "joints moved between the reads": (read(10), read(20, value=2.5)),
        }
        for name, reads in cases.items():
            with self.subTest(case=name):
                self.assertIsNone(confirm(*reads))
        self.assertIsNotNone(confirm(read(10), read(20, value=1.4)))
        # a stale cached read arriving between two good ones is skipped, not compared
        self.assertIsNotNone(confirm(read(10), read(-60), read(20)))
        # the panel writes local time without a zone; a read with a zone is compared the same way
        zoned = datetime.fromtimestamp(stop + 20).astimezone().isoformat()
        self.assertIsNotNone(confirm(read(10), read(0, at=zoned)))

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

    def test_no_docking_sensor_is_needed(self):
        # TK 2026-11-23: the docking_alignment check was dropped; the Pinky's own state decides
        self.assertEqual(self.statuses(self.send(self.transfer())), ["accepted"])
        (state,) = self.send(
            field_wire.build_message("robot_state_request", CONTROL, "shelf-01", {})
        )
        self.assertEqual(state["payload"]["state"]["docking"]["alignment"], "unknown")

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
                | {"sent_at": (datetime.now(UTC) - timedelta(seconds=9)).isoformat()},
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

    def slow_start(self, delay_s, error=None):
        """The mock's start taking delay_s like the panel's /api/eval/start (2026-11-23 field run)."""
        executor, plain = self.executor, self.executor.start
        executor.stops = 0
        plain_stop = executor.stop

        def start(task):
            time.sleep(delay_s)
            if error is not None:
                raise error
            plain(task)

        def stop():
            executor.stops += 1
            plain_stop()

        executor.start, executor.stop = start, stop
        return executor

    def test_slow_start_keeps_the_loop_free(self):
        # the field run: a 3 s start blocked the loop, the Pinky state looked stale, the arms stopped
        self.slow_start(3.5)
        self.executor.run_s = 0.3
        task = self.transfer()
        began = time.monotonic()
        self.assertEqual(self.statuses(self.send(task)), ["accepted"])
        self.assertLess(
            time.monotonic() - began, 1.0
        )  # handle() did not wait for the panel
        seq, events = 1, []
        while time.monotonic() - began < 6:
            seq += 1
            self.host.observe_pinky(pinky_state(seq=seq))  # the Pinky keeps reporting
            self.host.handle(
                self.keep_running(task["command_id"]),
                endpoint="shelf-01",
                channel="cmd",
            )
            new = self.host.poll()
            events += new
            if any(e["payload"]["status"] in {"completed", "failed"} for e in new):
                break
            time.sleep(0.25)
        self.assertEqual(
            events[-1]["payload"]["status"], "completed", events[-1]["payload"]
        )
        timing = self.host._data["commands"][task["command_id"]]["timing"]
        order = ["sent", "received", "accepted", "start_sent", "start_done", "ended"]
        self.assertEqual([k for k in order if k in timing], order)
        self.assertEqual(sorted(timing[k] for k in order), [timing[k] for k in order])
        self.assertGreaterEqual(timing["start_done"] - timing["start_sent"], 3.4)
        self.assertTrue(any(k.startswith("step:") for k in timing))

    def test_slow_start_refusal_ends_the_task_from_poll(self):
        self.slow_start(0.5, StartRefused("PANEL_TRAY_NOT_EMPTY", "tray", hold=False))
        self.assertEqual(self.statuses(self.send(self.transfer())), ["accepted"])
        done = self.run_to_end()[-1]["payload"]
        self.assertEqual(
            (done["status"], done["reason"]), ("failed", "panel_tray_not_empty")
        )
        self.assertIsNone(self.host.active)
        self.assertIsNone(self.host.hold)  # hold=False refusal: no motion happened

    def test_start_returning_after_the_task_ended_is_stopped_again(self):
        self.slow_start(1.0)
        self.executor.run_s = 30
        self.send(self.transfer())
        self.host.observe_pinky(
            pinky_state(stopped=False, seq=2)
        )  # the Pinky leaves meanwhile
        failed = self.run_to_end()[-1]["payload"]
        self.assertEqual(failed["reason"], "pinky_left")
        stops = self.executor.stops
        deadline = time.monotonic() + 3
        while self.executor.stops == stops and time.monotonic() < deadline:
            self.host.poll()
            time.sleep(0.05)
        self.assertGreater(
            self.executor.stops, stops
        )  # the late start got the stop again

    def test_no_state_fault_during_a_task(self):
        # 2026-11-23 23:06: a slow arm read while the panel started the run became
        # fault observation_stale in robot_state, and Fleet held on the participant fault
        self.executor.run_s = 30
        self.send(self.transfer())
        self.host._sample = lambda: (
            {"armed": True, "busy": False, "error": None, "arms": {}},
            time.time() - 10,
        )
        for endpoint in ("shelf-01", "omx1", "omx2"):
            msg = self.host.state(endpoint)
            field_wire.validate(msg)
            s = msg["payload"]["state"]
            self.assertIsNone(s["fault"], endpoint)
            self.assertFalse(s["ready"])
            self.assertFalse(s["stopped"])
            age = (
                datetime.now(UTC) - field_wire.timestamp(msg["payload"]["observed_at"])
            ).total_seconds()
            self.assertLess(age, 1.0)  # a current report, not the stale sample's time
        self.host._sample = lambda: (
            {"armed": True, "busy": False, "error": "PANEL_DOWN", "arms": {}},
            time.time(),
        )
        self.assertIsNone(self.host.state("shelf-01")["payload"]["state"]["fault"])
        # idle again: a stale or failing executor is a fault as before
        self.host._data["active"] = None
        self.host._sample = lambda: (
            {"armed": True, "busy": False, "error": None, "arms": {}},
            time.time() - 10,
        )
        self.assertEqual(
            self.host.state("shelf-01")["payload"]["state"]["fault"],
            "observation_stale",
        )
        self.host._sample = lambda: (
            {"armed": True, "busy": False, "error": "PANEL_DOWN", "arms": {}},
            time.time(),
        )
        self.assertEqual(
            self.host.state("omx1")["payload"]["state"]["fault"], "panel_down"
        )

    def test_acceptance_uses_the_recent_sample_not_a_slow_panel_read(self):
        # 2026-11-23 23:32 / 23:41: 2.8-4 s of direct panel reads during the checks; no Pinky
        # state was read meanwhile and the transfer ended pinky_observation_stale at once
        plain = self.executor.readiness
        fresh = plain()

        def slow():
            time.sleep(3)
            return plain()

        self.executor.readiness = slow
        self.host._sample = lambda: (
            fresh,
            time.time(),
        )  # the sampler's reading, 0 s old
        began = time.monotonic()
        task = self.transfer()
        self.assertEqual(self.statuses(self.send(task)), ["accepted"])
        self.assertLess(time.monotonic() - began, 1.0)
        timing = self.host._data["commands"][task["command_id"]]["timing"]
        self.assertLess(timing["checked"] - timing["received"], 1.0)

    def placed_then(self, outcome):
        """The executor reports placed once, keeps aligning a moment, then ends with outcome."""
        observe, state = self.executor.observe, {"placed_at": None}

        def run():
            seen = observe()
            if state["placed_at"] is None:
                state["placed_at"] = time.monotonic()
                return {
                    "state": "placed",
                    "grasp_released": True,
                    "placement": {"slot": "2B"},
                }
            if time.monotonic() - state["placed_at"] < 0.3:
                return {"state": "running", "step": "정렬 중"}
            return outcome(seen)

        self.executor.observe = run

    def ack(self, event):
        message = field_wire.build_message(
            "event_stored",
            CONTROL,
            "shelf-01",
            {"acked_message_id": event["message_id"]},
        )
        self.host.handle(message, endpoint="shelf-01", channel="ack")

    def drain(self, seconds):
        events, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            new = self.host.poll()
            for e in new:
                field_wire.validate(e)
            events += new
            time.sleep(0.05)
        return events

    def test_store_completes_at_placement_then_reports_the_alignment(self):
        # TK 2026-11-24: completed (shelf_placed) the moment the box is placed so the Pinky may
        # leave; after the alignment a second completed (shelf_align, arms home) tells control
        # the shelf is free for the next command. No renewal or Pinky check in between.
        self.host.report_placement = True
        self.executor.run_s = 0.2
        self.placed_then(
            lambda seen: {
                "state": "finished",
                "evidence": "shelf_align",
                "arms": self.executor._arms(moving=False),
            }
        )
        task = self.transfer()
        self.send(task)
        first = [e for e in self.drain(0.2) if e["payload"]["status"] == "completed"]
        self.assertEqual(len(first), 1)
        r = first[0]["payload"]["result"]
        self.assertEqual(
            (r["evidence"], r["placement_confirmed"], r["release_confirmed"]),
            ("shelf_placed", True, True),
        )
        self.assertEqual(
            {a["home_reached"] for a in r["arms"]}, {False}
        )  # still aligning
        self.assertEqual(self.host.active, task["command_id"])  # busy until aligned
        self.ack(
            first[0]
        )  # control stored the placement report: now the Pinky may leave
        self.host.observe_pinky(pinky_state(stopped=False, node="N19", seq=5))
        later = [e for e in self.drain(1.0) if e["command_id"] == task["command_id"]]
        self.assertEqual([e["payload"]["status"] for e in later], ["completed"])
        r = later[0]["payload"]["result"]
        self.assertEqual(r["evidence"], "shelf_align")
        self.assertEqual({a["home_reached"] for a in r["arms"]}, {True})
        seqs = [e["payload"]["sequence"] for e in first + later]
        self.assertEqual(seqs, sorted(seqs))
        self.assertIsNone(self.host.active)
        self.assertIsNone(self.host.hold)
        self.assertIn(
            "placed", self.host._data["commands"][task["command_id"]]["timing"]
        )

    def test_alignment_failure_after_placement_is_reported_and_holds(self):
        self.host.report_placement = True
        self.placed_then(
            lambda seen: {"state": "failed", "error": "PANEL_ALIGN_HOLD", "arms": {}}
        )
        task = self.transfer()
        self.send(task)
        events = [e for e in self.drain(1.0) if e["command_id"] == task["command_id"]]
        self.assertEqual(
            [
                e["payload"]["status"]
                for e in events
                if e["payload"]["status"] != "in_progress"
            ],
            ["completed", "failed"],
        )
        self.assertEqual(events[-1]["payload"]["reason"], "panel_align_hold")
        self.assertEqual(self.host.hold, task["command_id"])

    def test_broker_cut_right_after_the_placement_report_stops_the_arms(self):
        # #156 review: no control renewal (and no ACK) after the intermediate report still stops
        # the arms through the watchdog, reported as the command's second report
        self.host.report_placement = True
        self.host.watchdog_s = 0.6
        self.placed_then(lambda seen: {"state": "running", "step": "정렬 중"})
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        events = [e for e in self.drain(1.5) if e["command_id"] == task["command_id"]]
        statuses = [
            e["payload"]["status"]
            for e in events
            if e["payload"]["status"] != "in_progress"
        ]
        self.assertEqual(statuses[:2], ["completed", "failed"])
        self.assertEqual(events[-1]["payload"]["reason"], "control_watchdog_expired")
        self.assertEqual(self.host.hold, task["command_id"])

    def test_pinky_leaving_before_the_placement_report_is_stored_stops_the_arms(self):
        # #156 review: until control stored the placement report the Pinky check stays on
        self.host.report_placement = True
        self.placed_then(lambda seen: {"state": "running", "step": "정렬 중"})
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        first = [e for e in self.drain(0.2) if e["payload"]["status"] == "completed"]
        self.assertEqual(len(first), 1)
        self.host.observe_pinky(
            pinky_state(stopped=False, node="N19", seq=5)
        )  # no ACK yet
        events = [e for e in self.drain(0.5) if e["command_id"] == task["command_id"]]
        self.assertEqual(events[-1]["payload"]["status"], "failed")
        self.assertEqual(events[-1]["payload"]["reason"], "pinky_left")

    def test_final_report_names_the_alignment_without_a_first_report(self):
        # #156 review: no placement report (judge missed it, or the option is off) must not
        # turn a crooked load into a plain completed: the evidence is always named
        self.host.report_placement = False
        self.placed_then(
            lambda seen: {
                "state": "finished",
                "evidence": "shelf_load_not_aligned",
                "arms": self.executor._arms(moving=False),
            }
        )
        task = self.transfer()
        self.send(task)
        done = [
            e
            for e in self.drain(1.0)
            if e["command_id"] == task["command_id"]
            and e["payload"]["status"] == "completed"
        ]
        self.assertEqual(len(done), 1)
        self.assertEqual(
            done[0]["payload"]["result"]["evidence"], "shelf_load_not_aligned"
        )
        self.assertIsNone(
            self.host.hold
        )  # control holds the slot (#158), the arms are home

    def test_no_placement_report_while_a_stop_is_pending(self):
        # #156 review: control lets the Pinky leave on shelf_placed; a stop that is not yet
        # confirmed must not be overtaken by it
        self.host.report_placement = True
        self.executor.run_s = 30
        task = self.transfer()
        self.send(task)
        self.assertEqual(
            self.statuses(self.send(self.stop_command(task["command_id"]))),
            ["accepted"],
        )
        self.executor.observe = lambda: {
            "state": "placed",
            "grasp_released": True,
            "placement": {"slot": "2B"},
        }
        events = [e for e in self.drain(0.3) if e["command_id"] == task["command_id"]]
        self.assertNotIn("completed", self.statuses(events))

    def stale_sampler(self):
        fresh = self.executor.readiness()
        self.host._sampled = True
        self.host._sample = lambda: (
            fresh,
            time.time() - 5,
        )  # sampler stuck on a slow read
        calls = []
        plain = self.executor.readiness

        def slow():
            calls.append(1)
            time.sleep(3)
            return plain()

        self.executor.readiness = slow
        return fresh, calls

    def test_stale_sample_defers_acceptance_without_a_panel_read(self):
        # #156 review: an old readiness sample made handle() read the panel inside the MQTT loop
        fresh, calls = self.stale_sampler()
        task = self.transfer()
        began = time.monotonic()
        self.assertEqual(self.send(task), [])  # nothing answered yet
        self.assertEqual(self.send(task), [])  # a redelivery while waiting
        self.assertLess(time.monotonic() - began, 0.5)
        self.assertEqual(calls, [])
        self.assertEqual(self.host.poll(), [])  # still old: keeps waiting
        self.host._sample = lambda: (fresh, time.time())  # the next sample arrives
        self.assertEqual(self.statuses(self.host.poll())[:1], ["accepted"])
        self.assertEqual(calls, [])

    def test_stale_sample_for_too_long_rejects_the_transfer(self):
        self.stale_sampler()
        task = self.transfer()
        with patch.object(cooperative_v2, "ACCEPT_WAIT_S", 0.2):
            self.assertEqual(self.send(task), [])
            events = self.drain(0.4)
        self.assertEqual(self.statuses(events), ["rejected"])
        self.assertEqual(events[0]["payload"]["reason"], "executor_state_stale")
        self.assertEqual(self.executor.tasks, [])

    def first_model_list(self, ready_after: int | None):
        """A sampler with a fresh reading and an executor whose first model list is loading."""
        fresh = self.executor.readiness()
        self.host._sampled = True
        self.host._sample = lambda: (fresh, time.time())
        asked = []

        def models_ready():
            asked.append(1)
            return ready_after is not None and len(asked) > ready_after

        self.executor.models_ready = models_ready
        return asked

    def test_first_model_list_defers_acceptance_without_a_panel_read(self):
        # #156 review: before the panel's first model list, check() waited for the panel inside
        # the MQTT loop (reproduced with a 3 s panel). Now the transfer waits for poll().
        asked = self.first_model_list(ready_after=2)
        task = self.transfer()
        began = time.monotonic()
        self.assertEqual(self.send(task), [])  # list loading: nothing answered yet
        self.assertLess(time.monotonic() - began, 0.5)
        self.assertEqual(self.host.poll(), [])  # still loading
        self.assertEqual(self.statuses(self.host.poll())[:1], ["accepted"])
        self.assertEqual(len(asked), 3)

    def test_first_model_list_missing_for_too_long_rejects_the_transfer(self):
        self.first_model_list(ready_after=None)  # e.g. the panel is not up yet
        task = self.transfer()
        with patch.object(cooperative_v2, "ACCEPT_WAIT_S", 0.2):
            self.assertEqual(self.send(task), [])
            events = self.drain(0.4)
        self.assertEqual(self.statuses(events), ["rejected"])
        self.assertEqual(events[0]["payload"]["reason"], "executor_state_stale")
        self.assertEqual(self.executor.tasks, [])

    def test_stop_for_a_waiting_transfer_keeps_it_from_starting(self):
        fresh, _ = self.stale_sampler()
        task = self.transfer()
        self.assertEqual(self.send(task), [])
        events = self.send(self.stop_command(task["command_id"]))
        by_id = {e["command_id"]: e["payload"] for e in events}
        self.assertEqual(by_id[task["command_id"]]["status"], "rejected")
        self.assertEqual(by_id[task["command_id"]]["reason"], "stop_requested")
        self.host._sample = lambda: (fresh, time.time())
        self.drain(0.3)
        self.assertEqual(self.executor.tasks, [])

    def test_outbound_never_completes_at_placement(self):
        self.host.report_placement = True
        self.executor.run_s = 0.2
        self.send(self.transfer("retrieve"))
        statuses = self.statuses(self.run_to_end())
        self.assertEqual(statuses[-1], "completed")

    def test_wrist_check_refusal_reaches_control_as_its_reason(self):
        # panel 3.22.0 refused before the policy (err 13): the executor reports PANEL_WRIST_CHECK
        # with the arms home; control gets failed panel_wrist_check and the Host keeps no HOLD
        observe = self.executor.observe

        def refused():
            seen = observe()
            if seen.get("state") == "finished":
                return {
                    "state": "failed",
                    "error": "PANEL_WRIST_CHECK",
                    "arms": seen["arms"],
                    "recovered": True,
                    "detail": "left move changed both wrist cameras",
                }
            return seen

        self.executor.observe = refused
        task = self.transfer()
        self.send(task)
        done = self.run_to_end()[-1]["payload"]
        self.assertEqual(
            (done["status"], done["reason"]), ("failed", "panel_wrist_check")
        )
        self.assertIsNone(self.host.hold)
        record = self.host._data["commands"][task["command_id"]]
        self.assertIn("wrist cameras", record["detail"])

    def test_v1_journal_is_refused(self):
        self.journal.save({"identity": {"profile": "roport-omx-task-v1"}})
        self.journal.close()
        self.journal = Journal(self.path)
        with self.assertRaises(ValueError):
            FieldHost(self.journal, mock.MockExecutor(), control=CONTROL)


if __name__ == "__main__":
    unittest.main()
