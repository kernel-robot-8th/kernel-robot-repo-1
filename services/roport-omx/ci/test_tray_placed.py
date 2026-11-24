"""Unloading completed at the tray (panel 3.24.1 tray judge, TK 2026-11-24).

The panel's tray judge sees the box on the Pinky tray, the commanded slot empty and both hands out of
the tray zone (job.info.tray_placed, while the arms still go home). The v2 Host then sends completed
(evidence "pinky_placed"): control stores it and lets the Pinky leave. Once the arms are home with the
grippers released, a second completed (evidence "pinky_load"). A hand that comes back into the tray zone
and cannot get out is held by the panel: failed + HOLD.
"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_cooperative_v2 as v2
import test_panel_executor as pe

from roport_omx.mqtt_link import field_wire
from roport_omx.panel_executor import PanelExecutor


def tray_placed(**change):
    """job.info.tray_placed of panel 3.24.1 (mode on), captured just now."""
    return {
        "operation": "OUTBOUND",
        "on_pinky": True,
        "slot_empty": True,
        "slot": "2B",
        "box": 16,
        "tips_clear": True,
        "grippers_released": True,
        "tray_marker": True,
        "tray_changed": 0.61,
        "tray_reference": "first_frame(checked_vs_pre_mission)",
        "tray_reference_state": "ok",
        "pre_mission_match": True,
        "observed_ts": time.time() - 0.3,
        "observed_clock": "capture",
        "source": "eval_judge",
        "judge_version": "1124b",
        "mode": "on",
        "judged_at": "2026-11-24T21:30:00.000+09:00",
    } | change


class ExecutorTrayTest(unittest.TestCase):
    def setUp(self):
        self.panel = pe.FakePanel()
        self.panel.status["version"] = "3.24.1"
        self.ex = PanelExecutor(pe.ROUTES, box_marker=16, base_url=self.panel.url)
        self.ex.report_placement = True  # v2 default
        self.ex.report_unload_placement = True
        self.ex._version = (3, 24, 1)
        self.ex.repo_id, self.ex.slot, self.ex.operation, self.ex.box = (
            "tkoo/eval_x",
            "2b",
            "OUTBOUND",
            16,
        )
        self.ex._placed, self.ex._recovery, self.ex._missing_since = False, None, None

    def tearDown(self):
        self.panel.server.shutdown()
        self.panel.server.server_close()

    def running(self, **info):
        self.panel.status["job"] = {
            "running": True,
            "kind": "eval",
            "info": {"repo_id": "tkoo/eval_x", "phase": "천천히 내려놓는 중"} | info,
        }

    def test_placed_once_with_the_evidence(self):
        self.running(tray_placed=tray_placed())
        seen = self.ex.observe()
        self.assertEqual(
            (seen["state"], seen["evidence"], seen["grasp_released"]),
            ("placed", "pinky_placed", True),
        )
        self.assertEqual(seen["placement"]["box"], 16)
        self.assertEqual(self.ex.observe()["state"], "running")  # one report only

    def test_not_placed_unless_every_condition_holds(self):
        cases = {
            # #169 review: the v2 default (report_placement on) alone never reports the unloading early
            "unloading option off": ({}, {"report_unload_placement": False}),
            "inbound": ({}, {"operation": "INBOUND"}),
            "panel 3.24.0": ({}, {"_version": (3, 24, 0)}),
            "record mode": ({"mode": "record"}, {}),
            "not on the tray": ({"on_pinky": False}, {}),
            "slot not empty": ({"slot_empty": False}, {}),
            "another slot": ({"slot": "1B"}, {}),
            "another box": ({"box": 17}, {}),
            "hands in the tray zone": ({"tips_clear": False}, {}),
            "still grasping": ({"grippers_released": False}, {}),
            "tray not checked against this mission's photo": (
                {"tray_reference_state": "not_checked", "pre_mission_match": False},
                {},
            ),
            "control-step time": ({"observed_clock": "control_step"}, {}),
            "too old": ({"observed_ts": time.time() - 10}, {}),
            "not the judge": ({"source": "pinky_read"}, {}),
        }
        for name, (change, attrs) in cases.items():
            with self.subTest(name):
                self.setUp()
                for k, v in attrs.items():
                    setattr(self.ex, k, v)
                self.running(tray_placed=tray_placed(**change))
                self.assertEqual(self.ex.observe()["state"], "running", name)
                self.tearDown()
        self.setUp()

    def test_other_run_is_ignored(self):
        self.running(tray_placed=tray_placed())
        self.panel.status["job"]["info"]["repo_id"] = "tkoo/other"
        self.assertNotEqual(self.ex.observe().get("state"), "placed")

    def test_held_over_the_tray_zone_fails_with_hold_code(self):
        self.running(
            tray_placed=tray_placed(),
            tray_hold={"reason": "tray_reentry", "side": "left"},
        )
        seen = self.ex.observe()
        self.assertEqual(
            (seen["state"], seen["error"]), ("failed", "PANEL_TRAY_REENTRY_HOLD")
        )
        self.assertIn("PANEL_TRAY_REENTRY_HOLD", pe.panel_executor.HOLD_ERRORS)

    def test_after_the_run_the_judged_values_complete_the_unloading(self):
        # panel 3.24.1: no camera check after a tray placement; last_result.pinky holds the judged values
        pinky = {
            "ran": False,
            "skipped": "tray_placed",
            "source": "eval_judge",
            "box": 16,
            "slot": "2B",
            "on_pinky": True,
            "slot_empty": True,
            "tray_reference_state": "ok",
            "pre_mission_match": True,
        }
        result = {
            "repo_id": "tkoo/eval_x",
            "exit_ok": True,
            "stage": "done",
            "final": True,
            "pinky": pinky,
            "arms": {"OMX 1": pe.arm("left"), "OMX 2": pe.arm("right")},
        }
        seen = PanelExecutor._result(result, "2b", "OUTBOUND", 16)
        self.assertEqual((seen["state"], seen["evidence"]), ("finished", "pinky_load"))
        bad = dict(pinky, pre_mission_match=False, tray_reference_state="not_checked")
        seen = PanelExecutor._result(dict(result, pinky=bad), "2b", "OUTBOUND", 16)
        self.assertEqual(
            (seen["state"], seen["error"]), ("failed", "PANEL_PINKY_NO_REFERENCE")
        )


class FieldHostTrayTest(v2.FieldHostTest):
    def tray_then(self, outcome, hold=0.3):
        """The executor reports the tray placement once, the arms go home a moment, then outcome."""
        observe, state = self.executor.observe, {"at": None}

        def run():
            seen = observe()
            if state["at"] is None:
                state["at"] = time.monotonic()
                return {
                    "state": "placed",
                    "step": "tray_placed",
                    "evidence": "pinky_placed",
                    "grasp_released": True,
                    "placement": {
                        "slot": "2B",
                        "box": 16,
                        "on_pinky": True,
                        "slot_empty": True,
                        "tray_marker": True,
                        "tray_changed": 0.61,
                        "tray_reference_state": "ok",
                        "observed_ts": 1795530000.25,
                        "observed_clock": "capture",
                    },
                }
            if time.monotonic() - state["at"] < hold:
                return {"state": "running", "step": "천천히 내려놓는 중"}
            return outcome(seen)

        self.executor.observe = run

    def test_retrieve_completes_at_the_tray_then_when_home(self):
        self.host.report_placement = True
        self.host.report_unload_placement = True
        self.executor.run_s = 0.2
        self.tray_then(
            lambda seen: {
                "state": "finished",
                "evidence": "pinky_load",
                "arms": self.executor._arms(moving=False),
            }
        )
        task = self.transfer("retrieve")
        self.send(task)
        first = [e for e in self.drain(0.2) if e["payload"]["status"] == "completed"]
        self.assertEqual(len(first), 1)
        r = first[0]["payload"]["result"]
        self.assertEqual(
            (r["evidence"], r["placement_confirmed"], r["release_confirmed"]),
            ("pinky_placed", True, True),
        )
        self.assertEqual({a["home_reached"] for a in r["arms"]}, {False})
        # the camera check of that moment, for control's record (the Pinky leaves on it)
        self.assertEqual(r["unload_check"]["on_pinky"], True)
        self.assertEqual(r["unload_check"]["slot_empty"], True)
        self.assertEqual(r["unload_check"]["tray_reference_state"], "ok")
        self.assertEqual(
            r["unload_check"]["observed_clock"], "capture"
        )  # the arms still go home
        self.assertEqual(self.host.active, task["command_id"])
        self.ack(first[0])  # control stored it: the Pinky may leave now
        self.assertTrue(
            self.host._data["commands"][task["command_id"]].get("placed_stored")
        )
        self.host.observe_pinky(v2.pinky_state(stopped=False, node="N19", seq=5))
        later = [e for e in self.drain(1.0) if e["command_id"] == task["command_id"]]
        self.assertEqual([e["payload"]["status"] for e in later], ["completed"])
        r = later[0]["payload"]["result"]
        self.assertEqual(r["evidence"], "pinky_load")
        self.assertEqual({a["home_reached"] for a in r["arms"]}, {True})
        self.assertIsNone(self.host.active)
        self.assertIsNone(self.host.hold)

    def test_retrieve_held_over_the_tray_after_the_first_report(self):
        self.host.report_placement = True
        self.host.report_unload_placement = True
        self.tray_then(
            lambda seen: {
                "state": "failed",
                "error": "PANEL_TRAY_REENTRY_HOLD",
                "arms": {},
            }
        )
        task = self.transfer("retrieve")
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
        self.assertEqual(events[-1]["payload"]["reason"], "panel_tray_reentry_hold")
        self.assertEqual(self.host.hold, task["command_id"])

    def test_pinky_leaving_before_control_stored_it_still_stops_the_arms(self):
        # until control has stored the first report the Pinky must stay: leaving then is a lost visit
        self.host.report_placement = True
        self.host.report_unload_placement = True
        self.tray_then(
            lambda seen: {"state": "running", "step": "천천히 내려놓는 중"}, hold=30
        )
        task = self.transfer("retrieve")
        self.send(task)
        first = [e for e in self.drain(0.2) if e["payload"]["status"] == "completed"]
        self.assertEqual(len(first), 1)
        self.host.observe_pinky(v2.pinky_state(stopped=False, node="N19", seq=5))
        events = self.host.local_safety()
        self.assertEqual([e["payload"]["status"] for e in events], ["failed"])

    def test_without_the_option_one_report_after_home(self):
        # #169 review: the v2 default (report_placement on) keeps one pinky_load report after home
        self.host.report_placement = True
        self.host.report_unload_placement = False
        self.executor.run_s = 0.2
        self.tray_then(
            lambda seen: {
                "state": "finished",
                "evidence": "pinky_load",
                "arms": self.executor._arms(moving=False),
            }
        )
        task = self.transfer("retrieve")
        self.send(task)
        events = [e for e in self.drain(1.0) if e["command_id"] == task["command_id"]]
        done = [e for e in events if e["payload"]["status"] == "completed"]
        self.assertEqual(len(done), 1)
        self.assertEqual(done[0]["payload"]["result"]["evidence"], "pinky_load")
        for e in events:
            field_wire.validate(e)


def load_tests(loader, tests, pattern):
    """Only this file's tests (FieldHostTrayTest borrows test_cooperative_v2's set-up, not its tests)."""
    suite = unittest.TestSuite(loader.loadTestsFromTestCase(ExecutorTrayTest))
    own = [n for n in vars(FieldHostTrayTest) if n.startswith("test_")]
    suite.addTests(FieldHostTrayTest(n) for n in own)
    return suite


if __name__ == "__main__":
    unittest.main()
