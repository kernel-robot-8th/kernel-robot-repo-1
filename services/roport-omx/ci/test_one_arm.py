"""One-arm transfers (arm_action_command, TK 2026-11-24; Fleet SLOTS: A = omx1 at N21, C = omx2 at N22).

v2 accepts a transfer on the arm's own topic when its slot is in that arm's column and the Pinky stands at that
column's stop; the panel (3.25) runs the one-arm model with the placement judge (loading) or the tray judge and a
second tray read with the arm home (unloading); one completed after the arm is home, with only that arm.
"""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_cooperative_v2 as v2
import test_panel_executor as pe

from roport_omx import panel_executor
from roport_omx.mqtt_link import field_wire
from roport_omx.panel_executor import PanelExecutor, Route

OUT_LEFT, IN_RIGHT = (
    "/models/outbound_leftarm/pretrained_model",
    "/models/right_inbound_trimstart/pretrained_model",
)
ONE_ARM_OPTIONS = {
    "async_mode": True,
    "actions_per_chunk": 100,
    "chunk_size_threshold": 0.2,
    "aggregate_fn": "weighted_average",
}


def arm_action(host, endpoint, operation, slot, *, node_pinky="pinky1", **change):
    shelf = {"kind": "shelf", "slot_id": slot}
    pinky = {"kind": "pinky", "robot_id": node_pinky, "position": "deck"}
    store = operation == "store"
    payload = {
        "expected_boot_version": host.boot_version,
        "operation": "transfer",
        "workcell_id": "shelf-01",
        "calibration_version": "model-num-22-cal",
        "package_id": "21",
        "arrival_command_id": "cmd-route-001",
        "expected_pinky_boot_version": "boot-pinky1",
        "source_location": pinky if store else shelf,
        "destination_location": shelf if store else pinky,
        "model_version": "real_act_rs1107x129_60k_1107",
    } | change
    return v2.command("arm_action_command", endpoint, payload)


class ExecutorOneArmTest(unittest.TestCase):
    def setUp(self):
        self.panel = pe.FakePanel()
        self.panel.status["version"] = "3.25.0"
        routes = dict(pe.ROUTES)
        routes[("INBOUND", "2c", "DEDICATED")] = Route(
            IN_RIGHT,
            "INBOUND tray->shelf 2C | marker {box}",
            120,
            15,
            eval_options=dict(ONE_ARM_OPTIONS),
        )
        routes[("OUTBOUND", "2a", "DEDICATED")] = Route(
            OUT_LEFT,
            "OUTBOUND shelf 2A->tray | marker {box}",
            120,
            15,
            eval_options=dict(ONE_ARM_OPTIONS),
        )
        self.panel.models = [pe.MODEL, IN_RIGHT, OUT_LEFT]
        self.ex = PanelExecutor(routes, box_marker=16, base_url=self.panel.url)
        self.ex.readiness()

    def tearDown(self):
        self.panel.server.shutdown()
        self.panel.server.server_close()

    def task(self, operation, slot, arm):
        return {
            "operation": operation,
            "slot_id": slot,
            "manipulation_mode": "DEDICATED",
            "assigned_arm_id": arm,
            "robot_ids": [arm],
            "package_id": "21",
            "pinky_id": "pinky1",
            "expires_at": "2099-01-01T00:00:00+00:00",
        }

    def test_start_body_loading_right_2c(self):
        self.ex.start(self.task("INBOUND", "2c", "omx2"))
        b = self.panel.starts[-1]
        self.assertEqual(
            (b["arm"], b["slot"], b["end_on_place"], b["end_on_return"]),
            ("right", "2C", True, False),
        )
        self.assertNotIn("shelf_align", b)
        self.assertNotIn("fast_release", b)
        self.assertNotIn("pinky_check", b)
        for k, v in ONE_ARM_OPTIONS.items():
            self.assertEqual(b[k], v)
        self.assertEqual(self.ex.arm_names, ("omx2",))

    def test_start_body_unloading_left_2a(self):
        self.ex.start(self.task("OUTBOUND", "2a", "omx1"))
        b = self.panel.starts[-1]
        self.assertEqual(
            (b["arm"], b["slot"], b["pinky_check"], b["draw_target"]),
            ("left", "2A", True, False),
        )
        self.assertNotIn("end_on_place", b)

    def test_panel_before_3_25_refuses_one_arm(self):
        self.ex._version = (3, 24, 1)
        with self.assertRaises(Exception) as ctx:
            self.ex.check(self.task("INBOUND", "2c", "omx2"))
        self.assertIn("UNSUPPORTED_SLOT", str(ctx.exception.args))

    def test_loading_result(self):
        placed = {
            "placement": {
                "on_shelf": True,
                "source": "eval_judge",
                "slot": "2C",
                "one_arm": True,
            }
        }
        seen = PanelExecutor._result_one_arm(placed, "2c", "INBOUND", ("omx2",))
        self.assertEqual(
            (seen["state"], seen["evidence"], list(seen["arms"])),
            ("finished", "shelf_load_one_arm", ["omx2"]),
        )
        for result in (
            {"placement": {"on_shelf": None, "source": "eval_judge", "slot": "2C"}},
            {"placement": {"on_shelf": True, "source": "eval_judge", "slot": "2A"}},
            {},
        ):
            seen = PanelExecutor._result_one_arm(result, "2c", "INBOUND", ("omx2",))
            self.assertEqual(
                (seen["state"], seen["error"]), ("failed", "PANEL_BOX_NOT_ON_SHELF")
            )
        self.assertIn("PANEL_BOX_NOT_ON_SHELF", panel_executor.HOLD_ERRORS)
        seen = PanelExecutor._result_one_arm(
            {"stopped_by": "중지"}, "2c", "INBOUND", ("omx2",)
        )
        self.assertTrue(seen["operator_stop"])

    def test_unloading_result(self):
        check = {
            "ran": True,
            "on_pinky": True,
            "slot_empty": True,
            "slot": "2A",
            "tray_reference_state": "eval_first_frame",
            "observed_ts": 1.0,
            "observed_clock": "capture",
        }
        seen = PanelExecutor._result_one_arm(
            {"pinky": check}, "2a", "OUTBOUND", ("omx1",)
        )
        self.assertEqual((seen["state"], seen["evidence"]), ("finished", "pinky_load"))
        self.assertEqual(
            seen["unload_check"]["tray_reference_state"], "eval_first_frame"
        )
        cases = {
            "PANEL_PINKY_CHECK_NOT_RUN": {"ran": False},
            "PANEL_BOX_DROPPED": dict(check, on_pinky=False),
            "PANEL_BOX_STILL_ON_SHELF": dict(check, on_pinky=False, slot_empty=False),
            "PANEL_SLOT_UNREADABLE": dict(check, slot_empty=None),
            "PANEL_PINKY_NO_REFERENCE": dict(check, tray_reference_state="not_checked"),
            "PANEL_PINKY_UNREADABLE": dict(check, on_pinky=None),
        }
        for code, p in cases.items():
            with self.subTest(code):
                seen = PanelExecutor._result_one_arm(
                    {"pinky": p}, "2a", "OUTBOUND", ("omx1",)
                )
                self.assertEqual(seen["error"], code)

    def test_wrist_check_refusal_is_its_own_cause(self):
        # as #163 for two arms: panel 3.25 checks the wrist cameras before a one-arm run too. Refused (err 13),
        # the policy never ran: PANEL_WRIST_CHECK with the numbers (no HOLD), not a missing placement / tray read
        result = {
            "exit_ok": False,
            "err": {"code": 13, "name": "err_code_13"},
            "wrist_check": {
                "ok": False,
                "why": "right move changed both wrist cameras",
                "change": {"right": {"wrist_left": 0.027, "wrist_right": 0.044}},
            },
        }
        for operation, slot, arm in (
            ("INBOUND", "2c", "omx2"),
            ("OUTBOUND", "2a", "omx1"),
        ):
            with self.subTest(operation=operation):
                seen = PanelExecutor._result_one_arm(result, slot, operation, (arm,))
                self.assertEqual(seen["state"], "failed")
                self.assertEqual(seen["error"], "PANEL_WRIST_CHECK")
                self.assertIn("wrist_right=0.044", seen["detail"])
                self.assertNotIn("PANEL_WRIST_CHECK", panel_executor.HOLD_ERRORS)
                stopped = PanelExecutor._result_one_arm(
                    result | {"stopped_by": "중지"}, slot, operation, (arm,)
                )
                self.assertEqual(stopped["error"], "PANEL_STOPPED")

    def test_arm_at_a_one_arm_home_counts_as_home(self):
        # the two-arm model says omx2 is away; the one-arm right model (?arm=right) says it is at its home
        orig = self.panel.arms
        away = dict(orig)
        away["OMX 2"] = pe.live("right", home=False)

        def arms_for(model, arm):
            return orig if arm == "right" else away

        self.panel.arms = away
        handler_get = self.panel.server.RequestHandlerClass.do_GET

        def do_get(handler):
            if handler.path.startswith("/api/arms?") and "arm=right" in handler.path:
                self.panel.arms = orig
                try:
                    return handler_get(handler)
                finally:
                    self.panel.arms = away
            return handler_get(handler)

        self.panel.server.RequestHandlerClass.do_GET = do_get
        arms = self.ex._arms_now()
        self.assertIs(arms["omx2"]["returned"], True)
        # and without one-arm routes the two-arm reading stands as before
        ex2 = PanelExecutor(pe.ROUTES, box_marker=16, base_url=self.panel.url)
        ex2._version = (3, 25, 0)
        self.assertIs(ex2._arms_now()["omx2"]["returned"], False)

    def test_no_early_tray_report_for_one_arm(self):
        self.ex.report_placement = True
        self.ex.start(self.task("OUTBOUND", "2a", "omx1"))
        self.panel.status["job"] = {
            "running": True,
            "kind": "eval",
            "info": {
                "repo_id": self.ex.repo_id,
                "tray_placed": {
                    "source": "eval_judge",
                    "operation": "OUTBOUND",
                    "mode": "on",
                },
            },
        }
        self.assertEqual(self.ex.observe()["state"], "running")


class FieldHostOneArmTest(v2.FieldHostTest):
    def finish_with(self, evidence, **extra):
        observe = self.executor.observe

        def run():
            seen = observe()
            if seen.get("state") == "finished":
                # #171 review: the real PanelExecutor's one-arm observation holds only the task's arm
                only = self.executor.tasks[-1].get("robot_ids") or []
                arms = {k: v for k, v in (seen.get("arms") or {}).items() if k in only}
                return dict(seen, evidence=evidence, arms=arms, **extra)
            return seen

        self.executor.observe = run

    def test_store_on_omx1_2a_completes_once_with_that_arm(self):
        self.host.observe_pinky(v2.pinky_state(node="N21", seq=2))
        self.executor.run_s = 0.2
        self.finish_with("shelf_load_one_arm")
        task = arm_action(self.host, "omx1", "store", "2a")
        (accepted,) = self.send(task)
        self.assertEqual(
            (accepted["source"], accepted["payload"]["status"]), ("omx1", "accepted")
        )
        t = self.executor.tasks[-1]
        self.assertEqual(
            (t["operation"], t["slot_id"], t["manipulation_mode"], t["robot_ids"]),
            ("INBOUND", "2a", "DEDICATED", ["omx1"]),
        )
        events = self.run_to_end()
        done = [e for e in events if e["payload"]["status"] == "completed"]
        self.assertEqual(len(done), 1)
        r = done[0]["payload"]["result"]
        self.assertEqual(done[0]["source"], "omx1")
        self.assertEqual(
            (r["evidence"], r["slot_id"], r["pinky_id"]),
            ("shelf_load_one_arm", "2a", "pinky1"),
        )
        self.assertEqual([a["robot_id"] for a in r["arms"]], ["omx1"])
        self.assertEqual({a["home_reached"] for a in r["arms"]}, {True})
        self.assertIsNone(self.host.active)

    def test_retrieve_on_omx2_2c_with_its_tray_check(self):
        self.host.observe_pinky(v2.pinky_state(node="N22", seq=2))
        self.executor.run_s = 0.2
        self.finish_with(
            "pinky_load",
            unload_check={
                "on_pinky": True,
                "slot_empty": True,
                "tray_reference_state": "eval_first_frame",
            },
        )
        self.send(arm_action(self.host, "omx2", "retrieve", "2c"))
        self.assertEqual(self.executor.tasks[-1]["operation"], "OUTBOUND")
        done = [e for e in self.run_to_end() if e["payload"]["status"] == "completed"]
        r = done[0]["payload"]["result"]
        self.assertEqual(
            (
                r["evidence"],
                r["unload_check"]["on_pinky"],
                [a["robot_id"] for a in r["arms"]],
            ),
            ("pinky_load", True, ["omx2"]),
        )

    def test_refusals(self):
        self.host.observe_pinky(v2.pinky_state(node="N21", seq=2))
        (e,) = self.send(arm_action(self.host, "omx1", "store", "2c"))
        self.assertEqual(
            e["payload"]["reason"], "slot_arm_mismatch"
        )  # C is omx2's column
        (e,) = self.send(arm_action(self.host, "omx2", "store", "2c"))
        self.assertEqual(
            e["payload"]["reason"], "pinky_visit_not_confirmed"
        )  # the Pinky is at N21, not N22
        self.host.observe_pinky(v2.pinky_state(node="N11", seq=3))
        (e,) = self.send(arm_action(self.host, "omx1", "store", "2a"))
        self.assertEqual(
            e["payload"]["reason"], "pinky_visit_not_confirmed"
        )  # N11 is the B stop
        msg = arm_action(self.host, "omx1", "store", "2a")
        msg["payload"]["destination_location"] = {
            "kind": "pinky",
            "robot_id": "pinky2",
            "position": "deck",
        }
        msg = v2.command("arm_action_command", "omx1", msg["payload"])
        (e,) = self.send(msg)
        self.assertEqual(e["payload"]["reason"], "invalid_transfer_locations")

    def test_one_arm_evidence_is_not_a_two_arm_completion(self):
        self.host.observe_pinky(v2.pinky_state(node="N21", seq=2))
        self.executor.run_s = 0.2
        self.finish_with(
            "shelf_align"
        )  # an aligner's word never completes a one-arm loading
        self.send(arm_action(self.host, "omx1", "store", "2a"))
        statuses = [e["payload"]["status"] for e in self.run_to_end()]
        self.assertEqual(statuses[-1], "failed")

    def test_keep_running_counts_on_the_arm_topic(self):
        self.host.observe_pinky(v2.pinky_state(node="N21", seq=2))
        self.executor.run_s = 30
        task = arm_action(self.host, "omx1", "store", "2a")
        self.send(task)
        before = self.host._deadline
        time.sleep(0.05)
        self.host.handle(
            self.keep_running(task["command_id"]) | {"target": "shelf-01"},
            endpoint="shelf-01",
            channel="cmd",
        )
        self.assertEqual(
            self.host._deadline, before
        )  # the workcell topic does not renew a one-arm task
        renew = field_wire.build_message(
            "robot_state_request",
            v2.CONTROL,
            "omx1",
            {
                "expected_boot_version": self.host.boot_version,
                "keep_running_command_id": task["command_id"],
            },
            expires_at=self.keep_running(task["command_id"])["expires_at"],
        ) | {"sent_at": self.keep_running(task["command_id"])["sent_at"]}
        self.host.handle(renew, endpoint="omx1", channel="cmd")
        self.assertGreater(self.host._deadline, before)

    def test_pinky_leaving_n21_stops_the_arm(self):
        self.host.observe_pinky(v2.pinky_state(node="N21", seq=2))
        self.executor.run_s = 30
        self.send(arm_action(self.host, "omx1", "store", "2a"))
        self.host.observe_pinky(v2.pinky_state(node="N19", stopped=False, seq=5))
        events = self.host.local_safety()
        self.assertEqual([e["payload"]["status"] for e in events], ["failed"])
        self.assertEqual(events[0]["payload"]["reason"], "pinky_left")

    def test_no_early_report_for_one_arm(self):
        self.host.report_placement = True
        self.host.observe_pinky(v2.pinky_state(node="N21", seq=2))
        self.executor.run_s = 0.2
        observe, state = self.executor.observe, {"n": 0}

        def run():
            state["n"] += 1
            if state["n"] == 1:
                return {
                    "state": "placed",
                    "evidence": "pinky_placed",
                    "grasp_released": True,
                    "placement": {},
                }
            seen = observe()
            return (
                dict(seen, evidence="shelf_load_one_arm")
                if seen.get("state") == "finished"
                else seen
            )

        self.executor.observe = run
        self.send(arm_action(self.host, "omx1", "store", "2a"))
        done = [e for e in self.run_to_end() if e["payload"]["status"] == "completed"]
        self.assertEqual(len(done), 1)
        self.assertEqual(done[0]["payload"]["result"]["evidence"], "shelf_load_one_arm")


def load_tests(loader, tests, pattern):
    """Only this file's tests (FieldHostOneArmTest borrows test_cooperative_v2's set-up, not its tests)."""
    suite = unittest.TestSuite(loader.loadTestsFromTestCase(ExecutorOneArmTest))
    suite.addTests(
        FieldHostOneArmTest(n)
        for n in vars(FieldHostOneArmTest)
        if n.startswith("test_")
    )
    return suite


if __name__ == "__main__":
    unittest.main()
