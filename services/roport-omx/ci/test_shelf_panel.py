"""The reviewed shelf panel contract, using in-memory panel responses."""

import json
import tempfile
import unittest
from datetime import UTC, datetime
from pathlib import Path
from unittest.mock import patch

from roport_omx import shelf_panel
from roport_omx.shelf_panel import PanelExecutor, Route, load_routes

ROOT = Path(__file__).resolve().parents[1]
MODEL = "test-model"
ROUTES = {("INBOUND", "2b", "COOPERATIVE"): Route(MODEL, "INBOUND 2B {box}", 40, 15)}


class ShelfPanelTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def test_readiness_reports_the_bus_read_behind_the_arm_flags(self):
        executor = PanelExecutor(ROUTES)
        read_at = datetime.now(UTC).isoformat()
        arms = {
            name: {
                "side": name,
                "at_home": True,
                "joints": {"shoulder_pan": 0.0, "gripper": 49.0},
            }
            for name in ("left", "right")
        }
        status = {"version": "3.21.1", "armed": True, "job": {"running": False}}
        with (
            # readiness() also keeps the model list warm off the MQTT loop (#156); not under test
            patch.object(executor, "_refresh_models"),
            patch.object(
                executor,
                "_call",
                side_effect=[
                    status,
                    {"source": "bus", "at": read_at, "arms": arms},
                    status,
                    {"source": "bus", "arms": arms},
                    status | {"job": {"running": True}},
                ],
            ),
        ):
            ready = executor.readiness()
            self.assertEqual(ready["arms_read"]["at"], read_at)
            self.assertEqual(
                ready["arms_read"]["joints"],
                {f"omx{i}": row["joints"] for i, row in enumerate(arms.values(), 1)},
            )
            self.assertIsNone(executor.readiness()["arms_read"])
            self.assertIsNone(executor.readiness()["arms_read"])

    def test_route_versions_for_v2(self):
        # #135 review: v2 runs a route only for the exact model/calibration pair it declares
        path = Path(self.dir.name) / "routes.json"
        route = {
            "operation": "INBOUND",
            "slot_id": "2b",
            "model_path": MODEL,
            "task": "t",
            "reset_s": 5,
        }
        path.write_text(
            json.dumps(
                {
                    "model_version": "m-top",
                    "calibration_version": "c-top",
                    "routes": [
                        route,
                        route | {"slot_id": "1b", "model_version": "m-1b"},
                    ],
                }
            )
        )
        routes = load_routes(path)
        executor = PanelExecutor(routes, box_marker=16)
        task = {"operation": "INBOUND", "manipulation_mode": "COOPERATIVE"}
        self.assertEqual(
            executor.versions(task | {"slot_id": "2b"}),
            {"model_version": "m-top", "calibration_version": "c-top"},
        )
        self.assertEqual(
            executor.versions(task | {"slot_id": "1b"})["model_version"], "m-1b"
        )
        self.assertIsNone(executor.versions(task | {"slot_id": "3b"}))
        path.write_text(json.dumps({"routes": [route]}))
        bare = PanelExecutor(load_routes(path), box_marker=16)
        self.assertIsNone(bare.versions(task | {"slot_id": "2b"}))  # v2 refuses it
        path.write_text(json.dumps({"routes": [route | {"model_version": ""}]}))
        with self.assertRaises(ValueError):
            load_routes(path)
        load_routes(ROOT / "config/shelf_routes.example.json")

    def test_placement_judge_counts_only_in_the_slot_it_watches(self):
        # the judge's camera area and shelf edge are those of 2B (panel handover_fk.json): a
        # "placed" for a 1B or 3B mission says nothing about that slot
        result = {
            "placement": {
                "on_shelf": True,
                "slot": "1B",
                "box": 16,
                "source": "eval_judge",
                "tips_clear": True,
            },
            "handover": {
                "mode": "handover",
                "end_reason": "placed",
                "judge": {"box": {"id": 16, "flat": True}},
            },
        }
        self.assertFalse(shelf_panel._judge_placed(result, "1B", 16))
        two_b = result | {"placement": result["placement"] | {"slot": "2B"}}
        self.assertTrue(shelf_panel._judge_placed(two_b, "2B", 16))
        self.assertTrue(shelf_panel._judge_placed(two_b, "2b", 16))
        self.assertFalse(shelf_panel._judge_placed(two_b, "2B", 17))

    def test_wrist_check_refusal_is_reported_with_its_numbers(self):
        # task/omx-cooperative #157: panel 3.22.0 refuses to start when the wrist cameras cannot be
        # told apart (err 13); the arms wait at home: PANEL_WRIST_CHECK with the numbers
        result = {
            "exit_ok": False,
            "err": {"code": 13, "name": "err_code_13", "detail": "ambiguous"},
            "wrist_check": {
                "ok": False,
                "why": "left move changed both wrist cameras",
                "change": {"left": {"wrist_left": 0.031, "wrist_right": 0.024}},
            },
        }
        seen = PanelExecutor._result(result, "2b", "INBOUND", 16)
        self.assertEqual(seen["state"], "failed")
        self.assertEqual(seen["error"], "PANEL_WRIST_CHECK")
        self.assertIn("left move changed both wrist cameras", seen["detail"])
        self.assertIn("wrist_left=0.031", seen["detail"])
        self.assertNotIn("PANEL_WRIST_CHECK", shelf_panel.HOLD_ERRORS)  # no HOLD
        # a later home-return failure keeps the first cause within the 200-character cap
        detail = shelf_panel._then("home return did not finish", seen["detail"] * 5)
        self.assertTrue(detail.startswith("home return did not finish | first: "))
        self.assertEqual(len(detail), 200)

    def test_possible_drop_waits_for_a_person(self):
        # task/omx-cooperative #156 (TK 2026-11-24): a box that may be on the floor is a HOLD
        self.assertIn("PANEL_BOX_DROPPED", shelf_panel.HOLD_ERRORS)
        self.assertIn("PANEL_BOX_NOT_ON_SHELF", shelf_panel.HOLD_ERRORS)
        self.assertNotIn("PANEL_BOX_STILL_ON_SHELF", shelf_panel.HOLD_ERRORS)

    def test_expired_model_list_answers_at_once_and_refreshes_in_background(self):
        # task/omx-cooperative #156: a transfer's checks never wait for the panel's model list
        executor = PanelExecutor(ROUTES)
        clock = 1.0
        runs = [{"runs": [{"model_path": MODEL}]}, {"runs": []}]
        with (
            patch.object(shelf_panel.time, "monotonic", side_effect=lambda: clock),
            patch.object(executor, "_call", side_effect=runs) as call,
        ):
            self.assertEqual(executor._model_paths(), {MODEL})  # first call waits
            clock += shelf_panel.MODELS_TTL_S + 1
            self.assertEqual(executor._model_paths(), {MODEL})  # known list at once
            executor._models_refresh.join(5)
            self.assertEqual(call.call_count, 2)
            self.assertEqual(executor._model_paths(), set())
