"""Panel executor against a fake panel HTTP server; no device, no real panel."""

import json
import sys
import tempfile
import threading
import unittest
import urllib.parse
from datetime import UTC, datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx import panel_executor
from roport_omx.cooperative import PROFILE, CooperativeHost
from roport_omx.mqtt_link.journal import Journal
from roport_omx.panel_executor import PanelExecutor, Route, load_routes

MODEL = "/models/real_act_2b/pretrained_model"
ROUTES = {
    ("INBOUND", "2b"): Route(MODEL, "INBOUND tray->shelf 2B | marker {box}", 40, 15)
}


def arm(side, *, home=True, gripper_open=True):
    """last_result arm; gripper_open=True means released (closed on nothing, 50.0), False = holding (59.6)."""
    grip = 50.0 if gripper_open else 59.6
    return {
        "side": side,
        "joints": {"shoulder_pan": 0.0, "gripper": grip},
        "gripper": grip,
        "gripper_open": False,
        "at_home": home,
    }


def live(side, *, home=True, grip=49.0):
    """Panel 3.11 /api/arms entry for one arm, read from the bus while idle."""
    return {
        "side": side,
        "joints": {"shoulder_pan": 0.0, "gripper": grip},
        "gripper": grip,
        "at_home": home,
    }


def align(*, far=(), check="ok", **change):
    """last_result.align of panel 3.9 after a successful shelf alignment."""
    rest_arm = {"joints": {"shoulder_pan": 1.0}, "gripper_state": "closed"}
    return {
        "ran": True,
        "ok": True,
        "box": 16,
        "slot": "2B",
        "returncode": 0,
        "stopped_by": None,
        "shelf": "ok",
        "align": "aligned",
        "released": True,
        "skipped": None,
        "rest": {
            "check": check,
            "tolerance": 5.0,
            "arms": {
                "OMX 1": rest_arm | {"side": "left", "far_joints": list(far)},
                "OMX 2": rest_arm | {"side": "right", "far_joints": []},
            },
        },
    } | change


class FakePanel:
    def __init__(self):
        self.status = {
            "version": "3.12.0",
            "armed": True,
            "job": {"running": False},
            "last_result": None,
        }
        self.models = [MODEL]
        self.starts = []
        self.start_error = None
        self.arms = {
            "OMX 1": live("left"),
            "OMX 2": live("right"),
        }  # None = panel before 3.11
        self.arms_models = []
        server = ThreadingHTTPServer(("127.0.0.1", 0), self._handler())
        self.server = server
        self.url = f"http://127.0.0.1:{server.server_address[1]}"
        threading.Thread(target=server.serve_forever, daemon=True).start()

    def _handler(self):
        panel = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _json(self, body, code=200):
                raw = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                if self.path == "/api/status":
                    self._json(panel.status)
                elif self.path.startswith("/api/arms?"):
                    query = urllib.parse.urlsplit(self.path).query
                    panel.arms_models.append(urllib.parse.parse_qs(query)["model"][0])
                    if panel.arms is None:
                        self._json({"error": "not found"}, 404)
                    else:
                        self._json({"source": "bus", "arms": panel.arms, "error": None})
                elif self.path == "/api/train/runs":
                    self._json({"runs": [{"model_path": m} for m in panel.models]})
                else:
                    self._json({"error": "unknown"}, 404)

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if self.path != "/api/eval/start":
                    self._json({"error": "unknown"}, 404)
                elif panel.start_error:
                    self._json({"error": panel.start_error}, 400)
                else:
                    panel.starts.append(body)
                    repo = f"user/eval_test_{len(panel.starts)}"
                    panel.status["job"] = {
                        "running": True,
                        "kind": "eval",
                        "info": {"repo_id": repo, "phase": "모델 불러오는 중"},
                    }
                    self._json({"ok": True, "repo_id": repo})

        return Handler

    def finish(self, **result):
        repo = self.status["job"]["info"]["repo_id"]
        self.status["job"] = {
            "running": False,
            "kind": "eval",
            "info": {"repo_id": repo},
        }
        self.status["last_result"] = {
            "kind": "eval",
            "source": "mqtt",
            "repo_id": repo,
            "exit_ok": True,
            "stopped_by": None,
            "err": None,
            "arms": {"OMX 1": arm("left"), "OMX 2": arm("right")},
            "home_check": "ok",
        } | result


class PanelExecutorTest(unittest.TestCase):
    def setUp(self):
        self.panel = FakePanel()
        self.dir = tempfile.TemporaryDirectory()
        self.journal = Journal(Path(self.dir.name) / "journal.json")
        self.executor = PanelExecutor(ROUTES, box_marker=16, base_url=self.panel.url)
        self.host = CooperativeHost(
            self.journal,
            self.executor,
            site="test",
            control="control-test",
            binding="test-binding",
        )

    def tearDown(self):
        self.panel.server.shutdown()
        self.panel.server.server_close()
        self.journal.close()
        self.dir.cleanup()

    def inbound(self, slot="2b"):
        now = datetime.now(UTC)
        return {
            "schema_version": "1.0",
            "profile": PROFILE,
            "message_id": str(uuid4()),
            "type": "shelf_inbound.requested",
            "site_id": "test",
            "source": "control-test",
            "target": "shelf-01",
            "sent_at": now.isoformat(),
            "expires_at": (now + timedelta(seconds=10)).isoformat(),
            "task_id": "t1",
            "step_id": "s1",
            "command_id": str(uuid4()),
            "execution_id": str(uuid4()),
            "payload": {
                "binding_id": "test-binding",
                "expected_boot_id": self.host.boot_id,
                "operation": "INBOUND",
                "workcell_id": "shelf-01",
                "manipulation_mode": "COOPERATIVE",
                "assigned_arm_id": None,
                "robot_ids": ["omx1", "omx2"],
                "package_id": "box-16",
                "quantity": 1,
                "slot_id": slot,
                "pinky_id": "pinky1",
                "dock_session_id": "dock-1",
                "resource_token": "grant-1",
                "model_version": "m1",
                "calibration_version": "c1",
            },
        }

    def send(self, message):
        return self.host.handle(message, channel="cmd")

    def test_routed_slot_starts_eval_and_completes_on_home_return(self):
        (accepted,) = self.send(self.inbound())
        self.assertEqual(accepted["payload"]["phase"], "ACCEPTED")
        (start,) = self.panel.starts
        self.assertEqual(
            start,
            {
                "source": "mqtt",
                "model_path": MODEL,
                "task": "INBOUND tray->shelf 2B | marker 16",
                "episodes": 1,
                "episode_s": 40.0,
                "reset_s": 15.0,
                "target_marker": 16,
                "end_on_return": True,
            },
        )
        (progress,) = self.host.poll()
        self.assertEqual(progress["payload"]["step"], "모델 불러오는 중")
        self.panel.finish()
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(
            (done["type"], p["phase"]), ("shelf_inbound.completed", "COMPLETE")
        )
        self.assertEqual(
            p["arms"]["omx1"],
            {"connected": True, "grasping": False, "returned": True, "safe": True},
        )
        self.assertIs(p["departure_ready"], True)
        self.assertIsNone(p["placement_confirmed"])

    def test_model_cache_fetches_on_fresh_boot_and_refreshes_after_ttl(self):
        clock = 1.0
        with (
            patch.object(
                panel_executor, "time", SimpleNamespace(monotonic=lambda: clock)
            ),
            patch.object(self.executor, "_call", wraps=self.executor._call) as call,
        ):
            self.assertEqual(self.executor._model_paths(), {MODEL})
            self.panel.models = []
            clock += 1
            self.assertEqual(self.executor._model_paths(), {MODEL})
            self.assertEqual(call.call_count, 1)
            clock += panel_executor.MODELS_TTL_S + 1
            self.assertEqual(self.executor._model_paths(), set())
            self.assertEqual(call.call_count, 2)

    def test_refusals_happen_before_anything_starts(self):
        cases = [("NOT_ARMED", {"armed": False}), ("NOT_ARMED", {"armed": None})]
        for code, change in cases:
            with self.subTest(change):
                self.panel.status |= change
                (event,) = self.send(self.inbound())
                self.assertEqual(event["payload"]["error_code"], code)
        self.panel.status["armed"] = True
        (event,) = self.send(self.inbound(slot="1a"))
        self.assertEqual(event["payload"]["error_code"], "UNSUPPORTED_SLOT")
        self.panel.models = []
        self.executor._models = (0.0, set())
        (event,) = self.send(self.inbound())
        self.assertEqual(event["payload"]["error_code"], "MODEL_NOT_READY")
        self.assertEqual(self.panel.starts, [])

    def test_route_filter_is_sent_with_the_start(self):
        self.executor.routes = {
            ("INBOUND", "2b"): Route(
                MODEL, "t {box}", 40, 15, "중", False, {"async_mode": True}
            )
        }
        self.send(self.inbound())
        (start,) = self.panel.starts
        self.assertEqual(
            (start["output_filter"], start["filter_gripper"], start["async_mode"]),
            ("중", False, True),
        )

    def test_unknown_box_is_refused(self):
        self.executor.box_marker = None
        (event,) = self.send(self.inbound())
        self.assertEqual(event["payload"]["error_code"], "BOX_MARKER_UNKNOWN")
        self.assertEqual(self.panel.starts, [])

    def test_arm_away_from_home_or_closed_gripper_fails_and_holds(self):
        for arms in (
            {"OMX 1": arm("left"), "OMX 2": arm("right", home=False)},
            {"OMX 1": arm("left", gripper_open=False), "OMX 2": arm("right")},
            {"OMX 1": arm("left", home=None), "OMX 2": arm("right")},
        ):
            with self.subTest(arms):
                self.host.clear_hold()
                self.send(self.inbound())
                self.panel.finish(arms=arms)
                events = self.host.poll()
                self.assertEqual(events[-1]["type"], "shelf_inbound.failed")
                self.assertEqual(
                    events[-1]["payload"]["error_code"], "ARM_NOT_RETURNED"
                )
                self.assertIs(events[-1]["payload"]["departure_ready"], False)
                self.assertIsNotNone(self.host.hold)

    def test_panel_error_or_stop_fails_with_its_code(self):
        self.send(self.inbound())
        self.panel.finish(exit_ok=False, err={"code": "err_code_7", "name": "E7"})
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["error_code"], "PANEL_ERR_CODE_7")
        self.assertIs(failed["payload"]["arms"]["omx1"]["safe"], False)
        self.host.clear_hold()
        self.send(self.inbound())
        self.panel.finish(exit_ok=False, stopped_by="비상 정지")
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["error_code"], "PANEL_EMERGENCY_STOP")
        self.assertIsNotNone(self.host.hold)

    def test_operator_stop_is_reported_without_hold(self):
        # 즉시 중단 can leave a shutdown error behind; the stop is still the cause.
        cases = [
            ("중지", None, "PANEL_STOPPED"),
            ("즉시 중단", {"code": 7, "name": "err_code_7"}, "PANEL_ABORTED"),
        ]
        for stopped_by, err, code in cases:
            with self.subTest(stopped_by):
                self.send(self.inbound())
                self.panel.finish(exit_ok=False, stopped_by=stopped_by, err=err)
                (failed,) = self.host.poll()
                self.assertEqual(failed["type"], "shelf_inbound.failed")
                self.assertEqual(failed["payload"]["error_code"], code)
                self.assertIs(failed["payload"]["departure_ready"], False)
                self.assertIsNone(self.host.hold)
        # Still gated by the panel: a stop turns armed off until a person turns it on.
        self.panel.status["armed"] = False
        (event,) = self.send(self.inbound())
        self.assertEqual(event["payload"]["error_code"], "NOT_ARMED")
        self.assertEqual(len(self.panel.starts), 2)

    def test_start_refused_by_panel_fails_and_holds(self):
        self.panel.start_error = "카메라 확인 검사 실패"
        events = self.send(self.inbound())
        self.assertEqual(
            [e["payload"]["phase"] for e in events], ["ACCEPTED", "FAILED"]
        )
        self.assertEqual(events[1]["payload"]["error_code"], "EXECUTOR_START_FAILED")
        self.assertIn("카메라", events[1]["payload"]["detail"])
        self.assertIsNotNone(self.host.hold)

    def test_alignment_is_awaited_and_judged_by_its_rest_check(self):
        self.send(self.inbound())
        self.host.poll()
        # Policy left the arms away from home; alignment then ends at its rest pose.
        arms = {"OMX 1": arm("left", home=False), "OMX 2": arm("right", home=False)}
        self.panel.finish(arms=arms, align=align(), stage="align", final=False)
        (waiting,) = self.host.poll()
        self.assertEqual(waiting["payload"]["step"], "align")
        repo = self.panel.status["last_result"]["repo_id"]
        self.panel.status["job"] = {
            "running": True,
            "kind": "align",
            "info": {"eval_repo_id": repo, "phase": "선반 확인 중"},
        }
        (aligning,) = self.host.poll()
        self.assertEqual(aligning["payload"]["step"], "선반 확인 중")
        self.panel.status["job"] = {"running": False}
        self.panel.status["last_result"] |= {"stage": "done", "final": True}
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(done["type"], "shelf_inbound.completed")
        self.assertEqual(p["evidence"], "shelf_align")
        self.assertIs(p["arms"]["omx1"]["returned"], True)

    def test_alignment_failures(self):
        stopped = {"ok": False, "returncode": 2, "shelf": None, "align": None}
        cases = [
            (
                "PANEL_ERR_CODE_7",
                {
                    "exit_ok": False,
                    "err": {"code": "err_code_7"},
                    "align": align(ran=False, ok=False, skipped="평가 오류", rest=None),
                },
            ),
            ("PANEL_ALIGN_SKIPPED", {"align": align(ran=False, ok=False, rest=None)}),
            (
                "PANEL_ALIGN_STOPPED",
                {"align": align(stopped_by="즉시 중단", **stopped)},
            ),
            (
                "PANEL_EMERGENCY_STOP",
                {"align": align(stopped_by="비상 정지", **stopped)},
            ),
            ("PANEL_ALIGN_FAILED", {"align": align(ok=False, shelf="not_found")}),
            ("ARM_NOT_RETURNED", {"align": align(far=["elbow_flex"], check="far")}),
            ("ARM_NOT_RETURNED", {"align": align(check="unknown")}),
            (
                "ARM_NOT_RETURNED",
                {
                    "arms": {
                        "OMX 1": arm("left", gripper_open=False),
                        "OMX 2": arm("right"),
                    },
                    "align": align(),
                },
            ),
        ]
        for code, change in cases:
            with self.subTest(code=code, change=change):
                self.host.clear_hold()
                self.send(self.inbound())
                self.panel.finish(stage="done", final=True, **change)
                events = self.host.poll()
                self.assertEqual(events[-1]["type"], "shelf_inbound.failed")
                self.assertEqual(events[-1]["payload"]["error_code"], code)
                if code == "PANEL_ALIGN_STOPPED":  # a person's stop, arms set down
                    self.assertIsNone(self.host.hold)
                else:
                    self.assertIsNotNone(self.host.hold)

    def test_hold_waiting_for_a_person_fails_at_once(self):
        stopped = {"ok": False, "returncode": 2, "released": None, "rest": None}
        cases = [
            ("PANEL_EMERGENCY_STOP", align(stopped_by="비상 정지", **stopped)),
            (
                "PANEL_ALIGN_RELEASE_FAILED",
                align(stopped_by="즉시 중단", **stopped | {"released": False}),
            ),
            ("PANEL_HOLD", align(**stopped)),
        ]
        for code, change in cases:
            with self.subTest(code):
                self.host.clear_hold()
                self.send(self.inbound())
                self.panel.finish(align=change, stage="hold", final=False)
                (failed,) = self.host.poll()
                self.assertEqual(failed["payload"]["error_code"], code)
                self.assertIsNotNone(self.host.hold)

    def test_lost_result_becomes_unavailable_after_grace(self):
        self.send(self.inbound())
        self.panel.status["job"] = {"running": False}
        original = panel_executor.RESULT_GRACE_S
        panel_executor.RESULT_GRACE_S = 0.0
        try:
            events = self.host.poll()
        finally:
            panel_executor.RESULT_GRACE_S = original
        self.assertEqual(events[-1]["payload"]["error_code"], "RESULT_UNAVAILABLE")

    def status(self):
        now = datetime.now(UTC)
        message = self.inbound() | {
            "type": "status.requested",
            "expires_at": (now + timedelta(seconds=10)).isoformat(),
            "payload": {"binding_id": "test-binding"},
        }
        (event,) = self.send(message)
        return event["payload"]

    def test_return_detected_by_panel_completes_without_model_home(self):
        self.send(self.inbound())
        self.assertIs(self.panel.starts[-1]["end_on_return"], True)
        arms = {"OMX 1": arm("left", home=None), "OMX 2": arm("right", home=False)}
        last = {"episode": 0, "returned": True, "elapsed_s": 21.9}
        self.panel.finish(arms=arms, last_episode=last)
        events = self.host.poll()
        self.assertEqual(events[-1]["type"], "shelf_inbound.completed")
        self.assertEqual(events[-1]["payload"]["evidence"], "arm_return_only")
        # a gripper still on the box edge above 54.6 is not released even after a detected return
        self.send(self.inbound())
        arms = {"OMX 1": arm("left", gripper_open=False), "OMX 2": arm("right")}
        self.panel.finish(arms=arms, last_episode=last)
        events = self.host.poll()
        self.assertEqual(events[-1]["type"], "shelf_inbound.failed")
        self.assertEqual(events[-1]["payload"]["error_code"], "ARM_NOT_RETURNED")

    def test_status_reports_live_arms_with_closed_grippers_as_empty(self):
        payload = self.status()
        self.assertTrue(payload["ready"])
        self.assertEqual(self.panel.arms_models, [MODEL])
        for flags in payload["arms"].values():
            self.assertEqual(
                flags,
                {"connected": True, "grasping": False, "returned": True, "safe": True},
            )

    def test_live_arms_half_gripper_or_away_from_home(self):
        self.panel.arms = {
            "OMX 1": live("left", grip=59.6),
            "OMX 2": live("right", home=False, grip=54.6),
        }
        arms = self.status()["arms"]
        self.assertIs(arms["omx1"]["grasping"], True)
        self.assertIs(arms["omx2"]["grasping"], False)
        self.assertIs(arms["omx2"]["returned"], False)
        self.panel.arms = {
            "OMX 1": live("left", home=None) | {"gripper": None, "joints": {"x": 1}}
        }
        arms = self.status()["arms"]
        self.assertEqual(
            arms["omx1"],
            {"connected": True, "grasping": None, "returned": None, "safe": True},
        )
        self.assertIsNone(arms["omx2"]["connected"])

    def test_panel_older_than_3_12_is_not_ready(self):
        for version in ("3.11.0", "3.9.1", None, "dev"):
            with self.subTest(version):
                self.panel.status["version"] = version
                payload = self.status()
                self.assertFalse(payload["ready"])
                self.assertIn("PANEL_VERSION", payload["executor_error"])
        self.panel.status["version"] = "3.12.0"
        self.assertTrue(self.status()["ready"])

    def test_old_panel_or_busy_leaves_arms_unknown(self):
        self.panel.arms = None
        arms = self.status()["arms"]
        self.assertTrue(all(v is None for a in arms.values() for v in a.values()))
        self.panel.arms = {"OMX 1": live("left"), "OMX 2": live("right")}
        self.panel.status["job"] = {"running": True}
        before = len(self.panel.arms_models)
        payload = self.status()
        self.assertFalse(payload["ready"])
        self.assertEqual(
            len(self.panel.arms_models), before
        )  # no bus read while a run owns it

    def test_routes_file(self):
        path = Path(self.dir.name) / "routes.json"
        route = {
            "operation": "INBOUND",
            "slot_id": "2b",
            "model_path": MODEL,
            "task": "t",
            "episode_s": 40,
            "reset_s": 15,
        }
        path.write_text(json.dumps({"routes": [route]}))
        self.assertEqual(load_routes(path)[("INBOUND", "2b")].model_path, MODEL)
        for bad in ({"slot_id": "3a"}, {"operation": "LOAD"}, {"model_path": ""}):
            with self.subTest(bad):
                path.write_text(json.dumps({"routes": [route | bad]}))
                with self.assertRaises(ValueError):
                    load_routes(path)
        path.write_text(json.dumps({"routes": [route, route]}))
        with self.assertRaises(ValueError):
            load_routes(path)
        path.write_text(json.dumps({"routes": [route | {"output_filter": "최강"}]}))
        with self.assertRaises(ValueError):
            load_routes(path)
        path.write_text(json.dumps({"routes": [route | {"eval_options": {"x": 1}}]}))
        with self.assertRaises(ValueError):
            load_routes(path)
        path.write_text(json.dumps({"routes": [route | {"output_filter": "중"}]}))
        self.assertEqual(load_routes(path)[("INBOUND", "2b")].output_filter, "중")
        example = ROOT / "config/cooperative_routes.example.json"
        self.assertEqual(list(load_routes(example)), [("INBOUND", "2b")])


if __name__ == "__main__":
    unittest.main()
