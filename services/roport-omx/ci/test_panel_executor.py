"""Panel executor against a fake panel HTTP server; no device, no real panel."""

import json
import sys
import tempfile
import threading
import unittest
from datetime import UTC, datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
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
    return {
        "side": side,
        "joints": {"shoulder_pan": 0.0, "gripper": 60.0 if gripper_open else 10.0},
        "gripper_open": gripper_open,
        "at_home": home,
    }


class FakePanel:
    def __init__(self):
        self.status = {"armed": True, "job": {"running": False}, "last_result": None}
        self.models = [MODEL]
        self.starts = []
        self.start_error = None
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

    def test_start_refused_by_panel_fails_and_holds(self):
        self.panel.start_error = "카메라 확인 검사 실패"
        events = self.send(self.inbound())
        self.assertEqual(
            [e["payload"]["phase"] for e in events], ["ACCEPTED", "FAILED"]
        )
        self.assertEqual(events[1]["payload"]["error_code"], "EXECUTOR_START_FAILED")
        self.assertIn("카메라", events[1]["payload"]["detail"])
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
        example = ROOT / "config/cooperative_routes.example.json"
        self.assertEqual(list(load_routes(example)), [("INBOUND", "2b")])


if __name__ == "__main__":
    unittest.main()
