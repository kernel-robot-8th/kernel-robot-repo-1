"""Panel executor against a fake panel HTTP server; no device, no real panel."""

import json
import sys
import tempfile
import threading
import time
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
from roport_omx.cooperative import PROFILE, CooperativeHost, StartRefused
from roport_omx.mqtt_link.journal import Journal
from roport_omx.panel_executor import PanelExecutor, Route, load_routes

MODEL = "/models/real_act_2b/pretrained_model"
ROUTES = {
    ("INBOUND", "2b", "COOPERATIVE"): Route(
        MODEL, "INBOUND tray->shelf 2B | marker {box}", 40, 15
    )
}
OUT_MODEL = "/models/outbound_act_re1112x37_60k_1113/pretrained_model"
OUT_ROUTE = {
    ("OUTBOUND", "2b", "COOPERATIVE"): Route(
        OUT_MODEL, "OUTBOUND shelf 2B->pinky | marker {box}", 40, 15
    )
}


def pinky(on_pinky=True, on_shelf=False, **change):
    """last_result.pinky of the panel's camera-only check after an unloading evaluation.

    Shaped like panel 3.20 in the field (11/21 logs/pinky_result.json): the commanded slot's own
    slot_empty, and the tray compared with the photo taken right before this mission.
    """
    return {
        "ran": True,
        "on_pinky": on_pinky,
        "on_shelf": on_shelf,
        "slot": "2B",
        "slot_empty": None if not isinstance(on_shelf, bool) else not on_shelf,
        "pinky": "ok" if on_pinky else "tray_empty",
        "shelf": "ok" if on_shelf else "marker_not_found",
        "box": 16,
        "tray_reference": "pre_mission",
        "tray_reference_state": "ok",
        "pre_mission_match": True,
        "returncode": 0,
        "skipped": None,
    } | change


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
        "on_shelf": True,
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
            "version": "3.20.0",
            "armed": True,
            "job": {"running": False},
            "last_result": None,
            "gpu_link": {
                "ok": False,  # SSH tunnel; not needed for a mission since 3.13
                "infer": {
                    "transport": "udp",
                    "udp": {"ok": True, "stage": "ok", "error": "", "at": None},
                },
            },
        }
        # seconds since the panel's last UDP check; None keeps "at" as set
        self.udp_age = 1.0
        self.home_returns = 0  # POST /api/home/return calls
        self.home_return_bodies = []
        self.home_return_error = None
        self.models = [MODEL]
        self.starts = []
        self.start_error = None
        self.start_refusal = None  # JSON body of a 400 refusal (panel 3.20 tray check)
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
                    body = json.loads(json.dumps(panel.status))
                    infer = (body.get("gpu_link") or {}).get("infer") or {}
                    if panel.udp_age is not None and isinstance(infer.get("udp"), dict):
                        infer["udp"]["at"] = time.time() - panel.udp_age
                    self._json(body)
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
                if self.path == "/api/home/return":
                    if panel.home_return_error:
                        self._json({"error": panel.home_return_error}, 400)
                        return
                    panel.home_returns += 1
                    panel.home_return_bodies.append(body)
                    panel.status["job"] = {
                        "running": True,
                        "kind": "home",
                        "info": {"phase": "홈 위치로 이동 중"},
                    }
                    self._json({"ok": True, "mode": "return_home"})
                elif self.path != "/api/eval/start":
                    self._json({"error": "unknown"}, 404)
                elif panel.start_refusal:
                    self._json(panel.start_refusal, 400)
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

    def place(self, *, on_shelf=True, slot="2B", **evidence):
        """Panel 3.17: the evaluation ended, the aligner read the shelf and now aligns."""
        info = self.status["job"]["info"]
        repo = info.get("repo_id") or info.get("eval_repo_id")
        self.status["job"] = {
            "running": True,
            "kind": "align",
            "info": {"phase": "정렬 중", "eval_repo_id": repo},
        }
        self.status["last_result"] = {
            "kind": "eval",
            "source": "mqtt",
            "repo_id": repo,
            "exit_ok": True,
            "stopped_by": None,
            "err": None,
            "arms": {"OMX 1": arm("left"), "OMX 2": arm("right")},
            "stage": "align",
            "final": False,
            "placement": {
                "on_shelf": on_shelf,
                "slot": slot,
                "box": 16,
                "source": "align",
                "observed_at": "2026-11-18T01:00:00.000+09:00",
                "observed_ts": time.time() - 0.5,
                "observed_clock": "capture",
                "reported_at": "2026-11-18T01:00:00.200+09:00",
                # panel 3.20: the arms measured when the evaluation ended on its return
                "eval_end": {
                    "end_reason": "returned",
                    "all_at_home": True,
                    "arms": {
                        "left": {"at_home": True, "gripper": 50.0},
                        "right": {"at_home": True, "gripper": 49.0},
                    },
                },
            }
            | evidence,
        }

    def finish(self, **result):
        info = self.status["job"]["info"]
        repo = info.get("repo_id") or info.get("eval_repo_id")
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
                "package_id": "16",
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

    def test_routed_slot_starts_eval_and_completes_after_shelf_alignment(self):
        # the base start body: a panel before 3.17 takes no slot / pinky_id
        self.panel.status["version"] = "3.16.0"
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
                "shelf_align": True,
                "fast_release": True,
            },
        )
        (progress,) = self.host.poll()
        self.assertEqual(progress["payload"]["step"], "모델 불러오는 중")
        self.panel.finish(align=align(), stage="done", final=True)
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(
            (done["type"], p["phase"]), ("shelf_inbound.completed", "COMPLETE")
        )
        self.assertEqual(p["evidence"], "shelf_align")
        self.assertEqual(
            p["arms"]["omx1"],
            {"connected": True, "grasping": False, "returned": True, "safe": True},
        )
        self.assertIs(p["departure_ready"], True)
        self.assertIsNone(p["placement_confirmed"])

    def test_expiry_after_panel_checks_or_acceptance_never_posts_start(self):
        for stage in ("readiness", "check", "accepted_save", "panel_post"):
            with self.subTest(stage=stage):
                self.host.clear_hold()
                command = self.inbound()
                clock = datetime.now(UTC)
                ready, check, start, save = (
                    self.executor.readiness,
                    self.executor.check,
                    self.executor.start,
                    self.journal.save,
                )

                def delay():
                    nonlocal clock
                    clock += timedelta(seconds=20)

                def delayed_ready():
                    result = ready()
                    if stage == "readiness":
                        delay()
                    return result

                def delayed_check(task):
                    check(task)
                    if stage == "check":
                        delay()

                def delayed_save(data):
                    save(data)
                    if stage == "accepted_save" and data["active"]:
                        delay()

                def delayed_start(task):
                    if stage == "panel_post":
                        delay()
                    start(task)

                with (
                    patch("roport_omx.mqtt_link.messages.datetime") as clock_type,
                    patch.object(self.executor, "readiness", delayed_ready),
                    patch.object(self.executor, "check", delayed_check),
                    patch.object(self.executor, "start", delayed_start),
                    patch.object(self.journal, "save", delayed_save),
                ):
                    clock_type.now.side_effect = lambda tz: clock
                    clock_type.fromisoformat = datetime.fromisoformat
                    events = self.send(command)
                self.assertEqual(self.panel.starts, [])
                self.assertIn(events[-1]["payload"]["phase"], {"REJECTED", "FAILED"})
                self.assertIn("expired_command", str(events[-1]["payload"]))

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
        (event,) = self.send(self.inbound(slot="1b"))  # both arms, but no model there
        self.assertEqual(event["payload"]["error_code"], "UNSUPPORTED_SLOT")
        self.panel.models = []
        self.executor._models = (0.0, set())
        (event,) = self.send(self.inbound())
        self.assertEqual(event["payload"]["error_code"], "MODEL_NOT_READY")
        self.assertEqual(self.panel.starts, [])

    def test_route_filter_is_sent_with_the_start(self):
        self.executor.routes = {
            ("INBOUND", "2b", "COOPERATIVE"): Route(
                MODEL, "t {box}", 40, 15, "중", False, {"async_mode": True}
            )
        }
        self.send(self.inbound())
        (start,) = self.panel.starts
        self.assertEqual(
            (start["output_filter"], start["filter_gripper"], start["async_mode"]),
            ("중", False, True),
        )

    def test_package_id_names_the_box(self):
        # same rule as OMX3: the box's marker number as a string, no leading zero
        # (" 16" or full-width digits never reach the executor: the Host refuses them as fields)
        for bad in ("box-16", "016", "18", "23", "1.6", "100"):
            with self.subTest(package_id=bad):
                message = self.inbound()
                message["payload"]["package_id"] = bad
                (event,) = self.send(message)
                self.assertEqual(event["payload"]["error_code"], "INVALID_TARGET")
        self.assertEqual(self.panel.starts, [])
        message = self.inbound()
        message["payload"]["package_id"] = "17"
        self.send(message)
        (start,) = self.panel.starts
        self.assertEqual(
            (start["task"], start["target_marker"]),
            ("INBOUND tray->shelf 2B | marker 17", 17),
        )

    def test_route_takes_only_the_boxes_its_model_learned(self):
        key = ("INBOUND", "2b", "COOPERATIVE")
        self.executor.routes = {
            key: Route(MODEL, "t {box}", 40, 15, markers=frozenset({16, 20}))
        }
        message = self.inbound()
        message["payload"]["package_id"] = "17"
        (event,) = self.send(message)
        self.assertEqual(event["payload"]["error_code"], "MODEL_NOT_READY")
        message = self.inbound()
        message["payload"]["package_id"] = "20"
        (event,) = self.send(message)
        self.assertEqual(event["payload"]["phase"], "ACCEPTED")

    def test_one_arm_slot_is_refused_even_with_a_route(self):
        # 2026-11-21 review: panel 3.21 has no single-arm control-tower evaluation; with a
        # route for 2a the start would run its default arm set (both arms)
        self.executor.routes |= {
            ("INBOUND", "2a", "DEDICATED"): self.executor.routes[
                ("INBOUND", "2b", "COOPERATIVE")
            ]
        }
        message = self.inbound(slot="2a")
        message["payload"] |= {
            "manipulation_mode": "DEDICATED",
            "assigned_arm_id": "omx1",
            "robot_ids": ["omx1"],
        }
        (event,) = self.send(message)
        self.assertEqual(event["payload"]["error_code"], "UNSUPPORTED_SLOT")
        self.assertEqual(self.panel.starts, [])  # nothing sent to the panel
        task = {
            "manipulation_mode": "DEDICATED",
            "operation": "INBOUND",
            "slot_id": "2a",
        }
        with self.assertRaises(StartRefused):
            self.executor.start(task)
        self.assertEqual(self.panel.starts, [])

    def test_one_arm_slot_needs_its_own_route(self):
        message = self.inbound(slot="2a")
        message["payload"] |= {
            "manipulation_mode": "DEDICATED",
            "assigned_arm_id": "omx1",
            "robot_ids": ["omx1"],
        }
        (event,) = self.send(message)  # no single-arm model yet
        self.assertEqual(event["payload"]["error_code"], "UNSUPPORTED_SLOT")

    def test_arm_away_from_home_or_closed_gripper_fails(self):
        both = {"OMX 1": arm("left"), "OMX 2": arm("right")}
        for arms, rest in (
            (
                both,
                align(far=("shoulder_pan",)),
            ),  # alignment ended away from its rest pose
            (
                {"OMX 1": arm("left", gripper_open=False), "OMX 2": arm("right")},
                align(),
            ),
            (both, align(check="unknown")),
        ):
            with self.subTest(arms=arms, rest=rest["rest"]["check"]):
                self.host.clear_hold()
                self.send(self.inbound())
                self.panel.finish(arms=arms, align=rest, stage="done", final=True)
                events = self.host.poll()
                self.assertEqual(events[-1]["type"], "shelf_inbound.failed")
                self.assertEqual(
                    events[-1]["payload"]["error_code"], "ARM_NOT_RETURNED"
                )
                self.assertIs(events[-1]["payload"]["departure_ready"], False)
                self.assertEqual(
                    events[-1]["payload"]["mission_result"], "MISSION_FAILED"
                )
                self.assertIsNone(self.host.hold)  # live read: both arms home, released

    def test_panel_error_or_stop_fails_with_its_code(self):
        self.send(self.inbound())
        self.panel.finish(exit_ok=False, err={"code": "err_code_7", "name": "E7"})
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["error_code"], "PANEL_ERR_CODE_7")
        # arms already home and released (live read): no home move needed, no HOLD
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_FAILED")
        self.assertEqual(self.panel.home_returns, 0)
        self.assertIsNone(self.host.hold)
        self.send(self.inbound())
        self.panel.finish(exit_ok=False, stopped_by="비상 정지")
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["error_code"], "PANEL_EMERGENCY_STOP")
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
        self.assertEqual(self.panel.home_returns, 0)  # a frozen arm waits for a person
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

    def test_failed_mission_sends_arms_home_then_reports_without_hold(self):
        def fail_with_arm_away(home):
            self.panel.arms = {"OMX 1": live("left"), "OMX 2": live("right", home=home)}
            self.send(self.inbound())
            self.panel.finish(align=align(on_shelf=False, ok=False), final=True)
            (progress,) = self.host.poll()  # the panel moves the arms home first
            self.assertEqual(progress["payload"]["step"], "home_return")
            self.assertEqual(self.panel.home_returns, self.homes + 1)
            self.homes += 1
            self.assertEqual(
                self.host.poll(), []
            )  # still moving: same step, no new event

        self.homes = 0
        for home in (False, None):
            with self.subTest(home=home):
                fail_with_arm_away(home)
                self.panel.status["job"] = {"running": False, "kind": "home"}
                self.panel.arms["OMX 2"] = live("right")  # read at home afterwards
                (failed,) = self.host.poll()
                p = failed["payload"]
                self.assertEqual(p["error_code"], "PANEL_BOX_NOT_ON_SHELF")
                self.assertEqual(p["mission_result"], "MISSION_FAILED")
                self.assertIs(p["arms"]["omx2"]["returned"], True)
                self.assertFalse(p["departure_ready"])
                self.assertIsNone(self.host.hold)
        # the move ended but an arm is still away: a person takes over
        fail_with_arm_away(False)
        self.panel.status["job"] = {"running": False, "kind": "home"}
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
        self.assertIsNotNone(self.host.hold)
        self.host.clear_hold()
        # the panel refuses the home move (e.g. no start pose recorded): HOLD at once
        self.panel.home_return_error = "돌아갈 홈 자세 기록이 없습니다."
        self.panel.arms = {"OMX 1": live("left"), "OMX 2": live("right", home=False)}
        self.send(self.inbound())
        self.panel.finish(exit_ok=False, err={"code": "err_code_7"})
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
        self.assertIn("home return refused", failed["payload"]["detail"])
        self.assertIsNotNone(self.host.hold)

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
            (
                "PANEL_ALIGN_STOPPED",
                {"align": align(stopped_by="즉시 중단", **stopped)},
            ),
            (
                "PANEL_EMERGENCY_STOP",
                {"align": align(stopped_by="비상 정지", **stopped)},
            ),
            # the aligner found the box marker nowhere on the shelf, or outside the slot: not loaded
            (
                "PANEL_BOX_NOT_ON_SHELF",
                {
                    "align": align(
                        ok=False, shelf="marker_not_found", on_shelf=False, align=None
                    )
                },
            ),
            (
                "PANEL_BOX_NOT_ON_SHELF",
                {
                    "align": align(
                        ok=False, shelf="read_off_shelf", on_shelf=False, align=None
                    )
                },
            ),
            ("ARM_NOT_RETURNED", {"align": align(far=["elbow_flex"], check="far")}),
            # not aligned in time and the arms did not come back to rest: failed, not "not aligned"
            (
                "ARM_NOT_RETURNED",
                {
                    "align": align(
                        ok=False, align="timeout", far=["elbow_flex"], check="far"
                    )
                },
            ),
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
                result = events[-1]["payload"]["mission_result"]
                if (
                    code == "PANEL_EMERGENCY_STOP"
                ):  # frozen in place until a person acts
                    self.assertEqual(result, "MISSION_HOLD")
                    self.assertIsNotNone(self.host.hold)
                else:  # a person's stop, or arms read back home: the next mission may start
                    self.assertEqual(result, "MISSION_FAILED")
                    self.assertIsNone(self.host.hold)

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

    def test_loaded_but_not_aligned_completes_without_hold(self):
        cases = [
            # armed turned off before the aligner started; the camera-only read saw the box
            ("PANEL_ALIGN_SKIPPED", align(ran=False, ok=False, rest=None)),
            # on the shelf, but not straightened within the aligner's time limit
            ("PANEL_ALIGN_FAILED", align(ok=False, align="timeout", taps=7)),
        ]
        for code, result in cases:
            with self.subTest(code=code):
                self.send(self.inbound())
                self.assertIs(self.panel.starts[-1]["end_on_return"], True)
                self.assertIs(self.panel.starts[-1]["shelf_align"], True)
                self.panel.finish(align=result, stage="done", final=True)
                events = self.host.poll()
                p = events[-1]["payload"]
                self.assertEqual(
                    (events[-1]["type"], p["phase"]),
                    ("shelf_inbound.completed", "COMPLETE"),
                )
                self.assertEqual(p["mission_result"], "MISSION_COMPLETE_NOT_ALIGNED")
                self.assertEqual(p["evidence"], "shelf_load_not_aligned")
                self.assertEqual(p["align_error"], code)
                self.assertIs(p["departure_ready"], True)
                self.assertIsNone(self.host.hold)

    def test_shelf_unknown_holds_for_a_person(self):
        # Nobody could tell whether the box is on the shelf: the mission cannot go on.
        returned = {"last_episode": {"episode": 0, "returned": True, "elapsed_s": 21.9}}
        unread = {
            "ok": False,
            "shelf": "marker_not_found",
            "on_shelf": None,
            "align": None,
        }
        cases = [
            (
                "PANEL_SHELF_UNREADABLE",
                {"align": align(**unread)},
            ),  # cameras unreadable
            # another job took the robot first: no alignment, no shelf read
            (
                "PANEL_ALIGN_SKIPPED",
                {"align": align(ran=False, ok=False, on_shelf=None, rest=None)},
            ),
        ]
        for code, change in cases:
            with self.subTest(code=code):
                self.host.clear_hold()
                self.send(self.inbound())
                self.panel.finish(stage="done", final=True, **change)
                (failed,) = self.host.poll()
                self.assertEqual(failed["payload"]["error_code"], code)
                self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
                self.assertIsNotNone(self.host.hold)
        # no aligner set up on this PC: the panel never looked at the shelf
        self.host.clear_hold()
        self.send(self.inbound())
        self.panel.finish(**returned)
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["error_code"], "PANEL_ALIGN_NOT_RUN")
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
        self.assertEqual(self.panel.home_returns, 0)

    def test_alignment_at_another_slot_fails(self):
        # The panel aligns at the slot of its own config; a 2b mission aligned at 2A failed.
        self.send(self.inbound())
        self.panel.finish(align=align(slot="2A"), stage="done", final=True)
        events = self.host.poll()
        self.assertEqual(events[-1]["type"], "shelf_inbound.failed")
        self.assertEqual(events[-1]["payload"]["error_code"], "PANEL_ALIGN_WRONG_SLOT")
        self.assertEqual(events[-1]["payload"]["mission_result"], "MISSION_FAILED")
        self.assertIsNone(self.host.hold)

    def outbound(self):
        message = self.inbound()
        message["type"] = "shelf_outbound.requested"
        message["payload"]["operation"] = "OUTBOUND"
        return message

    def use_unload_route(self):
        self.executor.routes = ROUTES | OUT_ROUTE
        self.panel.models.append(OUT_MODEL)

    def test_unload_route_needs_panel_3_20(self):
        # 3.20 takes the empty-tray photo right before the unloading (2026-11-21 review)
        self.panel.status["version"] = "3.14.5"
        self.assertTrue(self.status()["ready"])  # loading only: 3.14 is enough
        self.use_unload_route()
        for version in ("3.15.0", "3.19.9"):
            with self.subTest(version=version):
                self.panel.status["version"] = version
                payload = self.status()
                self.assertFalse(payload["ready"])
                self.assertIn("needs 3.20.0+", payload["executor_error"])
        self.panel.status["version"] = "3.20.0"
        self.assertTrue(self.status()["ready"])

    def test_unload_checks_pinky_and_completes_when_the_box_is_on_it(self):
        self.use_unload_route()
        (accepted,) = self.send(self.outbound())
        self.assertEqual(accepted["payload"]["phase"], "ACCEPTED")
        start = self.panel.starts[-1]
        self.assertEqual(start["model_path"], OUT_MODEL)
        self.assertEqual(start["task"], "OUTBOUND shelf 2B->pinky | marker 16")
        self.assertEqual((start["pinky_check"], start["shelf_align"]), (True, False))
        self.assertNotIn(
            "fast_release", start
        )  # no aligner after unloading: full safe stop
        # the box number is for the Pinky check only: no target circle on the policy's camera
        self.assertIs(start["draw_target"], False)
        # read at the commanded slot, with this Pinky's tray photo
        self.assertEqual((start["slot"], start["pinky_id"]), ("2B", "pinky1"))
        for change in (
            {},  # ended on its return
            {"exit_ok": False, "err": {"code": "err_code_7"}},  # no time or error rule
        ):
            with self.subTest(change=change):
                if change:
                    self.send(self.outbound())
                self.panel.finish(pinky=pinky(), stage="done", final=True, **change)
                events = self.host.poll()
                p = events[-1]["payload"]
                self.assertEqual(events[-1]["type"], "shelf_outbound.completed")
                self.assertEqual(p["mission_result"], "MISSION_COMPLETE")
                self.assertEqual(p["evidence"], "pinky_load")
                self.assertIs(p["departure_ready"], True)
                self.assertIsNone(self.host.hold)

    def test_unload_box_dropped_or_left_on_shelf_fails_and_arms_go_home(self):
        self.use_unload_route()
        cases = [
            ("PANEL_BOX_DROPPED", pinky(on_pinky=False, on_shelf=False)),
            ("PANEL_BOX_STILL_ON_SHELF", pinky(on_pinky=False, on_shelf=True)),
        ]
        for code, check in cases:
            with self.subTest(code=code):
                self.send(self.outbound())
                self.panel.finish(pinky=check, stage="done", final=True)
                (failed,) = self.host.poll()
                self.assertEqual(failed["type"], "shelf_outbound.failed")
                self.assertEqual(failed["payload"]["error_code"], code)
                self.assertEqual(failed["payload"]["mission_result"], "MISSION_FAILED")
                self.assertIsNone(self.host.hold)
        # an arm left away from home: sent home first, then reported without HOLD
        self.panel.arms["OMX 1"] = live("left", home=False)
        self.send(self.outbound())
        self.panel.finish(pinky=pinky(on_pinky=False), stage="done", final=True)
        (progress,) = self.host.poll()
        self.assertEqual(progress["payload"]["step"], "home_return")
        self.panel.status["job"] = {"running": False, "kind": "home"}
        self.panel.arms["OMX 1"] = live("left")
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_FAILED")
        self.assertEqual(self.panel.home_returns, 1)
        self.assertEqual(self.panel.home_return_bodies, [{"source": "mqtt"}])
        self.assertIsNone(self.host.hold)

    def test_unload_needs_the_slot_empty_and_the_box_on_the_tray(self):
        # 2026-11-17 rule: commanded slot empty AND box on the tray (box marker on the tray, or
        # the tray changed from its empty reference photo); no marker on the tray itself.
        self.use_unload_route()
        cases = [
            ("completed", None, {"slot_empty": True, "on_pinky": True}),
            (
                "failed",
                "PANEL_BOX_STILL_ON_SHELF",
                {"slot_empty": False, "on_pinky": False},
            ),
            ("failed", "PANEL_BOX_DROPPED", {"slot_empty": True, "on_pinky": False}),
            (
                "held",
                "PANEL_PINKY_CHECK_CONFLICT",
                {"slot_empty": False, "on_pinky": True},
            ),
            ("held", "PANEL_SLOT_UNREADABLE", {"slot_empty": None, "on_pinky": True}),
            # slot_empty wins over the shelf-wide on_shelf when the picker reports both
            (
                "completed",
                None,
                {"slot_empty": True, "on_shelf": True, "on_pinky": True},
            ),
        ]
        for outcome, code, values in cases:
            with self.subTest(values=values):
                self.host.clear_hold()
                self.send(self.outbound())
                check = pinky(tray_changed=0.7, box_on_tray=True, **values)
                self.panel.finish(pinky=check, stage="done", final=True)
                (event,) = self.host.poll()
                p = event["payload"]
                if outcome == "completed":
                    self.assertEqual(p["mission_result"], "MISSION_COMPLETE")
                    self.assertEqual(p["evidence"], "pinky_load")
                else:
                    self.assertEqual(p["error_code"], code)
                    held = outcome == "held"
                    self.assertEqual(
                        p["mission_result"],
                        "MISSION_HOLD" if held else "MISSION_FAILED",
                    )
                    self.assertIs(self.host.hold is not None, held)

    def test_unload_refused_by_the_tray_check_fails_without_hold(self):
        # panel 3.20 checks the Pinky tray right before unloading and refuses a non-empty one
        self.use_unload_route()
        cases = [
            ("not_empty", "PANEL_TRAY_NOT_EMPTY", {"not_empty_why": "box_marker"}),
            ("no_frame", "PANEL_TRAY_CHECK_FAILED", {}),
        ]
        for state, code, extra in cases:
            with self.subTest(state=state):
                self.panel.start_refusal = {
                    "error": "하차 시작 전 쟁반 확인: 평가를 시작하지 않았습니다.",
                    "refused": "tray_check",
                    "tray_check": {"tray_check": state, **extra},
                }
                accepted, failed = self.send(self.outbound())
                p = failed["payload"]
                self.assertEqual(failed["type"], "shelf_outbound.failed")
                self.assertEqual(
                    (p["error_code"], p["mission_result"]), (code, "MISSION_FAILED")
                )
                self.assertIsNone(self.host.hold)
        self.assertEqual(self.panel.starts, [])
        self.assertEqual(self.panel.home_returns, 0)  # nothing moved, nothing sent home

    def test_unload_tray_judged_only_against_this_missions_photo(self):
        self.use_unload_route()
        cases = [
            {"tray_reference_state": "reference_stale"},
            {"tray_reference_state": "reference_mismatch"},
            {"tray_reference_state": "ok", "pre_mission_match": False},
        ]
        for change in cases:
            with self.subTest(change=change):
                self.host.clear_hold()
                self.send(self.outbound())
                check = pinky(slot_empty=True, **change)
                self.panel.finish(pinky=check, stage="done", final=True)
                (event,) = self.host.poll()
                p = event["payload"]
                self.assertEqual(
                    (p["error_code"], p["mission_result"]),
                    ("PANEL_PINKY_NO_REFERENCE", "MISSION_HOLD"),
                )
        self.host.clear_hold()
        self.send(self.outbound())
        good = {"tray_reference_state": "ok", "pre_mission_match": True}
        self.panel.finish(
            pinky=pinky(slot_empty=True, **good), stage="done", final=True
        )
        (event,) = self.host.poll()
        self.assertEqual(event["payload"]["mission_result"], "MISSION_COMPLETE")

    def test_unload_without_this_missions_tray_photo_holds(self):
        # 2026-11-21 review repro: no box marker, tray_changed 0.3 → on_pinky true, and the
        # pre-mission photo fields left out → the Host sent COMPLETE and departure_ready
        self.use_unload_route()
        missing = [
            ("tray_reference_state", "pre_mission_match", "tray_reference"),
            ("pre_mission_match",),
            ("tray_reference_state",),
        ]
        for keys in missing:
            with self.subTest(missing=keys):
                self.host.clear_hold()
                self.send(self.outbound())
                check = pinky(tray_changed=0.3, box_on_tray=False)
                for key in keys:
                    del check[key]
                self.panel.finish(pinky=check, stage="done", final=True)
                (event,) = self.host.poll()
                p = event["payload"]
                self.assertEqual(event["type"], "shelf_outbound.failed")
                self.assertEqual(
                    (p["error_code"], p["mission_result"]),
                    ("PANEL_PINKY_NO_REFERENCE", "MISSION_HOLD"),
                )
                self.assertIsNot(p.get("departure_ready"), True)
                self.assertIsNotNone(self.host.hold)

    def test_unload_needs_the_commanded_slots_own_reading(self):
        # 2026-11-21 review: without slot_empty the shelf-wide on_shelf=false completed it
        self.use_unload_route()
        no_slot_reading = pinky()
        del no_slot_reading["slot_empty"]
        cases = [
            no_slot_reading,  # on_shelf false only
            pinky(slot="2C"),  # read at another slot
            pinky(slot=None),
        ]
        for check in cases:
            with self.subTest(check={k: check.get(k) for k in ("slot", "slot_empty")}):
                self.host.clear_hold()
                self.send(self.outbound())
                self.panel.finish(pinky=check, stage="done", final=True)
                (event,) = self.host.poll()
                p = event["payload"]
                self.assertEqual(
                    (p["error_code"], p["mission_result"]),
                    ("PANEL_SLOT_UNREADABLE", "MISSION_HOLD"),
                )
                self.assertIsNot(p.get("departure_ready"), True)

    def test_unload_unknown_pinky_holds_for_a_person(self):
        self.use_unload_route()
        unread = pinky(on_pinky=None, on_shelf=None, ran=False, skipped="시간 초과")
        tray_unread = pinky(on_pinky=None, slot_empty=True, pinky="no_reference")
        cases = [
            ("PANEL_SLOT_UNREADABLE", {"pinky": unread}),
            ("PANEL_PINKY_UNREADABLE", {"pinky": tray_unread}),
            (
                "PANEL_PINKY_CHECK_NOT_RUN",
                {},
            ),  # panel before pinky_check: no key at all
            (
                "PANEL_PINKY_CHECK_CONFLICT",
                {"pinky": pinky(on_pinky=True, on_shelf=True)},
            ),
            (
                "PANEL_EMERGENCY_STOP",
                {"exit_ok": False, "stopped_by": "비상 정지", "pinky": unread},
            ),
        ]
        for code, change in cases:
            with self.subTest(code=code):
                self.host.clear_hold()
                self.send(self.outbound())
                self.panel.finish(stage="done", final=True, **change)
                (failed,) = self.host.poll()
                self.assertEqual(failed["payload"]["error_code"], code)
                self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
                self.assertIsNotNone(self.host.hold)
        self.assertEqual(self.panel.home_returns, 0)

    def test_failed_mission_never_moves_arms_while_armed_is_off(self):
        # The panel's home move does not check armed; the Host does.
        self.panel.arms["OMX 2"] = live("right", home=False)
        self.send(self.inbound())
        self.panel.status["armed"] = False  # e.g. the panel turned it off on an anomaly
        self.panel.finish(align=align(on_shelf=False, ok=False), final=True)
        (failed,) = self.host.poll()
        self.assertEqual(failed["payload"]["error_code"], "PANEL_NOT_ARMED_FOR_HOME")
        self.assertIn("PANEL_BOX_NOT_ON_SHELF", failed["payload"]["detail"])
        self.assertEqual(failed["payload"]["mission_result"], "MISSION_HOLD")
        self.assertEqual(self.panel.home_returns, 0)
        self.assertIsNotNone(self.host.hold)

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

    def test_panel_older_than_3_14_is_not_ready(self):
        # 3.12 reached the AI server over HTTP (SSH tunnel); 3.13 ignores "shelf_align" in a start
        for version in ("3.13.0", "3.12.0", "3.11.0", "3.9.1", None, "dev"):
            with self.subTest(version):
                self.panel.status["version"] = version
                payload = self.status()
                self.assertFalse(payload["ready"])
                self.assertIn("PANEL_VERSION", payload["executor_error"])
        for version in ("3.14.0", "3.14.5"):
            self.panel.status["version"] = version
            self.assertTrue(self.status()["ready"])

    def test_ai_link_must_be_udp_checked_and_fresh(self):
        infer = self.panel.status["gpu_link"]["infer"]
        cases = [
            (
                "AI_LINK_DOWN: network timeout",
                {"ok": False, "stage": "network", "error": "timeout"},
            ),
            ("AI_LINK_DOWN: starting", {"ok": None, "stage": ""}),
            ("AI_LINK_STALE", {"ok": True}, 25.0),
            ("AI_LINK_STALE: no check time", {"ok": True, "at": None}, None),
        ]
        for code, change, *age in cases:
            with self.subTest(code):
                self.panel.udp_age = age[0] if age else 1.0
                infer["udp"] = {
                    "ok": True,
                    "stage": "ok",
                    "error": "",
                    "at": None,
                } | change
                payload = self.status()
                self.assertFalse(payload["ready"])
                self.assertTrue(payload["executor_error"].startswith(code))
                (rejected,) = self.send(self.inbound())
                self.assertEqual(rejected["payload"]["error_code"], "EXECUTOR_ERROR")
                self.assertEqual(self.panel.starts, [])
                self.assertIsNone(self.host.hold)  # refused before acceptance: no HOLD
        self.panel.udp_age = 1.0
        infer["udp"] = {"ok": True, "stage": "ok", "error": "", "at": None}
        self.assertTrue(self.status()["ready"])
        infer["transport"] = "http"
        self.assertIn("AI_LINK_NOT_UDP", self.status()["executor_error"])
        self.panel.status["gpu_link"] = None  # GPU-role panel: no remote link reported
        self.assertIn("AI_LINK_UNKNOWN", self.status()["executor_error"])

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

    def test_route_without_episode_s_has_no_time_limit(self):
        # TK 2026-11-22: no mission time limit by default; the panel's longest episode is 600 s
        path = Path(self.dir.name) / "routes.json"
        route = {
            "operation": "INBOUND",
            "slot_id": "2b",
            "model_path": MODEL,
            "task": "t {box}",
            "reset_s": 15,
        }
        key = ("INBOUND", "2b", "COOPERATIVE")
        for change, expected in (
            ({}, 600.0),
            ({"episode_s": None}, 600.0),
            ({"episode_s": 40}, 40.0),
        ):
            with self.subTest(change=change):
                path.write_text(json.dumps({"routes": [route | change]}))
                self.assertEqual(load_routes(path)[key].episode_s, expected)
        for bad in (2, 601, "long"):
            with self.subTest(bad=bad):
                path.write_text(json.dumps({"routes": [route | {"episode_s": bad}]}))
                with self.assertRaises(ValueError):
                    load_routes(path)
        path.write_text(json.dumps({"routes": [route]}))
        self.executor.routes = load_routes(path)
        self.send(self.inbound())
        self.assertEqual(self.panel.starts[-1]["episode_s"], 600.0)

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
        key = ("INBOUND", "2b", "COOPERATIVE")
        self.assertEqual(load_routes(path)[key].model_path, MODEL)
        self.assertIsNone(load_routes(path)[key].markers)
        path.write_text(json.dumps({"routes": [route | {"markers": [16, 17, 20]}]}))
        self.assertEqual(load_routes(path)[key].markers, frozenset({16, 17, 20}))
        one_arm = route | {"slot_id": "2a"}
        path.write_text(json.dumps({"routes": [one_arm]}))
        self.assertIn(("INBOUND", "2a", "DEDICATED"), load_routes(path))
        for bad in (
            {"slot_id": "3a"},
            {"operation": "LOAD"},
            {"model_path": ""},
            {"manipulation_mode": "DEDICATED"},  # 2B is worked by both arms
            {"markers": [18]},  # the Pinky tray marker, not a box
            {"markers": []},
            {"markers": ["16"]},
        ):
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
        self.assertEqual(load_routes(path)[key].output_filter, "중")
        example = ROOT / "config/cooperative_routes.example.json"
        self.assertEqual(list(load_routes(example)), [key])

    # Loading in two reports (2026-11-17): COMPLETE when the box is in its slot, then the alignment.
    def use_two_reports(self, version="3.20.0"):
        self.panel.status["version"] = version
        self.executor.report_placement = True

    def test_panel_3_17_gets_the_commanded_slot_and_pinky(self):
        self.panel.status["version"] = "3.17.2"
        self.send(self.inbound())
        start = self.panel.starts[-1]
        self.assertEqual((start["slot"], start["pinky_id"]), ("2B", "pinky1"))
        self.assertNotIn("end_on_place", start)

    def test_place_shadow_needs_panel_3_18_and_yields_to_end_on_place(self):
        self.executor.place_shadow = True
        cases = [("3.17.3", False, {}), ("3.18.0", False, {"place_shadow": True})]
        cases.append(("3.18.0", True, {"end_on_place": True}))
        for version, hand_over, want in cases:
            with self.subTest(version=version, hand_over=hand_over):
                self.host.clear_hold()
                self.panel.status["version"] = version
                self.panel.status["job"] = {"running": False}
                self.host._data["active"] = None
                self.executor.end_on_place = hand_over
                self.send(self.inbound())
                start = self.panel.starts[-1]
                got = {
                    k: start[k] for k in ("place_shadow", "end_on_place") if k in start
                }
                self.assertEqual(got, want)

    def test_end_on_place_needs_the_option_and_panel_3_18(self):
        self.executor.end_on_place = True
        for version, sent in (("3.17.2", False), ("3.18.0", True)):
            with self.subTest(version=version):
                self.host.clear_hold()
                self.panel.status["version"] = version
                self.panel.status["job"] = {"running": False}
                self.host._data["active"] = None
                self.send(self.inbound())
                self.assertIs("end_on_place" in self.panel.starts[-1], sent)

    def test_two_reports_complete_on_placement_then_aligned(self):
        self.use_two_reports()
        self.send(self.inbound())
        self.host.poll()  # loading the model
        self.panel.place()
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(
            (done["type"], p["phase"]), ("shelf_inbound.completed", "COMPLETE")
        )
        self.assertEqual(
            (p["evidence"], p["mission_result"], p["align_state"]),
            ("shelf_placed", "MISSION_COMPLETE", "PENDING"),
        )
        self.assertIs(p["departure_ready"], True)
        self.assertIs(p["placement_confirmed"], True)
        self.assertEqual(p["placement"]["slot"], "2B")
        # still aligning: the next mission waits
        (busy,) = self.send(self.inbound())
        self.assertEqual(busy["payload"]["error_code"], "HOST_BUSY")
        (step,) = self.host.poll()
        self.assertEqual(
            (step["type"], step["payload"]["step"]),
            ("shelf_inbound.progress", "정렬 중"),
        )
        self.panel.finish(align=align(), stage="done", final=True)
        (aligned,) = self.host.poll()
        a = aligned["payload"]
        self.assertEqual(
            (aligned["type"], a["phase"], a["align_result"], a["mission_result"]),
            ("shelf_inbound.progress", "ALIGNED", "ALIGNED", "MISSION_COMPLETE"),
        )
        self.assertGreater(a["phase_revision"], p["phase_revision"])
        self.assertIsNone(self.host.hold)
        self.assertEqual(self.host.poll(), [])
        (accepted,) = self.send(self.inbound())
        self.assertEqual(accepted["payload"]["phase"], "ACCEPTED")

    def test_two_reports_not_aligned_keeps_the_complete(self):
        self.use_two_reports()
        self.send(self.inbound())
        self.panel.place()
        (done,) = self.host.poll()
        self.assertEqual(done["payload"]["evidence"], "shelf_placed")
        self.panel.finish(
            align=align(ok=False, align="timeout"), stage="done", final=True
        )
        (late,) = self.host.poll()
        a = late["payload"]
        self.assertEqual(
            (a["phase"], a["mission_result"], a["align_error"]),
            ("NOT_ALIGNED", "MISSION_COMPLETE_NOT_ALIGNED", "PANEL_ALIGN_FAILED"),
        )
        self.assertIsNone(self.host.hold)

    def test_two_reports_emergency_stop_while_aligning_holds(self):
        self.use_two_reports()
        self.send(self.inbound())
        self.panel.place()
        self.host.poll()
        self.panel.finish(
            align=align(ok=False, stopped_by="비상 정지", released=False),
            stage="hold",
            final=True,
        )
        (late,) = self.host.poll()
        a = late["payload"]
        self.assertEqual(
            (late["type"], a["phase"], a["error_code"]),
            ("shelf_inbound.progress", "ALIGN_HOLD", "PANEL_EMERGENCY_STOP"),
        )
        self.assertIsNotNone(self.host.hold)

    def test_placement_is_not_reported_without_the_option_or_elsewhere(self):
        cases = {
            "option off": {"option": False},
            "not on the shelf": {"on_shelf": False},
            "unknown": {"on_shelf": None},
            "another slot": {"slot": "2A"},
        }
        for name, case in cases.items():
            with self.subTest(case=name):
                self.host.clear_hold()
                self.panel.status["job"] = {"running": False}
                self.host._data["active"] = None
                self.use_two_reports()
                self.executor.report_placement = case.get("option", True)
                self.send(self.inbound())
                self.panel.place(
                    on_shelf=case.get("on_shelf", True), slot=case.get("slot", "2B")
                )
                events = self.host.poll()
                self.assertFalse(
                    [e for e in events if e["type"].endswith(".completed")]
                )
                self.panel.finish(align=align(), stage="done", final=True)
                (done,) = self.host.poll()
                self.assertEqual(done["payload"]["evidence"], "shelf_align")

    def test_handover_not_taken_but_arms_home_is_complete_not_aligned(self):
        # panel 3.18: the aligner never took the arms; the panel brought them home itself
        placement = {"on_shelf": True, "slot": "2B", "box": 16, "source": "eval_judge"}
        skipped = {
            "ran": False,
            "ok": False,
            "box": 16,
            "slot": "2B",
            "handover": True,
            "skipped": "넘겨받지 못함 (failed: 8 s)",
        }
        cases = [
            ("failed", True, True, "completed", None),
            ("stopped", True, True, "completed", None),
            ("failed", False, True, "failed", "MISSION_HOLD"),
            ("failed", True, False, "failed", "MISSION_HOLD"),
        ]
        for state, fallback_ok, on_shelf, kind, result in cases:
            with self.subTest(state=state, fallback_ok=fallback_ok, on_shelf=on_shelf):
                self.host.clear_hold()
                self.panel.status["job"] = {"running": False}
                self.host._data["active"] = None
                self.send(self.inbound())
                self.panel.finish(
                    align=skipped,
                    placement=placement | {"on_shelf": on_shelf},
                    handover={
                        "mode": "handover",
                        "state": state,
                        "fallback": "home",
                        "fallback_ok": fallback_ok,
                    },
                    stage="done",
                    final=True,
                )
                (event,) = self.host.poll()
                p = event["payload"]
                code = "PANEL_HANDOVER_" + state.upper()
                self.assertTrue(event["type"].endswith("." + kind))
                if kind == "completed":
                    self.assertEqual(
                        (p["mission_result"], p["align_error"]),
                        ("MISSION_COMPLETE_NOT_ALIGNED", code),
                    )
                else:
                    self.assertEqual(
                        (p["mission_result"], p["error_code"]), (result, code)
                    )

    def test_shelf_unreadable_but_judge_saw_it_placed_is_loaded_not_aligned(self):
        # TK 2026-11-22 (field 16:09): hand-over taken, the aligner then found front_2 moved
        # (camera_moved, on_shelf null) and stopped; the placement judge had seen box 16 in 2B with
        # the hands clear and the arms ended home → loaded, not aligned, the Pinky may leave
        unread = align(
            ok=False, on_shelf=None, shelf="camera_moved", align="camera_moved"
        )
        judge = {
            "on_shelf": True,
            "slot": "2B",
            "box": 16,
            "source": "eval_judge",
            "tips_clear": True,
        }
        moved_on = {
            "on_shelf": None,
            "slot": "2B",
            "box": 16,
            "source": "align",
            "first": {"source": "eval_judge", "on_shelf": True},
        }
        placed = {"mode": "handover", "state": "taken", "end_reason": "placed"}
        cases = [
            ("judge on top", judge, placed, "completed"),
            ("judge kept under first", moved_on, placed, "completed"),
            ("hands not clear", judge | {"tips_clear": False}, placed, "held"),
            ("another slot", judge | {"slot": "2C"}, placed, "held"),
            (
                "no judge (no hand-over)",
                {"on_shelf": None, "slot": "2B", "source": "align"},
                None,
                "held",
            ),
            (
                "first without placed end",
                moved_on,
                placed | {"end_reason": "returned"},
                "held",
            ),
        ]
        for name, placement, handover, outcome in cases:
            with self.subTest(case=name):
                self.host.clear_hold()
                self.panel.status["job"] = {"running": False}
                self.host._data["active"] = None
                self.send(self.inbound())
                extra = {"handover": handover} if handover else {}
                self.panel.finish(
                    align=unread, placement=placement, stage="done", final=True, **extra
                )
                (event,) = self.host.poll()
                p = event["payload"]
                if outcome == "completed":
                    self.assertEqual(event["type"], "shelf_inbound.completed")
                    self.assertEqual(
                        (p["mission_result"], p["evidence"], p["align_error"]),
                        (
                            "MISSION_COMPLETE_NOT_ALIGNED",
                            "shelf_load_not_aligned",
                            "PANEL_SHELF_UNREADABLE",
                        ),
                    )
                    self.assertIs(p["departure_ready"], True)
                    self.assertIsNone(self.host.hold)
                else:
                    self.assertEqual(
                        (p["mission_result"], p["error_code"]),
                        ("MISSION_HOLD", "PANEL_SHELF_UNREADABLE"),
                    )
                    self.assertIsNot(p.get("departure_ready"), True)
        # the aligner saw the shelf without the box: still not loaded, whatever the judge said
        self.host.clear_hold()
        self.panel.status["job"] = {"running": False}
        self.host._data["active"] = None
        self.send(self.inbound())
        self.panel.finish(
            align=align(ok=False, on_shelf=False),
            placement=judge,
            handover=placed,
            stage="done",
            final=True,
        )
        events = self.host.poll()
        self.assertEqual(
            events[-1]["payload"].get("error_code"), "PANEL_BOX_NOT_ON_SHELF"
        )
        self.assertNotEqual(events[-1]["type"], "shelf_inbound.completed")

    def test_two_reports_handover_not_taken_reports_not_aligned(self):
        self.use_two_reports("3.20.0")
        self.executor.end_on_place = True
        self.send(self.inbound())
        self.assertIs(self.panel.starts[-1]["end_on_place"], True)
        self.panel.place()
        self.panel.status["last_result"]["placement"] |= {
            "source": "eval_judge",
            "tips_clear": True,
            "tip_y": {"left": 0.12, "right": 0.13},
            "y_out": 0.185,
        }
        (done,) = self.host.poll()
        self.assertEqual(done["payload"]["evidence"], "shelf_placed")
        self.panel.finish(
            align={"ran": False, "ok": False, "handover": True, "slot": "2B"},
            placement={"on_shelf": True, "slot": "2B", "source": "eval_judge"},
            handover={"state": "failed", "fallback": "home", "fallback_ok": True},
            stage="done",
            final=True,
        )
        (late,) = self.host.poll()
        a = late["payload"]
        self.assertEqual(
            (a["phase"], a["align_error"]), ("NOT_ALIGNED", "PANEL_HANDOVER_FAILED")
        )
        self.assertIsNone(self.host.hold)

    def test_handover_aligner_hold_holds_and_never_sends_the_arms_home(self):
        # 2026-11-21: after taking the arms over, the aligner found no safe way home and left
        # them by the shelf with the torque on (handover_align "hold", exit 2). Panel 3.21.0
        # closes that as an ordinary alignment (stage done), so the Host must not recover it.
        self.executor.end_on_place = True
        self.panel.status["version"] = "3.21.0"
        for on_shelf in (False, True, None):
            for stage in ("done", "hold"):  # "hold": the panel fix agreed the same day
                with self.subTest(on_shelf=on_shelf, stage=stage):
                    self.host.clear_hold()
                    self.panel.status["job"] = {"running": False}
                    self.host._data["active"] = None
                    homes = self.panel.home_returns
                    self.send(self.inbound())
                    self.panel.finish(
                        exit_ok=True,
                        align=align(
                            ok=False,
                            align="plan_error",
                            on_shelf=on_shelf,
                            released=False,
                            returncode=2,
                            handover_align="hold",
                        ),
                        handover={"mode": "handover", "state": "taken"},
                        stage=stage,
                        final=stage == "done",
                    )
                    (event,) = self.host.poll()
                    p = event["payload"]
                    self.assertEqual(event["type"], "shelf_inbound.failed")
                    self.assertEqual(
                        (p["error_code"], p["mission_result"]),
                        ("PANEL_HANDOVER_HOLD", "MISSION_HOLD"),
                    )
                    self.assertIsNotNone(self.host.hold)
                    self.assertEqual(self.panel.home_returns, homes)

    def test_handover_aligned_judges_grippers_after_the_alignment(self):
        # 2026-11-21 14:59 field: hand-over taken, aligned, rest ok; the eval snapshot is the
        # moment of release with both grippers opened wide (59.98, 60.61 > 54.6). The Host
        # judged "still grasping" from it and reported ARM_NOT_RETURNED.
        self.executor.end_on_place = True
        self.panel.status["version"] = "3.21.1"
        self.send(self.inbound())
        wide = {"OMX 1": ("left", 59.98), "OMX 2": ("right", 60.61)}
        after = {"OMX 1": ("left", 50.55), "OMX 2": ("right", 48.84)}
        rest_arms = {
            name: {
                "side": side,
                "joints": {"shoulder_pan": 0.0, "gripper": grip},
                "gripper": grip,
                "gripper_state": "closed",
                "far_joints": [],
            }
            for name, (side, grip) in after.items()
        }
        self.panel.finish(
            arms={
                name: arm(side) | {"gripper": grip, "at_home": False}
                for name, (side, grip) in wide.items()
            },
            align=align(handover_align="home_first", rest_ref="home")
            | {"rest": {"check": "ok", "tolerance": 5.0, "arms": rest_arms}},
            handover={"mode": "handover", "end_reason": "placed", "state": "taken"},
            stage="done",
            final=True,
        )
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(
            (done["type"], p["mission_result"], p["evidence"]),
            ("shelf_inbound.completed", "MISSION_COMPLETE", "shelf_align"),
        )
        self.assertEqual(self.panel.home_returns, 0)
        self.assertIsNone(self.host.hold)

    def test_early_complete_needs_released_hands_and_a_fresh_capture(self):
        # control agreement 2026-11-18: slot placement + hands really off the box + capture <= 3 s
        held = {
            "all_at_home": True,
            "arms": {"left": {"gripper": 50.0}, "right": {"gripper": 59.6}},
        }
        cases = {
            "panel before 3.20": ({}, "3.19.0"),
            "a gripper still closed on the box": ({"eval_end": held}, "3.20.0"),
            "arms not home": ({"eval_end": {"all_at_home": False}}, "3.20.0"),
            "no arm reading": ({"eval_end": None}, "3.20.0"),
            "hand-over, fingertips in": (
                {"source": "eval_judge", "tips_clear": False},
                "3.20.0",
            ),
            "capture 4 s old": ({"observed_ts": time.time() - 4.0}, "3.20.0"),
            "no capture time": ({"observed_ts": None}, "3.20.0"),
            # 2026-11-21 review: the control step's time is not a capture time
            "control step time": ({"observed_clock": "control_step"}, "3.20.0"),
            "clock not said": ({"observed_clock": None}, "3.20.0"),
        }
        for name, (evidence, version) in cases.items():
            with self.subTest(case=name):
                self.host.clear_hold()
                self.panel.status["job"] = {"running": False}
                self.host._data["active"] = None
                self.use_two_reports(version)
                self.send(self.inbound())
                self.panel.place(**evidence)
                events = self.host.poll()
                self.assertFalse(
                    [e for e in events if e["type"].endswith(".completed")]
                )
                self.panel.finish(align=align(), stage="done", final=True)
                (done,) = self.host.poll()  # one report after the alignment, as before
                self.assertEqual(done["payload"]["evidence"], "shelf_align")

    def test_early_complete_says_the_grasp_is_released(self):
        self.use_two_reports()
        self.send(self.inbound())
        self.panel.place()
        (done,) = self.host.poll()
        p = done["payload"]
        self.assertEqual(p["evidence"], "shelf_placed")
        self.assertIs(p["grasp_released"], True)
        self.assertIs(p["placement"]["eval_end"]["all_at_home"], True)
        self.assertIsInstance(p["placement"]["observed_ts"], float)
        self.assertEqual(p["placement"]["observed_clock"], "capture")

    def test_outbound_never_reports_a_placement(self):
        self.use_unload_route()
        self.use_two_reports()
        self.send(self.outbound())
        self.panel.place()
        events = self.host.poll()
        self.assertFalse([e for e in events if e["type"].endswith(".completed")])


if __name__ == "__main__":
    unittest.main()
