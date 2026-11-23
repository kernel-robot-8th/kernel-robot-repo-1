"""shelf_direct: LocalPanel(Host 쪽 Panel API 흉내) + 현장 PanelExecutor. 가짜 worker 프로세스, 장치 없음."""

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.cooperative import Rejected
from roport_omx.panel_executor import PanelExecutor, PanelRefused, Route
from roport_omx.shelf_direct.contract import STATE_KEYS
from roport_omx.shelf_direct.devices import RobotLock
from roport_omx.shelf_direct.local_panel import DirectConfig, LocalPanel

WORKER = ROOT / "ci" / "fake_shelf_worker.py"
MODEL = "/models/real_act_in"
HOME = {"joints": {k.removesuffix(".pos"): 0.0 for k in STATE_KEYS}, "std": {}}
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
AT_HOME = {
    s: {**dict.fromkeys(JOINTS, 0.0), "gripper": 40.0} for s in ("left", "right")
}
ARMS_DONE = {
    name: {"side": side, "joints": AT_HOME[side], "gripper": 40.0, "at_home": True}
    for side, name in (("left", "OMX 1"), ("right", "OMX 2"))
}
ROUTES = {
    ("INBOUND", "2b", "COOPERATIVE"): Route(MODEL, "INBOUND 2B | marker {box}", 40, 15)
}
TASK = {
    "command_id": "cmd-1",
    "operation": "INBOUND",
    "slot_id": "2b",
    "manipulation_mode": "COOPERATIVE",
    "package_id": "16",
    "pinky_id": "pinky2",
    "expires_at": "2999-01-01T00:00:00Z",
}


class Lock(RobotLock):
    def acquire(self, owner):  # 시험 PC의 다른 로봇 프로그램은 보지 않는다
        import fcntl

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fd = os.open(self.path, os.O_RDWR | os.O_CREAT)
        fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)


class Panel(LocalPanel):
    def _worker_args(self, cmd, req_path):
        return [sys.executable, str(WORKER), cmd, str(req_path)]

    def _run_worker(self, cmd, req, timeout=15.0):
        self.reads = getattr(self, "reads", 0) + 1
        return self.arms_now


class LocalPanelTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        d = Path(self.tmp.name)
        self.env = {"FAKE_WORKER_HOME_LOG": str(d / "home.json")}
        os.environ.update(self.env)
        cfg = DirectConfig(state_dir=str(d / "state"), lock_path=str(d / "robot.lock"))
        self.panel = Panel(cfg, [MODEL], model_info=lambda m: {"home": HOME}, link_check=lambda: None,
                           lock=Lock(d / "robot.lock"))  # fmt: skip
        self.panel.arms_now = AT_HOME
        self.addCleanup(self.panel.close)
        self.ex = PanelExecutor(ROUTES, end_on_place=True)
        self.ex._call = lambda method, path, body=None, timeout=None: (
            self.panel.request(method, path, body)
        )
        time.sleep(0.05)  # 링크 점검 1회

    def mode(self, mode, result=None):
        os.environ["FAKE_WORKER_MODE"] = mode
        os.environ["FAKE_WORKER_RESULT"] = json.dumps(result or {})

    def wait_done(self, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            obs = self.ex.observe()
            if obs["state"] != "running":
                return obs
            time.sleep(0.05)
        self.fail("still running")

    def test_readiness_needs_armed_and_reports_arms_read(self):
        r = self.ex.readiness()
        self.assertEqual((r["armed"], r["busy"], r["error"]), (False, False, None))
        self.assertTrue(
            all(a["returned"] and a["grasping"] is False for a in r["arms"].values())
        )
        self.assertEqual(sorted(r["arms_read"]["joints"]), ["omx1", "omx2"])
        self.ex.check(TASK)  # 모델을 GPU 서버가 안다
        self.mode("ok")
        with self.assertRaises(
            PanelRefused
        ):  # armed 꺼짐: 시작 거절, 팔은 움직이지 않음
            self.ex.start(TASK)

    def test_unknown_model_is_not_ready(self):
        self.panel.model_info = lambda m: (_ for _ in ()).throw(
            RuntimeError("MODEL_NOT_ALLOWED")
        )
        self.panel.homes.clear()
        with self.assertRaises(Rejected):
            self.ex.check(TASK)

    def test_mission_runs_and_is_judged_like_the_panel(self):
        self.panel.set_armed(True)
        self.ex.readiness()
        self.mode("ok", {"exit_ok": True, "stage": "done", "final": True, "arms": ARMS_DONE,
                         "align": {"ran": True, "ok": True, "slot": "2B", "on_shelf": True, "released": True,
                                   "rest": {"check": "ok", "arms": {
                                       n: {"side": a["side"], "gripper": 40.0, "far_joints": []}
                                       for n, a in ARMS_DONE.items()}}}})  # fmt: skip
        self.ex.start(TASK)
        body = json.loads(
            next(Path(self.panel.c.dir()).glob("req_*.json")).read_text()
        )["body"]
        self.assertEqual(
            (body["end_on_place"], body["shelf_align"], body["target_marker"]),
            (True, True, 16),
        )
        self.assertEqual(self.ex.readiness()["busy"], True)
        obs = self.wait_done()
        self.assertEqual(
            (obs["state"], obs.get("evidence")), ("finished", "shelf_align")
        )
        self.assertTrue(self.panel.armed["on"])
        self.assertFalse(self.panel.lock.held)

    def test_stop_turns_armed_off_and_lands(self):
        self.panel.set_armed(True)
        self.ex.readiness()
        self.mode("wait_stop", {"exit_ok": True})
        self.ex.start(TASK)
        time.sleep(0.3)
        self.ex.stop()
        self.assertFalse(self.panel.armed["on"])
        obs = self.wait_done()
        self.assertEqual(
            (obs["state"], obs["error"], obs["operator_stop"]),
            ("failed", "PANEL_STOPPED", True),
        )

    def test_tray_refusal_is_a_start_refusal_without_hold(self):
        self.panel.set_armed(True)
        self.mode("refused")
        with self.assertRaises(PanelRefused) as ctx:
            self.panel.eval_start({"source": "mqtt", "model_path": MODEL})
        self.assertEqual(ctx.exception.reply["refused"], "tray_check")
        self.assertFalse(self.panel.lock.held)

    def test_crashed_worker_is_an_error_and_disarms_then_home_return(self):
        self.panel.set_armed(True)
        self.ex.readiness()
        self.mode("crash")
        self.ex.start(TASK)
        self.panel.arms_now = {
            s: dict(a, shoulder_pan=40.0) for s, a in AT_HOME.items()
        }
        obs = self.wait_done()
        # 결과를 남기지 못하고 끝난 worker: 오류, armed 꺼짐 → 집으로 보내지 않음(사람 확인)
        self.assertEqual(
            (obs["state"], obs["error"]), ("failed", "PANEL_NOT_ARMED_FOR_HOME")
        )
        self.assertFalse(self.panel.armed["on"])
        self.panel.set_armed(True)
        self.panel.request("POST", "/api/home/return", {"source": "mqtt"})
        self.assertTrue(self.panel.status()["job"]["running"])
        end = time.monotonic() + 5
        while self.panel.status()["job"]["running"] and time.monotonic() < end:
            time.sleep(0.05)
        self.assertEqual(json.loads(Path(self.env["FAKE_WORKER_HOME_LOG"]).read_text())["left"]["port"],
                         "/dev/omx_follower_2")  # fmt: skip

    def test_home_return_refused_on_hold_or_disarmed(self):
        self.panel.last_status = {"last_result": {"stage": "hold"}}
        self.panel.set_armed(True)
        with self.assertRaises(PanelRefused):
            self.panel.home_return({"source": "mqtt"})
        self.panel.set_armed(False)
        self.panel.last_status = {}
        with self.assertRaises(PanelRefused):
            self.panel.home_return({"source": "mqtt"})

    def test_arm_read_is_cached_and_busy_while_running(self):
        self.panel.arms(MODEL)
        self.panel.arms(MODEL)
        self.assertEqual(self.panel.reads, 1)
        self.panel.set_armed(True)
        self.mode("wait_stop")
        self.panel.eval_start({"source": "mqtt", "model_path": MODEL})
        self.assertEqual(self.panel.arms(MODEL)["source"], "busy")
        self.panel.stop()

    def arm_request(self, armed, age=0.0):
        req_path, state_path = self.panel._arm_paths()
        req_path.write_text(json.dumps({"request_id": f"r{armed}{age}", "armed": armed, "by": "tk",
                                        "requested_at": time.time() - age}))  # fmt: skip
        self.panel.status()
        return json.loads(state_path.read_text())

    def test_local_arm_command(self):
        st = self.arm_request(True)
        self.assertEqual((st["on"], st["request_id"]), (True, "rTrue0.0"))
        self.assertTrue(self.ex.readiness()["armed"])
        st = self.arm_request(False)
        self.assertEqual((st["on"], st["off_reason"]), (False, "로컬 명령 (tk)"))
        st = self.arm_request(True, age=120)
        self.assertEqual((st["on"], st["refused"]), (False, "request too old"))

    def test_direct_executor_routes_calls_to_the_local_panel(self):
        from roport_omx.shelf_direct.executor import DirectExecutor

        ex = DirectExecutor(ROUTES, self.panel, end_on_place=True)
        self.assertEqual(ex.readiness()["armed"], False)
        ex.check(TASK)

    def test_link_state_feeds_ai_readiness(self):
        self.panel.link = {
            "ok": False,
            "at": time.time(),
            "stage": "ping",
            "error": "timeout",
        }
        self.assertTrue(self.ex.readiness()["error"].startswith("AI_LINK_DOWN"))


if __name__ == "__main__":
    unittest.main()
