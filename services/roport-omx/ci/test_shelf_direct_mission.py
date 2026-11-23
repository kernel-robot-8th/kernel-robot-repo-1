"""shelf_direct: 미션 흐름. 가짜 runner + 실제 Aligner(가짜 real_runner 프로세스)로 돌리고, 결과를 현장
PanelExecutor 판정 함수(_result)에 그대로 넣어 v2 결과가 Panel 경로와 같은 의미인지 본다."""

import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.panel_executor import PanelExecutor
from roport_omx.shelf_direct.aligner import Aligner, AlignerConfig
from roport_omx.shelf_direct.contract import STATE_KEYS
from roport_omx.shelf_direct.mission import Mission, arm_states
from roport_omx.shelf_direct.runner import RunResult

FAKE = ROOT / "ci" / "fake_real_runner.py"
HOME = {
    "joints": {k.removesuffix(".pos"): 0.0 for k in STATE_KEYS},
    "std": {},
}
AT_HOME = tuple(0.0 if "gripper" not in k else 40.0 for k in STATE_KEYS)
BODY_IN = {
    "source": "mqtt",
    "model_path": "/models/in",
    "task": "INBOUND 2B | marker 16",
    "target_marker": 16,
    "episode_s": 60,
    "end_on_return": True,
    "shelf_align": True,
    "fast_release": True,
    "end_on_place": True,
    "slot": "2B",
    "pinky_id": "pinky2",
}
BODY_OUT = {
    "source": "mqtt",
    "model_path": "/models/out",
    "task": "OUTBOUND",
    "target_marker": 16,
    "episode_s": 60,
    "end_on_return": True,
    "shelf_align": False,
    "pinky_check": True,
    "draw_target": False,
    "slot": "2B",
    "pinky_id": "pinky2",
}


class FakeWatch:
    def __init__(self, hit=None):
        self.hit = hit

    def placed(self):
        return self.hit

    def summary(self):
        return {"version": "t", "updates": 3, "errors": 0, "hit": self.hit}


HIT = {
    "placed": True,
    "box": {"id": 16, "flat": True},
    "judged_ts": time.time(),
    "judged_at": "2026-11-23T12:00:00.000Z",
    "frame_ts": time.time(),
    "frame_at": "2026-11-23T12:00:00.000Z",
    "tip_y": [0.1, 0.12],
    "y_out": 0.18,
    "version": "1117e",
    "joints": {"left_gripper": 60.0, "right_gripper": 61.0},
}


class FakeRunner:
    def __init__(self, result, on_run=None):
        self.result, self.on_run = result, on_run

    def run(self, task, stop):
        if self.on_run:
            self.on_run(stop)
        return self.result


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.homes = []

    def mission(
        self,
        body,
        result,
        *,
        mode="ok",
        watch=None,
        on_run=None,
        setup_error=None,
        **acfg,
    ):
        aligner = Aligner(
            AlignerConfig(
                cmd=(sys.executable, str(FAKE)),
                work_dir=self.dir / "work",
                cwd=self.dir,
                extra_env={
                    "FAKE_RR_MODE": mode,
                    "FAKE_RR_LOG": str(self.dir / "argv.jsonl"),
                },
                **acfg,
            )
        )
        self.addCleanup(aligner.kill)

        def make_runner(b, plan, al):
            if setup_error:
                raise setup_error
            if plan["handover"]:  # 정렬기 심박이 뜰 때까지(실제 평가는 수십 초)
                end = time.monotonic() + 5
                while not al.handover_ready() and time.monotonic() < end:
                    time.sleep(0.02)
            return FakeRunner(result, on_run), watch

        def home(arms):
            self.homes.append(arms)
            return {"left": True, "right": True}

        return Mission(
            body,
            "repo-1",
            make_runner=make_runner,
            aligner=aligner,
            read_arms=lambda: {
                s: {"shoulder_pan": 0.0, "gripper": 40.0} for s in ("left", "right")
            },
            return_home=home,
            model_home=HOME,
            ports={"left": "/dev/a", "right": "/dev/b"},
            status_path=self.dir / "status.json",
        )

    def status(self):
        return json.loads((self.dir / "status.json").read_text())

    @staticmethod
    def judged(result, operation, box=16):
        return PanelExecutor._result(result, "2b", operation, box)

    def test_inbound_handover_taken_aligned_is_shelf_align(self):
        res = RunResult(
            "placed",
            landing="hold",
            last_state=AT_HOME,
            elapsed_s=20.0,
            state_at=time.time(),
        )
        m = self.mission(BODY_IN, res, watch=FakeWatch(HIT))
        self.assertEqual(m.run(), 0)
        st = self.status()
        r = st["last_result"]
        self.assertEqual((st["phase"], st["running"]), ("done", False))
        self.assertEqual((r["stage"], r["final"], r["exit_ok"]), ("done", True, True))
        self.assertEqual(r["handover"]["state"], "taken")
        self.assertEqual(r["placement"]["first"]["source"], "eval_judge")
        self.assertEqual(r["align"]["rest"]["check"], "ok")
        obs = self.judged(r, "INBOUND")
        self.assertEqual(
            (obs["state"], obs.get("evidence")), ("finished", "shelf_align")
        )

    def test_inbound_returned_then_align(self):
        res = RunResult("returned", landing="fast", last_state=AT_HOME, elapsed_s=30.0)
        m = self.mission(dict(BODY_IN, end_on_place=False), res)
        m.run()
        r = self.status()["last_result"]
        self.assertNotIn("handover", r)
        self.assertTrue(r["align"]["ok"])
        self.assertEqual(self.judged(r, "INBOUND")["evidence"], "shelf_align")

    def test_handover_not_taken_goes_home_and_reports_not_aligned(self):
        res = RunResult(
            "placed", landing="hold", last_state=AT_HOME, start_poses={"left": {"a": 1}}
        )
        m = self.mission(
            BODY_IN, res, mode="silent", watch=FakeWatch(HIT), taken_wait_s=0.5
        )
        m.run()
        r = self.status()["last_result"]
        self.assertEqual(r["handover"]["state"], "failed")
        self.assertEqual(
            (r["handover"]["fallback"], r["handover"]["fallback_ok"]), ("home", True)
        )
        self.assertEqual(self.homes[0]["left"], {"port": "/dev/a", "joints": {"a": 1}})
        obs = self.judged(r, "INBOUND")
        self.assertEqual(
            (obs["state"], obs["evidence"]), ("finished", "shelf_load_not_aligned")
        )
        self.assertEqual(obs["align_error"], "PANEL_HANDOVER_FAILED")

    def test_register_mismatch_holds(self):
        res = RunResult("placed", landing="hold", last_state=AT_HOME)
        m = self.mission(BODY_IN, res, mode="mismatch", watch=FakeWatch(HIT))
        m.run()
        r = self.status()["last_result"]
        self.assertEqual((r["stage"], r["final"]), ("hold", False))
        self.assertEqual(r["handover"]["fallback"], "hold")
        self.assertEqual(self.homes, [])

    def test_outbound_tray_refused_does_not_run(self):
        ran = []
        m = self.mission(
            BODY_OUT,
            RunResult("returned"),
            mode="tray_bad",
            on_run=lambda s: ran.append(1),
        )
        self.assertEqual(m.run(), 4)
        st = self.status()
        self.assertEqual((st["phase"], st["refused"]), ("refused", "tray_check"))
        self.assertEqual(st["tray_check"]["tray_check"], "not_empty")
        self.assertEqual(ran, [])

    def test_outbound_pinky_load(self):
        m = self.mission(
            BODY_OUT, RunResult("returned", landing="soft", last_state=AT_HOME)
        )
        m.run()
        r = self.status()["last_result"]
        self.assertTrue(r["pinky"]["pre_mission_match"])
        obs = self.judged(r, "OUTBOUND")
        self.assertEqual((obs["state"], obs["evidence"]), ("finished", "pinky_load"))

    def test_stop_during_eval_is_operator_stop(self):
        res = RunResult("stopped", landing="soft", last_state=AT_HOME)
        m = self.mission(
            dict(BODY_IN, end_on_place=False),
            res,
            on_run=lambda s: m.request_stop("stop"),
        )
        m.run()
        r = self.status()["last_result"]
        self.assertEqual(r["stopped_by"], "중지")
        self.assertNotIn("align", r)
        obs = self.judged(r, "INBOUND")
        self.assertEqual(
            (obs["state"], obs["error"], obs["operator_stop"]),
            ("failed", "PANEL_STOPPED", True),
        )

    def test_stop_during_align_releases(self):
        res = RunResult("returned", landing="fast", last_state=AT_HOME)
        m = self.mission(dict(BODY_IN, end_on_place=False), res, mode="stop_wait")
        t = threading.Thread(target=m.run)
        t.start()
        end = time.monotonic() + 5
        while m.aligner.kind != "align" and time.monotonic() < end:
            time.sleep(0.02)
        time.sleep(0.3)
        m.request_stop("stop")
        t.join(15)
        r = self.status()["last_result"]
        self.assertEqual(r["align"]["stopped_by"], "중지")
        self.assertTrue(r["align"]["released"])
        obs = self.judged(r, "INBOUND")
        self.assertEqual(
            (obs["state"], obs["error"]), ("failed", "PANEL_ALIGN_STOPPED")
        )
        self.assertTrue(obs["operator_stop"])

    def test_setup_failure_is_inference_error(self):
        m = self.mission(BODY_IN, None, setup_error=RuntimeError("GPU down"))
        self.assertEqual(m.run(), 1)
        r = self.status()["last_result"]
        self.assertEqual(r["err"]["code"], "INFERENCE_UNAVAILABLE")
        self.assertEqual(
            self.judged(r, "INBOUND")["error"], "PANEL_INFERENCE_UNAVAILABLE"
        )


class ArmStatesTest(unittest.TestCase):
    def test_home_check_uses_three_sigma(self):
        home = {
            "joints": {"left_shoulder_pan": 0.0, "right_shoulder_pan": 0.0},
            "std": {"left_shoulder_pan": 2.0},
        }
        st = {
            "left_shoulder_pan.pos": 5.0,
            "left_gripper.pos": 58.0,
            "right_shoulder_pan.pos": 2.0,
            "right_gripper.pos": 40.0,
        }
        a = arm_states(st, home)
        self.assertTrue(a["OMX 1"]["at_home"])  # 5 <= 3 x 2
        self.assertEqual(a["OMX 2"]["far_joints"], ["shoulder_pan"])  # 2 > 3 x 0.5
        self.assertEqual(
            (a["OMX 1"]["gripper_state"], a["OMX 2"]["gripper_state"]),
            ("open", "closed"),
        )
        self.assertIsNone(arm_states(st, None)["OMX 1"]["at_home"])


if __name__ == "__main__":
    unittest.main()
