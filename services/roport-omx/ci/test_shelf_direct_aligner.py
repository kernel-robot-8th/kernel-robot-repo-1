"""shelf_direct: 정렬기 실행·감독. 가짜 real_runner(ci/fake_real_runner.py)를 실제 프로세스로 띄워 파일 계약을 본다."""

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.shelf_direct.aligner import (
    DEAD_PANEL_URL,
    Aligner,
    AlignerConfig,
    normalize_slot,
    pinky_number,
    rest_check,
    rest_reference,
)

FAKE = ROOT / "ci" / "fake_real_runner.py"
HOME = {
    "joints": {
        f"{s}_{j}": 0.0 for s in ("left", "right") for j in ("shoulder_pan", "gripper")
    }
}


class AlignerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.log = self.dir / "argv.jsonl"

    def make(self, mode="ok", **kw):
        cfg = AlignerConfig(
            cmd=(sys.executable, str(FAKE)),
            work_dir=self.dir / "work",
            cwd=self.dir,
            extra_env={"FAKE_RR_MODE": mode, "FAKE_RR_LOG": str(self.log)},
            **kw,
        )
        a = Aligner(cfg)
        self.addCleanup(a.kill)
        return a

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    # ------------------------------------------------------------ 출고
    def test_tray_check_accepts_and_pinky_read_matches_reference(self):
        a = self.make()
        pre = a.tray_check(16, "2b", "pinky2")
        self.assertTrue(pre["accepted"], pre)
        self.assertEqual(pre["returncode"], 0)
        call = self.calls()[0]
        argv = call["argv"]
        self.assertEqual(argv[argv.index("--step") + 1], "tray_check")
        self.assertEqual(argv[argv.index("--slot") + 1], "2B")
        self.assertEqual(argv[argv.index("--pinky") + 1], "2")
        self.assertIn("--auto", argv)
        self.assertEqual(call["panel"], DEAD_PANEL_URL)  # Panel 서버를 쓰지 않는다
        self.assertEqual(call["cwd"], os.path.expanduser("~"))
        p = a.pinky_read(16, "2B", "pinky2", pre)
        self.assertTrue(p["on_pinky"])
        self.assertTrue(p["pre_mission_match"])
        argv = self.calls()[1]["argv"]
        self.assertEqual(argv[argv.index("--tray-ref") + 1], pre["tray_ref_file"])

    def test_tray_not_empty_or_timeout_is_refused(self):
        self.assertEqual(
            self.make("tray_bad").tray_check(16, None, None)["accepted"], False
        )
        out = self.make("hang", read_timeout_s=0.5).tray_check(16, None, "pinky1")
        self.assertEqual((out["tray_check"], out["accepted"]), ("timeout", False))
        p = self.make("hang", read_timeout_s=0.5).pinky_read(16, None, None, None)
        self.assertEqual((p["skipped"], p["on_pinky"]), ("시간 초과", None))

    # ------------------------------------------------------------ 입고: 넘겨받기
    def standby(self, mode="ok"):
        a = self.make(mode)
        a.start_standby(16, "2B", "repo-1", HOME)
        end = time.monotonic() + 5
        while not a.handover_ready() and time.monotonic() < end:
            time.sleep(0.02)
        self.assertTrue(a.handover_ready())
        call = self.calls()[0]
        self.assertEqual(call["threads"], "1")
        self.assertIn("--home-pose", call["argv"])
        return a

    def placed(self, a, repo="repo-1"):
        a.write_episode_end({"repo_id": repo, "episode": 0, "end_reason": "placed"})
        a.eval_done(True, "placed")

    def test_handover_taken_then_aligned(self):
        a = self.standby()
        self.placed(a)
        st = a.wait_taken()
        self.assertEqual(st["state"], "taken")
        rc = a.wait(5)
        self.assertEqual(rc, 0)
        align = a.align_result(rc)
        self.assertTrue(align["ok"])
        self.assertEqual(align["handover_align"], "from_handover")
        self.assertTrue(align["home_given"])
        self.assertEqual(a.placement("align")["on_shelf"], True)

    def test_handover_not_placed_and_wrong_repo(self):
        a = self.standby()
        a.eval_done(False, "returned")
        self.assertEqual(a.wait_taken()["state"], "not_placed")
        self.assertEqual(a.wait(5), 0)
        a = self.standby()
        self.placed(a, repo="other")
        self.assertEqual(a.wait_taken()["state"], "not_placed")

    def test_handover_register_mismatch_exits_3(self):
        a = self.standby("mismatch")
        self.placed(a)
        self.assertEqual(a.wait_taken()["state"], "regs_mismatch")
        self.assertEqual(a.wait(5), 3)

    def test_not_taken_in_time_fails_and_stops_runner(self):
        a = self.make("silent", taken_wait_s=0.5)
        a.start_standby(16, "2B", "repo-1", None)
        time.sleep(0.3)
        self.placed(a)
        st = a.wait_taken()
        self.assertEqual(st["state"], "failed")
        self.assertTrue((a.c.dir() / "handover_stop").exists())

    def test_heartbeat_only_from_a_live_standby(self):
        a = self.standby()
        a.stop()
        self.assertFalse(a.handover_ready())

    # ------------------------------------------------------------ 입고: 정렬·읽기·되돌리기
    def test_align_after_eval_and_stop(self):
        a = self.make()
        a.start_align(16, "2B", None)
        rc = a.wait(5)
        self.assertTrue(a.align_result(rc)["ok"])
        self.assertNotIn("--home-pose", self.calls()[0]["argv"])

        a = self.make("stop_wait")
        a.start_align(16, "2B", HOME)
        time.sleep(0.5)
        rc = a.stop()
        self.assertEqual(rc, 2)
        align = a.align_result(rc, stopped_by="즉시 중단")
        self.assertFalse(align["ok"])
        self.assertTrue(align["early_exit"])
        self.assertIsNone(align["released"])

    def test_shelf_read_and_release(self):
        a = self.make()
        a.start_shelf_read(16, None)
        self.assertEqual(a.wait(5), 0)
        self.assertEqual(a.placement("shelf_read")["from_result"], True)
        a.start_release(16)
        self.assertEqual(a.wait(5), 0)
        b = self.make("no_route")
        b.start_release(16)
        self.assertEqual(b.wait(5), 3)

    def test_one_process_at_a_time(self):
        a = self.make("hang")
        a.start_align(16, None, None)
        with self.assertRaises(RuntimeError):
            a.start_shelf_read(16, None)


class HelpersTest(unittest.TestCase):
    def test_names(self):
        self.assertEqual(pinky_number("Pinky3"), "3")
        self.assertIsNone(pinky_number("robot"))
        self.assertEqual(normalize_slot("2b", "1A"), "2B")
        self.assertEqual(normalize_slot("x9", "2B"), "2B")

    def test_rest_check(self):
        before = {
            f"{s}_{j}": 0.0
            for s in ("left", "right")
            for j in ("shoulder_pan", "elbow_flex")
        }
        after = {
            "left": {"shoulder_pan": 4.0, "elbow_flex": 0.0, "gripper": 60.0},
            "right": {"shoulder_pan": 0.0, "elbow_flex": 7.0, "gripper": 40.0},
        }
        r = rest_check(before, after)
        self.assertEqual(r["check"], "far")
        self.assertEqual(r["arms"]["OMX 1"]["far_joints"], [])
        self.assertEqual(r["arms"]["OMX 2"]["far_joints"], ["elbow_flex"])
        self.assertEqual(r["arms"]["OMX 1"]["gripper_state"], "open")
        self.assertEqual(r["arms"]["OMX 2"]["gripper_state"], "closed")
        self.assertEqual(rest_check(None, after)["check"], "unknown")

    def test_rest_reference_order(self):
        self.assertEqual(
            rest_reference({"rest_target": "home", "home_pose": {"a": 1}}, None)[1],
            "home",
        )
        self.assertEqual(rest_reference({"start_pose": {"a": 1}}, None)[1], "runner")
        ref, src = rest_reference({}, {"left_x.pos": 2.0})
        self.assertEqual((ref, src), ({"left_x": 2.0}, "eval_end"))


if __name__ == "__main__":
    unittest.main()
