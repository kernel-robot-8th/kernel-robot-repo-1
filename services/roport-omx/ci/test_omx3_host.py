"""OMX 3 Host 검사 (가짜 runner, 표준 라이브러리만)."""

import sys
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.omx3.contract import ErrorCode, Outcome, Phase  # noqa: E402
from roport_omx.omx3.host import Omx3Host  # noqa: E402
from roport_omx.omx3.settings import Omx3Settings  # noqa: E402


class FakeRobot:
    def ready(self):
        return None


class FakeRunner:
    """run()은 gate가 열릴 때까지 RUNNING에 머문다."""

    def __init__(self, precheck=None, outcome=Phase.COMPLETE, error=None):
        self.settings = Omx3Settings()
        self.robot = FakeRobot()
        self.code, self.outcome, self.error = precheck, outcome, error
        self.gate = threading.Event()
        self.gate.set()
        self.runs, self.abort = [], None

    def precheck(self, marker_id):
        return self.code

    def run(self, marker_id, abort, on_phase):
        self.runs.append(marker_id)
        self.abort = abort
        for phase in (Phase.MARKER_CHECK, Phase.HOMING, Phase.RUNNING):
            on_phase(phase, {})
        self.gate.wait(5)
        if self.error:
            raise self.error
        if abort.is_set():
            return Outcome(Phase.FAILED, ErrorCode.CANCELED, marker_id)
        on_phase(Phase.VERIFY, {})
        code = None if self.outcome is Phase.COMPLETE else ErrorCode.TIMEOUT
        return Outcome(self.outcome, code, marker_id, placement_confirmed=True)


def request(cmd="c1", package_id="17", **payload):
    return {
        "command_id": cmd,
        "execution_id": "e1",
        "payload": {"package_id": package_id, **payload},
    }


def make(runner=None, journal=None):
    events = []
    runner = runner or FakeRunner()
    host = Omx3Host(
        runner, lambda kind, req, p: events.append((kind, p)), journal_path=journal
    )
    return host, runner, events


def kinds(events):
    return [k for k, _ in events]


class Accept(unittest.TestCase):
    def test_accepted_progress_completed(self):
        host, runner, events = make()
        host.handle_load(request())
        host.wait(5)
        self.assertEqual(
            kinds(events),
            ["load.accepted"] + ["load.progress"] * 3 + ["load.completed"],
        )
        self.assertEqual(
            [p["phase"] for k, p in events if k == "load.progress"],
            ["HOMING", "RUNNING", "VERIFY"],
        )
        done = events[-1][1]
        self.assertEqual((done["phase"], done["marker_id"]), ("COMPLETE", 17))
        self.assertTrue(all(p["boot_id"] == host.boot_id for _, p in events))
        self.assertTrue(host.status()["ready"])

    def test_failed_outcome_and_run_exception(self):
        for runner, code in (
            (FakeRunner(outcome=Phase.FAILED), "TIMEOUT"),
            (FakeRunner(error=OSError("bus")), "ROBOT_ERROR"),
        ):
            with self.subTest(code):
                host, _, events = make(runner)
                host.handle_load(request())
                host.wait(5)
                self.assertEqual(events[-1][0], "load.failed")
                self.assertEqual(events[-1][1]["error_code"], code)
                self.assertIsNone(host.status()["active"])

    def test_abort_reaches_runner(self):
        host, runner, events = make()
        runner.gate.clear()
        host.handle_load(request())
        host.abort()
        runner.gate.set()
        host.wait(5)
        self.assertEqual(events[-1][1]["error_code"], "CANCELED")


class Duplicate(unittest.TestCase):
    def test_same_request_replays_last_event_without_running(self):
        host, runner, events = make()
        host.handle_load(request())
        host.wait(5)
        host.handle_load(request())
        self.assertEqual(len(runner.runs), 1)
        self.assertEqual(events[-1], events[-2])

    def test_same_key_other_payload_conflicts(self):
        host, runner, events = make()
        host.handle_load(request())
        host.wait(5)
        host.handle_load(request(package_id="18"))
        self.assertEqual(len(runner.runs), 1)
        self.assertEqual(
            (events[-1][0], events[-1][1]["error_code"]),
            ("load.rejected", "IDEMPOTENCY_CONFLICT"),
        )

    def test_busy_then_same_command_accepted_later(self):
        host, runner, events = make()
        runner.gate.clear()
        host.handle_load(request("c1"))
        host.handle_load(request("c2"))
        self.assertEqual(events[-1][1]["error_code"], "BUSY")
        runner.gate.set()
        host.wait(5)
        host.handle_load(request("c2"))  # BUSY는 기록하지 않으므로 다시 받는다
        host.wait(5)
        self.assertEqual(len(runner.runs), 2)


class Rejected(unittest.TestCase):
    def test_invalid_target_and_precheck_reject_without_run(self):
        for runner, req, code, marker in (
            (FakeRunner(), request(package_id=17), "INVALID_TARGET", None),
            (FakeRunner(), request(package_id="3"), "INVALID_TARGET", None),
            (
                FakeRunner(precheck=ErrorCode.MARKER_NOT_VISIBLE),
                request(),
                "MARKER_NOT_VISIBLE",
                17,
            ),
        ):
            with self.subTest(code=code, req=req["payload"]):
                host, _, events = make(runner)
                host.handle_load(req)
                self.assertEqual(runner.runs, [])
                kind, p = events[-1]
                self.assertEqual(
                    (kind, p["error_code"], p["marker_id"]),
                    ("load.rejected", code, marker),
                )
                host.handle_load(req)  # 같은 거절을 다시 보낸다
                self.assertEqual(events[-1], events[-2])


class Journal(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "omx3.jsonl"

    def tearDown(self):
        self.dir.cleanup()

    def test_restart_replays_finished_command(self):
        host, _, _ = make(journal=self.path)
        host.handle_load(request())
        host.wait(5)
        host2, runner2, events2 = make(journal=self.path)
        host2.handle_load(request())
        self.assertEqual(runner2.runs, [])
        self.assertEqual(events2[-1][0], "load.completed")
        self.assertEqual(host2.reconciling, [])

    def test_unfinished_run_blocks_until_cleared(self):
        host, runner, _ = make(journal=self.path)
        runner.gate.clear()
        host.handle_load(request("c1"))  # 실행 중 프로세스가 죽었다고 본다
        with self.path.open("a") as f:
            f.write('{"key": "c9/e1", "fp"')  # 쓰다 끊긴 줄
        host2, runner2, events2 = make(journal=self.path)
        self.assertEqual(host2.reconciling, ["c1/e1"])
        self.assertFalse(host2.status()["ready"])
        host2.handle_load(request("c2"))
        self.assertEqual(events2[-1][1]["error_code"], "RECONCILING")
        host2.clear_reconciling("c1/e1")
        host2.handle_load(request("c3"))
        host2.wait(5)
        self.assertEqual(events2[-1][0], "load.completed")
        self.assertEqual(make(journal=self.path)[0].reconciling, [])  # 해제도 기록됨
        runner.gate.set()
        host.wait(5)


if __name__ == "__main__":
    unittest.main()
