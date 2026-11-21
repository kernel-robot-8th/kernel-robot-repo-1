"""HOLD release from the panel on this PC (local files); simulated executor, no broker or Paho."""

import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "ci"))

try:
    import roport_omx.mqtt_link.mqtt  # noqa: F401
except ImportError:  # system Python in CI has no Paho; HoldChannel never needs it
    stub = types.ModuleType("roport_omx.mqtt_link.mqtt")
    stub.MqttConfig = stub.MqttTransport = object
    sys.modules["roport_omx.mqtt_link.mqtt"] = stub

from roport_omx.cooperative import CooperativeHost
from roport_omx.mqtt_link.journal import Journal
from test_cooperative import BINDING, CONTROL, SITE, mock, request, task_payload

spec = importlib.util.spec_from_file_location(
    "cooperative_host", ROOT / "scripts/cooperative_host.py"
)
ch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ch)

CHECKED = {"arms_safe": True, "package_checked": True, "pinky_checked": True}


class HoldReleaseTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "shelf-01.json"
        self.executor = mock.MockExecutor(run_s=0)
        self.journal = Journal(self.path)
        self.host = self.make_host()
        self.holds = ch.HoldChannel(self.path)

    def tearDown(self):
        self.journal.close()
        self.dir.cleanup()

    def make_host(self):
        return CooperativeHost(
            self.journal, self.executor, site=SITE, control=CONTROL, binding=BINDING
        )

    def fail_with_arm_away(self):
        self.executor.unreturned = {"omx2"}
        message = request("shelf_inbound.requested", task_payload(self.host.boot_id))
        self.host.handle(message, channel="cmd")
        self.host.poll()
        self.assertEqual(self.host.hold, message["command_id"])
        return message["command_id"]

    def release(self, **changes):
        body = {"request_id": "r1", "hold": self.host.hold, "checks": CHECKED} | changes
        return self.host.release_hold(body)

    def test_hold_info_names_the_cause(self):
        self.assertIsNone(self.host.hold_info())
        command_id = self.fail_with_arm_away()
        info = self.host.hold_info()
        self.assertEqual(info["command_id"], command_id)
        self.assertEqual(info["error_code"], "ARM_NOT_RETURNED")

    def test_release_needs_the_same_hold_all_checks_and_arms_home(self):
        self.fail_with_arm_away()
        self.assertEqual(self.release(hold="other")["reason"], "HOLD_CHANGED")
        partly = CHECKED | {"pinky_checked": False}
        self.assertEqual(self.release(checks=partly)["reason"], "CHECKS_MISSING")
        refused = self.release()  # omx2 still away from home
        self.assertEqual(refused["reason"], "ARMS_NOT_HOME")
        self.assertIsNotNone(self.host.hold)
        self.executor.unreturned = set()  # a person brought the arm home
        cleared = self.release(by="panel")
        self.assertIs(cleared["cleared"], True)
        self.assertIsNone(self.host.hold)
        self.assertIsNone(self.journal.data["hold"])  # saved: survives a restart
        self.assertEqual(self.release()["reason"], "NO_HOLD")

    def test_restart_hold_has_unknown_result(self):
        message = request("shelf_inbound.requested", task_payload(self.host.boot_id))
        self.executor.run_s = 60
        self.host.handle(message, channel="cmd")
        restarted = self.make_host()  # previous process left the task running
        self.assertEqual(restarted.hold_info()["error_code"], "RESULT_UNKNOWN")

    def test_channel_reads_a_request_once_and_writes_state_and_result(self):
        command_id = self.fail_with_arm_away()
        self.holds.tick(self.host, now=1000.0)
        state = json.loads(self.holds.state_path.read_text())
        self.assertEqual(state["hold"]["command_id"], command_id)
        self.assertEqual(state["boot_id"], self.host.boot_id)
        self.executor.unreturned = set()
        ask = {"request_id": "r7", "hold": command_id, "checks": CHECKED}
        self.holds.request_path.write_text(json.dumps(ask | {"requested_at": 1001.0}))
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1002.0)
        self.assertFalse(self.holds.request_path.exists())  # read once
        result = json.loads(self.holds.result_path.read_text())
        self.assertEqual((result["request_id"], result["cleared"]), ("r7", True))
        self.assertIsNone(json.loads(self.holds.state_path.read_text())["hold"])

    def test_channel_refuses_old_or_broken_requests(self):
        command_id = self.fail_with_arm_away()
        self.executor.unreturned = set()
        ask = {"request_id": "r8", "hold": command_id, "checks": CHECKED}
        for i, body in enumerate(
            (
                json.dumps(ask | {"requested_at": 900.0}),  # 100 s or more old
                json.dumps(ask),  # no time
                "{not json",
            )
        ):
            with self.subTest(body=body):
                self.holds.request_path.write_text(body)
                with self.assertLogs("cooperative_host", "WARNING"):
                    # each answer stays on top RESULT_SHOWN_S before the next request
                    self.holds.tick(self.host, now=1000.0 + 2 * i)
                result = json.loads(self.holds.result_path.read_text())
                self.assertEqual(result["reason"], "STALE_REQUEST")
                self.assertEqual(self.host.hold, command_id)

    def ask_release(self, command_id, request_id="r9", at=1001.0):
        ask = {"request_id": request_id, "hold": command_id, "checks": CHECKED}
        self.holds.request_path.write_text(json.dumps(ask | {"requested_at": at}))

    def count_releases(self):
        calls = []
        release = self.host.release_hold

        def counted(request):
            calls.append(request)
            return release(request)

        self.host.release_hold = counted
        return calls

    def test_result_file_error_keeps_the_host_and_writes_the_result_later(self):
        # 2026-11-21 review: the result file could not be replaced → IsADirectoryError stopped
        # the Host after the journal had cleared the HOLD; the screen kept the old HOLD
        command_id = self.fail_with_arm_away()
        self.executor.unreturned = set()
        calls = self.count_releases()
        self.holds.result_path.mkdir()  # replace() onto a directory fails
        self.ask_release(command_id)
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1002.0)  # does not raise
        self.assertIsNone(self.journal.data["hold"])  # the release itself is saved
        self.assertFalse(self.holds.request_path.exists())
        self.assertIsNone(json.loads(self.holds.state_path.read_text())["hold"])
        self.holds.result_path.rmdir()
        self.holds.tick(self.host, now=1003.0)
        result = json.loads(self.holds.result_path.read_text())
        self.assertEqual((result["request_id"], result["cleared"]), ("r9", True))
        self.assertEqual(len(calls), 1)  # written again, not run again

    def panel_answer(self, request_id, *, old_panel=False):
        """What the panel's request_hold_release() reads for its own request_id (every 100 ms).

        old_panel: a panel that looks only at the top of the file (before the results map).
        """
        try:
            res = json.loads(self.holds.result_path.read_text())
        except (OSError, ValueError):
            return None
        if res.get("request_id") == request_id:
            return res
        return None if old_panel else (res.get("results") or {}).get(request_id)

    def test_each_panel_request_reads_its_own_answer_after_a_write_failure(self):
        # 2026-11-21 reviews: while the result file could not be written a second request came;
        # after the repair the first answer (the HOLD was cleared) must still reach its panel
        command_id = self.fail_with_arm_away()
        self.executor.unreturned = set()
        calls = self.count_releases()
        self.holds.result_path.mkdir()
        self.ask_release(command_id, "r9", at=1001.0)
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(
                self.host, now=1002.0
            )  # r9 cleared the HOLD, result not written
        self.ask_release(command_id, "r10", at=1003.0)  # the person presses again
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1004.0)
        self.assertTrue(self.holds.request_path.exists())  # r10 left unread
        self.assertEqual(len(calls), 1)
        self.holds.result_path.rmdir()
        self.holds.tick(self.host, now=1005.0)  # repaired: r9's answer is written
        for old_panel in (False, True):
            self.assertIs(self.panel_answer("r9", old_panel=old_panel)["cleared"], True)
        self.holds.tick(self.host, now=1005.5)  # r9 stays on top for RESULT_SHOWN_S
        self.assertTrue(self.holds.request_path.exists())
        self.assertIs(self.panel_answer("r9", old_panel=True)["cleared"], True)
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1006.1)  # then r10 is answered
        self.assertEqual(len(calls), 2)
        self.assertEqual(self.panel_answer("r10")["reason"], "NO_HOLD")
        self.assertIs(self.panel_answer("r9")["cleared"], True)  # still found by its id

    def test_answers_are_kept_by_request_id_for_a_minute(self):
        command_id = self.fail_with_arm_away()
        self.ask_release(command_id, "r1", at=1001.0)
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1001.5)  # arms away: refused
        self.assertEqual(self.panel_answer("r1")["reason"], "ARMS_NOT_HOME")
        self.ask_release(command_id, "r2", at=1030.0)
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1030.0)
        self.assertEqual(self.panel_answer("r1")["reason"], "ARMS_NOT_HOME")
        self.ask_release(command_id, "r3", at=1070.0)
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1070.0)
        self.assertIsNone(self.panel_answer("r1"))  # older than RESULT_KEEP_S
        self.assertIsNotNone(self.panel_answer("r2"))
        self.assertEqual(self.panel_answer("r3")["request_id"], "r3")

    def test_state_file_error_is_retried(self):
        command_id = self.fail_with_arm_away()
        self.holds.state_path.mkdir()
        with self.assertLogs("cooperative_host", "WARNING"):
            self.holds.tick(self.host, now=1000.0)
        self.holds.state_path.rmdir()
        self.holds.tick(self.host, now=1001.0)
        state = json.loads(self.holds.state_path.read_text())
        self.assertEqual(state["hold"]["command_id"], command_id)

    def test_request_that_cannot_be_removed_runs_once(self):
        command_id = self.fail_with_arm_away()
        self.executor.unreturned = set()
        calls = self.count_releases()
        # the panel files only; the journal stays writable
        folder = Path(self.dir.name) / "panel"
        folder.mkdir()
        self.holds = ch.HoldChannel(folder / self.path.name)
        self.ask_release(command_id)
        folder.chmod(0o500)  # no unlink, no result/state writes
        try:
            with self.assertLogs("cooperative_host", "WARNING"):
                self.holds.tick(self.host, now=1002.0)
                self.holds.tick(self.host, now=1003.0)
        finally:
            folder.chmod(0o700)
        self.assertEqual(len(calls), 1)
        self.assertIsNone(self.host.hold)
        self.holds.tick(self.host, now=1004.0)
        self.assertIs(json.loads(self.holds.result_path.read_text())["cleared"], True)
        self.assertEqual(len(calls), 1)

    def test_journal_error_still_stops_the_host(self):
        command_id = self.fail_with_arm_away()

        def broken(_request):
            raise ch.StorageError("journal not saved")

        self.host.release_hold = broken
        self.ask_release(command_id)
        with self.assertRaises(ch.StorageError):
            self.holds.tick(self.host, now=1002.0)


if __name__ == "__main__":
    unittest.main()
