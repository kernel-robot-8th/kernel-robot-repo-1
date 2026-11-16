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
        for body in (
            json.dumps(ask | {"requested_at": 900.0}),  # 100 s old
            json.dumps(ask),  # no time
            "{not json",
        ):
            with self.subTest(body=body):
                self.holds.request_path.write_text(body)
                with self.assertLogs("cooperative_host", "WARNING"):
                    self.holds.tick(self.host, now=1000.0)
                result = json.loads(self.holds.result_path.read_text())
                self.assertEqual(result["reason"], "STALE_REQUEST")
                self.assertEqual(self.host.hold, command_id)


if __name__ == "__main__":
    unittest.main()
