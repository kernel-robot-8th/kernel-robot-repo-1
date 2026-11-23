import sys
import tempfile
import unittest
from datetime import UTC, datetime, timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roport_omx.panel_command_api import PanelCommandAPI
from roport_omx.panel_commands import PanelCommands


class PanelAPITest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.ledger = PanelCommands(Path(self.tmp.name) / "commands.sqlite")
        self.starts, self.stops = [], []
        self.api = PanelCommandAPI(
            self.ledger,
            validate_payload=self.validate,
            begin=self.starts.append,
            stop=lambda: self.stops.append(1),
        )
        self.body = {
            "command_id": "test-1",
            "expires_at": (datetime.now(UTC) + timedelta(seconds=10)).isoformat(),
            "payload": {"model": "calibrated"},
        }

    def tearDown(self):
        self.ledger.close()
        self.tmp.cleanup()

    @staticmethod
    def validate(payload):
        if payload.get("model") != "calibrated":
            raise ValueError("model unavailable")

    def submit(self):
        return self.api.handle("POST", "/api/control/commands", self.body)

    def test_duplicate_finish_and_rejected_payload(self):
        self.assertEqual(self.submit()[0], 202)
        self.assertEqual(self.submit()[0], 202)
        self.assertEqual(len(self.starts), 1)
        self.ledger.finish("test-1", {"state": "finished", "arms": {}})
        self.assertEqual(self.submit()[1]["state"], "completed")
        self.body["payload"]["model"] = "unapproved"
        self.assertEqual(self.submit()[0], 409)
        self.body["command_id"] = "test-2"
        self.assertEqual(self.submit()[0], 400)
        self.assertEqual(len(self.starts), 1)

    def test_cancel_before_start_and_running_confirmation(self):
        self.api.handle("POST", "/api/control/commands/test-1/cancel", {})
        self.assertEqual(self.submit()[1]["state"], "cancelled")
        self.assertFalse(self.starts)
        self.body["command_id"] = "test-2"
        self.submit()
        self.assertEqual(
            self.api.handle("POST", "/api/control/commands/test-2/cancel", {})[1][
                "state"
            ],
            "stopping",
        )
        self.ledger.confirm_stopped(
            "test-2", {"observed_at": datetime.now(UTC).isoformat(), "stopped": True}
        )
        self.assertEqual(
            self.api.handle("GET", "/api/control/commands/test-2")[1]["state"],
            "cancelled",
        )
        self.assertEqual(self.stops, [1])

    def test_failed_start_stays_unknown_and_does_not_retry(self):
        def failed(payload):
            raise RuntimeError("process start reply missing")

        self.api.begin = failed
        self.assertEqual(self.submit()[0], 503)
        self.assertEqual(self.submit()[1]["state"], "unknown")

    def test_no_remote_completion_or_arbitrary_paths(self):
        self.assertEqual(
            self.api.handle(
                "POST", "/api/control/commands/test-1/finish", {"state": "finished"}
            )[0],
            400,
        )
        self.assertEqual(self.api.handle("POST", "/api/eval/start", self.body)[0], 404)
        self.assertEqual(
            self.api.handle("GET", "/api/control/capabilities")[1],
            {"command_cancel": "durable"},
        )
        self.assertFalse(self.starts)

    def test_invalid_envelope_is_400_and_busy_is_409(self):
        original = dict(self.body)
        for changes in (
            {"expires_at": "bad"},
            {"expires_at": "2026-11-16T00:00:00"},
            {"extra": True},
            {"expires_at": None},
        ):
            self.body = original | changes
            self.assertEqual(self.submit()[0], 400, changes)
        self.body = {k: v for k, v in original.items() if k != "expires_at"}
        self.assertEqual(self.submit()[0], 400)
        self.body = original
        self.assertEqual(self.submit()[0], 202)
        self.body = original | {"command_id": "busy-2"}
        self.assertEqual(self.submit()[0], 409)
