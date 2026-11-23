import json
import sys
import tempfile
import threading
import unittest
from datetime import UTC, datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from test_field_panel import ROUTES

from roport_omx.managed_panel import ManagedPanelExecutor, PanelStartRejected
from roport_omx.field_panel import PanelHTTPError
from roport_omx.panel_command_api import PanelCommandAPI
from roport_omx.panel_commands import PanelCommands


class PanelCommandsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "panel.sqlite"
        self.now = datetime.now(UTC)
        self.ledger = PanelCommands(self.path, clock=lambda: self.now)
        self.starts, self.stops = [], []
        self.request = {
            "command_id": "test-1",
            "expires_at": (self.now + timedelta(seconds=30)).isoformat(),
            "payload": {"model": "test"},
        }

    def tearDown(self):
        self.ledger.close()
        self.tmp.cleanup()

    def test_cancel_before_delayed_request_survives_restart(self):
        self.ledger.cancel("test-1", lambda: self.stops.append(1))
        self.ledger.close()
        self.ledger = PanelCommands(self.path, clock=lambda: self.now)
        self.assertEqual(self.ledger.submit(self.request)["state"], "cancelled")
        self.ledger.start("test-1", self.starts.append)
        self.assertEqual(self.starts, [])
        self.assertEqual(self.stops, [])

    def test_duplicate_and_conflict_never_start_twice(self):
        self.ledger.submit(self.request)
        self.ledger.start("test-1", self.starts.append)
        self.ledger.submit(self.request)
        self.ledger.start("test-1", self.starts.append)
        self.assertEqual(len(self.starts), 1)
        with self.assertRaisesRegex(ValueError, "reused"):
            self.ledger.submit(self.request | {"payload": {"model": "other"}})

    def test_expiry_rechecked_at_actual_start(self):
        self.ledger.submit(self.request)
        self.now += timedelta(seconds=31)
        self.assertEqual(
            self.ledger.start("test-1", self.starts.append)["state"], "expired"
        )
        self.assertEqual(self.starts, [])

    def test_running_cancel_requires_device_confirmation_and_ignores_late_completion(
        self,
    ):
        self.ledger.submit(self.request)
        self.ledger.start("test-1", self.starts.append)
        self.ledger.cancel("test-1", lambda: self.stops.append(1))
        self.assertEqual(self.ledger.get("test-1")["state"], "stopping")
        self.ledger.finish("test-1", {"state": "finished"})
        self.assertEqual(self.ledger.get("test-1")["state"], "stopping")
        self.ledger.confirm_stopped(
            "test-1", {"stopped": True, "observed_at": self.now.isoformat()}
        )
        self.assertEqual(self.ledger.get("test-1")["state"], "cancelled")

    def test_cancel_rejects_old_future_or_moving_motor_evidence(self):
        self.ledger.submit(self.request)
        self.ledger.start("test-1", self.starts.append)
        self.ledger.cancel("test-1", lambda: None)
        for seconds, stopped in [(-1, True), (1, True), (0, False)]:
            with self.assertRaisesRegex(ValueError, "post-cancel"):
                self.ledger.confirm_stopped(
                    "test-1",
                    {
                        "stopped": stopped,
                        "observed_at": (
                            self.now + timedelta(seconds=seconds)
                        ).isoformat(),
                    },
                )
            self.assertEqual(self.ledger.get("test-1")["state"], "stopping")
        at = self.now.isoformat()
        self.now += timedelta(seconds=4)
        with self.assertRaisesRegex(ValueError, "post-cancel"):
            self.ledger.confirm_stopped("test-1", {"stopped": True, "observed_at": at})

    def test_restart_blocks_new_work_until_cancel_and_observation(self):
        self.ledger.submit(self.request)
        self.ledger.start("test-1", self.starts.append)
        self.ledger.close()
        self.ledger = PanelCommands(self.path, clock=lambda: self.now)
        self.assertEqual(self.ledger.get("test-1")["state"], "unknown")
        with self.assertRaisesRegex(ValueError, "reconciliation"):
            self.ledger.submit(self.request | {"command_id": "test-2"})
        self.ledger.cancel("test-1", lambda: self.stops.append(1))
        self.ledger.confirm_stopped(
            "test-1", {"stopped": True, "observed_at": self.now.isoformat()}
        )
        self.assertEqual(
            self.ledger.submit(self.request | {"command_id": "test-2"})["state"],
            "pending",
        )

    def test_restart_requires_a_new_stop_observation(self):
        self.ledger.submit(self.request)
        self.ledger.start("test-1", self.starts.append)
        self.ledger.cancel("test-1", lambda: None)
        old = self.now.isoformat()
        self.ledger.close()
        self.now += timedelta(seconds=1)
        self.ledger = PanelCommands(self.path, clock=lambda: self.now)
        self.ledger.cancel("test-1", lambda: None)
        with self.assertRaisesRegex(ValueError, "post-cancel"):
            self.ledger.confirm_stopped("test-1", {"stopped": True, "observed_at": old})
        fresh = self.now.isoformat()
        self.now += timedelta(seconds=1)
        self.ledger.cancel("test-1", lambda: None)
        self.ledger.confirm_stopped("test-1", {"stopped": True, "observed_at": fresh})
        self.assertEqual(self.ledger.get("test-1")["state"], "cancelled")

    def test_managed_client_never_falls_back_to_legacy_panel(self):
        client = ManagedPanelExecutor(ROUTES, box_marker=16)
        with patch.object(client, "_call", return_value={}) as call:
            with self.assertRaisesRegex(ValueError, "cancellation unavailable"):
                client.prepare()
            call.assert_called_once_with("GET", "/api/control/capabilities", timeout=3)

    def test_start_reply_identity_state_and_http_outcomes(self):
        client = ManagedPanelExecutor(ROUTES, box_marker=16)
        task = {
            "command_id": "test-1",
            "expires_at": self.request["expires_at"],
            "slot_id": "2b",
            "operation": "INBOUND",
        }
        for state in ("pending", "running"):
            with patch.object(
                client, "_call", return_value={"command_id": "test-1", "state": state}
            ):
                client.start(task)
        for state in ("cancelled", "expired", "completed", "failed"):
            with patch.object(
                client, "_call", return_value={"command_id": "test-1", "state": state}
            ):
                with self.assertRaises(PanelStartRejected):
                    client.start(task)
        for reply in (
            {"command_id": "wrong", "state": "running"},
            {"command_id": "test-1", "state": "unknown"},
            [],
            {},
        ):
            with patch.object(client, "_call", return_value=reply):
                with self.assertRaises(ValueError) as error:
                    client.start(task)
                self.assertNotIsInstance(error.exception, PanelStartRejected)
        for status in (400, 409, 503):
            with patch.object(
                client, "_call", side_effect=PanelHTTPError(status, "test error")
            ):
                with self.assertRaises(
                    PanelStartRejected if status < 500 else PanelHTTPError
                ):
                    client.start(task)

    def test_capability_is_checked_once_after_success(self):
        client = ManagedPanelExecutor(ROUTES, box_marker=16)
        with (
            patch.object(
                client, "_call", return_value={"command_cancel": "durable"}
            ) as call,
            patch("roport_omx.field_panel.PanelExecutor.prepare", return_value={}),
        ):
            client.prepare()
            client.prepare()
            call.assert_called_once_with("GET", "/api/control/capabilities", timeout=3)

    def test_local_begin_cannot_run_after_cancel_confirmation(self):
        entered, release, stopping = (
            threading.Event(),
            threading.Event(),
            threading.Event(),
        )
        order = []
        self.ledger.submit(self.request)

        def begin(payload):
            entered.set()
            if not release.wait(1):
                raise TimeoutError("test begin gate")
            order.append("created")

        starter = threading.Thread(target=lambda: self.ledger.start("test-1", begin))

        def cancel():
            stopping.set()
            self.ledger.cancel("test-1", lambda: order.append("stop"))

        stopper = threading.Thread(target=cancel)
        starter.start()
        self.assertTrue(entered.wait(1))
        stopper.start()
        self.assertTrue(stopping.wait(1))
        try:
            stopper.join(0.02)
            self.assertTrue(stopper.is_alive())
            self.assertEqual(order, [])
        finally:
            release.set()
            starter.join(1)
            stopper.join(1)
        self.assertEqual(order, ["created", "stop"])
        self.ledger.confirm_stopped(
            "test-1", {"observed_at": self.now.isoformat(), "stopped": True}
        )
        self.assertEqual(self.ledger.get("test-1")["state"], "cancelled")

    def test_real_http_client_uses_command_identity_and_durable_cancel(self):
        ledger = self.ledger
        api = PanelCommandAPI(
            ledger,
            validate_payload=lambda payload: None,
            begin=self.starts.append,
            stop=lambda: self.stops.append(1),
        )

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, response):
                status, result = response
                raw = json.dumps(result).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                self.reply(api.handle("GET", self.path))

            def do_POST(self):
                body = json.loads(
                    self.rfile.read(int(self.headers.get("Content-Length", "0")))
                )
                self.reply(api.handle("POST", self.path, body))

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            client = ManagedPanelExecutor(
                ROUTES, box_marker=16, base_url=f"http://127.0.0.1:{server.server_port}"
            )
            task = {
                "command_id": "test-1",
                "expires_at": self.request["expires_at"],
                "slot_id": "2b",
                "operation": "INBOUND",
            }
            client.start(task)
            self.assertEqual(client.observe()["state"], "running")
            client.cancel()
            self.assertFalse(client.cancellation_status()["settled"])
            ledger.confirm_stopped(
                "test-1",
                {"observed_at": self.now.isoformat(), "stopped": True},
            )
            self.assertEqual(
                client.cancellation_status(),
                {"settled": True, "error": None, "local_release_only": True},
            )
            # Delayed duplicate POST cannot resurrect the cancelled command.
            with self.assertRaisesRegex(ValueError, "panel_command_cancelled"):
                client.start(task)
            ledger.start("test-1", self.starts.append)
            self.assertEqual(len(self.starts), 1)
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
