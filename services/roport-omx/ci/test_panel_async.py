"""Real panel HTTP client with delayed device I/O; no hardware connection."""

import sys
import threading
import time
import unittest
from datetime import UTC, datetime, timedelta
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from test_field_host import wait_until
from test_panel_executor import ROUTES, FakePanel

from roport_omx.field_executors import PanelFieldExecutor
from roport_omx.panel_executor import PanelExecutor, PanelHTTPError
from roport_omx.managed_panel import ManagedPanelExecutor, PanelStartRejected


class AsyncPanelTest(unittest.TestCase):
    def setUp(self):
        self.panel = FakePanel(managed=True)
        self.gates = []
        self.stops = 0
        self.sample = self.stationary()
        self.adapter = PanelFieldExecutor(
            ManagedPanelExecutor(ROUTES, box_marker=16, base_url=self.panel.url),
            model_version="test-model",
            calibration_version="test-cal",
            stop=self.stop,
            stop_status=lambda: self.sample,
        )
        self.task = {
            "robot_ids": ["omx1", "omx2"],
            "operation": "INBOUND",
            "slot_id": "2b",
            "command_id": "test-command",
            "package_id": "test-package",
            "arrival_command_id": "test-arrival",
            "expires_at": (datetime.now(UTC) + timedelta(seconds=30)).isoformat(),
        }

    @staticmethod
    def stationary():
        return {"observed_at": datetime.now(UTC).isoformat(), "stopped": True}

    def stop(self):
        self.stops += 1
        self.panel.status["job"]["running"] = False
        self.sample = self.stationary()

    def tearDown(self):
        self.adapter.close()
        for gate in self.gates:
            gate.set()
        # Drain the started device callbacks before disposing their fake HTTP peer.
        futures = [self.adapter.start_future, self.adapter.stop_future]
        futures += [e["future"] for e in self.adapter.reads.values()]
        for future in futures:
            if future:
                try:
                    future.result(timeout=2)
                except Exception:
                    pass
        self.panel.server.shutdown()
        self.panel.server.server_close()
        self.panel.ledger.close()

    def block(self):
        entered, release = threading.Event(), threading.Event()
        self.gates.append(release)

        def delay():
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test gate was not released")

        return entered, release, delay

    def ready(self):
        wait_until(self.adapter.readiness, lambda r: r.get("armed"))

    def prompt(self, call):
        started = time.monotonic()
        result = call()
        self.assertLess(time.monotonic() - started, 0.2)
        return result

    def test_delayed_preparation_never_blocks_readiness_or_stop(self):
        entered, release, delay = self.block()
        self.panel.before_get = lambda path: (
            delay() if path == "/api/train/runs" else None
        )
        self.assertFalse(self.prompt(self.adapter.readiness)["armed"])
        self.assertTrue(entered.wait(1))
        self.prompt(self.adapter.abort)
        self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.stops, 1)
        self.assertFalse(self.prompt(self.adapter.readiness)["armed"])
        self.assertEqual(self.prompt(self.adapter.observe)["state"], "stopped")
        self.assertFalse(self.panel.starts)
        release.set()

    def test_cancel_confirms_while_start_is_blocked_and_prevents_late_start(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.before_start = delay
        self.adapter.start(self.task)
        self.assertTrue(entered.wait(1))
        self.prompt(self.adapter.abort)
        wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")
        self.assertFalse(self.adapter.start_future.done())
        self.assertFalse(self.panel.starts)
        release.set()
        with self.assertRaisesRegex(PanelStartRejected, "cancelled"):
            self.adapter.start_future.result(timeout=1)
        self.assertFalse(self.panel.starts)
        self.assertEqual(self.stops, 1)
        with self.assertRaisesRegex(ValueError, "held"):
            self.adapter.start(self.task)

    def test_running_device_cancels_before_its_start_http_reply_arrives(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.after_start = delay
        self.adapter.start(self.task)
        self.assertTrue(entered.wait(1))
        self.assertTrue(self.panel.status["job"]["running"])
        self.adapter.abort()
        wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")
        self.assertFalse(self.adapter.start_future.done())
        self.assertFalse(self.panel.status["job"]["running"])
        release.set()
        self.adapter.start_future.result(timeout=1)
        self.assertEqual(self.stops, 1)
        self.assertEqual(len(self.panel.starts), 1)

    def test_lost_start_reply_is_reconciled_without_replaying_start(self):
        self.ready()
        original = self.adapter.panel._call

        def timeout_start(method, path, body=None, **kwargs):
            if method == "POST" and path == "/api/control/commands":
                original(method, path, body, **kwargs)
                raise TimeoutError("response lost")
            return original(method, path, body, **kwargs)

        self.adapter.panel._call = timeout_start
        self.adapter.start(self.task)
        with self.assertRaises(TimeoutError):
            self.adapter.start_future.result(timeout=1)
        self.assertEqual(self.adapter.observe()["error"], "panel_start_unconfirmed")
        self.adapter.abort()
        wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")
        self.assertEqual(len(self.panel.starts), 1)
        self.assertFalse(self.panel.status["job"]["running"])

    def test_stop_callback_delay_does_not_block_owner_or_fake_completion(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)
        entered, release, delay = self.block()
        self.adapter.stop = delay
        self.prompt(self.adapter.abort)
        self.assertTrue(entered.wait(1))
        self.sample = self.stationary()
        self.assertEqual(self.prompt(self.adapter.observe)["state"], "stopping")
        release.set()
        wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")

    def test_queued_start_expires_before_any_post(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.before_get = lambda path: delay() if path == "/api/status" else None
        wait_until(self.adapter.readiness, lambda r: entered.is_set())
        self.task["expires_at"] = (
            datetime.now(UTC) + timedelta(seconds=0.05)
        ).isoformat()
        self.prompt(lambda: self.adapter.start(self.task))
        time.sleep(0.06)
        release.set()
        with self.assertRaisesRegex(ValueError, "expired_command"):
            self.adapter.start_future.result(timeout=1)
        self.assertFalse(self.panel.starts)

    def test_close_cancels_a_start_queued_behind_a_read(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.before_get = lambda path: delay() if path == "/api/status" else None
        wait_until(self.adapter.readiness, lambda r: entered.is_set())
        self.adapter.start(self.task)
        self.prompt(self.adapter.close)
        release.set()
        self.adapter.start_future.result(timeout=1)
        self.assertFalse(self.panel.starts)

    def test_stale_readiness_is_not_reissued_as_a_fresh_motor_sample(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.before_get = lambda path: (
            delay() if path.startswith("/api/arms?") else None
        )
        wait_until(self.adapter.readiness, lambda r: entered.is_set())
        with patch(
            "roport_omx.field_executors.monotonic", return_value=time.monotonic() + 4
        ):
            self.assertFalse(self.prompt(self.adapter.readiness)["armed"])
        release.set()

    def test_delayed_result_read_does_not_block_stop(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)
        entered, release, delay = self.block()
        self.panel.before_get = lambda path: (
            delay() if path.startswith("/api/control/commands/") else None
        )
        self.prompt(self.adapter.observe)
        self.assertTrue(entered.wait(1))
        self.prompt(self.adapter.abort)
        self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.prompt(self.adapter.observe)["state"], "stopping")
        release.set()
        wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")

    def test_stop_failure_cannot_be_replaced_by_stationary_sample(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)

        def fail():
            raise OSError("device stop unavailable")

        self.adapter.stop = fail
        self.adapter.abort()
        with self.assertRaises(OSError):
            self.adapter.stop_future.result(timeout=1)
        self.sample = self.stationary()
        self.assertEqual(self.adapter.observe()["state"], "stopping")

    def test_delayed_managed_cancel_does_not_delay_local_motor_stop(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)
        entered, release, delay = self.block()
        self.adapter.panel.cancel = lambda **kwargs: delay()
        self.adapter.abort()
        self.assertTrue(entered.wait(1))
        self.assertEqual(self.stops, 1)
        self.assertEqual(self.adapter.observe()["state"], "stopping")
        release.set()
        self.adapter.stop_future.result(timeout=1)

    def test_transient_cancel_failure_retries_and_logs(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)
        original = self.adapter.panel.cancel
        calls = []

        def cancel(**kwargs):
            calls.append(kwargs["timeout"])
            if len(calls) == 1:
                raise TimeoutError("first cancel response lost")
            return original(**kwargs)

        self.adapter.panel.cancel = cancel
        with self.assertLogs("roport_omx.field_executors", level="ERROR"):
            self.adapter.abort()
            wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(0 < value <= 1 for value in calls))
        self.assertEqual(self.stops, 1)

    def test_legacy_client_is_rejected_before_connection(self):
        with self.assertRaisesRegex(ValueError, "managed_panel_required"):
            PanelFieldExecutor(
                PanelExecutor(ROUTES, box_marker=16),
                model_version="test",
                calibration_version="test",
            )

    def test_normal_completion_does_not_request_an_extra_stop_on_close(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)
        self.panel.finish()
        wait_until(self.adapter.observe, lambda r: r["state"] == "finished")
        self.adapter.close()
        self.assertEqual(self.stops, 0)
        self.assertIsNone(self.adapter.stop_future)

    def test_definitive_http_refusal_preserves_failure_reason(self):
        self.ready()
        original = self.adapter.panel._call

        def refused(method, path, body=None, **kwargs):
            if method == "POST" and path == "/api/control/commands":
                raise PanelHTTPError(409, "command_conflict")
            return original(method, path, body, **kwargs)

        self.adapter.panel._call = refused
        self.adapter.start(self.task)
        with self.assertRaises(PanelStartRejected):
            self.adapter.start_future.result(timeout=1)
        self.assertFalse(self.adapter.start_unknown)
        self.assertEqual(self.adapter.observe()["error"], "panel_start_rejected_409")
        self.assertFalse(self.panel.starts)

    def test_failed_refresh_discards_previous_cached_sample(self):
        from concurrent.futures import Future

        future = Future()
        future.set_exception(OSError("read failed"))
        self.adapter.reads["ready"] = {
            "future": future,
            "sample": (time.monotonic(), {"armed": True}),
            "since": time.monotonic(),
        }
        with self.assertRaises(OSError):
            self.adapter._sample("ready", lambda: {})
        self.assertIsNone(self.adapter.reads["ready"]["sample"])

    def test_cancel_retry_exhaustion_stays_unconfirmed(self):
        from unittest.mock import Mock

        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)
        self.adapter.panel.cancel = Mock(side_effect=TimeoutError("panel offline"))
        with self.assertLogs("roport_omx.field_executors", level="ERROR"):
            self.adapter.abort()
            with self.assertRaises(TimeoutError):
                self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.adapter.panel.cancel.call_count, 3)
        self.assertEqual(self.adapter.observe()["state"], "stopping")
        self.assertFalse(self.adapter.panel.cancellation_settled())
        # The failure path was already checked; close without another test-only network failure.
        self.adapter.panel.cancel = lambda **kwargs: None

    def test_owner_close_waits_for_an_already_requested_stop(self):
        entered, release, delay = self.block()
        self.adapter.stop = delay
        self.adapter.abort()
        self.assertTrue(entered.wait(1))
        closer = threading.Thread(target=self.adapter.close)
        closer.start()
        closer.join(0.02)
        self.assertTrue(closer.is_alive())
        release.set()
        closer.join(1)
        self.assertFalse(closer.is_alive())
        self.assertTrue(self.adapter.stop_future.done())

    def test_arm_change_after_cached_acceptance_is_rechecked_before_post(self):
        self.ready()
        entered, release, delay = self.block()
        original = self.adapter.panel.check

        def check(task, *, cached=False):
            if not cached:
                delay()
            original(task, cached=cached)

        self.adapter.panel.check = check
        self.adapter.start(self.task)
        self.assertTrue(entered.wait(1))
        self.panel.arms["OMX 2"]["at_home"] = False
        release.set()
        with self.assertRaisesRegex(ValueError, "arm_not_safely_home"):
            self.adapter.start_future.result(timeout=1)
        self.assertFalse(self.panel.starts)
        self.assertEqual(self.adapter.observe()["error"], "arm_not_safely_home")


if __name__ == "__main__":
    unittest.main()
