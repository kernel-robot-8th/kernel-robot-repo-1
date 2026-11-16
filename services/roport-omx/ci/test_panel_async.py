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
from roport_omx.panel_executor import PanelExecutor


class AsyncPanelTest(unittest.TestCase):
    def setUp(self):
        self.panel = FakePanel()
        self.gates = []
        self.stops = 0
        self.sample = self.stationary()
        self.adapter = PanelFieldExecutor(
            PanelExecutor(ROUTES, box_marker=16, base_url=self.panel.url),
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

    def test_abort_while_starting_repeats_stop_after_late_start_reply(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.before_start = delay
        self.prompt(lambda: self.adapter.start(self.task))
        self.assertTrue(entered.wait(1))
        with patch(
            "roport_omx.field_executors.monotonic", return_value=time.monotonic() + 4
        ):
            self.assertIsNone(self.adapter.readiness()["error"])
        self.prompt(self.adapter.abort)
        self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.stops, 1)
        self.assertEqual(self.prompt(self.adapter.observe)["state"], "stopping")
        release.set()
        self.adapter.start_future.result(timeout=1)
        wait_until(self.adapter.observe, lambda r: r["state"] == "stopped")
        self.assertGreaterEqual(self.stops, 2)
        self.assertEqual(len(self.panel.starts), 1)
        self.assertFalse(self.panel.status["job"]["running"])
        with self.assertRaisesRegex(ValueError, "held"):
            self.adapter.start(self.task)

    def test_lost_start_reply_never_certifies_stop_or_retries_remote_start(self):
        self.ready()
        entered, release, delay = self.block()
        self.panel.before_start = delay
        self.adapter.panel.start_timeout = 0.05
        self.adapter.start(self.task)
        self.assertTrue(entered.wait(1))
        with self.assertRaises(TimeoutError):
            self.adapter.start_future.result(timeout=1)
        self.assertEqual(self.adapter.observe()["state"], "failed")
        self.adapter.abort()
        self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.adapter.observe()["state"], "stopping")
        # The old remote API may still execute after the client times out. Never call this safe.
        release.set()
        wait_until(lambda: self.panel.starts, bool)
        self.assertTrue(self.panel.status["job"]["running"])
        self.assertEqual(self.adapter.observe()["state"], "stopping")
        with self.assertRaisesRegex(ValueError, "held"):
            self.adapter.start(self.task)
        self.assertEqual(len(self.panel.starts), 1)

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
        self.panel.before_get = lambda path: delay() if path == "/api/status" else None
        self.prompt(self.adapter.observe)
        self.assertTrue(entered.wait(1))
        self.prompt(self.adapter.abort)
        self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.prompt(self.adapter.observe)["state"], "stopped")
        release.set()

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
        self.adapter.panel.cancel = delay
        self.adapter.abort()
        self.assertTrue(entered.wait(1))
        self.assertEqual(self.stops, 1)
        self.assertEqual(self.adapter.observe()["state"], "stopping")
        release.set()
        self.adapter.stop_future.result(timeout=1)

    def test_managed_cancel_error_still_invokes_local_motor_stop(self):
        self.ready()
        self.adapter.start(self.task)
        self.adapter.start_future.result(timeout=1)

        def fail_cancel():
            raise OSError("panel unavailable")

        self.adapter.panel.cancel = fail_cancel
        self.adapter.abort()
        with self.assertRaises(OSError):
            self.adapter.stop_future.result(timeout=1)
        self.assertEqual(self.stops, 1)
        self.assertEqual(self.adapter.observe()["state"], "stopping")

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


if __name__ == "__main__":
    unittest.main()
