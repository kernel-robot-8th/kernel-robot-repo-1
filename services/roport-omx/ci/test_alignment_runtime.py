"""Validate the alignment-to-model handoff and motor recovery with fake devices."""

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_lerobot_runtime import FakeDevices

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from act_test_support import MockClock, synthetic_profile

from roport_omx.act_support import Rejected
from roport_omx.lerobot_runtime import (
    OMX_FEATURES,
    DeviceLock,
    _recover_alignment_velocity,
    evaluate_local_act,
    reconcile_evaluation,
)


class AlignmentRuntimeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.clock = MockClock()
        self.event = Event()
        self.event.continue_model = Event()
        self.event.phase_guard = RLock()
        hp = synthetic_profile()
        hp.pop("synthetic_fixture")
        hp["feature_names"] = list(OMX_FEATURES)
        self.pose = [0, 0, 0, 0, 0, 50]
        self.start = [0.2, 0, 0, 0, 0, 50]
        reference = self.root / "reference.json"
        reference.write_text(json.dumps([{"state": self.pose, "action": self.pose}]))
        self.profile = {
            "host_profile": hp,
            "device_mode": "hardware",
            "authorization": {"supervisor": True, "hardware": True},
            "operation": "INBOUND",
            "arm_id": "omx1",
            "fps": 20,
            "max_frames": 5,
            "bus_watchdog_ticks": 25,
            "shared_lock_path": str(self.root / "devices.lock"),
            "wall_clock": self.clock.wall,
            "stop_event": self.event,
            "act_checkpoint": str(self.root / "checkpoint"),
            "act_policy": Mock(),
            "shelf_level": 1,
            "camera_mapping": {},
            "demonstration_guard": {
                "path": str(reference),
                "state_tolerance": [0.1] * 6,
                "action_tolerance": [0.1] * 6,
                "lookahead_frames": 1,
            },
            "start_alignment": {
                "start": self.start,
                "target": self.pose,
                "start_tolerance": [0.05] * 6,
                "tracking_tolerance": [0.1] * 6,
                "max_rate": [2] * 6,
                "timeout_s": 0.35,
                "settled_samples": 2,
            },
            "alignment_start_report": Mock(return_value={"ready": True}),
        }
        self.devices = FakeDevices(self.clock, self.profile)
        self.devices.position = self.start
        self.devices.begin_alignment = Mock()
        self.devices.check_alignment_health = Mock()
        self.devices.finish_alignment = Mock()
        self.devices.send_alignment = Mock(side_effect=self.apply_alignment)
        self.profile["supervise_local_act"] = lambda sample: (
            "complete" if self.devices.actions else "run"
        )
        self.profile[
            "act_policy"
        ].select_action.return_value.tolist.return_value = self.pose

    def apply_alignment(self, action):
        self.assertEqual(self.event.alignment_phase, "ALIGNING")
        self.devices.position = [action[name] for name in OMX_FEATURES]

    def evaluate(self, *, continue_model=False, change_while_waiting=False):
        waiting_samples = 0

        def wait(seconds):
            nonlocal waiting_samples
            self.clock.advance(seconds)
            if self.event.alignment_phase == "WAITING_MODEL_START":
                self.profile["act_policy"].select_action.assert_not_called()
                self.devices.finish_alignment.assert_not_called()
                waiting_samples += 1
                if change_while_waiting:
                    self.devices.position = [0.15, *self.pose[1:]]
                if continue_model and (change_while_waiting or waiting_samples >= 2):
                    self.event.continue_model.set()
            return self.event.is_set()

        request = {
            "schema_version": "omx.workflow/1",
            "request_id": "test-alignment",
            "profile_id": "omx1",
            "operation": "evaluate",
            "parameters": {"prepare": True},
        }
        with (
            patch("roport_omx.lerobot_runtime.time.monotonic", self.clock.monotonic),
            patch.object(self.event, "wait", side_effect=wait),
        ):
            result = evaluate_local_act(request, self.profile, self.devices)
        self.assertNotIn("_start_alignment", self.profile)
        return result

    def test_explicit_start_reuses_device_session(self):
        result = self.evaluate(continue_model=True)
        self.assertEqual(result["state"], "SUCCEEDED", result)
        self.assertEqual(self.devices.started, ["evaluate"])
        self.assertEqual(self.event.alignment_phase, "MODEL_RUNNING")
        self.assertGreaterEqual(self.devices.send_alignment.call_count, 2)
        self.devices.finish_alignment.assert_called_once_with()
        self.profile["act_policy"].select_action.assert_called_once()

    def test_refused_device_lock_leaves_no_alignment_phase(self):
        Path(self.profile["shared_lock_path"]).write_text('{"state":"RECONCILING"}')
        with self.assertRaises(Rejected):
            self.evaluate(continue_model=True)
        self.assertFalse(hasattr(self.event, "alignment_phase"))

    def test_alignment_setup_io_time_is_not_a_measurement_gap(self):
        # begin_alignment reads registers and writes velocity; that time is not a missed cycle.
        self.profile["start_alignment"]["timeout_s"] = 5
        self.devices.begin_alignment.side_effect = lambda: self.clock.advance(1.0)
        result = self.evaluate(continue_model=True)
        self.assertEqual(result["state"], "SUCCEEDED", result)

    def test_waiting_times_out_without_model_calls(self):
        result = self.evaluate()
        self.assertEqual(result["state"], "FAILED", result)
        self.assertIn("timed out", result["error"])
        self.profile["act_policy"].select_action.assert_not_called()

    def test_waiting_stops_if_pose_changes_even_with_continue_request(self):
        result = self.evaluate(continue_model=True, change_while_waiting=True)
        self.assertEqual(result["state"], "FAILED", result)
        self.assertIn("drifted", result["error"])
        self.profile["act_policy"].select_action.assert_not_called()

    def test_pending_velocity_requires_verified_torque_off_before_reconcile(self):
        names = [feature.removesuffix(".pos") for feature in OMX_FEATURES]
        original = dict.fromkeys(names, 12)
        off = dict.fromkeys(names, 0)
        on = dict.fromkeys(names, 1)
        path = self.root / "omx1.json"
        path.write_text("{}")
        self.profile["follower_configs"] = {
            "omx1": SimpleNamespace(calibration_dir=path.parent, id="omx1")
        }
        self.profile["calibration_files"] = {
            "omx1": {
                "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        }
        bus = Mock(is_calibrated=True, is_connected=True, motors=off)
        module = SimpleNamespace(
            OmxFollower=Mock(return_value=SimpleNamespace(bus=bus))
        )
        job = {
            "result": {
                "physical_stop": {
                    "evidence": {
                        "ownership_released": True,
                        "evidence": {
                            "alignment_velocity_restore_pending": True,
                            "alignment_original_profile_velocity": original,
                        },
                    }
                }
            }
        }
        lock_path = Path(self.profile["shared_lock_path"])
        lock = DeviceLock(lock_path).acquire()
        lock.mark_unresolved()
        lock.release()
        with patch.dict(
            sys.modules, {"lerobot.robots.omx_follower.omx_follower": module}
        ):
            bus.sync_read.side_effect = [off, on]
            with self.assertRaisesRegex(Rejected, "motor torque is off"):
                reconcile_evaluation(self.profile, job, {"stopped_and_supported": True})
            bus.sync_write.assert_not_called()
            self.assertTrue(lock_path.read_text())
            with self.assertRaises(Rejected):
                _recover_alignment_velocity(self.profile, {"invalid": -1})

            bus.sync_read.side_effect = [off, off, original]
            reconcile_evaluation(self.profile, job, {"stopped_and_supported": True})
            bus.sync_write.assert_called_once_with(
                "Profile_Velocity", original, normalize=False
            )
            bus.enable_torque.assert_not_called()
            self.assertEqual(lock_path.read_text(), "")
