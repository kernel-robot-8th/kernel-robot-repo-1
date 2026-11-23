"""Fake device/Host checks: no LeRobot import, serial open, torque or GPU."""

import sys
import tempfile
import unittest
import hashlib
import json
from copy import deepcopy
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from act_test_support import MockClock, synthetic_profile
from roport_omx.act_support import (
    Rejected,
)
from roport_omx.lerobot_runtime import (
    DeviceLock,
    LeRobotDatasetWriter,
    LeRobotDevices,
    OMX_FEATURES,
    record,
    reconcile_devices,
)


class FakeWriter:
    def __init__(self):
        self.frames, self.aborted, self.finished = [], False, False

    def add_frame(self, frame):
        self.frames.append(frame)

    def finish(self):
        self.finished = True
        return {"complete": True, "frames": len(self.frames)}

    def abort(self):
        self.aborted = True


class FakeDevices:
    def __init__(self, clock, profile):
        self.clock, self.profile = clock, profile
        self.started, self.stops, self.actions = [], [], []
        self.position = [0, 0, 0, 0, 0, 50]
        self.seq, self.inactive_unsafe = 0, False
        self.omit_inactive, self.fail_start, self.confirm_stop = False, False, True

    def start(self, profile, operation):
        self.started.append(operation)
        if self.fail_start:
            raise RuntimeError("partial startup failure")

    def capture(self):
        self.clock.advance()
        self.seq += 1
        arms = {
            arm: {
                "positions": list(self.position)
                if arm == "omx1"
                else [0, 0, 0, 0, 0, 50],
                "observation_seq": self.seq,
                "captured_at": self.clock.stamp(),
            }
            for arm in ("omx1", "omx2")
        }
        if self.inactive_unsafe:
            arms["omx2"]["positions"][0] = 10
        if self.omit_inactive:
            del arms["omx2"]
        return {
            "arms": arms,
            "observation": dict(
                zip(self.profile["host_profile"]["feature_names"], self.position)
            ),
        }

    def read_action(self):
        return dict(
            zip(self.profile["host_profile"]["feature_names"], [1, 0, 0, 0, 0, 50])
        )

    def send_action(self, action):
        actual = {**action, self.profile["host_profile"]["feature_names"][0]: 0.5}
        self.actions.append(actual)
        return actual

    def stop(self, reason):
        self.stops.append(reason)
        return {
            "confirmed": self.confirm_stop,
            "supported": self.confirm_stop,
            "manual_recovery_required": not self.confirm_stop,
            "evidence": "fake support sensor",
        }


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.clock, self.writer = MockClock(), FakeWriter()
        self.profile = {
            "device_mode": "fake",
            "authorization": {"supervisor": True},
            "host_profile": synthetic_profile(),
            "operation": "INBOUND",
            "arm_id": "omx1",
            "fps": 10000,
            "max_frames": 2,
            "shared_lock_path": str(self.root / "devices.lock"),
            "writer_factory": lambda request, profile: self.writer,
            "wall_clock": self.clock.wall,
        }
        self.devices = FakeDevices(self.clock, self.profile)

    def request(self, operation="record"):
        return {
            "schema_version": "omx.workflow/1",
            "request_id": "fake-job",
            "profile_id": "fake",
            "operation": operation,
            "parameters": {},
        }

    def test_local_act_blocks_off_path_target_before_sending(self):
        from roport_omx.lerobot_runtime import evaluate_local_act

        reference = self.root / "reference.json"
        reference.write_text(json.dumps([
            {"state": [0, 0, 0, 0, 0, 50], "action": [1, 0, 0, 0, 0, 50]},
            {"state": [2, 0, 0, 0, 0, 50], "action": [3, 0, 0, 0, 0, 50]},
        ]))
        self.profile.update(
            act_checkpoint=str(self.root / "checkpoint"),
            shelf_level=1,
            camera_mapping={},
            supervise_local_act=lambda _: "run",
            demonstration_guard={
                "path": str(reference), "state_tolerance": [0.5] * 6,
                "action_tolerance": [0.5] * 6, "lookahead_frames": 2,
            },
        )
        with patch("roport_omx.panel_act.PanelAct") as factory:
            factory.return_value.select_action.return_value = SimpleNamespace(tolist=lambda: [3, 0, 0, 0, 0, 50])
            result = evaluate_local_act(self.request("evaluate"), self.profile, self.devices)
        self.assertEqual(self.devices.actions, [])
        self.assertIn("leaves the reviewed demonstration path", result["error"])
        self.assertTrue(result["physical_stop"]["confirmed"])

    def test_record_saves_actual_clipped_action(self):
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "SUCCEEDED")
        self.assertTrue(self.writer.finished)
        frame = self.writer.frames[0]
        key = self.profile["host_profile"]["feature_names"][0]
        self.assertEqual(frame["requested_action"][key], 1)
        self.assertEqual(frame["actual_action"][key], 0.5)
        self.assertIsInstance(frame["action_ns"], int)

    def test_missing_authorization_or_writer_never_starts(self):
        for key in ("authorization", "writer_factory"):
            profile = {k: v for k, v in self.profile.items() if k != key}
            with self.assertRaises(Rejected):
                record(self.request(), profile, self.devices)
        self.assertFalse(self.devices.started)

    def test_untrusted_request_parameters_never_start(self):
        request = self.request()
        request["parameters"] = {"port": "/dev/forbidden"}
        with self.assertRaises(Rejected):
            record(request, self.profile, self.devices)
        self.assertFalse(self.devices.started)

    def test_shared_lock_contention_and_release(self):
        lock = DeviceLock(self.profile["shared_lock_path"]).acquire()
        try:
            with self.assertRaisesRegex(Rejected, "lock"):
                record(self.request(), self.profile, self.devices)
            self.assertFalse(self.devices.started)
        finally:
            lock.release()
        self.assertEqual(
            record(self.request(), self.profile, self.devices)["state"], "SUCCEEDED"
        )

    def test_crash_marker_requires_reconciliation_after_lock_disappears(self):
        lock = DeviceLock(self.profile["shared_lock_path"]).acquire()
        lock.mark_unresolved()
        lock.release()
        with self.assertRaisesRegex(Rejected, "reconciliation"):
            record(self.request(), self.profile, self.devices)
        self.assertFalse(self.devices.started)
        self.assertTrue(reconcile_devices(self.profile, self.devices)["confirmed"])
        self.assertEqual(
            record(self.request(), self.profile, self.devices)["state"], "SUCCEEDED"
        )

    def test_partial_start_failure_calls_physical_stop(self):
        self.devices.fail_start = True
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertTrue(result["physical_stop"]["confirmed"])
        self.assertEqual(len(self.devices.stops), 1)
        self.assertTrue(self.writer.aborted)

    def test_writer_configuration_failure_precedes_device_start(self):
        def unavailable(request, profile):
            raise ValueError("missing field dataset configuration")

        self.profile["writer_factory"] = unavailable
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertFalse(self.devices.started)
        self.assertFalse(self.devices.stops)

    def test_stale_measured_observation_blocks_recording_motion(self):
        original = self.devices.capture

        def stale():
            sample = original()
            sample["arms"]["omx2"]["captured_at"] = self.clock.stamp(-3)
            return sample

        self.devices.capture = stale
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertFalse(self.devices.actions)

    def test_a_slow_leader_read_that_outlives_the_observation_blocks_motion(self):
        original = self.devices.read_action

        def slow():
            # The measured sample is fresh when taken but older than observation_max_age by the write.
            self.clock.advance(
                self.profile["host_profile"]["timing"]["observation_max_age"] + 1
            )
            return original()

        self.devices.read_action = slow
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertFalse(self.devices.actions)

    def test_missing_current_camera_blocks_recording_motion(self):
        self.profile["camera_mapping"] = {"observation.images.side": "side"}
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertFalse(self.devices.actions)

    def test_stop_holding_item_keeps_lock_until_physical_recovery(self):
        self.devices.confirm_stop = False
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "RECONCILING")
        self.assertFalse(result["physical_stop"]["confirmed"])
        with self.assertRaises(Rejected):
            DeviceLock(self.profile["shared_lock_path"]).acquire()
        self.devices.confirm_stop = True
        self.assertTrue(reconcile_devices(self.profile, self.devices)["confirmed"])
        DeviceLock(self.profile["shared_lock_path"]).acquire().release()

    def test_cancellation_aborts_incomplete_dataset(self):
        stop = Event()
        self.profile["stop_event"] = stop
        original = self.writer.add_frame

        def add(frame):
            original(frame)
            stop.set()

        self.writer.add_frame = add
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertTrue(self.writer.aborted)
        self.assertFalse(self.writer.finished)
        self.assertEqual(len(self.devices.actions), 1)

    def test_inactive_arm_missing_or_unsafe_blocks_send(self):
        for field in ("omit_inactive", "inactive_unsafe"):
            setattr(self.devices, field, True)
            result = record(self.request(), self.profile, self.devices)
            self.assertEqual(result["state"], "FAILED")
            self.assertFalse(self.devices.actions)
            setattr(self.devices, field, False)

    def test_dataset_finalization_failure_is_not_success(self):
        self.writer.finish = lambda: {"complete": False}
        result = record(self.request(), self.profile, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertTrue(self.writer.aborted)

    def test_lazy_real_adapter_rejects_without_importing_lerobot(self):
        adapter = LeRobotDevices()
        with patch.dict(sys.modules, {"lerobot": None}), self.assertRaises(Rejected):
            adapter.start(self.profile, "record")
        self.assertEqual(adapter.arms, {})

    def test_physical_stop_does_not_release_ownership_with_open_serial(self):
        adapter = LeRobotDevices()
        adapter.profile = {
            "physical_stop": lambda devices, reason: {
                "confirmed": True,
                "supported": True,
                "manual_recovery_required": False,
                "evidence": "fake physical stop",
            }
        }
        bus = SimpleNamespace(is_connected=True)
        adapter.arms = {"omx1": SimpleNamespace(bus=bus, cameras={})}
        self.assertFalse(adapter.stop("fake stop")["confirmed"])
        bus.is_connected = False
        self.assertTrue(adapter.stop("fake stop")["confirmed"])

    def test_evaluation_hold_releases_bus_without_claiming_physical_stop(self):
        from roport_omx.lerobot_runtime import reconcile_evaluation

        adapter = LeRobotDevices()
        adapter.profile = {"arm_id": "omx1", "shared_lock_path": str(self.root / "arm.lock")}
        adapter.watchdog_ticks = 25
        adapter.evaluation_armed = True
        hold = {"joint": 123}
        bus = Mock()
        bus.motors = {"joint": object()}
        bus.is_connected = True
        bus.sync_read.side_effect = [{"joint": 25}, hold, hold]
        bus.disconnect.side_effect = lambda **kwargs: setattr(bus, "is_connected", False)
        adapter.arms = {"omx1": SimpleNamespace(bus=bus, cameras={})}
        stop = adapter.stop("panel request")
        bus.sync_write.assert_called_once_with("Goal_Position", hold, normalize=False)
        bus.disconnect.assert_called_once_with(disable_torque=False)
        self.assertFalse(stop["confirmed"])
        self.assertTrue(stop["ownership_released"])
        self.assertTrue(stop["evidence"]["hold_requested"])

        marker = DeviceLock(adapter.profile["shared_lock_path"]).acquire()
        marker.mark_unresolved()
        marker.release()
        job = {"result": {"physical_stop": {"evidence": stop}}}
        with self.assertRaises(Rejected):
            reconcile_evaluation(adapter.profile, job, None)
        self.assertTrue(reconcile_evaluation(adapter.profile, job, {"stopped_and_supported": True})["confirmed"])
        self.assertEqual(Path(adapter.profile["shared_lock_path"]).read_text(), "")

    def hardware_profile(self):
        hp = deepcopy(self.profile["host_profile"])
        hp.pop("synthetic_fixture")
        hp["feature_names"] = list(OMX_FEATURES)
        configs, files = {}, {}
        for name in ("omx1", "omx2", "leader"):
            path = self.root / f"{name}.json"
            path.write_text(json.dumps({"verified": "fake-file"}))
            configs[name] = SimpleNamespace(
                calibration_dir=self.root,
                id=name,
                port=f"FAKE-{name}",
                use_degrees=False,
                max_relative_target=5,
                cameras={"side": SimpleNamespace(height=2, width=3, fps=10000)},
            )
            files[name] = {
                "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "calibration_version": hp["calibration_version"],
            }
        return {
            **self.profile,
            "device_mode": "hardware",
            "host_profile": hp,
            "authorization": {"supervisor": True, "hardware": True},
            "follower_configs": {key: configs[key] for key in ("omx1", "omx2")},
            "leader_config": configs["leader"],
            "calibration_files": files,
            "physical_stop": lambda devices, reason: {"confirmed": True},
            "startup_verify": lambda devices: True,
            "capture_metadata": lambda sample: sample,
            "camera_mapping": {"observation.images.side": "side"},
            "dataset_profile": {
                "cameras": {
                    "observation.images.side": {"shape": [2, 3, 3], "fps": 10000}
                }
            },
        }

    def test_lazy_pinned_adapter_calls_only_injected_fake_api(self):
        calls = []

        class FakeFollower:
            def __init__(self, config):
                self.config = config
                self.bus = self
                self.motors = {"joint": 0, "gripper": 0}
                self.writes = []
                self.is_calibrated = True
                self.cameras = {}
                self.torque_enabled = False
                self.goal = {"joint": 0, "gripper": 0}

            def connect(self, *, calibrate=None):
                calls.append((self.config.id, calibrate))

            def get_observation(self):
                return dict(zip(OMX_FEATURES, [0, 0, 0, 0, 0, 50]))

            def sync_read(self, register, *, normalize=True):
                if normalize:
                    assert register == "Present_Position"
                    return dict(zip([key.removesuffix(".pos") for key in OMX_FEATURES], [0, 0, 0, 0, 0, 50]))
                return {
                    "Bus_Watchdog": dict(self.motors),
                    "Torque_Enable": dict.fromkeys(self.motors, int(self.torque_enabled)),
                    "Operating_Mode": {"joint": 4, "gripper": 5},
                    "Present_Position": {"joint": 123, "gripper": 456},
                    "Goal_Position": self.goal,
                }[register]

            def sync_write(self, register, values, *, normalize=True):
                assert register == "Goal_Position"
                if normalize:
                    assert self.torque_enabled
                else:
                    assert not self.torque_enabled
                    self.goal = values

            def enable_torque(self):
                assert self.goal == {"joint": 123, "gripper": 456}
                self.torque_enabled = True

            def write(self, register, motor, ticks, *, normalize):
                assert register == "Bus_Watchdog" and normalize is False
                self.writes.append(ticks)
                self.motors[motor] = ticks

            def send_action(self, action):
                return {**action, "gripper.pos": 49}

        class FakeLeader(FakeFollower):
            def get_action(self):
                return self.get_observation()

        modules = {
            "lerobot.robots.omx_follower.omx_follower": SimpleNamespace(
                OmxFollower=FakeFollower
            ),
            "lerobot.teleoperators.omx_leader.omx_leader": SimpleNamespace(
                OmxLeader=FakeLeader
            ),
        }
        profile = self.hardware_profile()
        adapter = LeRobotDevices()
        with patch.dict(sys.modules, modules):
            adapter.start(profile, "record")
            sample = adapter.capture()
            action = adapter.send_action(adapter.read_action())
        self.assertEqual(calls, [("omx1", False), ("omx2", False), ("leader", False)])
        self.assertEqual(set(sample["arms"]), {"omx1", "omx2"})
        self.assertEqual(action["gripper.pos"], 49)
        self.assertTrue(all(not follower.writes for follower in adapter.arms.values()))

        adapter = LeRobotDevices()
        with patch.dict(sys.modules, modules):
            with self.assertRaisesRegex(Rejected, "bus_watchdog_ticks"):
                adapter.start(profile, "evaluate")
            adapter.start(dict(profile, bus_watchdog_ticks=50), "evaluate")
        for follower in adapter.arms.values():
            self.assertEqual(follower.writes, [])
            self.assertFalse(follower.torque_enabled)
        bus = adapter.arms["omx1"].bus
        with patch.object(bus, "sync_write", return_value=None):
            with self.assertRaisesRegex(Rejected, "initial hold position readback"):
                adapter.send_action(action)
        self.assertFalse(bus.torque_enabled)
        adapter.send_action(action)
        self.assertTrue(adapter.arms["omx1"].torque_enabled)
        self.assertFalse(adapter.arms["omx2"].torque_enabled)
        adapter.arms["omx1"].motors["gripper"] = 255
        with self.assertRaisesRegex(Rejected, "Watchdog tripped"):
            adapter.send_action(action)
        self.assertEqual(adapter.arms["omx1"].writes, [50, 50, 50, 50])

    def test_bad_calibration_and_missing_safety_callback_precede_adapter_import(self):
        profile = self.hardware_profile()
        profile["calibration_files"]["omx1"]["sha256"] = "wrong"
        with (
            patch.dict(sys.modules, {"lerobot": None}),
            self.assertRaisesRegex(Rejected, "calibration"),
        ):
            LeRobotDevices().start(profile, "record")

        profile = self.hardware_profile()
        del profile["capture_metadata"]
        with (
            patch.dict(sys.modules, {"lerobot": None}),
            self.assertRaisesRegex(Rejected, "capture_metadata"),
        ):
            LeRobotDevices().start(profile, "record")

    def test_camera_and_driver_limit_mismatch_precede_adapter_import(self):
        for mismatch in ("camera", "limit", "degree"):
            profile = self.hardware_profile()
            if mismatch == "camera":
                profile["dataset_profile"]["cameras"]["observation.images.side"][
                    "fps"
                ] = 30
            elif mismatch == "limit":
                profile["follower_configs"]["omx1"].max_relative_target = 6
            else:
                profile["host_profile"]["use_degrees"] = True
                profile["host_profile"]["feature_units"] = ["degrees"] * 5 + [
                    "range_0_100"
                ]
            with (
                self.subTest(mismatch=mismatch),
                patch.dict(sys.modules, {"lerobot": None}),
                self.assertRaises(Rejected),
            ):
                LeRobotDevices().start(profile, "record")

    def test_actual_dataset_writer_uses_pinned_save_finalize_and_inspection_api(self):
        calls, payloads = [], []

        class Dataset:
            @classmethod
            def create(cls, **kwargs):
                calls.append("create")
                Path(kwargs["root"]).mkdir()
                return cls()

            def add_frame(self, payload):
                payloads.append(payload)

            def save_episode(self, *, parallel_encoding):
                self.parallel = parallel_encoding
                calls.append("save_episode")

            def finalize(self):
                calls.append("finalize")

        profile = {
            **self.profile,
            "dataset_path": str(self.root / "dataset"),
            "dataset_profile": {"explicit": "fake"},
            "repo_id": "fake/dataset",
            "dataset_features": {},
            "task": "fake task",
            "camera_mapping": {"observation.images.side": "side"},
            "annotate_frame": lambda frame: {"action_ns": frame["action_ns"]},
            "manifest_factory": lambda dataset, frames: {
                "complete": True,
                "frames": frames,
            },
        }
        modules = {
            "lerobot.datasets.lerobot_dataset": SimpleNamespace(LeRobotDataset=Dataset),
            "numpy": SimpleNamespace(
                asarray=lambda data, dtype: data, float32="float32"
            ),
            "roport_omx.learning": SimpleNamespace(
                inspect_dataset=lambda root, expected: {"complete": True}
            ),
        }
        with patch.dict(sys.modules, modules):
            writer = LeRobotDatasetWriter(profile)
            sample = self.devices.capture()
            sample["observation"]["side"] = "fake-rgb-array"
            actual = self.devices.read_action()
            actual[self.profile["host_profile"]["feature_names"][0]] = 0.5
            writer.add_frame(
                {"sample": sample, "actual_action": actual, "action_ns": 123}
            )
            self.assertTrue(writer.finish()["complete"])
        self.assertEqual(calls, ["create", "save_episode", "finalize"])
        self.assertEqual(payloads[0]["action"][0], 0.5)
        self.assertEqual(payloads[0]["observation.images.side"], "fake-rgb-array")


if __name__ == "__main__":
    unittest.main()
