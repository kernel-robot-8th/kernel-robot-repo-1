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
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from dedicated_demo import MockClock, mock_envelopes, synthetic_profile
from roport_omx.dedicated import (
    PHASES,
    VERDICTS,
    WAITS,
    DedicatedHost,
    Profile,
    Rejected,
)
from roport_omx.lerobot_runtime import (
    DeviceLock,
    LeRobotDatasetWriter,
    LeRobotDevices,
    OMX_FEATURES,
    evaluate,
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

    def host(self):
        host = DedicatedHost(
            Profile(self.profile["host_profile"]),
            self.root / "host.sqlite",
            host_boot_id="mock-host-boot",
            wall_clock=self.clock.wall,
            monotonic_clock=self.clock.monotonic,
        )
        self.addCleanup(host.close)
        return host

    def configure_evaluation(self, host):
        self.profile["max_frames"] = 20
        command, permit = mock_envelopes(self.clock)
        self.profile["execution_context"] = lambda: {
            "command": command,
            "permit": permit,
        }

        def exchange(request):
            if request["type"] == "policy.reset":
                return {**request, "type": "policy.reset.ack"}
            return {
                **request,
                "type": "action.candidate",
                "action_seq": 0,
                "actions": [[1, 0, 0, 0, 0, 50]],
            }

        def supervise(host, sample):
            if host.state in PHASES and len(self.devices.actions) > PHASES.index(
                host.state
            ):
                return {"phase_complete": True}
            if host.state in WAITS:
                index = WAITS.index(host.state)
                evidence = {
                    "execution_id": command["execution_id"],
                    "phase": host.phase,
                    "phase_revision": host.phase_revision,
                    "verdict_kind": VERDICTS[index],
                    "confirmer": "fake-supervisor",
                    "positive": True,
                    "observation_seq": self.devices.seq,
                    "captured_at": self.clock.stamp(),
                }
                if index < 3:
                    evidence["next_policy_id"] = host.profile["policies"]["INBOUND"][
                        PHASES[index + 1]
                    ]["policy_id"]
                return {"evidence": evidence}
            return {}

        self.profile.update(exchange=exchange, supervise=supervise)

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

            def connect(self, *, calibrate):
                calls.append((self.config.id, calibrate))

            def get_observation(self):
                return dict(zip(OMX_FEATURES, [0, 0, 0, 0, 0, 50]))

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

    def test_evaluate_runs_host_candidate_and_next_action_then_safe_return(self):
        host = self.host()
        self.configure_evaluation(host)
        with (
            patch.object(
                host, "validate_candidate", wraps=host.validate_candidate
            ) as validate,
            patch.object(host, "next_action", wraps=host.next_action) as next_action,
        ):
            result = evaluate(
                self.request("evaluate"), self.profile, host, self.devices
            )
        self.assertEqual(result["state"], "SUCCEEDED", result)
        self.assertEqual(host.state, "SAFE_RETURNED")
        self.assertEqual(validate.call_count, 4)
        self.assertEqual(next_action.call_count, 4)
        self.assertEqual(len(result["actions"]), 4)
        self.assertFalse(host.status()["departure_allowed"])

    def test_expired_permit_cannot_send(self):
        host = self.host()
        self.configure_evaluation(host)
        original = self.profile["execution_context"]

        def context():
            result = deepcopy(original())
            self.clock.advance(10)
            return result

        self.profile["execution_context"] = context
        result = evaluate(self.request("evaluate"), self.profile, host, self.devices)
        self.assertEqual(result["state"], "FAILED")
        self.assertFalse(self.devices.actions)
        self.assertEqual(host.state, "HOLD")

    def test_invalid_candidate_and_inference_timeout_hold_without_motion(self):
        for timeout in (False, True):
            with self.subTest(timeout=timeout):
                host = (
                    self.host()
                    if not timeout
                    else DedicatedHost(
                        Profile(self.profile["host_profile"]),
                        self.root / "other.sqlite",
                        host_boot_id="mock-host-boot",
                        wall_clock=self.clock.wall,
                        monotonic_clock=self.clock.monotonic,
                    )
                )
                if timeout:
                    self.addCleanup(host.close)
                self.configure_evaluation(host)
                original = self.profile["exchange"]

                def exchange(request):
                    result = original(request)
                    if request["type"] == "policy.infer":
                        if timeout:
                            self.clock.advance(1.1)
                        else:
                            result["execution_id"] = "stale"
                    return result

                self.profile["exchange"] = exchange
                result = evaluate(
                    self.request("evaluate"), self.profile, host, self.devices
                )
                self.assertEqual(result["state"], "FAILED")
                self.assertEqual(host.state, "HOLD")
                self.assertFalse(self.devices.actions)


if __name__ == "__main__":
    unittest.main()
