"""Cover the OMX1 preflight without importing drivers or opening real serial ports."""

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.act_support import Rejected
from roport_omx.lerobot_runtime import DeviceLock, LeRobotDevices, OMX_FEATURES
from roport_omx.local_act_deployment import inspect_start, start_report


class LocalActDeploymentTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.pose = [0, 0, 0, 0, 0, 50]
        reference = root / "reference.json"
        reference.write_text(json.dumps([{"state": self.pose, "action": self.pose}]))
        calibration = {
            feature.removesuffix(".pos"): {
                "id": motor_id,
                "homing_offset": -7,
                "range_min": 0,
                "range_max": 4000,
                "drive_mode": 0,
            }
            for motor_id, feature in enumerate(OMX_FEATURES, start=11)
        }
        path = root / "calibration.json"
        data = json.dumps(calibration).encode()
        path.write_bytes(data)
        self.profile = {
            "arm_id": "omx1",
            "shared_lock_path": str(root / "robot.lock"),
            "follower_configs": {
                name: SimpleNamespace(port=name) for name in ("omx1", "omx2")
            },
            "calibration_files": {
                name: {"path": str(path), "sha256": hashlib.sha256(data).hexdigest()}
                for name in ("omx1", "omx2")
            },
            "demonstration_guard": {
                "path": str(reference),
                "state_tolerance": [1] * 6,
                "action_tolerance": [1] * 6,
                "lookahead_frames": 1,
            },
            "host_profile": {
                "safe_positions": {name: self.pose for name in ("omx1", "omx2")},
                "safe_tolerance": [1] * 6,
            },
        }

    def fake_sdk(self, *, override=None):
        sdk = Mock(COMM_SUCCESS=0)
        ports = {
            name: Mock(
                name=name,
                port_name=name,
                is_open=True,
                openPort=Mock(return_value=True),
                setBaudRate=Mock(return_value=True),
            )
            for name in self.profile["follower_configs"]
        }
        sdk.PortHandler.side_effect = ports.__getitem__
        packet = sdk.PacketHandler.return_value
        packet.ping.side_effect = lambda _, motor: (1060 if motor <= 13 else 1200, 0, 0)
        registers = {10: 0, 20: 2**32 - 7, 48: 4000, 52: 0, 64: 0, 70: 0, 98: 0, 132: 2000}

        def read(port, motor, address):
            if override is not None:
                change = override(port, motor, address)
                if change is not None:
                    return change, 0, 0
            return registers[address], 0, 0

        packet.read1ByteTxRx.side_effect = read
        packet.read4ByteTxRx.side_effect = read
        return sdk, ports

    def assert_read_only_and_released(self, sdk, ports):
        packet = sdk.PacketHandler.return_value
        self.assertEqual(
            {"ping", "read1ByteTxRx", "read4ByteTxRx"},
            {call[0] for call in packet.method_calls},
        )
        for port in ports:
            port.closePort.assert_called_once()
        DeviceLock(self.profile["shared_lock_path"]).acquire().release()

    def test_inspection_normalizes_positions_and_only_reads_motors(self):
        sdk, ports = self.fake_sdk()
        with patch.dict(sys.modules, {"dynamixel_sdk": sdk}):
            report = inspect_start(self.profile)
        self.assertTrue(report["ready"])
        self.assertEqual(tuple(report["current"]), tuple(self.pose))
        self.assertEqual(report["unit"], "lerobot_normalized")
        self.assertEqual(sdk.PacketHandler.return_value.ping.call_count, 12)
        self.assert_read_only_and_released(sdk, ports.values())

    def test_inspection_refuses_a_torque_or_calibration_error_and_releases_ownership(self):
        for address, value, error in ((64, 1, "토크"), (20, 0, "calibration differs")):
            with self.subTest(address=address):
                def override(port, motor, addr):
                    if port.port_name == "omx2" and motor == 11 and addr == address:
                        return value
                    return None

                sdk, ports = self.fake_sdk(override=override)
                with (
                    patch.dict(sys.modules, {"dynamixel_sdk": sdk}),
                    self.assertRaisesRegex(Rejected, error),
                ):
                    inspect_start(self.profile)
                self.assert_read_only_and_released(sdk, ports.values())

    def test_start_report_separates_active_pose_and_inactive_arm_mismatch(self):
        shifted = [2, *self.pose[1:]]
        report = start_report(self.profile, {"omx1": shifted, "omx2": self.pose})
        self.assertFalse(report["ready"])
        self.assertTrue(report["inactive_arm_ready"])
        self.assertEqual(report["delta"], [2, 0, 0, 0, 0, 0])
        report = start_report(self.profile, {"omx1": self.pose, "omx2": shifted})
        self.assertFalse(report["ready"])
        self.assertFalse(report["inactive_arm_ready"])
        self.assertEqual(report["delta"], [0] * 6)
        with self.assertRaises(Rejected):
            start_report(self.profile, {"omx1": self.pose})

    def test_stop_without_arming_releases_after_torque_off_readback(self):
        for torque_off, confirmed in ((True, True), (False, False)):
            with self.subTest(torque_off=torque_off):
                device = LeRobotDevices()
                device.profile = self.profile
                device.watchdog_ticks = 25
                device.startup_torque_off_verified = True
                bus = Mock(motors={"gripper": object()}, is_connected=True)
                bus.sync_read.return_value = {"gripper": 0 if torque_off else 1}
                bus.disconnect.side_effect = lambda **_: setattr(bus, "is_connected", False)
                device.arms = {"omx1": SimpleNamespace(bus=bus, cameras={})}
                result = device.stop("preflight refused")
                self.assertIs(result["confirmed"], confirmed)
                self.assertIs(result["manual_recovery_required"], not confirmed)
                self.assertTrue(result["ownership_released"])
                bus.sync_read.assert_called_once_with("Torque_Enable", normalize=False)
                bus.sync_write.assert_not_called()
                bus.disconnect.assert_called_once_with(disable_torque=False)

    def test_expired_camera_frame_blocks_watchdog_torque_and_goal_writes(self):
        device = LeRobotDevices()
        device.profile = {
            **self.profile,
            "host_profile": {
                "feature_names": OMX_FEATURES,
                "joint_bounds": [[-100, 100]] * 5 + [[0, 100]],
                "max_step": [1] * 6,
                "timing": {"observation_max_age": 0.2},
            },
        }
        device.watchdog_ticks = 25
        device.camera_times = [4.7]
        action = dict(zip(OMX_FEATURES, self.pose, strict=True))
        bus = Mock()
        bus.sync_read.return_value = {
            name.removesuffix(".pos"): value for name, value in action.items()
        }
        device.arms = {"omx1": SimpleNamespace(bus=bus)}
        with (
            patch("roport_omx.lerobot_runtime.time.perf_counter", return_value=5),
            self.assertRaisesRegex(Rejected, "camera frame expired"),
        ):
            device.send_action(action)
        bus.sync_write.assert_not_called()
        bus.write.assert_not_called()
        bus.enable_torque.assert_not_called()


if __name__ == "__main__":
    unittest.main()
