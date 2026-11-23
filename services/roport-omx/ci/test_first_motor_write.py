"""The first motor write after enabling torque is re-checked against the arm's current position."""

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from act_test_support import synthetic_profile

from roport_omx.act_support import Rejected
from roport_omx.lerobot_runtime import LeRobotDevices
from roport_omx.local_act_profile import OMX_FEATURES

KEYS = [name.removesuffix(".pos") for name in OMX_FEATURES]
START = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]


def reading(values):
    return dict(zip(KEYS, values, strict=True))


def action(values):
    return dict(zip(OMX_FEATURES, values, strict=True))


def adapter(reads):
    """An armed-for-evaluation adapter whose follower bus returns `reads` for Present_Position."""
    hp = synthetic_profile()
    hp["feature_names"] = list(OMX_FEATURES)
    bus = Mock(motors={key: object() for key in KEYS})
    bus.sync_read.side_effect = [reading(r) for r in reads]
    devices = LeRobotDevices()
    devices.profile = {"host_profile": hp, "arm_id": "omx1"}
    devices.arms = {"omx1": Mock(bus=bus)}
    devices.watchdog_ticks = 10
    devices.evaluation_armed = False
    return devices, bus


def goal_writes(bus):
    return [c for c in bus.sync_write.call_args_list if c.args[0] == "Goal_Position"]


class FirstMotorWriteTests(unittest.TestCase):
    def send(self, devices, values):
        with (
            patch.object(devices, "_verify_watchdog"),
            patch.object(devices, "_check_evaluation_cameras"),
            patch("roport_omx.lerobot_runtime._seed_evaluation_hold"),
        ):
            return devices.send_action(action(values))

    def test_an_arm_that_moved_while_torque_was_off_is_not_driven(self):
        devices, bus = adapter([START, [20.0, 0.0, 0.0, 0.0, 0.0, 50.0]])
        with self.assertRaises(Rejected):
            self.send(devices, [3.0, 0.0, 0.0, 0.0, 0.0, 50.0])
        self.assertEqual(goal_writes(bus), [])

    def test_an_arm_that_did_not_move_is_driven_once(self):
        devices, bus = adapter([START, START])
        self.send(devices, [3.0, 0.0, 0.0, 0.0, 0.0, 50.0])
        self.assertEqual(len(goal_writes(bus)), 1)
        self.assertEqual(devices.applied_actions, 1)

    def test_later_actions_do_not_read_the_position_a_second_time(self):
        devices, bus = adapter([START, START, START])
        self.send(devices, [3.0, 0.0, 0.0, 0.0, 0.0, 50.0])
        reads_after_first = bus.sync_read.call_count
        self.send(devices, [4.0, 0.0, 0.0, 0.0, 0.0, 50.0])
        self.assertEqual(bus.sync_read.call_count - reads_after_first, 1)
        self.assertEqual(len(goal_writes(bus)), 2)


if __name__ == "__main__":
    unittest.main()
