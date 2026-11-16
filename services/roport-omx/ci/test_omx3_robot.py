"""OMX 3 로봇 어댑터 검사: 가짜 lerobot 모듈로 장비 없이(표준 라이브러리만)."""

import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class FakeBus:
    def __init__(self):
        self.present = {"shoulder_pan": 1.0, "gripper": 58.0}
        self.writes, self.is_connected, self.closed_torque = [], False, []

    def connect(self):
        if self.is_connected:
            raise RuntimeError("already connected")  # vendor check_if_already_connected
        self.is_connected = True

    def disconnect(self, disable_torque=True):
        self.closed_torque.append(disable_torque)
        self.is_connected = False

    def sync_read(self, name):
        assert name == "Present_Position"
        return dict(self.present)

    def sync_write(self, name, goal):
        self.writes.append((name, goal))


class FakeCamera:
    def __init__(self):
        self.is_connected, self.thread, self.fail = False, None, False

    def connect(self):
        if self.fail:
            raise ConnectionError("camera open failed")
        self.is_connected, self.thread = True, object()

    def disconnect(self):
        self.is_connected, self.thread = False, None


class FakeOmxFollower:
    """vendored OmxFollower와 같은 순서: 버스 연결 → 카메라 연결, 둘 다 열려야 is_connected."""

    calibrated = True

    def __init__(self, config):
        self.config, self.bus, self.actions = config, FakeBus(), []
        self.cameras = {"front": FakeCamera()}
        self.connect_args = None
        self.calibration_fpath = "cal.json"

    @property
    def is_connected(self):
        return self.bus.is_connected and all(
            c.is_connected for c in self.cameras.values()
        )

    @property
    def is_calibrated(self):
        return self.calibrated

    def connect(self, calibrate=True):
        self.connect_args = calibrate
        self.bus.connect()
        for cam in self.cameras.values():
            cam.connect()

    def get_observation(self):
        obs = {f"{j}.pos": float(i) for i, j in enumerate(JOINTS)}
        return {**obs, "front": "frame", "wrist": "ignored"}

    def send_action(self, action):
        self.actions.append(action)
        return action


def _install_fake_lerobot():
    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    module("lerobot")
    module("lerobot.cameras")
    module("lerobot.cameras.opencv", OpenCVCameraConfig=lambda **kw: kw)
    module("lerobot.robots")
    module(
        "lerobot.robots.omx_follower",
        OmxFollower=FakeOmxFollower,
        OmxFollowerConfig=lambda **kw: kw,
    )


_saved = {k: v for k, v in sys.modules.items() if k.startswith("lerobot")}
_install_fake_lerobot()
from roport_omx.omx3.adapters.lerobot_robot import LeRobotOmx  # noqa: E402
from roport_omx.omx3.settings import JOINTS  # noqa: E402

for _k in [k for k in sys.modules if k.startswith("lerobot")]:
    del sys.modules[_k]
sys.modules.update(_saved)


class Config(unittest.TestCase):
    def test_matches_training_setup(self):
        r = LeRobotOmx("/dev/omx_follower_1", "/dev/v4l/front")
        cfg = r.robot.config
        self.assertEqual(cfg["id"], "omx_follower_arm")
        self.assertIsNone(cfg["calibration_dir"])  # LeRobot 기본 폴더(패널과 같은 파일)
        self.assertFalse(cfg["disable_torque_on_disconnect"])
        cam = cfg["cameras"]["front"]
        self.assertEqual(
            (cam["width"], cam["height"], cam["fps"], cam["fourcc"]),
            (640, 480, 30, "MJPG"),
        )
        self.assertEqual(list(cfg["cameras"]), ["front"])


class Connect(unittest.TestCase):
    def test_connect_never_calibrates(self):
        r = LeRobotOmx("p", 0)
        self.assertEqual(r.ready(), "not connected")
        r.connect()
        self.assertIs(r.robot.connect_args, False)
        self.assertIsNone(r.ready())

    def test_calibration_mismatch_refuses(self):
        r = LeRobotOmx("p", 0)
        r.robot.calibrated = False
        with self.assertRaises(RuntimeError):
            r.connect()
        self.assertEqual(r.ready(), "not connected")
        self.assertFalse(r.robot.bus.is_connected)
        self.assertFalse(r.robot.cameras["front"].is_connected)

    def test_camera_failure_closes_bus_and_allows_reconnect(self):
        # 모터 버스가 열린 뒤 카메라가 실패: 버스도 닫아야 다음 연결이 막히지 않는다(#91 리뷰)
        r = LeRobotOmx("p", 0)
        cam = r.robot.cameras["front"]
        cam.fail = True
        with self.assertRaises(ConnectionError):
            r.connect()
        self.assertFalse(r.robot.bus.is_connected)
        self.assertEqual(r.robot.bus.closed_torque, [False])  # 토크는 끄지 않는다
        cam.fail = False
        r.connect()
        self.assertIsNone(r.ready())

    def test_disconnect_keeps_torque(self):
        r = LeRobotOmx("p", 0)
        r.connect()
        r.disconnect()
        self.assertEqual(r.ready(), "not connected")
        self.assertEqual(r.robot.bus.closed_torque, [False])
        r.disconnect()  # 이미 닫혀도 예외 없음


class Motion(unittest.TestCase):
    def setUp(self):
        self.r = LeRobotOmx("p", 0)
        self.r.connect()

    def test_observe_in_joint_order(self):
        obs = self.r.observe()
        self.assertEqual(list(obs.state), [0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
        self.assertEqual(obs.front, "frame")

    def test_send_maps_joints_and_checks_length(self):
        self.r.send([10, 11, 12, 13, 14, 15])
        self.assertEqual(
            self.r.robot.actions[-1],
            {f"{j}.pos": float(10 + i) for i, j in enumerate(JOINTS)},
        )
        with self.assertRaises(ValueError):
            self.r.send([1, 2, 3])

    def test_hold_keeps_last_gripper_goal(self):
        # 물품에 막혀 58에 멈춘 그리퍼: 현재 위치(58)가 아니라 마지막 목표(45)를 유지해 쥔 힘을 지킨다
        self.r.hold()
        self.assertEqual(self.r.robot.bus.writes[-1][1]["gripper"], 58.0)  # 보낸 적 없음
        self.r.send([0, 0, 0, 0, 0, 45])
        self.r.hold()
        name, goal = self.r.robot.bus.writes[-1]
        self.assertEqual(name, "Goal_Position")
        self.assertEqual(goal, {"shoulder_pan": 1.0, "gripper": 45.0})


if __name__ == "__main__":
    unittest.main()
