"""OMX 3 field factory 검사: 가짜 어댑터 모듈로 장비·GPU·cv2·lerobot 없이."""

import os
import sys
import tempfile
import types
import unittest
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import ClassVar
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.field_executors import WarehouseExecutor
from roport_omx.omx3 import field
from roport_omx.omx3.ports import Observation

INIT = [0.0, -62.3, 53.9, 42.2, 0.0, 56.5]
T0 = datetime(2026, 11, 23, tzinfo=UTC)


class FakeRobot:
    def __init__(self):
        self.state, self.calls = list(INIT), []

    def ready(self):
        return None

    def observe(self):
        return Observation(list(self.state), object())

    def send(self, action):
        self.calls.append(("send", action))

    def hold(self):
        self.calls.append("hold")


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def wall(self):
        return T0 + timedelta(seconds=self.t)


def watch_samples(states, dt=0.1):
    robot, clock = FakeRobot(), Clock()
    watch = field.StopWatch(robot, clock, clock.wall)
    for state in states:
        robot.state = state
        watch.observe()
        clock.t += dt
    return watch


class StopWatchTest(unittest.TestCase):
    def test_no_samples_is_not_stopped(self):
        self.assertFalse(field.StopWatch(FakeRobot()).status()["stopped"])

    def test_still_for_half_second_is_stopped_at_last_sample_time(self):
        watch = watch_samples([[v + 0.02 * (i % 2) for v in INIT] for i in range(6)])
        self.assertEqual(
            watch.status(),
            {"stopped": True, "observed_at": (T0 + timedelta(seconds=0.5)).isoformat()},
        )

    def test_short_span_is_not_stopped(self):
        self.assertFalse(watch_samples([INIT] * 5).status()["stopped"])  # 0.4 s

    def test_moving_joint_is_not_stopped(self):
        states = [INIT] * 5 + [[*INIT[:5], INIT[5] + 0.31]]
        self.assertFalse(watch_samples(states).status()["stopped"])

    def test_old_motion_leaves_window(self):
        moving = [[v + i for v in INIT] for i in range(5)]
        watch = watch_samples(moving + [INIT] * 12)
        self.assertTrue(watch.status()["stopped"])

    def test_delegates_device_calls(self):
        robot = FakeRobot()
        watch = field.StopWatch(robot)
        watch.send([1] * 6)
        watch.hold()
        self.assertEqual(robot.calls, [("send", [1] * 6), "hold"])
        self.assertIsNone(watch.ready())


class FakeLeRobotOmx(FakeRobot):
    instances: ClassVar[list] = []

    def __init__(self, port, camera):
        super().__init__()
        self.port, self.camera, self.connected = port, camera, False
        self.robot = types.SimpleNamespace(id="omx_follower_arm", calibration_fpath=CAL)
        self.fail = FakeLeRobotOmx.fail_next
        FakeLeRobotOmx.instances.append(self)

    def connect(self):
        EVENTS.append("connect")
        if self.fail:
            raise ConnectionError("no arm")
        self.connected = True

    fail_next = False


class FakeClient:
    instances: ClassVar[list] = []

    def __init__(self, server, key, model):
        self.server, self.key, self.model = server, key, model
        FakeClient.instances.append(self)

    def load(self):
        EVENTS.append("load")
        return {"n_obs": 1}

    def close(self):
        EVENTS.append("close")


EVENTS = []
CAL = None


def fake_modules():
    lr = types.ModuleType("lerobot_robot")
    lr.LeRobotOmx = FakeLeRobotOmx
    mk = types.ModuleType("marker")
    mk.ArucoTargetMarker = lambda marker_id: None
    ui = types.ModuleType("udp_infer")
    ui.UdpInferClient = FakeClient
    ui.read_key = lambda path: b"k" * 32
    base = "roport_omx.omx3.adapters."
    return {base + "lerobot_robot": lr, base + "marker": mk, base + "udp_infer": ui}


class BuildTest(unittest.TestCase):
    def setUp(self):
        global CAL
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        CAL = Path(tmp.name) / "omx_follower_arm.json"
        CAL.write_text('{"shoulder_pan": {}}')
        EVENTS.clear()
        FakeLeRobotOmx.fail_next = False
        model = "/srv/il/checkpoints/me/marker_act_x/pretrained_model"
        env = {
            "ROPORT_OMX3_INFER_SERVER": "192.0.2.10:5600",
            "ROPORT_OMX3_MODEL": model,
        }
        for patch in (
            mock.patch.dict(sys.modules, fake_modules()),
            mock.patch.dict(os.environ, env),
        ):
            patch.start()
            self.addCleanup(patch.stop)

    def test_builds_omx3_executor_without_alignment_sensor(self):
        executors, sensor = field.build()
        self.assertIsNone(sensor)
        self.assertEqual(list(executors), ["omx3"])
        executor = executors["omx3"]
        self.assertIsInstance(executor, WarehouseExecutor)
        self.assertEqual(EVENTS, ["load", "connect"])  # GPU 확인 뒤에만 토크를 쓴다
        self.assertEqual(
            executor.marker_ids, {"15": 15, "16": 16, "17": 17, "21": 21, "22": 22}
        )
        self.assertEqual(executor.versions["model_version"], "marker_act_x")
        self.assertRegex(
            executor.versions["calibration_version"], r"^omx_follower_arm-[0-9a-f]{8}$"
        )
        client, robot = FakeClient.instances[-1], FakeLeRobotOmx.instances[-1]
        self.assertEqual(client.server, ("192.0.2.10", 5600))
        self.assertEqual(robot.port, field.DEFAULT_PORT)
        self.assertIs(executor.runner.robot.robot, robot)
        # 상태 조회 때 읽은 관절 값이 정지 판정에 쓰인다
        clock = executor.runner.robot.clock = Clock()
        self.assertFalse(executor.readiness()["arms"]["omx3"]["stopped"])
        clock.t = 0.6  # 0.5 s 넘게 같은 자세
        self.assertTrue(executor.readiness()["arms"]["omx3"]["stopped"])

    def test_connect_failure_closes_client(self):
        FakeLeRobotOmx.fail_next = True
        with self.assertRaises(ConnectionError):
            field.build()
        self.assertEqual(EVENTS, ["load", "connect", "close"])

    def test_model_version_without_pretrained_suffix(self):
        self.assertEqual(field.model_version("/a/b/model_x"), "model_x")


if __name__ == "__main__":
    unittest.main()
