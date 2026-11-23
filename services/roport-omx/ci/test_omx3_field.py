"""OMX 3 field factory 검사: 가짜 어댑터 모듈로 장비·GPU·cv2·lerobot 없이."""

import json
import os
import sys
import tempfile
import time
import types
import unittest
from datetime import UTC, datetime, timedelta
from pathlib import Path
from threading import Event
from typing import ClassVar
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts"), str(ROOT / "ci")]

import test_field_host as tfh
import test_omx3_runner as tfh_runner

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

    def test_hold_drops_samples_before_stop_request(self):
        # #149 리뷰 재현: 정지 요청 전 0.6 s 정지 표본 + 요청 0.01 s 뒤 표본 1개는 정지가 아니다
        robot, clock = FakeRobot(), Clock()
        watch = field.StopWatch(robot, clock, clock.wall)
        for _ in range(7):
            watch.observe()
            clock.t += 0.1
        self.assertTrue(watch.status()["stopped"])
        watch.hold()
        clock.t += 0.01
        watch.observe()
        self.assertFalse(watch.status()["stopped"])
        for _ in range(5):
            clock.t += 0.1
            watch.observe()
        self.assertTrue(watch.status()["stopped"])  # hold 뒤 0.5 s 모이면 정지

    def test_hold_failure_still_drops_samples(self):
        robot = FakeRobot()
        robot.hold = mock.Mock(side_effect=OSError("bus"))
        watch = watch_samples([INIT] * 7)
        watch.robot = robot
        with self.assertRaises(OSError):
            watch.hold()
        self.assertEqual(watch.status()["observed_at"], None)

    def test_samples_between_stop_request_and_hold_are_dropped(self):
        # #149 리뷰: abort 뒤·hold 전에 끝난 관측이 이전 정지 표본과 이어지지 않는다
        robot, clock = FakeRobot(), Clock()
        watch = field.StopWatch(robot, clock, clock.wall)
        for _ in range(7):
            watch.observe()
            clock.t += 0.1
        watch.invalidate()
        for _ in range(7):
            watch.observe()  # Runner가 아직 hold 전
            clock.t += 0.1
        self.assertEqual(watch.status()["observed_at"], None)
        watch.hold()
        for _ in range(6):
            watch.observe()
            clock.t += 0.1
        self.assertTrue(watch.status()["stopped"])


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

    def disconnect(self):
        EVENTS.append("disconnect")

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
        self.sizes = Path(tmp.name) / "sizes.json"
        self.sizes.write_text(json.dumps(SIZES))
        EVENTS.clear()
        FakeLeRobotOmx.fail_next = False
        model = "/srv/il/checkpoints/me/marker_act_x/pretrained_model"
        env = {
            "ROPORT_OMX3_INFER_SERVER": "192.0.2.10:5600",
            "ROPORT_OMX3_MODEL": model,
            "ROPORT_OMX3_PACKAGE_SIZES": str(self.sizes),
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
        self.assertEqual(executor.package_sizes, SIZES)
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

    def test_package_sizes_must_cover_trained_markers_before_connect(self):
        for sizes in (
            {k: v for k, v in SIZES.items() if k != "22"},  # 빠진 상자
            {**SIZES, "22": "unknown"},  # 관제가 배정하지 못하는 크기
            {**SIZES, "18": "small"},  # 학습하지 않은 마커
        ):
            self.sizes.write_text(json.dumps(sizes))
            with self.assertRaises(ValueError):
                field.build()
        self.assertEqual(EVENTS, [])  # GPU·팔 모두 건드리기 전에 거절

    def test_model_version_without_pretrained_suffix(self):
        self.assertEqual(field.model_version("/a/b/model_x"), "model_x")


class SlowRobot(FakeRobot):
    """관측 한 번에 시간이 걸리는 팔: 종료가 Runner의 hold를 기다리는지 본다."""

    delay = 0.02
    slow = None  # 이 Event가 있으면 느린 관측이 시작될 때 알린다

    def observe(self):
        delay = self.delay
        if self.slow is not None and delay > 0.1:
            self.slow.set()
        time.sleep(delay)
        return super().observe()

    def send(self, action):
        super().send(action)
        self.state = list(action)

    def disconnect(self):
        self.calls.append("disconnect")


class FakeMarker:
    def __init__(self, marker_id):
        pass

    def reset(self):
        pass

    def find(self, frame):
        return (100.0, 300.0)  # 쟁반 밖: 완료되지 않고 정책이 계속 돈다

    def mark(self, frame):
        return frame, (100.0, 300.0)


class FakePolicy:
    def reset(self, marker_id):
        pass

    def act(self, state, frame):
        return [INIT[0] + 20, *INIT[1:]]  # init을 벗어난 채 머문다: 종료 전까지 실행 중


class CloseTest(unittest.TestCase):
    def make(self):
        robot, client = SlowRobot(), mock.Mock()
        watch = field.StopWatch(robot)
        runner = field.Runner(watch, FakePolicy(), FakeMarker)
        executor = field.FieldWarehouseExecutor(
            runner,
            devices=(client, robot),
            marker_ids={"15": 15},
            model_version="m",
            calibration_version="c",
            package_sizes={"15": "small"},
            watch=watch,
        )
        return executor, robot, client

    def test_close_during_run_waits_for_hold_then_disconnects(self):
        executor, robot, client = self.make()
        expires = (datetime.now(UTC) + timedelta(seconds=60)).isoformat()
        executor.start({"command_id": "c1", "package_id": "15", "expires_at": expires})
        wait_for(lambda: any(c[0] == "send" for c in robot.calls if c != "hold"))
        executor.close()
        self.assertIsNone(executor.host.status()["active"])
        self.assertIn("hold", robot.calls)
        self.assertEqual(robot.calls[-1], "disconnect")
        self.assertLess(robot.calls.index("hold"), robot.calls.index("disconnect"))
        client.close.assert_called_once()
        self.assertTrue(executor.observe()["stopped"])  # 정지 근거를 확인한 뒤 정리

    def test_close_keeps_ownership_until_runner_stops(self):
        # #149 리뷰: Runner가 늦어도 close()가 먼저 반환해 hold 없이 프로세스가 끝나지 않게
        executor, robot, client = self.make()
        expires = (datetime.now(UTC) + timedelta(seconds=60)).isoformat()
        executor.start({"command_id": "c1", "package_id": "15", "expires_at": expires})
        wait_for(lambda: any(c[0] == "send" for c in robot.calls if c != "hold"))
        robot.slow = Event()
        robot.delay = 0.8  # 취소 때 진행 중인 관측이 로그 간격보다 오래 걸린다
        robot.slow.wait(2)
        with (
            mock.patch.object(field, "CLOSE_LOG_S", 0.1),
            self.assertLogs("roport_omx.omx3.field", level="ERROR"),
        ):
            executor.close()
        self.assertIsNone(executor.host.status()["active"])
        self.assertLess(robot.calls.index("hold"), robot.calls.index("disconnect"))
        client.close.assert_called_once()

    def test_idle_close_holds_and_disconnects(self):
        executor, robot, _ = self.make()
        executor.close()
        self.assertEqual(robot.calls[0], "hold")
        self.assertEqual(robot.calls[-1], "disconnect")
        self.assertTrue(executor.observe()["stopped"])


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition not reached")
        time.sleep(0.01)


class FieldHostIntegrationTest(unittest.TestCase):
    """실제 FieldOmxHost + Runner + StopWatch(#149 리뷰 재현 경로). 팔·카메라는 가짜."""

    observe_pinky = tfh.FieldHostTest.observe
    command = tfh.FieldHostTest.command
    stop_command = tfh.FieldHostTest.stop_command

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.journal = tfh.Journal(Path(temp.name) / "journal.json")
        self.addCleanup(self.journal.close)

    def connect(self, robot, policy, where, clock=None):
        watch = field.StopWatch(robot)
        markers = lambda marker_id: tfh_runner.FakeMarker(where)
        settings = tfh_runner.S
        if clock is None:
            runner = field.Runner(watch, policy, markers, settings)
        else:
            runner = field.Runner(watch, policy, markers, settings, clock, clock.sleep)
        executor = field.FieldWarehouseExecutor(
            runner,
            watch=watch,
            marker_ids={"package-17": 17},
            package_sizes={"package-17": "large"},
            model_version="sim-model",
            calibration_version="sim-cal",
        )
        executors = tfh.virtual_executors(0)
        executors["omx3"] = executor
        self.host = tfh.FieldOmxHost(self.journal, executors)
        self.observe_pinky("N01")
        # 접수 전 정지 근거: 상태 조회(_arms)의 관측이 0.5 s 넘게 모여야 ready
        wait_for(lambda: executor.readiness()["arms"]["omx3"]["stopped"])
        return executor

    def accept(self, target="omx3"):
        events = self.host.handle(self.command(target))
        self.assertEqual(events[0]["payload"]["status"], "accepted", events)

    def poll_until(self, status, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.observe_pinky("N01")
            for event in self.host.poll():
                if event["payload"]["status"] == status:
                    return event
            time.sleep(0.01)
        raise AssertionError(f"no {status} event")

    def test_normal_load_completes_after_fresh_stop_samples(self):
        robot, policy = tfh_runner.FakeRobot(), tfh_runner.ScriptPolicy(away=4)
        where = tfh_runner.box_until(robot, 4)
        executor = self.connect(robot, policy, where, tfh_runner.Clock())
        self.accept()
        executor.host.wait(2)
        # VERIFY 직후에는 정지 표본이 없다: 실패하지 않고 완료를 보류한다
        self.assertEqual(executor.observe(), {"state": "running", "step": "VERIFY"})
        done = self.poll_until("completed")
        self.assertEqual(done["payload"]["result"]["package_size"], "large")
        self.assertTrue(done["payload"]["result"]["arms"][0]["stopped"])

    def test_unconfirmed_stop_after_verify_fails_separately(self):
        robot, policy = tfh_runner.FakeRobot(), tfh_runner.ScriptPolicy(away=4)
        where = tfh_runner.box_until(robot, 4)
        executor = self.connect(robot, policy, where, tfh_runner.Clock())
        self.accept()
        executor.host.wait(2)
        with mock.patch.object(field, "CONFIRM_S", 0.0):
            executor.observe()
            time.sleep(0.01)
            self.assertEqual(
                executor.observe(), {"state": "failed", "error": field.STOP_UNCONFIRMED}
            )

    def test_stop_during_run_completes_only_after_hold(self):
        robot = SlowFakeRobot()
        policy = tfh_runner.ScriptPolicy(
            away=10**6
        )  # 정지 요청 전까지 같은 자세로 머문다
        self.connect(robot, policy, lambda frame: tfh_runner.BOX)
        self.accept()
        wait_for(lambda: robot.sent > 0)
        time.sleep(0.7)  # 정지 요청 전 0.5 s 넘는 '정지' 표본이 쌓인다
        self.host.handle(self.stop_command(target="omx3"))
        self.assertEqual(self.host.poll(), [])
        done = self.poll_until("completed")
        self.assertGreater(robot.held, 0)
        self.assertGreaterEqual(done["payload"]["observed_at"], robot.held_at)


class SlowFakeRobot(tfh_runner.FakeRobot):
    held_at = None

    def observe(self):
        time.sleep(0.03)
        return super().observe()

    def hold(self):
        super().hold()
        self.held_at = datetime.now(UTC).isoformat()


SIZES = {"15": "small", "16": "small", "17": "large", "21": "large", "22": "small"}


if __name__ == "__main__":
    unittest.main()
