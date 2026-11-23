"""shelf_direct: 양팔 UDP 추론 클라이언트·async 정책·출력 필터. 장치·GPU·cv2 없이."""

import os
import stat
import sys
import tempfile
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.infer_link.client import InferenceError, InferenceTimeout
from roport_omx.shelf_direct.contract import CAMERAS, STATE_KEYS, WIDTH, join, split
from roport_omx.shelf_direct.infer import BiUdpInferClient, read_key
from roport_omx.shelf_direct.policy import ActionFilter, BiChunkPolicy
from roport_omx.shelf_direct.ports import Observation, PolicyUnavailable

IMAGES = {cam: f"rgb-{cam}" for cam in CAMERAS}


def fake_encoder(rgb, size, quality):
    return f"{rgb}|{size}|{quality}".encode()


class FakeLink:
    """infer_link InferenceClient 대신: 요청을 기록하고 정해진 응답을 돌려준다."""

    def __init__(self, replies=None, info=None):
        self.replies = list(replies or [])
        self.info = info or {"type": "act", "n_obs": 1, "image_size": None}
        self.calls = []

    def load(self, model, timeout=180.0):
        self.loaded = model
        return dict(self.info)

    def infer(self, header, data, *, timeout, context=None):
        self.calls.append((header, data, timeout, context))
        reply = self.replies.pop(0) if self.replies else {"actions": [[0.0] * WIDTH]}
        if isinstance(reply, BaseException):
            raise reply
        return reply

    def close(self):
        pass


def client(link, **kw):
    return BiUdpInferClient(
        ("127.0.0.1", 1), b"", "model-a", client=link, encoder=fake_encoder, **kw
    )


class ContractTest(unittest.TestCase):
    def test_order_is_left_then_right_motor_order(self):
        self.assertEqual(STATE_KEYS[0], "left_shoulder_pan.pos")
        self.assertEqual(STATE_KEYS[5], "left_gripper.pos")
        self.assertEqual(STATE_KEYS[6], "right_shoulder_pan.pos")
        self.assertEqual(STATE_KEYS[11], "right_gripper.pos")
        values = list(range(12))
        self.assertEqual(join(split(values)), values)
        with self.assertRaises(ValueError):
            split([0.0] * 6)


class InferClientTest(unittest.TestCase):
    def test_request_has_twelve_states_and_four_jpegs(self):
        link = FakeLink(replies=[{"actions": [[1.0] * WIDTH] * 3}])
        c = client(link)
        c.load()
        c.context = {"command_id": "cmd-1"}
        rows = c.infer([0.5] * WIDTH, IMAGES, "INBOUND")
        self.assertEqual(rows, [[1.0] * WIDTH] * 3)
        header, data, timeout, context = link.calls[0]
        self.assertEqual(header["robot_type"], "bi_omx_follower")
        self.assertEqual(header["chunk"], 100)
        self.assertEqual(header["task"], "INBOUND")
        self.assertEqual(timeout, 1.5)
        self.assertEqual(context, {"command_id": "cmd-1"})
        obs = header["obs"][0]
        self.assertEqual(obs["state"], [0.5] * WIDTH)
        self.assertEqual(
            sorted(obs["images"]), sorted(f"observation.images.{c}" for c in CAMERAS)
        )
        for cam in CAMERAS:  # 각 영상의 [offset, len]이 합친 데이터 안을 가리킨다
            off, n = obs["images"][f"observation.images.{cam}"]
            self.assertEqual(data[off : off + n], f"rgb-{cam}|None|90".encode())

    def test_image_size_from_load_is_used(self):
        link = FakeLink(info={"n_obs": 1, "image_size": [240, 320]})
        c = client(link)
        c.load()
        c.infer([0.0] * WIDTH, IMAGES, "t")
        _, data, _, _ = link.calls[0]
        self.assertIn(b"(240, 320)", data)

    def test_refuses_before_load_and_multi_obs_models(self):
        c = client(FakeLink())
        with self.assertRaises(RuntimeError):
            c.infer([0.0] * WIDTH, IMAGES, "t")
        with self.assertRaises(ValueError):
            client(FakeLink(info={"n_obs": 2})).load()

    def test_missing_camera_or_wrong_state_is_refused(self):
        c = client(FakeLink())
        c.load()
        with self.assertRaises(ValueError):
            c.infer([0.0] * 6, IMAGES, "t")
        with self.assertRaises(ValueError):
            c.infer([0.0] * WIDTH, {"front": "x"}, "t")

    def test_resends_on_timeout_then_gives_up(self):
        link = FakeLink(replies=[InferenceTimeout("x"), {"actions": [[2.0] * WIDTH]}])
        c = client(link)
        c.load()
        self.assertEqual(c.infer([0.0] * WIDTH, IMAGES, "t"), [[2.0] * WIDTH])
        self.assertEqual((len(link.calls), c.timeouts), (2, 1))

        link = FakeLink(replies=[InferenceTimeout("x")] * 3)
        c = client(link)
        c.load()
        with self.assertRaises(PolicyUnavailable):
            c.infer([0.0] * WIDTH, IMAGES, "t")
        self.assertEqual(len(link.calls), 3)

    def test_server_error_and_bad_rows_are_unavailable(self):
        for reply in (
            InferenceError("MODEL_MISMATCH"),
            {"actions": [[0.0] * 6]},
            {"actions": []},
            {"actions": [[float("nan")] * WIDTH]},
        ):
            c = client(FakeLink(replies=[reply]))
            c.load()
            with self.assertRaises(PolicyUnavailable, msg=str(reply)):
                c.infer([0.0] * WIDTH, IMAGES, "t")

    def test_key_file_must_be_owner_only(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "k"
            path.write_text("00" * 32)
            os.chmod(path, 0o644)
            with self.assertRaises(ValueError):
                read_key(path)
            os.chmod(path, stat.S_IRUSR | stat.S_IWUSR)
            self.assertEqual(read_key(path), bytes(32))


class GatedClient:
    """infer()가 gate가 열릴 때까지 기다린다. 묶음 i번째 행 = [base + i] * 12."""

    def __init__(self, chunk=10):
        self.chunk, self.calls, self.gates = chunk, [], []
        self.fail = None

    def infer(self, state, images, task):
        gate = threading.Event()
        self.calls.append((tuple(state), task))
        self.gates.append(gate)
        gate.wait(5)
        if self.fail:
            raise self.fail
        base = 100 * len(self.calls)
        return [[float(base + i)] * WIDTH for i in range(self.chunk)]


def obs(v=0.0):
    return Observation(state=(v,) * WIDTH, images=IMAGES)


class ChunkPolicyTest(unittest.TestCase):
    def run_until(self, policy, gc, n):
        gc.gates[n - 1].set()
        policy.wait_idle(5)

    def test_waits_without_advancing_then_runs_the_chunk_from_its_start(self):
        gc = GatedClient(chunk=10)
        p = BiChunkPolicy(gc, actions_per_chunk=10, threshold=0.3)
        p.new_episode("task-a")
        self.assertIsNone(p.act(obs()))  # 첫 추론 시작
        self.assertIsNone(p.act(obs()))  # 기다리는 동안 단계가 늘지 않는다
        self.assertEqual(len(gc.calls), 1)
        self.run_until(p, gc, 1)
        got = [p.act(obs())[0] for _ in range(7)]
        self.assertEqual(got, [100.0 + i for i in range(7)])  # 묶음 처음부터
        self.assertEqual(gc.calls[0][1], "task-a")

    def test_refill_below_threshold_drops_steps_already_run_latest_only(self):
        gc = GatedClient(chunk=10)
        p = BiChunkPolicy(gc, actions_per_chunk=10, threshold=0.3)
        p.new_episode("t")
        p.act(obs())
        self.run_until(p, gc, 1)
        for _ in range(7):  # 남은 3개 = 0.3 x 10 → 다음 act에서 재요청
            p.act(obs())
        self.assertEqual(p.act(obs())[0], 107.0)  # 이 단계(7)에서 두 번째 추론 시작
        self.assertEqual(len(gc.calls), 2)
        self.assertEqual(p.act(obs())[0], 108.0)  # 추론 중에는 남은 동작 실행
        self.run_until(p, gc, 2)
        # 두 번째 묶음은 단계 7부터: 7·8은 이미 실행, 9부터 새 값으로 교체(latest_only)
        self.assertEqual(p.act(obs())[0], 202.0)

    def test_filter_and_new_episode(self):
        gc = GatedClient(chunk=4)
        f = ActionFilter(0.5)
        p = BiChunkPolicy(gc, actions_per_chunk=4, threshold=0.0, action_filter=f)
        p.new_episode("t")
        p.act(obs())
        self.run_until(p, gc, 1)
        self.assertEqual(p.act(obs())[0], 100.0)  # 첫 목표는 그대로
        self.assertEqual(p.act(obs())[0], 100.5)
        p.new_episode("t2")  # 큐·필터 초기화, 이전 묶음은 버린다
        self.assertIsNone(p.act(obs()))
        self.run_until(p, gc, 2)
        self.assertEqual(p.act(obs())[0], 200.0)

    def test_inference_failure_surfaces_on_next_act(self):
        gc = GatedClient()
        gc.fail = PolicyUnavailable("no reply")
        p = BiChunkPolicy(gc, actions_per_chunk=10)
        p.new_episode("t")
        p.act(obs())
        self.run_until(p, gc, 1)
        with self.assertRaises(PolicyUnavailable):
            p.act(obs())


class FilterTest(unittest.TestCase):
    def test_gripper_alpha_defaults_to_alpha_like_the_panel(self):
        f = ActionFilter(0.25)
        f([0.0] * WIDTH)
        y = f([100.0] * WIDTH)
        self.assertEqual(y, [25.0] * WIDTH)

    def test_gripper_alpha_one_leaves_grippers_unfiltered(self):
        f = ActionFilter(0.25, gripper_alpha=1.0)
        f([0.0] * WIDTH)
        y = f([100.0] * WIDTH)
        self.assertEqual(y[5], 100.0)
        self.assertEqual(y[11], 100.0)
        self.assertEqual(y[0], 25.0)

    def test_invalid_alpha(self):
        for a in (0, -1, 1.5):
            with self.assertRaises(ValueError):
                ActionFilter(a)


if __name__ == "__main__":
    unittest.main()
