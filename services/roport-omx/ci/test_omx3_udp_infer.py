"""OMX 3 UDP 추론 어댑터 검사: 루프백 가짜 GPU 서버 + 가짜 cv2. GPU·cv2·로봇 없이 돈다."""

import os
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.infer_link import wire  # noqa: E402
from roport_omx.infer_link.client import InferenceClient, InferenceTimeout  # noqa: E402
from roport_omx.infer_link.wire import ACTIONS, ERROR, LOADED, Message  # noqa: E402
from roport_omx.omx3.inference import ChunkPolicy  # noqa: E402
from roport_omx.omx3.settings import Omx3Settings  # noqa: E402

KEY = b"k" * 32
MODEL = "/srv/il/checkpoints/sangjin/marker_act/pretrained_model"


class FakeFrame:
    """RGB 영상 대신: 크기와 지금까지 거친 변환만 가진다."""

    def __init__(self, h=480, w=640, steps=()):
        self.shape, self.steps = (h, w, 3), list(steps)


CALLS = []


def _resize(img, size, interpolation):
    CALLS.append(("resize", size, interpolation))
    return FakeFrame(size[1], size[0], [*img.steps, "resize"])


def _cvt(img, code):
    CALLS.append(("cvtColor", code))
    return FakeFrame(*img.shape[:2], [*img.steps, "bgr"])


def _imencode(ext, img, params):
    CALLS.append(("imencode", ext, params))
    body = f"jpeg {img.shape[0]}x{img.shape[1]} {'+'.join(img.steps)}".encode()
    return True, types.SimpleNamespace(tobytes=lambda: body)


FAKE_CV2 = types.SimpleNamespace(
    INTER_AREA=3,
    COLOR_RGB2BGR=4,
    IMWRITE_JPEG_QUALITY=1,
    resize=_resize,
    cvtColor=_cvt,
    imencode=_imencode,
)

import roport_omx.omx3.adapters as _adapters  # noqa: E402

# 가짜 cv2로 어댑터를 따로 불러오고 import 캐시는 원래대로 되돌린다(test_omx3_marker와 같은 방식)
_ISOLATED = ("cv2", f"{_adapters.__name__}.udp_infer")
_saved = {k: sys.modules.pop(k, None) for k in _ISOLATED}
_saved_attr = vars(_adapters).pop("udp_infer", None)
sys.modules["cv2"] = FAKE_CV2
try:
    from roport_omx.omx3.adapters.udp_infer import UdpInferClient, read_key
finally:
    for _k in _ISOLATED:
        sys.modules.pop(_k, None)
    sys.modules.update({k: v for k, v in _saved.items() if v is not None})
    vars(_adapters).pop("udp_infer", None)
    if _saved_attr is not None:
        vars(_adapters)["udp_infer"] = _saved_attr


class FakeGpu:
    """LOAD에는 loaded를, INFER에는 actions를 답한다. 받은 요청(header, data)을 기록한다."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.address = self.sock.getsockname()
        self.reassembler = wire.Reassembler(KEY)
        self.loaded = {"ok": True, "type": "act", "image_size": [480, 640], "n_obs": 1}
        self.actions = [[float(k), 0, 0, 0, 0, 0] for k in range(100)]
        self.load_ms = 0.0  # 이 요청 때 모델을 올린 시간(README 응답 필드)
        self.load_delay = 0.0
        self.silent, self.requests, self.kinds = False, [], []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join()
        self.sock.close()

    def _run(self):
        while not self.stop.is_set():
            try:
                packet, peer = self.sock.recvfrom(65535)
            except TimeoutError:
                continue
            req = self.reassembler.feed(packet, peer)
            if req is None or self.silent:
                continue
            self.kinds.append(req.kind)  # 받은 순서(LOAD·INFER)
            if req.kind == wire.LOAD:
                time.sleep(self.load_delay)  # 모델을 올리는 동안
                kind, reply = LOADED, dict(self.loaded)
            elif req.header.get("model") != MODEL:
                kind, reply = ERROR, {"code": "MODEL_NOT_LOADED"}
            else:
                self.requests.append((req.header, req.data))
                kind = ACTIONS
                reply = {k: req.header.get(k) for k in wire.ECHO_FIELDS}
                reply |= {
                    "actions": self.actions,
                    "model_version": MODEL,
                    "load_ms": self.load_ms,
                }
            out = Message(kind, req.session, req.request_id, reply)
            for datagram in wire.pack(out, KEY):
                self.sock.sendto(datagram, peer)


class Adapter(unittest.TestCase):
    def setUp(self):
        CALLS.clear()
        self.gpu = FakeGpu()
        self.addCleanup(self.gpu.close)
        link = InferenceClient(self.gpu.address, KEY, bind_host="127.0.0.1")
        self.client = UdpInferClient(self.gpu.address, KEY, MODEL, client=link)
        self.addCleanup(self.client.close)

    def test_request_matches_readme_format(self):
        self.client.load()
        actions = self.client.infer(
            [1, 2, 3, 4, 5, 6], FakeFrame(), "pick 17", "omx_follower"
        )
        self.assertEqual(len(actions), 100)
        ((header, data),) = self.gpu.requests
        self.assertEqual(
            {
                k: header[k]
                for k in ("model", "overrides", "task", "robot_type", "chunk", "rtc")
            },
            {
                "model": MODEL,
                "overrides": {},
                "task": "pick 17",
                "robot_type": "omx_follower",
                "chunk": 100,
                "rtc": None,
            },
        )
        (obs,) = header["obs"]  # 관측 1개(ACT n_obs 1)
        self.assertEqual(obs["state"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(obs["images"], {"observation.images.front": [0, len(data)]})
        # 같은 크기면 축소 없이 RGB → BGR 뒤 품질 90 JPEG
        self.assertEqual(data, b"jpeg 480x640 bgr")
        self.assertEqual(CALLS, [("cvtColor", 4), ("imencode", ".jpg", [1, 90])])

    def test_resizes_to_loaded_image_size(self):
        self.gpu.loaded["image_size"] = [240, 320]
        self.client.load()
        self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        ((_, data),) = self.gpu.requests
        self.assertEqual(data, b"jpeg 240x320 resize+bgr")
        self.assertEqual(CALLS[0], ("resize", (320, 240), 3))  # (w, h), INTER_AREA

    def test_without_image_size_sends_frame_as_is(self):
        self.gpu.loaded.pop("image_size")
        self.client.load()
        self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.assertEqual(self.gpu.requests[0][1], b"jpeg 480x640 bgr")

    def test_refuses_infer_before_load_and_multi_obs_models(self):
        with self.assertRaises(RuntimeError):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.assertEqual(self.gpu.requests, [])
        self.gpu.loaded["n_obs"] = 2
        with self.assertRaises(ValueError):
            self.client.load()
        with self.assertRaises(RuntimeError):  # 실패한 load 뒤에도 보내지 않는다
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")

    def test_failed_reload_blocks_infer(self):
        # 성공 → 재로드 실패(n_obs 2, 무응답) → 이전 영상 크기로 보내지 않는다
        self.gpu.loaded["image_size"] = [240, 320]
        self.client.load()
        self.gpu.loaded["n_obs"] = 2
        with self.assertRaises(ValueError):
            self.client.load()
        with self.assertRaises(RuntimeError):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.gpu.loaded["n_obs"] = 1
        self.client.load()
        self.gpu.silent = True
        with self.assertRaises(InferenceTimeout):
            self.client.load(timeout=0.1)
        with self.assertRaises(RuntimeError):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.assertEqual(self.gpu.requests, [])

    def test_rejects_zero_or_missing_obs_count(self):
        for n_obs in (0, None):
            with self.subTest(n_obs=n_obs):
                self.gpu.loaded["n_obs"] = n_obs
                with self.assertRaises(ValueError):
                    self.client.load()

    def test_server_reload_during_run_is_logged(self):
        self.client.load()
        with self.assertNoLogs("roport_omx.omx3.adapters.udp_infer"):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.gpu.load_ms = 850.0  # 캐시에서 밀려 서버가 다시 올림
        with self.assertLogs("roport_omx.omx3.adapters.udp_infer", "WARNING") as logs:
            actions = self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.assertEqual(len(actions), 100)  # 받은 동작은 그대로 쓴다
        self.assertIn("load_ms=850.0", logs.output[0])

    def test_waiting_for_the_link_is_bounded(self):
        # load()가 잠금을 오래 잡고 있어도 infer()는 timeout_s 안에 시간 초과로 끝난다
        self.client.load()
        self.client.timeout_s = 0.1
        self.client._lock.acquire()
        try:
            with self.assertRaises(InferenceTimeout):
                self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        finally:
            self.client._lock.release()
        self.assertEqual(self.gpu.requests, [])

    def _reload_in_background(self):
        failed = []

        def reload():
            try:
                self.client.load()
            except ValueError as exc:
                failed.append(exc)

        loader = threading.Thread(target=reload)
        loader.start()
        return loader, failed

    def test_infer_waiting_behind_a_failed_reload_sends_nothing(self):
        # 성공 로드 → 재로드 중(잠금 잡음) 추론 → 재로드 실패 → 그 추론은 보내지 않는다
        self.client.load()
        self.gpu.loaded["n_obs"], self.gpu.load_delay = 2, 0.3
        self.client.timeout_s = 2.0
        loader, failed = self._reload_in_background()
        time.sleep(0.1)
        with self.assertRaises(RuntimeError):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        loader.join()
        self.assertEqual(len(failed), 1)
        self.assertEqual(self.gpu.requests, [])

    def test_infer_paused_in_encoding_never_sends_after_a_failed_reload(self):
        # #115 리뷰 재현: 추론이 인코딩 중 멈춘 사이 재로드가 시작·실패해도, 실패 뒤에 보내지 않는다
        self.client.load()
        self.gpu.loaded["n_obs"] = 2
        self.client.timeout_s = 2.0
        entered, go = threading.Event(), threading.Event()

        def slow_cvt(img, code):
            entered.set()
            go.wait(2)
            return _cvt(img, code)

        FAKE_CV2.cvtColor = slow_cvt
        self.addCleanup(setattr, FAKE_CV2, "cvtColor", _cvt)
        out = []
        worker = threading.Thread(
            target=lambda: out.append(
                self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
            )
        )
        worker.start()
        self.assertTrue(entered.wait(2))
        loader, failed = self._reload_in_background()
        time.sleep(0.1)  # 재로드는 인코딩이 끝날 때까지 잠금 앞에서 기다린다
        go.set()
        worker.join()
        loader.join()
        self.assertEqual(len(failed), 1)
        # 추론은 재로드 전에 끝났고(그때 상태는 유효), 실패한 재로드 뒤 INFER는 없다
        self.assertEqual(self.gpu.kinds[1:], [wire.INFER, wire.LOAD])
        with self.assertRaises(RuntimeError):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")

    def test_lock_wait_counts_toward_the_request_timeout(self):
        # 잠금을 만료 직전에 얻고 응답이 없어도 전체가 timeout_s 안에 끝난다
        self.client.load()
        self.gpu.silent = True
        self.client.timeout_s = 0.2
        self.client._lock.acquire()
        threading.Timer(0.15, self.client._lock.release).start()
        start = time.monotonic()
        with self.assertRaises(InferenceTimeout):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")
        self.assertLess(time.monotonic() - start, 0.3)

    def test_no_reply_raises_timeout(self):
        self.client.load()
        self.gpu.silent = True
        self.client.timeout_s = 0.1
        with self.assertRaises(InferenceTimeout):
            self.client.infer([0] * 6, FakeFrame(), "t", "omx_follower")

    def test_drives_chunk_policy(self):
        # 7단계 ChunkPolicy에 끼우면 그대로 동작 묶음이 나온다(task 문장·robot_type 포함)
        self.client.load()
        policy = ChunkPolicy(self.client, Omx3Settings())
        policy.reset(17)
        action, deadline = (
            None,
            time.monotonic() + 5,
        )  # 바쁜 CI에서도 흔들리지 않게 상한 5초
        while action is None and time.monotonic() < deadline:
            action = policy.act([0.0] * 6, FakeFrame())
            if action is None:
                time.sleep(0.002)
        self.assertEqual(action, [0.0, 0, 0, 0, 0, 0])
        header = self.gpu.requests[0][0]
        self.assertEqual(
            header["task"], "Pick up the box with marker 17 and put it on the tray"
        )
        self.assertEqual(header["robot_type"], "omx_follower")


class ReadKey(unittest.TestCase):
    def write(self, text, mode):
        fd, path = tempfile.mkstemp()
        self.addCleanup(os.remove, path)
        with os.fdopen(fd, "w") as f:
            f.write(text)
        os.chmod(path, mode)
        return path

    def test_reads_hex_key_owner_only(self):
        self.assertEqual(read_key(self.write("6b" * 32 + "\n", 0o600)), KEY)

    def test_refuses_shared_permissions_and_short_keys(self):
        with self.assertRaises(ValueError):
            read_key(self.write("6b" * 32, 0o644))
        with self.assertRaises(ValueError):
            read_key(self.write("6b" * 8, 0o600))


if __name__ == "__main__":
    unittest.main()
