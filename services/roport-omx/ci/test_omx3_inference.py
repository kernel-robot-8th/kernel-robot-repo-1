"""OMX 3 동작 묶음 정책 검사 (가짜 추론 클라이언트, 표준 라이브러리만)."""

import logging
import math
import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.omx3 import inference  # noqa: E402
from roport_omx.omx3.inference import ChunkPolicy  # noqa: E402
from roport_omx.omx3.ports import PolicyUnavailable  # noqa: E402
from roport_omx.omx3.settings import Omx3Settings  # noqa: E402

S = Omx3Settings(infer_timeout_s=0.5, refill_at=3)
inference.RETRY_S = 0.01
logging.getLogger("roport_omx.omx3.inference").setLevel(logging.CRITICAL)


def chunk(start, n=6):
    """동작 번호를 shoulder_pan에 담은 묶음: 행 k = [start + k, 0, 0, 0, 0, 0]."""
    return [[float(start + k), 0, 0, 0, 0, 0] for k in range(n)]


class FakeClient:
    """요청을 기록하고, replies 순서대로 응답한다(함수면 호출, 예외면 던짐)."""

    def __init__(self, *replies):
        self.replies, self.calls = list(replies), []
        self.release = threading.Event()
        self.release.set()

    def infer(self, state, marked_front, task, robot_type):
        self.calls.append((list(state), marked_front, task, robot_type))
        self.release.wait(5)
        reply = self.replies.pop(0) if self.replies else chunk(0)
        if isinstance(reply, Exception):
            raise reply
        return reply(len(self.calls)) if callable(reply) else reply


def wait_calls(client, n):
    for _ in range(500):
        if len(client.calls) >= n:
            return
        time.sleep(0.005)


def pan(action):
    return action[0]


class Consume(unittest.TestCase):
    def test_first_act_waits_then_consumes_chunk_in_order(self):
        client = FakeClient(chunk(0))
        p = ChunkPolicy(client, S)
        p.reset(17)
        self.assertEqual([pan(p.act([1] * 6, "f")) for _ in range(2)], [0.0, 1.0])
        state, frame, task, robot_type = client.calls[0]
        self.assertEqual((state, frame), ([1] * 6, "f"))
        self.assertEqual(task, "Pick up the box with marker 17 and put it on the tray")
        self.assertEqual(robot_type, "omx_follower")

    def test_requests_next_chunk_early_and_drops_executed_steps(self):
        # 묶음 6개, refill_at 3: 3개 남으면 미리 요청. 응답 전에 실행한 스텝 수만큼 앞을 버린다
        client = FakeClient(chunk(0), chunk(100))
        p = ChunkPolicy(client, S)
        p.reset(17)
        got = [pan(p.act([0] * 6, i)) for i in range(3)]  # 0,1,2 → 3개 남음
        client.release.clear()
        got.append(pan(p.act([0] * 6, "at-request")))  # 미리 요청(아직 응답 없음), 3
        wait_calls(client, 2)
        got.append(pan(p.act([0] * 6, "x")))  # 큐의 남은 동작으로 계속, 4
        client.release.set()
        for _ in range(500):  # 응답 반영까지
            with p._cond:
                if not p._pending:
                    break
            time.sleep(0.005)
        got.append(pan(p.act([0] * 6, "y")))
        # 새 묶음 0번 = 요청한 스텝(3), 1번 = 다음 스텝(4). 둘 다 이미 실행했으므로 102부터
        self.assertEqual(got, [0, 1, 2, 3, 4, 102])
        self.assertEqual(client.calls[1][1], "at-request")
        self.assertEqual(len(client.calls), 2)  # 응답을 기다리는 동안 중복 요청 없음


class Failure(unittest.TestCase):
    def test_transient_failure_retries_with_new_request(self):
        client = FakeClient(TimeoutError("lost"), chunk(0))
        p = ChunkPolicy(client, S)
        p.reset(17)
        self.assertEqual(pan(p.act([0] * 6, "f")), 0.0)
        self.assertEqual(len(client.calls), 2)

    def test_bad_reply_is_rejected_and_retried(self):
        for bad in ([], [[1, 2, 3]], [[math.nan, 0, 0, 0, 0, 0]]):
            with self.subTest(bad=bad):
                client = FakeClient(bad, chunk(0))
                p = ChunkPolicy(client, S)
                p.reset(17)
                self.assertEqual(pan(p.act([0] * 6, "f")), 0.0)

    def test_no_reply_within_timeout_raises(self):
        client = FakeClient()
        client.release.clear()  # 응답이 오지 않음
        p = ChunkPolicy(client, S)
        p.reset(17)
        t0 = time.monotonic()
        with self.assertRaises(PolicyUnavailable):
            p.act([0] * 6, "f")
        self.assertGreaterEqual(time.monotonic() - t0, S.infer_timeout_s - 0.05)
        client.release.set()

    def test_keeps_failing_raises_after_timeout(self):
        client = FakeClient(*[ConnectionError("down")] * 1000)
        p = ChunkPolicy(client, S)
        p.reset(17)
        with self.assertRaises(PolicyUnavailable):
            p.act([0] * 6, "f")
        self.assertLess(len(client.calls), 100)  # 재요청 간격(RETRY_S)으로 몰리지 않음


class Reset(unittest.TestCase):
    def test_late_reply_from_previous_command_is_dropped(self):
        client = FakeClient(chunk(0), chunk(500))
        p = ChunkPolicy(client, S)
        p.reset(17)
        for i in range(3):
            p.act([0] * 6, i)
        client.release.clear()
        p.act([0] * 6, "old")  # 미리 요청(chunk 500 대기)
        wait_calls(client, 2)
        p.reset(18)  # 새 명령
        client.release.set()
        time.sleep(0.05)  # 이전 명령 응답 도착
        client.replies = [chunk(900)]
        self.assertEqual(pan(p.act([0] * 6, "new")), 900.0)
        self.assertIn("marker 18", client.calls[-1][2])


if __name__ == "__main__":
    unittest.main()
