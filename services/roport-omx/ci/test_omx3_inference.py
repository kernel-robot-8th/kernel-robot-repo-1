"""OMX 3 동작 묶음 정책 검사 (가짜 추론 클라이언트, 표준 라이브러리만)."""

import itertools
import logging
import math
import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ci"))

import test_omx3_runner as R  # noqa: E402
from roport_omx.omx3 import inference  # noqa: E402
from roport_omx.omx3.contract import ErrorCode  # noqa: E402
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
    """요청을 기록하고, replies 순서대로 응답한다(예외면 던짐). release가 닫히면 응답을 붙잡는다."""

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
        return reply


def until(cond, limit=2.0):
    end = time.monotonic() + limit
    while not cond() and time.monotonic() < end:
        time.sleep(0.002)


def tick(p, frame="f"):
    """runner처럼 주기마다 새 관측으로 act()를 부른다: 동작이 나올 때까지(관측 번호 = 주기 번호)."""
    for n in range(1000):
        action = p.act([float(n)] * 6, frame)
        if action is not None:
            return action
        time.sleep(0.002)
    raise AssertionError("no action")


def settled(p):
    until(lambda: not p._pending)


def pan(action):
    return action[0]


class Consume(unittest.TestCase):
    def test_returns_none_until_first_chunk_then_in_order(self):
        client = FakeClient(chunk(0))
        client.release.clear()
        p = ChunkPolicy(client, S)
        p.reset(17)
        self.assertIsNone(p.act([1] * 6, "f"))  # 기다리지 않는다
        client.release.set()
        self.assertEqual([pan(tick(p)) for _ in range(2)], [0.0, 1.0])
        state, frame, task, robot_type = client.calls[0]
        self.assertEqual((state, frame), ([1] * 6, "f"))
        self.assertEqual(task, "Pick up the box with marker 17 and put it on the tray")
        self.assertEqual(robot_type, "omx_follower")

    def test_requests_next_chunk_early_and_drops_executed_steps(self):
        # 묶음 6개, refill_at 3: 3개 남으면 미리 요청. 응답 전에 실행한 스텝 수만큼 앞을 버린다
        client = FakeClient(chunk(0), chunk(100))
        p = ChunkPolicy(client, S)
        p.reset(17)
        got = [pan(tick(p)) for _ in range(3)]  # 0,1,2 → 3개 남음
        client.release.clear()
        got.append(pan(p.act([0] * 6, "at-request")))  # 미리 요청(응답 대기), 3
        until(lambda: len(client.calls) == 2)
        got.append(pan(p.act([0] * 6, "x")))  # 큐의 남은 동작으로 계속, 4
        client.release.set()
        settled(p)
        got.append(pan(p.act([0] * 6, "y")))
        # 새 묶음 0번 = 요청한 스텝(3), 1번 = 다음 스텝(4). 둘 다 이미 실행했으므로 102부터
        self.assertEqual(got, [0, 1, 2, 3, 4, 102])
        self.assertEqual(client.calls[1][1], "at-request")
        self.assertEqual(len(client.calls), 2)  # 응답을 기다리는 동안 중복 요청 없음

    def test_too_late_reply_keeps_running_queue(self):
        # 2스텝 실행 뒤 2개짜리 응답: 보정하면 비므로 남은 큐(5)를 버리지 않는다(#96 리뷰)
        client = FakeClient(chunk(0), chunk(100, n=2))
        p = ChunkPolicy(client, S)
        p.reset(17)
        for _ in range(3):
            tick(p)
        client.release.clear()
        self.assertEqual(pan(p.act([0] * 6, "at-request")), 3.0)
        until(lambda: len(client.calls) == 2)
        self.assertEqual(pan(p.act([0] * 6, "x")), 4.0)
        client.release.set()
        settled(p)
        self.assertEqual(pan(p.act([0] * 6, "y")), 5.0)


class Failure(unittest.TestCase):
    def test_retry_uses_new_observation(self):
        # 큐가 빈 채 실패: 다음 요청은 그 뒤 주기의 관측으로(#96 리뷰)
        client = FakeClient(TimeoutError("lost"), chunk(0))
        p = ChunkPolicy(client, S)
        p.reset(17)
        self.assertEqual(pan(tick(p)), 0.0)
        self.assertEqual(len(client.calls), 2)
        self.assertNotEqual(client.calls[0][0], client.calls[1][0])

    def test_bad_reply_is_rejected_and_retried(self):
        for bad in ([], [[1, 2, 3]], [[math.nan, 0, 0, 0, 0, 0]]):
            with self.subTest(bad=bad):
                client = FakeClient(bad, chunk(0))
                p = ChunkPolicy(client, S)
                p.reset(17)
                self.assertEqual(pan(tick(p)), 0.0)

    def test_no_reply_within_timeout_raises(self):
        client = FakeClient()
        client.release.clear()  # 응답이 오지 않음
        p = ChunkPolicy(client, S)
        p.reset(17)
        t0 = time.monotonic()
        with self.assertRaises(PolicyUnavailable):
            tick(p)
        self.assertGreaterEqual(time.monotonic() - t0, S.infer_timeout_s)
        client.release.set()

    def test_keeps_failing_raises_after_timeout(self):
        client = FakeClient(*[ConnectionError("down")] * 1000)
        p = ChunkPolicy(client, S)
        p.reset(17)
        with self.assertRaises(PolicyUnavailable):
            tick(p)
        self.assertLess(len(client.calls), 100)  # 재요청 간격(RETRY_S)으로 몰리지 않음


class Reset(unittest.TestCase):
    def test_late_reply_from_previous_command_is_dropped(self):
        client = FakeClient(chunk(0), chunk(500))
        p = ChunkPolicy(client, S)
        p.reset(17)
        for _ in range(3):
            tick(p)
        client.release.clear()
        p.act([0] * 6, "old")  # 미리 요청(chunk 500 대기)
        until(lambda: len(client.calls) == 2)
        p.reset(19)  # 새 명령
        client.release.set()
        until(lambda: not client.replies)  # 이전 명령의 응답(chunk 500)이 도착해 버려짐
        client.replies = [chunk(900)]
        self.assertEqual(pan(tick(p)), 900.0)
        self.assertIn("marker 19", client.calls[-1][2])


class WithRunner(unittest.TestCase):
    """실제 Runner + ChunkPolicy, 실제 시계."""

    def test_cancel_while_waiting_for_inference_sends_nothing(self):
        # 추론 대기 중 취소 → 응답이 와도 모터 목표 0회, CANCELED·hold(#96 리뷰 P1)
        client = FakeClient(chunk(0, n=100))
        client.release.clear()
        robot, abort = R.FakeRobot(), threading.Event()
        clock = type("Real", (), {"__call__": lambda s: time.monotonic()})()
        clock.sleep = time.sleep
        runner, _ = R.make(robot, ChunkPolicy(client, S), lambda f: R.BOX, clock)
        result = []
        worker = threading.Thread(target=lambda: result.append(runner.run(17, abort)))
        worker.start()
        until(lambda: client.calls)  # 추론 요청이 나감(HOMING은 이미 init이라 전송 0회)
        abort.set()
        client.release.set()
        worker.join(5)
        self.assertEqual(result[0].error_code, ErrorCode.CANCELED)
        self.assertEqual(robot.sent, 0)
        self.assertGreaterEqual(robot.held, 1)


E = Omx3Settings(infer_timeout_s=0.5, ensemble_coeff=0.01)


def ensemble_policy(client, settings=E):
    p = ChunkPolicy(client, settings)
    p.reset(17)
    return p


class TemporalEnsemble(unittest.TestCase):
    """패널 "부드럽게 실행"과 같은 계산: 같은 스텝을 가리키는 예측을 exp(-0.01 x 스텝 차이)로 평균."""

    def test_first_chunk_alone_is_used_as_is(self):
        p = ensemble_policy(FakeClient(chunk(0)))
        self.assertEqual(pan(tick(p)), 0.0)

    def test_overlapping_predictions_use_act_weights(self):
        # 스텝 0 관측 → 예측 A(값 0..), 스텝 1 관측 → 예측 B(값 100..).
        # 스텝 1은 B 요청만 하고 A[1], 스텝 2 = A[2]·B[1] 가중 평균
        client = FakeClient(chunk(0), chunk(100))
        p = ensemble_policy(client)
        self.assertEqual(pan(tick(p)), 0.0)  # 스텝 0: A만
        self.assertEqual(pan(p.act([1.0] * 6, "f")), 1.0)  # 스텝 1: B는 아직 요청 중
        settled(p)
        action = pan(p.act([2.0] * 6, "f"))
        w_old, w_new = 1.0, math.exp(-0.01 * 1)  # 오래된 예측(A)이 더 무겁다
        self.assertAlmostEqual(action, (w_old * 2.0 + w_new * 101.0) / (w_old + w_new))

    def test_requests_again_as_soon_as_previous_reply_arrives(self):
        client = FakeClient(*[chunk(0, 50) for _ in range(10)])
        p = ensemble_policy(client)
        for _ in range(6):
            tick(p)
            settled(p)
        self.assertGreaterEqual(len(client.calls), 6)  # 매 스텝(묶음 통째 교체라면 1번)

    def test_ensemble_every_limits_request_rate(self):
        client = FakeClient(*[chunk(0, 50) for _ in range(10)])
        p = ensemble_policy(client, Omx3Settings(ensemble_coeff=0.01, ensemble_every=3))
        for _ in range(6):
            tick(p)
            settled(p)
        self.assertEqual(len(client.calls), 2)  # 스텝 0, 3

    def test_no_jump_where_a_chunk_would_be_replaced(self):
        # 다른 궤적(0..과 100..)이 섞이면 출력은 두 예측 사이에서 연속으로 움직인다
        client = FakeClient(chunk(0, 20), *[chunk(100, 20) for _ in range(20)])
        p = ensemble_policy(client)
        out = []
        for _ in range(10):
            out.append(pan(tick(p)))
            settled(p)
        steps = [b - a for a, b in itertools.pairwise(out)]
        self.assertLess(max(steps), 100)  # 통째 교체였다면 한 번에 약 100 튐
        self.assertTrue(all(0 <= v <= 120 for v in out))

    def test_failed_request_is_retried_even_before_any_action(self):
        p = ensemble_policy(FakeClient(OSError("lost"), chunk(0)))
        self.assertEqual(pan(tick(p)), 0.0)

    def test_finished_predictions_are_dropped_and_starvation_raises(self):
        client = FakeClient(chunk(0, 2))
        p = ensemble_policy(client)
        tick(p)
        settled(p)
        client.release.clear()  # 다음 요청은 응답하지 않는다
        tick(p)
        with self.assertRaises(PolicyUnavailable):
            for _ in range(400):
                p.act([0.0] * 6, "f")
                time.sleep(0.005)
        self.assertEqual(p._chunks, [])
        client.release.set()

    def test_reset_drops_previous_predictions(self):
        client = FakeClient(chunk(0, 50), chunk(500, 50))
        p = ensemble_policy(client)
        tick(p)
        settled(p)
        p.reset(17)
        self.assertEqual(p._chunks, [])
        self.assertEqual(pan(tick(p)), 500.0)

    def test_ensemble_every_must_be_positive(self):
        with self.assertRaises(ValueError):
            Omx3Settings(ensemble_every=0)


if __name__ == "__main__":
    unittest.main()
