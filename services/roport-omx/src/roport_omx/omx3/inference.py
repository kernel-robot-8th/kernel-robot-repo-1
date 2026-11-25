"""원격 추론 동작 묶음 정책: 통신 방식을 감춘 InferClient 위에서 ports.Policy를 구현한다.

ACT는 한 번 추론에 N개(예 100) 동작을 돌려준다. 남은 동작을 실행하는 동안 다음 묶음을
백그라운드로 미리 요청해 30 fps 루프가 추론 지연에 막히지 않게 한다.

act()는 기다리지 않는다. 동작이 없으면 None을 돌려주고, runner는 그 주기에 목표를 보내지 않은 채
다음 주기에 새로 관측한다. 그래서 취소는 한 주기 안에 반영되고, 재요청은 늘 그 주기의 관측을 쓴다.

settings.ensemble_coeff를 주면 시간 앙상블(ACT 논문 Algorithm 2, LeRobot ACTTemporalEnsembler와
같은 가중치)을 쓴다: 앞 요청이 끝나는 대로 ensemble_every 스텝마다 새로 예측하고, 같은 스텝을
가리키는 모든 예측을 w = exp(-coeff x 관측 스텝 차이)로 평균한다(오래된 예측일수록 무겁다).
패널 "부드럽게 실행"(매 스텝 예측, coeff 0.01)과 같은 계산이며, 원격 추론 왕복(약 40 ms)이 한
주기(33 ms)보다 길어 예측 간격만 1~2스텝이 된다. 묶음을 통째로 갈아 끼울 때의 튐이 없어진다.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from collections.abc import Sequence
from typing import Any, Protocol

from .ports import PolicyUnavailable
from .settings import JOINTS, Omx3Settings

log = logging.getLogger(__name__)
RETRY_S = 0.1  # 실패 뒤 다시 요청하기까지 최소 간격


class InferClient(Protocol):
    def infer(
        self, state: Sequence[float], marked_front: Any, task: str, robot_type: str
    ) -> Sequence[Sequence[float]]:
        """관측 1개 → JOINTS 순서 절대 목표값 N x 6. 안에서 시간 제한을 두고 넘으면 예외."""


class ChunkPolicy:
    def __init__(self, client: InferClient, settings: Omx3Settings | None = None):
        self.client = client
        self.settings = settings or Omx3Settings()
        self._lock = threading.Lock()
        self._queue: deque[list[float]] = deque()
        self._gen = 0  # reset마다 올려 이전 명령의 늦은 응답을 버린다
        self._step = 0  # act()로 꺼낸 동작 수
        self._pending = False
        self._starved_at: float | None = None  # 큐가 빈 채로 기다리기 시작한 시각
        self._task = ""
        self._chunks: list[
            tuple[int, list[list[float]]]
        ] = []  # 앙상블: (관측 스텝, 동작들)
        self._asked_at: int | None = None  # 앙상블: 마지막 요청의 스텝

    def reset(self, marker_id: int) -> None:
        with self._lock:
            self._gen += 1
            self._queue.clear()
            self._chunks.clear()
            self._step, self._pending, self._starved_at = 0, False, None
            self._asked_at = None
            self._task = self.settings.task.format(marker_id=marker_id)

    def act(self, state: Sequence[float], marked_front: Any) -> list[float] | None:
        s = self.settings
        with self._lock:
            if s.ensemble_coeff is None:
                if not self._pending and len(self._queue) <= s.refill_at:
                    self._request(list(state), marked_front)
                action = self._queue.popleft() if self._queue else None
            else:
                if not self._pending and (
                    self._asked_at is None
                    or self._step - self._asked_at >= s.ensemble_every
                ):
                    self._asked_at = self._step
                    self._request(list(state), marked_front)
                action = self._ensembled(self._step)
            if action is not None:
                self._starved_at = None
                self._step += 1
                return action
            now = time.monotonic()
            if self._starved_at is None:
                self._starved_at = now
            elif now - self._starved_at > s.infer_timeout_s:
                raise PolicyUnavailable(f"no chunk in {s.infer_timeout_s}s")
            return None

    def _request(self, state: list[float], frame: Any) -> None:
        self._pending = True
        args = (self._gen, self._step, state, frame, self._task)
        threading.Thread(target=self._worker, args=args, daemon=True).start()

    def _worker(self, gen: int, step: int, state, frame, task: str) -> None:
        try:
            chunk = _check(
                self.client.infer(state, frame, task, self.settings.robot_type)
            )
        except Exception:  # 다음 act()가 그 주기의 관측으로 다시 요청한다
            log.exception("inference failed")
            chunk = None
            time.sleep(RETRY_S)  # 바로 실패하는 클라이언트로 요청이 몰리지 않게
        with self._lock:
            if gen != self._gen:  # 그 사이 reset: 이전 명령의 응답
                return
            self._pending = False
            if self.settings.ensemble_coeff is not None:
                if chunk:
                    self._chunks.append((step, chunk))
                else:  # 실패: 스텝이 멈춰 있어도 다음 act()가 바로 다시 요청하게
                    self._asked_at = None
                return
            # 요청 뒤 이미 실행한 스텝만큼 앞을 버린다: 묶음 첫 동작은 요청 시점 관측 기준.
            # 다 버려지면(응답이 너무 늦음) 실행 중이던 큐를 그대로 둔다(#96 리뷰)
            rest = chunk[self._step - step :] if chunk else []
            if rest:
                self._queue = deque(rest)

    def _ensembled(self, t: int) -> list[float] | None:
        """스텝 t를 가리키는 모든 예측의 가중 평균. 끝난 묶음은 버린다(호출자가 잠금을 잡는다)."""
        self._chunks = [(b, rows) for b, rows in self._chunks if t < b + len(rows)]
        preds = [(b, rows[t - b]) for b, rows in self._chunks if b <= t]
        if not preds:
            return None
        oldest = min(b for b, _ in preds)
        coeff = self.settings.ensemble_coeff
        weights = [math.exp(-coeff * (b - oldest)) for b, _ in preds]
        total = sum(weights)
        return [
            sum(w * row[j] for w, (_, row) in zip(weights, preds, strict=True)) / total
            for j in range(len(JOINTS))
        ]


def _check(chunk: Sequence[Sequence[float]]) -> list[list[float]]:
    rows = [[float(v) for v in row] for row in chunk]
    if not rows:
        raise ValueError("empty chunk")
    for row in rows:
        if len(row) != len(JOINTS) or not all(math.isfinite(v) for v in row):
            raise ValueError(f"bad action row: {row}")
    return rows
