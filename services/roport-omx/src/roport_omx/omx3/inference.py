"""원격 추론 동작 묶음 정책: 통신 방식을 감춘 InferClient 위에서 ports.Policy를 구현한다.

ACT는 한 번 추론에 N개(예 100) 동작을 돌려준다. 남은 동작을 실행하는 동안 다음 묶음을
백그라운드로 미리 요청해 30 fps 루프가 추론 지연에 막히지 않게 한다.
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
        self._cond = threading.Condition()
        self._queue: deque[list[float]] = deque()
        self._gen = 0  # reset마다 올려 이전 명령의 늦은 응답을 버린다
        self._step = 0  # act()로 꺼낸 동작 수
        self._pending = False
        self._task = ""

    def reset(self, marker_id: int) -> None:
        with self._cond:
            self._gen += 1
            self._queue.clear()
            self._step, self._pending = 0, False
            self._task = self.settings.task.format(marker_id=marker_id)

    def act(self, state: Sequence[float], marked_front: Any) -> list[float]:
        s = self.settings
        with self._cond:
            if not self._pending and len(self._queue) <= s.refill_at:
                self._request(list(state), marked_front)
            # 첫 묶음이나 지연·유실: 실패하면 제한 시간 안에서 새 관측으로 다시 요청한다
            deadline = time.monotonic() + s.infer_timeout_s
            while not self._queue:
                if not self._pending:
                    self._request(list(state), marked_front)
                left = deadline - time.monotonic()
                if left <= 0:
                    raise PolicyUnavailable(f"no chunk in {s.infer_timeout_s}s")
                self._cond.wait_for(lambda: self._queue or not self._pending, left)
            self._step += 1
            return self._queue.popleft()

    def _request(self, state: list[float], frame: Any) -> None:
        self._pending = True
        args = (self._gen, self._step, state, frame, self._task)
        threading.Thread(target=self._worker, args=args, daemon=True).start()

    def _worker(self, gen: int, step: int, state, frame, task: str) -> None:
        try:
            chunk = _check(
                self.client.infer(state, frame, task, self.settings.robot_type)
            )
        except Exception:  # 다음 act()가 새 관측으로 다시 요청한다
            log.exception("inference failed")
            chunk = None
            time.sleep(RETRY_S)  # 바로 실패하는 클라이언트로 요청이 몰리지 않게
        with self._cond:
            if gen != self._gen:  # 그 사이 reset: 이전 명령의 응답
                return
            self._pending = False
            if chunk is not None:
                # 요청 뒤 이미 실행한 스텝만큼 앞을 버린다: 묶음 첫 동작은 요청 시점 관측 기준
                self._queue = deque(chunk[self._step - step :])
            self._cond.notify_all()


def _check(chunk: Sequence[Sequence[float]]) -> list[list[float]]:
    rows = [[float(v) for v in row] for row in chunk]
    if not rows:
        raise ValueError("empty chunk")
    for row in rows:
        if len(row) != len(JOINTS) or not all(math.isfinite(v) for v in row):
            raise ValueError(f"bad action row: {row}")
    return rows
