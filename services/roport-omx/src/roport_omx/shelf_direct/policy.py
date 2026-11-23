"""async 동작 묶음 정책 + 출력 필터. Panel(omx_async.AsyncRunner, omx_run.ActionFilter)의 현장 규칙을 옮겼다.

async 규칙 (Panel과 같음):
- 제어 단계 t마다 동작 하나를 큐에서 꺼낸다. 큐에 남은 동작이 threshold x actions_per_chunk 이하이고
  추론이 돌고 있지 않으면, 이번 관측으로 백그라운드 추론을 시작한다(시작 단계 t0).
- 끝난 묶음은 t0, t0+1, ... 단계를 덮는다. 그 사이 이미 실행한 단계는 버리고, 큐에 남은 단계는
  aggregate(latest_only면 새 값으로 교체)로 합친다.
- 큐가 비면 Panel은 그 단계에서 추론을 기다린다(단계 번호가 늘지 않음). 여기서는 기다리는 대신 None을
  돌려주고 단계 번호를 그대로 둔다. 그래서 묶음의 단계 대응은 Panel과 같고, runner는 그동안에도
  정지 요청을 볼 수 있다(OMX3 ChunkPolicy와 같은 방식).
- 추론 실패는 다음 act()에서 PolicyUnavailable로 올린다.

출력 필터: 목표에 지수 이동 평균(alpha = 새 값 가중치). 에피소드마다 첫 목표는 그대로 쓴다.
Panel은 그리퍼를 "off"로 설정해도 양팔 이름(left_gripper.pos)을 그리퍼로 찾지 못해 그리퍼에도
같은 alpha를 걸어 왔다. 현장에서 검증된 동작과 같게 gripper_alpha 기본값은 alpha이고,
1.0을 주면 그리퍼는 거르지 않는다.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Sequence

from .contract import GRIPPER_INDEXES, WIDTH
from .ports import Observation, PolicyUnavailable

log = logging.getLogger(__name__)

AGGREGATE = {
    "weighted_average": lambda old, new: 0.3 * old + 0.7 * new,
    "average": lambda old, new: 0.5 * old + 0.5 * new,
    "latest_only": lambda old, new: new,
}


class ActionFilter:
    def __init__(self, alpha: float, gripper_alpha: float | None = None):
        if not 0 < alpha <= 1:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.gripper_alpha = alpha if gripper_alpha is None else gripper_alpha
        if not 0 < self.gripper_alpha <= 1:
            raise ValueError("gripper_alpha must be in (0, 1]")
        self.prev: list[float] | None = None

    def reset(self) -> None:
        self.prev = None

    def __call__(self, action: Sequence[float]) -> list[float]:
        x = [float(v) for v in action]
        if self.prev is None or len(self.prev) != len(x):
            self.prev = x
            return list(x)
        y = []
        for i, (p, v) in enumerate(zip(self.prev, x, strict=True)):
            a = self.gripper_alpha if i in GRIPPER_INDEXES else self.alpha
            y.append(p + a * (v - p))
        self.prev = y
        return list(y)


class BiChunkPolicy:
    def __init__(
        self,
        client,  # infer(state, images, task) -> N x 12 (BiUdpInferClient)
        *,
        actions_per_chunk: int = 100,
        threshold: float = 0.25,
        aggregate: str = "latest_only",
        action_filter: ActionFilter | None = None,
    ):
        if aggregate not in AGGREGATE:
            raise ValueError(f"unknown aggregate {aggregate!r}")
        self.client = client
        self.chunk = max(int(actions_per_chunk), 1)
        self.threshold = min(max(float(threshold), 0.0), 1.0)
        self.aggregate = aggregate
        self.filter = action_filter
        self.task = ""
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._gen = 0  # 에피소드 번호: 이전 에피소드의 늦은 묶음은 버린다
        self._reset_queue()

    def _reset_queue(self) -> None:
        self._queue: list[tuple[int, list[float]]] = []  # (단계, 동작)
        self._t = 0  # 다음에 실행할 단계
        self._error: BaseException | None = None

    def new_episode(self, task: str) -> None:
        with self._lock:
            self._gen += 1
            self.task = task
            self._reset_queue()
        if self.filter is not None:
            self.filter.reset()

    @property
    def busy(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def act(self, observation: Observation) -> list[float] | None:
        with self._lock:
            if self._error is not None:
                err, self._error = self._error, None
                if isinstance(err, PolicyUnavailable):
                    raise err
                raise PolicyUnavailable(f"inference failed: {err}") from err
            t = self._t
            self._queue = [q for q in self._queue if q[0] >= t]
            have = len(self._queue)
            start = not self.busy and (have == 0 or have <= self.threshold * self.chunk)
            if start:
                self._start(t, observation)
            if not have:
                return (
                    None  # 묶음을 기다린다. 단계 번호는 그대로(Panel과 같은 단계 대응)
                )
            _, action = self._queue.pop(0)
            self._t = t + 1
        return self.filter(action) if self.filter is not None else action

    def _start(self, t0: int, observation: Observation) -> None:
        gen = self._gen
        self._worker = threading.Thread(
            target=self._infer,
            args=(
                gen,
                t0,
                tuple(observation.state),
                dict(observation.images),
                self.task,
            ),
            daemon=True,
        )
        self._worker.start()

    def _infer(self, gen: int, t0: int, state, images, task: str) -> None:
        try:
            chunk = self.client.infer(state, images, task)
            rows = _check(chunk)
        except BaseException as exc:  # noqa: BLE001 - 어떤 실패든 control thread에서 올린다
            with self._lock:
                if gen == self._gen:
                    self._error = exc
            return
        with self._lock:
            if gen != self._gen:
                return
            self._merge(t0, rows)

    def _merge(self, t0: int, rows: list[list[float]]) -> None:
        now = self._t
        old = {ts: a for ts, a in self._queue if ts >= now}
        agg = AGGREGATE[self.aggregate]
        new = []
        for i, row in enumerate(rows):
            ts = t0 + i
            if ts < now:
                continue  # 묶음을 계산하는 동안 이미 실행한 단계
            if ts in old:
                prev = old.pop(ts)
                row = [agg(o, n) for o, n in zip(prev, row, strict=True)]
            new.append((ts, row))
        last = new[-1][0] if new else now - 1
        self._queue = new + [(ts, old[ts]) for ts in sorted(old) if ts > last]

    def wait_idle(self, timeout: float | None = None) -> None:
        """테스트·종료용: 돌고 있는 추론이 끝날 때까지 기다린다."""
        worker = self._worker
        if worker is not None:
            worker.join(timeout)


def _check(chunk: Sequence[Sequence[float]]) -> list[list[float]]:
    rows = [[float(v) for v in row] for row in chunk]
    if not rows or any(len(r) != WIDTH for r in rows):
        raise PolicyUnavailable(f"chunk rows must have {WIDTH} values")
    return rows
