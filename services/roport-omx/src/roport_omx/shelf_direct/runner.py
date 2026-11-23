"""평가 한 번의 실행: CONNECT → RUNNING(30 Hz) → LANDING → DONE (OMX3 Runner와 같은 역할).

Panel omx_run.py의 평가 경로(episodes=1, 녹화 없음)를 옮겼다.
- 고정 주기(start + k/fps). 밀리면 지금부터 다시 맞춘다.
- 끝나는 이유: returned(복귀 감지), placed(놓음 판정 등 end_checks), timeout(episode_s),
  stopped(stop 요청), frozen(비상 정지), error(추론·안전 시작·장치 오류), refused(연결 전 점검 실패).
- 착지: placed이고 넘겨받기 준비(hold_ready)면 hold(토크 유지), returned이고 fast_release면 fast,
  나머지는 soft(처음 자세로 돌아가 착지). frozen이면 release까지 토크를 켠 채 기다린 뒤 soft.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

from .contract import GRIPPER_INDEXES, WIDTH
from .ports import (
    Observation,
    Policy,
    PolicyUnavailable,
    Robot,
    RobotNotReady,
    SafeStartRefused,
)

log = logging.getLogger(__name__)
EndCheck = Callable[[Observation, float], str | None]


@dataclass(frozen=True)
class RunSettings:
    fps: float = 30.0
    episode_s: float = 600.0  # 현장 경로표 값
    end_on_return: bool = True
    fast_release: bool = False  # 입고: 복귀로 끝나면 빠른 착지(정렬기가 이어서 잡음)
    return_leave: float = 20.0
    return_tol: float = 8.0
    return_grip_max: float = 54.6
    return_hold_s: float = 0.3

    def __post_init__(self):
        if not 3 <= self.episode_s <= 600:
            raise ValueError("episode_s must be 3..600")
        if self.fps <= 0:
            raise ValueError("fps must be > 0")


DEFAULT_RUN = RunSettings()


class ReturnWatch:
    """처음 자세(첫 5단계 평균)에서 어느 관절이 leave 넘게 벗어났다가, 모든 관절이 tol 안으로 돌아오고
    모든 그리퍼가 grip_max 이하(놓음)인 상태가 hold_s 유지되면 끝. 값은 Panel 모델 22 평가 기준."""

    def __init__(self, s: RunSettings):
        self.s = s
        self.reset()

    def reset(self) -> None:
        self.start: list[float] | None = None
        self.firsts: list[list[float]] = []
        self.left = False
        self.since: float | None = None

    def __call__(self, state: Sequence[float], now: float) -> bool:
        joints = [v for i, v in enumerate(state) if i not in GRIPPER_INDEXES]
        grips = [state[i] for i in GRIPPER_INDEXES]
        if self.start is None:
            self.firsts.append(list(joints))
            if len(self.firsts) == 5:
                self.start = [sum(c) / 5 for c in zip(*self.firsts, strict=True)]
            return False
        dev = max(abs(v - s) for v, s in zip(joints, self.start, strict=True))
        if dev > self.s.return_leave:
            self.left = True
        if (
            self.left
            and dev <= self.s.return_tol
            and all(g <= self.s.return_grip_max for g in grips)
        ):
            self.since = self.since if self.since is not None else now
            return now - self.since >= self.s.return_hold_s
        self.since = None
        return False


@dataclass
class RunResult:
    end_reason: str
    elapsed_s: float | None = None
    error: str | None = None
    landing: str = "none"
    landed_home: bool | None = None
    last_state: tuple[float, ...] | None = None
    state_at: float | None = None
    start_poses: dict = field(default_factory=dict)
    frames: int = 0
    stalls: int = 0  # 동작이 없어 보내지 못한 주기
    loop_ms: dict = field(default_factory=dict)


class ShelfRunner:
    def __init__(
        self,
        robot: Robot,
        policy: Policy,
        settings: RunSettings = DEFAULT_RUN,
        *,
        end_checks: Sequence[EndCheck] = (),
        hold_ready: Callable[[], bool] = lambda: False,
        on_phase: Callable[[str, dict], None] = lambda _phase, _info: None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self.robot, self.policy, self.s = robot, policy, settings
        self.end_checks, self.hold_ready, self.on_phase = (
            list(end_checks),
            hold_ready,
            on_phase,
        )
        self.clock, self.sleep = clock, sleep
        self.watch = ReturnWatch(settings)

    def run(self, task: str, stop: threading.Event) -> RunResult:
        self.on_phase("CONNECT", {})
        try:
            self.robot.connect()
        except RobotNotReady as exc:
            self.on_phase("DONE", {"end_reason": "refused"})
            return RunResult("refused", error=str(exc))
        result = RunResult("timeout")
        try:
            self._loop(task, stop, result)
        except (PolicyUnavailable, SafeStartRefused) as exc:
            result.end_reason, result.error = "error", f"{type(exc).__name__}: {exc}"
        except Exception as exc:  # 장치 오류: 그래도 착지한다
            log.exception("run loop failed")
            result.end_reason, result.error = "error", f"{type(exc).__name__}: {exc}"
        result.start_poses = self.robot.start_poses()
        frozen = self.robot.freeze.is_set()
        if frozen:
            result.end_reason = "frozen"
        if result.end_reason == "placed" and not frozen and self.hold_ready():
            mode = "hold"
        elif result.end_reason == "returned" and self.s.fast_release and not frozen:
            mode = "fast"
        else:
            mode = "soft"
        result.landing = mode
        self.on_phase("LANDING", {"mode": mode, "end_reason": result.end_reason})
        try:
            self.robot.land(mode)
        except Exception as exc:
            log.exception("landing failed")
            result.error = result.error or f"landing: {exc}"
        finally:
            self.robot.disconnect()
        self.on_phase("DONE", {"end_reason": result.end_reason})
        return result

    def _loop(self, task: str, stop: threading.Event, result: RunResult) -> None:
        self.policy.new_episode(task)
        self.watch.reset()
        interval = 1 / self.s.fps
        start = due = self.clock()
        durations: list[float] = []
        self.on_phase("RUNNING", {})
        while self.clock() - start < self.s.episode_s:
            t0 = self.clock()
            if stop.is_set() or self.robot.freeze.is_set():
                result.end_reason = "stopped"
                break
            obs = self.robot.observe()
            if len(obs.state) != WIDTH:
                raise ValueError(f"observation needs {WIDTH} joints")
            result.last_state, result.state_at = obs.state, obs.at
            if self.s.end_on_return and self.watch(obs.state, t0):
                result.end_reason = "returned"
                break
            reason = next((r for r in (c(obs, t0) for c in self.end_checks) if r), None)
            if reason:
                result.end_reason = reason
                break
            action = self.policy.act(obs)
            if action is None:
                result.stalls += 1
            else:
                self.robot.send(action)
            result.frames += 1
            durations.append(self.clock() - t0)
            due += interval
            if self.clock() > due + interval:  # 밀림: 지금부터 다시
                due = self.clock()
            self.sleep(max(due - self.clock(), 0.0))
        result.elapsed_s = round(self.clock() - start, 1)
        if durations:
            ms = sorted(d * 1000 for d in durations)
            result.loop_ms = {
                "p50": round(ms[len(ms) // 2], 1),
                "p99": round(ms[int(len(ms) * 0.99)], 1),
            }
