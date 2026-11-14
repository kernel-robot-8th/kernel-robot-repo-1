"""명령 한 건의 실행: MARKER_CHECK → HOMING → RUNNING → VERIFY → COMPLETE | FAILED | REJECTED."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable

from . import judge
from .contract import ErrorCode, Outcome, Phase
from .ports import Marker, Policy, PolicyUnavailable, Robot
from .settings import Omx3Settings

PhaseCallback = Callable[[Phase, dict], None]
log = logging.getLogger(__name__)


class Runner:
    def __init__(
        self,
        robot: Robot,
        policy: Policy,
        marker_factory: Callable[[int], Marker],
        settings: Omx3Settings | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self.robot, self.policy, self.marker_factory = robot, policy, marker_factory
        self.settings = settings or Omx3Settings()
        self.clock, self.sleep = clock, sleep
        self._lock = threading.Lock()

    def precheck(self, marker_id: int) -> ErrorCode | None:
        """모터를 움직이지 않고 시작 가능 여부만 본다(Host가 접수 전에 부른다)."""
        if self._lock.locked():  # 실행 중이면 팔이 init이 아니어도 BUSY로 답한다
            return ErrorCode.BUSY
        return self._precheck(self.marker_factory(marker_id))

    def _precheck(self, marker: Marker) -> ErrorCode | None:
        s = self.settings
        if self.robot.ready() is not None:
            return ErrorCode.ROBOT_NOT_READY
        try:
            obs = self.robot.observe()
            # 앞 명령이 물품을 쥔 채 멈췄을 수 있다. 그 자세에서 HOMING하면 그리퍼가 열려
            # 물품이 떨어지므로, init·그리퍼 열림이 아니면 거절하고 작업자가 복구한다(#65 리뷰)
            at_init = judge.near_init(obs.state, s) and judge.gripper_open(obs.state, s)
        except Exception:  # 관측 실패 = 시작할 수 없음
            log.exception("precheck observe failed")
            return ErrorCode.ROBOT_NOT_READY
        if not at_init:
            log.warning("arm not at init: %s", list(obs.state))
            return ErrorCode.ROBOT_NOT_READY
        if marker.find(obs.front) is None:
            return ErrorCode.MARKER_NOT_VISIBLE
        return None

    def run(
        self,
        marker_id: int,
        abort: threading.Event | None = None,
        on_phase: PhaseCallback | None = None,
    ) -> Outcome:
        emit = on_phase or (lambda phase, info: None)
        if not self._lock.acquire(blocking=False):  # 한 팔에 명령은 한 번에 하나
            out = Outcome(Phase.REJECTED, ErrorCode.BUSY, marker_id)
            emit(out.phase, out.to_dict())
            return out
        try:
            return self._run(marker_id, abort or threading.Event(), emit)
        except BaseException:  # KeyboardInterrupt 등(Exception은 _run이 결과로 바꾼다)
            self._safe_hold()
            raise
        finally:
            self._lock.release()

    def _run(
        self, marker_id: int, abort: threading.Event, emit: PhaseCallback
    ) -> Outcome:
        s = self.settings
        t0 = self.clock()
        marker = self.marker_factory(marker_id)  # 명령 1개 = 마커 객체 1개

        def end(phase: Phase, code: ErrorCode | None = None, **kw) -> Outcome:
            if phase is Phase.FAILED:  # 거절(REJECTED)은 모터 쓰기 0회를 지킨다
                self._safe_hold()
            out = Outcome(phase, code, marker_id, self.clock() - t0, **kw)
            emit(phase, out.to_dict())
            return out

        emit(Phase.MARKER_CHECK, {"marker_id": marker_id})
        code = self._precheck(marker)
        if code is not None:
            return end(Phase.REJECTED, code)

        emit(Phase.HOMING, {})
        try:
            if not self._go_init(abort):
                if abort.is_set():
                    return end(Phase.FAILED, ErrorCode.CANCELED)
                return end(Phase.FAILED, ErrorCode.HOMING_FAILED)
        except Exception as exc:  # noqa: BLE001 - 장비 오류는 실패로 보고
            return end(Phase.FAILED, ErrorCode.ROBOT_ERROR, detail=repr(exc))

        marker.reset()  # 학습 데이터처럼 에피소드 시작에서 원 상태를 비운다
        self.policy.reset()
        emit(Phase.RUNNING, {})
        period, held = 1.0 / s.fps, 0
        start = next_tick = self.clock()
        try:
            while held < s.done_hold_frames:
                if abort.is_set():
                    return end(Phase.FAILED, ErrorCode.CANCELED)
                if self.clock() - start > s.timeout_s:
                    return end(Phase.FAILED, ErrorCode.TIMEOUT)
                obs = self.robot.observe()
                frame, seen = marker.mark(
                    obs.front
                )  # 매 프레임 그린다(가림 처리 상태 유지)
                held = held + 1 if judge.is_done(obs.state, seen, s) else 0
                # 완료 조건이 맞는 동안은 정책 동작을 보내지 않는다(대기 자세 움찔거림 방지).
                # 마지막 목표 자세에 멈춘 채 확인하고, 조건이 깨지면 다시 정책을 실행한다.
                if held == 0:
                    self.robot.send(self.policy.act(obs.state, frame))
                next_tick += period
                delay = next_tick - self.clock()
                if delay < 0:  # 늦으면 밀린 주기를 몰아 보내지 않는다
                    next_tick, delay = self.clock(), 0.0
                self.sleep(delay)
        except PolicyUnavailable as exc:
            return end(Phase.FAILED, ErrorCode.INFERENCE_UNAVAILABLE, detail=str(exc))
        except Exception as exc:  # noqa: BLE001 - 장비 오류는 실패로 보고
            return end(Phase.FAILED, ErrorCode.ROBOT_ERROR, detail=repr(exc))

        # 완료 보고 뒤 Pinky가 바로 움직이므로, 멈춘 뒤 한 번 더 보고 확인한다(#61 리뷰)
        emit(Phase.VERIFY, {})
        self._safe_hold()
        try:
            obs = self.robot.observe()
        except Exception as exc:  # noqa: BLE001 - 장비 오류는 실패로 보고
            return end(Phase.FAILED, ErrorCode.ROBOT_ERROR, detail=repr(exc))
        result = {
            "placement_confirmed": judge.in_tray(marker.find(obs.front), s),
            "arm_returned": judge.near_init(obs.state, s),
            "gripper_open": judge.gripper_open(obs.state, s),
        }
        if all(result.values()):
            return end(Phase.COMPLETE, **result)
        return end(Phase.FAILED, ErrorCode.VERIFY_FAILED, **result)

    def _go_init(self, abort: threading.Event) -> bool:
        """init 근처(precheck 통과)에서 학습 시작값으로 천천히 맞춘다. 그리퍼는 관절 5개 도착 뒤에만."""
        s, period = self.settings, 1.0 / self.settings.fps
        target = list(s.init_pose)
        deadline = self.clock() + s.home_timeout_s
        while self.clock() < deadline:
            if abort.is_set():
                return False
            state = [float(v) for v in self.robot.observe().state]
            # 중력 처짐 등으로 init 영역을 벗어나면 더 밀지 않고 멈춘다(HOMING_FAILED, #65 2차 리뷰)
            if not (judge.near_init(state, s) and judge.gripper_open(state, s)):
                return False
            err = [t - v for v, t in zip(state, target, strict=True)]
            if all(abs(e) <= s.home_tolerance for e in err):
                return True
            # 목표는 매 주기 실제 자세에서 한 걸음만: 팔이 걸려도 목표가 앞서가지 않는다(#65 리뷰)
            goal = [
                v + max(-s.home_step, min(s.home_step, e))
                for v, e in zip(state, err, strict=True)
            ]
            if any(abs(e) > s.home_tolerance for e in err[:5]):
                goal[5] = state[5]
            self.robot.send(goal)
            self.sleep(period)
        return False

    def _safe_hold(self) -> None:
        try:
            self.robot.hold()
        except Exception:  # 결과 보고를 막지 않는다
            log.exception("hold failed")
