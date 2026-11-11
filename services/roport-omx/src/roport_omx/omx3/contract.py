"""OMX 3 실행 단계·오류 코드·결과.

MQTT ``load.*`` envelope은 관제 공통 모듈이 붙인다. 여기서는 실행부 안의 값만 정의한다
(Tech Spec v9 제안 기준, 관제와 최종 합의 전).

전제: 관제는 Pinky가 상차 노드(N01)에 정차해 ``loading`` 상태일 때만 상차 명령을 보낸다.
그래서 명령 수신 = 트레이가 학습 위치에 있음으로 보고, 실행부는 트레이 유무를 따로 보지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class Phase(StrEnum):
    MARKER_CHECK = "MARKER_CHECK"
    HOMING = "HOMING"
    RUNNING = "RUNNING"
    VERIFY = "VERIFY"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    REJECTED = "REJECTED"  # 시작 전 거절, 모터 동작 없음


class ErrorCode(StrEnum):
    INVALID_TARGET = "INVALID_TARGET"
    MARKER_NOT_VISIBLE = "MARKER_NOT_VISIBLE"
    ROBOT_NOT_READY = "ROBOT_NOT_READY"
    BUSY = "BUSY"
    IDEMPOTENCY_CONFLICT = "IDEMPOTENCY_CONFLICT"
    RECONCILING = "RECONCILING"
    HOMING_FAILED = "HOMING_FAILED"
    TIMEOUT = "TIMEOUT"
    CANCELED = "CANCELED"
    INFERENCE_UNAVAILABLE = "INFERENCE_UNAVAILABLE"
    ROBOT_ERROR = "ROBOT_ERROR"
    VERIFY_FAILED = "VERIFY_FAILED"  # 루프는 끝났지만 보고 직전 재확인에서 조건이 깨짐


@dataclass(frozen=True)
class Outcome:
    """명령 한 건의 최종 결과."""

    phase: Phase
    error_code: ErrorCode | None = None
    marker_id: int | None = None
    duration_s: float = 0.0
    placement_confirmed: bool = False
    arm_returned: bool = False
    gripper_open: bool = False
    detail: str = ""

    @property
    def success(self) -> bool:
        return self.phase is Phase.COMPLETE

    @property
    def departure_ready(self) -> bool:
        """Pinky 출발 조건(관제가 판단): 성공이고 놓기·팔 복귀·그리퍼 열림이 모두 확인됨."""
        return (
            self.success
            and self.placement_confirmed
            and self.arm_returned
            and self.gripper_open
        )

    def to_dict(self) -> dict:
        return {
            "phase": str(self.phase),
            "success": self.success,
            "error_code": str(self.error_code) if self.error_code else None,
            "marker_id": self.marker_id,
            "duration_s": round(self.duration_s, 3),
            "placement_confirmed": self.placement_confirmed,
            "arm_returned": self.arm_returned,
            "gripper_open": self.gripper_open,
            "departure_ready": self.departure_ready,
            "detail": self.detail,
        }
