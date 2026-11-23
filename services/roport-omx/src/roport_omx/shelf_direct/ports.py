"""실행부가 쓰는 장비·모델 인터페이스. 실물 어댑터와 테스트용 가짜를 바꿔 끼운다 (OMX3 ports.py와 같은 역할).

모든 메서드는 어댑터 안에서 시간 제한을 두고, 넘으면 예외를 던진다. 응답 없이 멈추면 runner가 끊지 못한다.
"""

from __future__ import annotations

import threading
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal, Protocol

LandMode = Literal["soft", "fast", "hold"]


class RobotNotReady(RuntimeError):
    """시작 전 점검(모터 응답, 한 바퀴 넘은 값, 포트) 실패. 토크를 켜기 전에 던진다."""


class SafeStartRefused(RuntimeError):
    """안전 시작이 움직이지 않기로 한 경우(범위 밖 관절, 첫 목표와 차이가 너무 큼)."""


class PolicyUnavailable(RuntimeError):
    """추론 서버가 정해진 횟수 안에 답하지 않았거나 응답이 잘못됨."""


@dataclass(frozen=True)
class Observation:
    state: tuple[float, ...]  # contract.STATE_KEYS 순서 12개
    images: Mapping[str, Any] = field(default_factory=dict)  # 카메라 이름 → HxWx3 RGB
    at: float = 0.0  # time.time() (판정 기록용)


class ArmIO(Protocol):
    """한 팔의 모터 버스(정규화 단위: 관절 -100~100, 그리퍼 0~100)."""

    side: str

    def read(self) -> dict[str, float]:
        """Present_Position {joint: value}."""

    def write_goals(self, goals: Mapping[str, float]) -> None: ...

    def set_pwm(self, ratio: float) -> None:
        """Goal_PWM 출력 제한(0~1). 실패한 모터는 건너뛴다."""

    def torque_off(self) -> None: ...

    def reboot(self, joint: str) -> None:
        """회전 관절의 다회전 값을 리셋(재부팅) 후 토크를 다시 켠다."""

    def close(self, *, torque_off: bool) -> None: ...


class Robot(Protocol):
    freeze: threading.Event  # 비상 정지: 이동을 멈추고 토크를 켠 채 release까지 기다림
    release: threading.Event

    def connect(self) -> None:
        """시작 전 점검 → 버스·카메라 연결 → 출력 제한. 실패하면 RobotNotReady."""

    def observe(self) -> Observation: ...

    def send(self, action: Sequence[float]) -> None:
        """12개 목표. 안전 시작(저속 접근) 중이면 제한된 목표를 보낸다."""

    def start_poses(self) -> dict[str, dict[str, float]]:
        """팔마다 실행을 시작한 자세(첫 목표를 보내기 직전 실측). 아직 없으면 빈 dict."""

    def land(self, mode: LandMode) -> None:
        """soft: 처음 자세로 돌아가 착지·토크 끔. fast: 제자리에서 출력만 줄이고 토크 끔.
        hold: 토크를 켠 채 포트만 닫음(정렬기 넘겨받기)."""

    def disconnect(self) -> None: ...


class Policy(Protocol):
    def new_episode(self, task: str) -> None: ...

    def act(self, observation: Observation) -> Sequence[float] | None:
        """이번 주기의 목표 12개. 아직 없으면 None(이번 주기는 보내지 않음). 추론 실패는 PolicyUnavailable."""
