"""실행부가 쓰는 장비·모델 인터페이스. 실물(5~8단계 어댑터)과 가짜(테스트)를 바꿔 끼운다."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class Observation:
    state: Sequence[float]  # JOINTS 순서 6개, omx_follower 정규화 단위
    front: Any  # 정면 카메라 원본 프레임(640x480 RGB). 원은 아직 안 그려짐


class Robot(Protocol):
    def ready(self) -> str | None:
        """명령을 받을 수 있으면 None, 아니면 이유."""

    def observe(self) -> Observation: ...

    def send(self, action: Sequence[float]) -> None:
        """JOINTS 순서 절대 목표값."""

    def hold(self) -> None:
        """현재 자세를 유지한다. 그리퍼를 열거나 토크를 끄지 않는다(물품 낙하 방지)."""


class Marker(Protocol):
    """명령 1개 = 객체 1개. 가려지면 마지막 위치를 유지하는 상태가 있다."""

    def reset(self) -> None: ...

    def find(self, frame: Any) -> tuple[float, float] | None:
        """상태를 바꾸지 않고 마커 중심(px)만 찾는다."""

    def mark(self, frame: Any) -> tuple[Any, tuple[float, float] | None]:
        """원을 그린 사본과, 이번 프레임에서 찾은 마커 중심(못 찾으면 None)."""


class Policy(Protocol):
    def reset(self) -> None: ...

    def act(self, state: Sequence[float], marked_front: Any) -> Sequence[float]:
        """다음 한 스텝의 JOINTS 순서 절대 목표값."""


class PolicyUnavailable(RuntimeError):
    """추론 서버 응답 없음·유실이 허용 시간을 넘음. runner가 INFERENCE_UNAVAILABLE로 보고한다."""
