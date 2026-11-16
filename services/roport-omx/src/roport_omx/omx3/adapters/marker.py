"""ports.Marker 구현: 학습에 쓴 TargetMarker 그림 그대로, 이번 프레임 검출점을 함께 돌려준다."""

from __future__ import annotations

from .target_mark import TargetMarker


class ArucoTargetMarker(TargetMarker):
    """명령 1개 = 객체 1개. 가려진 프레임에는 마지막 위치에 원을 그린다(TargetMarker 동작)."""

    seen: tuple[float, float] | None = None

    def find(self, frame) -> tuple[float, float] | None:
        # TargetMarker.__call__도 이 find를 부르므로 검출을 한 번만 하고 결과를 남긴다
        p = super().find(frame)
        self.seen = None if p is None else (float(p[0]), float(p[1]))
        return self.seen

    def mark(self, frame):
        out = self(frame)
        return out, self.seen
