"""완료 판정: 관절값과 마커 픽셀 위치만 쓴다(장비·영상 처리 없음).

완료 = 대상 마커가 트레이 영역에 보이고, 팔이 init 근처이며, 그리퍼가 열려 있다.
연속 프레임 유지는 실행부(runner)가, 마커 검출은 마커 어댑터가 맡는다.
"""

from __future__ import annotations

from collections.abc import Sequence

from .settings import JOINTS, Omx3Settings

Point = tuple[float, float]


def _check(state: Sequence[float]) -> None:
    if len(state) != len(JOINTS):
        raise ValueError(f"state는 {len(JOINTS)}개여야 합니다: {len(state)}")


def near_init(state: Sequence[float], s: Omx3Settings) -> bool:
    """관절 5개(그리퍼 제외)가 init 자세 허용 오차 안인가."""
    _check(state)
    return all(
        abs(float(v) - t) <= s.init_tolerance
        for v, t in zip(state[:5], s.init_pose[:5], strict=True)
    )


def gripper_open(state: Sequence[float], s: Omx3Settings) -> bool:
    _check(state)
    return float(state[5]) >= s.gripper_open_min


def in_tray(point: Point | None, s: Omx3Settings) -> bool:
    """마커 중심(px)이 트레이 영역 안인가. 못 찾았으면(None) False."""
    if point is None:
        return False
    x0, y0, x1, y1 = s.tray_region
    return x0 <= point[0] <= x1 and y0 <= point[1] <= y1


def is_done(state: Sequence[float], point: Point | None, s: Omx3Settings) -> bool:
    return in_tray(point, s) and near_init(state, s) and gripper_open(state, s)
