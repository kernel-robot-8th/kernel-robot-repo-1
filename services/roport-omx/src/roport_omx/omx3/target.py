"""상차 요청 payload의 ``package_id`` → 상자의 ArUco 마커 번호.

관제와 합의한 형식: ``package_id``는 상자에 붙은 ArUco 마커 번호의 문자열(예: ``"17"``).
포장물품 마커(ArUco v1.4 PKG1~PKG8 = 15~22)만 받는다. 트레이·바닥·선반·예비 마커는
상차 위치 근처에 실제로 있으므로 팔이 엉뚱한 대상을 고르지 않게 거절한다.
"""

from __future__ import annotations

import re
from collections.abc import Mapping

from .settings import PACKAGE_MARKER_IDS

_NUMBER = re.compile(r"[1-9]?[0-9]")  # ASCII만, 앞자리 0·공백·부호 없음


class InvalidTarget(ValueError):
    """대상 ID가 없거나 포장물품 마커가 아니다. 동작 없이 거절한다."""


def resolve_marker_id(
    payload,
    field: str = "package_id",
    allowed: frozenset[int] = PACKAGE_MARKER_IDS,
) -> int:
    if not isinstance(payload, Mapping):
        raise InvalidTarget(f"payload가 객체가 아닙니다: {type(payload).__name__}")
    value = payload.get(field)
    if not isinstance(value, str):
        raise InvalidTarget(f"{field}는 문자열이어야 합니다: {value!r}")
    if not _NUMBER.fullmatch(value):
        raise InvalidTarget(f"{field}가 마커 번호 문자열이 아닙니다: {value!r}")
    marker = int(value)
    if marker not in allowed:
        raise InvalidTarget(f"포장물품 마커가 아닙니다: {marker}")
    return marker
