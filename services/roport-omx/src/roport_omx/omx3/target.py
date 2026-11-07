"""payload의 대상 ID → ArUco 마커 번호.

번호(17), 숫자 문자열("17"), 표에 있는 이름("PKG3")을 모두 받는다.
찾을 필드 이름과 이름 표는 ``Omx3Settings``에서 바꾼다.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

MARKER_ID_MIN, MARKER_ID_MAX = 0, 49  # DICT_4X4_50


class InvalidTarget(ValueError):
    """대상 ID가 없거나 해석할 수 없다. 동작 없이 거절한다."""


def resolve_marker_id(
    payload: Mapping,
    fields: Sequence[str] = ("marker_id", "package_id"),
    table: Mapping[str, int] | None = None,
) -> int:
    """``fields`` 순서대로 처음 값이 있는 필드를 마커 번호로 바꾼다."""
    for name in fields:
        if payload.get(name) is not None:
            return to_marker_id(payload[name], table)
    raise InvalidTarget(f"대상 ID 필드가 없습니다: {', '.join(fields)}")


def to_marker_id(value, table: Mapping[str, int] | None = None) -> int:
    if isinstance(value, bool):  # bool은 int의 하위 타입이라 먼저 막는다
        raise InvalidTarget("대상 ID가 bool입니다")
    if isinstance(value, int):
        marker = value
    elif isinstance(value, str):
        text = value.strip()
        if table and text in table:
            marker = table[text]
        elif text.isdecimal():
            marker = int(text)
        else:
            raise InvalidTarget(f"알 수 없는 대상 ID: {value!r}")
    else:
        raise InvalidTarget(f"대상 ID 형식이 아닙니다: {type(value).__name__}")
    if not MARKER_ID_MIN <= marker <= MARKER_ID_MAX:
        raise InvalidTarget(f"마커 번호 범위 밖: {marker}")
    return marker
