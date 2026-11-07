"""OMX 3 실행 설정."""

from __future__ import annotations

from dataclasses import dataclass, field

# ArUco 마커 ID 표 v1.4(docs/aruco/markers.yaml)의 포장물품 PKG1~PKG8 = ID 15~22
DEFAULT_ID_TABLE = {f"PKG{n}": 14 + n for n in range(1, 9)}


@dataclass(frozen=True)
class Omx3Settings:
    # payload에서 대상 ID를 찾을 필드(앞에서부터)와 이름 → 마커 번호 표.
    # 관제와 형식이 정해지면 이 값만 바꾼다.
    id_fields: tuple[str, ...] = ("marker_id", "package_id")
    id_table: dict[str, int] = field(default_factory=lambda: dict(DEFAULT_ID_TABLE))
