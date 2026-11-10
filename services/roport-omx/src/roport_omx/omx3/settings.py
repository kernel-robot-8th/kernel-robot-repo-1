"""OMX 3 실행 설정."""

from __future__ import annotations

from dataclasses import dataclass

# ArUco 마커 ID 표 v1.4(docs/aruco/markers.yaml) group: package = PKG1~PKG8
PACKAGE_MARKER_IDS = frozenset(range(15, 23))


@dataclass(frozen=True)
class Omx3Settings:
    # 상차 요청 payload에서 대상 상자의 ArUco 마커 번호를 담은 필드(관제와 합의: 문자열)
    id_field: str = "package_id"
    package_ids: frozenset[int] = PACKAGE_MARKER_IDS
