"""OMX 3 실행 설정."""

from __future__ import annotations

from dataclasses import dataclass

# ArUco 마커 ID 표 v1.4(docs/aruco/markers.yaml) group: package = PKG1~PKG8
PACKAGE_MARKER_IDS = frozenset(range(15, 23))

# 관절 상태·동작의 순서(LeRobot omx_follower, 학습 데이터와 같음)
JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class Omx3Settings:
    # 상차 요청 payload에서 대상 상자의 ArUco 마커 번호를 담은 필드(관제와 합의: 문자열)
    id_field: str = "package_id"
    package_ids: frozenset[int] = PACKAGE_MARKER_IDS

    # 완료 판정. 값은 omx_follower 정규화 단위(관절 -100~100, 그리퍼 0~100).
    # 학습 데이터의 시작·끝 자세. 바꾸면 재학습이 필요하다.
    init_pose: tuple[float, ...] = (0.0, -62.3, 53.9, 42.2, 0.0, 56.5)
    # 아래 셋은 실측 전 기본값(HANDOFF_marker_act.md·MODEL_USAGE.md 기준). 현장에서 다시 정한다.
    init_tolerance: float = 5.0  # 관절 5개 각각 허용 오차
    # init 그리퍼(56.5)보다 충분히 낮게: 읽기 잡음으로 완료를 놓치지 않게 한다(실측 전 임시값, #65 리뷰)
    gripper_open_min: float = 50.0
    # 정면 영상(640x480)에서 정차한 Pinky 트레이(12.5 x 15.2 cm) 영역 (x0, y0, x1, y1) px, 경계 포함.
    # Pinky N01(P4) 정밀 도킹 자세(PR #59)에 따라 바뀐다. 도킹 값이 바뀌면 다시 재고,
    # 도킹 반복 오차(약 1 cm·1°)만큼 픽셀 여유를 둔다.
    tray_region: tuple[int, int, int, int] = (399, 0, 567, 140)

    # 실행 상태기계
    fps: int = 30  # 학습 데이터·정책 주기
    timeout_s: float = 40.0  # 정책 루프 최대 시간(동작 1회 약 25~30초)
    done_hold_frames: int = 15  # 완료 조건이 연속으로 유지돼야 하는 프레임 수(0.5초)
    home_step: float = 1.0  # init 이동 때 한 주기당 관절 최대 변화량(정규화 단위)
    # HOMING 도착 기준. 정책 시작 자세를 학습 시작값에 맞춘다(실측 전 기본값)
    home_tolerance: float = 1.0
    home_timeout_s: float = 20.0

    def __post_init__(self) -> None:
        if len(self.init_pose) != len(JOINTS):
            raise ValueError(
                f"init_pose는 {len(JOINTS)}개여야 합니다: {len(self.init_pose)}"
            )
