"""양팔 관절·카메라 이름과 순서. 학습 데이터(LeRobot bi_omx_follower)와 같아야 한다."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

SIDES = ("left", "right")  # left = OMX1, right = OMX2 (현장 robot.json)
ARM_IDS = {"left": "omx1", "right": "omx2"}
# 한 팔의 모터 순서(LeRobot omx_follower, ID 11~16)
JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
ARM_JOINTS = JOINTS[:5]  # 그리퍼 제외
# observation.state / action 12개: 왼팔 6개 다음 오른팔 6개
STATE_KEYS = tuple(f"{side}_{joint}.pos" for side in SIDES for joint in JOINTS)
WIDTH = len(STATE_KEYS)
GRIPPER_INDEXES = tuple(i for i, key in enumerate(STATE_KEYS) if "gripper" in key)
CAMERAS = ("front", "front_2", "wrist_left", "wrist_right")  # 현장 shelf 모델 4대
ROBOT_TYPE = "bi_omx_follower"


def join(arms: Mapping[str, Mapping[str, float]]) -> list[float]:
    """{"left": {joint: v}, "right": {...}} → 12개 목록."""
    return [float(arms[side][joint]) for side in SIDES for joint in JOINTS]


def split(values: Sequence[float]) -> dict[str, dict[str, float]]:
    """12개 목록 → {"left": {joint: v}, "right": {...}}."""
    if len(values) != WIDTH:
        raise ValueError(f"expected {WIDTH} values, got {len(values)}")
    n = len(JOINTS)
    return {
        side: {j: float(values[i * n + k]) for k, j in enumerate(JOINTS)}
        for i, side in enumerate(SIDES)
    }
