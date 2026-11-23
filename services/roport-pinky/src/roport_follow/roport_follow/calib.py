"""체스보드 보정(ROS 없음).

1) 렌즈: 여러 각도의 체스보드 사진 → 초점·주점·왜곡(cv2.calibrateCamera).
2) 바닥: 체스보드를 로봇 앞 바닥에 평평하게 둔 사진 한 장 → 바닥 기준 카메라 높이·숙임·roll.
   보드가 바닥 어디에 있든 높이·숙임은 같은 값이 나온다(평면만 쓰기 때문).
보드 = data/calib/chessboard_9x6_25mm.pdf(안쪽 모서리 9×6, 칸 25 mm).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

PATTERN = (9, 6)          # 안쪽 모서리 (열, 행)
SQUARE_M = 0.025


def board_points(pattern=PATTERN, square_m=SQUARE_M):
    cols, rows = pattern
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return pts


def find_corners(gray, pattern=PATTERN):
    """체스보드 안쪽 모서리(N×1×2, 화소). 못 찾으면 None."""
    import cv2
    # 바닥에 눕힌 보드는 원근이 심해 SB(섹터 기반, 더 정확)부터 시도하고, 안 되면 예전 방식
    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if ok:
        return corners.astype(np.float32)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)


def calibrate_intrinsics(corner_sets, image_size, pattern=PATTERN, square_m=SQUARE_M, fix_k3=True):
    """corner_sets: find_corners 결과 목록(10장 이상 권장) → (K, dist, rms).

    fix_k3(기본): 세 번째 방사 왜곡 계수를 0으로 고정. 이 카메라는 낮게 숙여져 있어 보드 사진이 화면 위쪽에
    몰리기 쉬운데, k3까지 풀면 사진이 없는 화면 아래 모서리 보정이 과해진다(6422 11-16 시험: 10~13 px → 3~4 px).
    """
    import cv2
    if len(corner_sets) < 5:
        raise ValueError(f'need at least 5 chessboard views, got {len(corner_sets)}')
    obj = [board_points(pattern, square_m)] * len(corner_sets)
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    rms, k, dist, _, _ = cv2.calibrateCamera(obj, corner_sets, image_size, None, None, flags=flags)
    return k, dist.ravel(), float(rms)


@dataclass(frozen=True)
class FloorPose:
    height_m: float       # 바닥 위 렌즈 높이(보드 두께 더함)
    pitch_deg: float      # 아래로 숙인 각(+)
    roll_deg: float       # 광축 둘레 회전(화면 시계 방향 +)
    board_dist_m: float   # 렌즈 아래 바닥 → 보드 첫 모서리 수평 거리(참고)
    reproj_px: float      # 재투영 오차(평균 화소)


def floor_pose(corners, k, dist, board_thickness_m=0.0, pattern=PATTERN, square_m=SQUARE_M):
    """바닥에 평평하게 둔 체스보드의 모서리 → 바닥 기준 카메라 자세."""
    import cv2
    obj = board_points(pattern, square_m)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, np.asarray(k, float), np.asarray(dist, float))
    if not ok:
        raise ValueError('solvePnP failed')
    proj, _ = cv2.projectPoints(obj, rvec, tvec, np.asarray(k, float), np.asarray(dist, float))
    reproj = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)))
    r, _ = cv2.Rodrigues(rvec)
    center = (-r.T @ tvec).ravel()                 # 보드 좌표계에서 렌즈 위치
    up = np.array([0.0, 0.0, 1.0 if center[2] > 0 else -1.0])   # 렌즈 쪽 법선
    axis = r.T @ np.array([0.0, 0.0, 1.0])         # 광축(보드 좌표)
    x_cam = r.T @ np.array([1.0, 0.0, 0.0])        # 화면 오른쪽(보드 좌표)
    y_cam = r.T @ np.array([0.0, 1.0, 0.0])        # 화면 아래(보드 좌표)
    height = abs(center[2]) + board_thickness_m
    pitch = math.degrees(math.asin(float(np.clip(-axis @ up, -1, 1))))
    # roll(floor_geom.CameraModel 과 같은 정의): x·up = -sin r·cos p, y·up = -cos r·cos p
    roll = math.degrees(math.atan2(-float(x_cam @ up), -float(y_cam @ up)))
    foot = center - up * abs(center[2])
    board_dist = float(np.linalg.norm(foot[:2]))
    return FloorPose(height, pitch, roll, board_dist, reproj)
