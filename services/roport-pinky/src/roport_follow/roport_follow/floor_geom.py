"""바닥 기하: 카메라 화소 ↔ 바닥 위 점(ROS 없음, numpy·OpenCV만).

좌표계
- 카메라: OpenCV 관례(x 오른쪽, y 아래, z 앞).
- 바닥(F): 원점 = 렌즈 바로 아래 바닥, X 앞, Y 왼쪽, Z 위. 단위 m.
- 카메라는 높이 height_m, 아래로 pitch_rad 숙임, 광축 기준 roll_rad(시계 방향 +) 회전.

YOLO 상자 아래 끝(발이 바닥에 닿는 점)은 바닥 위에 있다고 보고 거리·방향을 구한다.
바닥이 평평하다는 가정이 깨지거나(턱, 카펫 두께) 차체가 기울면 오차가 커진다.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

FRAME_W, FRAME_H = 640, 480


def _default_k():
    # 10-31 실측(100 mm 마커 58 cm = 93 px → 초점 약 540 px), 주점은 화면 가운데로 가정
    return [[540.0, 0.0, FRAME_W / 2], [0.0, 540.0, FRAME_H / 2], [0.0, 0.0, 1.0]]


@dataclass
class CameraModel:
    k: list = field(default_factory=_default_k)          # 3x3
    dist: list = field(default_factory=lambda: [0.0] * 5)
    width: int = FRAME_W
    height: int = FRAME_H
    height_m: float = 0.063                                # 렌즈 높이(10-31 실측)
    # 10-31 「58 cm 앞 벽에서 12 cm까지 보임」을 화면 위 끝으로 보고 계산한 값(추정)
    pitch_deg: float = 18.35
    roll_deg: float = 0.0
    source: str = 'default (10-31 estimate, not calibrated)'

    # ---- 불러오기·저장 ----
    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).expanduser().read_text())
        known = {k: data[k] for k in cls.__dataclass_fields__ if k in data}
        cam = cls(**known)
        cam.validate()
        return cam

    def save(self, path, extra=None):
        self.validate()
        data = asdict(self)
        if extra:
            data.update(extra)
        p = Path(path).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')

    def validate(self):
        k = np.asarray(self.k, float)
        if k.shape != (3, 3) or not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError('k must be a finite 3x3 camera matrix with positive focal lengths')
        d = np.asarray(self.dist, float).ravel()
        if d.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(d)):
            raise ValueError('dist must have 4, 5, 8, 12 or 14 finite values')
        if not (0.005 <= self.height_m <= 1.0):
            raise ValueError('height_m must be 0.005..1.0 m')
        if not (-5.0 <= self.pitch_deg <= 80.0):
            raise ValueError('pitch_deg must be -5..80 (down = +)')
        if not (-30.0 <= self.roll_deg <= 30.0):
            raise ValueError('roll_deg must be -30..30')
        if not (self.width > 0 and self.height > 0):
            raise ValueError('image size must be positive')

    # ---- 기하 ----
    @property
    def K(self):
        return np.asarray(self.k, float)

    @property
    def D(self):
        return np.asarray(self.dist, float).ravel()

    def rotation(self):
        """카메라 축(열 = x_c, y_c, z_c)을 바닥 좌표로 나타낸 3x3 행렬 R_fc."""
        p, r = math.radians(self.pitch_deg), math.radians(self.roll_deg)
        z = np.array([math.cos(p), 0.0, -math.sin(p)])          # 광축
        x0 = np.array([0.0, -1.0, 0.0])                          # 화면 오른쪽 = 바닥 -Y
        y0 = np.cross(z, x0)                                     # 화면 아래
        # roll: 광축 둘레로 회전(화면이 시계 방향으로 돌아간 만큼 +)
        x = math.cos(r) * x0 + math.sin(r) * y0
        y = -math.sin(r) * x0 + math.cos(r) * y0
        return np.column_stack([x, y, z])

    def undistort(self, uv):
        import cv2
        pts = np.asarray(uv, float).reshape(-1, 1, 2)
        n = cv2.undistortPoints(pts, self.K, self.D)
        return n.reshape(-1, 2)

    def pixel_to_ground(self, u, v):
        """화소 (u, v) → 바닥 (X 앞, Y 왼쪽) m. 수평선 위(바닥과 안 만남)면 None."""
        xn, yn = self.undistort([[u, v]])[0]
        d = self.rotation() @ np.array([xn, yn, 1.0])
        if d[2] >= -1e-6:
            return None
        t = self.height_m / -d[2]
        return float(t * d[0]), float(t * d[1])

    def ground_to_pixel(self, x, y, z=0.0):
        """바닥 좌표 점 → 화소 (u, v). 카메라 뒤쪽이면 None."""
        import cv2
        r = self.rotation()
        pc = r.T @ (np.array([x, y, z]) - np.array([0.0, 0.0, self.height_m]))
        if pc[2] <= 1e-6:
            return None
        rvec = np.zeros(3)
        uv, _ = cv2.projectPoints(pc.reshape(1, 1, 3), rvec, np.zeros(3), self.K, self.D)
        return float(uv[0, 0, 0]), float(uv[0, 0, 1])

    def horizon_v(self, u=None):
        """화면 가운데 열에서 수평선(무한히 먼 바닥) 화소 행. 왜곡은 무시한 근사."""
        k = self.K
        return float(k[1, 2] - k[1, 1] * math.tan(math.radians(self.pitch_deg)))

    def visible_top_height(self, dist_m):
        """앞 dist_m 거리의 연직면에서 화면 위 끝에 보이는 높이(m). 왜곡·roll 무시 근사."""
        k = self.K
        up = math.atan2(k[1, 2], k[1, 1]) - math.radians(self.pitch_deg)   # 위 끝 광선의 올려본 각
        return self.height_m + dist_m * math.tan(up)


@dataclass(frozen=True)
class GroundObject:
    x_m: float          # 앞 거리(렌즈 아래 바닥 기준)
    y_m: float          # 왼쪽 +
    dist_m: float       # 수평 거리
    bearing_deg: float  # 왼쪽 +
    width_m: float      # 상자 폭을 바닥 거리에서 환산


def box_to_ground(cam: CameraModel, box):
    """YOLO 상자 (x1, y1, x2, y2) 화소 → 아래 끝 가운데 점의 바닥 위치·폭. 수평선 위면 None.

    상자 아래 끝이 화면 아래 가장자리에 닿아 있으면 발이 잘린 것이라 실제로는 더 가깝다(호출 쪽에서 표시).
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    if not (x2 > x1 and y2 > y1):
        raise ValueError('box must have x2 > x1 and y2 > y1')
    mid = cam.pixel_to_ground((x1 + x2) / 2, y2)
    if mid is None:
        return None
    left = cam.pixel_to_ground(x1, y2)
    right = cam.pixel_to_ground(x2, y2)
    width = math.dist(left, right) if left and right else float('nan')
    x, y = mid
    return GroundObject(x, y, math.hypot(x, y), math.degrees(math.atan2(y, x)), width)


def distance_from_height(cam: CameraModel, box, object_height_m):
    """키를 아는 물체: 거리 ≈ 초점 × 키 ÷ 상자 세로(화소). 머리·발이 잘리면 틀린다(근사)."""
    x1, y1, x2, y2 = (float(v) for v in box)
    h_px = y2 - y1
    if h_px <= 0 or object_height_m <= 0:
        return None
    return float(cam.K[1, 1] * object_height_m / h_px)


def fit_pose(cam: CameraModel, samples, fix_height_m=None):
    """줄자 표시 클릭으로 숙임(과 높이)을 맞춘다. samples = [(u, v, 앞 거리 m)].

    렌즈 아래 바닥에서 앞쪽으로 잰 거리(X)를 쓴다. 높이를 줄자로 쟀으면 fix_height_m 로 고정(점 1개로 충분),
    아니면 점 2개 이상(가까운 것·먼 것 섞기)으로 높이·숙임을 같이 맞춘다. → (height_m, pitch_deg, rms_cm)
    """
    if not samples:
        raise ValueError('need at least one (u, v, distance) sample')
    if fix_height_m is None and len(samples) < 2:
        raise ValueError('need 2+ samples to fit height and pitch (or give fix_height_m)')
    uv = np.array([[s[0], s[1]] for s in samples], float)
    true = np.array([s[2] for s in samples], float)
    if np.any(true <= 0):
        raise ValueError('distances must be positive')
    n = cam.undistort(uv)                                   # (N, 2) 정규 좌표

    def dists(h, pitch_deg):
        p = np.radians(pitch_deg)[:, None]                  # (P, 1)
        r = np.radians(cam.roll_deg)
        yn = math.sin(r) * n[:, 0] + math.cos(r) * n[:, 1]   # roll 보정 뒤 세로 정규 좌표(N,)
        dz = -yn * np.cos(p) - np.sin(p)                    # 바닥 방향 성분(아래 -)
        dx = np.cos(p) - yn * np.sin(p)
        with np.errstate(divide='ignore', invalid='ignore'):
            x = np.where(dz < -1e-6, h * dx / -dz, np.inf)
        return x                                            # (P, N)

    heights = [fix_height_m] if fix_height_m else np.arange(0.03, 0.15, 0.0005)
    best = None
    for lo, hi, step in ((0.0, 60.0, 0.1), (None, None, 0.005)):
        if lo is None:
            lo, hi = best[1] - 0.2, best[1] + 0.2
        pitches = np.arange(lo, hi, step)
        for h in heights:
            err = dists(h, pitches) - true
            rms = np.sqrt(np.mean(err ** 2, axis=1))
            i = int(np.argmin(rms))
            if best is None or rms[i] < best[2]:
                best = (float(h), float(pitches[i]), float(rms[i]))
        if fix_height_m is None:
            heights = np.arange(max(0.01, best[0] - 0.001), best[0] + 0.001, 0.0001)
    return best[0], best[1], best[2] * 100
