"""ArUco: 고객 카드(DICT_4X4_50 ID 35~49, 팀 마커 표의 스마트카트 고객 칸) 거리·방향, 지연 측정용 화면 시계(DICT_4X4_250)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CARD_DICT = 'DICT_4X4_50'
CARD_IDS = range(35, 50)     # 지금 쓰는 카드 = 35·36(스마트카트 2대)
CARD_SIZE_M = 0.030
CLOCK_DICT = 'DICT_4X4_250'
CLOCK_STEP_MS = 40          # 25 Hz 로 바뀌는 시계 마커(모니터 60 Hz 에서 프레임마다 그려짐)
CLOCK_MOD = 250


def _detector(name, subpix=False):
    """subpix: 모서리 서브픽셀 보정. 30 mm 카드는 화면에서 20 px 안팎이라 모서리 1 px 이 거리 5 % 가 된다
    (합성 영상 0.9 m: 5.5 → 2.6 cm, 1.2 m: 3.9 → 1.0 cm, 11-19 test_follow)."""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    # OpenCV 4.6(apt, CI)은 DetectorParameters() 가 비어 있어 인식에서 segfault → 옛 생성 함수
    p = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
         else cv2.aruco.DetectorParameters_create())
    if subpix:
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(cv2.aruco, 'ArucoDetector'):
        det = cv2.aruco.ArucoDetector(d, p)
        return lambda img: det.detectMarkers(img)
    return lambda img: cv2.aruco.detectMarkers(img, d, parameters=p)


@dataclass(frozen=True)
class Card:
    id: int
    corners: np.ndarray     # 4x2 화소
    dist_m: float           # 카메라 → 카드 중심(광선 거리)
    x_m: float              # 카메라 좌표 오른쪽 +
    z_m: float              # 카메라 좌표 앞
    side_px: float          # 화면에서 한 변 평균 길이
    y_m: float = math.nan   # 카메라 좌표 아래 +(바닥 투영에 씀)


class CardDetector:
    def __init__(self, k, dist, size_m=CARD_SIZE_M, ids=CARD_IDS):
        self.detect = _detector(CARD_DICT, subpix=True)
        self.k, self.dist = np.asarray(k, float), np.asarray(dist, float)
        self.size, self.ids = float(size_m), set(ids)

    def __call__(self, image):
        import cv2
        corners, ids, _ = self.detect(image)
        out = []
        if ids is None:
            return out
        h = self.size / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
        for c, i in zip(corners, ids.ravel()):
            if int(i) not in self.ids:
                continue
            c = c.reshape(4, 2)
            ok, _, t = cv2.solvePnP(obj, c, self.k, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            t = t.ravel()
            side = float(np.mean([np.linalg.norm(c[j] - c[(j + 1) % 4]) for j in range(4)]))
            out.append(Card(int(i), c, float(np.linalg.norm(t)), float(t[0]), float(t[2]), side, float(t[1])))
        return out


# ---- 지연 측정: 노트북 화면에 시각을 담은 마커를 띄우고 로봇 카메라로 찍는다(시계 맞춤 필요 없음) ----
def clock_id(now_ms):
    return int(now_ms // CLOCK_STEP_MS) % CLOCK_MOD


def latency_ms(seen_id, now_ms):
    """찍힌 시계 마커 ID 와 받은 시각 → 화면에서 노트북 수신까지 지연(ms, ±CLOCK_STEP_MS)."""
    return ((int(now_ms // CLOCK_STEP_MS) - int(seen_id)) % CLOCK_MOD) * CLOCK_STEP_MS


def clock_image(now_ms, size_px=600):
    import cv2
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CLOCK_DICT))
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    inner = int(size_px * 0.8)
    m = gen(d, clock_id(now_ms), inner)
    img = np.full((size_px, size_px), 255, np.uint8)
    o = (size_px - inner) // 2
    img[o:o + inner, o:o + inner] = m
    return img


class ClockReader:
    def __init__(self):
        self.detect = _detector(CLOCK_DICT)

    def __call__(self, image):
        _, ids, _ = self.detect(image)
        return None if ids is None or len(ids) != 1 else int(ids.ravel()[0])


def percentile(values, q):
    if not values:
        return math.nan
    s = sorted(values)
    return s[min(len(s) - 1, int(round(q / 100 * (len(s) - 1))))]
