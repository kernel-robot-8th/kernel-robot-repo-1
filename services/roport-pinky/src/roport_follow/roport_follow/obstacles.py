"""인형 상자 → Nav2 비용 지도용 바닥 점(ROS 없음, 순수 계산).

좌표계: base_footprint(바퀴 축 가운데 바닥, X 앞, Y 왼쪽, Z 위, m). 카메라 바닥 좌표(floor_geom)의 원점은
렌즈 바로 아래 바닥이므로 렌즈 위치(cam_x_m, cam_y_m)만큼 옮긴다.

- 거리 기준은 비용 지도(obstacle_max_range)와 같은 바퀴 축(base_footprint 원점)이다.
- 표시 점(mark): 상자 아래 끝을 폭 방향으로 나눠 바닥에 투영하고, 안쪽(먼 쪽)으로 depth 만큼 줄을 더한다.
  높이 z_m(기본 5 cm)에 두어 ObstacleLayer 의 min_obstacle_height(0) 위에 있게 한다.
- 지움 점(clear): 카메라 시야 안 clear_range_m 거리에 부채꼴로 둔다. obstacle_max_range 보다 멀어 표시는 안 되고,
  센서 원점 → 점 광선이 그 사이 칸을 비운다(인형이 치워지면 사라짐). 인형이 없을 때도 매번 보낸다.
- 사람(human)은 시연에 쓰지 않으므로(11-19 결정) 기본 대상에서 뺀다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .floor_geom import CameraModel

OBSTACLE_CLASSES = ('adult', 'child')


@dataclass
class ObstacleConfig:
    cam_x_m: float = 0.035          # base_footprint → 렌즈 앞 거리(URDF 카메라 링크 약 3.5 cm, 실측 시 바꿈)
    cam_y_m: float = 0.0
    classes: tuple = OBSTACLE_CLASSES
    min_conf: float = 0.35
    max_range_m: float = 1.2        # 이보다 먼 인형은 표시 안 함(작아서 위치 오차 큼). nav2 obstacle_max_range 와 맞춤
    clear_range_m: float = 1.25     # 지움 점 거리(> max_range_m)
    spacing_m: float = 0.01         # 표시 점 간격(지역 비용 지도 해상도 1 cm)
    min_depth_m: float = 0.03       # 인형 앞뒤 두께(상자 폭으로 정하되 이 범위로 자름)
    max_depth_m: float = 0.08
    cut_extend_m: float = 0.05      # 상자 아래가 화면 아래 끝에 닿으면(발이 잘림) 로봇 쪽으로 더 칠함
    z_m: float = 0.05
    # 시야 가장자리는 지우지 않음. 광선은 바퀴 축에서 나가므로 화면 아래 모서리 쪽은 축 15 cm 밖도 카메라 사각이다
    # (3°면 d660 ±25° 부근 축 0.15~0.2 m 칸을 못 보는데 지움 — 11-19 검토). 6°면 시험 카메라·d660·6422 모두 0칸
    fov_margin_deg: float = 6.0
    edge_px: float = 2.0
    # 인형 기억(doll_memory) 모드: 시야 밖 방향도 매번 지우는 둘레 점의 끝 간격. 2 cm 면 축 0.62 m 안의 1 cm 칸은
    # 빠짐없이 지워지고 그 밖은 듬성듬성 지워진다(가까운 곳만 확실하면 됨, 점 수 약 350)
    ring_spacing_m: float = 0.02

    def validate(self):
        if not (0 < self.max_range_m < self.clear_range_m <= 3.0):
            raise ValueError('need 0 < max_range_m < clear_range_m <= 3.0')
        if not (0.002 <= self.spacing_m <= 0.05):
            raise ValueError('spacing_m must be 0.002..0.05')
        if not (0 < self.min_depth_m <= self.max_depth_m <= 0.3):
            raise ValueError('need 0 < min_depth_m <= max_depth_m <= 0.3')
        if not (0.0 <= self.min_conf <= 1.0):
            raise ValueError('min_conf must be 0..1')
        if not (0.005 <= self.ring_spacing_m <= 0.1):
            raise ValueError('ring_spacing_m must be 0.005..0.1')
        if not all(math.isfinite(v) for v in (self.cam_x_m, self.cam_y_m, self.z_m)):
            raise ValueError('camera offset and z must be finite')


@dataclass(frozen=True)
class DollObstacle:
    name: str
    conf: float
    x_m: float          # base_footprint 기준 가까운 끝 가운데
    y_m: float
    width_m: float
    cut: bool           # 상자 아래가 화면 아래 끝(발 잘림)
    points: tuple       # ((x, y, z), ...) base_footprint


def _ground(cam, u, v):
    return cam.pixel_to_ground(u, v)


def doll_obstacles(cam: CameraModel, dets, cfg: ObstacleConfig):
    """Detection 목록 → DollObstacle 목록(대상 클래스·신뢰도·거리 조건을 통과한 것만)."""
    out = []
    for d in dets:
        if d.name not in cfg.classes or d.conf < cfg.min_conf:
            continue
        x1, y1, x2, y2 = (float(v) for v in d.box)
        if not (x2 > x1 and y2 > y1):
            continue
        cut = y2 >= cam.height - cfg.edge_px
        v = min(y2, cam.height - 1.0)
        left, mid, right = _ground(cam, x1, v), _ground(cam, (x1 + x2) / 2, v), _ground(cam, x2, v)
        if mid is None or left is None or right is None:
            continue
        if math.hypot(mid[0] + cfg.cam_x_m, mid[1] + cfg.cam_y_m) > cfg.max_range_m:   # 비용 지도와 같은 축 기준
            continue
        width = math.dist(left, right)
        depth = min(max(width, cfg.min_depth_m), cfg.max_depth_m)
        n = max(2, int(math.ceil(width / cfg.spacing_m)) + 1)
        near = [_ground(cam, x1 + (x2 - x1) * i / (n - 1), v) for i in range(n)]
        near = [p for p in near if p is not None]
        pts = []
        back = -cfg.cut_extend_m if cut else 0.0
        rows = int(math.ceil((depth - back) / cfg.spacing_m)) + 1
        for gx, gy in near:
            r = math.hypot(gx, gy)
            if r < 1e-6:
                continue
            ux, uy = gx / r, gy / r                       # 렌즈에서 멀어지는 방향
            for k in range(rows):
                s = back + (depth - back) * k / max(rows - 1, 1)
                if r + s < 0.0:
                    continue
                pts.append((round(gx + ux * s + cfg.cam_x_m, 4), round(gy + uy * s + cfg.cam_y_m, 4), cfg.z_m))
        if pts:
            out.append(DollObstacle(d.name, float(d.conf), mid[0] + cfg.cam_x_m, mid[1] + cfg.cam_y_m,
                                    width, cut, tuple(pts)))
    return out


def clear_fov_deg(cam: CameraModel, cfg: ObstacleConfig):
    """지우는 시야의 (왼쪽 +, 오른쪽 +) 각도(도). 초점·주점으로 구하고 가장자리 fov_margin_deg 를 뺀다."""
    k = cam.K
    left = math.degrees(math.atan2(k[0, 2], k[0, 0])) - cfg.fov_margin_deg     # 화면 왼쪽 = +Y
    right = math.degrees(math.atan2(cam.width - k[0, 2], k[0, 0])) - cfg.fov_margin_deg
    return left, right


def clear_points(cam: CameraModel, cfg: ObstacleConfig):
    """카메라 시야 안 지움 점(base_footprint). 좌우 시야각은 초점·주점으로 구함(왜곡 무시 근사).

    광선 끝 간격 = 점 간격(spacing_m)의 0.8배: 광선 사이가 비용 지도 칸보다 벌어지면 그 칸은 안 지워진다
    (2° 간격으로는 0.5 m 에서 약 2 cm 벌어져 1 cm 칸이 남음 — 실제 costmap 노드 시험에서 확인).
    """
    left, right = clear_fov_deg(cam, cfg)
    step = math.degrees(0.8 * cfg.spacing_m / cfg.clear_range_m)
    n = max(2, int(math.ceil((left + right) / step)) + 1)
    pts = []
    for i in range(n):
        t = math.radians(-right + (left + right) * i / (n - 1))
        pts.append((round(cfg.cam_x_m + cfg.clear_range_m * math.cos(t), 4),
                    round(cfg.cam_y_m + cfg.clear_range_m * math.sin(t), 4), cfg.z_m))
    return pts


def clear_ring(cam: CameraModel, cfg: ObstacleConfig):
    """시야 밖 방향의 지움 점(base_footprint, 바퀴 축 둘레 clear_range_m 원에서 clear_points 의 부채꼴을 뺀 나머지).

    인형 기억 모드에서만 보낸다: 로봇 인형 층이 「노트북이 이번에 보낸 것」만 남기도록 둘레를 매번 지운다.
    카메라가 못 보는 곳도 지우므로, 거기 있는 인형은 doll_memory 가 기억해서 같은 묶음에 다시 넣어야 한다.
    """
    left, right = clear_fov_deg(cam, cfg)
    a0, a1 = math.radians(left), math.radians(360.0 - right)
    n = max(2, int(math.ceil((a1 - a0) * cfg.clear_range_m / cfg.ring_spacing_m)) + 1)
    return [(round(cfg.clear_range_m * math.cos(a0 + (a1 - a0) * i / (n - 1)), 4),
             round(cfg.clear_range_m * math.sin(a0 + (a1 - a0) * i / (n - 1)), 4), cfg.z_m) for i in range(n)]


def blind_distance(cam: CameraModel):
    """렌즈 아래 바닥 → 화면 아래 끝 가운데가 닿는 바닥까지 거리(m). 이 안은 카메라가 못 본다."""
    g = cam.pixel_to_ground(cam.width / 2, cam.height - 1)
    return None if g is None else g[0]


def cloud_points(cam, dets, cfg):
    """한 장 처리 결과 → (보낼 점 전체, 인형 목록). 점 = 표시 점 + 지움 점."""
    obs = doll_obstacles(cam, dets, cfg)
    pts = [p for o in obs for p in o.points]
    return pts + clear_points(cam, cfg), obs
