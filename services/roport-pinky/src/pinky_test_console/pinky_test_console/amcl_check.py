"""AMCL 위치 추정과 지도의 정합 확인 (ROS 의존성 없음, 단위 테스트 대상).

- 지도(OccupancyGrid) → 점유 칸까지 거리장(DistanceField)
- 라이다 스캔 → map 좌표 점 → 지도 벽에서 5·10 cm 안에 든 비율(정합 점수)
- 배치 데이터(roport_gz_world/layout/roport_airport.yaml)의 frame·nodes 읽기:
  실측 좌표(cm, 원점 = 콘센트 벽·통로 모서리) ↔ map 좌표(m)
- 기록: <amcl_dir>/amcl_<YYYYmmdd>.csv 에 한 줄씩 추가
"""

import csv
import math
import os
import re
import time
from collections import deque

import yaml

DEFAULT_AMCL_DIR = '~/roport_amcl'
OCCUPIED_MIN = 65          # OccupancyGrid 값 >= 65 를 점유로 봄(map_server occupied_thresh 0.65)
THRESHOLDS = (0.05, 0.10)  # m, 정합 점수 기준 거리
MAX_SCAN_POINTS = 360
INIT_STD_XY = 0.05         # m, 노드·실측 좌표로 주는 초기 위치 표준편차
INIT_STD_YAW_DEG = 5.0
_LABEL_RE = re.compile(r'^[A-Za-z0-9_-]{0,40}$')
CSV_FIELDS = [
    'time', 'label', 'truth_x_cm', 'truth_y_cm', 'truth_yaw_deg',
    'amcl_x_cm', 'amcl_y_cm', 'amcl_yaw_deg', 'err_xy_cm', 'err_yaw_deg',
    'std_x_cm', 'std_y_cm', 'std_yaw_deg', 'within_5cm', 'within_10cm', 'median_cm', 'points',
]


class AmclCheckError(ValueError):
    pass


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


# ---------------- 실측 ↔ map 좌표 ----------------
class Frame:
    """map = R(yaw) * (실측 cm / 100) + (x_m, y_m)  (roport_gz_world.layout.Frame 과 같은 식)."""

    def __init__(self, x_m=0.0, y_m=0.0, yaw_deg=0.0):
        self.x_m, self.y_m, self.yaw_deg = float(x_m), float(y_m), float(yaw_deg)

    def to_map(self, x_cm, y_cm, yaw_deg=0.0):
        a = math.radians(self.yaw_deg)
        x, y = x_cm / 100.0, y_cm / 100.0
        return (math.cos(a) * x - math.sin(a) * y + self.x_m,
                math.sin(a) * x + math.cos(a) * y + self.y_m,
                math.radians(yaw_deg + self.yaw_deg))

    def to_measured(self, x_m, y_m, yaw_rad=0.0):
        a = math.radians(self.yaw_deg)
        dx, dy = x_m - self.x_m, y_m - self.y_m
        return ((math.cos(a) * dx + math.sin(a) * dy) * 100.0,
                (-math.sin(a) * dx + math.cos(a) * dy) * 100.0,
                wrap_deg(math.degrees(yaw_rad) - self.yaw_deg))


def load_layout(path):
    """배치 yaml → (Frame, nodes). nodes: [{id, role, place, x_cm, y_cm, yaw_deg, nav2_switch}].

    yaw_deg 는 정차·전환 노드에 로봇이 도착했을 때의 방향 추정(마지막 연결 노드 → 이 노드).
    """
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or data.get('units') != 'cm':
        raise AmclCheckError(f'{path}: not a layout file (units: cm)')
    fr = data.get('frame') or {}
    try:
        frame = Frame(fr['x_m'], fr['y_m'], fr['yaw_deg'])
    except (KeyError, TypeError, ValueError):
        raise AmclCheckError(f'{path}: frame needs x_m, y_m, yaw_deg') from None
    items = (data.get('nodes') or {}).get('items') or []
    by_id = {n.get('id'): n for n in items}
    nodes = []
    for n in items:
        try:
            x, y = float(n['at'][0]), float(n['at'][1])
        except (KeyError, IndexError, TypeError, ValueError):
            raise AmclCheckError(f"{path}: node {n.get('id')!r} needs at: [x, y]") from None
        links = n.get('links') or []
        yaw = 0.0
        if len(links) == 1 and links[0] in by_id:
            px, py = by_id[links[0]]['at']
            yaw = math.degrees(math.atan2(y - py, x - px))
        nodes.append({'id': str(n.get('id')), 'role': str(n.get('role', '')),
                      'place': str(n.get('place', '')), 'x_cm': x, 'y_cm': y,
                      'yaw_deg': round(yaw, 1), 'nav2_switch': bool(n.get('nav2_switch'))})
    # Nav2 전환 노드를 맨 앞에
    nodes.sort(key=lambda k: (not k['nav2_switch'], k['id']))
    return frame, nodes


# ---------------- 거리장 ----------------
class DistanceField:
    """점유 칸까지의 거리(m). 칸마다 가장 가까운 점유 칸을 전파(brushfire)한다."""

    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        if width <= 0 or height <= 0 or len(data) != width * height:
            raise AmclCheckError('grid size and data length do not match')
        self.w, self.h, self.res = int(width), int(height), float(resolution)
        self.ox, self.oy = float(origin_x), float(origin_y)
        self.known = [v >= 0 for v in data]
        nearest = [-1] * (self.w * self.h)
        queue = deque()
        for i, v in enumerate(data):
            if v >= OCCUPIED_MIN:
                nearest[i] = i
                queue.append(i)
        self.has_obstacles = bool(queue)
        while queue:
            i = queue.popleft()
            cx, cy = i % self.w, i // self.w
            sx, sy = nearest[i] % self.w, nearest[i] // self.w
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nx, ny = cx + dx, cy + dy
                    if (dx == 0 and dy == 0) or not (0 <= nx < self.w and 0 <= ny < self.h):
                        continue
                    j = ny * self.w + nx
                    d_new = (nx - sx) ** 2 + (ny - sy) ** 2
                    if nearest[j] < 0:
                        nearest[j] = nearest[i]
                        queue.append(j)
                    else:
                        ox, oy = nearest[j] % self.w, nearest[j] // self.w
                        if d_new < (nx - ox) ** 2 + (ny - oy) ** 2:
                            nearest[j] = nearest[i]
                            queue.append(j)
        self.nearest = nearest

    def distance(self, x, y):
        """map 좌표 점 → 가장 가까운 점유 칸(정사각형)까지 거리(m). 지도 밖·점유 없음이면 None."""
        c = int(math.floor((x - self.ox) / self.res))
        r = int(math.floor((y - self.oy) / self.res))
        if not (0 <= c < self.w and 0 <= r < self.h) or self.nearest[r * self.w + c] < 0:
            return None
        k = self.nearest[r * self.w + c]
        kx, ky = k % self.w, k // self.w
        # 점 → 점유 칸 정사각형까지 거리 (칸 안이면 0)
        x0, y0 = self.ox + kx * self.res, self.oy + ky * self.res
        dx = max(x0 - x, 0.0, x - (x0 + self.res))
        dy = max(y0 - y, 0.0, y - (y0 + self.res))
        return math.hypot(dx, dy)


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, pose,
                   max_points=MAX_SCAN_POINTS):
    """라이다 거리 배열 + 라이다의 map 자세 (x, y, yaw) → map 좌표 점 목록."""
    px, py, pyaw = pose
    valid = [(i, r) for i, r in enumerate(ranges)
             if math.isfinite(r) and range_min <= r <= range_max]
    step = max(1, math.ceil(len(valid) / max_points)) if valid else 1
    out = []
    for i, r in valid[::step]:
        a = pyaw + angle_min + i * angle_increment
        out.append((px + r * math.cos(a), py + r * math.sin(a)))
    return out


def match_score(points, field, thresholds=THRESHOLDS):
    """점 목록 → {points, within: [비율...], median_m, classes}. 지도 밖 점은 '먼 점'으로 센다.

    classes: 점마다 처음 만족한 기준 번호(0 = 5 cm 안, 1 = 10 cm 안, len(thresholds) = 밖).
    """
    if not points:
        return {'points': 0, 'within': [None] * len(thresholds), 'median_m': None,
                'classes': []}
    dists = []
    for x, y in points:
        d = field.distance(x, y)
        dists.append(float('inf') if d is None else d)
    classes = [next((k for k, t in enumerate(thresholds) if d <= t), len(thresholds))
               for d in dists]
    ordered = sorted(dists)
    n = len(ordered)
    median = ordered[n // 2] if n % 2 else (ordered[n // 2 - 1] + ordered[n // 2]) / 2
    return {
        'points': n,
        'within': [round(sum(d <= t for d in dists) / n, 3) for t in thresholds],
        'median_m': None if math.isinf(median) else round(median, 4),
        'classes': classes,
    }


def pose_std(covariance):
    """PoseWithCovariance 6x6(행 우선) → (std_x m, std_y m, std_yaw rad)."""
    c = list(covariance)
    return tuple(math.sqrt(max(c[k], 0.0)) for k in (0, 7, 35))


# ---------------- 기록 ----------------
def validate_label(label):
    if not isinstance(label, str) or not _LABEL_RE.fullmatch(label):
        raise AmclCheckError('label must be 0-40 chars of A-Z a-z 0-9 _ -')
    return label


class AmclRecorder:
    def __init__(self, amcl_dir=DEFAULT_AMCL_DIR, clock=time.time):
        self.amcl_dir = os.path.abspath(os.path.expanduser(amcl_dir))
        self._clock = clock

    @property
    def path(self):
        return os.path.join(self.amcl_dir,
                            time.strftime('amcl_%Y%m%d.csv', time.localtime(self._clock())))

    def record(self, frame, amcl, score, label='', truth=None):
        """amcl: {x, y, yaw, std} (map m·rad), truth: (x_cm, y_cm, yaw_deg|None) | None."""
        validate_label(label)
        if amcl is None:
            raise AmclCheckError('no amcl_pose yet')
        ax, ay, ayaw = frame.to_measured(amcl['x'], amcl['y'], amcl['yaw'])
        row = {
            'time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self._clock())),
            'label': label,
            'amcl_x_cm': round(ax, 1), 'amcl_y_cm': round(ay, 1), 'amcl_yaw_deg': round(ayaw, 1),
            'std_x_cm': round(amcl['std'][0] * 100, 1), 'std_y_cm': round(amcl['std'][1] * 100, 1),
            'std_yaw_deg': round(math.degrees(amcl['std'][2]), 1),
            'within_5cm': score['within'][0], 'within_10cm': score['within'][1],
            'median_cm': None if score['median_m'] is None else round(score['median_m'] * 100, 1),
            'points': score['points'],
        }
        if truth is not None:
            tx, ty, tyaw = truth
            row.update({'truth_x_cm': tx, 'truth_y_cm': ty,
                        'err_xy_cm': round(math.hypot(ax - tx, ay - ty), 1)})
            if tyaw is not None:
                row.update({'truth_yaw_deg': tyaw, 'err_yaw_deg': round(wrap_deg(ayaw - tyaw), 1)})
        os.makedirs(self.amcl_dir, exist_ok=True)
        path = self.path
        new = not os.path.isfile(path)
        with open(path, 'a', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)
        return row

    def rows(self, limit=50):
        try:
            with open(self.path, newline='', encoding='utf-8') as f:
                return list(csv.DictReader(f))[-limit:]
        except OSError:
            return []
