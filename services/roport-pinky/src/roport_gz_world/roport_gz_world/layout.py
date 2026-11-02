"""배치 데이터(layout/*.yaml) 읽기·검사·좌표 변환.

배치 데이터는 실측 좌표계(cm)로 적고, world·지도는 SLAM 지도(map) 좌표계(m)로 만든다.
바닥 테이프는 노드 그래프(nodes)의 연결 구간과 정차 가로선에서 만든다.
map = R(yaw) * (실측 / 100) + (x_m, y_m)
"""

import math
from dataclasses import dataclass

import yaml


class LayoutError(ValueError):
    pass


@dataclass(frozen=True)
class Frame:
    x_m: float
    y_m: float
    yaw_deg: float

    def to_world(self, x_cm, y_cm):
        """실측 (cm) → world/map (m)."""
        yaw = math.radians(self.yaw_deg)
        x, y = x_cm / 100.0, y_cm / 100.0
        return (math.cos(yaw) * x - math.sin(yaw) * y + self.x_m,
                math.sin(yaw) * x + math.cos(yaw) * y + self.y_m)

    def to_measured(self, x_m, y_m):
        """world/map (m) → 실측 (cm)."""
        yaw = math.radians(self.yaw_deg)
        dx, dy = x_m - self.x_m, y_m - self.y_m
        return ((math.cos(yaw) * dx + math.sin(yaw) * dy) * 100.0,
                (-math.sin(yaw) * dx + math.cos(yaw) * dy) * 100.0)


@dataclass(frozen=True)
class Box:
    """world 좌표의 상자. 크기·위치 단위 m, yaw rad."""

    name: str
    cx: float
    cy: float
    cz: float
    sx: float
    sy: float
    sz: float
    yaw: float
    provisional: bool = False


@dataclass(frozen=True)
class Node:
    """라인 그래프 노드. 좌표는 실측 cm(테이프 중심선)."""

    id: str
    x: float
    y: float
    role: str
    links: tuple
    place: str = ''
    bar: str = ''              # 정차 가로선 방향: 'x'(가로) · 'y'(세로) · ''(없음)
    nav2_switch: bool = False
    provisional: bool = False


@dataclass(frozen=True)
class MapSpec:
    resolution: float
    origin: tuple
    width: int
    height: int
    lidar_height: float        # cm


def _point(value, where):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value)):
        raise LayoutError(f'{where}: point must be [x, y] numbers, got {value!r}')
    return float(value[0]), float(value[1])


def _positive(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise LayoutError(f'{where}: must be a positive number, got {value!r}')
    return float(value)


def _segment_box(frame, name, p0, p1, width, height, z0, extend, provisional):
    """선분(cm) → 상자(m). extend: 양 끝을 폭의 절반만큼 늘려 모서리를 닫는다."""
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if length <= 0:
        raise LayoutError(f'{name}: zero-length segment')
    mx, my = (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0
    cx, cy = frame.to_world(mx, my)
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0]) + math.radians(frame.yaw_deg)
    total = length + (width if extend else 0.0)
    return Box(name, cx, cy, (z0 + height / 2.0) / 100.0, total / 100.0, width / 100.0,
               height / 100.0, yaw, provisional)


class Layout:
    def __init__(self, data):
        if not isinstance(data, dict):
            raise LayoutError('layout must be a mapping')
        if data.get('units') != 'cm':
            raise LayoutError("units must be 'cm'")
        f = data.get('frame') or {}
        try:
            self.frame = Frame(float(f['x_m']), float(f['y_m']), float(f['yaw_deg']))
        except (KeyError, TypeError, ValueError) as e:
            raise LayoutError(f'frame needs x_m, y_m, yaw_deg: {e}') from None
        d = data.get('defaults') or {}
        self.wall_thickness = _positive(d.get('wall_thickness'), 'defaults.wall_thickness')
        self.wall_height = _positive(d.get('wall_height'), 'defaults.wall_height')
        self.object_height = _positive(d.get('object_height'), 'defaults.object_height')
        self.line_width = _positive(d.get('line_width'), 'defaults.line_width')
        self.raw = data
        self.version = str(data.get('version', ''))
        self.map = self._map_spec(data.get('map'))
        self.walls = self._walls(data.get('walls') or [])
        self.objects = self._objects(data.get('objects') or [])
        spec = data.get('nodes') or {}
        self.graph_version = str(spec.get('graph_version', ''))
        self.bar_length = _positive(spec.get('bar_length', 12), 'nodes.bar_length')
        self.nodes = self._nodes(spec.get('items') or [])
        self.lines = self._lines(data.get('lines') or []) + self._tape_from_nodes()
        self.markers = self._markers(data.get('markers') or {})
        names = [b.name for b in self.walls + self.objects + self.lines + self.markers]
        dup = {n for n in names if names.count(n) > 1}
        if dup:
            raise LayoutError(f'duplicate names: {sorted(dup)}')

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as f:
            return cls(yaml.safe_load(f))

    def _walls(self, items):
        out = []
        for i, w in enumerate(items):
            name = w.get('name') or f'wall_{i}'
            out.append(_segment_box(
                self.frame, name, _point(w.get('from'), name), _point(w.get('to'), name),
                self.wall_thickness, float(w.get('height', self.wall_height)), 0.0,
                extend=True, provisional=bool(w.get('provisional', False))))
        return out

    def _objects(self, items):
        out = []
        for i, o in enumerate(items):
            name = o.get('name') or f'object_{i}'
            (x0, y0), (x1, y1) = _point(o.get('min'), name), _point(o.get('max'), name)
            if x1 <= x0 or y1 <= y0:
                raise LayoutError(f'{name}: max must be greater than min')
            h = _positive(o.get('height', self.object_height), f'{name}.height')
            cx, cy = self.frame.to_world((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            out.append(Box(name, cx, cy, h / 200.0, (x1 - x0) / 100.0, (y1 - y0) / 100.0,
                           h / 100.0, math.radians(self.frame.yaw_deg),
                           bool(o.get('provisional', False))))
        return out

    def _lines(self, items):
        out = []
        for i, line in enumerate(items):
            name = line.get('name') or f'line_{i}'
            pts = [_point(p, name) for p in line.get('points') or []]
            if len(pts) < 2:
                raise LayoutError(f'{name}: needs at least 2 points')
            if line.get('closed'):
                pts.append(pts[0])
            for k, (p0, p1) in enumerate(zip(pts, pts[1:])):
                # 두께 0.1 cm 테이프, 바닥(z=0) 위
                out.append(_segment_box(
                    self.frame, f'{name}_{k}', p0, p1, self.line_width, 0.1, 0.0,
                    extend=True, provisional=bool(line.get('provisional', False))))
        return out

    @staticmethod
    def _map_spec(m):
        if m is None:
            return None
        try:
            spec = MapSpec(float(m['resolution']), (float(m['origin'][0]), float(m['origin'][1])),
                           int(m['width']), int(m['height']), float(m['lidar_height']))
        except (KeyError, IndexError, TypeError, ValueError) as e:
            raise LayoutError(
                f'map needs resolution, origin [x, y], width, height, lidar_height: {e}') from None
        if spec.resolution <= 0 or spec.width <= 0 or spec.height <= 0:
            raise LayoutError('map: resolution, width, height must be positive')
        return spec

    def _nodes(self, items):
        nodes = []
        for i, n in enumerate(items):
            nid = n.get('id')
            if not nid:
                raise LayoutError(f'node #{i} needs an id')
            x, y = _point(n.get('at'), nid)
            bar = n.get('bar', '')
            if bar not in ('', 'x', 'y'):
                raise LayoutError(f"{nid}: bar must be 'x' or 'y', got {bar!r}")
            nodes.append(Node(nid, x, y, str(n.get('role', '')), tuple(n.get('links') or []),
                              str(n.get('place', '')), bar, bool(n.get('nav2_switch', False)),
                              bool(n.get('provisional', False))))
        ids = [n.id for n in nodes]
        dup = {i for i in ids if ids.count(i) > 1}
        if dup:
            raise LayoutError(f'duplicate node ids: {sorted(dup)}')
        by_id = {n.id: n for n in nodes}
        for n in nodes:
            for other in n.links:
                if other not in by_id:
                    raise LayoutError(f'{n.id}: unknown link {other}')
                if n.id not in by_id[other].links:
                    raise LayoutError(f'{n.id} -> {other}: link must be listed on both nodes')
                o = by_id[other]
                # 테이프는 x축 또는 y축 방향으로만 붙인다
                if abs(n.x - o.x) > 1e-6 and abs(n.y - o.y) > 1e-6:
                    raise LayoutError(f'{n.id} -> {other}: link must be axis-aligned')
        return nodes

    def edges(self):
        """양방향 연결을 한 번씩: [(Node, Node), ...]."""
        by_id = {n.id: n for n in self.nodes}
        seen, out = set(), []
        for n in self.nodes:
            for other in n.links:
                key = tuple(sorted((n.id, other)))
                if key not in seen:
                    seen.add(key)
                    out.append((n, by_id[other]))
        return out

    @staticmethod
    def edge_provisional(a, b):
        """위치를 추정한 분기 줄기(추정 정차 노드로 가는 구간)만 추정으로 본다.
        간선 위의 추정 분기점은 간선 자체(실측)를 추정으로 만들지 않는다."""
        return (a.provisional and bool(a.bar)) or (b.provisional and bool(b.bar))

    def _tape_from_nodes(self):
        """노드 연결 구간 + 정차 가로선 → 바닥 테이프 상자."""
        out = []
        for a, b in self.edges():
            out.append(_segment_box(
                self.frame, f'tape_{a.id}_{b.id}', (a.x, a.y), (b.x, b.y), self.line_width,
                0.1, 0.0, extend=True, provisional=self.edge_provisional(a, b)))
        half = self.bar_length / 2.0
        for n in self.nodes:
            if not n.bar:
                continue
            dx, dy = (half, 0.0) if n.bar == 'x' else (0.0, half)
            out.append(_segment_box(
                self.frame, f'bar_{n.id}', (n.x - dx, n.y - dy), (n.x + dx, n.y + dy),
                self.line_width, 0.1, 0.0, extend=False, provisional=n.provisional))
        return out

    def room(self):
        """벽 중심선의 바운딩 박스(실측 cm): (x0, y0, x1, y1)."""
        xs = [p for w in self.raw.get('walls') or [] for p in (w['from'][0], w['to'][0])]
        ys = [p for w in self.raw.get('walls') or [] for p in (w['from'][1], w['to'][1])]
        return min(xs), min(ys), max(xs), max(ys)

    def _markers(self, spec):
        defaults = spec.get('defaults') or {}
        out = []
        for m in spec.get('items') or []:
            name = m.get('name')
            if not name:
                raise LayoutError('marker needs a name')
            size = _positive(m.get('size', defaults.get('size')), f'{name}.size')
            zc = _positive(m.get('center_height', defaults.get('center_height')),
                           f'{name}.center_height')
            cx_cm, cy_cm = _point(m.get('center'), name)
            facing = float(m.get('facing_deg', 0.0))
            cx, cy = self.frame.to_world(cx_cm, cy_cm)
            # 판의 두께 방향 = facing 방향
            out.append(Box(f'marker_{name}', cx, cy, zc / 100.0, 0.005, size / 100.0,
                           size / 100.0, math.radians(facing + self.frame.yaw_deg),
                           bool(m.get('provisional', defaults.get('provisional', False)))))
        return out

    def provisional_names(self):
        # 직접 적은 lines 는 구간 번호(_k)를 떼고, 노드에서 만든 tape_·bar_ 는 그대로
        return sorted({b.name.rsplit('_', 1)[0]
                       if b in self.lines and not b.name.startswith(('tape_', 'bar_'))
                       else b.name
                       for b in self.walls + self.objects + self.lines + self.markers
                       if b.provisional})
