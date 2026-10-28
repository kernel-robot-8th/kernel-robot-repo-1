import math
import os

import pytest

from roport_gz_world.layout import Layout, LayoutError

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT = os.path.join(PKG, 'layout', 'roport_airport.yaml')
PGM = os.path.join(PKG, 'maps', 'airport_map.pgm')
MAP_YAML = os.path.join(PKG, 'maps', 'airport_map.yaml')

ROBOT_HALF_DIAGONAL = math.hypot(0.06, 0.06)  # nav2_params footprint 12 x 12 cm


@pytest.fixture(scope='module')
def layout():
    return Layout.load(LAYOUT)


def _by_name(boxes, name):
    return next(b for b in boxes if b.name == name)


def test_frame_round_trip(layout):
    for x, y in [(0, 0), (443, 220), (138, 91.8), (152.4, 202.0)]:
        wx, wy = layout.frame.to_world(x, y)
        mx, my = layout.frame.to_measured(wx, wy)
        assert mx == pytest.approx(x, abs=1e-9) and my == pytest.approx(y, abs=1e-9)


def test_outer_walls_443_by_220(layout):
    aisle = _by_name(layout.walls, 'outer_aisle')
    window = _by_name(layout.walls, 'outer_window')
    # 선분 길이 + 두께(모서리를 닫으려고 양 끝을 0.25 cm씩 늘림)
    assert aisle.sx == pytest.approx(4.43 + 0.005)
    assert _by_name(layout.walls, 'outer_left').sx == pytest.approx(2.20 + 0.005)
    assert layout.frame.to_measured(aisle.cx, aisle.cy)[1] == pytest.approx(0.0, abs=1e-6)
    assert layout.frame.to_measured(window.cx, window.cy)[1] == pytest.approx(220.0, abs=1e-6)


def test_wall_thickness_and_line_width(layout):
    assert all(w.sy == pytest.approx(0.005) for w in layout.walls)   # 0.5 cm
    assert all(w.sz == pytest.approx(0.20) for w in layout.walls)    # 임시 20 cm
    assert all(ln.sy == pytest.approx(0.02) for ln in layout.lines)  # 2 cm
    assert all(ln.sz == pytest.approx(0.001) for ln in layout.lines)


def test_objects_inside_room(layout):
    for o in layout.objects:
        mx, my = layout.frame.to_measured(o.cx, o.cy)
        eps = 1e-6  # 벽에 붙은 물체의 부동소수점 경계
        assert o.sx / 2 * 100 - eps <= mx <= 443 - o.sx / 2 * 100 + eps, o.name
        assert o.sy / 2 * 100 - eps <= my <= 220 - o.sy / 2 * 100 + eps, o.name


def test_known_dimensions(layout):
    box = _by_name(layout.objects, 'box_67x21_5')
    assert (box.sx, box.sy) == (pytest.approx(0.67), pytest.approx(0.215))
    cam = _by_name(layout.objects, 'camera_base')
    assert (cam.sx, cam.sy) == (pytest.approx(0.32), pytest.approx(0.15))
    # 03 공용 내부공간 높이 128.2: 아래 벽 y = 220 - 128.2
    w2 = _by_name(layout.walls, 'zone03_bottom')
    assert layout.frame.to_measured(w2.cx, w2.cy)[1] == pytest.approx(91.8, abs=1e-6)
    # 입구 66: 아래 벽 끝 x = 377
    end_x = layout.frame.to_measured(w2.cx, w2.cy)[0] + (w2.sx * 100 - 0.5) / 2
    assert end_x == pytest.approx(377.0, abs=1e-6)


def test_lidar_visibility_heights(layout):
    """Pinky 라이다 높이 0.125 m: SLAM 지도에 보인 물체는 더 높고, OMX3 받침은 더 낮다."""
    lidar = 0.028 + 0.067 + 0.030
    for o in layout.objects:
        if o.name == 'omx3_base':
            assert o.sz < lidar
        else:
            assert o.sz > lidar, o.name


def test_visual_only_items_below_lidar(layout):
    """Gazebo GPU 라이다는 visual 을 감지한다: 충돌 없는 표시물은 라이다 평면 아래에 있어야
    실제 SLAM 지도(마커·라인이 안 보임)와 같은 스캔이 나온다."""
    lidar = 0.028 + 0.067 + 0.030
    for b in layout.lines + layout.markers:
        assert b.cz + b.sz / 2 < lidar, b.name


def _dist_point_box(px, py, b):
    c, s = math.cos(-b.yaw), math.sin(-b.yaw)
    lx = c * (px - b.cx) - s * (py - b.cy)
    ly = s * (px - b.cx) + c * (py - b.cy)
    dx = max(abs(lx) - b.sx / 2, 0.0)
    dy = max(abs(ly) - b.sy / 2, 0.0)
    return math.hypot(dx, dy)


def test_spawn_point_is_free(layout):
    """원본 launch 는 로봇을 world (0,0)에 스폰한다 = SLAM 시작 위치."""
    clearance = min(_dist_point_box(0.0, 0.0, b) for b in layout.walls + layout.objects)
    assert clearance > ROBOT_HALF_DIAGONAL + 0.03, clearance


def test_provisional_values_are_marked(layout):
    names = layout.provisional_names()
    for expected in ('omx3_base', 'partition_warehouse', 'partition_delivery', 'shelf_table',
                     'stub_p4', 'marker_P5'):
        assert expected in names
    assert 'box_67x21_5' not in names and 'outer_aisle' not in names


def _read_pgm(path):
    data = open(path, 'rb').read()
    header = [ln for ln in data.split(b'\n')[:4] if not ln.startswith(b'#')]
    w, h = map(int, header[1].split())
    return w, h, data[len(data) - w * h:]


def test_walls_match_real_slam_map(layout):
    """실측 벽(world 좌표)이 실제 SLAM 지도의 점유 칸과 겹치는지 (frame 값 회귀 방지)."""
    import yaml
    meta = yaml.safe_load(open(MAP_YAML))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    w, h, px = _read_pgm(PGM)
    occ = [(ox + (c + 0.5) * res, oy + (h - 1 - r + 0.5) * res)
           for r in range(h) for c in range(w) if px[r * w + c] < 50]
    dists = []
    for b in layout.walls:
        n = max(2, int(b.sx / 0.05))
        for i in range(n):
            t = (i / (n - 1) - 0.5) * (b.sx - 0.005)
            x, y = b.cx + t * math.cos(b.yaw), b.cy + t * math.sin(b.yaw)
            dists.append(min(math.hypot(x - a, y - c) for a, c in occ))
    mean = sum(dists) / len(dists)
    assert mean < 0.03, mean
    assert max(dists) < 0.07, max(dists)


@pytest.mark.parametrize('patch, message', [
    ({'units': 'm'}, 'units'),
    ({'frame': {'x_m': 0}}, 'frame'),
    ({'walls': [{'name': 'z', 'from': [1, 1], 'to': [1, 1]}]}, 'zero-length'),
    ({'objects': [{'name': 'o', 'min': [5, 5], 'max': [1, 1]}]}, 'greater than min'),
    ({'walls': [{'name': 'dup', 'from': [0, 0], 'to': [1, 0]},
                {'name': 'dup', 'from': [0, 1], 'to': [1, 1]}]}, 'duplicate'),
    ({'lines': [{'name': 'l', 'points': [[0, 0]]}]}, 'at least 2'),
    ({'walls': [{'name': 'w', 'from': [0, 'x'], 'to': [1, 0]}]}, 'point must be'),
])
def test_invalid_layout(patch, message):
    import yaml
    data = yaml.safe_load(open(LAYOUT))
    data.update(patch)
    with pytest.raises(LayoutError, match=message):
        Layout(data)
