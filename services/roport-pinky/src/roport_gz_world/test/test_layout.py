import math
import os

import pytest

from roport_gz_world import generate_map
from roport_gz_world.layout import Layout, LayoutError

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT = os.path.join(PKG, 'layout', 'roport_airport.yaml')
MAPS = os.path.normpath(os.path.join(PKG, '..', '..', 'maps'))
SLAM_PGM = os.path.join(MAPS, 'airport_map_slam_20261027.pgm')
SLAM_YAML = os.path.join(MAPS, 'airport_map_slam_20261027.yaml')

ROBOT_HALF_DIAGONAL = math.hypot(0.06, 0.06)  # nav2_params footprint 12 x 12 cm
LIDAR = 0.028 + 0.067 + 0.030                 # Pinky 라이다 높이 0.125 m


@pytest.fixture(scope='module')
def layout():
    return Layout.load(LAYOUT)


def _by_name(boxes, name):
    return next(b for b in boxes if b.name == name)


def _node(layout, nid):
    return next(n for n in layout.nodes if n.id == nid)


def test_frame_round_trip(layout):
    for x, y in [(0, 0), (444.5, 221), (138, 92), (152.4, 202.0)]:
        wx, wy = layout.frame.to_world(x, y)
        mx, my = layout.frame.to_measured(wx, wy)
        assert mx == pytest.approx(x, abs=1e-9) and my == pytest.approx(y, abs=1e-9)


def test_outer_walls_444_5_by_221(layout):
    aisle = _by_name(layout.walls, 'outer_aisle')
    window = _by_name(layout.walls, 'outer_window')
    # 선분 길이 + 두께(모서리를 닫으려고 양 끝을 0.25 cm씩 늘림). G 138 + H 306.5
    assert aisle.sx == pytest.approx(4.445 + 0.005)
    assert _by_name(layout.walls, 'outer_left').sx == pytest.approx(2.21 + 0.005)
    assert layout.frame.to_measured(aisle.cx, aisle.cy)[1] == pytest.approx(0.0, abs=1e-6)
    assert layout.frame.to_measured(window.cx, window.cy)[1] == pytest.approx(221.0, abs=1e-6)
    assert layout.room() == (0, 0, 444.5, 221)


def test_wall_thickness_height_and_tape_width(layout):
    assert all(w.sy == pytest.approx(0.005) for w in layout.walls)   # 0.5 cm
    assert all(w.sz == pytest.approx(0.24) for w in layout.walls)    # 실측 24 cm
    assert all(ln.sy == pytest.approx(0.024) for ln in layout.lines)  # 테이프 24 mm
    assert all(ln.sz == pytest.approx(0.001) for ln in layout.lines)


def test_objects_inside_room(layout):
    x0, y0, x1, y1 = layout.room()
    for o in layout.objects:
        mx, my = layout.frame.to_measured(o.cx, o.cy)
        eps = 1e-6  # 벽에 붙은 물체의 부동소수점 경계
        assert x0 + o.sx / 2 * 100 - eps <= mx <= x1 - o.sx / 2 * 100 + eps, o.name
        assert y0 + o.sy / 2 * 100 - eps <= my <= y1 - o.sy / 2 * 100 + eps, o.name


def test_zone03_and_entrance(layout):
    # 03 아래 벽: 통로 벽 → 03 아래 벽 ㅂ 94.5(두 벽 두께 포함) → 벽 중심 y 94
    w = _by_name(layout.walls, 'zone03_bottom')
    assert layout.frame.to_measured(w.cx, w.cy)[1] == pytest.approx(94.0, abs=1e-6)
    # I = H 306.5 - 입구 65 = 241.5 → 아래 벽 끝 x = 379.5
    end_x = layout.frame.to_measured(w.cx, w.cy)[0] + (w.sx * 100 - 0.5) / 2
    assert end_x == pytest.approx(379.5, abs=1e-6)
    assert 444.5 - end_x == pytest.approx(65.0)
    box = _by_name(layout.objects, 'box_03')
    assert (box.sx, box.sy) == (pytest.approx(0.674), pytest.approx(0.21))


def test_line_dimensions_from_measurement(layout):
    half = layout.line_width / 2  # 바깥 기준 치수 → 중심선
    n10, n05 = _node(layout, 'N10'), _node(layout, 'N05')
    assert n10.x - half == pytest.approx(20.5)                     # a
    assert n10.y - half == pytest.approx(72.0)                     # b
    assert n05.x - n10.x + 2 * half == pytest.approx(93.2)         # D
    assert n05.y - n10.y + 2 * half == pytest.approx(91.3)         # c
    n12 = _node(layout, 'N12')
    # 2026-11-03 저녁 재측정 기록지: 벽·칸막이 면 → 세로선 왼쪽 가장자리
    wall_face = layout.raw['defaults']['wall_thickness'] / 2
    assert _node(layout, 'N16').x - half - wall_face == pytest.approx(44.5)        # [24]
    assert _node(layout, 'N17').x - half - wall_face == pytest.approx(91.5)        # [25]
    for stop, length in (('N16', 25), ('N17', 25)):                                # R
        assert _node(layout, stop).y + half - (n05.y + half) == pytest.approx(length)
    for stop, length in (('N01', 29.9), ('H1', 40), ('H2', 40)):                   # Q U
        assert (n10.y - half) - (_node(layout, stop).y - half) == pytest.approx(length)
    h1, h2 = _node(layout, 'H1'), _node(layout, 'H2')
    assert h1.x - half - (280 + wall_face) == pytest.approx(25)                    # [21]
    assert h2.x - h1.x == pytest.approx(33)                                        # [22]
    assert n12.x + half - (h2.x - half) == pytest.approx(38.5)                     # [23]
    # 분기 x: 바깥선 → 바깥선
    assert _node(layout, 'N18').x + half - (n10.x - half) == pytest.approx(43.6)
    n07 = _node(layout, 'N07')
    assert _node(layout, 'N19').x + half - (n07.x - half) == pytest.approx(94.0)


def test_delivery_and_warehouse_dimensions(layout):
    raw = {w['name']: w for w in layout.raw['walls']}
    p, t = raw['partition_warehouse'], raw['partition_delivery']
    assert t['from'][0] - p['from'][0] == pytest.approx(142)       # 인도장 가로
    assert (p['to'][1], t['to'][1]) == (54, 49)                    # P · T
    objs = {o['name']: o for o in layout.raw['objects']}
    cam = objs['camera_tripod']
    assert cam['min'][0] - 138 == pytest.approx(58)                # G1 게이트 벽 → 삼각대
    omx3 = objs['omx3_base']
    assert omx3['max'][0] - omx3['min'][0] == pytest.approx(27)    # d
    # P4 가로선은 OMX3 받침 앞 e 9.6 (Q 29.9 로 계산한 값과 0.5 cm 이내)
    n01 = _node(layout, 'N01')
    assert n01.y - layout.line_width / 2 - omx3['max'][1] == pytest.approx(9.6, abs=0.6)
    # N11 가로선은 OMX1·2 받침 오목 안
    half = layout.bar_length / 2
    assert objs['omx12_left']['max'][0] < _node(layout, 'N11').x - half
    assert _node(layout, 'N11').x + half < objs['omx12_right']['min'][0]
    assert objs['omx12_right']['min'][0] - objs['omx12_left']['max'][0] == pytest.approx(25.5)  # [6]
    face = 138 + layout.raw['defaults']['wall_thickness'] / 2      # 칸막이-좌 면
    assert objs['shelf_02']['min'][0] - face == pytest.approx(45.9)                # [2]
    assert objs['shelf_02']['max'][0] - objs['shelf_02']['min'][0] == pytest.approx(50)  # [3]
    assert objs['omx12_left']['min'][0] - face == pytest.approx(35.5)              # [4]


def test_omx12_from_walls(layout):
    # 2026-11-03 저녁 재측정 기록지: 벽-선반(통로 벽) 면 기준, 벽-03 면 기준은 0.5 cm 안에서 확인
    raw = {w['name']: w for w in layout.raw['walls']}
    half_t = layout.raw['defaults']['wall_thickness'] / 2
    face03 = raw['zone03_bottom']['from'][1] - half_t
    aisle_face = half_t
    objs = {o['name']: o for o in layout.raw['objects']}
    n11 = _node(layout, 'N11')
    bar_end = n11.y - layout.line_width / 2           # 가로선의 선반 쪽 가장자리
    for name in ('omx12_left', 'omx12_right'):
        assert objs[name]['max'][1] - aisle_face == pytest.approx(50.5)          # [8]
        assert face03 - objs[name]['max'][1] == pytest.approx(42.5, abs=0.6)     # [11]
    back = objs['omx12_back']['max'][1]
    assert back - aisle_face == pytest.approx(31.5)                              # [10]
    assert face03 - back == pytest.approx(61.5, abs=0.6)                         # [12]
    assert bar_end - aisle_face == pytest.approx(35.7)                           # [15]
    assert bar_end - back == pytest.approx(4, abs=0.3)                           # [13] V
    assert face03 - aisle_face == pytest.approx(93.5)                            # Ⓓ 벽 → 벽
    # 바닥 P7 마커(2 cm)는 가로선과 오목 바닥 사이
    p7 = next(m for m in layout.raw['markers']['items'] if m['name'] == 'P7')
    assert objs['omx12_back']['max'][1] + 1 < p7['center'][1] < bar_end - 1


def test_node_graph(layout):
    ids = {n.id for n in layout.nodes}
    assert {'N13', 'N20', 'H3'}.isdisjoint(ids)
    switch = [n.id for n in layout.nodes if n.nav2_switch]
    assert switch == ['N12']
    places = {n.place: n.id for n in layout.nodes if n.place}
    assert places == {'P1': 'H1', 'P2': 'H2', 'P4': 'N01', 'P5': 'N16', 'P6': 'N17', 'P7': 'N11'}
    # 정차 노드(가로선) = 막다른 노드
    for n in layout.nodes:
        assert bool(n.bar) == (len(n.links) == 1), n.id
    # 모든 노드가 한 그래프로 이어짐
    by_id = {n.id: n for n in layout.nodes}
    seen, todo = set(), ['N12']
    while todo:
        cur = todo.pop()
        if cur not in seen:
            seen.add(cur)
            todo += by_id[cur].links
    assert seen == ids


def test_tape_and_bars_from_nodes(layout):
    names = {ln.name for ln in layout.lines}
    assert len([n for n in names if n.startswith('tape_')]) == len(layout.edges())
    bars = [ln for ln in layout.lines if ln.name.startswith('bar_')]
    assert {b.name for b in bars} == {f'bar_{n.id}' for n in layout.nodes if n.bar}
    assert all(b.sx == pytest.approx(0.12) for b in bars)          # 가로선 12 cm 고정


def test_nodes_clear_of_obstacles(layout):
    """테이프 노드가 벽·물체 안에 있지 않은지."""
    for n in layout.nodes:
        wx, wy = layout.frame.to_world(n.x, n.y)
        clearance = min(_dist_point_box(wx, wy, b) for b in layout.walls + layout.objects
                        if b.sz > LIDAR)
        assert clearance > 0.02, (n.id, clearance)


def test_lidar_visibility_heights(layout):
    """지도에 그리는 물체는 라이다보다 높고, 창고박스(11 cm)는 낮다."""
    for o in layout.objects:
        if o.name == 'warehouse_box':
            assert o.sz < LIDAR
        else:
            assert o.sz > LIDAR, o.name
    assert layout.map.lidar_height == pytest.approx(LIDAR * 100)


def test_visual_only_items_below_lidar(layout):
    """Gazebo GPU 라이다는 visual 을 감지한다: 충돌 없는 표시물은 라이다 평면 아래에 있어야
    실제 SLAM 지도(마커·라인이 안 보임)와 같은 스캔이 나온다."""
    for b in layout.lines + layout.markers:
        assert b.cz + b.sz / 2 < LIDAR, b.name


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
    for expected in ('omx12_left', 'omx12_right', 'omx12_back', 'marker_P5', 'marker_P7'):
        assert expected in names
    for measured in ('box_03', 'outer_aisle', 'zone03_bottom', 'partition_delivery', 'omx3_base',
                     'camera_tripod', 'tape_N18_N01', 'tape_N19_N11', 'bar_N11', 'bar_N16'):
        assert measured not in names


def _read_pgm(path):
    data = open(path, 'rb').read()
    header = [ln for ln in data.split(b'\n')[:4] if not ln.startswith(b'#')]
    w, h = map(int, header[1].split())
    return w, h, data[len(data) - w * h:]


def test_walls_match_slam_map(layout):
    """실측 벽(world 좌표)이 2026-10-27 SLAM 지도의 점유 칸과 겹치는지 (실측·frame 값 검증)."""
    import yaml
    meta = yaml.safe_load(open(SLAM_YAML))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    w, h, px = _read_pgm(SLAM_PGM)
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


def test_map_has_same_grid_as_slam_map(layout):
    import yaml
    meta = yaml.safe_load(open(SLAM_YAML))
    w, h, _ = _read_pgm(SLAM_PGM)
    m = layout.map
    assert (m.width, m.height) == (w, h)
    assert m.resolution == pytest.approx(float(meta['resolution']))
    assert m.origin == (pytest.approx(meta['origin'][0]), pytest.approx(meta['origin'][1]))


def test_committed_map_matches_layout():
    """배치 데이터를 고친 뒤 지도를 다시 생성하지 않으면 실패한다."""
    assert generate_map.main(['--layout', LAYOUT, '--output-dir', MAPS, '--check']) == 0


def test_map_pixels(layout):
    px = generate_map.build_pixels(layout)
    m = layout.map
    assert set(px) == {generate_map.OCCUPIED, generate_map.UNKNOWN, generate_map.FREE}

    def at(x_cm, y_cm):
        wx, wy = layout.frame.to_world(x_cm, y_cm)
        c = int((wx - m.origin[0]) / m.resolution)
        r = m.height - 1 - int((wy - m.origin[1]) / m.resolution)
        return px[r * m.width + c]

    assert at(152, 202) == generate_map.FREE        # SLAM 시작 위치(스폰)
    assert at(412, 80) == generate_map.FREE         # 03 입구
    assert at(359, 150) == generate_map.OCCUPIED    # 03 상자
    assert at(25, 20) == generate_map.FREE          # 창고박스 11 cm < 라이다
    assert at(64, 15) == generate_map.OCCUPIED      # OMX3 받침(벽 높이로 둠)
    assert at(280, 25) == generate_map.OCCUPIED     # 인도장 오른쪽 칸막이 T
    assert at(250, 73.2) == generate_map.FREE       # 테이프는 안 그림
    assert at(200, -10) == generate_map.UNKNOWN     # 벽 바깥


def test_png_round_trip():
    px = bytes([0, 205, 254, 0, 254, 205])
    assert generate_map.png_pixels(generate_map.png_bytes(3, 2, px)) == px


def test_check_detects_stale_map(tmp_path):
    (tmp_path / 'airport_map.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
    assert generate_map.main(['--layout', LAYOUT, '--output-dir', str(tmp_path), '--check']) == 1


@pytest.mark.parametrize('patch, message', [
    ({'units': 'm'}, 'units'),
    ({'frame': {'x_m': 0}}, 'frame'),
    ({'walls': [{'name': 'z', 'from': [1, 1], 'to': [1, 1]}]}, 'zero-length'),
    ({'objects': [{'name': 'o', 'min': [5, 5], 'max': [1, 1]}]}, 'greater than min'),
    ({'walls': [{'name': 'dup', 'from': [0, 0], 'to': [1, 0]},
                {'name': 'dup', 'from': [0, 1], 'to': [1, 1]}]}, 'duplicate'),
    ({'lines': [{'name': 'l', 'points': [[0, 0]]}]}, 'at least 2'),
    ({'walls': [{'name': 'w', 'from': [0, 'x'], 'to': [1, 0]}]}, 'point must be'),
    ({'map': {'resolution': 0.05}}, 'map needs'),
    ({'nodes': {'items': [{'id': 'A', 'at': [0, 0], 'links': ['B']}]}}, 'unknown link'),
    ({'nodes': {'items': [{'id': 'A', 'at': [0, 0], 'links': ['B']},
                          {'id': 'B', 'at': [1, 0]}]}}, 'both nodes'),
    ({'nodes': {'items': [{'id': 'A', 'at': [0, 0], 'links': ['B']},
                          {'id': 'B', 'at': [1, 1], 'links': ['A']}]}}, 'axis-aligned'),
    ({'nodes': {'items': [{'id': 'A', 'at': [0, 0]}, {'id': 'A', 'at': [1, 0]}]}}, 'duplicate'),
])
def test_invalid_layout(patch, message):
    import yaml
    data = yaml.safe_load(open(LAYOUT))
    data.update(patch)
    with pytest.raises(LayoutError, match=message):
        Layout(data)
