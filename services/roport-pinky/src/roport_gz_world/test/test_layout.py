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
    # v16: 통로 벽은 x 280(02 오른쪽 칸막이)에서 나뉘고 동쪽은 2026-11-19 B 로 기울임
    raw = {w['name']: w for w in layout.raw['walls']}
    assert raw['outer_aisle']['from'][0] == 0
    assert raw['outer_aisle']['to'][0] == raw['outer_aisle_east']['from'][0] == 280
    assert raw['outer_aisle_east']['to'][0] == 444.5
    assert raw['outer_aisle_east']['to'] == raw['outer_right']['from']
    assert raw['partition_delivery']['from'] == raw['outer_aisle_east']['from']   # 틈 없음
    assert aisle.sx == pytest.approx(2.80 + 0.005)
    assert _by_name(layout.walls, 'outer_left').sx == pytest.approx(2.21 + 0.005)
    assert layout.frame.to_measured(aisle.cx, aisle.cy)[1] == pytest.approx(0.0, abs=1e-6)
    assert layout.frame.to_measured(window.cx, window.cy)[1] == pytest.approx(221.0, abs=1e-6)
    assert layout.room() == (0, -4.38, 444.5, 221)


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


def _zone03_bottom(layout):
    """03 아래 벽 꺾은선(v16): 서쪽부터 이어진 선분 [(from, to), ...]."""
    segs = [(w['from'], w['to']) for w in layout.raw['walls']
            if w['name'] == 'zone03_bottom' or w['name'].startswith('zone03_bottom_')]
    return sorted(segs, key=lambda s: s[0][0])


def _wall_y(segs, x):
    """꺾은선 위 x 에서의 벽 중심 y."""
    for (x0, y0), (x1, y1) in segs:
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    raise AssertionError(f'x {x} outside wall')


def test_zone03_and_entrance(layout):
    segs = _zone03_bottom(layout)
    # 꺾은선은 끊김 없이 이어지고, 왼쪽 벽(게이트 벽)에서 시작한다
    for (_, end), (start, _) in zip(segs, segs[1:]):
        assert end == start
    raw = {w['name']: w for w in layout.raw['walls']}
    assert segs[0][0] == raw['zone03_left']['from'] and segs[0][0][0] == 138
    # I = H 306.5 - 입구 65 = 241.5 → 아래 벽 끝 x = 379.5
    end_x = segs[-1][1][0]
    assert end_x == pytest.approx(379.5, abs=1e-6)
    assert 444.5 - end_x == pytest.approx(65.0)
    # 생성한 벽 상자도 꺾은선 끝까지(양 끝 0.25 cm 늘림)
    last = _by_name(layout.walls, 'zone03_bottom_n12')
    mx, _ = layout.frame.to_measured(last.cx, last.cy)
    assert mx + (last.sx * 100 - 0.5) / 2 * math.cos(last.yaw - math.radians(
        layout.frame.yaw_deg)) == pytest.approx(379.5, abs=1e-6)
    box = _by_name(layout.objects, 'box_03')
    assert (box.sx, box.sy) == (pytest.approx(0.674), pytest.approx(0.21))


def test_trunk_to_walls_2026_11_19(layout):
    """2026-11-19 줄자 실측(사용자): 본선 테이프 가장자리 → 벽 면. 본선이 강의실 벽과 평행하지 않아
    노드는 그대로 두고 벽을 맞췄다(사용자 결정)."""
    half_t = layout.raw['defaults']['wall_thickness'] / 2
    half = layout.line_width / 2
    trunk = _node(layout, 'N12').y
    segs = _zone03_bottom(layout)
    # A: 본선 03 쪽 가장자리 → 03 아래 벽 면
    for nid, a in (('N19', 16.2), ('N24', 16.2), ('N09', 15.5), ('N08', 15.2), ('N12', 14.8)):
        n = _node(layout, nid)
        assert n.y == trunk                                             # 본선 노드 y 그대로
        face = _wall_y(segs, n.x) - half_t
        assert face - (n.y + half) == pytest.approx(a, abs=1e-6), nid
    # x 138~N19 는 실측 없음: N19 값으로 평평
    assert _wall_y(segs, 138) == pytest.approx(_wall_y(segs, _node(layout, 'N19').x))
    # B: 통로 벽 면(벽 선) → 본선 통로 쪽 가장자리. 서쪽 b 72 와 같은 기준(벽 선 = 면)
    east = next(w for w in layout.raw['walls'] if w['name'] == 'outer_aisle_east')
    (x0, y0), (x1, y1) = east['from'], east['to']
    for nid, b in (('N08', 75.0), ('N12', 75.4)):
        n = _node(layout, nid)
        aisle = y0 + (y1 - y0) * (n.x - x0) / (x1 - x0)
        assert (n.y - half) - aisle == pytest.approx(b, abs=0.1), nid
    # 통로 벽에 붙인 P1·P2 마커와 03 아래 벽에 붙은 삼각대는 벽을 따라 옮김
    for m in layout.raw['markers']['items']:
        if m['name'] in ('P1', 'P2'):
            aisle = y0 + (y1 - y0) * (m['center'][0] - x0) / (x1 - x0)
            assert m['center'][1] - aisle == pytest.approx(0.5, abs=0.01), m['name']
    cam = {o['name']: o for o in layout.raw['objects']}['camera_tripod']
    assert cam['min'][1] == pytest.approx(_wall_y(segs, cam['min'][0]))
    assert cam['max'][1] - cam['min'][1] == pytest.approx(25.5)        # ㄹ


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
    # 벽-왼쪽은 폼보드 없는 강의실 벽 → 면 = x 0 (2026-11-06 사용자 확인)
    assert _node(layout, 'N16').x - half == pytest.approx(44.5)                    # [24]
    assert _node(layout, 'N17').x - half == pytest.approx(91.5)                    # [25]
    # 02 인도장 세로선: 벽-왼쪽 면 → 왼쪽 가장자리 b · c · d (2026-11-06)
    for nid, dist in (('N21', 160), ('N11', 208.2), ('N22', 256.8)):
        assert _node(layout, nid).x - half == pytest.approx(dist), nid
    for stop, length in (('N16', 25), ('N17', 25)):                                # R
        assert _node(layout, stop).y + half - (n05.y + half) == pytest.approx(length)
    assert (n10.y - half) - (_node(layout, 'N01').y - half) == pytest.approx(29.9)  # Q
    # 2026-11-16 본선 가운데 → 가로선 가운데(본선 N07~N09 직선, 선반 벽이 휨 → 벽 기준 안 씀).
    # H1·H2 는 v14 U 40(바깥선 기준)을 대신함
    for branch, stop, length in (('N23', 'N21', 41.2), ('N24', 'N22', 41.5),
                                 ('N09', 'H1', 40.2), ('N08', 'H2', 40.2)):
        assert _node(layout, branch).y - _node(layout, stop).y == pytest.approx(length), branch
    # N11 은 사용자 결정(2026-11-16)으로 v14 위치 유지: 실측 N19→N11 38.75 와 2.7 cm 차이
    assert _node(layout, 'N19').y - _node(layout, 'N11').y == pytest.approx(38.75 - 2.7)
    h1, h2 = _node(layout, 'H1'), _node(layout, 'H2')
    assert h1.x - half - (280 + wall_face) == pytest.approx(25)                    # [21]
    assert h2.x - h1.x == pytest.approx(33)                                        # [22]
    assert n12.x + half - (h2.x - half) == pytest.approx(38.5)                     # [23]
    # 분기 x: 바깥선 → 바깥선
    assert _node(layout, 'N18').x + half - (n10.x - half) == pytest.approx(43.6)
    # 분기는 정차 노드 세로선과 같은 x
    for branch, stop in (('N19', 'N11'), ('N23', 'N21'), ('N24', 'N22')):
        assert _node(layout, branch).x == _node(layout, stop).x
        assert _node(layout, branch).y == _node(layout, 'N07').y


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
    ol, orr = objs['omx12_left'], objs['omx12_right']
    assert orr['min'][0] - ol['max'][0] == pytest.approx(25.8)                     # [6] 11-05
    assert orr['max'][0] - ol['min'][0] == pytest.approx(72.6)                     # [5]+[6]+[7]
    lw = layout.line_width
    assert _node(layout, 'N11').x - lw / 2 - ol['max'][0] == pytest.approx(11.4)   # N11 ↔ OMX1
    assert orr['min'][0] - (_node(layout, 'N11').x + lw / 2) == pytest.approx(11.8, abs=0.3)  # ↔ OMX2
    # 새 세로선은 받침 바깥 약 11 cm(손그림 A1 11.7 · C1 11)
    assert ol['min'][0] - (_node(layout, 'N21').x + lw / 2) == pytest.approx(11.0, abs=0.8)
    assert _node(layout, 'N22').x - lw / 2 - orr['max'][0] == pytest.approx(11.0, abs=0.8)
    for nid in ('N21', 'N22'):                                      # 가로선이 받침에 안 닿음
        n = _node(layout, nid)
        assert n.x + half < ol['min'][0] or n.x - half > orr['max'][0], nid
    # 칸막이 면 기준 값은 N11 기준으로 놓은 받침과 0.5 cm 안(a 139.8 은 1.9 cm 어긋나 안 씀)
    face = 138 + layout.raw['defaults']['wall_thickness'] / 2      # 칸막이-좌 면(가정)
    assert objs['shelf_02']['min'][0] - face == pytest.approx(45.9, abs=0.5)       # [2]
    assert objs['shelf_02']['max'][0] - objs['shelf_02']['min'][0] == pytest.approx(50)  # [3]
    assert ol['min'][0] - face == pytest.approx(35.5, abs=0.5)                     # [4]
    right = 280 + layout.raw['defaults']['wall_thickness'] / 2     # 칸막이-우 면
    assert right - orr['max'][0] == pytest.approx(34, abs=0.5)                     # D1


def test_omx12_from_walls(layout):
    # 2026-11-03 저녁 재측정 기록지: 벽-선반(통로 벽) 면 기준, 벽-03 면 기준은 0.5 cm 안에서 확인
    half_t = layout.raw['defaults']['wall_thickness'] / 2
    # [11]·[12]·Ⓓ 는 2026-11-03 벽-03 면 기준이다. v16 은 03 아래 벽을 2026-11-19 A(본선 기준)로
    # 3.15 cm 옮겼으므로(02 구간), 이 값들은 v15 벽 면 93.75 에 대고 받침 위치만 확인한다.
    face03 = 93.75
    v16_face03 = _wall_y(_zone03_bottom(layout), 209.4) - half_t
    assert face03 - v16_face03 == pytest.approx(3.15)
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
    # [13] V 3.8(11-05)은 [15]·[10]으로 계산한 4.2와 0.4 차이
    assert bar_end - back == pytest.approx(3.8, abs=0.5)                         # [13] V
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
    dists, moved = [], []
    for b in layout.walls:
        n = max(2, int(b.sx / 0.05))
        for i in range(n):
            t = (i / (n - 1) - 0.5) * (b.sx - 0.005)
            x, y = b.cx + t * math.cos(b.yaw), b.cy + t * math.sin(b.yaw)
            d = min(math.hypot(x - a, y - c) for a, c in occ)
            (moved if b.name in V16_TRUNK_WALLS else dists).append(d)
    everything = dists + moved
    assert sum(everything) / len(everything) < 0.03
    assert max(dists) < 0.07, max(dists)
    # v16: 본선 쪽 벽은 2026-11-19 본선 ↔ 벽 실측으로 옮겼다(본선 테이프가 벽과 평행하지 않음).
    # SLAM 지도는 실제 벽 위치라서 여기서는 옮긴 만큼(최대 4.55 cm) + 칸 크기만큼 더 멀다(실측 7.4 cm).
    assert max(moved) < 0.08, max(moved)


# 2026-11-19 실측으로 옮기거나 기울인 벽(SLAM 지도와 거리가 커지는 쪽)
V16_TRUNK_WALLS = {'outer_aisle_east', 'zone03_bottom_n24', 'zone03_bottom_n09',
                   'zone03_bottom_n08', 'zone03_bottom_n12'}


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
