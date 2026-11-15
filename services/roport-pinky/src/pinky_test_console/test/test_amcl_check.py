import csv
import math

import pytest

from pinky_test_console.amcl_check import (
    AmclCheckError,
    AmclRecorder,
    DistanceField,
    Frame,
    load_layout,
    match_score,
    pose_std,
    scan_to_points,
)

# roport_gz_world/layout/roport_airport.yaml 과 같은 형식의 작은 배치
LAYOUT = """
units: cm
frame: {x_m: -1.520, y_m: -2.0225, yaw_deg: 0.10}
nodes:
  items:
    - {id: N08, at: [339.5, 73.2], links: [N12, H2]}
    - {id: H2, at: [339.5, 33.2], place: P2, bar: x, links: [N08]}
    - {id: N12, at: [378.3, 73.2], role: 간선 끝·Nav2 전환, nav2_switch: true, links: [N08]}
"""


@pytest.fixture
def layout_path(tmp_path):
    p = tmp_path / 'layout.yaml'
    p.write_text(LAYOUT, encoding='utf-8')
    return str(p)


def wall_grid():
    """10 x 6 칸, 5 cm/칸, 원점 (0, 0). x = 0 열과 y = 5 행이 벽, 오른쪽 위 한 칸은 미지."""
    w, h = 10, 6
    data = [0] * (w * h)
    for r in range(h):
        data[r * w + 0] = 100
    for c in range(w):
        data[5 * w + c] = 100
    data[0 * w + 9] = -1
    return w, h, data


def test_frame_round_trip_matches_layout_formula():
    f = Frame(-1.520, -2.0225, 0.10)
    x, y, yaw = f.to_map(152.0, 202.0, 90.0)
    mx, my, myaw = f.to_measured(x, y, yaw)
    assert (mx, my, myaw) == (pytest.approx(152.0), pytest.approx(202.0), pytest.approx(90.0))
    # SLAM 시작 위치(실측 약 152, 202)가 map 원점 근처
    assert math.hypot(x, y) < 0.02


def test_load_layout_nodes_and_yaw(layout_path):
    frame, nodes = load_layout(layout_path)
    assert frame.x_m == pytest.approx(-1.520)
    assert nodes[0]['id'] == 'N12' and nodes[0]['nav2_switch']      # Nav2 전환 노드가 맨 앞
    by_id = {n['id']: n for n in nodes}
    assert by_id['N12']['yaw_deg'] == pytest.approx(0.0)    # N08 → N12 = +x 로 도착
    assert by_id['H2']['yaw_deg'] == pytest.approx(-90.0)   # N08 → H2 = 통로 쪽
    assert by_id['N08']['yaw_deg'] == 0.0                   # 분기 노드는 기본 0


def test_load_layout_rejects_other_yaml(tmp_path):
    p = tmp_path / 'map.yaml'
    p.write_text('image: a.pgm\nresolution: 0.05\n')
    with pytest.raises(AmclCheckError, match='not a layout'):
        load_layout(str(p))


def test_distance_field():
    w, h, data = wall_grid()
    f = DistanceField(w, h, 0.05, 0.0, 0.0, data)
    assert f.distance(0.02, 0.10) == pytest.approx(0.0)           # 벽 칸 안
    assert f.distance(0.20, 0.10) == pytest.approx(0.15)          # 벽(x 0~0.05)에서 15 cm
    assert f.distance(0.30, 0.20) == pytest.approx(0.05)          # 위 벽(y 0.25~)에서 5 cm
    assert f.distance(-0.10, 0.10) is None                        # 지도 밖
    with pytest.raises(AmclCheckError):
        DistanceField(w, h, 0.05, 0.0, 0.0, data[:-1])


def test_distance_field_matches_brute_force():
    w, h, data = wall_grid()
    data[2 * w + 6] = 100   # 가운데 기둥
    f = DistanceField(w, h, 0.05, 0.0, 0.0, data)
    occ = [(i % w, i // w) for i, v in enumerate(data) if v >= 65]
    for r in range(h):
        for c in range(w):
            x, y = (c + 0.5) * 0.05, (r + 0.5) * 0.05
            brute = min(math.hypot(max(ox * 0.05 - x, 0, x - (ox + 1) * 0.05),
                                   max(oy * 0.05 - y, 0, y - (oy + 1) * 0.05)) for ox, oy in occ)
            assert f.distance(x, y) == pytest.approx(brute), (c, r)


def test_scan_to_points_filters_and_transforms():
    ranges = [0.5, float('inf'), 0.01, 1.0, float('nan')]
    pts = scan_to_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                         range_min=0.05, range_max=3.0, pose=(1.0, 2.0, math.pi / 2))
    # i=0: 0.5 m 앞(+y 방향), i=3: 1.0 m, 각도 270° + 90° = 0 방향
    assert pts == [pytest.approx((1.0, 2.5)), pytest.approx((2.0, 2.0))]
    many = scan_to_points([1.0] * 1000, 0.0, 0.001, 0.05, 3.0, (0, 0, 0), max_points=100)
    assert len(many) == 100


def test_match_score():
    w, h, data = wall_grid()
    f = DistanceField(w, h, 0.05, 0.0, 0.0, data)
    pts = [(0.03, 0.1), (0.08, 0.1), (0.13, 0.1), (0.30, 0.1), (5.0, 5.0)]
    s = match_score(pts, f)
    # 거리: 0, 0.03, 0.08, 0.25, 지도 밖(inf)
    assert s['points'] == 5
    assert s['within'] == [0.4, 0.6]
    assert s['median_m'] == pytest.approx(0.08)
    assert s['classes'] == [0, 0, 1, 2, 2]
    assert match_score([], f)['within'] == [None, None]


def test_pose_std():
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = 0.0004, 0.0009, 0.01
    assert pose_std(cov) == (pytest.approx(0.02), pytest.approx(0.03), pytest.approx(0.1))


def test_recorder_writes_error_against_truth(tmp_path):
    rec = AmclRecorder(str(tmp_path / 'amcl'), clock=lambda: 1793616000.0)
    frame = Frame(-1.520, -2.0225, 0.0)
    mx, my, myaw = frame.to_map(378.3, 76.2, 3.0)
    amcl = {'x': mx, 'y': my, 'yaw': myaw, 'std': (0.02, 0.03, 0.05)}
    score = {'points': 300, 'within': [0.7, 0.9], 'median_m': 0.031}
    row = rec.record(frame, amcl, score, 'N12', truth=(378.3, 73.2, 0.0))
    assert row['err_xy_cm'] == pytest.approx(3.0)
    assert row['err_yaw_deg'] == pytest.approx(3.0)
    assert row['std_y_cm'] == 3.0 and row['median_cm'] == 3.1
    rec.record(frame, amcl, score, 'no_truth')
    rows = list(csv.DictReader(open(rec.path)))
    assert [r['label'] for r in rows] == ['N12', 'no_truth'] and rows[1]['err_xy_cm'] == ''
    assert rec.rows(1)[0]['label'] == 'no_truth'


def test_recorder_rejects_bad_input(tmp_path):
    rec = AmclRecorder(str(tmp_path))
    score = {'points': 1, 'within': [1.0, 1.0], 'median_m': 0.0}
    with pytest.raises(AmclCheckError, match='label'):
        rec.record(Frame(), {'x': 0, 'y': 0, 'yaw': 0, 'std': (0, 0, 0)}, score, '../x')
    with pytest.raises(AmclCheckError, match='amcl_pose'):
        rec.record(Frame(), None, score, 'ok')
