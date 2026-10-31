"""마커 인식 확인 로직 테스트 (카메라·ROS 없이 합성 영상으로)."""

import time

import cv2
import numpy as np
import pytest

from pinky_test_console.marker_detect import (
    estimate_distance_m,
    load_marker_table,
    MarkerDetector,
    MarkerError,
    MarkerWorker,
    parse_camera_spec,
    PushSource,
    ros_image_to_bgr,
)

TABLE_YAML = """
dictionary: DICT_4X4_50
markers:
  - {id: 1, name: P1, group: place, size_mm: 150, quiet_mm: 10, mount: wall, use: "스마트카트 대기"}
  - {id: 7, name: P7, group: place, size_mm: 100, quiet_mm: 10, mount: wall, use: "인도장 정차"}
range_test: {id: 0, sizes_mm: [150]}
"""


def marker_frame(marker_id, side=120, at=(100, 80), size=(480, 640)):
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    img = np.full(size, 255, np.uint8)
    img[at[1]:at[1] + side, at[0]:at[0] + side] = gen(dictionary, marker_id, side)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


class FakeClock:
    def __init__(self):
        self.t = 10.0

    def __call__(self):
        return self.t


@pytest.fixture
def table_path(tmp_path):
    p = tmp_path / 'markers.yaml'
    p.write_text(TABLE_YAML, encoding='utf-8')
    return str(p)


@pytest.mark.parametrize('spec,expected', [
    ('picamera2', ('picamera2', None)),
    ('v4l2:0', ('v4l2', 0)),
    (' v4l2:12 ', ('v4l2', 12)),
    ('topic:/camera/image_raw', ('topic', '/camera/image_raw')),
])
def test_parse_camera_spec(spec, expected):
    assert parse_camera_spec(spec) == expected


@pytest.mark.parametrize('spec', ['', 'v4l2:', 'v4l2:abc', 'rtsp://x', 'topic:', 'topic:a b',
                                  'topic:/a;rm', '/dev/video0'])
def test_parse_camera_spec_rejects(spec):
    with pytest.raises(MarkerError):
        parse_camera_spec(spec)


def test_load_table(table_path):
    dict_name, table = load_marker_table(table_path)
    assert dict_name == 'DICT_4X4_50'
    assert table[7] == {'name': 'P7', 'size_mm': 100.0, 'use': '인도장 정차'}
    assert table[0]['name'] == 'TEST'
    assert load_marker_table('') == ('DICT_4X4_50', {})


def test_detect_synthetic_marker():
    dets = MarkerDetector().detect(marker_frame(7, side=120))
    assert [d['id'] for d in dets] == [7]
    assert dets[0]['side_px'] == pytest.approx(120, abs=3)


def test_detect_nothing_on_blank():
    assert MarkerDetector().detect(np.full((480, 640, 3), 255, np.uint8)) == []


def test_unknown_dictionary():
    with pytest.raises(MarkerError):
        MarkerDetector('DICT_NOPE')


def test_distance_estimate():
    # hfov 60°, 폭 640 px -> f = 320 / tan(30°) = 554.3 px. 100 mm 마커가 100 px -> 0.554 m
    assert estimate_distance_m(100, 640, 60, 100) == pytest.approx(0.5543, abs=1e-3)
    assert estimate_distance_m(100, 640, 0, 100) is None
    assert estimate_distance_m(0, 640, 60, 100) is None


def test_worker_checklist_and_rate(table_path):
    clock = FakeClock()
    dict_name, table = load_marker_table(table_path)
    w = MarkerWorker(PushSource(), MarkerDetector(dict_name), table, hfov_deg=60, clock=clock)
    for i in range(10):
        clock.t += 0.1
        w.process(marker_frame(7) if i % 2 == 0 else marker_frame(99 % 50))
    s = w.snapshot()
    assert s['fps'] == pytest.approx(10, abs=0.5)
    items = {c['id']: c for c in s['checklist']}
    assert items[7]['seen'] and items[7]['rate'] == pytest.approx(0.5)
    assert items[1]['seen'] is False and items[1]['in_table']
    assert items[49]['in_table'] is False and items[49]['seen']   # 표에 없는 ID 도 보인다
    assert s['detections'][0]['corners'] is None                 # JSON 을 가볍게
    assert w.jpeg()[:2] == b'\xff\xd8'
    w.reset_seen()
    assert not {c['id']: c for c in w.snapshot()['checklist']}[7]['seen']


def test_worker_distance_uses_table_size(table_path):
    _, table = load_marker_table(table_path)
    w = MarkerWorker(PushSource(), MarkerDetector(), table, hfov_deg=60)
    d = w.process(marker_frame(7, side=100))[0]
    assert d['name'] == 'P7' and d['distance_m'] == pytest.approx(0.55, abs=0.03)


def test_worker_rotate_180_still_detects():
    w = MarkerWorker(PushSource(), MarkerDetector(), rotate_180=True)
    assert [d['id'] for d in w.process(marker_frame(1))] == [1]


def test_worker_thread_with_push_source():
    src = PushSource(timeout=0.05)
    w = MarkerWorker(src, MarkerDetector())
    w.start()
    src.push(marker_frame(3))
    deadline = time.monotonic() + 2.0
    while w.jpeg() is None and time.monotonic() < deadline:
        time.sleep(0.01)
    w.stop()
    assert w.jpeg() is not None
    assert {c['id'] for c in w.snapshot()['checklist']} == {3}


def test_push_source_timeout():
    assert PushSource(timeout=0.01).read() is None


def test_ros_image_conversion_with_padding():
    rgb = np.zeros((2, 3, 3), np.uint8)
    rgb[..., 0] = 200          # R
    step = 3 * 3 + 2           # 줄 끝 여백 2바이트
    data = b''.join(row.tobytes() + b'\x00\x00' for row in rgb)
    bgr = ros_image_to_bgr(2, 3, 'rgb8', step, data)
    assert bgr.shape == (2, 3, 3) and bgr[0, 0].tolist() == [0, 0, 200]
    mono = ros_image_to_bgr(1, 2, 'mono8', 2, b'\x05\x06')
    assert mono.shape == (1, 2, 3)
    with pytest.raises(MarkerError):
        ros_image_to_bgr(1, 1, 'yuv422', 2, b'\x00\x00')


def test_range_recorder_stats_and_csv(tmp_path, table_path):
    from pinky_test_console.marker_detect import RangeRecorder
    clock = FakeClock()
    _, table = load_marker_table(table_path)
    rec = RangeRecorder(str(tmp_path / 'mk'), table, clock=clock)
    w = MarkerWorker(PushSource(), MarkerDetector(), table, clock=clock, recorder=rec)
    rec.start(58, 1, note='벽 높이 2 cm')
    for i in range(10):
        clock.t += 0.11
        w.process(marker_frame(7, side=100) if i % 5 else np.full((480, 640, 3), 255, np.uint8))
    r = rec.last_result
    assert r['success'] and r['marker_id'] == 7 and r['name'] == 'P7'
    assert r['rate'] == pytest.approx(0.8, abs=0.11)        # 빈 화면 2장
    assert r['side_px_med'] == pytest.approx(100, abs=3)
    assert r['top_px'] == pytest.approx(80, abs=2) and r['frame_h'] == 480
    assert r['pred_50mm_px'] == pytest.approx(50, abs=2)    # P7 100 mm 기준
    rows = rec.records()
    assert len(rows) == 1 and rows[0]['distance_cm'] == '58.0' and rows[0]['note'] == '벽 높이 2 cm'


def test_range_recorder_fixed_id_not_seen(tmp_path):
    from pinky_test_console.marker_detect import RangeRecorder
    clock = FakeClock()
    rec = RangeRecorder(str(tmp_path / 'mk'), clock=clock)
    rec.start(20, 1, marker_id=4)
    for _ in range(3):
        clock.t += 0.4
        rec.add([{'id': 7, 'corners': [[0, 0], [1, 0], [1, 1], [0, 1]], 'side_px': 1.0}], 480)
    r = rec.last_result
    assert r['marker_id'] == 4 and r['rate'] == 0.0 and r['side_px_med'] is None


def test_range_recorder_validation_and_no_frames(tmp_path):
    from pinky_test_console.marker_detect import RangeRecorder
    clock = FakeClock()
    rec = RangeRecorder(str(tmp_path / 'mk'), clock=clock)
    for kw in ({'distance_cm': 0, 'duration': 5}, {'distance_cm': 30, 'duration': 31},
               {'distance_cm': 30, 'duration': 5, 'marker_id': 50},
               {'distance_cm': 30, 'duration': 5, 'marker_id': '7'},
               {'distance_cm': True, 'duration': 5},
               {'distance_cm': 30, 'duration': 5, 'note': 'a,b'}):
        with pytest.raises(MarkerError):
            rec.start(**kw)
    rec.start(30, 1)
    with pytest.raises(MarkerError, match='already'):
        rec.start(30, 1)
    clock.t += 2
    assert rec.poll()['success'] is False and rec.records() == []
