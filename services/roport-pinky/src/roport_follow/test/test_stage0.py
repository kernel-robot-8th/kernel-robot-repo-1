"""FFT1-186 0단계 도구: 바닥 기하·체스보드 바닥 자세·MPJPEG·카드 거리·지연 계산(하드웨어 없음)."""
import math

import cv2
import numpy as np
import pytest

from roport_follow.calib import board_points, floor_pose
from roport_follow.floor_geom import CameraModel, box_to_ground, distance_from_height, fit_pose
from roport_follow.markers import (CARD_DICT, CardDetector, ClockReader, clock_image, clock_id,
                                   latency_ms)
from roport_follow.stream import iter_mpjpeg


def test_principal_ray_hits_floor_at_h_over_tan_pitch():
    cam = CameraModel(height_m=0.063, pitch_deg=18.35)
    x, y = cam.pixel_to_ground(320, 240)
    assert x == pytest.approx(0.063 / math.tan(math.radians(18.35)), rel=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('roll', [0.0, 3.0, -4.0])
@pytest.mark.parametrize('pt', [(0.25, 0.0), (0.6, 0.15), (1.2, -0.3), (0.4, 0.2)])
def test_ground_pixel_round_trip(pt, roll):
    cam = CameraModel(pitch_deg=18.0, roll_deg=roll, dist=[0.05, -0.02, 0.0, 0.0, 0.0])
    uv = cam.ground_to_pixel(*pt)
    assert uv is not None
    back = cam.pixel_to_ground(*uv)
    assert back[0] == pytest.approx(pt[0], abs=1e-4)
    assert back[1] == pytest.approx(pt[1], abs=1e-4)


def test_above_horizon_is_not_floor():
    cam = CameraModel()
    assert cam.pixel_to_ground(320, cam.horizon_v() - 5) is None


def test_left_is_positive_y():
    cam = CameraModel()
    assert cam.pixel_to_ground(100, 400)[1] > 0      # 화면 왼쪽 = 로봇 왼쪽
    assert cam.pixel_to_ground(540, 400)[1] < 0


def test_box_to_ground_matches_projected_doll():
    cam = CameraModel()
    foot = cam.ground_to_pixel(0.6, 0.05)
    left = cam.ground_to_pixel(0.6, 0.065)
    right = cam.ground_to_pixel(0.6, 0.035)
    box = (left[0], foot[1] - 80, right[0], foot[1])
    g = box_to_ground(cam, box)
    assert g.x_m == pytest.approx(0.6, abs=0.005)
    assert g.y_m == pytest.approx(0.05, abs=0.005)
    assert g.width_m == pytest.approx(0.03, abs=0.003)
    assert g.bearing_deg > 0


def test_box_bottom_above_horizon_returns_none_and_bad_box_rejected():
    cam = CameraModel()
    hv = cam.horizon_v()
    assert box_to_ground(cam, (300, hv - 50, 340, hv - 10)) is None
    with pytest.raises(ValueError):
        box_to_ground(cam, (10, 10, 5, 20))


def test_distance_from_height_simple_pinhole():
    cam = CameraModel()
    assert distance_from_height(cam, (0, 100, 10, 208), 0.12) == pytest.approx(0.6)
    assert distance_from_height(cam, (0, 100, 10, 100), 0.12) is None


def test_visible_top_height_reproduces_1031_estimate():
    # 기본값은 「58 cm 앞에서 12 cm까지 보임」에서 나온 값이라 같은 답이 나와야 한다
    assert CameraModel().visible_top_height(0.58) == pytest.approx(0.12, abs=0.002)


@pytest.mark.parametrize('bad', [dict(height_m=0), dict(pitch_deg=95), dict(k=[[0, 0, 0]] * 3),
                                 dict(dist=[0.0, 0.0, 0.0])])
def test_invalid_camera_rejected(bad, tmp_path):
    cam = CameraModel(**bad)
    with pytest.raises(ValueError):
        cam.validate()


def test_camera_json_round_trip(tmp_path):
    cam = CameraModel(height_m=0.0655, pitch_deg=17.2, roll_deg=1.1, source='test')
    p = tmp_path / 'cam.json'
    cam.save(p, extra={'note': 'x'})
    back = CameraModel.load(p)
    assert (back.height_m, back.pitch_deg, back.roll_deg, back.source) == (0.0655, 17.2, 1.1, 'test')


@pytest.mark.parametrize('h,pitch,roll', [(0.063, 18.35, 0.0), (0.07, 22.0, 2.0), (0.058, 15.0, -1.5)])
def test_floor_pose_recovers_camera_from_board_on_floor(h, pitch, roll):
    cam = CameraModel(height_m=h, pitch_deg=pitch, roll_deg=roll)
    # 보드를 렌즈 아래에서 앞 12~25 cm, 가운데에 평평하게 둔다(바닥 좌표 → 화소)
    obj = board_points()
    x0, y0 = 0.12, 0.10
    corners = []
    for px, py, _ in obj:
        uv = cam.ground_to_pixel(x0 + py, y0 - px)   # 보드 X = 로봇 오른쪽, Y = 앞
        corners.append(uv)
    corners = np.array(corners, np.float32).reshape(-1, 1, 2)
    fp = floor_pose(corners, cam.K, cam.D)
    assert fp.height_m == pytest.approx(h, abs=2e-4)
    assert fp.pitch_deg == pytest.approx(pitch, abs=0.05)
    assert fp.roll_deg == pytest.approx(roll, abs=0.05)
    assert fp.reproj_px < 0.01
    fp2 = floor_pose(corners, cam.K, cam.D, board_thickness_m=0.003)
    assert fp2.height_m == pytest.approx(h + 0.003, abs=2e-4)


def _part(data, stamp=None):
    head = b'--frame\r\nContent-Type: image/jpeg\r\n'
    if stamp is not None:
        head += b'X-Robot-Time: %.3f\r\n' % stamp
    return head + b'Content-Length: %d\r\n\r\n' % len(data) + data + b'\r\n'


def test_mpjpeg_parser_handles_split_chunks_and_headers():
    blob = _part(b'abc', 1.5) + _part(b'\xff\xd8xyz' * 10)
    chunks = [blob[i:i + 7] for i in range(0, len(blob), 7)]
    it = iter(chunks)
    out = list(iter_mpjpeg(lambda n: next(it, b'')))
    assert [d for _, d in out] == [b'abc', b'\xff\xd8xyz' * 10]
    assert out[0][0]['x-robot-time'] == '1.500'


def test_mpjpeg_parser_rejects_missing_length_and_huge_frames():
    bad = b'--frame\r\nContent-Type: image/jpeg\r\n\r\nxxxx'
    with pytest.raises(ValueError):
        it = iter([bad, b''])
        list(iter_mpjpeg(lambda n: next(it, b'')))
    huge = b'--frame\r\nContent-Length: 999999999\r\n\r\n'
    with pytest.raises(ValueError):
        it2 = iter([huge, b''])
        list(iter_mpjpeg(lambda n: next(it2, b''), max_frame=1000))


def test_latency_math_wraps():
    assert latency_ms(clock_id(10_000), 10_000) == 0
    assert latency_ms(clock_id(10_000), 10_000 + 120) == 120
    assert latency_ms(clock_id(9_990), 9_990 + 250 * 40 - 40) == 250 * 40 - 40   # 감는 경계


def test_clock_image_is_readable():
    img = clock_image(123_456, 400)
    assert ClockReader()(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)) == clock_id(123_456)


def _card_scene(cam, marker_id, size_m, dist_m):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, CARD_DICT))
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    tex = gen(d, marker_id, 120)
    tex = cv2.copyMakeBorder(tex, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)
    s = size_m * 160 / 120 / 2                       # 흰 여백 포함 텍스처의 반 크기(m)
    obj = np.array([[-s, -s, dist_m], [s, -s, dist_m], [s, s, dist_m], [-s, s, dist_m]], np.float32)
    uv, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), cam.K, cam.D)
    src = np.array([[0, 0], [159, 0], [159, 159], [0, 159]], np.float32)
    m = cv2.getPerspectiveTransform(src, uv.reshape(4, 2).astype(np.float32))
    img = cv2.warpPerspective(tex, m, (640, 480), borderValue=200)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_card_distance_from_pnp():
    cam = CameraModel()
    img = _card_scene(cam, 35, 0.03, 0.40)
    cards = CardDetector(cam.K, cam.D, size_m=0.03)(img)
    assert [c.id for c in cards] == [35]
    assert cards[0].dist_m == pytest.approx(0.40, rel=0.03)


def test_card_ids_outside_range_ignored():
    cam = CameraModel()
    img = _card_scene(cam, 12, 0.03, 0.40)       # 4X4_50 의 12번 = 선반 등 기존 마커(고객 칸 35~49 밖)
    assert CardDetector(cam.K, cam.D, size_m=0.03)(img) == []


def _clicks(cam, ds):
    return [(*cam.ground_to_pixel(d, 0.0), d) for d in ds]


def test_fit_pose_with_measured_height_from_one_click():
    truth = CameraModel(height_m=0.064, pitch_deg=21.3)
    guess = CameraModel()                       # 기본 추정값에서 시작
    h, p, rms = fit_pose(guess, _clicks(truth, [0.40]), fix_height_m=0.064)
    assert (h, p) == (pytest.approx(0.064), pytest.approx(21.3, abs=0.02))
    assert rms < 0.05


def test_fit_pose_height_and_pitch_from_several_clicks():
    truth = CameraModel(height_m=0.0672, pitch_deg=16.4, roll_deg=1.5)
    guess = CameraModel(roll_deg=1.5)
    h, p, rms = fit_pose(guess, _clicks(truth, [0.25, 0.40, 0.60, 0.80, 1.20]))
    assert h == pytest.approx(0.0672, abs=0.0003)
    assert p == pytest.approx(16.4, abs=0.05)


def test_fit_pose_rejects_bad_input():
    with pytest.raises(ValueError):
        fit_pose(CameraModel(), [])
    with pytest.raises(ValueError):
        fit_pose(CameraModel(), [(320, 400, 0.3)])
