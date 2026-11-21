"""인형 상자 → Nav2 비용 지도 점·덮어쓰기 파일(ROS·YOLO·로봇 없음)."""
import math
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from roport_follow.detector import Detection
from roport_follow.floor_geom import CameraModel
from roport_follow.nav2_doll import build_overrides
from roport_follow.obstacles import ObstacleConfig, blind_distance, clear_points, cloud_points, doll_obstacles
from roport_follow.ros_obstacles import ObstacleSink

PINKY = Path(__file__).resolve().parents[2]
BASE_PARAMS = PINKY / 'pinky_pro' / 'pinky_navigation' / 'params' / 'nav2_params.yaml'
SMARTCART_OVERRIDES = PINKY / 'roport_smartcart' / 'config' / 'nav2_overrides.yaml'


def _cam():
    # d660 보정과 비슷한 값(초점 563.5, 높이 6.3 cm, 숙임 8.4°)
    return CameraModel(k=[[563.5, 0, 320], [0, 563.5, 240], [0, 0, 1]], height_m=0.063, pitch_deg=8.4)


def _box_at(cam, x, y, width=0.05, height_m=0.06):
    """바닥 (x 앞, y 왼쪽)에 선 인형의 상자(아래 끝 = 그 점)."""
    ul = cam.ground_to_pixel(x, y + width / 2)
    ur = cam.ground_to_pixel(x, y - width / 2)
    top = cam.ground_to_pixel(x, y, height_m)
    return (ul[0], top[1], ur[0], ul[1])


def _det(name, box, conf=0.9):
    return Detection(0, name, conf, tuple(box))


def test_doll_ahead_becomes_points_in_base_footprint():
    cam, cfg = _cam(), ObstacleConfig(cam_x_m=0.035)
    obs = doll_obstacles(cam, [_det('child', _box_at(cam, 0.50, 0.10, width=0.03))], cfg)
    assert len(obs) == 1
    o = obs[0]
    assert o.x_m == pytest.approx(0.535, abs=0.005) and o.y_m == pytest.approx(0.10, abs=0.005)
    assert o.width_m == pytest.approx(0.03, abs=0.003) and not o.cut
    xs = [p[0] for p in o.points]
    ys = [p[1] for p in o.points]
    assert min(xs) == pytest.approx(0.535, abs=0.01)                 # 가까운 끝 = 상자 아래
    assert max(xs) - min(xs) >= 0.025                                 # 안쪽으로 두께(최소 3 cm)
    assert 0.08 < min(ys) and max(ys) < 0.13
    assert all(p[2] == pytest.approx(0.05) for p in o.points)
    # 1 cm 간격이라 지역 비용 지도(1 cm)에 빈칸 없이 칠해짐
    assert len(o.points) >= 3 * 4


def test_only_adult_child_above_confidence_within_range():
    cam, cfg = _cam(), ObstacleConfig()
    near = _box_at(cam, 0.6, 0.0)
    dets = [_det('human', near), _det('adult', near, conf=0.2), _det('child', _box_at(cam, 1.5, 0.0)),
            _det('adult', near)]
    obs = doll_obstacles(cam, dets, cfg)
    assert [o.name for o in obs] == ['adult']                         # 사람·낮은 신뢰도·먼 인형 제외


def test_feet_cut_box_extends_toward_robot():
    cam, cfg = _cam(), ObstacleConfig()
    box = (300.0, 300.0, 340.0, 480.0)                                # 아래 끝이 화면 아래(발 잘림)
    (o,) = doll_obstacles(cam, [_det('adult', box)], cfg)
    assert o.cut
    edge = cam.pixel_to_ground(320, 479)[0] + cfg.cam_x_m
    assert min(p[0] for p in o.points) < edge - 0.04                  # 화면 아래 끝보다 로봇 쪽까지


def test_clear_points_fan_inside_view_beyond_mark_range():
    cam, cfg = _cam(), ObstacleConfig(cam_x_m=0.035)
    pts = clear_points(cam, cfg)
    half = math.degrees(math.atan2(320, 563.5))
    assert len(pts) >= 20
    gaps = [math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:])]
    assert max(gaps) <= 0.8 * cfg.spacing_m + 1e-3                    # 광선 끝 간격 < 비용 지도 칸(1 cm)
    for x, y, _ in pts:
        assert math.hypot(x - 0.035, y) == pytest.approx(cfg.clear_range_m, abs=1e-3)
        assert abs(math.degrees(math.atan2(y, x - 0.035))) <= half - cfg.fov_margin_deg + 0.01   # 좌표 0.1 mm 반올림
    assert cfg.clear_range_m > cfg.max_range_m


def test_no_doll_still_sends_clear_points_and_blind_zone():
    cam, cfg = _cam(), ObstacleConfig()
    pts, obs = cloud_points(cam, [], cfg)
    assert obs == [] and pts == clear_points(cam, cfg)
    assert 0.05 < blind_distance(cam) < 0.15                          # 렌즈 앞 약 10 cm 는 못 봄


def test_config_rejects_clear_inside_mark_range():
    with pytest.raises(ValueError):
        ObstacleConfig(max_range_m=1.2, clear_range_m=1.0).validate()


def _frame(robot_time, age=0.0, seq=1):
    return SimpleNamespace(robot_time=robot_time, recv_time=time.time() - age, seq=seq, image=None)


def test_sink_uses_robot_time_and_skips_stale_frames():
    cam, sent = _cam(), []
    sink = ObstacleSink(cam, ObstacleConfig(), lambda pts, stamp: sent.append((len(pts), stamp)))
    box = _box_at(cam, 0.5, 0.0)
    assert sink.handle(_frame(1763500000.25), [_det('child', box)])
    assert sent[-1][1] == 1763500000.25 and sent[-1][0] > len(clear_points(cam, sink.cfg))
    sink.handle(_frame(float('nan')), [])
    assert sent[-1][1] is None                                        # 로봇 시각 없음 → 발행 쪽 현재 시각
    assert sink.handle(_frame(1.0, age=2.0), [_det('child', box)]) is None
    assert sink.skipped == 1 and sink.sent == 2 and len(sent) == 2


class _Src:
    def __init__(self):
        self.seq, self.img = 1, np.zeros((480, 640, 3), np.uint8)

    def latest(self):
        return SimpleNamespace(seq=self.seq, image=self.img, robot_time=100.0, recv_time=time.time())


def test_session_publishes_once_per_new_frame_even_with_yolo_hidden(tmp_path):
    from roport_follow.session import ViewSession
    cam, sent, calls = _cam(), [], []
    sink = ObstacleSink(cam, ObstacleConfig(), lambda pts, stamp: sent.append(stamp))
    a = SimpleNamespace(out=str(tmp_path), name='t', save_dir=None, card_mm=30.0, yolo=False, prefix='p',
                        burst_sec=0.5, fps=10.0, lens_height_cm=0.0, camera=str(tmp_path / 'c.json'),
                        obstacle_sink=sink, doll_height_cm=0.0)
    src = _Src()
    ses = ViewSession(a, cam, src)
    box = _box_at(cam, 0.5, 0.0)
    ses.yolo = lambda img: calls.append(1) or [_det('adult', box)]
    ses.step()
    ses.step()                                                        # 같은 영상: 다시 추론·발행 안 함
    src.seq = 2
    ses.step()
    assert len(calls) == 2 and sent == [100.0, 100.0]
    st = ses.state()['obstacles']
    assert st['sent'] == 2 and st['dolls'][0]['name'] == 'adult'
    assert ses.state()['show']['yolo'] is False                       # 화면 표시는 꺼진 그대로


def test_nav2_overrides_add_doll_layer_and_keep_smartcart_values():
    base = yaml.safe_load(BASE_PARAMS.read_text())
    over = yaml.safe_load(SMARTCART_OVERRIDES.read_text())
    out = build_overrides(base, over)
    local = out['local_costmap']['local_costmap']['ros__parameters']
    glob = out['global_costmap']['global_costmap']['ros__parameters']
    assert local['plugins'] == ['voxel_layer', 'doll_layer', 'inflation_layer']
    assert glob['plugins'] == ['static_layer', 'obstacle_layer', 'doll_layer', 'inflation_layer']
    src = local['doll_layer']['doll']
    assert src['topic'] == '/doll_obstacles' and src['data_type'] == 'PointCloud2'
    assert src['obstacle_max_range'] < src['raytrace_max_range']
    # 인형 기억 모드(11-21): 로봇 바로 앞도 찍고 지우며, 가장 최근 묶음 하나만 쓴다(노트북이 기억해 다시 보냄)
    assert src['raytrace_min_range'] == 0.0 and src['obstacle_min_range'] == 0.0
    assert src['observation_persistence'] == 0.0
    old = build_overrides(base, over, keep_near_m=0.15, persistence_s=1.0)   # 예전 방식도 만들 수 있음
    assert old['local_costmap']['local_costmap']['ros__parameters']['doll_layer']['doll']['raytrace_min_range'] == 0.15
    assert src['expected_update_rate'] == 1.0
    assert glob['doll_layer']['doll']['expected_update_rate'] == 0.0
    # smartcart 값은 그대로(예: 지역 해상도·팽창, planner 허용 오차)
    assert local['resolution'] == 0.01 and local['inflation_layer']['inflation_radius'] == 1.0
    assert out['planner_server'] == over['planner_server']
    assert 'doll_layer' not in str(over)                              # 원본 dict 는 안 바뀜
    # 두 번 넣어도 한 번만
    again = build_overrides(base, out)
    assert again['local_costmap']['local_costmap']['ros__parameters']['plugins'].count('doll_layer') == 1


def test_overrides_merge_through_smartcart_build_params(tmp_path):
    profiles = pytest.importorskip('roport_smartcart.profiles')
    path = tmp_path / 'doll.yaml'
    path.write_text(yaml.safe_dump(build_overrides(yaml.safe_load(BASE_PARAMS.read_text()),
                                                   yaml.safe_load(SMARTCART_OVERRIDES.read_text()))))
    params = profiles.build_params(str(BASE_PARAMS), str(path), 'real',
                                   footprint=[[.06, .06], [.06, -.06], [-.06, -.06], [-.06, .06]],
                                   max_linear=.08, max_angular=.5)
    local = params['local_costmap']['local_costmap']['ros__parameters']
    assert 'doll_layer' in local['plugins'] and local['doll_layer']['doll']['topic'] == '/doll_obstacles'
    assert params['global_costmap']['global_costmap']['ros__parameters']['doll_layer']['enabled'] is True


def _unseen_cleared_cells(cam, cfg, keep_near=0.15, raytrace_max=1.30):
    """바퀴 축 → 지움 점 광선 위 keep_near~끝 사이(1 cm 간격) 중 카메라 화면 밖인 점 수."""
    n = 0
    for x, y, _ in clear_points(cam, cfg):
        r = math.hypot(x, y)
        s = keep_near
        while s <= min(r, raytrace_max):
            gx, gy = x * s / r - cfg.cam_x_m, y * s / r - cfg.cam_y_m
            uv = cam.ground_to_pixel(gx, gy) if gx > 0 else None
            if uv is None or not (0 <= uv[0] < cam.width and 0 <= uv[1] < cam.height):
                n += 1
            s += 0.01
    return n


def test_clear_rays_never_wipe_cells_the_camera_cannot_see():
    cam = _cam()
    assert _unseen_cleared_cells(cam, ObstacleConfig()) == 0            # 기본 6°
    assert _unseen_cleared_cells(cam, ObstacleConfig(fov_margin_deg=3.0)) > 0   # 3°면 화면 아래 모서리 사각을 지움


@pytest.mark.parametrize('name', ['camera_d660.json', 'camera_6422.json'])
def test_clear_rays_inside_view_with_real_calibration(name):
    path = Path('~/roport_follow').expanduser() / name
    if not path.exists():
        pytest.skip(f'{path} 없음(노트북 보정 파일, 저장소 밖)')
    assert _unseen_cleared_cells(CameraModel.load(path), ObstacleConfig()) == 0


def test_sink_skips_robot_to_laptop_lag_and_missing_robot_time():
    cam, sent = _cam(), []
    sink = ObstacleSink(cam, ObstacleConfig(), lambda pts, stamp: sent.append(stamp), require_robot_time=True)
    now = time.time()
    f = lambda rt: SimpleNamespace(robot_time=rt, recv_time=now, seq=1, image=None)  # noqa: E731
    assert sink.handle(f(now - 0.10), []) is not None                  # 최소 지연 0.10 s(시계 차 포함)
    assert sink.handle(f(now - 0.30), []) is not None and sink.lag_s == pytest.approx(0.20)
    assert sink.handle(f(now - 0.75), []) is None                      # 최소보다 0.65 s 더 밀림
    assert sink.handle(f(float('nan')), []) is None                    # 로봇 영상인데 로봇 시각 없음
    assert sink.skipped == 2 and len(sent) == 2


def test_overrides_in_sim_keep_inflation_last(tmp_path):
    profiles = pytest.importorskip('roport_smartcart.profiles')
    path = tmp_path / 'doll.yaml'
    path.write_text(yaml.safe_dump(build_overrides(yaml.safe_load(BASE_PARAMS.read_text()),
                                                   yaml.safe_load(SMARTCART_OVERRIDES.read_text()))))
    params = profiles.build_params(str(BASE_PARAMS), str(path), 'sim')
    assert params['local_costmap']['local_costmap']['ros__parameters']['plugins'] == [
        'voxel_layer', 'doll_layer', 'sonar_layer', 'inflation_layer']


def test_sink_info_rate_pause_and_geometry():
    cam, sent = _cam(), []
    sink = ObstacleSink(cam, ObstacleConfig(), lambda pts, stamp: sent.append(stamp))
    box = _box_at(cam, 0.5, 0.0)
    for _ in range(4):
        sink.handle(_frame(100.0), [_det('child', box)])
    info = sink.info()
    assert info['sent'] == 4 and info['rate_hz'] == 2.0 and info['last_age_s'] < 0.5
    assert info['dolls'][0]['name'] == 'child' and info['points_cm']
    assert info['geom']['fov_left_deg'] == pytest.approx(29.6 - 6.0, abs=0.1)
    sink.paused = True
    assert sink.handle(_frame(100.0), [_det('child', box)]) is None and len(sent) == 4   # 멈춤: 안 보냄
    assert sink.info(now=time.time() + 3)['rate_hz'] == 0.0


def test_session_test_log_csv(tmp_path):
    import csv
    from roport_follow.session import ViewSession
    cam = _cam()
    sink = ObstacleSink(cam, ObstacleConfig(), lambda pts, stamp: None)
    a = SimpleNamespace(out=str(tmp_path), name='t', save_dir=None, card_mm=30.0, yolo=False, prefix='p',
                        burst_sec=0.5, fps=10.0, lens_height_cm=0.0, camera=str(tmp_path / 'c.json'),
                        obstacle_sink=sink, doll_height_cm=0.0)
    ses = ViewSession(a, cam, _Src())
    assert ses.obstacle_pause(True) is True and sink.paused
    path = ses.test_log(4, 'min 18cm\x07')
    ses.test_log(5)
    rows = list(csv.reader(open(path)))
    assert rows[0][:3] == ['time', 'step', 'note'] and rows[1][1:3] == ['4', 'min 18cm'] and len(rows) == 3
    with pytest.raises(ValueError):
        ses.test_log(True)


def test_check_overrides_reports_same_different_missing(tmp_path):
    from roport_follow.nav2_doll import check_overrides
    out = build_overrides(yaml.safe_load(BASE_PARAMS.read_text()), yaml.safe_load(SMARTCART_OVERRIDES.read_text()))
    p = tmp_path / 'doll.yaml'
    p.write_text(yaml.safe_dump(out))
    assert check_overrides(p, '/doll_obstacles', 1.2)['status'] == 'same'
    assert check_overrides(p, '/doll_obstacles', 1.0)['status'] == 'different'
    assert check_overrides(tmp_path / 'none.yaml', '/doll_obstacles', 1.2)['status'] == 'missing'
