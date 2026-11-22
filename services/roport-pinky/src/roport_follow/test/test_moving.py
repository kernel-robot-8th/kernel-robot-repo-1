"""움직이는 인형 예측(FFT1-190): odom 기준 추적, 로봇 움직임 빼기, 예상 자리 점, 버퍼 보간, 발행부 연결."""
import math
import time
from types import SimpleNamespace

import pytest

from roport_follow.moving import MovingConfig, MovingTracker, to_base, to_odom
from roport_follow.obstacles import DollObstacle
from roport_follow.ros_obstacles import OdomBuffer


def doll(x, y, name='child'):
    pts = tuple((x + dx, y + dy, 0.05) for dx in (0.0, 0.02) for dy in (-0.01, 0.0, 0.01))
    return DollObstacle(name, 0.9, x, y, 0.03, False, pts)


def test_odom_base_round_trip():
    pose = (1.0, -0.5, math.radians(30))
    ox, oy = to_odom(pose, 0.4, 0.1)
    assert to_base(pose, ox, oy) == pytest.approx((0.4, 0.1))


def test_static_doll_with_moving_robot_is_not_moving():
    """로봇이 0.1 m/s 로 앞으로 가면 로봇 좌표에서 인형이 다가오지만 odom 에서는 멈춰 있음."""
    tr = MovingTracker(MovingConfig())
    for k in range(6):
        t, rx = 0.1 * k, 0.01 * k
        tracks = tr.update(t, [doll(1.0 - rx, 0.0)], (rx, 0.0, 0.0))
    assert len(tracks) == 1 and tracks[0].speed < 0.01 and not tr.moving(tracks[0])
    assert tr.predicted_points((0.05, 0.0, 0.0), tracks) == []


def test_crossing_doll_velocity_and_prediction():
    cfg = MovingConfig(horizon_s=1.0, step_s=0.1)
    tr = MovingTracker(cfg)
    pose = (0.0, 0.0, 0.0)
    for k in range(6):                                  # 왼쪽에서 오른쪽으로 0.1 m/s
        tracks = tr.update(0.1 * k, [doll(0.8, 0.3 - 0.01 * k)], pose)
    t0 = tracks[0]
    assert t0.vx == pytest.approx(0.0, abs=1e-6) and t0.vy == pytest.approx(-0.1, abs=1e-3)
    assert tr.moving(t0)
    pts = tr.predicted_points(pose, tracks)
    assert len(pts) == 10 * 3                           # 10 단계 × 점 절반(6 → 3)
    assert min(p[1] for p in pts) == pytest.approx(0.25 - 0.1 - 0.01, abs=2e-3)    # 1 s 뒤 0.1 m 더 오른쪽


def test_slow_doll_not_predicted_and_needs_hits():
    tr = MovingTracker(MovingConfig(min_speed_mps=0.03, min_hits=3))
    pose = (0.0, 0.0, 0.0)
    a = tr.update(0.0, [doll(0.8, 0.0)], pose)
    b = tr.update(0.1, [doll(0.8, -0.01)], pose)        # 0.1 m/s 지만 2번째
    assert not tr.moving(b[0])
    for k in range(2, 6):
        c = tr.update(0.1 * k, [doll(0.8, -0.001 * k)], pose)   # 0.01 m/s 로 느려짐
    assert a[0].id == c[0].id and not tr.moving(c[0])


def test_association_by_class_and_gate():
    tr = MovingTracker(MovingConfig(gate_m=0.2, max_speed_mps=0.5))
    pose = (0.0, 0.0, 0.0)
    a = tr.update(0.0, [doll(0.8, 0.0, 'adult'), doll(0.8, 0.5, 'child')], pose)
    b = tr.update(0.1, [doll(0.82, 0.0, 'adult'), doll(0.8, 0.52, 'child')], pose)
    assert [x.id for x in a] == [x.id for x in b]
    c = tr.update(0.2, [doll(0.82, -0.5, 'adult')], pose)     # 0.1 s 에 0.5 m: 새 트랙
    assert c[0].id not in [x.id for x in a]


def test_no_pose_disables_tracking():
    tr = MovingTracker(MovingConfig())
    assert tr.update(0.0, [doll(0.8, 0.0)], None) == [] and tr.tracks == []


def test_tracks_dropped_after_timeout():
    tr = MovingTracker(MovingConfig(drop_after_s=0.5))
    pose = (0.0, 0.0, 0.0)
    a = tr.update(0.0, [doll(0.8, 0.0)], pose)
    tr.update(0.7, [], pose)
    b = tr.update(0.8, [doll(0.8, 0.0)], pose)
    assert a[0].id != b[0].id


def test_summary_time_to_contact_for_approaching_doll():
    tr = MovingTracker(MovingConfig())
    pose = (0.0, 0.0, 0.0)
    for k in range(5):
        tracks = tr.update(0.1 * k, [doll(1.0 - 0.02 * k, 0.0)], pose)     # 0.2 m/s 다가옴
    s = tr.summary(pose, tracks)[0]
    assert s['moving'] and s['ttc_s'] == pytest.approx(0.92 / 0.2, abs=0.2)


def test_config_validation():
    with pytest.raises(ValueError):
        MovingConfig(min_speed_mps=0.6, max_speed_mps=0.5).validate()
    with pytest.raises(ValueError):
        MovingConfig(horizon_s=0.0).validate()


def test_odom_buffer_interpolates_and_bounds():
    b = OdomBuffer(None, keep_s=3.0, lead_s=0.2)
    b.add(10.0, 0.0, 0.0, 0.0)
    b.add(10.1, 0.01, 0.0, math.radians(10))
    x, y, yaw = b(10.05)
    assert x == pytest.approx(0.005) and math.degrees(yaw) == pytest.approx(5.0, abs=0.01)
    assert b(10.25) == pytest.approx((0.01, 0.0, math.radians(10)))      # 0.2 s 안: 마지막 값
    assert b(10.4) is None and b(9.9) is None


def test_obstacle_sink_adds_predicted_points_only_with_pose():
    from roport_follow.floor_geom import CameraModel
    from roport_follow.obstacles import ObstacleConfig
    from roport_follow.ros_obstacles import ObstacleSink
    from roport_follow.detector import Detection

    cam = CameraModel()
    sent = []
    poses = {'ok': True}
    sink = ObstacleSink(cam, ObstacleConfig(), lambda pts, st: sent.append(len(pts)), require_robot_time=True,
                        motion=MovingTracker(MovingConfig()),
                        pose_lookup=lambda t: (0.0, 0.0, 0.0) if poses['ok'] else None)
    # 같은 인형 상자를 오른쪽으로 옮겨 가며(화면 왼쪽 → 오른쪽) 보낸다
    for k in range(6):
        u = 250 + 20 * k
        det = Detection(0, 'child', 0.9, (u, 200.0, u + 30, 260.0))
        fr = SimpleNamespace(recv_time=time.time(), robot_time=100.0 + 0.1 * k)
        sink.handle(fr, [det])
    info = sink.info()
    assert info['moving']['pose_ok'] and info['moving']['tracks'][0]['moving'] and sink.predicted > 0
    with_pred = sent[-1]
    poses['ok'] = False
    sink.handle(SimpleNamespace(recv_time=time.time(), robot_time=100.7), [Detection(0, 'child', 0.9, (370.0, 200.0, 400.0, 260.0))])
    assert sink.predicted == 0 and sent[-1] < with_pred and not sink.info()['moving']['pose_ok']
