"""고객 카드 추종 대상(FFT1-188 1단계): 카드 → base_footprint 바닥 점, 놓침, 튐 거름, 발행 규칙."""
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from roport_follow.floor_geom import CameraModel
from roport_follow.follow import LOST, SEARCHING, TRACKING, FollowConfig, FollowTracker, card_floor_point
from roport_follow.markers import Card, CardDetector
from roport_follow.ros_follow import FollowSink

CAM = CameraModel()          # 기본 렌즈 높이 6.3 cm·숙임 18.35°
CFG = FollowConfig(card_id=35)


def card_at(x, y, h=0.10, cid=35, side_px=20.0, cam=CAM, cfg=CFG):
    """base_footprint (x, y), 높이 h 의 카드 중심 → solvePnP 가 냈을 카메라 좌표의 Card."""
    p = np.array([x - cfg.cam_x_m, y - cfg.cam_y_m, h - cam.height_m])
    t = cam.rotation().T @ p
    return Card(cid, np.zeros((4, 2)), float(np.linalg.norm(t)), float(t[0]), float(t[2]), side_px, float(t[1]))


def render_card(x, y, h=0.10, size=0.03, cid=35, cam=CAM, cfg=CFG):
    """카드(로봇을 바라보는 세로 판)를 합성 영상에 그린다. 검은 사각형 한 변 = size."""
    import cv2
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gen = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    m = gen(d, cid, 200)
    cx, cy = x - cfg.cam_x_m, y - cfg.cam_y_m
    s = size / 2
    # 화면 왼쪽 = 로봇 왼쪽(+Y): 왼쪽 위, 오른쪽 위, 오른쪽 아래, 왼쪽 아래
    pts3 = [(cx, cy + s, h + s), (cx, cy - s, h + s), (cx, cy - s, h - s), (cx, cy + s, h - s)]
    dst = np.array([cam.ground_to_pixel(*p) for p in pts3], np.float32)
    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    img = np.full((cam.height, cam.width), 255, np.uint8)
    warp = cv2.warpPerspective(m, cv2.getPerspectiveTransform(src, dst), (cam.width, cam.height),
                               flags=cv2.INTER_LINEAR, borderValue=255)
    mask = cv2.warpPerspective(np.full_like(m, 255), cv2.getPerspectiveTransform(src, dst),
                               (cam.width, cam.height), flags=cv2.INTER_NEAREST, borderValue=0)
    img[mask > 0] = warp[mask > 0]
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


@pytest.mark.parametrize('x, y', [(0.5, 0.0), (0.8, 0.15), (1.0, -0.2)])
def test_floor_point_inverts_camera_geometry(x, y):
    fx, fy = card_floor_point(CAM, card_at(x, y), CFG)
    assert fx == pytest.approx(x, abs=1e-6) and fy == pytest.approx(y, abs=1e-6)


def test_floor_point_needs_camera_y():
    c = card_at(0.8, 0.0)
    no_y = Card(c.id, c.corners, c.dist_m, c.x_m, c.z_m, c.side_px)      # y_m 기본 nan
    assert card_floor_point(CAM, no_y, CFG) is None


@pytest.mark.parametrize('x, y', [(0.6, 0.0), (0.9, 0.12), (1.2, -0.15)])
def test_rendered_card_detected_near_true_floor_point(x, y):
    det = CardDetector(CAM.K, CAM.D, size_m=0.03, ids=[35])
    cards = det(render_card(x, y))
    assert [c.id for c in cards] == [35]
    fx, fy = card_floor_point(CAM, cards[0], CFG)
    # 합성 영상(서브픽셀 보정): 1.2 m 까지 5 cm 안. OpenCV 4.11 에서는 3 cm 안이지만 CI 의 OpenCV 4.6(apt)은
    # 모서리 보정 결과가 달라 1.2 m 에서 4.05 cm 가 나온다(11-21 사용자 결정으로 기준 3 → 5 cm).
    # 실제 카메라 정확도는 멈춘 카드 실측으로 따로 확인
    assert math.hypot(fx - x, fy - y) < 0.05


def test_tracker_states_and_lost_after_timeout():
    tr = FollowTracker(CAM, FollowConfig(card_id=35, lost_after_s=1.0))
    assert tr.update(0.0, [])['state'] == SEARCHING
    st = tr.update(0.1, [card_at(0.8, 0.0)])
    assert st['state'] == TRACKING and st['seen'] and st['x_m'] == pytest.approx(0.8, abs=1e-3)
    st = tr.update(0.9, [])
    assert st['state'] == TRACKING and not st['seen'] and st['age_s'] == pytest.approx(0.8)
    assert tr.update(1.2, [])['state'] == LOST
    st = tr.update(1.3, [card_at(0.85, 0.0)])            # 다시 보면 tracking
    assert st['state'] == TRACKING and st['seen']


def test_tracker_ignores_other_ids_but_reports_them():
    tr = FollowTracker(CAM, CFG)
    st = tr.update(0.0, [card_at(0.8, 0.0, cid=36), card_at(0.7, 0.1, cid=40)])
    assert st['state'] == SEARCHING and st['others'] == [36, 40] and 'x_m' not in st


def test_tracker_rejects_far_small_and_jumping_cards():
    tr = FollowTracker(CAM, FollowConfig(card_id=35, max_range_m=1.5, smooth=1.0))
    assert tr.update(0.0, [card_at(1.8, 0.0)])['state'] == SEARCHING          # 너무 멂
    assert tr.update(0.0, [card_at(0.8, 0.0, side_px=4)])['state'] == SEARCHING   # 너무 작음
    tr.update(0.0, [card_at(0.8, 0.0)])
    st = tr.update(0.1, [card_at(0.8, 0.6)])            # 0.1 s 에 0.6 m: 허용 0.15 + 0.15 = 0.30 넘음
    assert not st['seen'] and st['rejected'] == 1 and st['y_m'] == pytest.approx(0.0, abs=1e-3)
    st = tr.update(0.2, [card_at(0.8, 0.2)])            # 0.2 s 에 0.2 m: 허용 0.45 안
    assert st['seen'] and st['y_m'] == pytest.approx(0.2, abs=1e-3)


def test_tracker_picks_candidate_closest_to_previous():
    tr = FollowTracker(CAM, FollowConfig(card_id=35, smooth=1.0))
    tr.update(0.0, [card_at(0.8, 0.2)])
    st = tr.update(0.1, [card_at(0.6, -0.1), card_at(0.82, 0.21)])
    assert st['x_m'] == pytest.approx(0.82, abs=1e-3)


def test_config_validation():
    with pytest.raises(ValueError):
        FollowConfig(card_id=7).validate()
    with pytest.raises(ValueError):
        FollowConfig(lost_after_s=0.0).validate()
    with pytest.raises(ValueError):
        FollowConfig(smooth=0.0).validate()


def _frame(age_s=0.0, robot_time=None):
    now = time.time()
    return SimpleNamespace(recv_time=now - age_s, robot_time=math.nan if robot_time is None else robot_time)


def test_sink_publishes_status_every_frame_with_robot_stamp():
    sent = []
    sink = FollowSink(CAM, FollowConfig(card_id=35), lambda st, stamp: sent.append((st, stamp)),
                      require_robot_time=True)
    sink.handle(_frame(robot_time=100.0), [card_at(0.8, 0.0)])
    sink.handle(_frame(robot_time=100.1), [])
    assert [s[1] for s in sent] == [100.0, 100.1]
    assert sent[0][0]['seen'] and not sent[1][0]['seen']
    info = sink.info()
    assert info['sent'] == 2 and info['state'] == TRACKING and info['card_id'] == 35


def test_sink_skips_stale_or_unstamped_robot_frames():
    sent = []
    sink = FollowSink(CAM, FollowConfig(card_id=35), lambda st, stamp: sent.append(st), require_robot_time=True)
    assert sink.handle(_frame(age_s=0.8, robot_time=1.0), [card_at(0.8, 0.0)]) is None   # 밀린 영상
    assert sink.handle(_frame(), [card_at(0.8, 0.0)]) is None                              # 로봇 시각 없음
    assert sent == [] and sink.skipped == 2


def test_sink_without_robot_time_uses_recv_time_and_none_stamp():
    got = []
    sink = FollowSink(CAM, FollowConfig(card_id=35), lambda st, stamp: got.append(stamp))
    sink.handle(_frame(), [card_at(0.8, 0.0)])
    assert got == [None]


def test_publisher_target_only_when_seen():
    pytest.importorskip('rclpy')
    pytest.importorskip('geometry_msgs.msg')
    from roport_follow.ros_follow import RosFollowPublisher

    class Pub:
        def __init__(self):
            self.msgs = []

        def publish(self, m):
            self.msgs.append(m)

    class Clock:
        def now(self):
            return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=5, nanosec=0))

    pubs = {}

    class Node:
        def create_publisher(self, typ, topic, depth):
            pubs[topic] = Pub()
            return pubs[topic]

        def get_clock(self):
            return Clock()

    p = RosFollowPublisher(Node())
    p({'state': TRACKING, 'seen': True, 'x_m': 0.8, 'y_m': -0.1, 'card_id': 35}, 12.25)
    p({'state': LOST, 'seen': False, 'card_id': 35}, 13.0)
    tgt, st = pubs['/follow/target'].msgs, pubs['/follow/status'].msgs
    assert len(tgt) == 1 and tgt[0].header.frame_id == 'base_footprint' and tgt[0].point.x == pytest.approx(0.8)
    assert tgt[0].header.stamp.sec == 12 and tgt[0].header.stamp.nanosec == 250_000_000
    assert len(st) == 2 and '"state": "lost"' in st[1].data


def test_customer_doll_excluded_from_obstacles_while_tracking():
    """고객 카드를 단 인형(추종 대상 근처 25 cm 안)은 장애물 점으로 보내지 않음. 놓치면 다시 장애물."""
    from roport_follow.detector import Detection
    from roport_follow.obstacles import ObstacleConfig
    from roport_follow.ros_obstacles import ObstacleSink

    sent = []
    sink = ObstacleSink(CAM, ObstacleConfig(), lambda pts, st: sent.append(pts), require_robot_time=True)
    det = Detection(1, 'child', 0.9, (300.0, 200.0, 340.0, 260.0))
    fr = SimpleNamespace(recv_time=time.time(), robot_time=50.0)
    obs = sink.handle(fr, [det])
    assert len(obs) == 1
    near = (obs[0].x_m, obs[0].y_m + 0.05)              # 카드 바닥 점이 인형 앞끝에서 5 cm
    n_all = len(sent[-1])
    assert sink.handle(fr, [det], exclude=near) == [] and sink.excluded == 1 and len(sent[-1]) < n_all
    assert sink.info()['excluded_customer'] == 1
    far = (obs[0].x_m, obs[0].y_m + 0.6)                # 다른 곳의 고객: 이 인형은 장애물 그대로
    assert len(sink.handle(fr, [det], exclude=far)) == 1 and sink.excluded == 0


def test_follow_sink_exclude_point_only_when_tracking():
    sink = FollowSink(CAM, FollowConfig(card_id=35), lambda st, stamp: None, require_robot_time=True)
    assert sink.exclude_point() is None
    sink.handle(_frame(robot_time=1.0), [card_at(0.8, 0.1)])
    assert sink.exclude_point() == pytest.approx((0.8, 0.1), abs=1e-3)
    sink.handle(_frame(robot_time=1.4), [])                 # 가려짐(tracking, seen=false): 예전 좌표를 주지 않음
    assert sink.last['state'] == 'tracking' and sink.last['seen'] is False and sink.exclude_point() is None
    sink.handle(_frame(robot_time=1.6), [card_at(0.7, 0.0)])
    assert sink.exclude_point() == pytest.approx((0.75, 0.05), abs=1e-3)      # 다시 보임(위치는 직전 값과 평균)
    sink.handle(_frame(robot_time=5.5), [])                 # 놓침(기본 1 s = SR-22)
    assert sink.last['state'] == 'lost' and sink.exclude_point() is None
    assert FollowConfig().lost_after_s == 1.0


# ------------------------------------------------------------------ 인형으로 잇기(임시, 11-21)
def _fb_tracker(**kw):
    return FollowTracker(CAM, FollowConfig(card_id=35, doll_fallback=True, smooth=1.0, **kw))


def test_doll_fallback_is_off_by_default():
    tr = FollowTracker(CAM, FollowConfig(card_id=35, smooth=1.0))
    tr.update(0.0, [card_at(0.6, 0.0)], dolls=[(0.45, 0.0)])
    st = tr.update(0.2, [], dolls=[(0.50, 0.0)])
    assert st['seen'] is False and st['source'] is None


def test_doll_fallback_continues_with_customer_doll_when_card_hidden():
    """카드가 안 보이면(옆면·어린이 인형에 가림) 마지막 고객 자리 근처의 어른 인형으로 잇는다.
    카드가 보일 때 배운 위치 차(카드 − 인형)를 더해 자리가 튀지 않게 한다."""
    tr = _fb_tracker()
    for i in range(12):                                       # 카드와 인형이 같이 보임: 인형은 15 cm 가깝게 투영
        st = tr.update(0.2 * i, [card_at(0.60, 0.0)], dolls=[(0.45, 0.0)])
    assert st['source'] == 'card' and tr.offset[0] == pytest.approx(0.15, abs=0.01)
    st = tr.update(2.6, [], dolls=[(0.55, 0.05)])             # 카드 가려짐, 고객 인형은 10 cm 더 감
    assert st['seen'] is True and st['source'] == 'doll' and st['state'] == 'tracking'
    assert (st['x_m'], st['y_m']) == pytest.approx((0.70, 0.05), abs=0.02)
    st = tr.update(2.8, [card_at(0.72, 0.05)], dolls=[(0.57, 0.05)])   # 카드가 다시 보이면 카드 기준
    assert st['source'] == 'card' and st['fallback_s'] is None


def test_doll_fallback_ignores_far_or_ambiguous_dolls_and_never_starts_without_card():
    tr = _fb_tracker()
    assert tr.update(0.0, [], dolls=[(0.5, 0.0)])['state'] == 'searching'       # 카드 없이 시작하지 않음
    tr.update(1.0, [card_at(0.60, 0.0)], dolls=[])
    assert tr.update(1.2, [], dolls=[(1.10, 0.3)])['seen'] is False            # 멀리 있는 다른 어른 인형
    st = tr.update(1.4, [], dolls=[(0.58, 0.10), (0.58, -0.12)])               # 비슷하게 가까운 두 인형: 고르지 않음
    assert st['seen'] is False and st['source'] is None
    assert tr.update(1.6, [], dolls=[(0.58, 0.05), (1.0, -0.4)])['source'] == 'doll'   # 하나만 가까우면 이음


def test_doll_fallback_has_time_budget_then_normal_lost():
    tr = _fb_tracker()
    tr.update(0.0, [card_at(0.60, 0.0)], dolls=[(0.60, 0.0)])
    t = 0.0
    while t < 8.0:
        t = round(t + 0.2, 1)
        assert tr.update(t, [], dolls=[(0.60, 0.0)])['source'] == 'doll'
    st = tr.update(8.6, [], dolls=[(0.60, 0.0)])               # 인형만으로 8 s 넘음: 더 잇지 않음
    assert st['seen'] is False and st['state'] == 'tracking'
    assert tr.update(12.5, [], dolls=[(0.60, 0.0)])['state'] == 'lost'          # 그 뒤 4 s 못 보면 놓침
    assert tr.update(12.7, [], dolls=[(0.60, 0.0)])['seen'] is False            # 놓친 뒤에는 카드를 다시 봐야 함
    st = tr.update(13.0, [card_at(0.60, 0.0)], dolls=[(0.60, 0.0)])
    assert st['state'] == 'tracking' and st['source'] == 'card'


def test_doll_fallback_compensates_robot_rotation_with_odom_pose():
    """로봇이 도는 동안 고객은 화면에서 옆으로 크게 움직인다 → 마지막 자리를 odom 에 기억해 지금 로봇 기준으로 옮김."""
    tr = _fb_tracker()
    tr.update(0.0, [card_at(0.60, 0.0)], dolls=[(0.60, 0.0)], pose=(0.0, 0.0, 0.0))
    turned = (0.0, 0.0, math.radians(-35))                    # 로봇이 오른쪽으로 35° 돎 → 고객은 왼쪽 35° 에 보임
    doll = (0.60 * math.cos(math.radians(35)), 0.60 * math.sin(math.radians(35)))
    st = tr.update(0.4, [], dolls=[doll], pose=turned)
    assert st['source'] == 'doll' and st['bearing_deg'] == pytest.approx(35.0, abs=1.0)
    plain = _fb_tracker()                                      # 자세가 없으면 같은 인형이 34 cm 떨어져 보여 못 이음
    plain.update(0.0, [card_at(0.60, 0.0)], dolls=[(0.60, 0.0)])
    assert plain.update(0.05, [], dolls=[doll])['seen'] is False
