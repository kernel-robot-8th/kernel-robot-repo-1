"""인형 기억(doll_memory): 카메라 사각에 든 인형 유지, 오검출·치워진 인형은 잊음(ROS·로봇 없음)."""
import math
import time
from types import SimpleNamespace

import pytest

from roport_follow.doll_memory import DollMemory, MemoryConfig
from roport_follow.obstacles import DollObstacle, ObstacleConfig, clear_fov_deg, clear_points, clear_ring
from roport_follow.ros_obstacles import ObstacleSink
from test_obstacles import _box_at, _cam, _det


def doll(x, y, name='child'):
    """base_footprint (x 앞, y 왼쪽)에 선 인형(점 3개)."""
    return DollObstacle(name, 0.9, x, y, 0.04, False, ((x, y - 0.02, 0.05), (x, y, 0.05), (x, y + 0.02, 0.05)))


def centers(out):
    return sorted((round(o['x_m'], 2), round(o['y_m'], 2)) for o in out)


def test_confirmed_doll_stays_when_it_enters_the_blind_zone():
    """25회차(11-21): 30 cm 앞에서 본 어린이 인형에 14 cm 까지 다가가면 카메라에 안 보여도 계속 보내야 한다."""
    m = DollMemory()
    m.update(0.0, (0.0, 0.0, 0.0), [doll(0.30, 0.02)])
    m.update(0.2, (0.02, 0.0, 0.0), [doll(0.28, 0.02)])
    out = m.update(2.0, (0.16, 0.0, 0.0), [])                     # 로봇이 16 cm 전진: 인형은 축 14 cm 앞(사각)
    assert centers(out) == [(0.14, 0.02)] and out[0]['seen'] is False
    xs = [p[0] for p in out[0]['points']]
    assert min(xs) == pytest.approx(0.14, abs=1e-3)               # 점도 로봇 기준으로 다시 옮김
    for t in (5.0, 30.0, 60.0):                                   # 드라이버 제한(30 s 무이동)보다 오래 기억(child 60 s)
        assert centers(m.update(t, (0.16, 0.0, 0.0), [])) == [(0.14, 0.02)]
    assert m.info()['remembered'][0]['name'] == 'child'
    assert m.update(61.0, (0.16, 0.0, 0.0), []) == []            # 60 s 넘으면 잊음(손으로 옮긴 뒤 옛 기억 방지)


def test_clear_forgets_everything():
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.18, 0.0)])
    m.clear()
    assert m.update(0.4, (0, 0, 0), []) == [] and m.info()['count'] == 0


def test_doll_follows_robot_rotation_in_odom_frame():
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (1.0, 2.0, 0.0), [doll(0.30, 0.0)])           # odom (1.3, 2.0)
    out = m.update(1.0, (1.2, 2.0, math.pi / 2), [])              # 20 cm 전진 뒤 왼쪽으로 90° 돎
    assert centers(out) == [(0.0, -0.1)]                          # 인형은 로봇 오른쪽 10 cm


def test_single_frame_detection_is_sent_once_but_not_remembered():
    m = DollMemory()
    assert len(m.update(0.0, (0, 0, 0), [doll(0.5, 0.0)])) == 1   # 지금 보이는 것은 항상 보냄
    assert m.update(0.2, (0, 0, 0), []) == []                     # 한 번만 본 것은 기억 안 함
    assert m.update(1.5, (0, 0, 0), []) == [] and m.entries == []


def test_missed_once_then_seen_again_is_confirmed():
    m = DollMemory()
    m.update(0.0, (0, 0, 0), [doll(0.5, 0.0)])
    m.update(0.2, (0, 0, 0), [])
    m.update(0.4, (0, 0, 0), [doll(0.51, 0.01)])                  # 1 cm 흔들림: 같은 인형
    assert len(m.update(0.6, (0, 0, 0), [])) == 1                 # 이제 기억함


def test_visible_but_missing_doll_is_forgotten_after_three_frames():
    """치워진 인형·오검출: 카메라가 볼 수 있는 자리인데 3장 연속 안 보이면 잊는다."""
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.5, 0.05)])
    assert len(m.update(0.4, (0, 0, 0), [])) == 1
    assert len(m.update(0.6, (0, 0, 0), [])) == 1
    assert m.update(0.8, (0, 0, 0), []) == [] and m.entries == []


def test_out_of_view_adult_expires_but_child_stays():
    """adult(16 cm)는 라이다에도 보이고 대부분 고객 자신 → 시야 밖 3 s 뒤 잊음. child 는 유지."""
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.4, 0.0, 'adult'), doll(0.4, 0.15, 'child')])
    turned = (0.0, 0.0, math.pi / 2)                              # 90° 돌아 둘 다 시야 밖
    assert len(m.update(2.0, turned, [])) == 2
    out = m.update(3.5, turned, [])
    assert [o['name'] for o in out] == ['child']


def test_customer_area_and_far_dolls_are_dropped():
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.4, 0.0), doll(0.6, -0.3)])
    turned = (0.0, 0.0, math.pi)                                  # 뒤돌아 둘 다 시야 밖(기억 유지 대상)
    out = m.update(1.0, turned, [], exclude=(-0.4, 0.0), exclude_radius_m=0.15)   # 고객이 첫 인형 자리에
    assert centers(out) == [(-0.6, 0.3)]
    assert m.update(2.0, (3.0, 0.0, math.pi), []) == []           # 로봇이 keep_range 넘게 멀어짐


def test_missing_pose_keeps_memory_untouched_and_recovers_at_the_right_place():
    """odom 이 없는 영상: 마지막 자세를 다시 쓰지 않는다(18 cm 앞 인형으로 10 cm 다가가도 18 cm 로 남는 문제).
    기억은 그대로 두고 보이는 것만 보내며, odom 이 돌아오면 맞는 자리(8 cm)로 다시 보낸다."""
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0.5, 0.0, 0.0), [doll(0.18, 0.0)])
    assert m.update(1.0, None, []) == [] and m.info()['pose_ok'] is False     # 자세 모름: 기억한 것을 보내지 않음
    assert len(m.entries) == 1                                                 # 지우지도 않음
    out = m.update(1.2, (0.6, 0.0, 0.0), [])                                   # odom 복구(그동안 10 cm 전진)
    assert centers(out) == [(0.08, 0.0)] and m.info()['pose_ok'] is True
    fresh = DollMemory()
    assert len(fresh.update(0.0, None, [doll(0.3, 0.0)])) == 1    # 자세를 한 번도 못 받음: 보이는 것만
    assert fresh.update(0.2, None, []) == [] and fresh.entries == []


def test_sink_does_not_clear_around_the_robot_without_pose():
    """자세를 모르는 영상에서는 둘레(시야 밖) 지움 점을 보내지 않는다: 전에 맞게 찍은 칸을 지우지 않게."""
    cam, cfg, sent = _cam(), ObstacleConfig(), []
    poses = {'on': True}
    sink = ObstacleSink(cam, cfg, lambda pts, stamp: sent.append(len(pts)), require_robot_time=True,
                        memory=DollMemory(), pose_lookup=lambda t: (0.0, 0.0, 0.0) if poses['on'] else None)
    sink.handle(_frame(100.0), [])
    with_ring = sent[-1]
    poses['on'] = False
    sink.handle(_frame(100.2), [])
    assert sent[-1] == with_ring - len(sink._ring) and len(sink._ring) > 0
    poses['on'] = True
    sink.handle(_frame(100.4), [])
    assert sent[-1] == with_ring


def test_only_the_customer_detection_is_excluded_not_a_doll_standing_next_to_it():
    """리뷰 #130: 카드 옆 10 cm 에 따로 선 어린이 인형이 고객과 함께 장애물에서 빠졌음 → 고객으로 확인한 검출 하나만 뺀다."""
    cam, cfg, sent = _cam(), ObstacleConfig(), []
    sink = ObstacleSink(cam, cfg, lambda pts, stamp: sent.append(pts))
    customer = (0.60 + cfg.cam_x_m, 0.0)
    child = _det('child', _box_at(cam, 0.60, 0.10, width=0.04))        # 카드 옆 10 cm, 혼자 검출
    out = sink.handle(_frame(100.0), [child], exclude=customer)        # 카드는 보이는데 고객 인형 검출만 누락된 영상
    assert [o.name for o in out] == ['child'] and sink.excluded == 0 and len(sent[-1]) > 0    # 옆 인형은 그대로 장애물
    adult = _det('adult', _box_at(cam, 0.45, 0.02, width=0.08))        # 고객 인형(카드에서 약 15 cm)
    out = sink.handle(_frame(100.2), [adult, child], exclude=customer)
    assert [o.name for o in out] == ['child'] and sink.excluded == 1   # 고객만 빠지고 옆 인형은 장애물
    child2 = _det('child', _box_at(cam, 0.55, -0.08, width=0.04))
    out = sink.handle(_frame(100.4), [adult, child, child2], exclude=customer)
    assert sorted(o.name for o in out) == ['child', 'child'] and sink.excluded == 1
    out = sink.handle(_frame(100.6), [adult, child], exclude=None)     # 카드가 가려진 영상: 아무것도 빼지 않음
    assert sorted(o.name for o in out) == ['adult', 'child'] and sink.excluded == 0


def test_memory_removes_only_the_customer_doll_not_a_remembered_neighbour():
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.45, 0.02, 'adult'), doll(0.50, 0.10, 'child')])
    back = (0.0, 0.0, math.pi)                                          # 뒤돌아 둘 다 시야 밖(기억만 남음)
    out = m.update(1.0, back, [], exclude=(-0.45, -0.02), exclude_radius_m=m.cfg.match_m, exclude_name='adult')
    assert [o['name'] for o in out] == ['child']


def test_time_going_backwards_drops_memory():
    m = DollMemory()
    for t in (10.0, 10.2):
        m.update(t, (0, 0, 0), [doll(0.18, 0.0)])
    assert m.update(5.0, (0, 0, 0), []) == []


def test_config_validation():
    with pytest.raises(ValueError):
        MemoryConfig(match_m=0.0).validate()
    with pytest.raises(ValueError):
        MemoryConfig(vis_near_m=1.2, vis_far_m=1.0).validate()


def test_clear_ring_covers_everything_outside_the_camera_sector():
    cam, cfg = _cam(), ObstacleConfig()
    ring = clear_ring(cam, cfg)
    left, right = clear_fov_deg(cam, cfg)
    assert all(math.hypot(x, y) == pytest.approx(cfg.clear_range_m, abs=1e-3) for x, y, _ in ring)
    assert cfg.clear_range_m > cfg.max_range_m                    # 표시는 안 되고 지우기만(obstacle_max_range 밖)
    angs = sorted(math.degrees(math.atan2(y, x)) % 360 for x, y, _ in ring)
    assert angs[0] == pytest.approx(left, abs=0.5) and angs[-1] == pytest.approx(360 - right, abs=0.5)
    gaps = [b - a for a, b in zip(angs, angs[1:])]
    assert max(gaps) * math.pi / 180 * cfg.clear_range_m <= cfg.ring_spacing_m * 1.01
    # 끝 간격 2 cm → 축 0.62 m 안에서는 광선 사이가 1 cm 이하(지역 비용 지도 1 cm 칸이 빠짐없이 지워짐)
    assert max(gaps) * math.pi / 180 * 0.62 <= 0.0101
    assert 250 < len(ring) < 500


def _frame(robot_time, seq=1):
    return SimpleNamespace(robot_time=robot_time, recv_time=time.time(), seq=seq, image=None)


def test_sink_with_memory_resends_hidden_doll_and_ring():
    cam, cfg, sent = _cam(), ObstacleConfig(), []
    poses = {100.0: (0.0, 0.0, 0.0), 100.2: (0.0, 0.0, 0.0), 101.0: (0.15, 0.0, 0.0)}
    sink = ObstacleSink(cam, cfg, lambda pts, stamp: sent.append(pts), pose_lookup=poses.get,
                        memory=DollMemory())
    box = _box_at(cam, 0.26, 0.0)                                 # 렌즈 앞 26 cm = 축 약 29.5 cm
    sink.handle(_frame(100.0), [_det('child', box)])
    sink.handle(_frame(100.2), [_det('child', box)])
    sink.handle(_frame(101.0), [])                                # 15 cm 전진: 인형은 축 약 14.5 cm(사각), 검출 없음
    base = len(clear_points(cam, cfg)) + len(clear_ring(cam, cfg))
    doll_pts = [p for p in sent[-1] if math.hypot(p[0], p[1]) < 0.5]
    assert len(sent[-1]) > base and doll_pts
    assert min(p[0] for p in doll_pts) == pytest.approx(0.145, abs=0.02)
    info = sink.info()
    assert info['memory']['remembered'][0]['name'] == 'child' and info['dolls'] == []
    plain = ObstacleSink(cam, cfg, lambda pts, stamp: sent.append(pts))   # 기억 끔: 둘레 점 없음(예전 방식)
    plain.handle(_frame(102.0), [])
    assert len(sent[-1]) == len(clear_points(cam, cfg)) and plain.info()['memory'] is None


def test_session_forgets_dolls_while_robot_is_in_line_mode():
    """11-21 실로봇: 다시 시험하려고 로봇을 손으로 N08 에 옮기면 odom 은 그대로라 옛 기억이 로봇 옆에 남음 → 라인 모드면 비움."""
    from roport_follow.session import ViewSession
    m = DollMemory()
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.18, 0.0)])
    ses = SimpleNamespace(obstacles=SimpleNamespace(memory=m),
                          demo=SimpleNamespace(info=lambda: {'robot': {'observed': {'mode': 'nav2'}}}))
    ViewSession._forget_dolls_in_line_mode(ses)
    assert len(m.entries) == 1                                    # Nav2 모드: 그대로
    ses.demo = SimpleNamespace(info=lambda: {'robot': {'observed': {'mode': 'line'}}})
    ViewSession._forget_dolls_in_line_mode(ses)
    assert m.entries == []
    ses.demo = None                                               # 시험용 연결 없이 돌릴 때는 건드리지 않음
    ViewSession._forget_dolls_in_line_mode(ses)


def test_customer_exclusion_is_wider_for_adults_than_children():
    """11-21 실로봇: 고객 인형(adult)의 YOLO 위치가 카드와 12~20 cm 달라 15 cm 로는 고객 자신이 장애물로 찍힘.
    고객과 같은 종류(adult)는 30 cm, 고객 앞 어린이 인형은 15 cm 그대로(계속 장애물)."""
    cam, cfg, sent = _cam(), ObstacleConfig(), []
    sink = ObstacleSink(cam, cfg, lambda pts, stamp: sent.append(pts))
    assert sink.exclude_radius('adult') == 0.30 and sink.exclude_radius('child') == 0.15
    customer = (0.60 + cfg.cam_x_m, 0.0)                              # 카드로 잰 고객 위치(base_footprint)
    adult = _det('adult', _box_at(cam, 0.42, 0.05, width=0.08))       # 고객 인형의 YOLO 상자: 18 cm 가깝게 투영
    child = _det('child', _box_at(cam, 0.42, -0.05, width=0.04))      # 같은 거리의 어린이 인형
    out = sink.handle(_frame(100.0), [adult, child], exclude=customer)
    assert [o.name for o in out] == ['child'] and sink.excluded == 1
    far_adult = _det('adult', _box_at(cam, 0.25, 0.0, width=0.08))    # 고객에서 35 cm: 다른 어른 인형 → 장애물
    assert [o.name for o in sink.handle(_frame(100.2), [far_adult], exclude=customer)] == ['adult']
    m = DollMemory()                                                   # 기억도 같은 규칙으로 지움
    for t in (0.0, 0.2):
        m.update(t, (0, 0, 0), [doll(0.42, 0.05, 'adult'), doll(0.42, -0.05, 'child')])
    back = (0.0, 0.0, math.pi)                                         # 뒤돌아 둘 다 시야 밖
    out = m.update(1.0, back, [], exclude=(-0.635, 0.0), exclude_radius_m=sink.exclude_radius)
    assert [o['name'] for o in out] == ['child']
