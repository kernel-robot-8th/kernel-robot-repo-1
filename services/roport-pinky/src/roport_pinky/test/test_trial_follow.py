"""고객 카드 추종 판정부(trial_follow)와 TrialDriver 연결 — ROS·Nav2 없이 가짜로 확인."""
import json
import math
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest
from roport_pinky import trial_follow as tf
from roport_pinky.trial_driver import TrialDriver
from test_trial_driver import PROFILE

BT = Path(__file__).resolve().parents[1] / 'behavior_trees' / 'follow_target.xml'


def status(state='tracking', card=35, stamp=100.0, seen=True):
    return tf.parse_status(json.dumps({'state': state, 'card_id': card, 'seen': seen,
                                       'age_s': 0.0, 'stamp': stamp, 'frame_id': 'base_footprint',
                                       'others': [], 'rejected': 0}))


def frame(inputs, stamp, x=1.0, y=0.0, card=35, state='tracking', received=50.0, order='target'):
    """노트북 한 영상: target 과 같은 stamp 의 status. 도착 순서를 바꿔도 짝지어야 한다."""
    if order == 'target':
        inputs.on_target(stamp, 'base_footprint', x, y)
        inputs.on_status(status(state, card, stamp), received)
    else:
        inputs.on_status(status(state, card, stamp), received)
        inputs.on_target(stamp, 'base_footprint', x, y)


def shift(dx=0.0, dy=0.0):
    """TF 가짜: base_footprint 가 map 에서 (dx, dy), yaw 0."""
    return lambda stamp, x, y: (x + dx, y + dy)


def no_tf(stamp, x, y):
    raise LookupError('extrapolation')


POSE = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
# lost_after_s 4 = 시연값(아래 시간 시험들의 기준). 기본 1 s(SR-22)는 따로 검사한다
PAYLOAD = {'card_id': 35, 'standoff_m': 0.5, 'lost_after_s': 4.0, **PROFILE}


# ------------------------------------------------------------------ payload·status 검사
@pytest.mark.parametrize('change, code', [
    ({'card_id': 34}, 'invalid_card_id'), ({'card_id': 50}, 'invalid_card_id'),
    ({'card_id': True}, 'invalid_card_id'), ({'card_id': 35.0}, 'invalid_card_id'),
    ({'standoff_m': 0.15}, 'invalid_standoff'), ({'standoff_m': float('nan')}, 'invalid_standoff'),
    ({'standoff_m': 1.3}, 'invalid_standoff'), ({'max_duration_s': 0.0}, 'invalid_max_duration'),
    ({'max_duration_s': float('inf')}, 'invalid_max_duration')])
def test_payload_validation(change, code):
    with pytest.raises(ValueError, match=code):
        tf.parse_payload({**PAYLOAD, **change})


def test_payload_defaults():
    assert tf.parse_payload({'card_id': 49}) == (49, 0.5, None)
    assert tf.parse_payload({'card_id': 35, 'standoff_m': 1, 'max_duration_s': 60}) == (35, 1.0, 60.0)


@pytest.mark.parametrize('text', ['nope', '[]', json.dumps({'state': 'tracking'}),
    json.dumps({'state': 'moving', 'card_id': 35, 'seen': True, 'stamp': 1.0}),
    json.dumps({'state': 'tracking', 'card_id': 35, 'seen': 1, 'stamp': 1.0}),
    json.dumps({'state': 'tracking', 'card_id': 35, 'seen': True, 'stamp': 1.0, 'frame_id': 'map'})])
def test_malformed_status_is_ignored(text):
    assert tf.parse_status(text) is None


# ------------------------------------------------------------------ target ↔ status 짝짓기
@pytest.mark.parametrize('order', ['target', 'status'])
def test_target_is_accepted_only_with_matching_tracking_status(order):
    inputs = tf.FollowInput()
    frame(inputs, 100.0, order=order)
    assert inputs.target[35] == (100.0, 1.0, 0.0)


@pytest.mark.parametrize('kw', [{'card': 36}, {'state': 'lost'}])
def test_mismatched_card_or_state_target_is_ignored(kw):
    inputs = tf.FollowInput()
    frame(inputs, 100.0, **kw)
    assert 35 not in inputs.target


def test_unseen_status_does_not_accept_target_and_bad_frames_are_dropped():
    inputs = tf.FollowInput()
    inputs.on_target(100.0, 'base_footprint', 1.0, 0.0)
    inputs.on_status(status(stamp=100.0, seen=False), 50.0)
    assert inputs.target == {}
    inputs.on_target(101.0, 'camera', 1.0, 0.0)
    inputs.on_target(102.0, 'base_footprint', float('nan'), 0.0)
    inputs.on_target(103.0, 'base_footprint', 5.0, 0.0)
    assert len(inputs.pending) == 1


def test_older_target_never_replaces_newer():
    inputs = tf.FollowInput()
    frame(inputs, 101.0, x=1.2)
    frame(inputs, 100.0, x=0.8)
    assert inputs.target[35][0] == 101.0


# ------------------------------------------------------------------ 목표·standoff
def test_approach_goal_keeps_standoff_on_robot_target_line():
    x, y, yaw = tf.approach_goal({'x': 1.0, 'y': 1.0, 'yaw': math.pi / 2}, (1.0, 3.0), 0.5)
    assert (x, y, yaw) == pytest.approx((1.0, 2.5, math.pi / 2))


def test_off_center_customer_turns_in_place_first():
    """11-20: 고객이 정면에서 15° 넘게 벗어나면 앞으로 가지 않고 제자리에서 돈다(시야 ±24° 밖으로 안 나가게)."""
    pose = {'x': 1.0, 'y': 1.0, 'yaw': 0.0}
    assert tf.approach_goal(pose, (1.0, 3.0), 0.5) == pytest.approx((1.0, 1.0, math.pi / 2))      # 90°
    off25 = (1.0 + 2 * math.cos(math.radians(25)), 1.0 + 2 * math.sin(math.radians(25)))
    assert tf.approach_goal(pose, off25, 0.5)[:2] == pytest.approx((1.0, 1.0))                     # 25° → 회전만
    off10 = (1.0 + 2 * math.cos(math.radians(10)), 1.0 + 2 * math.sin(math.radians(10)))
    x, y, yaw = tf.approach_goal(pose, off10, 0.5)                                                 # 10° → 다가감
    assert math.hypot(x - 1.0, y - 1.0) == pytest.approx(1.5) and yaw == pytest.approx(math.radians(10))
    assert tf.TURN_FIRST_RAD == pytest.approx(math.radians(20)) and tf.TURN_DONE_RAD == pytest.approx(math.radians(6))


def test_turn_hysteresis_stops_left_right_wobble():
    """11-20 모서리: 15° 근처를 오가며 돌기·다가가기를 번갈아 해 좌우로 흔들림 → 20° 에서 돌기 시작, 6° 안에서 끝."""
    pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    at = lambda deg: (2 * math.cos(math.radians(deg)), 2 * math.sin(math.radians(deg)))
    goal, turning = tf.approach(pose, at(12), 0.5, turning=False)
    assert not turning and math.hypot(*goal[:2]) > 1.0                 # 12°: 다가감
    goal, turning = tf.approach(pose, at(22), 0.5, turning=False)
    assert turning and goal[:2] == pytest.approx((0.0, 0.0))           # 22°: 돌기 시작
    goal, turning = tf.approach(pose, at(12), 0.5, turning=True)
    assert turning and goal[:2] == pytest.approx((0.0, 0.0))           # 아직 12°: 계속 돌기(오락가락 안 함)
    goal, turning = tf.approach(pose, at(4), 0.5, turning=True)
    assert not turning and math.hypot(*goal[:2]) > 1.0                 # 6° 안: 다시 다가감


def test_inside_standoff_holds_position_facing_customer():
    assert tf.approach_goal(POSE, (0.0, 0.3), 0.5) == pytest.approx((0.0, 0.0, math.pi / 2))   # 90° 벗어남: 돎
    assert tf.approach_goal({**POSE, 'yaw': 1.0}, (0.01, 0.0), 0.5) == pytest.approx((0, 0, 1.0))


def test_target_is_transformed_with_tf_at_its_stamp_and_rate_limited():
    inputs, seen = tf.FollowInput(), []
    def transform(stamp, x, y):
        seen.append(stamp)
        return x + 2.0, y
    s = tf.FollowSession(PAYLOAD, started=50.0)
    frame(inputs, 100.0, x=1.5)
    kind, goal = s.step(inputs, 100.1, 50.0, transform, {'x': 2.0, 'y': 0.0, 'yaw': 0.0})
    assert kind == 'goal' and goal == pytest.approx((3.0, 0.0, 0.0, 100.0)) and seen == [100.0]
    frame(inputs, 100.1, x=1.6, received=50.1)
    assert s.step(inputs, 100.2, 50.1, transform, POSE) is None       # 0.25 s 안: 보내지 않음
    kind, goal = s.step(inputs, 100.3, 50.3, transform, {'x': 2.0, 'y': 0.0, 'yaw': 0.0})
    assert kind == 'goal' and goal[3] == 100.1
    assert s.step(inputs, 100.6, 50.6, transform, POSE) is None       # 새 target 없으면 다시 안 보냄


# ------------------------------------------------------------------ 정지 조건
def started(**payload):
    inputs = tf.FollowInput()
    frame(inputs, 100.0)
    s = tf.FollowSession({**PAYLOAD, **payload}, started=50.0)
    assert s.step(inputs, 100.1, 50.0, shift(), POSE)[0] == 'goal'
    return inputs, s


def test_lost_status_stops_immediately():
    inputs, s = started()
    inputs.on_status(status('lost', stamp=100.2), 50.2)
    assert s.step(inputs, 100.3, 50.3, shift(), POSE) == ('stop', 'follow_lost')


def test_searching_after_laptop_reset_is_lost():
    inputs, s = started()
    inputs.on_status(status('searching', stamp=100.2, seen=False), 50.2)
    assert s.step(inputs, 100.3, 50.3, shift(), POSE) == ('stop', 'follow_lost')


def test_status_silence_over_one_second_is_status_stale():
    """11-21: 노트북 부하로 1 s 끊김에 추종이 끝나 STATUS_MAX_AGE 1 → 2 s."""
    inputs, s = started()
    assert s.step(inputs, 101.0, 52.0, shift(), POSE) is None
    assert s.step(inputs, 101.1, 52.01, shift(), POSE) == ('stop', 'follow_status_stale')


def test_laptop_silence_is_status_stale_even_when_target_ages_first():
    inputs, s = started()                    # target stamp 100.0 = 카메라 시각(수신보다 이름)
    assert s.step(inputs, 104.55, 50.95, shift(), POSE) == ('stop', 'follow_status_stale')


def test_unseen_card_when_target_ages_is_lost_not_target_stale():
    inputs, s = started()                    # 카드가 가려짐: status 는 오지만 seen=false, 아직 tracking
    inputs.on_status(status('tracking', stamp=103.9, seen=False), 51.0)
    assert s.step(inputs, 104.55, 51.05, shift(), POSE) == ('stop', 'follow_lost')


def test_status_of_another_card_does_not_keep_follow_alive():
    inputs, s = started()
    inputs.on_status(status(card=36, stamp=100.9), 50.9)
    assert s.step(inputs, 102.2, 52.2, shift(), POSE) == ('stop', 'follow_status_stale')


def test_tracking_status_without_targets_is_target_stale():
    inputs, s = started()
    inputs.on_status(status(stamp=104.7), 51.7)          # tracking 이라도 target 이 안 옴(lost_after_s 4 + 지연 0.5)
    assert s.step(inputs, 104.7, 51.7, shift(), POSE) == ('stop', 'follow_target_stale')


def test_tf_failure_for_fresh_targets_is_reported_separately():
    inputs, s = started()
    frame(inputs, 101.8, received=50.8)
    assert s.step(inputs, 101.9, 50.9, no_tf, POSE) is None
    assert s.step(inputs, 104.55, 51.15, no_tf, POSE) == ('stop', 'follow_tf_unavailable')


def test_future_stamped_target_is_not_used():
    inputs, s = started()
    frame(inputs, 115.0, received=50.5)                 # 노트북 시계를 stamp 로 쓴 경우
    assert s.step(inputs, 100.6, 50.6, shift(), POSE) is None
    assert s.step(inputs, 104.55, 50.7, shift(), POSE) == ('stop', 'follow_target_stale')


def test_max_duration_stops():
    inputs, s = started(max_duration_s=2.0)
    frame(inputs, 101.9, received=51.9)
    assert s.step(inputs, 102.0, 52.0, shift(), POSE) == ('stop', 'follow_max_duration')


def test_soft_stops_are_only_normal_follow_endings():
    assert tf.SOFT_STOPS == {'follow_lost', 'follow_status_stale', 'follow_target_stale',
                             'follow_max_duration', 'follow_not_tracking', 'follow_card_mismatch'}
    assert not {'follow_tf_unavailable', 'follow_nav2_failed', 'nav2_not_ready'} & tf.SOFT_STOPS


# ------------------------------------------------------------------ 출발 검사
@pytest.mark.parametrize('prepare, code', [
    (lambda i: None, 'follow_status_stale'),
    (lambda i: frame(i, 100.0, card=36), 'follow_card_mismatch'),
    (lambda i: frame(i, 100.0, state='lost'), 'follow_not_tracking'),
    (lambda i: i.on_status(status(stamp=100.0), 50.0), 'follow_target_stale'),
])
def test_admission_rejections(prepare, code):
    inputs = tf.FollowInput()
    prepare(inputs)
    with pytest.raises(ValueError, match=code):
        tf.admit(PAYLOAD, inputs, 100.1, 50.1, shift(), POSE)


def test_admission_requires_tf_and_pose():
    inputs = tf.FollowInput()
    frame(inputs, 100.0)
    with pytest.raises(ValueError, match='follow_tf_unavailable'):
        tf.admit(PAYLOAD, inputs, 100.1, 50.1, no_tf, POSE)
    with pytest.raises(ValueError, match='nav2_not_ready'):
        tf.admit(PAYLOAD, inputs, 100.1, 50.1, shift(), None)
    assert tf.admit(PAYLOAD, inputs, 100.1, 50.1, shift(), POSE) == pytest.approx((0.5, 0, 0, 100.0))


# ------------------------------------------------------------------ BT 파일
def test_follow_tree_uses_goal_updater_without_recovery_motion():
    tree = ET.parse(BT)
    updater = tree.find('.//GoalUpdater')
    assert updater.attrib == {'input_goal': '{goal}', 'output_goal': '{updated_goal}'}
    assert updater.find('ComputePathToPose').attrib['goal'] == '{updated_goal}'
    assert tree.find('.//KeepRunningUntilFailure/FollowPath') is not None
    for name in ('Spin', 'BackUp', 'DriveOnHeading'):            # 움직이는 복구는 넣지 않음(대기·비우기만)
        assert tree.find('.//' + name) is None


def test_follow_tree_ports_match_installed_nav2_jazzy_nodes():
    from ament_index_python.packages import get_package_share_directory
    models = ET.parse(Path(get_package_share_directory('nav2_behavior_tree')) / 'nav2_tree_nodes.xml')
    ports = {m.attrib['ID']: {p.attrib['name'] for p in m if 'name' in p.attrib}
             for m in models.findall('.//TreeNodesModel/*')}
    builtin = {'root', 'BehaviorTree', 'KeepRunningUntilFailure', 'Sequence'}   # BehaviorTree.CPP v4
    for node in ET.parse(BT).iter():
        assert node.tag in ports or node.tag in builtin, node.tag
        if node.tag in ports:
            supplied = set(node.attrib) - {'name'}
            assert supplied <= ports[node.tag], (node.tag, supplied - ports[node.tag])


# ------------------------------------------------------------------ TrialDriver 연결(가짜 Nav2)
class FakeNav:
    def __init__(self):
        self.started, self.cancels, self.result, self.navigation = [], 0, None, None
        self.lock = threading.RLock()
        self.pose = dict(POSE)

    def start(self, payload, **kw):
        self.started.append((payload, kw))

    def cancel(self):
        self.cancels += 1
        self.result = None

    def quiescent(self):
        return True

    def snapshot(self):
        return {'nav2_ready': True, 'localization_ready': True, 'pose': self.pose, 'stopped': True}


class Clocks:
    ros, mono = 100.1, 50.1


def driver(monkeypatch, tmp_path):
    bt = tmp_path / 'follow_target.xml'
    bt.write_text('<root/>')
    d = object.__new__(TrialDriver)
    d.lock, d.nav = threading.RLock(), FakeNav()
    d.mode, d.nav_command, d.halted, d.failed_navigation = 'nav2', None, None, None
    d.follow_input, d.follow, d.follow_results, d.follow_bt = tf.FollowInput(), None, [], str(bt)
    d.leases, d.goals = [], []
    d._lease = d.leases.append
    d._publish_goal_update = d.goals.append
    d._follow_transform = shift()
    d._ros_now = lambda: Clocks.ros
    d.logic = SimpleNamespace(active=None, node='N12', safe_stop=lambda r: None,
                              drain_results=lambda: [])
    d.observe = lambda: {**PROFILE, 'fault': d.halted, 'stopped': True, 'line_enabled': False,
                         'nav2_ready': True, 'localization_ready': True, 'mode': 'nav2'}
    monkeypatch.setattr('roport_pinky.trial_driver.time.monotonic', lambda: Clocks.mono)
    Clocks.ros, Clocks.mono = 100.1, 50.1
    frame(d.follow_input, 100.0, x=1.5)
    return d


def follow_cmd(**payload):
    return {'type': 'follow.requested', 'command_id': 'f1', 'execution_id': 'f1',
            'payload': {**PAYLOAD, **payload}}


def test_driver_starts_nav2_follow_tree_and_holds_nav2_lease(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.validate_action(follow_cmd())
    assert d.nav.started == [] and d.leases == []            # 검사는 읽기 전용
    d.dispatch(follow_cmd())
    (payload, kw), = d.nav.started
    # 추종은 누적 복구 상한 없음(Nav2 가 비우기·대기를 따로 세서 복구 1번 = 2, 11-20 실로봇)
    assert kw == {'behavior_tree': d.follow_bt, 'stamp': 100.0, 'max_recoveries': None}
    assert payload['pose'] == pytest.approx({'x': 1.0, 'y': 0.0, 'yaw': 0.0})
    assert d.leases == ['nav2'] and d.logic.node is None
    # heartbeat 가 계속 nav2 로 갱신하는 조건(trial_guard 는 그대로)
    assert d.nav_command['type'] in ('navigate.requested', 'follow.requested')
    Clocks.ros, Clocks.mono = 100.4, 50.4
    frame(d.follow_input, 100.3, x=1.7, received=50.4)
    d._follow_tick()
    assert d.goals == [pytest.approx((1.2, 0.0, 0.0, 100.3))]
    assert d.drain_results() == []


@pytest.mark.parametrize('mode, code', [('line', 'not_in_nav2_mode')])
def test_driver_rejects_follow_outside_nav2(monkeypatch, tmp_path, mode, code):
    d = driver(monkeypatch, tmp_path)
    d.mode = mode
    with pytest.raises(ValueError, match=code):
        d.validate_action(follow_cmd())


def test_driver_rejects_follow_with_wrong_map_or_missing_tree(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    with pytest.raises(ValueError, match='map_mismatch'):
        d.validate_action(follow_cmd(map_version='old'))
    d.follow_bt = str(tmp_path / 'missing.xml')
    with pytest.raises(ValueError, match='follow_bt_missing'):
        d.validate_action(follow_cmd())


def test_lost_customer_cancels_reports_and_allows_new_follow(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.dispatch(follow_cmd())
    d.follow_input.on_status(status('lost', stamp=100.2), 50.2)
    Clocks.ros, Clocks.mono = 100.3, 50.3
    d._follow_tick()
    assert d.nav.cancels == 1 and d.nav_command is None and d.follow is None
    assert d.drain_results() == [(follow_cmd(), 'failed', {'error_code': 'follow_lost'})]
    assert d.halted is None
    d.safe_stop('follow_lost')                 # 관제 모듈이 failed 보고 뒤 다시 부른다
    assert d.halted is None
    frame(d.follow_input, 100.4, received=50.4)
    Clocks.ros, Clocks.mono = 100.5, 50.5
    d.validate_action(follow_cmd())            # 새 명령으로만 다시 시작


def test_nav2_failure_during_follow_latches(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.dispatch(follow_cmd())
    d.nav.result = ('nav2_goal_failed', 0.0)
    d._follow_tick()
    assert d.drain_results() == [(follow_cmd(), 'failed', {'error_code': 'follow_nav2_failed'})]
    assert d.halted == 'follow_nav2_failed'
    with pytest.raises(ValueError, match='follow_nav2_failed'):
        d.validate_action(follow_cmd())


def test_tf_failure_during_follow_latches(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.dispatch(follow_cmd())
    d._follow_transform = no_tf
    frame(d.follow_input, 100.9, received=50.9)
    Clocks.ros, Clocks.mono = 101.15, 51.0
    d._follow_tick()
    assert d.drain_results()[0][2] == {'error_code': 'follow_tf_unavailable'}
    assert d.halted == 'follow_tf_unavailable'


def test_operator_stop_ends_follow_without_driver_latch(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.dispatch(follow_cmd())
    d.safe_stop('operator_stop')
    assert d.nav.cancels == 1 and d.nav_command is None and d.halted is None
    d.safe_stop('operator_stop')               # 추종이 없으면 기존처럼 잠근다
    assert d.halted == 'operator_stop'


def test_fault_during_follow_still_latches(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.dispatch(follow_cmd())
    d.safe_stop('control_link_lost')
    assert d.halted == 'control_link_lost' and d.nav.cancels == 1


def test_inputs_lost_between_accept_and_dispatch_fail_without_motion(monkeypatch, tmp_path):
    d = driver(monkeypatch, tmp_path)
    d.validate_action(follow_cmd())
    Clocks.ros, Clocks.mono = 102.5, 52.5      # 저장 ACK 가 늦어 status 가 오래됨(STATUS_MAX_AGE 2 s)
    d.dispatch(follow_cmd())
    assert d.nav.started == [] and d.leases == [] and d.nav_command is None
    assert d.drain_results() == [(follow_cmd(), 'failed', {'error_code': 'follow_status_stale'})]
    assert d.halted is None


def test_target_age_allows_pipeline_latency():
    """11-20 실로봇: 노트북에서 0.65 s 끊김 + 처리·전송 0.4 s → stamp 나이 약 1.05 s 는 놓침이 아님."""
    inputs, s = started()
    inputs.on_status(status('tracking', stamp=100.4, seen=False), 50.95)
    assert s.step(inputs, 101.05, 50.95, shift(), POSE) is None
    assert s.step(inputs, 102.05, 50.99, shift(), POSE) is None      # 2 s 끊김 + 지연도 아직 놓침 아님
    assert s.step(inputs, 104.35, 50.99, shift(), POSE) is None      # 피해 가는 동안 4 s 까지 버팀(8 s 시험은 되돌림)
    assert s.target_max_age == 4.5 and tf.STATUS_MAX_AGE == 2.0


def test_follow_tree_recovers_before_failing():
    """막히면 1 s 대기 뒤 다시. 횟수는 추종 내내 누적되므로 넉넉히(100), 멈춤 기준은 드라이버의 30 s 무이동."""
    root = ET.parse(BT).getroot()
    recov = list(root.iter('RecoveryNode'))
    assert len(recov) == 1 and recov[0].attrib == {'number_of_retries': '100', 'name': 'FollowRecovery'}
    assert [c.tag for c in recov[0]] == ['PipelineSequence', 'Wait']
    assert root.find('.//RateController').attrib['hz'] == '4.0'
    assert not list(root.iter('Spin')) and not list(root.iter('BackUp'))


def test_follow_tree_never_clears_costmaps():
    """11-21 실로봇: 복구의 비용 지도 비우기가 로봇 14 cm 앞 어린이 인형(카메라로만 보이고 15 cm 안은 다시 안 찍힘)을
    지워 로봇이 인형을 침 → 추종 BT 는 비용 지도를 비우지 않는다."""
    root = ET.parse(BT).getroot()
    for tag in ('ClearEntireCostmap', 'ClearCostmapExceptRegion', 'ClearCostmapAroundRobot'):
        assert not list(root.iter(tag)), tag


def test_follow_tree_has_single_wait():
    """11-20 실로봇: 두 복구가 동시에 「대기」를 요청하면 behavior_server 가 먼저 것을 취소 → Wait 는 한 곳에만."""
    root = ET.parse(BT).getroot()
    waits = list(root.iter('Wait'))
    assert len(waits) == 1 and waits[0].attrib['wait_duration'] == '1.0'
    assert waits[0] in list(root.find('BehaviorTree/RecoveryNode'))


def test_close_customer_small_angle_jitter_does_not_rotate():
    """11-20: 고객이 가까이 서 있을 때 방향 측정 ±3~5° 흔들림에 좌우로 돌지 않음(10° 안이면 지금 방향 유지)."""
    pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    for deg in (-5, 3, 8):
        goal, turning = tf.approach(pose, (0.33 * math.cos(math.radians(deg)), 0.33 * math.sin(math.radians(deg))),
                                    0.3, turning=False)
        assert goal == pytest.approx((0.0, 0.0, 0.0)) and not turning
    goal, _ = tf.approach(pose, (0.33 * math.cos(math.radians(14)), 0.33 * math.sin(math.radians(14))), 0.3, False)
    assert goal[2] == pytest.approx(math.radians(14))                   # 10° 넘으면 고객 쪽으로 돎


def test_same_goal_is_not_resent_and_mode_dwell_holds():
    """11-20: 목표가 0.3~1 s 마다 「앞으로」↔「제자리」로 바뀌어 Nav2 가 매번 다시 시작 → 같은 목표는 다시 안 보내고,
    동작을 바꾼 뒤 0.8 s 는 유지한다."""
    inputs = tf.FollowInput()
    s = tf.FollowSession(PAYLOAD, started=50.0)
    pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    frame(inputs, 100.0, x=1.0, y=0.0)                       # 정면 1 m: 다가가기
    kind, g1 = s.step(inputs, 100.1, 50.0, shift(), pose)
    assert kind == 'goal' and not s.turning
    frame(inputs, 100.3, x=1.02, y=0.0, received=50.3)       # 2 cm 움직임: 같은 목표 → 안 보냄
    assert s.step(inputs, 100.4, 50.3, shift(), pose) is None
    frame(inputs, 100.6, x=0.6, y=0.5, received=50.6)        # 40° 로 벌어짐(0.6 s 뒤): 전환은 0.8 s 기다림
    kind, g = s.step(inputs, 100.7, 50.6, shift(), pose)
    assert kind == 'goal' and not s.turning and math.hypot(g[0], g[1]) > 0.05
    frame(inputs, 101.0, x=0.6, y=0.5, received=51.0)
    kind, g = s.step(inputs, 101.1, 51.0, shift(), pose)     # 첫 동작에서 0.8 s 넘음: 제자리 회전
    assert kind == 'goal' and s.turning and g[:2] == pytest.approx((0.0, 0.0))


def test_hold_margin_is_five_cm():
    """11-20: 간격을 더 가깝게 → 간격 + 5 cm 를 넘으면 다가감(0.10 이면 고객이 40 cm 안에서 로봇이 안 움직임)."""
    pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    assert tf.approach(pose, (0.34, 0.0), 0.3, False)[0] == pytest.approx((0.0, 0.0, 0.0))     # 대기
    goal, turning = tf.approach(pose, (0.40, 0.0), 0.3, False)
    assert goal == pytest.approx((0.10, 0.0, 0.0)) and not turning                           # 10 cm 다가감
    assert tf.parse_payload({'card_id': 35, 'standoff_m': 0.2})[1] == 0.2


def test_lost_after_defaults_to_sr22_one_second_and_demo_value_is_explicit():
    """놓침 기준 기본 = SR-22 의 1 s. 4 s 는 명령에 적었을 때만(시연값), 범위 밖은 거절."""
    base = {'card_id': 35, 'standoff_m': 0.5}
    assert tf.parse_lost_after(base) == 1.0 and tf.FollowSession(base, 50.0).target_max_age == 1.5
    assert tf.FollowSession({**base, 'lost_after_s': 4}, 50.0).target_max_age == 4.5
    for bad in (0.5, 8, True, '4', float('nan')):
        with pytest.raises(ValueError, match='invalid_lost_after'):
            tf.FollowSession({**base, 'lost_after_s': bad}, 50.0)
    inputs = tf.FollowInput()
    frame(inputs, 100.0)
    s = tf.FollowSession(base, started=50.0)
    assert s.step(inputs, 100.1, 50.0, shift(), POSE)[0] == 'goal'
    inputs.on_status(status('tracking', stamp=101.4, seen=False), 51.4)
    assert s.step(inputs, 101.45, 51.45, shift(), POSE) is None                      # 1.5 s 전까지는 버팀
    inputs.on_status(status('tracking', stamp=101.6, seen=False), 51.6)
    assert s.step(inputs, 101.6, 51.6, shift(), POSE) == ('stop', 'follow_lost')


def test_admit_rejects_hidden_card_and_accepts_when_seen_again():
    """카드가 가려진 채(tracking, seen=false)로는 옛 위치로 새 추종을 시작하지 않는다(주행 중 가림 유예와 별개)."""
    inputs = tf.FollowInput()
    frame(inputs, 100.0, x=1.0)                                  # 1 m 앞에서 마지막으로 봄
    inputs.on_status(status('tracking', stamp=103.0, seen=False), 53.0)
    with pytest.raises(ValueError, match='follow_not_tracking'):
        tf.admit(PAYLOAD, inputs, 103.05, 53.05, shift(), POSE)
    frame(inputs, 103.4, x=0.8, received=53.4)                   # 다시 보임
    assert tf.admit(PAYLOAD, inputs, 103.5, 53.5, shift(), POSE)[:2] == pytest.approx((0.3, 0.0))
    frame(inputs, 103.6, x=0.8, received=53.6)
    with pytest.raises(ValueError, match='follow_target_stale'):  # 본 지 1 s 넘은 위치로는 출발 안 함
        tf.admit(PAYLOAD, inputs, 104.7, 53.7, shift(), POSE)


def test_admit_rejects_position_held_by_doll_fallback():
    """노트북의 「인형으로 잇기」(임시)로 이어 잡은 위치는 새 주행의 출발 근거가 아니다. 주행 중에는 그대로 쓴다."""
    def doll_frame(inputs, stamp, received, source):
        body = {'state': 'tracking', 'card_id': 35, 'seen': True, 'age_s': 0.0, 'stamp': stamp,
                'frame_id': 'base_footprint', 'source': source}
        inputs.on_target(stamp, 'base_footprint', 0.8, 0.0)
        inputs.on_status(tf.parse_status(json.dumps(body)), received)
    inputs = tf.FollowInput()
    doll_frame(inputs, 100.0, 50.0, 'doll')
    with pytest.raises(ValueError, match='follow_not_tracking'):
        tf.admit(PAYLOAD, inputs, 100.1, 50.1, shift(), POSE)
    doll_frame(inputs, 100.4, 50.4, 'card')
    assert tf.admit(PAYLOAD, inputs, 100.5, 50.5, shift(), POSE)[0] == pytest.approx(0.3)
    assert status()['source'] == 'card'                                   # source 없는 status = 카드
    inputs, s = started()
    doll_frame(inputs, 100.5, 50.5, 'doll')                               # 주행 중: 인형으로 이은 위치도 목표로 씀
    assert s.step(inputs, 100.6, 50.6, shift(), POSE)[0] == 'goal'


def test_first_goal_turns_in_place_when_customer_is_off_to_the_side():
    """첫 목표는 첫 관측 방향으로 정한다: 23° 옆 1 m 고객이면 전진이 아니라 제자리 회전(유지 시간은 그 뒤 전환부터)."""
    inputs = tf.FollowInput()
    a = math.radians(23)
    frame(inputs, 100.0, x=math.cos(a), y=math.sin(a))
    x, y, yaw, _ = tf.admit(PAYLOAD, inputs, 100.1, 50.1, shift(), POSE)
    assert (x, y) == pytest.approx((POSE['x'], POSE['y'])) and yaw == pytest.approx(a, abs=1e-6)
    s = tf.FollowSession(PAYLOAD, started=50.0)
    kind, goal = s.step(inputs, 100.1, 50.1, shift(), POSE)
    assert kind == 'goal' and goal[:2] == pytest.approx((POSE['x'], POSE['y'])) and s.turning


def test_stop_goal_is_sent_when_customer_comes_inside_standoff():
    """0.57 m 고객 → 7 cm 앞 목표. 고객이 0.48 m 로 다가오면 지금 자리 목표를 반드시 보낸다(8 cm 미만이라고 거르지 않음)."""
    inputs = tf.FollowInput()
    frame(inputs, 100.0, x=0.57)
    s = tf.FollowSession(PAYLOAD, started=50.0)
    kind, goal = s.step(inputs, 100.1, 50.1, shift(), POSE)
    assert kind == 'goal' and goal[0] == pytest.approx(0.07)
    frame(inputs, 100.5, x=0.48, received=50.5)
    kind, goal = s.step(inputs, 100.6, 50.6, shift(), POSE)
    assert kind == 'goal' and goal[:3] == pytest.approx((POSE['x'], POSE['y'], POSE['yaw']))
    frame(inputs, 101.0, x=0.47, received=51.0)                  # 서 있는 동안에는 다시 보내지 않음
    assert s.step(inputs, 101.1, 51.1, shift(), POSE) is None
