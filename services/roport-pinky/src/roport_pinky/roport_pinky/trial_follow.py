"""고객 카드 추종(follow.requested) 판정부. ROS 없음, TrialDriver 가 lock 아래에서 부른다. FFT1-188.

입력(노트북 roport_follow 가 발행, 같은 ROS_DOMAIN_ID):
- follow/target  geometry_msgs/PointStamped, frame base_footprint, stamp = 로봇 카메라 시각(로봇 시계).
  카드를 이번 영상에서 받아들였을 때만 온다. 점 = 카드 바로 아래 바닥 점(x 앞, y 왼쪽, m).
- follow/status  std_msgs/String JSON, 처리한 영상마다: state(searching|tracking|lost), card_id, seen, stamp ...
  같은 영상의 target 과 status 는 stamp 가 같다. 같은 stamp 의 status 가 tracking·seen·요청 card_id 일 때만
  그 target 을 받아들인다(다른 카드·놓침 상태의 점은 버림).

출력: Nav2 GoalUpdater 입력(goal_update, map frame PoseStamped). 목표는 로봇→고객 직선에서
standoff_m 앞의 점이다(고객 점 자체는 고객 장애물/inflation 안이라 계획이 실패할 수 있어 TruncatePath 대신
목표를 미리 당긴다). 이미 standoff 안이면 현재 위치에 서서 고객 쪽을 본다.

정지(찾기·회전 없음, 새 명령으로만 다시 시작):
  follow_max_duration  max_duration_s 지남
  follow_status_stale  요청 카드의 status 를 1 s 넘게 못 받음(노트북 꺼짐·다른 카드 추종). target 이 먼저
                       끊겨도(카메라 지연만큼 stamp 가 앞섬) 그때 status 가 0.5 s 넘게 없으면 이 코드
  follow_lost          status state 가 lost(노트북 기준 1 s 못 봄, SR-22 예외 흐름) 또는 searching.
                       target(카메라 시각)이 노트북 lost 판정보다 먼저 1 s 를 넘을 때 마지막 status 가
                       seen=false 이면(카드를 못 보는 중) 이 코드
  follow_tf_unavailable 받아들인 target 을 map 으로 못 옮긴 채 1 s 지남
  follow_target_stale  받아들인 target 이 로봇 시각 기준 1 s 넘게 없음
"""

import json
import math

CARD_IDS = range(35, 50)          # 고객 카드 ID(DICT_4X4, roport_follow FollowConfig.valid_ids 와 같음)
STANDOFF_DEFAULT = 0.5            # m, 고객 바닥 점과 로봇 base_footprint 사이(직선)
# m, 아래 = footprint 반폭 0.06 + 앞 끝 0.042 + 인형 반폭·여유(0.2 면 로봇 앞 끝이 카드에서 약 16 cm, 카메라는 이 거리의
# 낮은 카드(5~8 cm)를 봄), 위 = 카드 인식 1.5 m 안. 11-20 실로봇: 간격을 더 가깝게(사용자) → 0.3 → 0.2
STANDOFF_RANGE = (0.2, 1.2)
MAX_DURATION_RANGE = (1.0, 3600.0)  # s
STATUS_MAX_AGE = 2.0              # s, status 수신(로봇 monotonic) 나이. 11-21 실로봇: 노트북 부하로 1 s 끊김에 추종이 끝남 → 2.0
STATUS_SILENT = 0.5               # s, target 이 끊긴 순간 status 도 이만큼 없으면 노트북 끊김으로 분류
# 놓침 기준 lost_after_s(payload, 선택): 노트북이 카드를 이 시간 못 보면 status lost 를 낸다. 기본 = 요구사항 SR-22 의 1 s.
# 4 s 까지는 명령에 명시했을 때만 쓰는 시연값이다(SR-22 변경 합의 전). 11-20~21 d660 실로봇: 인형을 피해 가는 동안
# 고객이 카메라 시야(±24°) 밖으로 나가 1~2 s 로는 추종이 끝남 → 4 s 로 시험. 8 s 는 그 설정에서 태엽 인형을 쳐서 쓰지 않음.
# 그동안 새 목표가 없으므로 로봇은 마지막 고객 위치 앞 목표로 계속 간다. 노트북 --follow-lost-s 와 같은 값을 준다.
LOST_AFTER_DEFAULT = 1.0
LOST_AFTER_RANGE = (1.0, 4.0)
# s, 받아들인 target 의 stamp 나이 상한 = lost_after_s + TARGET_DELAY. stamp 는 카메라 시각이라 노트북 처리·전송 지연
# (실측 0.3~0.5 s)이 더해진다. 지연 여유가 없으면 노트북에서 0.65 s 끊긴 것도 놓침이 됨(11-20 d660 실로봇).
TARGET_DELAY = 0.5
START_TARGET_MAX_AGE = 1.0        # s, 출발할 때는 방금 실제로 본 카드 위치만 쓴다
FUTURE_TOLERANCE = 0.1            # s, 로봇 시각보다 앞선 stamp 허용
STAMP_MATCH = 0.002               # s, target ↔ status stamp 대응
GOAL_PERIOD = 0.25                # s, goal_update 최소 간격(BT 는 4 Hz 로 읽음)
# 고객이 정면에서 TURN_FIRST_RAD 넘게 벗어나면 먼저 제자리 회전, TURN_DONE_RAD 안으로 들어오면 다시 다가감(히스테리시스).
# 15° 하나로 하면 모서리에서 경로가 휘어 15° 근처를 오가며 좌우로 계속 돌았음(11-20 실로봇). 카메라 시야 ±24°.
TURN_FIRST_RAD = math.radians(20)
TURN_DONE_RAD = math.radians(6)
# 고객이 가까이(standoff + HOLD_MARGIN_M 안) 있으면 이동할 일이 없다. 이때 카드 방향 측정이 ±3~5° 흔들려 Nav2(yaw 허용 5.7°)가
# 제자리에서 좌우로 계속 돌았음(11-20 실로봇) → HOLD_YAW_DEADBAND 안이면 지금 방향을 그대로 둔다.
HOLD_MARGIN_M = 0.05   # 11-20: 0.10 이면 고객이 간격 + 10 cm 안에서 안 움직여 간격이 벌어져 보임 → 0.05
# 목표 흔들림 막기(11-20 실로봇: 카드 방향 측정이 ±10~20° 흔들려 목표가 0.3~1 s 마다 「앞으로」↔「제자리 회전」으로 바뀌고,
# 바뀔 때마다 Nav2 제어기가 처음부터 다시 시작해 직진을 못 하고 좌우로만 움직임)
GOAL_MIN_MOVE_M = 0.08           # 직전에 보낸 목표와 이보다 가깝고
GOAL_MIN_TURN_RAD = math.radians(8)  # 방향 차이도 이보다 작으면 다시 보내지 않음
STOP_GOAL_MIN_M = 0.02           # 정지 목표는 직전 목표가 이보다 멀리 남아 있으면 걸러 내지 않음(서 있을 때 위치 떨림보다 큼)
MODE_DWELL_S = 0.8               # 「제자리 회전」·「다가가기」는 바꾼 뒤 이 시간은 유지
HOLD_YAW_DEADBAND = math.radians(10)
MAX_TARGET_RANGE = 4.0            # m, base_footprint 기준 이보다 먼 점은 버림
MIN_BEARING_DIST = 0.05           # m, 이보다 가까우면 방향 계산 안 하고 현재 yaw 유지
PENDING_TARGETS = 10
STATES = ('searching', 'tracking', 'lost')

# 정상적인 추종 종료(고객을 놓침·노트북 끊김·시간)와 수락 뒤 첫 목표 전에 노트북 입력이 바뀐 경우(구동 전).
# driver 를 잠그지 않아 새 follow.requested 로 다시 시작한다.
SOFT_STOPS = frozenset(('follow_lost', 'follow_status_stale', 'follow_target_stale',
                        'follow_max_duration', 'follow_not_tracking', 'follow_card_mismatch',
                        'follow_action_response_failed', 'follow_appearance_failed'))


def _finite(*values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


def parse_payload(payload):
    """follow.requested payload → (card_id, standoff_m, max_duration_s|None). 잘못되면 ValueError(코드)."""
    card = payload.get('card_id')
    if type(card) is not int or card not in CARD_IDS:
        raise ValueError('invalid_card_id')
    standoff = payload.get('standoff_m', STANDOFF_DEFAULT)
    if not _finite(standoff) or not STANDOFF_RANGE[0] <= standoff <= STANDOFF_RANGE[1]:
        raise ValueError('invalid_standoff')
    duration = payload.get('max_duration_s')
    if duration is not None and (not _finite(duration)
                                 or not MAX_DURATION_RANGE[0] <= duration <= MAX_DURATION_RANGE[1]):
        raise ValueError('invalid_max_duration')
    return card, float(standoff), None if duration is None else float(duration)


def parse_lost_after(payload):
    """follow.requested payload 의 lost_after_s(선택) → s. 잘못되면 ValueError(코드)."""
    value = payload.get('lost_after_s', LOST_AFTER_DEFAULT)
    if not _finite(value) or not LOST_AFTER_RANGE[0] <= value <= LOST_AFTER_RANGE[1]:
        raise ValueError('invalid_lost_after')
    return float(value)


def parse_status(text):
    """follow/status JSON → 필요한 필드 dict 또는 None(형식 오류는 받지 않은 것으로 본다)."""
    try:
        body = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(body, dict):
        return None
    card, stamp, seen = body.get('card_id'), body.get('stamp'), body.get('seen')
    if (body.get('state') not in STATES or type(card) is not int or type(seen) is not bool
            or not _finite(stamp) or body.get('frame_id', 'base_footprint') != 'base_footprint'):
        return None
    return {'state': body['state'], 'card_id': card, 'seen': seen, 'stamp': float(stamp),
            'source': body.get('source') if isinstance(body.get('source'), str) else 'card'}


def approach_goal(pose, target, standoff):
    """map 기준 로봇 pose{x,y,yaw}·고객 점(x,y) → 목표 (x, y, yaw). 회전 중이 아닐 때의 판단(approach 참고)."""
    return approach(pose, target, standoff, turning=False)[0]


def goal_for(pose, target, standoff, turning):
    """turning 을 그대로 따를 때의 목표 (x, y, yaw)."""
    dx, dy = target[0] - pose['x'], target[1] - pose['y']
    dist = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if dist > MIN_BEARING_DIST else pose['yaw']
    if dist <= standoff or turning:
        return pose['x'], pose['y'], yaw
    k = (dist - standoff) / dist
    return pose['x'] + dx * k, pose['y'] + dy * k, yaw


def approach(pose, target, standoff, turning):
    """→ ((x, y, yaw), turning). standoff 안이거나 회전 중이면 제자리에서 고객 쪽으로만 돈다.

    회전 시작 = 고객이 정면에서 TURN_FIRST_RAD 넘게 벗어남, 회전 끝 = TURN_DONE_RAD 안. 카메라 시야는 좌우 약 ±24° 라,
    앞으로 가며 천천히 돌면 옆으로 가는 고객이 시야 밖으로 나간다(11-20 실로봇: 24° 에서 놓침).
    """
    dx, dy = target[0] - pose['x'], target[1] - pose['y']
    dist = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if dist > MIN_BEARING_DIST else pose['yaw']
    off = abs(math.atan2(math.sin(yaw - pose['yaw']), math.cos(yaw - pose['yaw'])))
    turning = off > TURN_FIRST_RAD or (turning and off > TURN_DONE_RAD)
    if not turning and dist <= standoff + HOLD_MARGIN_M and off <= HOLD_YAW_DEADBAND:
        return (pose['x'], pose['y'], pose['yaw']), False        # 가까이·거의 정면: 움직이지 않음
    if dist <= standoff or turning:
        return (pose['x'], pose['y'], yaw), turning
    k = (dist - standoff) / dist
    return (pose['x'] + dx * k, pose['y'] + dy * k, yaw), turning


class FollowInput:
    """노트북 입력의 최신 상태. 카드별 마지막 status 와 stamp 로 짝지은 target."""

    def __init__(self):
        self.status = {}          # card_id → (status, 수신 monotonic)
        self.pending = []         # [(stamp, x, y)] status 를 아직 못 만난 target
        self.target = {}          # card_id → (stamp, x, y) 받아들인 최신 target

    def on_status(self, status, received):
        if status is None or status['card_id'] not in CARD_IDS:
            return
        self.status[status['card_id']] = (status, received)
        if status['seen'] and status['state'] == 'tracking':
            for item in self.pending:
                if abs(item[0] - status['stamp']) <= STAMP_MATCH:
                    self._accept(status['card_id'], item)
                    break

    def on_target(self, stamp, frame_id, x, y):
        if (frame_id != 'base_footprint' or not _finite(stamp, x, y)
                or math.hypot(x, y) > MAX_TARGET_RANGE):
            return
        item = (float(stamp), float(x), float(y))
        for status, _ in self.status.values():
            if (status['seen'] and status['state'] == 'tracking'
                    and abs(status['stamp'] - item[0]) <= STAMP_MATCH):
                self._accept(status['card_id'], item)
                return
        self.pending = (self.pending + [item])[-PENDING_TARGETS:]

    def _accept(self, card, item):
        old = self.target.get(card)
        if old is None or item[0] > old[0]:
            self.target[card] = item
        self.pending = [p for p in self.pending if p[0] > item[0] + STAMP_MATCH]


class FollowSession:
    """한 follow.requested 의 진행. step() 이 정지 코드 또는 새 목표를 낸다."""

    def __init__(self, payload, started):
        self.card, self.standoff, self.max_duration = parse_payload(payload)
        self.target_max_age = parse_lost_after(payload) + TARGET_DELAY
        self.started = started
        self.map_target = None    # (stamp, x, y) map 으로 옮긴 최신 target
        self.tf_failed = None     # 마지막으로 TF 에 실패한 target stamp
        self.published = None     # 마지막으로 보낸 목표의 stamp
        self.published_at = -math.inf
        self.turning = False      # 제자리 회전 중(히스테리시스)
        self.mode_since = None    # 첫 동작은 첫 관측 방향으로 정하고(None), 유지 시간(MODE_DWELL_S)은 그때부터 잰다
        self.last_goal = None     # 마지막으로 보낸 (x, y, yaw)

    def step(self, inputs, ros_now, mono_now, transform, pose):
        """→ ('stop', 코드) | ('goal', (x, y, yaw, stamp)) | None.

        transform(stamp, x, y) → map (x, y), 실패하면 LookupError. pose = map 기준 로봇 {x,y,yaw} 또는 None.
        """
        if self.max_duration is not None and mono_now - self.started >= self.max_duration:
            return 'stop', 'follow_max_duration'
        mine = inputs.status.get(self.card)
        if mine is None or not 0 <= mono_now - mine[1] <= STATUS_MAX_AGE:
            return 'stop', 'follow_status_stale'
        if mine[0]['state'] != 'tracking':
            return 'stop', 'follow_lost'
        target = inputs.target.get(self.card)
        if (target is not None and target[0] - ros_now <= FUTURE_TOLERANCE
                and (self.map_target is None or target[0] > self.map_target[0])
                and target[0] != self.tf_failed):
            try:
                self.map_target = (target[0], *transform(*target))
            except LookupError:
                self.tf_failed = target[0]
        if self.map_target is None or ros_now - self.map_target[0] > self.target_max_age:
            if mono_now - mine[1] > STATUS_SILENT:
                return 'stop', 'follow_status_stale'
            if not mine[0]['seen']:
                return 'stop', 'follow_lost'
            fresh = target is not None and -FUTURE_TOLERANCE <= ros_now - target[0] <= self.target_max_age
            return 'stop', 'follow_tf_unavailable' if fresh and self.tf_failed == target[0] \
                else 'follow_target_stale'
        if (pose is None or mono_now - self.published_at < GOAL_PERIOD
                or (self.published is not None and self.map_target[0] <= self.published)):
            return None
        self.published, self.published_at = self.map_target[0], mono_now
        goal, turning = approach(pose, self.map_target[1:], self.standoff, self.turning)
        if self.mode_since is None:                            # 첫 목표: 유지 시간 없이 관측대로
            self.mode_since = mono_now
        elif turning != self.turning:
            if mono_now - self.mode_since < MODE_DWELL_S:      # 막 바꾼 동작은 잠깐 유지
                turning = self.turning
                goal = goal_for(pose, self.map_target[1:], self.standoff, turning)
            else:
                self.mode_since = mono_now
        self.turning = turning
        # 거리 유지로 바뀌는 순간(목표 = 지금 자리)인데 직전에 보낸 목표가 앞쪽에 남아 있으면 반드시 보낸다.
        # 안 그러면 고객이 간격 안으로 들어와도 8 cm 미만 차이라 걸러져 예전 전진 목표로 계속 다가간다.
        stop_here = goal[0] == pose['x'] and goal[1] == pose['y']
        leftover = self.last_goal is not None and math.dist(self.last_goal[:2], goal[:2]) >= STOP_GOAL_MIN_M
        if not (stop_here and leftover) and self.last_goal is not None \
                and math.dist(goal[:2], self.last_goal[:2]) < GOAL_MIN_MOVE_M \
                and abs(math.atan2(math.sin(goal[2] - self.last_goal[2]),
                                   math.cos(goal[2] - self.last_goal[2]))) < GOAL_MIN_TURN_RAD:
            return None                                          # 거의 같은 목표: Nav2 를 다시 시작시키지 않음
        self.last_goal = goal
        return 'goal', (*goal, self.map_target[0])


def admit(payload, inputs, ros_now, mono_now, transform, pose):
    """출발 전 검사(상태를 바꾸지 않음). 첫 목표 (x, y, yaw, stamp) 를 돌려주거나 ValueError(코드)."""
    card = parse_payload(payload)[0]
    mine = inputs.status.get(card)
    if mine is None or not 0 <= mono_now - mine[1] <= STATUS_MAX_AGE:
        others = [c for c, (_, at) in inputs.status.items()
                  if c != card and 0 <= mono_now - at <= STATUS_MAX_AGE]
        raise ValueError('follow_card_mismatch' if others else 'follow_status_stale')
    if mine[0]['state'] != 'tracking':
        raise ValueError('follow_not_tracking')
    # 출발은 그 카드를 지금 실제로 보고 있을 때만: 가려진 동안(tracking, seen=false)의 옛 위치로 새 주행을 시작하지 않는다.
    # 주행 중 가림 유예(lost_after_s)는 step() 이 그대로 적용한다.
    target = inputs.target.get(card)
    if mine[0].get('source', 'card') != 'card':      # 카드 대신 인형으로 이어 잡은 위치(노트북 임시 기능)는 출발 근거가 아님
        raise ValueError('follow_not_tracking')
    if (not mine[0]['seen'] or target is None or abs(target[0] - mine[0]['stamp']) > STAMP_MATCH
            or ros_now - target[0] > START_TARGET_MAX_AGE):
        raise ValueError('follow_not_tracking' if not mine[0]['seen'] else 'follow_target_stale')
    if pose is None:
        raise ValueError('nav2_not_ready')
    session = FollowSession(payload, mono_now)
    result = session.step(inputs, ros_now, mono_now, transform, pose)
    if result is None or result[0] == 'stop':
        raise ValueError(result[1] if result else 'follow_target_stale')
    return result[1]
