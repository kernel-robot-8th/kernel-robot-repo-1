"""경로 주행 상태 기계(FFT1-117). 순수 Python, ROS 없음.

입력: IR 판정(roport_pinky_line.line_sensor.LineReading, 약 20 Hz), odom 자세(cm, rad), 시각(s).
출력: 속도 명령(선속도 m/s, 각속도 rad/s, ROS 기준 반시계 +) 또는 None(발행 안 함).

노드 감지 = IR 패턴 + 이동 거리(2026-11-03 결정, 테이프 추가 없음):
  가로선 노드(정차)  세 채널 모두 테이프(node)가 node_confirm 샘플 연속
  옆 선이 있는 노드  가운데 + 옆 선 쪽 채널이 테이프(분기·모서리·T자)
  odom 이동 거리가 예상 거리의 gate_min~gate_max(+slack) 안일 때만 노드로 인정한다.
  단 창 시작은 예상 위치 − gate_early_cm(5 cm)보다 앞서지 않는다(짧은 구간 이른 오감지 방지).
  예상 거리를 gate_max 넘게 지나도 노드가 없으면 정지(node_missed).
노드에서: 직진은 방향을 유지하며 pass_cm 통과, 회전은 turn_advance_cm(IR ↔ 바퀴 축 + 테이프 반폭)
  더 간 뒤 제자리 회전 → 가운데 채널이 새 선 위에 오고 좌우가 고르면(|오차| <= align_error) 다음 구간.
도착: load = 반시계 90° → 상차 완료(load_done)까지 대기 → 반시계 회전해 진입선을 찾음,
  park = 반시계 회전으로 진입선 확인(back_min_deg 이상) → 후진해 세 채널이 가로선을 보면 정지,
  handoff = 정지(Nav2 전환 가능), stop = 정지.
이어서 출발: 직전 경로가 끝난 노드에서 다시 출발하면 끝난 방향을 기억해 둔다. 첫 구간이 반대쪽이면
  (N12 handoff 뒤 되돌아가기 등) 먼저 반시계로 돌아 진입선을 찾고, 센서 위치(출발 노드 → 센서 거리)도
  끝난 동작으로 정한다. 정지·오류 뒤에는 기억을 지운다(사람이 옮겼을 수 있음).
  기억이 없고 출발 노드가 가로선 노드이면 센서가 가로선 위에 있다고 보고(0 cm) 거리 창을 쓴다.
경유 정차: 경로 중간의 가로선 노드(예: P5 → P4 상차 → P7)에서는 그 노드의 도착 동작(상차 대기 등)을 한 뒤
  같은 규칙(이어서 출발)으로 다음 구간을 이어 간다. 경로는 가로선 노드마다 구간 묶음으로 나눠 검사한다.
속도: set_speed 로 직선·회전 속도를 바꾼다(주행 중에도 다음 샘플부터). 상한은 설정의 max_linear·max_angular.
노드 앞 감속: 직선 추종 중 이동 거리가 예상 노드 위치 - approach_lead_cm(기본 5 cm)를 넘으면, 그리고 노드
  감지 뒤 회전 전 전진·후진은 approach_speed(base_speed 보다 크면 base_speed) 이하로 간다. 출발 위치를 몰라
  예상 위치가 없는 구간은 처음부터 approach_speed. 직진 통과 노드(pass_full_speed)는 감속하지 않는다. 실로봇 d660 0.06 m/s(2026-11-06)에서 노드를
  지나쳐 멈춰 회전이 107~117° 로 끝나고 비스듬히 출발한 문제 대응(0.03 m/s 는 70~93°).
정지 명령·오류(라인 놓침 지속, 노드 놓침, 회전 중 선 못 찾음, 후진 초과, odom·IR 없음)는 모두 0 속도.
정밀 도킹(FFT1-112): 상차 노드별 값 (a, r, b) = (advance_cm, rotate_deg, move_cm), 기본 (0, 90, 0) = 위 load 동작.
  가로선 정지 → a ≠ 0 이면 a 이동(접근 방향, 음수 = 후진) → 반시계 r° → b ≠ 0 이면 b 이동(새 방향) → 상차 대기 →
  b ≠ 0 이면 −b 이동 → a = 0 이면 반시계 회전해 진입선 찾기((180 − r)° 기준, 창 = turn_min·turn_max 와 같은 폭),
  a ≠ 0 이면 시계 r° → −a 이동(기본 정지 자세로) → 반시계 90 + turn_min ~ 90 + turn_max 로 진입선 찾기 → 끝·경유.
  도킹 이동은 odom 이 멈춘 뒤(구동 지연으로 남은 움직임을 먼저 흘려보냄) 시작 자세에서 부호 있는 거리로 재고,
  approach_speed 이하·끝으로 갈수록 감속(min_speed 까지)·방향 유지. 거리 초과·시간 초과·멈추지 않음은 오류 정지.
교차선 출발(start_cross): IR 3개가 route[0](가로선 노드 아님)을 지나는 선 위, 로봇은 start_facing 을 봄
  → ir_ahead_cm 전진(축을 노드 중심으로) → 첫 구간 방향이 다르면 선 찾기 회전 → 첫 구간(s0 = ir_ahead, gate_early 적용).
"""

import math
from collections import deque
from dataclasses import dataclass, replace

from roport_line_nav.dock import DockTable
from roport_line_nav.graph import (HANDOFF, LEFT, LOAD, PARK, RIGHT, STOP, STRAIGHT, GraphError,
                                   BACK, STRAIGHT as GO_STRAIGHT, plan_route, turn_of)
from roport_pinky_line.follower import HARD_MAX_ANGULAR, HARD_MAX_LINEAR

IDLE = 'idle'
FOLLOW = 'follow'
PASS = 'pass'             # 직진 노드 통과(방향 유지)
ADVANCE = 'advance'       # 회전 전 바퀴 축을 노드 위로
ROTATE = 'rotate'
LOADING = 'loading'       # 상차 완료 대기
REVERSE = 'reverse'       # 대기·충전 칸으로 후진
ARRIVED = 'arrived'
HANDOFF_STATE = 'handoff'
STOPPED = 'stopped'
ERROR = 'error'
TERMINAL = (IDLE, ARRIVED, HANDOFF_STATE, STOPPED, ERROR)
FACINGS = ('E', 'N', 'W', 'S')

# 도킹 이동(부호 있는 ADVANCE): odom 정지 판정·종료 조건
SETTLE_CM = 0.02          # 샘플 사이 이동이 이보다 작고
SETTLE_DEG = 0.2          # 회전이 이보다 작은 새 odom 샘플 비교가 node_confirm 번 이어지면 멈춘 것으로 봄
SETTLE_TIMEOUT_S = 1.5    # 0 명령 뒤 이 시간 안에 멈추지 않으면 오류
MOVE_GAIN = 0.01          # m/s per 남은 cm (3 cm 남으면 0.03)
MOVE_OVERRUN_CM = 2.0     # 목표보다 이만큼 더 가면(부호 있는 거리) 오류


@dataclass(frozen=True)
class NavConfig:
    base_speed: float = 0.03       # m/s, 직선 추종
    min_speed: float = 0.015       # m/s
    max_linear: float = 0.06       # m/s
    max_angular: float = 0.6       # rad/s
    kp: float = 1.0                # rad/s per 오차 1
    kd: float = 0.05
    # 속도 맞춤 kp(2026-11-07 시험판): 직선 속도가 gain_ref_speed 를 넘으면 kp × 속도/gain_ref_speed.
    # kp 는 rad/s 라 같은 오차에서 1 cm 당 방향 수정이 속도에 반비례(0.08 은 0.06 의 3/4)
    # → 0.08 m/s d660 에서 회전 뒤 10° 비스듬히 30 cm 끌려가다 반대쪽으로 넘어가 이른 옆선 오감지.
    # 0.06 까지는 실로봇 완주(0.03·0.06)한 kp 그대로. kd 항은 오차 변화율(∝ 속도)이라 그대로 둠.
    kp_speed_scale: bool = True
    gain_ref_speed: float = 0.06   # m/s
    slow_gain: float = 0.5
    heading_kp: float = 2.0        # 통과·전진·후진 중 방향 유지 (rad/s per rad)
    ir_ahead_cm: float = 3.0       # 가운데 IR ↔ 바퀴 축 (2026-11-03 실측 [26])
    half_line_cm: float = 1.2      # 테이프 24 mm 반폭
    turn_advance_cm: float = 4.2   # 옆 선 감지 → 바퀴 축이 옆 선 중심에 올 때까지 (ir_ahead + half_line)
    pass_cm: float = 5.0           # 직진 노드 통과 거리
    suppress_cm: float = 4.0       # 구간 시작 직후 노드 감지 안 함
    gate_min: float = 0.7          # 예상 거리 비율 (±30%, 2026-11-03 결정)
    gate_max: float = 1.3
    gate_slack_cm: float = 3.0
    # 감지 창 시작은 「예상 위치 − gate_early_cm」보다 앞서지 않게(2026-11-07 사용자 결정 1안).
    # 실로그 61건의 진짜 노드는 예상 위치 −3.4~+2.9 cm. N14→N02 모서리(진짜 18.3~18.5 cm, 0.03 5회)를
    # 0.06·0.08 에서 왼쪽 센서가 본선 가장자리를 밟아 10.9~11.2 cm 에서 오감지(3회) → 회전 실패.
    # 비율 창(gate_min)만으로는 짧은 구간 시작이 약 11 cm 라 이를 거르지 못함. 0 이면 끔.
    # 경로 안에서 이어지는 구간(회전·통과·상차·주차 뒤)에만 적용. 사람이 놓은 첫 구간은 출발 위치가
    # 가정값이라(시뮬: N01·N11 칸 7 cm 안쪽 출발에서 진짜 노드를 놓침) 비율 창만 쓴다.
    gate_early_cm: float = 5.0
    rotate_speed: float = 0.4      # rad/s
    rotate_min_speed: float = 0.15
    turn_min_deg: float = 60.0     # 90° 회전에서 새 선을 찾기 시작하는 각도
    turn_max_deg: float = 120.0    # 이 각도까지 못 찾으면 정지
    back_min_deg: float = 172.0    # 180° 회전(대기 칸·되돌아 출발)에서 선을 찾기 시작하는 각도
    back_max_deg: float = 210.0
    align_error: float = 0.25      # 회전 중 가운데 채널이 선 위이고 |오차|가 이 값 이하이면 정렬로 봄
    lost_timeout: float = 1.0      # s, 라인 놓침이 이보다 길면 정지(오류)
    node_confirm: int = 2          # 연속 샘플
    # 줄기 쪽에서 T자(양쪽 갈래)로 들어갈 때는 왼쪽·오른쪽 센서가 둘 다 tee_window_cm 안에서 선을 봐야 노드.
    # 2026-11-07 d660: P5→N14 에서 왼쪽 센서만 0.3 cm(4 샘플) 테이프 가장자리 → N14 오감지 → 회전 실패.
    # 실로그 16건의 진짜 T자 감지는 모두 양쪽이 켜짐(각 1.0~2.8 cm).
    tee_both: bool = True
    tee_window_cm: float = 1.5
    reverse_speed: float = 0.02    # m/s
    reverse_max_cm: float = 12.0
    # 노드 앞 감속(2026-11-06 d660: 0.06 m/s 에서 노드를 지나쳐 멈춤, 0.03 m/s 는 완주).
    # 실제 값 = min(approach_speed, base_speed). base_speed 를 바꿔도 이 값은 그대로.
    approach_speed: float = 0.03   # m/s, 0.03 = 실로봇 완주 속도(2026-11-04·11-06)
    # 예상 노드 위치 몇 cm 전부터 감속(2026-11-07 사용자: 노드 3~5 cm 전). 실로그 61건의 진짜 감지는 예상 위치
    # -3.4 ~ +2.9 cm → 5 cm 면 모두 감속 뒤 감지, 3 cm 면 일찍 오는 노드(-3.1·-3.4)를 빠른 속도로 만남
    approach_lead_cm: float = 5.0
    # 직진 통과 노드(N15·N23 등)는 다가갈 때·지날 때 감속하지 않음(2026-11-07 사용자)
    pass_full_speed: bool = True

    def __post_init__(self):
        if not 0.0 < self.max_linear <= HARD_MAX_LINEAR:
            raise ValueError(f'max_linear must be in (0, {HARD_MAX_LINEAR}]: {self.max_linear}')
        if not 0.0 < self.max_angular <= HARD_MAX_ANGULAR:
            raise ValueError(f'max_angular must be in (0, {HARD_MAX_ANGULAR}]: {self.max_angular}')
        if not 0.0 < self.min_speed <= self.base_speed <= self.max_linear:
            raise ValueError('need 0 < min_speed <= base_speed <= max_linear')
        if not 0.0 < self.reverse_speed <= self.max_linear:
            raise ValueError('need 0 < reverse_speed <= max_linear')
        # 실제 감속 속도는 min(approach_speed, base_speed) ≤ max_linear 라 상한은 하드 상한으로만 본다.
        # trial launch 가 max_linear 를 직선 속도(최소 0.02)로 낮춰도 기본 0.03 으로 시작되게 한다.
        if not (math.isfinite(self.approach_speed)
                and self.min_speed <= self.approach_speed <= HARD_MAX_LINEAR):
            raise ValueError(f'need min_speed <= approach_speed <= {HARD_MAX_LINEAR}')
        if not (math.isfinite(self.approach_lead_cm) and 0.0 <= self.approach_lead_cm <= 50.0):
            raise ValueError('need 0 <= approach_lead_cm <= 50')
        if not 0.0 < self.rotate_min_speed <= self.rotate_speed <= self.max_angular:
            raise ValueError('need 0 < rotate_min_speed <= rotate_speed <= max_angular')
        if not (math.isfinite(self.gate_early_cm) and 0.0 <= self.gate_early_cm <= 50.0):
            raise ValueError('need 0 <= gate_early_cm <= 50')
        if not 0.0 < self.gate_min < 1.0 < self.gate_max:
            raise ValueError('need 0 < gate_min < 1 < gate_max')
        if not 0.0 < self.turn_min_deg < 90.0 < self.turn_max_deg <= 180.0:
            raise ValueError('need 0 < turn_min_deg < 90 < turn_max_deg <= 180')
        if not 150.0 <= self.back_min_deg < 180.0 < self.back_max_deg <= 240.0:
            raise ValueError('need 150 <= back_min_deg < 180 < back_max_deg <= 240')
        if not 0.0 < self.align_error < 1.0:
            raise ValueError('need 0 < align_error < 1')
        if not (math.isfinite(self.gain_ref_speed) and 0.0 < self.gain_ref_speed <= HARD_MAX_LINEAR):
            raise ValueError(f'need 0 < gain_ref_speed <= {HARD_MAX_LINEAR}')
        if not (math.isfinite(self.kp) and math.isfinite(self.kd) and self.kp > 0 and self.kd >= 0):
            raise ValueError('need kp > 0, kd >= 0')
        if not (math.isfinite(self.tee_window_cm) and 0.0 < self.tee_window_cm <= 5.0):
            raise ValueError('need 0 < tee_window_cm <= 5')
        if self.node_confirm < 1 or self.reverse_max_cm <= 0 or self.lost_timeout <= 0:
            raise ValueError('node_confirm >= 1, reverse_max_cm > 0, lost_timeout > 0')


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float


def _clamp(v, limit):
    return max(-limit, min(limit, v))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RouteExecutor:
    def __init__(self, graph, config=None, dock=None):
        self.graph = graph
        self.cfg = config or NavConfig()
        self.dock = dock if dock is not None else DockTable(graph)
        self._dock_phase = None  # 상차 도착 동작 중 어디인지(표시용): a·r·b·load·-b·unr·-a·back·back_a
        self._dock_disp = (0.0, 0.0)   # 표시용 도킹 이동 (접근 방향 cm, 새 방향 cm)
        self.state = IDLE
        self.reason = ''
        self.route = []
        self.legs = []
        self.index = 0
        self.events = deque(maxlen=64)
        self.event_seq = 0       # 지금까지 기록한 이벤트 수(events 는 최근 64개만)
        self.route_seq0 = 0      # 현재 경로를 시작할 때의 event_seq
        self._steps = deque()
        self._zero_once = False
        self._odo = 0.0          # odom 누적 이동 거리 cm
        self._yaw_acc = 0.0      # odom 누적 회전 rad (감기지 않음)
        self._pose = None
        self._pose_stamp = None
        self._now = 0.0
        self._loaded = False     # 상차 완료를 받았는지
        self._leg_s0 = 0.0       # 구간 시작 때 출발 노드 → 센서 거리(표시용 위치 추정)
        self._progress = 0.0     # 현재 구간 진행률 0~1 (정지·오류 뒤에도 마지막 값 유지)
        self._seg_ends = set()   # 경유 정차 구간 묶음의 끝 Leg 번호
        self._end = None         # 직전 경로가 끝난 곳 {'node', 'facing', 's0'} (이어서 출발용)
        self._expected = None    # 지금 구간의 예상 노드 위치(센서 이동 거리 cm). 모르면 None
        self._pre = None         # 출발 전 제자리 회전의 시작 방향(도, 표시용)
        self._reset_primitive()

    # ------------------------------------------------------------ 명령
    def start(self, route, start_offset_cm=None, start_cross=False, start_facing=None):
        """경로 시작. 잘못된 경로는 GraphError(현재 상태 유지). 주행 중이면 거부.

        start_cross: IR 3개가 route[0] 을 지나는 교차선 위, 로봇은 start_facing(E·N·W·S)을 봄.
        """
        if self.state not in TERMINAL:
            raise GraphError(f'busy ({self.state}); stop first')
        legs, seg_ends = self._plan_segments([str(r) for r in route])
        if start_cross:
            self._check_cross(str(route[0]), start_facing, start_offset_cm)
        elif start_facing is not None:
            raise GraphError('start_facing is only for start_cross')
        self.route = [str(r) for r in route]
        self.legs = legs
        self._seg_ends = seg_ends
        self.index = 0
        self.reason = ''
        self._loaded = False
        self._steps.clear()
        self._pre = None
        self.route_seq0 = self.event_seq
        self._log('start', ' → '.join(self.route))
        self._dock_phase = None
        end = self._end
        self._end = None
        if start_cross:
            self._depart_cross(legs[0], start_facing)
            return
        self._depart(legs[0], end, start_offset_cm)

    def _check_cross(self, nid, facing, offset):
        if facing not in FACINGS:
            raise GraphError(f'start_facing must be one of {"/".join(FACINGS)}')
        if offset is not None:
            raise GraphError('start_cross cannot be combined with start_offset_cm')
        node = self.graph.nodes[nid]
        if node.bar:
            raise GraphError(f'start_cross needs a line node, {nid} is a stop (bar) node')
        cross = [turn_of(facing, self.graph.heading(nid, o)) for o in node.links]
        if LEFT not in cross and RIGHT not in cross:
            raise GraphError(f'{nid} has no line across facing {facing}')

    def _depart_cross(self, leg, facing):
        """교차선 위 출발: 축을 노드 중심으로 → (필요하면) 선 찾기 회전 → 첫 구간(s0 = ir_ahead)."""
        c = self.cfg
        turn = turn_of(facing, leg.heading)
        self._log('start_cross', f'{leg.frm} facing {facing} → {leg.heading}')
        self._pre = self._HEADING_DEG[facing]
        self._steps.append((ADVANCE, c.ir_ahead_cm))
        if turn == BACK:
            self._steps.append((ROTATE, 1.0, c.back_min_deg, c.back_max_deg, True))
        elif turn in (LEFT, RIGHT):
            self._steps.append((ROTATE, 1.0 if turn == LEFT else -1.0,
                                c.turn_min_deg, c.turn_max_deg, True))
        self._steps.append(('begin', c.ir_ahead_cm))
        self._next_step()

    def set_speed(self, base_speed=None, rotate_speed=None):
        """직선(m/s)·회전(rad/s) 속도. 범위 밖이면 ValueError(그대로 유지). → 적용된 (직선, 회전)."""
        c = self.cfg
        base = c.base_speed if base_speed is None else float(base_speed)
        rot = c.rotate_speed if rotate_speed is None else float(rotate_speed)
        if not (math.isfinite(base) and c.min_speed <= base <= c.max_linear):
            raise ValueError(f'base_speed must be in {c.min_speed}..{c.max_linear} m/s')
        if not (math.isfinite(rot) and c.rotate_min_speed <= rot <= c.max_angular):
            raise ValueError(
                f'rotate_speed must be in {c.rotate_min_speed}..{c.max_angular} rad/s')
        self.cfg = replace(c, base_speed=base, rotate_speed=rot)
        self._log('speed', f'{base:.3f} m/s, {rot:.2f} rad/s')
        return base, rot

    def set_dock(self, node, advance_cm, rotate_deg, move_cm):
        """상차 노드 도킹 값 적용. 주행 중·범위 밖·상차 노드 아님이면 ValueError(그대로 유지)."""
        self._check_idle('dock')
        v = self.dock.set(node, advance_cm, rotate_deg, move_cm)
        self._log('dock', f'{node} a={v.advance_cm:g} r={v.rotate_deg:g} b={v.move_cm:g}')
        return v

    def reset_dock(self, node):
        self._check_idle('dock')
        self.dock.reset(node)
        self._log('dock', f'{node} reset')

    def replace_dock(self, table, detail=''):
        """검사·저장을 끝낸 표로 바꿈(노드가 파일 저장 뒤 적용할 때). 주행 중이면 ValueError."""
        self._check_idle('dock')
        self.dock = table
        self._log('dock', detail)

    def _check_idle(self, what):
        if self.state not in TERMINAL:
            raise ValueError(f'{what} rejected while driving ({self.state}); stop first')

    def _plan_segments(self, route):
        """중간 가로선 노드에서 나눠 구간 묶음마다 plan_route. → (전체 Leg 목록, 묶음 끝 Leg 번호)."""
        cuts = [i for i in range(1, len(route) - 1) if self.graph.nodes.get(route[i]) is not None
                and self.graph.nodes[route[i]].bar]
        legs, ends, a = [], set(), 0
        for b in cuts + [len(route) - 1]:
            legs += plan_route(self.graph, route[a:b + 1])
            ends.add(len(legs) - 1)
            a = b
        return legs, ends

    def _depart(self, leg, end, s0_override=None):
        """이 노드에서 leg 로 출발: 필요하면 출발 전 회전, 첫 구간 거리 창."""
        facing, pre, s0 = self._start_plan(leg, end)
        if s0_override is not None:
            s0 = s0_override
        if pre is None:
            # 기억한 끝 위치 없이 가정한 거리(사람이 놓은 출발·지정 오프셋)면 gate_early 를 걸지 않음
            guess = end is None or end['node'] != leg.frm or s0_override is not None
            self._begin_leg(s0, guess)
            return
        direction, lo, hi = pre
        self._log('pre_rotate', f'{facing} → {leg.heading}')
        self._pre = self._HEADING_DEG[facing]
        self._steps.extend([(ROTATE, direction, lo, hi, True), ('begin', s0)])
        self._next_step()

    def _start_plan(self, leg, end):
        """→ (기억한 방향, 출발 전 회전 (방향, 최소°, 최대°) 또는 None, 출발 노드 → 센서 거리 cm).

        센서 거리를 모르면 None(첫 구간 최소 거리 제한 없음).
        """
        c = self.cfg
        if end is None or end['node'] != leg.frm:
            return None, None, (0.0 if self.graph.nodes[leg.frm].bar else None)
        facing = end['facing']
        turn = turn_of(facing, leg.heading)
        if turn == GO_STRAIGHT:
            return facing, None, end['s0']
        if turn == BACK:       # 바퀴 축 기준 반 바퀴: 센서가 축 반대편으로
            return (facing, (1.0, c.back_min_deg, c.back_max_deg),
                    2 * c.ir_ahead_cm + c.half_line_cm)
        return facing, (1.0 if turn == LEFT else -1.0, c.turn_min_deg, c.turn_max_deg), None

    def stop(self, reason='stop command'):
        if self.state in TERMINAL and self.state != IDLE:
            self._end = None     # 끝난 뒤의 정지 = 로봇을 옮길 수 있으니 끝난 방향을 잊음
            return
        self._finish(STOPPED, reason)

    def load_done(self):
        """상차 완료. LOADING 에서만 받는다."""
        if self.state != LOADING:
            return False
        self._loaded = True
        self._log('load_done', self.legs[self.index].to)
        self._next_step()
        return True

    def fail(self, reason):
        if self.state in TERMINAL:
            return
        self._finish(ERROR, reason)

    @property
    def active(self):
        return self.state not in TERMINAL

    @property
    def approach_speed(self):
        """노드 앞·통과·전진에 쓰는 직선 속도 m/s(base_speed 를 넘지 않음)."""
        return min(self.cfg.approach_speed, self.cfg.base_speed)

    # ------------------------------------------------------------ 한 샘플
    def step(self, reading, pose, now, pose_stamp=None):
        """IR 한 샘플마다 호출. pose = (x_cm, y_cm, yaw_rad) 또는 None(odom 없음).

        pose_stamp: 그 pose 의 odom 수신 시각 등(같은 odom 이 다시 들어오면 같은 값). None 이면 샘플마다 새 pose 로 본다.
        도킹 이동 전 멈춤 확인은 stamp 가 바뀐 샘플끼리만 비교한다(odom 이 IR 보다 늦게 갱신돼도 멈춤으로 세지 않게).
        """
        self._now = now
        self._pose_stamp = pose_stamp
        if self.state in TERMINAL:
            if self._zero_once:
                self._zero_once = False
                return Command(0.0, 0.0)
            return None
        if pose is None:
            self.fail('no_odom')
            return self.step(reading, pose, now)
        self._update_odom(pose)
        handler = {
            FOLLOW: self._follow, PASS: self._drive, ADVANCE: self._drive,
            ROTATE: self._rotate, LOADING: self._loading, REVERSE: self._reverse,
        }[self.state]
        cmd = handler(reading)
        if self.state in TERMINAL:   # 이 샘플에서 끝남
            self._zero_once = False
            return Command(0.0, 0.0)
        return cmd

    # ------------------------------------------------------------ 상태 보고
    def as_dict(self):
        leg = self.legs[self.index] if self.legs and self.index < len(self.legs) else None
        out = {
            'state': self.state, 'reason': self.reason, 'route': self.route,
            'index': self.index,
            'leg': None if leg is None else {
                'from': leg.frm, 'to': leg.to, 'heading': leg.heading, 'action': leg.action,
                'detect': leg.detect, 'sides': sorted(leg.sides)},
            'traveled_cm': round(self._odo - self._ref_odo, 1),
            'window_cm': None if self.state != FOLLOW else
            [round(self._min_ok, 1), round(self._max_ok, 1)],
            'rotated_deg': round(math.degrees(abs(self._yaw_acc - self._ref_yaw)), 1),
            'events': list(self.events),
            'event_seq': self.event_seq,
            'route_event_seq0': self.route_seq0,
            'dock': self.dock.as_dict(),
            'speed': {'base_speed': self.cfg.base_speed, 'rotate_speed': self.cfg.rotate_speed,
                      'min_speed': self.cfg.min_speed, 'max_linear': self.cfg.max_linear,
                      'rotate_min_speed': self.cfg.rotate_min_speed,
                      'max_angular': self.cfg.max_angular,
                      'approach_speed': self.approach_speed,
                      'approach_lead_cm': self.cfg.approach_lead_cm,
                      'pass_full_speed': self.cfg.pass_full_speed,
                      'kp': self.cfg.kp, 'kd': self.cfg.kd,
                      'kp_at_base': round(self.kp_for(self.cfg.base_speed), 3),
                      'kp_speed_scale': self.cfg.kp_speed_scale,
                      'gain_ref_speed': self.cfg.gain_ref_speed},
        }
        out.update(self._estimate(leg))
        return out

    _HEADING_DEG = {'E': 0.0, 'N': 90.0, 'W': 180.0, 'S': -90.0}

    def _estimate(self, leg):
        """화면 표시용 바퀴 축 위치 추정(실측 cm). 그래프 + 이동 거리로 계산, 제어에는 쓰지 않는다."""
        if leg is None:
            return {'leg_progress': None, 'pose_est': None}
        if self._pre is not None and self.state in (ROTATE, ADVANCE):   # 출발 전 전진·제자리 회전
            n = self.graph.nodes[leg.frm]
            x, y = n.x, n.y
            if self.state == ROTATE:
                turned = math.degrees(abs(self._yaw_acc - self._ref_yaw))
                heading = self._pre + self._prim['dir'] * turned
            else:   # 교차선 출발: 축이 노드 중심 ir_ahead 뒤에서 앞으로
                heading = self._pre
                back = max(0.0, self._prim['cm'] - (self._odo - self._ref_odo))
                ux, uy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
                x, y = n.x - ux * back, n.y - uy * back
            heading = (heading + 180.0) % 360.0 - 180.0
            self._progress = 0.0
            return {'leg_progress': 0.0, 'pose_est': {'x_cm': round(x, 1), 'y_cm': round(y, 1),
                                                      'heading_deg': round(heading, 1)}}
        if self.state == FOLLOW:
            axle = self._leg_s0 + (self._odo - self._ref_odo) - self.cfg.ir_ahead_cm
            self._progress = min(1.0, max(0.0, axle / leg.length)) if leg.length > 0 else 1.0
        elif self.state in (PASS, ADVANCE, ROTATE, LOADING, REVERSE, ARRIVED, HANDOFF_STATE):
            self._progress = 1.0
        a, b = self.graph.nodes[leg.frm], self.graph.nodes[leg.to]
        heading = self._HEADING_DEG[leg.heading]
        f = self._progress
        x, y = a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f
        if leg.action == LOAD and self._dock_phase is not None and \
                self.state in (ADVANCE, ROTATE, LOADING, ARRIVED):
            x, y, heading = self._dock_estimate(leg, x, y, heading)
        elif leg.action == PARK and self.state in (REVERSE, ARRIVED):
            heading += 180.0
        elif self.state == ROTATE and self._prim:
            heading += self._prim['dir'] * math.degrees(abs(self._yaw_acc - self._ref_yaw))
        heading = (heading + 180.0) % 360.0 - 180.0
        return {'leg_progress': round(f, 3), 'pose_est': {
            'x_cm': round(x, 1), 'y_cm': round(y, 1), 'heading_deg': round(heading, 1)}}

    def _dock_estimate(self, leg, x, y, approach):
        """상차 도착 동작 중 표시 자세(바퀴 축): 가로선 정지 위치(노드 중심에서 접근 반대로 half_line + ir_ahead)
        + a·접근 방향 + b·(접근 + r) 방향, 방향 = 접근 + 회전. 정지 밀림(구동 지연)은 모른다."""
        r = self.dock.get(leg.to).rotate_deg
        da, db = self._dock_disp
        da -= self.cfg.half_line_cm + self.cfg.ir_ahead_cm
        phase = self._dock_phase
        p = self._prim
        moving = self.state == ADVANCE and p.get('signed')
        turned = math.degrees(abs(self._yaw_acc - self._ref_yaw)) if self.state == ROTATE else 0.0
        if moving and phase in ('a', '-a'):
            da += p['proj']
        elif moving:
            db += p['proj']
        if phase in ('a', '-a'):
            heading = approach
        elif phase == 'r':
            heading = approach + (turned if self.state == ROTATE else r)
        elif phase == 'unr':                  # 시계로 r° 되돌림
            heading = approach + (r - turned if self.state == ROTATE else 0.0)
        elif phase == 'back':
            heading = approach + (r + turned if self.state == ROTATE else 180.0)
        elif phase == 'back_a':
            heading = approach + (turned if self.state == ROTATE else 180.0)
        else:      # b·load·-b
            heading = approach + r
        u = math.radians(approach)
        h = math.radians(approach + r)
        return (x + da * math.cos(u) + db * math.cos(h),
                y + da * math.sin(u) + db * math.sin(h), heading)

    # ------------------------------------------------------------ 내부
    def _log(self, kind, detail=''):
        self.event_seq += 1
        self.events.append({'t': round(self._now, 2), 'event': kind, 'detail': detail})

    def _reset_primitive(self):
        self._ref_odo = self._odo
        self._ref_yaw = self._yaw_acc
        self._hold_yaw = None if self._pose is None else self._pose[2]
        self._hits = 0
        self._lost_since = None
        self._prev_error = None
        self._prev_t = None
        self._min_ok = 0.0
        self._max_ok = 0.0
        self._prim = {}
        self._tee_seen = {0: None, 2: None}   # T자: 왼쪽·오른쪽 센서가 마지막으로 선을 본 이동 거리

    def _update_odom(self, pose):
        if self._pose is not None:
            self._odo += math.hypot(pose[0] - self._pose[0], pose[1] - self._pose[1])
            self._yaw_acc += _wrap(pose[2] - self._pose[2])
        self._pose = pose
        if self._hold_yaw is None:
            self._hold_yaw = pose[2]

    def _finish(self, state, reason=''):
        self.state = state
        self.reason = reason
        self._steps.clear()
        self._zero_once = True
        self._pre = None
        self._end = (self._end_of_leg(self.legs[self.index])
                     if state in (ARRIVED, HANDOFF_STATE) else None)
        self._log(state, reason)

    def _end_of_leg(self, leg):
        """leg 도착 동작 뒤 노드·로봇이 보는 방향·출발 노드 → 센서 거리(이어서 출발할 때)."""
        c = self.cfg
        back = {'E': 'W', 'W': 'E', 'N': 'S', 'S': 'N'}[leg.heading]
        if leg.action == LOAD:    # 가로선 앞에서 멈춘 뒤 축 기준 약 180°: 센서가 노드 바깥쪽으로
            # 도킹 ①(a)은 상차 뒤 −a 로 되돌아오므로 기본 정지 자세와 같음
            return {'node': leg.to, 'facing': back, 's0': 2 * c.ir_ahead_cm + c.half_line_cm}
        if leg.action == PARK:    # 후진해 가로선 가장자리에서 멈춤
            return {'node': leg.to, 'facing': back, 's0': c.half_line_cm}
        return {'node': leg.to, 'facing': leg.heading, 's0': None}

    def _begin_leg(self, s0_cm, guess=False):
        """구간 시작. s0_cm = 출발 노드에서 센서까지 거리(모르면 None → 최소 거리 제한 없음).

        guess: s0 가 가정값(사람이 놓은 출발 위치)이면 True → 비율 창만 쓰고 gate_early 는 안 씀.
        """
        leg = self.legs[self.index]
        self.state = FOLLOW
        self._reset_primitive()
        self._leg_s0 = 0.0 if s0_cm is None else s0_cm
        self._progress = 0.0
        c = self.cfg
        target = leg.length - c.half_line_cm      # 센서가 도착 노드 선의 가까운 가장자리에 닿는 거리
        if s0_cm is None:
            self._expected = None
            self._min_ok = 0.0
            self._max_ok = c.gate_max * leg.length + c.gate_slack_cm
        else:
            expected = max(0.0, target - s0_cm)
            self._expected = expected
            self._min_ok = max(0.0, c.gate_min * expected - c.gate_slack_cm)
            if c.gate_early_cm > 0 and not guess:
                self._min_ok = max(self._min_ok, expected - c.gate_early_cm)
            self._max_ok = c.gate_max * expected + c.gate_slack_cm
        self._log('leg', f'{leg.frm}→{leg.to} {leg.action}')

    def _hit(self, reading, leg, d=0.0):
        if leg.detect == 'bar':
            return reading.node
        on = reading.on_line
        if not on[1]:
            return False
        if self.cfg.tee_both and LEFT in leg.sides and RIGHT in leg.sides:
            for ch in (0, 2):
                if on[ch]:
                    self._tee_seen[ch] = d
            a, b = self._tee_seen[0], self._tee_seen[2]
            return (on[0] or on[2]) and a is not None and b is not None \
                and abs(a - b) <= self.cfg.tee_window_cm
        return (LEFT in leg.sides and on[0]) or (RIGHT in leg.sides and on[2])

    def _follow(self, reading):
        c = self.cfg
        leg = self.legs[self.index]
        d = self._odo - self._ref_odo
        if d >= c.suppress_cm and d >= self._min_ok and self._hit(reading, leg, d):
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= c.node_confirm:
            self._arrive(leg)
            return self._dispatch(reading)
        if d > self._max_ok:
            self.fail(f'node_missed: {leg.to} not found within {self._max_ok:.1f} cm')
            return None
        if reading.lost:
            if self._lost_since is None:
                self._lost_since = self._now
            elif self._now - self._lost_since > c.lost_timeout:
                self.fail(f'line_lost on {leg.frm}→{leg.to}')
                return None
            self._prev_error = None
            return Command(0.0, 0.0)
        self._lost_since = None
        e = reading.error
        de = 0.0
        if self._prev_error is not None and self._prev_t is not None and self._now > self._prev_t:
            de = (e - self._prev_error) / (self._now - self._prev_t)
        self._prev_error, self._prev_t = e, self._now
        # 예상 노드 위치 approach_lead_cm 전부터 감속(출발 위치 모름 = 처음부터). 직진 통과 구간은 감속 안 함
        if c.pass_full_speed and leg.action == STRAIGHT:
            speed = c.base_speed
        elif self._expected is None:
            speed = self.approach_speed
        else:
            speed = self.approach_speed if d >= self._expected - c.approach_lead_cm else c.base_speed
        angular = _clamp(-(self.kp_for(speed) * e + c.kd * de), c.max_angular)
        linear = speed * (1.0 - c.slow_gain * abs(e))
        linear = min(c.max_linear, max(c.min_speed, linear))
        return Command(linear, angular)

    def kp_for(self, speed):
        """직선 추종 kp. kp_speed_scale 이면 gain_ref_speed 를 넘는 속도에서 속도에 비례해 키움."""
        c = self.cfg
        if not c.kp_speed_scale or speed <= c.gain_ref_speed:
            return c.kp
        return c.kp * speed / c.gain_ref_speed

    def _arrive(self, leg):
        c = self.cfg
        self._log('node', f'{leg.to} ({leg.action})')
        steps = self._steps
        steps.clear()
        last = self.index == len(self.legs) - 1
        done = ('finish',) if last else ('via',)      # 경유 정차면 다음 구간으로
        if leg.action == STRAIGHT:
            steps.append((PASS, c.pass_cm))
            steps.append(('next', c.pass_cm - c.half_line_cm))
        elif leg.action in (LEFT, RIGHT):
            direction = 1.0 if leg.action == LEFT else -1.0
            steps.append((ADVANCE, c.turn_advance_cm))
            steps.append((ROTATE, direction, c.turn_min_deg, c.turn_max_deg, True))
            steps.append(('next', c.ir_ahead_cm))
        elif leg.action == LOAD:
            # (a 이동) → 반시계 r° → (b 이동) → 상차 대기 → (−b 이동) → 반시계로 진입선을 다시 찾음(합계 약 180°)
            # 기본 (0, 90, 0) 이면 반시계 90° → 상차 대기 → 반시계 turn_min~turn_max(예전과 같음)
            dv = self.dock.get(leg.to)
            r = dv.rotate_deg
            self._dock_disp = (0.0, 0.0)
            if dv.advance_cm:
                steps.append((ADVANCE, dv.advance_cm, 'a'))
            steps.append((ROTATE, 1.0, r, r, False, 'r'))
            if dv.move_cm:
                steps.append((ADVANCE, dv.move_cm, 'b'))
            steps.append((LOADING,))
            if dv.move_cm:
                steps.append((ADVANCE, -dv.move_cm, '-b'))
            if dv.advance_cm:
                # a ≠ 0(2026-11-09 사용자 결정 (나)): 시계 r° 로 접근 방향을 다시 보고 −a 만큼 되돌아와 기본 정지
                # 자세로 → 반시계 90 + turn_min ~ 90 + turn_max(기본값 경로의 합계)로 진입선 찾기.
                # a > +4.5 cm 그대로 돌아보면 IR 이 가로선 위에 남아 진입선을 구분 못 함(시뮬 a +6 에서 22/24 정지).
                steps.append((ROTATE, -1.0, r, r, False, 'unr'))
                steps.append((ADVANCE, -dv.advance_cm, '-a'))
                lo, hi = 90.0 + c.turn_min_deg, 90.0 + c.turn_max_deg
                phase = 'back_a'
            elif r == 90.0:
                lo, hi = c.turn_min_deg, c.turn_max_deg
                phase = 'back'
            else:     # 남은 각 (180 − r) 둘레로 기본과 같은 폭(기본 −30°·+30°)
                lo, hi = (180.0 - r) + (c.turn_min_deg - 90.0), (180.0 - r) + (c.turn_max_deg - 90.0)
                phase = 'back'
            steps.append((ROTATE, 1.0, lo, hi, True, phase))
            steps.append(done + (ARRIVED,))
        elif leg.action == PARK:
            steps.append((ROTATE, 1.0, c.back_min_deg, c.back_max_deg, True))
            steps.append((REVERSE, c.reverse_max_cm))
            steps.append(done + (ARRIVED,))
        elif leg.action == HANDOFF:
            steps.append(done + (HANDOFF_STATE,))
        elif leg.action == STOP:
            steps.append(done + (ARRIVED,))
        else:  # pragma: no cover - plan_route 가 막음
            raise GraphError(f'unknown action {leg.action}')
        if self.index not in self._seg_ends and leg.action not in (STRAIGHT, LEFT, RIGHT):
            raise GraphError('arrival action before the last leg')  # pragma: no cover
        self._next_step()

    def _next_step(self):
        if not self._steps:
            self._finish(ARRIVED, 'route done')
            return
        st = self._steps.popleft()
        kind = st[0]
        if kind == 'next':
            self.index += 1
            self._begin_leg(st[1])
            return
        if kind == 'finish':
            self._finish(st[1], 'route done')
            return
        if kind == 'via':        # 경유 정차 동작이 끝남 → 이어서 출발
            leg = self.legs[self.index]
            self._log('via', leg.to)
            end = self._end_of_leg(leg)
            self.index += 1
            self._loaded = False
            self._dock_phase = None
            self._depart(self.legs[self.index], end)
            return
        if kind == 'begin':      # 출발 전 회전이 끝남 → 첫 구간
            self._pre = None
            self._begin_leg(st[1])
            return
        self._dock_step_done()
        self.state = kind
        self._reset_primitive()
        if kind == ADVANCE and len(st) > 2:      # 도킹 이동: 부호 있는 거리, 멈춘 뒤 시작
            self._prim = {'cm': st[1], 'signed': True, 'phase': st[2], 'settled': False,
                          'still': 0, 'last': None, 'last_stamp': None, 't0': self._now,
                          'proj': 0.0}
            self._dock_phase = st[2]
        elif kind in (PASS, ADVANCE):
            self._prim = {'cm': st[1]}
        elif kind == ROTATE:
            self._prim = {'dir': st[1], 'min': math.radians(st[2]), 'max': math.radians(st[3]),
                          'acquire': st[4]}
            if len(st) > 5:
                self._dock_phase = st[5]
        elif kind == REVERSE:
            self._prim = {'cm': st[1]}
        elif kind == LOADING and self._dock_phase is not None:
            self._dock_phase = 'load'
        self._log(kind, '' if kind == LOADING else str(st[1:]))

    def _dock_step_done(self):
        """끝난 도킹 이동을 표시용 누적값에 더한다."""
        p = self._prim
        if p.get('signed'):
            a, b = self._dock_disp
            if p['phase'] in ('a', '-a'):
                a += p['proj']
            else:
                b += p['proj']
            self._dock_disp = (a, b)

    def _dispatch(self, reading):
        """상태가 바뀐 같은 샘플에서 새 상태의 명령을 낸다."""
        if self.state in TERMINAL:
            return None
        if self.state == FOLLOW:
            return Command(0.0, 0.0)  # 다음 샘플부터 추종
        return {PASS: self._drive, ADVANCE: self._drive, ROTATE: self._rotate,
                LOADING: self._loading, REVERSE: self._reverse}[self.state](reading)

    def _hold(self):
        return _clamp(self.cfg.heading_kp * _wrap(self._hold_yaw - self._pose[2]),
                      self.cfg.max_angular)

    def _drive(self, reading):
        if self._prim.get('signed'):
            return self._dock_move(reading)
        if self._odo - self._ref_odo >= self._prim['cm']:
            self._next_step()
            return self._dispatch(reading)
        fast = self.state == PASS and self.cfg.pass_full_speed     # 직진 통과는 감속 안 함
        return Command(self.cfg.base_speed if fast else self.approach_speed, self._hold())

    def _dock_move(self, reading):
        """도킹 이동: 0 명령으로 odom 이 멈추길 기다린 뒤, 그 자세에서 부호 있는 거리(방향 유지·감속)."""
        c = self.cfg
        p = self._prim
        x, y, yaw = self._pose
        if not p['settled']:
            stamp = self._pose_stamp
            fresh = stamp is None or stamp != p['last_stamp']
            if p['last'] is None:            # 첫 샘플은 비교 기준만(같은 pose 로 멈춤을 세지 않음)
                p['last'], p['last_stamp'] = self._pose, stamp
            elif fresh:                      # odom 이 새로 들어온 샘플끼리만 비교
                lx, ly, lyaw = p['last']
                still = (math.hypot(x - lx, y - ly) < SETTLE_CM
                         and abs(_wrap(yaw - lyaw)) < math.radians(SETTLE_DEG))
                p['last'], p['last_stamp'] = self._pose, stamp
                p['still'] = p['still'] + 1 if still else 0
            if p['still'] >= c.node_confirm:
                p.update(settled=True, origin=(x, y), t0=self._now)
                self._hold_yaw = yaw
            elif self._now - p['t0'] > SETTLE_TIMEOUT_S:
                self.fail('dock_move: odom did not settle')
                return None
            return Command(0.0, 0.0)
        ox, oy = p['origin']
        proj = (x - ox) * math.cos(self._hold_yaw) + (y - oy) * math.sin(self._hold_yaw)
        p['proj'] = proj
        target = p['cm']
        sign = 1.0 if target > 0 else -1.0
        remaining = (target - proj) * sign
        if remaining <= 0.0:
            self._next_step()
            return self._dispatch(reading)
        # 반대쪽으로 밀리거나(remaining 이 목표 + 여유보다 큼) 경로 길이가 넘치면 정지
        if remaining > abs(target) + MOVE_OVERRUN_CM or \
                self._odo - self._ref_odo > abs(target) + 2 * MOVE_OVERRUN_CM:
            self.fail(f'dock_move overrun ({proj:.1f} of {target:.1f} cm)')
            return None
        if self._now - p['t0'] > 2.0 + 2.0 * abs(target) / (c.min_speed * 100.0):
            self.fail(f'dock_move timeout ({proj:.1f} of {target:.1f} cm)')
            return None
        speed = min(self.approach_speed, max(c.min_speed, MOVE_GAIN * remaining))
        return Command(sign * speed, self._hold())

    def _rotate(self, reading):
        c = self.cfg
        p = self._prim
        turned = abs(self._yaw_acc - self._ref_yaw)
        if p['acquire']:
            if (turned >= p['min'] and reading.on_line[1]
                    and abs(reading.error) <= c.align_error):
                self._hits += 1
            else:
                self._hits = 0
            if self._hits >= c.node_confirm:
                self._log('line_found', f'{math.degrees(turned):.0f}°')
                self._next_step()
                return self._dispatch(reading)
            if turned > p['max']:
                self.fail(f'line_not_found after {math.degrees(turned):.0f}°')
                return None
            if turned < p['min']:
                speed = c.rotate_speed
            else:   # 찾기 시작하면 천천히(정렬 판정을 놓치지 않게)
                speed = max(c.rotate_min_speed, 0.6 * c.rotate_speed)
            return Command(0.0, p['dir'] * speed)
        remaining = p['max'] - turned
        if remaining <= math.radians(1.5):
            self._next_step()
            return self._dispatch(reading)
        speed = min(c.rotate_speed, max(c.rotate_min_speed, 1.5 * remaining))
        return Command(0.0, p['dir'] * speed)

    def _loading(self, reading):
        return Command(0.0, 0.0)

    def _reverse(self, reading):
        c = self.cfg
        if reading.node:
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= c.node_confirm:
            self._next_step()
            return self._dispatch(reading)
        if self._odo - self._ref_odo > self._prim['cm']:
            self.fail(f'park_overrun: bar not found within {self._prim["cm"]:.0f} cm')
            return None
        return Command(-min(c.reverse_speed, self.approach_speed), self._hold())


__all__ = ['NavConfig', 'Command', 'RouteExecutor', 'GraphError', 'IDLE', 'FOLLOW', 'PASS',
           'ADVANCE', 'ROTATE', 'LOADING', 'REVERSE', 'ARRIVED', 'HANDOFF_STATE', 'STOPPED',
           'ERROR', 'STOP', 'TERMINAL']
