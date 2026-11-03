"""경로 주행 상태 기계(FFT1-117). 순수 Python, ROS 없음.

입력: IR 판정(roport_pinky_line.line_sensor.LineReading, 약 20 Hz), odom 자세(cm, rad), 시각(s).
출력: 속도 명령(선속도 m/s, 각속도 rad/s, ROS 기준 반시계 +) 또는 None(발행 안 함).

노드 감지 = IR 패턴 + 이동 거리(2026-11-03 결정, 테이프 추가 없음):
  가로선 노드(정차)  세 채널 모두 테이프(node)가 node_confirm 샘플 연속
  옆 선이 있는 노드  가운데 + 옆 선 쪽 채널이 테이프(분기·모서리·T자)
  odom 이동 거리가 예상 거리의 gate_min~gate_max(+slack) 안일 때만 노드로 인정한다.
  예상 거리를 gate_max 넘게 지나도 노드가 없으면 정지(node_missed).
노드에서: 직진은 방향을 유지하며 pass_cm 통과, 회전은 turn_advance_cm(IR ↔ 바퀴 축 + 테이프 반폭)
  더 간 뒤 제자리 회전 → 가운데 채널이 새 선 위에 오고 좌우가 고르면(|오차| <= align_error) 다음 구간.
도착: load = 반시계 90° → 상차 완료(load_done)까지 대기 → 반시계 회전해 진입선을 찾음,
  park = 반시계 회전으로 진입선 확인(150° 이상) → 후진해 세 채널이 가로선을 보면 정지,
  handoff = 정지(Nav2 전환 가능), stop = 정지.
정지 명령·오류(라인 놓침 지속, 노드 놓침, 회전 중 선 못 찾음, 후진 초과, odom·IR 없음)는 모두 0 속도.
"""

import math
from collections import deque
from dataclasses import dataclass

from roport_line_nav.graph import (HANDOFF, LEFT, LOAD, PARK, RIGHT, STOP, STRAIGHT, GraphError,
                                   plan_route)
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


@dataclass(frozen=True)
class NavConfig:
    base_speed: float = 0.03       # m/s, 직선 추종
    min_speed: float = 0.015       # m/s
    max_linear: float = 0.06       # m/s
    max_angular: float = 0.6       # rad/s
    kp: float = 1.0                # rad/s per 오차 1
    kd: float = 0.05
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
    rotate_speed: float = 0.4      # rad/s
    rotate_min_speed: float = 0.15
    turn_min_deg: float = 60.0     # 90° 회전에서 새 선을 찾기 시작하는 각도
    turn_max_deg: float = 120.0    # 이 각도까지 못 찾으면 정지
    align_error: float = 0.25      # 회전 중 가운데 채널이 선 위이고 |오차|가 이 값 이하이면 정렬로 봄
    lost_timeout: float = 1.0      # s, 라인 놓침이 이보다 길면 정지(오류)
    node_confirm: int = 2          # 연속 샘플
    reverse_speed: float = 0.02    # m/s
    reverse_max_cm: float = 12.0

    def __post_init__(self):
        if not 0.0 < self.max_linear <= HARD_MAX_LINEAR:
            raise ValueError(f'max_linear must be in (0, {HARD_MAX_LINEAR}]: {self.max_linear}')
        if not 0.0 < self.max_angular <= HARD_MAX_ANGULAR:
            raise ValueError(f'max_angular must be in (0, {HARD_MAX_ANGULAR}]: {self.max_angular}')
        if not 0.0 < self.min_speed <= self.base_speed <= self.max_linear:
            raise ValueError('need 0 < min_speed <= base_speed <= max_linear')
        if not 0.0 < self.reverse_speed <= self.max_linear:
            raise ValueError('need 0 < reverse_speed <= max_linear')
        if not 0.0 < self.rotate_min_speed <= self.rotate_speed <= self.max_angular:
            raise ValueError('need 0 < rotate_min_speed <= rotate_speed <= max_angular')
        if not 0.0 < self.gate_min < 1.0 < self.gate_max:
            raise ValueError('need 0 < gate_min < 1 < gate_max')
        if not 0.0 < self.turn_min_deg < 90.0 < self.turn_max_deg <= 180.0:
            raise ValueError('need 0 < turn_min_deg < 90 < turn_max_deg <= 180')
        if not 0.0 < self.align_error < 1.0:
            raise ValueError('need 0 < align_error < 1')
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
    def __init__(self, graph, config=None):
        self.graph = graph
        self.cfg = config or NavConfig()
        self.state = IDLE
        self.reason = ''
        self.route = []
        self.legs = []
        self.index = 0
        self.events = deque(maxlen=64)
        self._steps = deque()
        self._zero_once = False
        self._odo = 0.0          # odom 누적 이동 거리 cm
        self._yaw_acc = 0.0      # odom 누적 회전 rad (감기지 않음)
        self._pose = None
        self._now = 0.0
        self._loaded = False     # 상차 완료를 받았는지(두 번째 회전 표시용)
        self._leg_s0 = 0.0       # 구간 시작 때 출발 노드 → 센서 거리(표시용 위치 추정)
        self._progress = 0.0     # 현재 구간 진행률 0~1 (정지·오류 뒤에도 마지막 값 유지)
        self._reset_primitive()

    # ------------------------------------------------------------ 명령
    def start(self, route, start_offset_cm=None):
        """경로 시작. 잘못된 경로는 GraphError(현재 상태 유지). 주행 중이면 거부."""
        if self.state not in TERMINAL:
            raise GraphError(f'busy ({self.state}); stop first')
        legs = plan_route(self.graph, route)
        self.route = [str(r) for r in route]
        self.legs = legs
        self.index = 0
        self.reason = ''
        self._loaded = False
        self._steps.clear()
        self._log('start', ' → '.join(self.route))
        self._begin_leg(start_offset_cm)

    def stop(self, reason='stop command'):
        if self.state in TERMINAL and self.state != IDLE:
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

    # ------------------------------------------------------------ 한 샘플
    def step(self, reading, pose, now):
        """IR 한 샘플마다 호출. pose = (x_cm, y_cm, yaw_rad) 또는 None(odom 없음)."""
        self._now = now
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
        }
        out.update(self._estimate(leg))
        return out

    _HEADING_DEG = {'E': 0.0, 'N': 90.0, 'W': 180.0, 'S': -90.0}

    def _estimate(self, leg):
        """화면 표시용 바퀴 축 위치 추정(실측 cm). 그래프 + 이동 거리로 계산, 제어에는 쓰지 않는다."""
        if leg is None:
            return {'leg_progress': None, 'pose_est': None}
        if self.state == FOLLOW:
            axle = self._leg_s0 + (self._odo - self._ref_odo) - self.cfg.ir_ahead_cm
            self._progress = min(1.0, max(0.0, axle / leg.length)) if leg.length > 0 else 1.0
        elif self.state in (PASS, ADVANCE, ROTATE, LOADING, REVERSE, ARRIVED, HANDOFF_STATE):
            self._progress = 1.0
        a, b = self.graph.nodes[leg.frm], self.graph.nodes[leg.to]
        heading = self._HEADING_DEG[leg.heading]
        if leg.action == LOAD and self.state in (LOADING, ARRIVED):
            heading += 90.0 if self.state == LOADING else 180.0   # 반시계 90° / 합계 약 180°
        elif leg.action == PARK and self.state in (REVERSE, ARRIVED):
            heading += 180.0
        elif self.state == ROTATE and self._prim:
            base = 90.0 if leg.action == LOAD and self._loaded else 0.0   # 상차 뒤 두 번째 회전
            heading += base + self._prim['dir'] * math.degrees(abs(self._yaw_acc - self._ref_yaw))
        heading = (heading + 180.0) % 360.0 - 180.0
        f = self._progress
        return {'leg_progress': round(f, 3), 'pose_est': {
            'x_cm': round(a.x + (b.x - a.x) * f, 1), 'y_cm': round(a.y + (b.y - a.y) * f, 1),
            'heading_deg': round(heading, 1)}}

    # ------------------------------------------------------------ 내부
    def _log(self, kind, detail=''):
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
        self._log(state, reason)

    def _begin_leg(self, s0_cm):
        """구간 시작. s0_cm = 출발 노드에서 센서까지 거리(모르면 None → 최소 거리 제한 없음)."""
        leg = self.legs[self.index]
        self.state = FOLLOW
        self._reset_primitive()
        self._leg_s0 = 0.0 if s0_cm is None else s0_cm
        self._progress = 0.0
        c = self.cfg
        target = leg.length - c.half_line_cm      # 센서가 도착 노드 선의 가까운 가장자리에 닿는 거리
        if s0_cm is None:
            self._min_ok = 0.0
            self._max_ok = c.gate_max * leg.length + c.gate_slack_cm
        else:
            expected = max(0.0, target - s0_cm)
            self._min_ok = max(0.0, c.gate_min * expected - c.gate_slack_cm)
            self._max_ok = c.gate_max * expected + c.gate_slack_cm
        self._log('leg', f'{leg.frm}→{leg.to} {leg.action}')

    def _hit(self, reading, leg):
        if leg.detect == 'bar':
            return reading.node
        on = reading.on_line
        if not on[1]:
            return False
        return (LEFT in leg.sides and on[0]) or (RIGHT in leg.sides and on[2])

    def _follow(self, reading):
        c = self.cfg
        leg = self.legs[self.index]
        d = self._odo - self._ref_odo
        if d >= c.suppress_cm and d >= self._min_ok and self._hit(reading, leg):
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
        angular = _clamp(-(c.kp * e + c.kd * de), c.max_angular)
        linear = c.base_speed * (1.0 - c.slow_gain * abs(e))
        linear = min(c.max_linear, max(c.min_speed, linear))
        return Command(linear, angular)

    def _arrive(self, leg):
        c = self.cfg
        self._log('node', f'{leg.to} ({leg.action})')
        steps = self._steps
        steps.clear()
        last = self.index == len(self.legs) - 1
        if leg.action == STRAIGHT:
            steps.append((PASS, c.pass_cm))
            steps.append(('next', c.pass_cm - c.half_line_cm))
        elif leg.action in (LEFT, RIGHT):
            direction = 1.0 if leg.action == LEFT else -1.0
            steps.append((ADVANCE, c.turn_advance_cm))
            steps.append((ROTATE, direction, c.turn_min_deg, c.turn_max_deg, True))
            steps.append(('next', c.ir_ahead_cm))
        elif leg.action == LOAD:
            # 반시계 90° → 상차 대기 → 반시계로 진입선을 다시 찾음(합계 약 180°)
            steps.append((ROTATE, 1.0, 90.0, 90.0, False))
            steps.append((LOADING,))
            steps.append((ROTATE, 1.0, c.turn_min_deg, c.turn_max_deg, True))
            steps.append(('finish', ARRIVED))
        elif leg.action == PARK:
            steps.append((ROTATE, 1.0, 150.0, 210.0, True))
            steps.append((REVERSE, c.reverse_max_cm))
            steps.append(('finish', ARRIVED))
        elif leg.action == HANDOFF:
            steps.append(('finish', HANDOFF_STATE))
        elif leg.action == STOP:
            steps.append(('finish', ARRIVED))
        else:  # pragma: no cover - plan_route 가 막음
            raise GraphError(f'unknown action {leg.action}')
        if not last and leg.action not in (STRAIGHT, LEFT, RIGHT):
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
        self.state = kind
        self._reset_primitive()
        if kind in (PASS, ADVANCE):
            self._prim = {'cm': st[1]}
        elif kind == ROTATE:
            self._prim = {'dir': st[1], 'min': math.radians(st[2]), 'max': math.radians(st[3]),
                          'acquire': st[4]}
        elif kind == REVERSE:
            self._prim = {'cm': st[1]}
        self._log(kind, '' if kind == LOADING else str(st[1:]))

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
        if self._odo - self._ref_odo >= self._prim['cm']:
            self._next_step()
            return self._dispatch(reading)
        return Command(self.cfg.base_speed, self._hold())

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
        return Command(-c.reverse_speed, self._hold())


__all__ = ['NavConfig', 'Command', 'RouteExecutor', 'GraphError', 'IDLE', 'FOLLOW', 'PASS',
           'ADVANCE', 'ROTATE', 'LOADING', 'REVERSE', 'ARRIVED', 'HANDOFF_STATE', 'STOPPED',
           'ERROR', 'STOP', 'TERMINAL']
