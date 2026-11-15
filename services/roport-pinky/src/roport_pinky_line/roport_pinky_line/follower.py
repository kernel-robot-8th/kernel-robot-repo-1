"""라인 위치 → 속도 명령(PD). 순수 Python.

단위: 선속도 m/s, 각속도 rad/s(ROS 기준 반시계 +).
오차가 양수면 라인이 오른쪽에 있으므로 오른쪽(각속도 -)으로 돈다.
라인 놓침·노드·IR 끊김은 모두 정지(0) 쪽으로 처리한다.
"""

from dataclasses import dataclass

# 설정 파일로도 넘을 수 없는 상한. 노드 안에서 강제한다.
HARD_MAX_LINEAR = 0.15
HARD_MAX_ANGULAR = 1.5

IDLE = 'idle'
FOLLOW = 'follow'
LINE_LOST = 'line_lost'
NODE = 'node'
NO_IR = 'no_ir'


@dataclass(frozen=True)
class FollowerConfig:
    base_speed: float = 0.05    # m/s, 오차 0일 때
    min_speed: float = 0.02     # m/s, 오차가 커도 이 아래로는 줄이지 않음
    max_linear: float = 0.10    # m/s
    max_angular: float = 0.6    # rad/s
    kp: float = 1.0             # rad/s per 오차 1
    kd: float = 0.05            # rad/s per (오차/s)
    slow_gain: float = 0.5      # 선속도 = base * (1 - slow_gain*|오차|)
    stop_at_node: bool = True
    node_confirm: int = 2       # 연속 샘플 수

    def __post_init__(self):
        if not 0.0 < self.max_linear <= HARD_MAX_LINEAR:
            raise ValueError(
                f'max_linear must be in (0, {HARD_MAX_LINEAR}] m/s: {self.max_linear}')
        if not 0.0 < self.max_angular <= HARD_MAX_ANGULAR:
            raise ValueError(
                f'max_angular must be in (0, {HARD_MAX_ANGULAR}] rad/s: {self.max_angular}')
        if not 0.0 <= self.min_speed <= self.base_speed <= self.max_linear:
            raise ValueError(
                'need 0 <= min_speed <= base_speed <= max_linear: '
                f'{self.min_speed}, {self.base_speed}, {self.max_linear}')
        if self.kp < 0 or self.kd < 0 or not 0.0 <= self.slow_gain <= 1.0:
            raise ValueError('kp, kd must be >= 0 and slow_gain in [0, 1]')
        if self.node_confirm < 1:
            raise ValueError(f'node_confirm must be >= 1: {self.node_confirm}')


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float
    state: str


def clamp(value, limit):
    return max(-limit, min(limit, value))


class LineFollower:
    def __init__(self, config=None):
        self.cfg = config or FollowerConfig()
        self.enabled = False
        self.state = IDLE
        self._prev_error = None
        self._node_count = 0
        self._node_latched = False
        self._passing_node = False

    def set_enabled(self, on):
        """켜면 노드 정지를 풀고, 지금 밟고 있는 노드는 벗어날 때까지 무시한다."""
        self.enabled = bool(on)
        self._prev_error = None
        self._node_count = 0
        self._node_latched = False
        self._passing_node = self.enabled
        self.state = FOLLOW if self.enabled else IDLE

    def _stop(self, state):
        self.state = state
        self._prev_error = None
        return Command(0.0, 0.0, state)

    def step(self, reading, dt):
        """IR 한 샘플마다 호출. 꺼져 있으면 None(발행하지 않음)."""
        if not self.enabled:
            self.state = IDLE
            return None

        if reading.node:
            self._node_count += 1
        else:
            self._node_count = 0
            self._passing_node = False
        if (self.cfg.stop_at_node and not self._passing_node
                and self._node_count >= self.cfg.node_confirm):
            self._node_latched = True
        if self._node_latched:
            return self._stop(NODE)

        if reading.lost:
            return self._stop(LINE_LOST)

        e = reading.error
        de = 0.0
        if self._prev_error is not None and dt > 0.0:
            de = (e - self._prev_error) / dt
        self._prev_error = e
        cfg = self.cfg
        angular = clamp(-(cfg.kp * e + cfg.kd * de), cfg.max_angular)
        linear = cfg.base_speed * (1.0 - cfg.slow_gain * abs(e))
        linear = min(cfg.max_linear, max(cfg.min_speed, linear))
        self.state = FOLLOW
        return Command(linear, angular, FOLLOW)

    def ir_timeout(self):
        """IR 입력이 끊겼을 때. 꺼져 있으면 None."""
        if not self.enabled:
            return None
        return self._stop(NO_IR)
