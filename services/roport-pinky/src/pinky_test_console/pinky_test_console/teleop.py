"""Teleop 속도 제한 + deadman 로직 (ROS 의존성 없음, 단위 테스트 대상).

단위: 선속도 m/s, 각속도 rad/s, 시간 s.

동작 규칙
- update(): 웹 입력을 받을 때마다 호출. 값을 서버 상한으로 자르고 입력 시각을 기록한다.
- 입력이 timeout(기본 0.3 s) 동안 없으면 expire()가 True를 돌려준다.
  호출 측은 즉시 속도 0을 발행해야 한다.
- 정지 후에도 zero_repeat 횟수만큼 속도 0을 더 발행한다(메시지 유실 대비).
  pinky_bringup은 cmd_vel 타임아웃이 없어 마지막 명령을 계속 유지하기 때문이다.
- 비활성 상태에서는 아무것도 발행하지 않는다. Nav2(velocity_smoother)도 /cmd_vel에
  발행하므로, 콘솔이 계속 0을 보내면 Nav2 주행을 막는다.
"""

import math
import threading
import time

MAX_LINEAR = 0.10    # m/s
MAX_ANGULAR = 0.6    # rad/s
DEADMAN_TIMEOUT = 0.3  # s
ZERO_REPEAT = 3      # 정지 후 추가로 보낼 0 메시지 수 (10 Hz 기준 0.3 s)


def clamp(value, limit):
    """value를 [-limit, limit]로 자른다. 숫자가 아니거나 NaN/inf면 0."""
    if isinstance(value, bool):
        return 0.0
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(v):
        return 0.0
    limit = abs(float(limit))
    return max(-limit, min(limit, v))


class TeleopGuard:
    """스레드 안전한 teleop 상태 머신. Flask 스레드와 ROS 타이머가 함께 사용한다."""

    def __init__(self, max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
                 timeout=DEADMAN_TIMEOUT, zero_repeat=ZERO_REPEAT,
                 clock=time.monotonic):
        if max_linear <= 0 or max_angular <= 0 or timeout <= 0:
            raise ValueError('limits and timeout must be positive')
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.timeout = float(timeout)
        self.zero_repeat = int(zero_repeat)
        self._clock = clock
        self._lock = threading.Lock()
        self._active = False
        self._cmd = (0.0, 0.0)
        self._last_input = None
        self._zeros_left = 0

    # ---- 웹 입력 쪽 ----
    def update(self, linear, angular):
        """웹 입력 반영. 잘린 (linear, angular)를 돌려준다."""
        cmd = (clamp(linear, self.max_linear), clamp(angular, self.max_angular))
        with self._lock:
            self._cmd = cmd
            self._last_input = self._clock()
            self._active = True
            self._zeros_left = 0
        return cmd

    def stop(self):
        """즉시 정지 요청. 직전에 움직이는 중이었으면 True."""
        with self._lock:
            was_active = self._active
            self._active = False
            self._cmd = (0.0, 0.0)
            self._zeros_left = self.zero_repeat
        return was_active

    # ---- ROS 타이머 쪽 ----
    def expire(self):
        """deadman 검사. 방금 만료됐으면 True (호출 측이 속도 0 발행)."""
        with self._lock:
            if not self._active:
                return False
            if self._clock() - self._last_input <= self.timeout:
                return False
            self._active = False
            self._cmd = (0.0, 0.0)
            self._zeros_left = self.zero_repeat
            return True

    def next_command(self):
        """10 Hz 발행 타이머가 호출. 발행할 (linear, angular) 또는 None(발행 안 함)."""
        with self._lock:
            if self._active:
                if self._clock() - self._last_input > self.timeout:
                    # watchdog보다 먼저 불려도 오래된 명령은 절대 내보내지 않는다.
                    self._active = False
                    self._cmd = (0.0, 0.0)
                    self._zeros_left = self.zero_repeat
                else:
                    return self._cmd
            if self._zeros_left > 0:
                self._zeros_left -= 1
                return (0.0, 0.0)
            return None

    @property
    def active(self):
        with self._lock:
            return self._active

    def snapshot(self):
        with self._lock:
            return {
                'active': self._active,
                'linear': self._cmd[0],
                'angular': self._cmd[1],
            }
