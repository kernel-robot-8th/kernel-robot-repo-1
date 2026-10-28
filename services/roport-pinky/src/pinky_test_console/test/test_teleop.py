import math

import pytest

from pinky_test_console.teleop import (
    DEADMAN_TIMEOUT,
    MAX_ANGULAR,
    MAX_LINEAR,
    ZERO_REPEAT,
    TeleopGuard,
    clamp,
)


class FakeClock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def test_required_limits():
    assert MAX_LINEAR == pytest.approx(0.10)
    assert MAX_ANGULAR == pytest.approx(0.6)
    assert DEADMAN_TIMEOUT == pytest.approx(0.3)


@pytest.mark.parametrize('value, limit, expected', [
    (0.05, 0.1, 0.05),
    (0.5, 0.1, 0.1),
    (-0.5, 0.1, -0.1),
    (10, 0.6, 0.6),
    (-1e9, 0.6, -0.6),
    (float('nan'), 0.1, 0.0),
    (float('inf'), 0.1, 0.0),
    (float('-inf'), 0.1, 0.0),
    ('0.5', 0.1, 0.1),       # 숫자 문자열은 숫자로 보고 자른다
    ('abc', 0.1, 0.0),
    (None, 0.1, 0.0),
    (True, 0.1, 0.0),        # bool 은 숫자로 취급하지 않음
    ([1], 0.1, 0.0),
])
def test_clamp(value, limit, expected):
    assert clamp(value, limit) == pytest.approx(expected)


def test_update_is_clamped_by_server():
    g = TeleopGuard(clock=FakeClock())
    assert g.update(1.0, 5.0) == (pytest.approx(MAX_LINEAR), pytest.approx(MAX_ANGULAR))
    assert g.update(-1.0, -5.0) == (pytest.approx(-MAX_LINEAR), pytest.approx(-MAX_ANGULAR))
    assert g.update(float('nan'), 0.3) == (0.0, pytest.approx(0.3))
    lin, ang = g.next_command()
    assert abs(lin) <= MAX_LINEAR and abs(ang) <= MAX_ANGULAR


def test_idle_publishes_nothing():
    """입력이 없으면 아무것도 발행하지 않는다 (Nav2 의 /cmd_vel 을 가리지 않음)."""
    g = TeleopGuard(clock=FakeClock())
    assert g.next_command() is None
    assert g.expire() is False


def test_deadman_expires_after_timeout():
    clk = FakeClock()
    g = TeleopGuard(clock=clk)
    g.update(0.1, 0.0)
    clk.advance(0.29)
    assert g.expire() is False
    assert g.next_command() == (pytest.approx(0.1), 0.0)
    clk.advance(0.02)  # 마지막 입력 후 0.31 s
    assert g.expire() is True       # 호출 측이 즉시 0 발행
    assert g.expire() is False      # 한 번만
    assert not g.active
    # 이후 ZERO_REPEAT 번 0, 그 다음 조용
    for _ in range(ZERO_REPEAT):
        assert g.next_command() == (0.0, 0.0)
    assert g.next_command() is None


def test_next_command_never_sends_stale_command():
    """watchdog 보다 publish 타이머가 먼저 불려도 오래된 비영 명령을 보내지 않는다."""
    clk = FakeClock()
    g = TeleopGuard(clock=clk)
    g.update(0.1, 0.6)
    clk.advance(0.5)
    assert g.next_command() == (0.0, 0.0)
    assert not g.active


def test_continuous_input_keeps_driving():
    clk = FakeClock()
    g = TeleopGuard(clock=clk)
    for _ in range(20):  # 브라우저가 10 Hz 로 보냄
        g.update(0.08, 0.0)
        clk.advance(0.1)
        assert g.expire() is False
        assert g.next_command() == (pytest.approx(0.08), 0.0)


def test_stop_sends_zero_burst():
    clk = FakeClock()
    g = TeleopGuard(clock=clk)
    g.update(0.1, 0.0)
    assert g.stop() is True
    assert g.stop() is False  # 두 번 불러도 안전 (이미 정지)
    zeros = [g.next_command() for _ in range(ZERO_REPEAT)]
    assert zeros == [(0.0, 0.0)] * ZERO_REPEAT
    assert g.next_command() is None


def test_stop_when_idle_reports_false():
    g = TeleopGuard(clock=FakeClock())
    assert g.stop() is False


def test_timeline_10hz_publish_and_20ms_watchdog():
    """실제 타이머 배치(10 Hz 발행 + 50 Hz watchdog)를 가짜 시계로 재현.

    마지막 입력 후 timeout + watchdog 주기(0.32 s) 안에 0 이 나가고,
    그 이후 비영 명령은 하나도 없어야 한다.
    """
    clk = FakeClock(0.0)
    g = TeleopGuard(clock=clk)
    published = []  # (t, lin, ang)
    last_input = 1.0
    dt = 0.01
    steps = int(3.0 / dt)
    for i in range(steps):
        t = round(i * dt, 6)
        clk.t = t
        if t < last_input + 1e-9 and i % 10 == 0:   # 웹 입력 10 Hz, 1.0 s 까지
            g.update(1.0, -9.0)
        if i % 2 == 0 and g.expire():               # watchdog 20 ms
            published.append((t, 0.0, 0.0))
        if i % 10 == 5:                             # publish 타이머 10 Hz
            cmd = g.next_command()
            if cmd is not None:
                published.append((t, *cmd))

    nonzero = [p for p in published if p[1] != 0.0 or p[2] != 0.0]
    zeros = [p for p in published if p[1] == 0.0 and p[2] == 0.0]
    assert nonzero, 'should drive while input arrives'
    assert all(abs(p[1]) <= MAX_LINEAR + 1e-12 and abs(p[2]) <= MAX_ANGULAR + 1e-12
               for p in nonzero)
    first_zero = zeros[0][0]
    assert last_input + DEADMAN_TIMEOUT - 1e-9 <= first_zero <= last_input + DEADMAN_TIMEOUT + 0.021
    assert all(p[0] <= last_input + DEADMAN_TIMEOUT for p in nonzero)
    assert published[-1][0] < 2.0, 'must go quiet after the zero burst'
    # 발행 간격 ≈ 0.1 s (10 Hz)
    times = [p[0] for p in nonzero]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert all(math.isclose(gap, 0.1, abs_tol=1e-6) for gap in gaps)


def test_invalid_construction():
    with pytest.raises(ValueError):
        TeleopGuard(max_linear=0)
    with pytest.raises(ValueError):
        TeleopGuard(timeout=-1)
