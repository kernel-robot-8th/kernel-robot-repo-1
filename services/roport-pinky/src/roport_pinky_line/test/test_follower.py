import pytest

from roport_pinky_line.follower import (
    FOLLOW, HARD_MAX_ANGULAR, HARD_MAX_LINEAR, IDLE, LINE_LOST, NO_IR, NODE,
    FollowerConfig, LineFollower)
from roport_pinky_line.line_sensor import LineReading

DT = 0.05  # 20 Hz


def reading(error=0.0, lost=False, node=False):
    on = (True, True, True) if node else (False, not lost, False)
    return LineReading(raw=(0, 0, 0), n=(0.0, 1.0, 0.0), on_line=on,
                       error=error, lost=lost, node=node)


def enabled(**kw):
    f = LineFollower(FollowerConfig(**kw))
    f.set_enabled(True)
    return f


def test_disabled_publishes_nothing():
    f = LineFollower()
    assert f.step(reading(), DT) is None
    assert f.ir_timeout() is None
    assert f.state == IDLE


def test_straight_line():
    cmd = enabled().step(reading(0.0), DT)
    assert cmd.state == FOLLOW
    assert cmd.linear == pytest.approx(0.05)
    assert cmd.angular == pytest.approx(0.0)


def test_line_right_turns_right_and_slows():
    f = enabled(kd=0.0)
    cmd = f.step(reading(0.4), DT)
    assert cmd.angular < 0          # 반시계 + 이므로 오른쪽은 -
    assert cmd.angular == pytest.approx(-0.4)
    assert 0.02 <= cmd.linear < 0.05
    assert enabled(kd=0.0).step(reading(-0.4), DT).angular > 0


def test_limits_are_enforced():
    f = enabled(kp=10.0, kd=0.0, max_angular=0.6, base_speed=0.1, max_linear=0.1)
    cmd = f.step(reading(1.0), DT)
    assert abs(cmd.angular) == pytest.approx(0.6)
    assert cmd.linear <= 0.1
    with pytest.raises(ValueError):
        FollowerConfig(max_linear=HARD_MAX_LINEAR + 0.01)
    with pytest.raises(ValueError):
        FollowerConfig(max_angular=HARD_MAX_ANGULAR + 0.1)
    with pytest.raises(ValueError):
        FollowerConfig(base_speed=0.2, max_linear=0.1)


def test_derivative_uses_error_change():
    f = enabled(kp=0.0, kd=0.1)
    assert f.step(reading(0.0), DT).angular == pytest.approx(0.0)
    assert f.step(reading(0.1), DT).angular == pytest.approx(-0.1 * 0.1 / DT)


def test_line_lost_stops_and_resumes():
    f = enabled()
    cmd = f.step(reading(lost=True), DT)
    assert (cmd.linear, cmd.angular, cmd.state) == (0.0, 0.0, LINE_LOST)
    assert f.step(reading(0.0), DT).state == FOLLOW


def test_node_stops_after_confirm_and_stays():
    f = enabled(node_confirm=2)
    f.step(reading(0.0), DT)
    assert f.step(reading(node=True), DT).state == FOLLOW   # 1샘플은 아직
    cmd = f.step(reading(node=True), DT)
    assert (cmd.linear, cmd.angular, cmd.state) == (0.0, 0.0, NODE)
    # 노드를 벗어나도 다시 켜기 전까지 정지 유지
    assert f.step(reading(0.0), DT).state == NODE


def test_reenable_passes_current_node_then_detects_next():
    f = enabled(node_confirm=2)
    f.step(reading(0.0), DT)
    for _ in range(3):
        f.step(reading(node=True), DT)
    assert f.state == NODE
    f.set_enabled(True)
    for _ in range(5):                                       # 밟고 있는 노드는 지나감
        assert f.step(reading(node=True), DT).state == FOLLOW
    f.step(reading(0.0), DT)
    f.step(reading(node=True), DT)
    assert f.step(reading(node=True), DT).state == NODE      # 다음 노드


def test_enabled_on_node_drives_off_it():
    # 대기 칸(가로 테이프) 위에서 켜면 그 노드에서 서지 않고 출발
    f = enabled(node_confirm=2)
    for _ in range(5):
        assert f.step(reading(node=True), DT).state == FOLLOW


def test_stop_at_node_off_keeps_following():
    f = enabled(stop_at_node=False)
    for _ in range(5):
        assert f.step(reading(node=True), DT).state == FOLLOW


def test_ir_timeout_stops():
    f = enabled()
    cmd = f.ir_timeout()
    assert (cmd.linear, cmd.angular, cmd.state) == (0.0, 0.0, NO_IR)


def test_disable_goes_idle():
    f = enabled()
    f.set_enabled(False)
    assert f.state == IDLE
    assert f.step(reading(), DT) is None
