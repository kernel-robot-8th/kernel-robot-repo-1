import pytest

from roport_line_nav.cmd_vel_guard import GuardLogic, ZERO_BURST


def test_relays_and_clamps():
    g = GuardLogic(0.3)
    assert g.relay(0.0, 0.03, -0.2) == (0.03, -0.2)
    assert g.relay(0.05, 9.0, -9.0) == (0.15, -1.5)          # 코드 상한
    assert g.relay(0.1, float('nan'), 0.1) == (0.0, 0.1)


def test_zero_after_silence_while_moving():
    g = GuardLogic(0.3)
    g.relay(0.0, 0.03, 0.0)
    assert not g.tick(0.2)
    sent = [g.tick(t / 100) for t in range(31, 60)]
    assert sum(sent) == ZERO_BURST and g.trips == 1
    assert not g.tick(5.0)                                    # 한 번만


def test_no_zero_when_last_command_was_zero_or_fresh():
    g = GuardLogic(0.3)
    assert not g.tick(10.0)                                   # 명령을 받은 적 없음
    g.relay(0.0, 0.0, 0.0)
    assert not g.tick(10.0)                                   # 이미 정지 명령
    for i in range(100):                                      # 20 Hz 정상 주행
        g.relay(i * 0.05, 0.03, 0.0)
        assert not g.tick(i * 0.05 + 0.02)


def test_new_command_cancels_zero_burst():
    g = GuardLogic(0.3)
    g.relay(0.0, 0.03, 0.0)
    assert g.tick(0.5)
    g.relay(0.55, 0.03, 0.0)
    assert not g.tick(0.6)


@pytest.mark.parametrize('bad', [0.0, 0.01, 5.0])
def test_timeout_range(bad):
    with pytest.raises(ValueError):
        GuardLogic(bad)
