"""노트북 인식 도구의 탑승 시간 알림 발행(boarding_alert.py). ROS 없이 발행 함수만 본다."""
import pytest

from roport_follow.boarding_alert import BoardingAlert


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(max_s=60.0):
    sent, clock = [], Clock()
    return BoardingAlert(sent.append, period_s=0.5, max_s=max_s, clock=clock), sent, clock


def test_on_publishes_true_now_and_on_each_tick_then_false_once_on_off():
    alert, sent, _ = make()
    assert alert.set(True) is True
    alert.tick()
    alert.tick()
    assert sent == [True, True, True]
    assert alert.set(False) is False
    alert.tick()
    assert sent == [True, True, True, False]


def test_off_without_on_publishes_nothing():
    alert, sent, _ = make()
    assert alert.set(False) is False
    alert.tick()
    assert sent == []


def test_turns_itself_off_after_max_time():
    alert, sent, clock = make(max_s=60.0)
    alert.set(True)
    clock.t += 59.9
    alert.tick()
    assert sent[-1] is True and alert.on
    clock.t += 0.2
    alert.tick()
    assert sent[-1] is False and not alert.on
    alert.tick()
    assert sent.count(False) == 1


def test_turning_on_again_restarts_the_max_time():
    alert, sent, clock = make(max_s=60.0)
    alert.set(True)
    clock.t += 50
    alert.set(True)
    clock.t += 50
    alert.tick()
    assert alert.on and sent[-1] is True


@pytest.mark.parametrize('value', [1, 0, 'true', None, {}])
def test_rejects_non_bool(value):
    alert, sent, _ = make()
    with pytest.raises(ValueError, match='invalid_alert'):
        alert.set(value)
    assert sent == []


def test_close_turns_off_and_is_safe_without_thread():
    alert, sent, _ = make()
    alert.set(True)
    alert.close()
    assert sent == [True, False] and not alert.on


def test_thread_republishes_and_close_stops_it():
    sent = []
    alert = BoardingAlert(sent.append, period_s=0.02, max_s=60.0).start()
    alert.set(True)
    import time
    time.sleep(0.15)
    alert.close()
    assert sent[0] is True and sent[-1] is False and sent.count(True) >= 3
    n = len(sent)
    time.sleep(0.08)
    assert len(sent) == n
