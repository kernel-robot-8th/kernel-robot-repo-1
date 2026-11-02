from pathlib import Path

import pytest

from roport_pinky_line.calibration import load_calibration
from roport_pinky_line.line_sensor import Calibration, LineSensor, normalize

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'robots.yaml'
# pinky_d660 실측값(2겹)
CAL = Calibration(carpet=(2308, 2376, 2233), tape=(299, 388, 281),
                  threshold=(1300, 1380, 1260))
CARPET = CAL.carpet
TAPE = CAL.tape


def feed(sensor, values, times=3):
    reading = None
    for _ in range(times):
        reading = sensor.update(values)
    return reading


def test_normalize_direction_and_clip():
    assert normalize(2308, 2308, 299) == 0.0
    assert normalize(299, 2308, 299) == 1.0
    assert normalize(4095, 2308, 299) == 0.0   # 가림·들림 → 카펫 쪽
    assert normalize(100, 2308, 299) == 1.0
    assert normalize(1303, 2308, 299) == pytest.approx(0.5, abs=0.01)


def test_all_robot_calibrations_load_and_are_valid():
    for robot in ('pinky_764e', 'pinky_6422', 'pinky_d660'):
        cal = load_calibration(CONFIG, robot)
        assert cal.hysteresis == 100
    assert load_calibration(CONFIG, 'pinky_d660') == CAL


def test_unknown_robot_lists_choices():
    with pytest.raises(KeyError, match='pinky_d660'):
        load_calibration(CONFIG, 'pinky_7642')


def test_threshold_outside_band_rejected():
    with pytest.raises(ValueError, match='left'):
        Calibration(carpet=(2000, 2000, 2000), tape=(300, 300, 300),
                    threshold=(350, 1000, 1000))


def test_center_on_line_zero_error():
    r = feed(LineSensor(CAL), (CARPET[0], TAPE[1], CARPET[2]))
    assert r.on_line == (False, True, False)
    assert r.error == pytest.approx(0.0)
    assert not r.lost and not r.node


def test_line_to_the_right_is_positive():
    r = feed(LineSensor(CAL), (CARPET[0], 900, TAPE[2]))
    assert r.on_line == (False, True, True)
    assert r.error > 0.3
    r = feed(LineSensor(CAL), (TAPE[0], 900, CARPET[2]))
    assert r.error < -0.3


def test_lost_and_node():
    assert feed(LineSensor(CAL), CARPET).lost
    assert feed(LineSensor(CAL), (4095, 4095, 4095)).lost
    r = feed(LineSensor(CAL), TAPE)
    assert r.node and not r.lost


def test_median_filters_single_spike():
    s = LineSensor(CAL)
    feed(s, (CARPET[0], TAPE[1], CARPET[2]))
    r = s.update((CARPET[0], 3600, CARPET[2]))   # pinky_6422 에서 본 단발 튐
    assert r.on_line[1] and not r.lost


def test_hysteresis_band():
    s = LineSensor(CAL)
    thr = CAL.threshold[1]
    # 임계값 바로 아래(히스 띠 안)로는 라인 진입하지 않음
    assert not feed(s, (CARPET[0], thr - 50, CARPET[2])).on_line[1]
    assert feed(s, (CARPET[0], thr - 150, CARPET[2])).on_line[1]
    # 라인 위에서는 임계값 바로 위(히스 띠 안)까지 유지
    assert feed(s, (CARPET[0], thr + 50, CARPET[2])).on_line[1]
    assert not feed(s, (CARPET[0], thr + 150, CARPET[2])).on_line[1]


def test_short_message_rejected():
    with pytest.raises(ValueError):
        LineSensor(CAL).update([100, 200])
    with pytest.raises(ValueError):
        LineSensor(CAL).update([100, 200, 300, 400])


@pytest.mark.parametrize('values', [
    (0, 0, 0),                   # 원본 드라이버의 I²C 읽기 실패(버퍼 0 그대로 발행)
    (CARPET[0], 0, CARPET[2]),   # 한 채널만 실패
    (CARPET[0], 4096, CARPET[2]),
])
def test_read_failure_values_rejected_without_touching_history(values):
    s = LineSensor(CAL)
    feed(s, (CARPET[0], TAPE[1], CARPET[2]))
    with pytest.raises(ValueError, match='out of range'):
        s.update(values)
    # 버린 샘플은 중앙값 기록에 남지 않는다
    r = s.update((CARPET[0], TAPE[1], CARPET[2]))
    assert r.raw == (CARPET[0], TAPE[1], CARPET[2])
    assert r.on_line == (False, True, False)
