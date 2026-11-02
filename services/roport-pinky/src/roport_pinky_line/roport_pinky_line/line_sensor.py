"""IR 3채널 원시값 → 라인 위치·라인 놓침·노드 판정.

ROS 없이 테스트할 수 있도록 순수 Python으로 둔다.
채널 순서는 ir_sensor/range data[0..2] = 왼쪽·가운데·오른쪽(3대 실측, 2026-10-31).
흰 테이프 위에서 값이 낮아진다(센서를 가리면 4095).
"""

from collections import deque
from dataclasses import dataclass
from statistics import median

CHANNELS = ('left', 'center', 'right')
# 12비트 ADC. 원본 드라이버는 I²C 읽기 실패를 확인하지 않고 그 채널을 0으로 발행한다
# (pinky_sensor_adc main_node.cpp). 테이프 최저 실측값은 258이므로 0은 읽기 실패로 본다.
ADC_MIN_VALID = 1
ADC_MAX = 4095


@dataclass(frozen=True)
class Calibration:
    """기체별 IR 기준값. 순서는 [왼쪽, 가운데, 오른쪽]."""

    carpet: tuple
    tape: tuple
    threshold: tuple
    hysteresis: int = 100

    def __post_init__(self):
        for name in ('carpet', 'tape', 'threshold'):
            values = getattr(self, name)
            if len(values) != 3:
                raise ValueError(f'{name} needs 3 values (left, center, right): {values}')
        if self.hysteresis < 0:
            raise ValueError(f'hysteresis must be >= 0: {self.hysteresis}')
        for i, ch in enumerate(CHANNELS):
            low = self.tape[i] + self.hysteresis
            high = self.carpet[i] - self.hysteresis
            # 값 < 임계값이면 라인. 히스테리시스 띠가 테이프·카펫 값 사이에 있어야 한다.
            if not low < self.threshold[i] < high:
                raise ValueError(
                    f'{ch}: need tape+hyst < threshold < carpet-hyst '
                    f'({self.tape[i]}+{self.hysteresis} < {self.threshold[i]} '
                    f'< {self.carpet[i]}-{self.hysteresis})')


@dataclass(frozen=True)
class LineReading:
    raw: tuple        # 중앙값 필터를 거친 값
    n: tuple          # 정규화 0(카펫)~1(테이프)
    on_line: tuple    # 채널별 라인 위 여부(히스테리시스 적용)
    error: float      # (nR - nL) / (nL + nC + nR). 양수 = 라인이 오른쪽. 놓치면 0.0
    lost: bool        # 어느 채널도 라인 위가 아님
    node: bool        # 세 채널 모두 라인 위(가로 테이프)

    def as_dict(self):
        return {
            'raw': list(self.raw),
            'n': [round(v, 3) for v in self.n],
            'on_line': list(self.on_line),
            'error': round(self.error, 3),
            'lost': self.lost,
            'node': self.node,
        }


def normalize(value, carpet, tape):
    """n = (카펫 - 값) / (카펫 - 테이프), 0~1로 자른다."""
    n = (carpet - value) / float(carpet - tape)
    return min(1.0, max(0.0, n))


class LineSensor:
    """3샘플 중앙값 → 정규화 → 히스테리시스 판정."""

    def __init__(self, calibration, window=3):
        if window < 1:
            raise ValueError(f'window must be >= 1: {window}')
        self.cal = calibration
        self._history = [deque(maxlen=window) for _ in CHANNELS]
        self._on = [False, False, False]

    def reset(self):
        for h in self._history:
            h.clear()
        self._on = [False, False, False]

    def update(self, values):
        if len(values) != 3:
            raise ValueError(f'expected 3 IR values, got {len(values)}')
        values = [int(v) for v in values]
        # 한 채널이라도 범위 밖이면 샘플 전체를 버린다(중앙값 기록에도 넣지 않음)
        if not all(ADC_MIN_VALID <= v <= ADC_MAX for v in values):
            raise ValueError(f'IR value out of range {ADC_MIN_VALID}..{ADC_MAX}: {values}')
        raw = []
        for h, v in zip(self._history, values):
            h.append(v)
            raw.append(median(h))
        cal = self.cal
        for i, v in enumerate(raw):
            # 라인 위에 있으면 임계값+히스 위로 올라가야 벗어남, 밖이면 임계값-히스 아래로 내려가야 진입
            limit = cal.threshold[i] + (cal.hysteresis if self._on[i] else -cal.hysteresis)
            self._on[i] = v < limit
        n = tuple(normalize(v, cal.carpet[i], cal.tape[i]) for i, v in enumerate(raw))
        on_line = tuple(self._on)
        lost = not any(on_line)
        total = sum(n)
        error = 0.0 if lost or total <= 0.0 else (n[2] - n[0]) / total
        return LineReading(
            raw=tuple(raw), n=n, on_line=on_line, error=error,
            lost=lost, node=all(on_line))
