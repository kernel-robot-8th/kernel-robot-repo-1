"""IR 원시값 측정·기록 (ROS 의존성 없음, 단위 테스트 대상).

pinky_sensor_adc main_node 의 ir_sensor/range (std_msgs/UInt16MultiArray) 는
12비트 ADC 원시값 3개를 data[0..2] 로 보낸다(원본 코드: adc 채널 2, 1, 0 순서).
data 순서가 좌·중·우 중 어느 센서인지는 실물에서 센서를 하나씩 가려 확인한다.

기록은 <ir_dir>/<YYYYmmdd_HHMMSS>_<label>.csv (t_sec, ir0, ir1, ir2) 로 저장한다.
"""

import csv
import math
import os
import re
import threading
import time
from collections import deque

DEFAULT_IR_DIR = '~/roport_ir'
CHANNELS = 3
ADC_MAX = 4095
HISTORY_SEC = 10.0
MIN_DURATION, MAX_DURATION = 1.0, 60.0
_LABEL_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
_FILE_RE = re.compile(r'^\d{8}_\d{6}_[A-Za-z0-9_-]{1,40}\.csv$')


class IrRecordError(ValueError):
    pass


def validate_label(label):
    if not isinstance(label, str) or not _LABEL_RE.fullmatch(label):
        raise IrRecordError('label must be 1-40 chars of A-Z a-z 0-9 _ -')
    return label


def validate_filename(name):
    if not isinstance(name, str) or not _FILE_RE.fullmatch(name):
        raise IrRecordError('invalid record file name')
    return name


def channel_stats(rows):
    """rows: [(t, v0, v1, v2), ...] -> 채널별 {mean, std, min, max} 와 개수."""
    out = []
    for c in range(CHANNELS):
        vals = [r[c + 1] for r in rows]
        if not vals:
            out.append(None)
            continue
        mean = sum(vals) / len(vals)
        std = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
        out.append({'mean': round(mean, 1), 'std': round(std, 1),
                    'min': min(vals), 'max': max(vals)})
    return {'n': len(rows), 'channels': out}


def suggest_thresholds(records):
    """label 이 carpet* 인 기록과 tape* 인 기록의 평균으로 채널별 참고 임계값.

    threshold = 두 평균의 중간, separation = |차이| / (std 합) (클수록 구분이 쉽다).
    """
    groups = {'carpet': [], 'tape': []}
    for rec in records:
        for key in groups:
            if rec['label'].startswith(key) and rec['stats']['n'] > 0:
                groups[key].append(rec['stats']['channels'])
    if not groups['carpet'] or not groups['tape']:
        return None
    result = []
    for c in range(CHANNELS):
        def avg(key, field):
            vals = [chs[c][field] for chs in groups[key]]
            return sum(vals) / len(vals)
        carpet, tape = avg('carpet', 'mean'), avg('tape', 'mean')
        spread = avg('carpet', 'std') + avg('tape', 'std')
        result.append({
            'carpet_mean': round(carpet, 1),
            'tape_mean': round(tape, 1),
            'threshold': round((carpet + tape) / 2.0, 1),
            'tape_is_higher': tape > carpet,
            'separation': round(abs(tape - carpet) / spread, 1) if spread > 0 else None,
        })
    return {'carpet_records': len(groups['carpet']), 'tape_records': len(groups['tape']),
            'channels': result}


class IrRecorder:
    """최근 값·주기 표시와 시간 지정 기록. ROS 콜백 스레드와 웹 스레드가 함께 쓴다."""

    def __init__(self, ir_dir=DEFAULT_IR_DIR, clock=time.monotonic):
        self.ir_dir = os.path.abspath(os.path.expanduser(str(ir_dir)))
        self._clock = clock
        self._lock = threading.Lock()
        self._history = deque()        # (t, v0, v1, v2), 최근 HISTORY_SEC
        self._rec = None               # 기록 중 상태
        self._last_result = None
        self._stats_cache = {}         # 파일 이름 -> (mtime, stats)

    # ---------------- 입력 ----------------
    def add_sample(self, values):
        if len(values) < CHANNELS:
            return
        now = self._clock()
        row = (now,) + tuple(int(v) for v in values[:CHANNELS])
        with self._lock:
            self._history.append(row)
            while self._history and now - self._history[0][0] > HISTORY_SEC:
                self._history.popleft()
            if self._rec is not None:
                self._rec['rows'].append(row)
        self.poll()

    # ---------------- 기록 ----------------
    def start(self, label, duration):
        validate_label(label)
        if isinstance(duration, bool) or not isinstance(duration, (int, float)) \
                or not MIN_DURATION <= duration <= MAX_DURATION:
            raise IrRecordError(f'duration must be {MIN_DURATION:g}-{MAX_DURATION:g} s')
        with self._lock:
            if self._rec is not None:
                raise IrRecordError('already recording')
            self._rec = {'label': label, 'duration': float(duration),
                         'start': self._clock(), 'wall': time.localtime(), 'rows': []}
        return self.recording_state()

    def cancel(self):
        with self._lock:
            was = self._rec is not None
            self._rec = None
        return was

    def poll(self):
        """기록 시간이 지났으면 파일로 저장한다. 센서가 멈춰도 끝나도록 웹 조회 때도 부른다."""
        with self._lock:
            rec = self._rec
            if rec is None or self._clock() - rec['start'] < rec['duration']:
                return None
            self._rec = None
        result = self._save(rec)
        with self._lock:
            self._last_result = result
        return result

    def _save(self, rec):
        rows = [r for r in rec['rows'] if r[0] - rec['start'] <= rec['duration']]
        base = {'label': rec['label'], 'duration': rec['duration']}
        if not rows:
            return dict(base, success=False,
                        msg='no samples: is pinky_sensor_adc main_node running?')
        os.makedirs(self.ir_dir, exist_ok=True)
        name = time.strftime('%Y%m%d_%H%M%S', rec['wall']) + f"_{rec['label']}.csv"
        path = os.path.join(self.ir_dir, name)
        with open(path, 'w', newline='', encoding='ascii') as f:
            w = csv.writer(f)
            w.writerow(['t_sec', 'ir0', 'ir1', 'ir2'])
            for r in rows:
                w.writerow([f'{r[0] - rec["start"]:.3f}', r[1], r[2], r[3]])
        return dict(base, success=True, file=name, stats=channel_stats(rows))

    # ---------------- 조회 ----------------
    def recording_state(self):
        with self._lock:
            rec = self._rec
            if rec is None:
                return None
            return {'label': rec['label'], 'duration': rec['duration'],
                    'elapsed': round(self._clock() - rec['start'], 1),
                    'samples': len(rec['rows'])}

    def snapshot(self):
        self.poll()
        now = self._clock()
        with self._lock:
            hist = list(self._history)
            last_result = self._last_result
        latest, age, hz = None, None, None
        if hist:
            latest = list(hist[-1][1:])
            age = round(now - hist[-1][0], 2)
            span = hist[-1][0] - hist[0][0]
            if len(hist) > 1 and span > 0:
                hz = round((len(hist) - 1) / span, 1)
        return {
            'latest': latest,
            'age_sec': age,
            'hz': hz,
            'adc_max': ADC_MAX,
            'history': [[round(r[0] - now, 2)] + list(r[1:]) for r in hist],
            'recording': self.recording_state(),
            'last_result': last_result,
        }

    def list_records(self):
        if not os.path.isdir(self.ir_dir):
            return []
        out = []
        for name in sorted(os.listdir(self.ir_dir), reverse=True):
            if not _FILE_RE.fullmatch(name):
                continue
            path = os.path.join(self.ir_dir, name)
            try:
                mtime = os.path.getmtime(path)
                cached = self._stats_cache.get(name)
                if cached is None or cached[0] != mtime:
                    cached = (mtime, channel_stats(self._read_rows(path)))
                    self._stats_cache[name] = cached
            except (OSError, ValueError):
                continue
            out.append({'file': name, 'label': name[16:-4], 'stats': cached[1]})
        return out

    @staticmethod
    def _read_rows(path):
        with open(path, newline='', encoding='ascii') as f:
            reader = csv.reader(f)
            next(reader, None)
            return [(float(r[0]), int(r[1]), int(r[2]), int(r[3])) for r in reader if r]

    def record_path(self, name):
        return os.path.join(self.ir_dir, validate_filename(name))
