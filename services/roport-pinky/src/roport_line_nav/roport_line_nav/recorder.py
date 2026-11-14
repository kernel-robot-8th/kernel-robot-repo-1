"""경로마다 상세 기록 CSV(약 20 Hz). 순수 Python.

파일: <log_dir>/route_<YYYYmmdd_HHMMSS>_<출발>-<도착>.csv, 최근 keep 개만 남긴다(2026-11-04 결정: 50개).
쓰기에 실패하면 기록만 끄고 주행에는 영향을 주지 않는다.
"""

import csv
import datetime
import os
import re

FIELDS = ('t', 'state', 'index', 'leg_from', 'leg_to', 'traveled_cm', 'rotated_deg',
          'ir0', 'ir1', 'ir2', 'on0', 'on1', 'on2', 'error', 'cmd_linear', 'cmd_angular',
          'odom_x_cm', 'odom_y_cm', 'odom_yaw_deg', 'events', 'rv_y_cm', 'rv_theta_deg')
FILE_RE = re.compile(r'^route_\d{8}_\d{6}_[A-Z0-9]{1,8}-[A-Z0-9]{1,8}\.csv$')


class RouteRecorder:
    def __init__(self, log_dir, keep=50):
        self.log_dir = os.path.abspath(os.path.expanduser(log_dir))
        self.keep = int(keep)
        self.path = None
        self.error = None
        self._f = None
        self._w = None
        self._t0 = None
        self._events_seen = 0

    @property
    def active(self):
        return self._f is not None

    def open(self, route, now, wall=None):
        self.close()
        if self.error:
            return None
        stamp = (wall or datetime.datetime.now()).strftime('%Y%m%d_%H%M%S')
        name = f'route_{stamp}_{route[0]}-{route[-1]}.csv'
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            self.path = os.path.join(self.log_dir, name)
            self._f = open(self.path, 'w', encoding='utf-8', newline='')
            self._w = csv.writer(self._f)
            self._w.writerow(FIELDS)
        except OSError as e:
            self.error = f'route log disabled: {e}'
            self._f = self._w = None
            return None
        self._t0 = now
        self._events_seen = None
        return self.path

    def write(self, now, state, reading, cmd, odom):
        """state = RouteExecutor.as_dict(). 새로 생긴 이벤트는 events 칸에 이어 적는다."""
        if self._w is None:
            return
        events = state.get('events') or []
        seq = state.get('event_seq', len(events))
        if self._events_seen is None:          # 이 파일의 첫 줄: 경로 시작 이벤트부터
            self._events_seen = state.get('route_event_seq0', seq)
        fresh = max(0, seq - self._events_seen)
        new = events[-fresh:] if fresh else []
        self._events_seen = seq
        leg = state.get('leg') or {}
        rv = state.get('reverse') or {}       # 라인 후진(FFT1-172) 중 ŷ·θ̂, 아니면 빈 칸
        raw = reading.raw if reading is not None else (None, None, None)
        on = reading.on_line if reading is not None else (None, None, None)
        od = (None, None, None) if odom is None else (
            round(odom[0], 1), round(odom[1], 1), round(odom[2] * 57.29578, 1))
        row = [round(now - self._t0, 3), state.get('state'), state.get('index'),
               leg.get('from'), leg.get('to'), state.get('traveled_cm'), state.get('rotated_deg'),
               *raw, *(int(v) if v is not None else None for v in on),
               None if reading is None else round(reading.error, 3),
               None if cmd is None else round(cmd[0], 3),
               None if cmd is None else round(cmd[1], 3),
               *od,
               ' | '.join(f"{e.get('event')}:{e.get('detail', '')}" for e in new),
               rv.get('y_cm'), rv.get('theta_deg')]
        try:
            self._w.writerow(row)
        except OSError as e:
            self.error = f'route log disabled: {e}'
            self.close()

    def close(self):
        if self._f is None:
            return
        try:
            self._f.close()
        except OSError:
            pass
        self._f = self._w = None
        self.prune()

    def prune(self):
        try:
            names = sorted(n for n in os.listdir(self.log_dir) if FILE_RE.match(n))
        except OSError:
            return
        for n in names[:-self.keep] if self.keep > 0 else []:
            try:
                os.remove(os.path.join(self.log_dir, n))
            except OSError:
                pass

    def files(self):
        try:
            names = sorted((n for n in os.listdir(self.log_dir) if FILE_RE.match(n)),
                           reverse=True)
        except OSError:
            return []
        out = []
        for n in names:
            st = os.stat(os.path.join(self.log_dir, n))
            out.append({'name': n, 'size': st.st_size, 'mtime': round(st.st_mtime)})
        return out
