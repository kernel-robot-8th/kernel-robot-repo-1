"""Read-only sensor evidence and explicit initial-position preconditions."""
import math
import time

from .amcl_check import AmclCheckError


class SmartcartMonitor:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.samples = {}
        self.stop_since = None

    def observe(self, name, message, *, stopped=None):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        previous = self.samples.get(name)
        if not math.isfinite(stamp) or (previous and stamp <= previous['stamp']):
            return
        now = self.clock()
        if name == 'odom':
            if not stopped or (previous and now - previous['received'] > 1.):
                self.stop_since = None
            elif self.stop_since is None:
                self.stop_since = now
        self.samples[name] = {'stamp': stamp, 'received': now}

    def snapshot(self, ros_now):
        now = self.clock()
        result = {}
        for name in ('scan', 'odom', 'tf', 'amcl'):
            sample = self.samples.get(name)
            age = None if sample is None else now - sample['received']
            stamp_age = None if sample is None else ros_now - sample['stamp']
            received_limit, stamp_limit = (5., 5.) if name == 'amcl' else (1., 2.)
            result[name] = {'fresh': age is not None and age <= received_limit and -.5 <= stamp_age <= stamp_limit,
                            'age_sec': None if age is None else round(age, 2),
                            'stamp_age_sec': None if stamp_age is None else round(stamp_age, 2)}
        result['stopped'] = (result['odom']['fresh'] and self.stop_since is not None
                             and now - self.stop_since >= .5)
        return result

    def require_initial_pose(self, ros_now, node_id, yaw_deg, confirmed, navigating):
        if node_id not in ('H1', 'H2'):
            raise AmclCheckError('초기 위치는 H1 또는 H2만 선택하세요')
        if type(yaw_deg) not in (int, float) or not math.isfinite(yaw_deg) or not -180 <= yaw_deg <= 180:
            raise AmclCheckError('실제 바라보는 방향을 -180~180도로 직접 입력하세요')
        status = self.snapshot(ros_now)
        if confirmed is not True or navigating or not status['scan']['fresh'] or not status['stopped']:
            raise AmclCheckError('로봇 배치 확인·최신 scan/odom·연속 정지·Nav2 비주행 상태가 필요합니다')
