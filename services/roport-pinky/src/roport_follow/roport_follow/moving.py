"""움직이는 인형: 영상 사이 추적 → 속도 → 앞으로 갈 자리를 장애물 점으로 더함(ROS 없음, 순수 계산). FFT1-190.

- 추적은 odom 좌표(로봇 바퀴 기준 위치를 시간에 따라 쌓은 좌표)에서 한다. 로봇이 움직여도 인형 속도에
  로봇 움직임이 섞이지 않게 한다. odom 자세(pose)가 없으면 예측하지 않는다(멈춘 인형과 같은 지금 동작).
- 연결: 같은 종류(adult/child)·가장 가까운 트랙, 거리 ≤ gate_m + max_speed × 경과 시간.
- 속도: 위치 차이의 지수 평균. min_hits 번 이상 보이고 min_speed 이상일 때만 「움직임」.
- 예측: 지금 표시 점을 속도 × t(step 간격, horizon 까지)만큼 옮긴 점. 로봇 쪽으로 오는 인형은 Nav2 가 미리 비켜 감.
  가까이 다가오는 인형에서 감속·정지는 Nav2 Collision Monitor 등 로봇 쪽 기능으로(여기서는 시간 여유 ttc 만 계산해 표시).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class MovingConfig:
    gate_m: float = 0.25            # 추적 연결 기본 반경
    max_speed_mps: float = 0.5      # 이보다 빠른 연결은 다른 물체로 봄(태엽 인형은 수 cm/s)
    min_speed_mps: float = 0.03     # 이보다 느리면 멈춘 인형
    min_hits: int = 3
    smooth: float = 0.5             # 속도 지수 평균
    horizon_s: float = 1.0          # 이 시간 뒤까지 예상 자리 표시
    step_s: float = 0.1
    drop_after_s: float = 0.5       # 이 시간 못 보면 트랙 지움

    def validate(self):
        if not (0.05 <= self.gate_m <= 1.0):
            raise ValueError('gate_m must be 0.05..1')
        if not (0.0 < self.min_speed_mps < self.max_speed_mps <= 3.0):
            raise ValueError('need 0 < min_speed < max_speed <= 3')
        if not (0.0 < self.horizon_s <= 3.0 and 0.02 <= self.step_s <= self.horizon_s):
            raise ValueError('need 0 < horizon <= 3 and 0.02 <= step <= horizon')
        if not (0.0 < self.smooth <= 1.0) or self.min_hits < 2:
            raise ValueError('smooth in (0,1], min_hits >= 2')


@dataclass
class Track:
    id: int
    name: str
    x: float                        # odom 좌표, 인형 가까운 끝 가운데
    y: float
    t: float
    vx: float = 0.0
    vy: float = 0.0
    hits: int = 1
    points_odom: tuple = field(default=(), repr=False)

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


def to_odom(pose, x, y):
    """base_footprint 점 → odom. pose = (x, y, yaw) of base_footprint in odom."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * x - s * y, py + s * x + c * y


def to_base(pose, x, y):
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - px, y - py
    return c * dx + s * dy, -s * dx + c * dy


class MovingTracker:
    def __init__(self, cfg: MovingConfig):
        cfg.validate()
        self.cfg = cfg
        self.tracks: list[Track] = []
        self._next = 1

    def update(self, t, obstacles, pose):
        """obstacles = obstacles.DollObstacle 목록(base_footprint), pose = 그 영상 시각의 odom 자세 또는 None."""
        cfg = self.cfg
        if pose is None:
            self.tracks = []        # 좌표 기준이 없으면 이어 붙이지 않음(예측 끔)
            return []
        self.tracks = [tr for tr in self.tracks if t - tr.t <= cfg.drop_after_s]
        free = list(self.tracks)
        out = []
        for o in obstacles:
            ox, oy = to_odom(pose, o.x_m, o.y_m)
            pts = tuple(to_odom(pose, px, py) + (pz,) for px, py, pz in o.points)
            best, best_d = None, None
            for tr in free:
                if tr.name != o.name:
                    continue
                dt = max(1e-3, t - tr.t)
                d = math.hypot(ox - tr.x, oy - tr.y)
                if d <= cfg.gate_m + cfg.max_speed_mps * dt and (best_d is None or d < best_d):
                    best, best_d = tr, d
            if best is None:
                tr = Track(self._next, o.name, ox, oy, t, points_odom=pts)
                self._next += 1
                self.tracks.append(tr)
            else:
                free.remove(best)
                dt = t - best.t
                if dt > 1e-3:
                    vx, vy = (ox - best.x) / dt, (oy - best.y) / dt
                    a = cfg.smooth if best.hits > 1 else 1.0
                    best.vx += a * (vx - best.vx)
                    best.vy += a * (vy - best.vy)
                best.x, best.y, best.t, best.points_odom = ox, oy, t, pts
                best.hits += 1
                tr = best
            out.append(tr)
        return out

    def moving(self, tr):
        return tr.hits >= self.cfg.min_hits and self.cfg.min_speed_mps <= tr.speed <= self.cfg.max_speed_mps

    def predicted_points(self, pose, tracks):
        """움직이는 트랙의 예상 자리 점(base_footprint). 지금 자리 점은 obstacles 가 이미 보냄."""
        cfg = self.cfg
        if pose is None:
            return []
        n = int(round(cfg.horizon_s / cfg.step_s))
        out = []
        for tr in tracks:
            if not self.moving(tr):
                continue
            pts = tr.points_odom[::2] or ((tr.x, tr.y, 0.05),)
            for k in range(1, n + 1):
                dt = k * cfg.step_s
                for px, py, pz in pts:
                    bx, by = to_base(pose, px + tr.vx * dt, py + tr.vy * dt)
                    out.append((round(bx, 4), round(by, 4), pz))
        return out

    def summary(self, pose, tracks):
        """화면용: 트랙별 속도와 로봇 쪽으로 오는지(닿기까지 시간, 없으면 None)."""
        res = []
        for tr in tracks:
            item = {'id': tr.id, 'name': tr.name, 'speed_cms': round(tr.speed * 100, 1), 'moving': self.moving(tr),
                    'ttc_s': None}
            if pose is not None and self.moving(tr):
                bx, by = to_base(pose, tr.x, tr.y)
                c, s = math.cos(pose[2]), math.sin(pose[2])
                vbx, vby = c * tr.vx + s * tr.vy, -s * tr.vx + c * tr.vy     # 로봇 좌표 속도(로봇 정지 가정)
                r = math.hypot(bx, by)
                closing = -(bx * vbx + by * vby) / r if r > 1e-6 else 0.0
                if closing > 1e-3:
                    item['ttc_s'] = round(r / closing, 1)
            res.append(item)
        return res
