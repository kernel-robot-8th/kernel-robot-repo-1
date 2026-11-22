"""본 인형을 기억해 두었다가 영상마다 다시 보낸다(ROS 없음, 순수 계산).

왜(11-21 실로봇): 어린이 인형(6 cm)은 라이다(13.6 cm)에 안 보이고 카메라로만 보인다. 카메라는 로봇 바로 앞(바퀴 축
약 13 cm 안)을 못 보므로, 인형이 그 안에 들어오면 「아까 본 기억」만 남는다. 그 기억을 로봇 비용 지도에만 두면
- 지도를 비우는 순간 진짜 인형이 사라져 로봇이 치고 지나가고(25회차),
- 비우지 않으면 잘못 찍힌 점이 로봇 15 cm 안에서 영영 남아 멈춘다(26회차).
→ 기억은 노트북이 갖고, 로봇 비용 지도(인형 층)는 노트북이 보낸 것만 그대로 비추게 한다
  (매 영상마다 로봇 둘레를 모두 지우고 다시 찍음: obstacles.clear_ring, nav2 인형 층 min range 0·persistence 0).

규칙
- 지금 보이는 인형은 항상 보낸다.
- confirm_frames(2) 번 이상 본 인형만 기억한다(한 장짜리 오검출은 그 한 장만 나가고 사라짐).
- 기억한 인형이 카메라가 볼 수 있는 곳(visible)에 있는데 miss_frames(3) 장 연속 안 보이면 잊는다(치워졌거나 오검출).
- 카메라가 못 보는 곳(로봇 바로 앞 사각·시야 밖)의 기억은 유지한다. 단 lidar_visible 클래스(adult: 16 cm, 라이다에도
  보이고 대부분 고객 자신)는 adult_keep_s(3 s), child 는 child_keep_s(60 s) 뒤 잊는다. 로봇이 keep_range_m 넘게
  멀어져도 잊는다. clear() 는 기억을 모두 지운다(로봇이 라인 모드일 때 세션이 부름: 손으로 옮긴 뒤 옛 기억 방지).
- 고객(추종 대상) 둘레 exclude 반경 안의 기억은 지운다.
좌표: 기억은 odom(바퀴 주행 기록) 좌표에 둔다. 로봇이 움직여도 인형 자리는 그대로다. pose = 영상 시각의 (x, y, yaw).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .moving import to_base, to_odom


@dataclass
class MemoryConfig:
    match_m: float = 0.10            # 이 거리 안이면 같은 인형(바닥 투영 오차·상자 흔들림 포함)
    confirm_frames: int = 2
    confirm_window_s: float = 1.0    # 아직 한 번만 본 인형은 이 시간 안에 다시 안 보이면 버림
    miss_frames: int = 3
    adult_keep_s: float = 3.0
    # 시야 밖 child 를 기억하는 가장 긴 시간. 드라이버는 30 s 못 움직이면 추종을 끝내므로 그보다 길면 충분하다.
    # 끝없이 두면 안 되는 이유(11-21): 로봇을 손으로 들어 옮기면 odom 이 그대로라 옛 기억이 엉뚱한 자리에 남음
    child_keep_s: float = 60.0
    lidar_visible: tuple = ('adult',)
    keep_range_m: float = 1.5
    vis_near_m: float = 0.22         # 바퀴 축 앞 이 거리부터 인형이 화면에 온전히 보인다고 봄(사각 0.13 m + 여유)
    vis_far_m: float = 1.0
    vis_margin_deg: float = 6.0      # 시야 가장자리 여유(상자가 잘리면 검출이 흔들림)
    max_entries: int = 40

    def validate(self):
        if not (0.02 <= self.match_m <= 0.5):
            raise ValueError('match_m must be 0.02..0.5')
        if self.confirm_frames < 1 or self.miss_frames < 1:
            raise ValueError('confirm_frames and miss_frames must be >= 1')
        if not (0 < self.vis_near_m < self.vis_far_m <= self.keep_range_m):
            raise ValueError('need 0 < vis_near_m < vis_far_m <= keep_range_m')


class _Entry:
    __slots__ = ('name', 'cx', 'cy', 'pts', 'first_t', 'last_t', 'hits', 'misses')

    def __init__(self, name, cx, cy, pts, t):
        self.name, self.cx, self.cy, self.pts = name, cx, cy, pts
        self.first_t = self.last_t = t
        self.hits, self.misses = 1, 0


class DollMemory:
    def __init__(self, cfg: MemoryConfig | None = None, fov_left_deg=24.0, fov_right_deg=24.0, cam_x_m=0.035, cam_y_m=0.0):
        self.cfg = cfg or MemoryConfig()
        self.cfg.validate()
        self.left = math.radians(max(1.0, fov_left_deg - self.cfg.vis_margin_deg))
        self.right = math.radians(max(1.0, fov_right_deg - self.cfg.vis_margin_deg))
        self.cam = (float(cam_x_m), float(cam_y_m))
        self.entries: list[_Entry] = []
        self.pose = None            # 마지막으로 쓴 자세(자세가 잠깐 없을 때 대신 씀: 서 있으면 그대로 맞음)
        self.pose_ok = False
        self.last_out = []

    def _visible(self, bx, by):
        """base_footprint 점이 카메라가 인형을 온전히 볼 수 있는 곳인가."""
        if bx < self.cfg.vis_near_m or math.hypot(bx, by) > self.cfg.vis_far_m:
            return False
        a = math.atan2(by - self.cam[1], bx - self.cam[0])      # 왼쪽 +
        return -self.right <= a <= self.left

    def update(self, t, pose, obs, exclude=None, exclude_radius_m=0.15):
        """t: 영상 시각(s). pose: 그 시각 odom 자세 (x, y, yaw) 또는 None. obs: 이번 영상의 DollObstacle 목록
        (고객은 이미 뺀 것). exclude: 고객 위치 (x, y) base_footprint 또는 None. exclude_radius_m: 반경(m) 또는
        종류 이름 → 반경 함수(고객과 같은 종류 adult 는 넓게).
        → [{'name', 'points'(base_footprint), 'x_m', 'y_m', 'seen', 'age_s'}, ...] 보낼 인형 전체."""
        cfg = self.cfg
        self.pose_ok = pose is not None
        if pose is None:
            pose = self.pose
        if pose is None:                      # 자세를 한 번도 못 받음: 기억 불가 → 지금 보이는 것만
            self.last_out = [{'name': o.name, 'points': tuple(o.points), 'x_m': o.x_m, 'y_m': o.y_m,
                              'seen': True, 'age_s': 0.0} for o in obs]
            return self.last_out
        self.pose = pose
        seen = set()
        for o in obs:
            cx, cy = to_odom(pose, o.x_m, o.y_m)
            pts = tuple((*to_odom(pose, x, y), z) for x, y, z in o.points)
            best = None
            for i, e in enumerate(self.entries):
                d = math.hypot(e.cx - cx, e.cy - cy)
                if i not in seen and d <= cfg.match_m and (best is None or d < best[0]):
                    best = (d, i)
            if best is None:
                self.entries.append(_Entry(o.name, cx, cy, pts, t))
                seen.add(len(self.entries) - 1)
            else:
                e = self.entries[best[1]]
                e.name, e.cx, e.cy, e.pts, e.last_t = o.name, cx, cy, pts, t
                e.hits, e.misses = e.hits + 1, 0
                seen.add(best[1])
        keep, out = [], []
        for i, e in enumerate(self.entries):
            bx, by = to_base(pose, e.cx, e.cy)
            if i in seen:
                keep.append(e)
                out.append(self._out(e, pose, bx, by, True, 0.0))
                continue
            age = t - e.last_t
            if age < 0:                                   # 시각이 거꾸로(재생·시계 바뀜): 기억을 믿지 않음
                continue
            if e.hits < cfg.confirm_frames:
                if age <= cfg.confirm_window_s:
                    keep.append(e)                        # 아직 확인 전: 보내지 않고 한 번 더 기다림
                continue
            if math.hypot(bx, by) > cfg.keep_range_m:
                continue
            r = exclude_radius_m(e.name) if callable(exclude_radius_m) else exclude_radius_m
            if exclude is not None and math.hypot(bx - exclude[0], by - exclude[1]) <= r:
                continue
            if self._visible(bx, by):
                e.misses += 1
                if e.misses >= cfg.miss_frames:
                    continue
            elif age > (cfg.adult_keep_s if e.name in cfg.lidar_visible else cfg.child_keep_s):
                continue
            keep.append(e)
            out.append(self._out(e, pose, bx, by, False, age))
        if len(keep) > cfg.max_entries:                   # 너무 많으면 오래 못 본 것부터 버림
            keep.sort(key=lambda e: e.last_t)
            drop = set(map(id, keep[:len(keep) - cfg.max_entries]))
            keep = [e for e in keep if id(e) not in drop]
            out = [o for o in out if o['_id'] not in drop]
        self.entries = keep
        for o in out:
            o.pop('_id', None)
        self.last_out = out
        return out

    @staticmethod
    def _out(e, pose, bx, by, seen, age):
        pts = tuple((*(round(v, 4) for v in to_base(pose, x, y)), z) for x, y, z in e.pts)
        return {'name': e.name, 'points': pts, 'x_m': bx, 'y_m': by, 'seen': seen, 'age_s': round(age, 2), '_id': id(e)}

    def clear(self):
        self.entries, self.last_out = [], []

    def info(self):
        """시험 화면·기록용."""
        return {'pose_ok': self.pose_ok, 'count': len(self.last_out),
                'remembered': [{'name': o['name'], 'x_cm': round(o['x_m'] * 100, 1), 'y_cm': round(o['y_m'] * 100, 1),
                                'age_s': o['age_s']} for o in self.last_out if not o['seen']]}
