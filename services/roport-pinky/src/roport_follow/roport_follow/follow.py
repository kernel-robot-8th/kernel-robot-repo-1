"""고객 카드 → 추종 대상 위치(ROS 없음, 순수 계산). FFT1-188.

- 따라갈 카드는 고객 확인 때 정한 ID 하나(SR: 다른 마커로 바꾸지 않음). 다른 ID 카드는 무시하고 목록만 남긴다.
- 위치: 카메라 좌표(solvePnP 의 카드 중심) → 바닥 좌표(렌즈 아래 바닥 원점, CameraModel.rotation) → base_footprint
  (렌즈 위치 cam_x_m·cam_y_m 만큼 옮김). 카드 높이와 상관없이 카드 바로 아래 바닥 점을 쓴다.
- 놓침: 마지막으로 본 뒤 lost_after_s(기본 4 s, 11-20 시험값. SR-22 예외 흐름은 1 s) 넘게 못 보면 lost.
- 튐 거름: 직전 위치에서 (0.15 m + 고객 최대 속도 × 경과 시간)보다 멀리 뛴 검출은 버린다(오검출·다른 카드 오인).
- 시각: 로봇 영상 시각(robot_time). 고객 뒤 몇 cm 에 설지는 로봇 쪽 Nav2(TruncatePath)가 정한다.
- 인형으로 잇기(doll_fallback, 임시·기본 끔, 11-21): 카드는 인형 등에 낮게 붙어 있어 고객이 방향을 틀거나(옆면)
  어린이 인형 뒤에 서면(가림) 몇 초씩 안 보인다. 카드로 고객을 확인한 뒤 카드가 안 보이는 동안만, 마지막 고객 자리
  근처의 어른 인형(YOLO 바닥 점)을 고객으로 보고 잇는다. 카드 없이 시작하지 않고, 연속 fb_max_s(8 s)까지만 잇는다.
  두 어른 인형이 비슷하게 가까우면 잇지 않는다(다른 인형을 따라갈 위험). 로봇이 도는 동안의 자리 변화는 odom 자세로 뺀다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

SEARCHING, TRACKING, LOST = 'searching', 'tracking', 'lost'


@dataclass
class FollowConfig:
    card_id: int = 35
    cam_x_m: float = 0.035          # base_footprint → 렌즈(obstacles.ObstacleConfig 와 같은 값)
    cam_y_m: float = 0.0
    lost_after_s: float = 4.0      # 11-20 사용자 결정: SR-22 1 s → 2 s(번짐·가림) → 밤 4 s(피해 가는 동안 시야 밖·가림)
    max_range_m: float = 1.5        # 이보다 먼 카드는 안 씀(30 mm 카드: 합성 영상 1.5 m 에서 이미 약 8 cm 오차)
    min_side_px: float = 6.0        # 화면에서 한 변이 이보다 작으면 안 씀
    max_speed_mps: float = 1.5      # 튐 거름에 쓰는 고객 최대 속도
    jump_base_m: float = 0.15
    smooth: float = 0.5             # 위치 지수 평균(1 = 거르지 않음)
    valid_ids: tuple = field(default=tuple(range(35, 50)))
    # 인형으로 잇기(임시, 기본 끔)
    doll_fallback: bool = False
    fb_gate_m: float = 0.30         # 마지막 고객 자리에서 이 거리 안의 어른 인형만
    fb_speed_mps: float = 0.30      # 카드를 못 본 시간만큼 넓힘(고객 걸음)
    fb_gate_max_m: float = 0.60
    fb_max_s: float = 8.0           # 카드 없이 인형만으로 잇는 가장 긴 시간
    fb_ambiguity_m: float = 0.10    # 두 인형의 거리 차가 이보다 작으면 잇지 않음
    fb_pair_m: float = 0.35         # 카드가 보일 때 이 거리 안의 어른 인형을 고객 인형으로 보고 위치 차를 배움

    def validate(self):
        if self.card_id not in self.valid_ids:
            raise ValueError(f'card_id must be a customer card id {self.valid_ids[0]}..{self.valid_ids[-1]}')
        if not (0.2 <= self.lost_after_s <= 10.0):
            raise ValueError('lost_after_s must be 0.2..10')
        if not (0.3 <= self.max_range_m <= 4.0):
            raise ValueError('max_range_m must be 0.3..4')
        if not (0.0 < self.smooth <= 1.0):
            raise ValueError('smooth must be in (0, 1]')
        if not all(math.isfinite(v) for v in (self.cam_x_m, self.cam_y_m, self.max_speed_mps, self.jump_base_m)):
            raise ValueError('offsets and speeds must be finite')


def card_floor_point(cam, card, cfg: FollowConfig):
    """카드 중심 바로 아래 바닥 점 (x 앞, y 왼쪽) m, base_footprint. 카메라 y 성분이 없으면 None."""
    t = np.array([card.x_m, card.y_m, card.z_m], float)
    if not np.all(np.isfinite(t)) or t[2] <= 0:
        return None
    p = cam.rotation() @ t
    return float(p[0] + cfg.cam_x_m), float(p[1] + cfg.cam_y_m)


class FollowTracker:
    def __init__(self, cam, cfg: FollowConfig):
        cfg.validate()
        self.cam, self.cfg = cam, cfg
        self.pos = None             # 거른 위치 (x, y)
        self.last_seen = None       # 마지막으로 받아들인 시각(로봇 시각 s)
        self.rejected = 0
        # 인형으로 잇기 상태
        self.pos_odom = None        # 마지막 고객 자리(odom) — 로봇이 움직여도 그대로
        self.offset = (0.0, 0.0)    # 카드 자리 − 고객 인형(YOLO) 자리(base_footprint), 보일 때 배움
        self.fb_since = None        # 인형으로만 잇기 시작한 시각
        self.source = None          # 'card' | 'doll' | None(이번 영상에서 못 봄)

    def reset(self):
        self.pos, self.last_seen = None, None
        self.pos_odom, self.offset, self.fb_since, self.source = None, (0.0, 0.0), None, None

    def _expected(self, pose):
        """지금 로봇 기준으로 본 마지막 고객 자리. odom 자세가 있으면 로봇 움직임을 뺀다."""
        if pose is not None and self.pos_odom is not None:
            from .moving import to_base
            return to_base(pose, *self.pos_odom)
        return self.pos

    def _remember(self, pose):
        if pose is not None and self.pos is not None:
            from .moving import to_odom
            self.pos_odom = to_odom(pose, *self.pos)

    def _fallback(self, t, dolls, pose):
        """카드를 못 본 영상: 마지막 고객 자리 근처의 어른 인형 → 고객 자리 (x, y) 또는 None."""
        cfg = self.cfg
        if not (cfg.doll_fallback and dolls and self.pos is not None and self.last_seen is not None):
            return None
        age = max(0.0, t - self.last_seen)
        if age > cfg.lost_after_s:
            return None                                   # 이미 놓침: 카드를 다시 봐야 함
        if self.fb_since is not None and t - self.fb_since > cfg.fb_max_s:
            return None                                   # 인형만으로 너무 오래 이음
        ref = self._expected(pose)
        gate = min(cfg.fb_gate_max_m, cfg.fb_gate_m + cfg.fb_speed_mps * age)
        cand = sorted((math.dist((x + self.offset[0], y + self.offset[1]), ref), x + self.offset[0], y + self.offset[1])
                      for x, y in dolls)
        cand = [c for c in cand if c[0] <= gate]
        if not cand or (len(cand) > 1 and cand[1][0] - cand[0][0] < cfg.fb_ambiguity_m):
            return None
        return cand[0][1], cand[0][2]

    def update(self, t, cards, dolls=None, pose=None):
        """t = 영상 시각(s), cards = markers.Card 목록 → 상태 dict.
        dolls = 어른 인형 바닥 점 [(x, y), ...] base_footprint(doll_fallback 일 때만 씀), pose = 그 시각 odom 자세."""
        cfg = self.cfg
        self.source = None
        seen, cand = False, []
        for c in cards:
            if c.id != cfg.card_id or c.side_px < cfg.min_side_px:
                continue
            p = card_floor_point(self.cam, c, cfg)
            if p is None or math.hypot(*p) > cfg.max_range_m:
                continue
            cand.append(p)
        if cand:
            ref = self.pos
            p = min(cand, key=lambda q: math.dist(q, ref)) if ref else min(cand, key=lambda q: math.hypot(*q))
            if self.pos is not None and self.last_seen is not None:
                dt = max(0.0, t - self.last_seen)
                if math.dist(p, self.pos) > cfg.jump_base_m + cfg.max_speed_mps * dt:
                    self.rejected += 1
                    p = None
            if p is not None:
                a = cfg.smooth if self.pos is not None else 1.0
                self.pos = p if self.pos is None else (self.pos[0] + a * (p[0] - self.pos[0]),
                                                       self.pos[1] + a * (p[1] - self.pos[1]))
                self.last_seen, seen = t, True
                self.source, self.fb_since = 'card', None
                if cfg.doll_fallback and dolls:           # 고객 인형의 YOLO 자리와 카드 자리의 차를 배움
                    near = min(dolls, key=lambda q: math.dist(q, p))
                    if math.dist(near, p) <= cfg.fb_pair_m:
                        off = (p[0] - near[0], p[1] - near[1])
                        self.offset = (self.offset[0] + 0.3 * (off[0] - self.offset[0]),
                                       self.offset[1] + 0.3 * (off[1] - self.offset[1]))
                self._remember(pose)
        if not seen:
            p = self._fallback(t, dolls, pose)
            if p is not None:
                base = self._expected(pose)
                self.pos = (base[0] + cfg.smooth * (p[0] - base[0]), base[1] + cfg.smooth * (p[1] - base[1]))
                if self.fb_since is None:
                    self.fb_since = t
                self.last_seen, seen, self.source = t, True, 'doll'
                self._remember(pose)
        if self.last_seen is None:
            state, age = SEARCHING, None
        else:
            age = max(0.0, t - self.last_seen)
            state = TRACKING if age <= cfg.lost_after_s else LOST
        out = {'state': state, 'card_id': cfg.card_id, 'seen': seen, 'age_s': None if age is None else round(age, 2),
               'others': sorted({c.id for c in cards if c.id != cfg.card_id}), 'rejected': self.rejected,
               'source': self.source,
               'fallback_s': None if self.fb_since is None else round(max(0.0, t - self.fb_since), 2)}
        if self.pos is not None:
            x, y = self.pos
            out.update(x_m=round(x, 3), y_m=round(y, 3), dist_m=round(math.hypot(x, y), 3),
                       bearing_deg=round(math.degrees(math.atan2(y, x)), 1))
        return out
