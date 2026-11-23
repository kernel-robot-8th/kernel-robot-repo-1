"""화면 겹치기 그림(격자·YOLO 상자·카드). OpenCV 창과 웹 화면이 같이 쓴다."""
from __future__ import annotations

import numpy as np

from .floor_geom import box_to_ground, distance_from_height

GRID_CM = (25, 40, 60, 80, 120, 160, 200)
GREEN, YELLOW, RED, CYAN, MAGENTA, WHITE = (0, 200, 0), (0, 220, 255), (0, 0, 255), (255, 200, 0), (255, 0, 255), (255, 255, 255)


def put(img, text, org, color=WHITE, scale=0.5):
    """테두리 있는 글자. 테두리는 같은 굵기로 1 px씩 비켜 그린다(OpenCV 5 에서 굵기가 다르면 자간이 달라 겹쳐 보임)."""
    import cv2
    x, y = int(org[0]), int(org[1])
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1)):
        cv2.putText(img, text, (x + dx, y + dy), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_grid(img, cam):
    import cv2
    h, w = img.shape[:2]
    hv = cam.horizon_v()
    if 0 <= hv < h:
        cv2.line(img, (0, int(hv)), (w, int(hv)), CYAN, 1)
        put(img, 'horizon', (4, int(hv) - 4), CYAN, 0.4)
    last_label_y = None
    for d_cm in GRID_CM:
        d = d_cm / 100
        pts = []
        for y in np.linspace(-0.6 * d, 0.6 * d, 41):   # 화각(약 ±31°) 안만: 밖까지 그리면 왜곡 다항식이 선을 휘게 함
            uv = cam.ground_to_pixel(d, y)
            if uv and -50 < uv[0] < w + 50 and 0 <= uv[1] < h:
                pts.append((int(uv[0]), int(uv[1])))
        if len(pts) >= 2:
            cv2.polylines(img, [np.array(pts, np.int32)], False, GREEN, 1)
            mid_y = pts[len(pts) // 2][1]
            if last_label_y is None or abs(mid_y - last_label_y) >= 14:   # 먼 거리 선은 촘촘해 글자 겹침 방지
                top = cam.visible_top_height(d) * 100
                put(img, f'{d_cm}cm (top {top:.0f}cm)', (w - 150, mid_y - 3), GREEN, 0.4)
                last_label_y = mid_y


def draw_dets(img, cam, dets, doll_h):
    import cv2
    h = img.shape[0]
    rows = []
    for d in dets:
        x1, y1, x2, y2 = (int(v) for v in d.box)
        g = box_to_ground(cam, d.box)
        cut = y2 >= h - 2
        col = YELLOW if cut else GREEN
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 2)
        txt = f'{d.name} {d.conf:.2f}'
        if g:
            txt += f' {g.dist_m * 100:.0f}cm {g.bearing_deg:+.0f}deg w{g.width_m * 100:.1f}'
        if cut:
            txt += ' FEET CUT(closer)'
        put(img, txt, (x1, max(12, y1 - 5)), col, 0.45)
        dh = distance_from_height(cam, d.box, doll_h) if doll_h else None
        if dh:
            put(img, f'by height {dh * 100:.0f}cm', (x1, min(h - 4, y2 + 14)), col, 0.4)
        rows.append((d, g, cut, dh))
    return rows


def draw_cards(img, cards):
    import cv2
    for c in cards:
        cv2.polylines(img, [c.corners.astype(np.int32)], True, MAGENTA, 2)
        x, y = c.corners.min(axis=0).astype(int)
        put(img, f'card {c.id} {c.dist_m * 100:.0f}cm side {c.side_px:.0f}px', (x, max(12, y - 5)), MAGENTA, 0.45)


def draw_obstacles(img, cam, sink):
    """Nav2 로 보낸 인형 표시 점(빨강)과 보낸 횟수. 점은 base_footprint → 렌즈 바닥 좌표로 되돌려 그린다."""
    import cv2
    cfg = sink.cfg
    for o in sink.last:
        for x, y, _ in o.points[::3]:
            uv = cam.ground_to_pixel(x - cfg.cam_x_m, y - cfg.cam_y_m)
            if uv:
                cv2.circle(img, (int(uv[0]), int(uv[1])), 1, RED, -1)
    lag = '' if sink.lag_s is None else f', lag {sink.lag_s * 1000:.0f}ms'
    mv = ''
    if getattr(sink, 'motion', None) is not None:
        moving = [m for m in sink.moving_info if m['moving']]
        mv = (f', moving {len(moving)} pred {sink.predicted}' if getattr(sink, 'moving_pose_ok', False)
              else ', moving: no odom')
    put(img, f'OBST {len(sink.last)} doll -> nav2 (sent {sink.sent}, stale {sink.skipped}{lag}{mv})',
        (6, 60), RED, 0.5)


def draw_follow(img, sink):
    """추종 대상 상태 한 줄(초록 = 따라가는 중, 노랑 = 아직 못 찾음, 빨강 = 놓침)."""
    st = sink.last or {'state': 'searching', 'card_id': sink.cfg.card_id}
    col = {'tracking': GREEN, 'lost': RED}.get(st['state'], YELLOW)
    pos = (f" {st['dist_m'] * 100:.0f}cm {st['bearing_deg']:+.0f}deg" if 'dist_m' in st else '')
    age = '' if st.get('age_s') is None else f" age {st['age_s']:.1f}s"
    src = ' [DOLL %.0fs]' % (st.get('fallback_s') or 0.0) if st.get('source') == 'doll' else ''
    put(img, f"FOLLOW card {st['card_id']} {st['state']}{src}{pos}{age} (sent {sink.sent}, stale {sink.skipped})",
        (6, 80), col, 0.5)
