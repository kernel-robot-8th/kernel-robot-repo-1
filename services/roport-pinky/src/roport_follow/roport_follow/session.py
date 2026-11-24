"""0단계 화면 세션: 영상 한 장 처리(격자·YOLO·카드·체스보드 겹치기)와 키·클릭 동작.

OpenCV 창(tool view)과 웹 화면(tool web)이 같은 세션을 쓴다. 로봇에는 카메라만 쓴다.
"""
from __future__ import annotations

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path

from .floor_geom import fit_pose
from .tool_draw import YELLOW, RED, WHITE, draw_cards, draw_dets, draw_follow, draw_grid, draw_obstacles, put

KEY_HELP = {
    'y': 'YOLO 켜기/끄기', 'a': '고객 카드 켜기/끄기', 'b': '체스보드 검출 켜기/끄기',
    'c': '체스보드 사진 저장(렌즈 보정용)', 'f': '바닥 체스보드로 자세 계산', 'h': '줄자 맞춤 계산',
    'x': '줄자 표본 지우기', 'w': '계산한 자세 저장', 's': '원본 사진 저장', 'r': '연속 저장 켜기/끄기',
    'v': '원본 영상 녹화 켜기/끄기',
    'enter': '측정 기록(CSV)',
}


class ViewSession:
    def __init__(self, a, cam, src):
        from .markers import CardDetector
        self.a, self.cam, self.src = a, cam, src
        out = Path(a.out).expanduser()
        self.calib_dir = out / 'calib' / a.name
        self.photo_dir = Path(a.save_dir).expanduser() if a.save_dir else out / 'photos' / a.name
        self.log_path = out / 'measure' / f'{a.name}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        self.yolo = None
        self.show = {'yolo': bool(a.yolo), 'card': True, 'board': False}
        self.cards_det = CardDetector(cam.K, cam.D, size_m=a.card_mm / 1000)
        self.typed, self.msg, self.msg_t = '', '', 0.0
        self.last_floor, self.floor_kind, self.saved = None, '', 0
        self.clicks, self.tape = [], []
        self.lock = threading.RLock()
        self.raw = self.corners = None
        self.rows, self.cards = [], []
        self.fps, self._fps_n, self._fps_t, self._last_seq = 0.0, 0, time.time(), None
        self.board_found = False
        self.last_result = {}
        # 연속 저장: 켜 두면 새 영상이 올 때 burst_sec 간격으로 원본을 저장(학습 사진 모으기)
        self.burst = False
        self.burst_sec = max(0.2, float(getattr(a, 'burst_sec', 0.5) or 0.5))
        self._burst_last = 0.0
        self.burst_count = 0
        # 원본 녹화: 겹쳐 그리기 전 카메라 영상만 mp4 로 저장(학습·시험 재생용)
        self.video_dir = out / 'video'
        self.rec, self.rec_path, self.rec_frames = None, None, 0
        # 인형 장애물 보내기(ros_obstacles.ObstacleSink, 없으면 None) — tool 의 --publish-obstacles
        self.obstacles = getattr(a, 'obstacle_sink', None)
        self._dets = None
        # 고객 카드 추종 대상 보내기(ros_follow.FollowSink, 없으면 None) — tool 의 --follow-card
        self.follow = getattr(a, 'follow_sink', None)
        self._cards = []
        # 시험용 로봇 명령 연결(demo_link.DemoLink, 없으면 None) — tool 의 --demo
        self.demo = getattr(a, 'demo_link', None)
        # 탑승 시간 알림 발행(boarding_alert.BoardingAlert, 없으면 None) — tool 의 --boarding-alert
        self.boarding = getattr(a, 'boarding', None)
        self.loc_view = getattr(a, 'loc_view', None)

    # ---- 알림
    def note(self, text, console=None):
        self.msg, self.msg_t = text, time.time()
        if console:
            print(console)

    # ---- 영상 한 장
    def step(self):
        """최신 영상을 처리해 겹친 그림(BGR)을 돌려준다. 새 영상이 없으면 None."""
        import cv2
        from .calib import find_corners
        fr = self.src.latest()
        if fr is None:
            return None
        with self.lock:
            new = fr.seq != self._last_seq
            self._last_seq = fr.seq
            raw = fr.image
            self.raw = raw
            if self.burst and new and time.time() - self._burst_last >= self.burst_sec:
                self._burst_last = time.time()
                self.burst_count += 1
                self._save_photo(raw, quiet=True)
            if self.rec is not None and new:
                self.rec.write(raw)
                self.rec_frames += 1
            img = raw.copy()
            draw_grid(img, self.cam)
            self.rows, self.cards, self.corners = [], [], None
            # 추종이 켜져 있으면 화면 표시(c)와 상관없이 새 영상마다 카드를 찾는다
            # (인형 장애물보다 먼저: 고객 카드를 단 인형은 장애물에서 뺀다)
            if self.show['card'] or self.follow is not None:
                if new or not self._cards:
                    self._cards = self.cards_det(raw)
                    if self.follow is not None and new and not self._follow_uses_dolls():
                        self.follow.handle(fr, self._cards)
                self.cards = self._cards
                draw_cards(img, self.cards)
            if self.follow is not None:
                draw_follow(img, self.follow)
            # 장애물 보내기가 켜져 있으면 화면 표시(y)와 상관없이 새 영상마다 검출한다
            if self.show['yolo'] or self.obstacles is not None:
                if self.yolo is None:
                    from .detector import Detector
                    print('YOLO 불러오는 중…')
                    self.yolo = Detector(self.a.model, conf=self.a.conf,
                                         threads=getattr(self.a, 'torch_threads', 0))
                if new or self._dets is None:          # 같은 영상은 다시 추론하지 않음
                    self._dets = self.yolo(raw)
                    if self.follow is not None and new and self._follow_uses_dolls():
                        # 인형으로 잇기(임시): 카드가 안 보이는 영상에서 쓸 어른 인형 자리를 함께 넘긴다
                        self.follow.handle(fr, self._cards or [], dolls=self._adult_points(self._dets))
                    if self.obstacles is not None:
                        ex = self.follow.exclude_point() if self.follow is not None else None
                        self._forget_dolls_in_line_mode()
                        self.obstacles.handle(fr, self._dets, exclude=ex)
            if self.obstacles is not None:
                draw_obstacles(img, self.cam, self.obstacles)
            if self.show['yolo']:
                dh = self.a.doll_height_cm / 100 if self.a.doll_height_cm else None
                self.rows = draw_dets(img, self.cam, self._dets, dh)
            if self.show['board']:
                self.corners = find_corners(cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY))
                if self.corners is not None:
                    cv2.drawChessboardCorners(img, (9, 6), self.corners, True)
            self.board_found = self.corners is not None
            for x, y, g in self.clicks:
                cv2.drawMarker(img, (x, y), RED, cv2.MARKER_CROSS, 14, 2)
                put(img, f'{g[0] * 100:.1f}cm L{g[1] * 100:+.1f}' if g else 'above horizon', (x + 8, y - 8), RED)
            for u, v, d in self.tape:
                cv2.circle(img, (int(u), int(v)), 5, YELLOW, 2)
                put(img, f'{d * 100:.0f}', (int(u) + 6, int(v) + 4), YELLOW, 0.45)
            if new:
                self._fps_n += 1
            if time.time() - self._fps_t >= 1.0:
                self.fps = self._fps_n / (time.time() - self._fps_t)
                self._fps_n, self._fps_t = 0, time.time()
            cam = self.cam
            put(img, f'{self.fps:4.1f} fps  cam: {cam.source[:44]}  h {cam.height_m * 100:.1f}cm '
                     f'pitch {cam.pitch_deg:.1f}', (6, img.shape[0] - 8), WHITE, 0.42)
            if self.rec is not None:
                put(img, f'RAW REC {self.rec_frames}', (img.shape[1] - 200, 18), RED, 0.6)
            if self.burst:
                put(img, f'REC {self.burst_count}', (img.shape[1] - 90, 18), RED, 0.6)
            put(img, f'true(cm): {self.typed or "-"}   yolo {"on" if self.show["yolo"] else "off"}  card '
                     f'{"on" if self.show["card"] else "off"}  board {"on" if self.show["board"] else "off"}',
                (6, 18), YELLOW, 0.5)
            if self.msg and time.time() - self.msg_t < 4:
                put(img, self.msg, (6, 40), YELLOW, 0.5)
            return img

    # ---- 입력
    def click(self, x, y):
        with self.lock:
            x, y = int(x), int(y)
            g = self.cam.pixel_to_ground(x, y)
            self.clicks[:] = [(x, y, g)]
            print(f'클릭 ({x},{y}) → ' + (f'앞 {g[0] * 100:.1f} cm, 왼쪽 {g[1] * 100:+.1f} cm' if g else '수평선 위(바닥 아님)'))
            if self.typed:   # 거리를 먼저 치고 클릭 = 줄자 표시 표본
                self.tape.append((x, y, int(self.typed) / 100))
                self.note(f'tape sample {len(self.tape)}: {self.typed}cm (h = fit)',
                          f'줄자 표본 {len(self.tape)}: ({x},{y}) = 앞 {self.typed} cm')
                self.typed = ''
            return g

    def set_typed(self, text):
        with self.lock:
            text = ''.join(ch for ch in str(text) if ch.isdigit())[:4]
            self.typed = text
            return self.typed

    def key(self, k):
        """k = 한 글자('y', 's', '0'~'9') 또는 'enter'·'backspace'. 결과 문구를 돌려준다."""
        import cv2
        from .calib import FloorPose, find_corners, floor_pose
        from .markers import CardDetector
        with self.lock:
            raw = self.raw
            if k in [str(i) for i in range(10)]:
                if len(self.typed) < 4:
                    self.typed += k
                return f'true {self.typed}'
            if k == 'backspace':
                self.typed = self.typed[:-1]
                return f'true {self.typed}'
            if k in ('y', 'a', 'b'):
                name = {'y': 'yolo', 'a': 'card', 'b': 'board'}[k]
                self.show[name] = not self.show[name]
                return f'{name} {"on" if self.show[name] else "off"}'
            if k == 'x':
                self.tape.clear()
                self.note('tape samples cleared')
                return self.msg
            if raw is None:
                self.note('no camera image yet')
                return self.msg
            if k == 's':
                p = self._save_photo(raw)
                self.note(f'saved {p.name}', f'사진 저장: {p}')
                return self.msg
            if k == 'r':
                self.burst = not self.burst
                if self.burst:
                    self.burst_count, self._burst_last = 0, 0.0
                    self.note(f'continuous save ON every {self.burst_sec:g}s -> {self.photo_dir}',
                              f'연속 저장 켬({self.burst_sec:g} s 간격) → {self.photo_dir}')
                else:
                    self.note(f'continuous save OFF ({self.burst_count} saved)', f'연속 저장 끔: {self.burst_count}장')
                return self.msg
            if k == 'v':
                return self._toggle_record(raw)
            if k in ('c', 'f'):
                c = self.corners if self.corners is not None else find_corners(cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY))
                if c is None:
                    self.note('no chessboard found (b = show detection)')
                    return self.msg
                if k == 'c':
                    self.calib_dir.mkdir(parents=True, exist_ok=True)
                    n = len(list(self.calib_dir.glob('*.png'))) + 1
                    p = self.calib_dir / f'board_{n:02d}.png'
                    cv2.imwrite(str(p), raw)
                    self.note(f'calib image {n} saved (aim for 15-20, vary angle/position)', f'렌즈 보정 사진 저장: {p}')
                    return self.msg
                fp = floor_pose(c, self.cam.K, self.cam.D, self.a.board_thickness_mm / 1000)
                self.last_floor, self.floor_kind = fp, 'floor chessboard'
                self.last_result = {'kind': 'floor', 'height_cm': round(fp.height_m * 100, 2),
                                    'pitch_deg': round(fp.pitch_deg, 2), 'roll_deg': round(fp.roll_deg, 2),
                                    'reproj_px': round(fp.reproj_px, 2), 'board_dist_cm': round(fp.board_dist_m * 100, 1)}
                self.note(f'floor: h {fp.height_m * 100:.2f}cm pitch {fp.pitch_deg:.2f} roll {fp.roll_deg:+.2f} '
                          f'reproj {fp.reproj_px:.2f}px (w = save)', f'바닥 자세: {fp}')
                return self.msg
            if k == 'h':
                fix = self.a.lens_height_cm / 100 if self.a.lens_height_cm else None
                try:
                    h_m, p_deg, rms_cm = fit_pose(self.cam, self.tape, fix_height_m=fix)
                except ValueError as e:
                    self.note(f'fit: {e}')
                    return self.msg
                self.last_floor = FloorPose(h_m, p_deg, self.cam.roll_deg, math.nan, math.nan)
                self.floor_kind = f'tape fit {len(self.tape)} pts rms {rms_cm:.1f}cm'
                self.last_result = {'kind': 'tape', 'height_cm': round(h_m * 100, 2), 'pitch_deg': round(p_deg, 2),
                                    'rms_cm': round(rms_cm, 2), 'points': len(self.tape)}
                self.note(f'tape fit: h {h_m * 100:.2f}cm pitch {p_deg:.2f} rms {rms_cm:.1f}cm '
                          f'({len(self.tape)} pts, w = save)',
                          f'줄자 맞춤: 높이 {h_m * 100:.2f} cm, 숙임 {p_deg:.2f}°, rms {rms_cm:.1f} cm')
                return self.msg
            if k == 'w':
                if self.last_floor is None:
                    self.note('nothing to save (f or h first)')
                    return self.msg
                fp = self.last_floor
                self.cam.height_m, self.cam.pitch_deg, self.cam.roll_deg = fp.height_m, fp.pitch_deg, fp.roll_deg
                self.cam.source = f'{self.floor_kind} {datetime.now():%Y-%m-%d %H:%M}' + (
                    '' if 'intrinsics' in self.cam.source else ' (lens = default estimate)')
                self.cam.save(Path(self.a.camera).expanduser())
                self.cards_det = CardDetector(self.cam.K, self.cam.D, size_m=self.a.card_mm / 1000)
                self.note(f'camera saved: {self.a.camera}', f'카메라 값 저장: {Path(self.a.camera).expanduser()}')
                return self.msg
            if k == 'enter':
                return self._log_row()
            return f'unknown key {k!r}'

    def _toggle_record(self, raw):
        import cv2
        if self.rec is not None:
            self.close()
            return self.msg
        self.video_dir.mkdir(parents=True, exist_ok=True)
        path = self.video_dir / f'{self.a.prefix}_{datetime.now():%Y%m%d_%H%M%S}_raw.mp4'
        fps = self.fps if self.fps >= 1.0 else float(getattr(self.a, 'fps', 10.0) or 10.0)
        w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), round(fps, 1), (raw.shape[1], raw.shape[0]))
        if not w.isOpened():
            self.note(f'cannot open video file {path.name}', f'녹화 파일을 열 수 없음: {path}')
            return self.msg
        self.rec, self.rec_path, self.rec_frames = w, path, 0
        self.note(f'raw recording ON {fps:.1f} fps -> {path.name}', f'원본 녹화 시작({fps:.1f} fps): {path}')
        return self.msg

    # ---- 시험 1 화면(인형 장애물)
    def obstacle_pause(self, paused):
        if self.obstacles is None:
            raise ValueError('obstacle publishing is off (--publish-obstacles)')
        with self.lock:
            self.obstacles.paused = bool(paused)
            self.note('obstacles PAUSED (not sending)' if paused else 'obstacles sending again',
                      '장애물 보내기 일시 멈춤(끊김 시험)' if paused else '장애물 보내기 다시 시작')
            return self.obstacles.paused

    def test_log(self, step, note=''):
        """시험 기록 한 줄(CSV): 시각·단계·메모·발행 상태·인형 목록. 파일 경로를 돌려준다."""
        import json
        if self.obstacles is None:
            raise ValueError('obstacle publishing is off (--publish-obstacles)')
        if not isinstance(step, int) or isinstance(step, bool) or not 0 <= step <= 9:
            raise ValueError('step must be an integer 0..9')
        note = ''.join(ch for ch in str(note) if ch.isprintable())[:100]
        with self.lock:
            info = self.obstacles.info()
            path = self.log_path.with_name(self.log_path.stem + '_test1.csv')
            path.parent.mkdir(parents=True, exist_ok=True)
            new = not path.exists()
            with path.open('a', newline='') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(['time', 'step', 'note', 'sent', 'rate_hz', 'stale', 'lag_ms', 'paused', 'dolls'])
                w.writerow([datetime.now().isoformat(timespec='seconds'), step, note, info['sent'], info['rate_hz'],
                            info['stale'], info['lag_ms'], info['paused'],
                            json.dumps(info['dolls'], ensure_ascii=False)])
            self.note(f'test log step {step} -> {path.name}', f'시험 기록 {step}단계 → {path}')
            return str(path)

    def close(self):
        """녹화 중이면 파일을 닫는다(도구 종료·녹화 끄기)."""
        with self.lock:
            if self.rec is None:
                return
            self.rec.release()
            self.note(f'raw recording OFF ({self.rec_frames} frames) {self.rec_path.name}',
                      f'원본 녹화 끝: {self.rec_frames}프레임 → {self.rec_path}')
            self.rec = None

    def _save_photo(self, raw, quiet=False):
        import cv2
        self.photo_dir.mkdir(parents=True, exist_ok=True)
        self.saved += 1
        dpart = f'_d{int(self.typed):03d}' if self.typed else ''
        p = self.photo_dir / f'{self.a.prefix}{dpart}_{datetime.now():%H%M%S}_{self.saved:04d}.jpg'
        cv2.imwrite(str(p), raw)
        if not quiet:
            print('사진 저장:', p)
        return p

    def _log_row(self):
        if not self.typed:
            self.note('type true distance (cm) first')
            return self.msg
        cam, typed = self.cam, self.typed
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not self.log_path.exists()
        with self.log_path.open('a', newline='') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(['time', 'true_cm', 'kind', 'id_or_class', 'conf', 'x1', 'y1', 'x2', 'y2',
                            'floor_cm', 'bearing_deg', 'width_cm', 'feet_cut', 'by_height_cm',
                            'card_cm', 'card_side_px', 'cam_h_cm', 'cam_pitch_deg'])
            now = datetime.now().isoformat(timespec='seconds')
            base = [cam.height_m * 100, cam.pitch_deg]
            for d, g, cut, dh in self.rows:
                w.writerow([now, typed, 'yolo', d.name, f'{d.conf:.3f}', *[f'{v:.1f}' for v in d.box],
                            f'{g.dist_m * 100:.1f}' if g else '', f'{g.bearing_deg:.1f}' if g else '',
                            f'{g.width_m * 100:.1f}' if g else '', int(cut),
                            f'{dh * 100:.1f}' if dh else '', '', '', *base])
            for c in self.cards:
                x1, y1 = c.corners.min(axis=0)
                x2, y2 = c.corners.max(axis=0)
                w.writerow([now, typed, 'card', c.id, '', f'{x1:.1f}', f'{y1:.1f}', f'{x2:.1f}', f'{y2:.1f}',
                            '', '', '', '', '', f'{c.dist_m * 100:.1f}', f'{c.side_px:.1f}', *base])
            if not self.rows and not self.cards:
                w.writerow([now, typed, 'none', '', '', '', '', '', '', '', '', '', '', '', '', '', *base])
        self.note(f'logged true {typed}cm: {len(self.rows)} yolo, {len(self.cards)} card -> {self.log_path.name}',
                  f'기록: 줄자 {typed} cm, YOLO {len(self.rows)}개, 카드 {len(self.cards)}개 → {self.log_path}')
        self.typed = ''
        return self.msg

    # ---- 웹 상태
    def _follow_uses_dolls(self):
        """인형으로 잇기가 켜져 있고 YOLO 가 도는가(카드 처리 순서를 YOLO 뒤로 미룸)."""
        return (self.follow is not None and self.obstacles is not None
                and getattr(self.follow.tracker.cfg, 'doll_fallback', False))

    def _adult_points(self, dets):
        from .obstacles import doll_obstacles
        return [(o.x_m, o.y_m) for o in doll_obstacles(self.cam, dets, self.obstacles.cfg) if o.name == 'adult']

    def _forget_dolls_in_line_mode(self):
        """로봇이 라인 모드(N12 전환 전)면 인형 기억을 비운다. 라인 주행은 비용 지도를 쓰지 않고, 시험을 다시 하려고
        로봇을 손으로 N08 에 옮기면 odom 이 그대로라 옛 기억이 로봇 옆 엉뚱한 자리에 남는다(11-21 실로봇)."""
        mem = getattr(self.obstacles, 'memory', None)
        if mem is None or self.demo is None:
            return
        observed = ((self.demo.info().get('robot') or {}).get('observed') or {})
        if observed.get('mode') == 'line' and mem.entries:
            mem.clear()

    def state(self):
        with self.lock:
            cam = self.cam
            n_calib = len(list(self.calib_dir.glob('*.png'))) if self.calib_dir.exists() else 0
            return {
                'fps': round(self.fps, 1), 'lens_height_cm': self.a.lens_height_cm or None, 'typed': self.typed, 'show': dict(self.show),
                'board_found': self.board_found, 'calib_images': n_calib,
                'tape': [{'u': u, 'v': v, 'cm': round(d * 100)} for u, v, d in self.tape],
                'camera': {'source': cam.source, 'height_cm': round(cam.height_m * 100, 2),
                           'pitch_deg': round(cam.pitch_deg, 2), 'roll_deg': round(cam.roll_deg, 2),
                           'fx': round(float(cam.K[0, 0]), 1), 'file': str(Path(self.a.camera).expanduser())},
                'pending': self.last_result if self.last_floor is not None else None,
                'click': [{'u': x, 'v': y, 'x_cm': None if g is None else round(g[0] * 100, 1),
                           'y_cm': None if g is None else round(g[1] * 100, 1)} for x, y, g in self.clicks],
                'yolo': [{'name': d.name, 'conf': round(d.conf, 2),
                          'dist_cm': None if g is None else round(g.dist_m * 100, 1), 'feet_cut': bool(cut)}
                         for d, g, cut, _ in self.rows],
                'cards': [{'id': c.id, 'dist_cm': round(c.dist_m * 100, 1), 'side_px': round(c.side_px)}
                          for c in self.cards],
                'msg': self.msg if time.time() - self.msg_t < 6 else '',
                'saved_photos': self.saved, 'log': str(self.log_path), 'photo_dir': str(self.photo_dir),
                'burst': self.burst, 'burst_count': self.burst_count, 'burst_sec': self.burst_sec,
                'rec': self.rec is not None, 'rec_frames': self.rec_frames,
                'rec_path': str(self.rec_path) if self.rec_path else None,
                'error': getattr(self.src, 'error', None),
                'obstacles': None if self.obstacles is None else self.obstacles.info(),
                'follow': None if self.follow is None else self.follow.info(),
                'demo': None if self.demo is None else self.demo.info(),
                'where': None if self.loc_view is None else self.loc_view.info(),
            }

    def demo_send(self, body):
        """시험 화면 버튼 → 로봇 demo_runner. 꺼져 있거나 요청이 틀리면 ValueError."""
        if self.demo is None:
            raise ValueError('robot commands are off (--demo)')
        msg = self.demo.send(body)
        self.note(f'demo {msg["op"]} sent', f'로봇에 보냄: {msg["op"]}')
        return msg
