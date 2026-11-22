"""FFT1-186 0단계 도구(노트북에서 실행, 로봇은 카메라만 씀 — bringup·모터 명령 없음).

  PY=~/dev_ws/roport/venvs/follow/bin/python
  $PY -m roport_follow.tool view --robot <로봇 IP>            # 실시간 화면·거리 격자·YOLO·카드·사진 저장
  $PY -m roport_follow.tool calib-intrinsics --images ~/roport_follow/calib/6422
  $PY -m roport_follow.tool calib-floor --image <바닥 체스보드 사진>
  $PY -m roport_follow.tool latency --robot <로봇 IP>          # 화면 시계 마커로 영상 지연
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from .floor_geom import CameraModel
from .tool_draw import YELLOW, draw_cards, draw_dets, draw_grid, put  # noqa: F401  (tool 밖에서도 씀)

DEFAULT_CAMERA = '~/roport_follow/camera_6422.json'
DEFAULT_OUT = '~/roport_follow'

KEYS = """키: y YOLO  a 카드  b 체스보드  c 체스보드 사진 저장(렌즈 보정용)  f 바닥 체스보드로 자세 계산
     줄자 맞춤: 숫자(cm) 입력 → 줄자 표시 클릭(여러 개) → h 맞춤, x 표본 지움
     w 계산한 자세 저장   s 원본 사진 저장   r 연속 저장(--burst-sec 간격)   v 원본 영상 녹화(~/roport_follow/video)   숫자 + Enter = 측정 기록(CSV), Backspace 지움
     n/p 사진 폴더 넘김(--source 폴더)  q·Esc 끝"""


def load_camera(path):
    p = Path(path).expanduser()
    if p.exists():
        cam = CameraModel.load(p)
        print(f'카메라 값: {p} ({cam.source})')
    else:
        cam = CameraModel()
        print(f'카메라 값 파일 없음 → 기본 추정값 사용(초점 540 px, 높이 6.3 cm, 숙임 {cam.pitch_deg}°). '
              f'보정 뒤 {p} 에 저장됨')
    return cam


def open_source(a):
    from .stream import LocalStream, RobotStream
    if a.robot:
        print(f'로봇 {a.robot} 카메라 연결(SSH, 카메라만 사용)…')
        return RobotStream(a.robot, key=a.key, fps=a.fps, quality=a.quality, rotate=not a.no_rotate,
                           exposure_us=getattr(a, 'cam_exposure_us', 0), gain=getattr(a, 'cam_gain', 0.0))
    if a.source is not None:
        return LocalStream(a.source, fps=a.fps)
    sys.exit('--robot <IP> 또는 --source <카메라번호|동영상|사진폴더> 가 필요함')


def cmd_view(a):
    import cv2
    from .session import ViewSession
    cam = load_camera(a.camera)
    pub = make_obstacle_sink(a, cam)
    src = open_source(a)
    ses = ViewSession(a, cam, src)
    cv2.namedWindow('roport_follow', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('roport_follow', lambda ev, x, y, *_: ses.click(x, y) if ev == cv2.EVENT_LBUTTONDOWN else None)
    print(KEYS)
    keymap = {10: 'enter', 13: 'enter', 8: 'backspace', 127: 'backspace'}
    try:
        while True:
            img = ses.step()
            if img is None:
                if getattr(src, 'error', None):
                    print('영상 끊김:', src.error)
                    break
                if cv2.waitKey(30) & 0xFF in (27, ord('q')):
                    break
                continue
            cv2.imshow('roport_follow', img)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break
            if k in (ord('n'), ord('p')) and hasattr(src, 'step'):
                src.step(1 if k == ord('n') else -1)
            elif k in keymap:
                ses.key(keymap[k])
            elif k != 255 and chr(k) in '0123456789yabcfhxwsrv':
                ses.key(chr(k))
    finally:
        ses.close()
        src.close()
        cv2.destroyAllWindows()
        if pub is not None:
            pub.close()


def cmd_calib_intrinsics(a):
    import cv2
    from .calib import calibrate_intrinsics, find_corners
    files = sorted(p for p in Path(a.images).expanduser().iterdir() if p.suffix.lower() in ('.png', '.jpg', '.jpeg'))
    sets, size, used = [], None, []
    for p in files:
        g = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        size = (g.shape[1], g.shape[0])
        c = find_corners(g)
        if c is not None:
            sets.append(c)
            used.append(p.name)
    print(f'체스보드 찾음 {len(sets)}/{len(files)}장')
    k, dist, rms = calibrate_intrinsics(sets, size, fix_k3=not a.free_k3)
    cam = load_camera(a.camera)
    cam.k, cam.dist, cam.width, cam.height = k.tolist(), dist.tolist(), size[0], size[1]
    cam.source = f'intrinsics {len(sets)} views rms {rms:.3f}px {datetime.now():%Y-%m-%d}' + (
        '' if 'floor' in cam.source else ' (height/pitch = default estimate)')
    cam.save(Path(a.camera).expanduser(), extra={'intrinsics_images': used})
    print(f'렌즈 보정 rms {rms:.3f} px (1 px 넘으면 사진을 다시 고를 것)\n초점 fx {k[0, 0]:.1f} fy {k[1, 1]:.1f} '
          f'주점 ({k[0, 2]:.1f}, {k[1, 2]:.1f})\n저장: {Path(a.camera).expanduser()}')


def cmd_calib_floor(a):
    import cv2
    from .calib import find_corners, floor_pose
    cam = load_camera(a.camera)
    g = cv2.imread(str(Path(a.image).expanduser()), cv2.IMREAD_GRAYSCALE)
    if g is None:
        sys.exit(f'사진을 못 읽음: {a.image}')
    c = find_corners(g)
    if c is None:
        sys.exit('체스보드를 못 찾음(보드 전체가 보이게, 9×6 안쪽 모서리)')
    fp = floor_pose(c, cam.K, cam.D, a.board_thickness_mm / 1000)
    print(f'바닥 자세: 높이 {fp.height_m * 100:.2f} cm, 숙임 {fp.pitch_deg:.2f}°, roll {fp.roll_deg:+.2f}°, '
          f'재투영 {fp.reproj_px:.2f} px, 보드까지 {fp.board_dist_m * 100:.1f} cm')
    if a.dry_run:
        return
    cam.height_m, cam.pitch_deg, cam.roll_deg = fp.height_m, fp.pitch_deg, fp.roll_deg
    cam.source = f'floor chessboard {datetime.now():%Y-%m-%d %H:%M}' + (
        '' if 'intrinsics' in cam.source else ' (lens = default estimate)')
    cam.save(Path(a.camera).expanduser())
    print('저장:', Path(a.camera).expanduser())


def cmd_latency(a):
    import cv2
    from .markers import ClockReader, clock_image, latency_ms, percentile
    src = open_source(a)
    reader = ClockReader()
    vals, last_seq, t_end = [], None, time.time() + a.seconds
    cv2.namedWindow('clock (show to robot camera)', cv2.WINDOW_NORMAL)
    cv2.namedWindow('robot view', cv2.WINDOW_NORMAL)
    print('노트북 화면의 「clock」 창을 로봇 카메라 앞 20~40 cm에 보이게 두세요.')
    try:
        while time.time() < t_end:
            cv2.imshow('clock (show to robot camera)', clock_image(time.time() * 1000))
            fr = src.latest()
            if fr is not None and fr.seq != last_seq:
                last_seq = fr.seq
                seen = reader(fr.image)
                if seen is not None:
                    vals.append(latency_ms(seen, fr.recv_time * 1000))
                view = fr.image.copy()
                put(view, f'samples {len(vals)}  median {percentile(vals, 50):.0f}ms', (6, 20), YELLOW)
                cv2.imshow('robot view', view)
            elif fr is None and getattr(src, 'error', None):
                print('영상 끊김:', src.error)
                break
            if cv2.waitKey(5) & 0xFF in (27, ord('q')):
                break
    finally:
        src.close()
        cv2.destroyAllWindows()
    if vals:
        print(f'지연(화면 → 노트북 수신, ±40 ms): 표본 {len(vals)}, 중앙 {percentile(vals, 50):.0f} ms, '
              f'90% {percentile(vals, 90):.0f} ms, 최대 {max(vals):.0f} ms')
    else:
        print('시계 마커를 한 번도 못 읽음(화면 밝기·거리·초점 확인)')


def _arg(*args, **kw):
    return args, kw


VIEW_ARGS = [
    _arg('--camera', default=DEFAULT_CAMERA),
    _arg('--name', default='6422', help='기체 이름(저장 폴더)'),
    _arg('--out', default=DEFAULT_OUT),
    _arg('--save-dir', help='s 로 저장할 폴더(기본 ~/roport_follow/photos/<name>)'),
    _arg('--prefix', default='img', help='저장 파일 앞부분(예: dollA_gate_ceil)'),
    _arg('--burst-sec', type=float, default=0.5, help='연속 저장(r) 간격 초(최소 0.2)'),
    _arg('--yolo', action='store_true', help='시작부터 YOLO 켬'),
    _arg('--model', default='~/dev_ws/roport/models/doll_ft_20261119/best.pt',
         help='기본 = 인형 모델(adult·child·human). COCO 는 ~/dev_ws/roport/models/yolo11n.pt'),
    _arg('--conf', type=float, default=0.25),
    _arg('--torch-threads', type=int, default=0,
         help='YOLO CPU 스레드 수(0 = torch 기본 = 전부). 11-19 측정상 노트북은 4 가 가장 빠름(실로봇 확인 전)'),
    _arg('--doll-height-cm', type=float, default=0.0, help='키로 거리 계산(비교용)'),
    _arg('--card-mm', type=float, default=30.0, help='카드 마커 검은 사각형 한 변(mm)'),
    _arg('--board-thickness-mm', type=float, default=0.0, help='바닥 체스보드 판 두께'),
    _arg('--lens-height-cm', type=float, default=0.0,
                   help='줄자로 잰 렌즈 높이(주면 줄자 맞춤에서 고정, 점 1개로도 맞춤)'),
    # 인형 장애물 → Nav2 비용 지도(FFT1-187). ROS 환경을 source 한 터미널에서만 켤 수 있음
    _arg('--publish-obstacles', action='store_true',
         help='인형(adult·child) 바닥 점을 PointCloud2 로 보냄(모터·Nav2 목표는 안 보냄)'),
    _arg('--obstacle-topic', default='/doll_obstacles'),
    _arg('--cam-x-cm', type=float, default=3.5, help='바퀴 축 가운데 → 렌즈 앞 거리(URDF 약 3.5 cm)'),
    _arg('--obstacle-conf', type=float, default=0.35, help='이 신뢰도 미만 상자는 장애물로 안 씀'),
    _arg('--nav2-file', default='~/roport_follow/nav2_overrides_doll.yaml',
         help='로봇에 넘긴 Nav2 덮어쓰기 파일의 노트북 사본(토픽·거리 일치 확인용, 읽기만)'),
    _arg('--max-range-cm', type=float, default=120.0,
         help='바퀴 축에서 이보다 먼 인형은 안 보냄. nav2_doll --max-range-cm 과 반드시 같게(다르면 지움 점이 벽으로 칠해짐)'),
    # 움직이는 인형 예측(FFT1-190): /odom 으로 로봇 움직임을 빼고 인형 속도를 구해 앞으로 갈 자리도 장애물로 보냄
    _arg('--predict-moving', action='store_true', help='움직이는 인형의 예상 자리(horizon 초 뒤까지)도 보냄(--publish-obstacles 필요)'),
    _arg('--predict-horizon-s', type=float, default=1.0),
    # 인형 기억(11-21): 본 인형을 노트북이 기억해 매번 다시 보내고 로봇 둘레를 지움(카메라 사각에 든 인형 유지,
    # 잘못 찍힌 점은 스스로 사라짐). 로봇 Nav2 인형 층은 min range 0·persistence 0 이어야 함(nav2_doll 기본값)
    _arg('--no-doll-memory', dest='doll_memory', action='store_false',
         help='인형 기억을 끔(예전 방식: 로봇 인형 층 keep-near 15 cm·persistence 1 s 와 짝)'),
    _arg('--odom-topic', default='/odom'),
    # 고객 카드 추종 대상(FFT1-188). 0 = 끔. ROS 환경을 source 한 터미널에서만 켤 수 있음
    _arg('--follow-card', type=int, default=0,
         help='따라갈 고객 카드 ID(35~49). 주면 /follow/target·/follow/status 를 보냄(모터·Nav2 목표는 안 보냄)'),
    _arg('--follow-lost-s', type=float, default=1.0,
         help='이 시간 넘게 카드를 못 보면 놓침(lost). 기본 1 s = SR-22. 실로봇 시연은 4(인형을 피해 가는 동안 버팀, '
              '로봇 follow.requested 의 lost_after_s 와 같은 값으로)'),
    _arg('--follow-max-range-cm', type=float, default=150.0, help='이보다 먼 카드는 안 씀'),
    _arg('--follow-doll-fallback', action='store_true',
         help='(임시) 카드가 안 보이는 동안 마지막 고객 자리 근처의 어른 인형으로 이어 추적(최대 8 s, --publish-obstacles 필요)'),
    _arg('--map-yaml', default='~/roport_follow/maps_v16/airport_map.yaml',
         help='시험 화면 「로봇이 아는 내 위치」에 그릴 지도(로봇 Nav2 와 같은 파일). --demo 일 때만 씀'),
    _arg('--demo', action='store_true',
         help='시험용: 화면 버튼(라인 출발·Nav2 전환·추종 시작·정지)을 로봇 demo_runner 로 보냄(관제 대신)'),
]


def make_ros_sinks(a, cam):
    """--publish-obstacles·--follow-card 이면 ROS 노드 하나를 만들어 두 발행을 함께 쓴다. 닫을 노드를 돌려준다."""
    a.obstacle_sink = a.follow_sink = None
    a.demo_link = None
    if not (getattr(a, 'publish_obstacles', False) or getattr(a, 'follow_card', 0) or getattr(a, 'demo', False)):
        return None
    from .ros_obstacles import make_ros_node
    node = make_ros_node('roport_follow')
    a.odom_buffer = None
    if getattr(a, 'publish_obstacles', False) and (getattr(a, 'predict_moving', False)
                                                 or getattr(a, 'doll_memory', True)):
        from .ros_obstacles import OdomBuffer
        a.odom_buffer = OdomBuffer(node, topic=a.odom_topic)
    if getattr(a, 'demo', False):
        from .demo_link import DemoLink
        a.demo_link = DemoLink(node)
        a.loc_view = None
        if Path(a.map_yaml).expanduser().is_file():
            from .localization_view import LocalizationView
            a.loc_view = LocalizationView(node, a.map_yaml)
            print(f'로봇이 아는 내 위치: {a.map_yaml} 위에 TF map→base_footprint·/scan·지역 비용 지도(읽기만)')
        else:
            print(f'주의: 지도 {a.map_yaml} 없음 → 「로봇이 아는 내 위치」 화면 끔')
        print('시험용 로봇 명령: demo/cmd → 로봇 demo_runner(관제 대신). 브라우저가 1 s 넘게 안 보면 로봇이 멈춤')
    if a.odom_buffer is not None or a.demo_link is not None:
        from .ros_obstacles import RosSpinner
        a.ros_spinner = RosSpinner(node)       # 구독(odom·demo/state)을 받으려면 노드를 돌려야 함
    make_obstacle_sink(a, cam, node)
    make_follow_sink(a, cam, node)
    return node


def make_follow_sink(a, cam, node):
    a.follow_sink = None
    if not getattr(a, 'follow_card', 0):
        return
    from .follow import FollowConfig
    from .ros_follow import FollowSink, RosFollowPublisher
    cfg = FollowConfig(card_id=a.follow_card, cam_x_m=a.cam_x_cm / 100, lost_after_s=a.follow_lost_s,
                       max_range_m=a.follow_max_range_cm / 100,
                       doll_fallback=bool(getattr(a, 'follow_doll_fallback', False)))
    pub = RosFollowPublisher(node)
    a.follow_sink = FollowSink(cam, cfg, pub, require_robot_time=bool(a.robot),
                               pose_lookup=getattr(a, 'odom_buffer', None))
    if cfg.doll_fallback:
        print(f'(임시) 인형으로 잇기: 카드가 안 보이면 마지막 고객 자리 {cfg.fb_gate_m * 100:g}~{cfg.fb_gate_max_m * 100:g} cm 안의 '
              f'어른 인형으로 최대 {cfg.fb_max_s:g} s 이어 추적')
    a.follow_sink.meta = {'card_mm': a.card_mm, 'target_topic': pub.target_topic, 'status_topic': pub.status_topic,
                          'frame': 'base_footprint', 'domain': os.environ.get('ROS_DOMAIN_ID', '0')}
    print(f'추종 대상 보내기: 카드 ID {a.follow_card}({a.card_mm:g} mm) → {pub.target_topic}·{pub.status_topic} '
          f'(base_footprint, {a.follow_lost_s:g} s 못 보면 lost, {a.follow_max_range_cm:g} cm 안)')


def make_obstacle_sink(a, cam, node):
    """--publish-obstacles 이면 a.obstacle_sink 를 만든다(node 는 make_ros_sinks 가 만든 공용 노드)."""
    a.obstacle_sink = None
    if not getattr(a, 'publish_obstacles', False):
        return
    from .obstacles import ObstacleConfig, blind_distance
    from .ros_obstacles import ObstacleSink, RosCloudPublisher
    cfg = ObstacleConfig(cam_x_m=a.cam_x_cm / 100, min_conf=a.obstacle_conf,
                         max_range_m=a.max_range_cm / 100, clear_range_m=a.max_range_cm / 100 + 0.05)
    if Path(a.camera).expanduser() == Path(DEFAULT_CAMERA).expanduser():
        print(f'주의: 카메라 값이 기본 {DEFAULT_CAMERA} 임. 스마트카트 시험 기체(d660)면 --camera ~/roport_follow/camera_d660.json')
    pub = RosCloudPublisher(topic=a.obstacle_topic, node=node)
    motion = memory = None
    if getattr(a, 'doll_memory', True) and getattr(a, 'odom_buffer', None) is not None:
        from .doll_memory import DollMemory
        from .obstacles import clear_fov_deg
        left, right = clear_fov_deg(cam, cfg)
        memory = DollMemory(fov_left_deg=left + cfg.fov_margin_deg, fov_right_deg=right + cfg.fov_margin_deg,
                            cam_x_m=cfg.cam_x_m, cam_y_m=cfg.cam_y_m)
        print(f'인형 기억: 본 인형을 {a.odom_topic} 기준으로 기억해 매번 다시 보내고 로봇 둘레를 지움 '
              f'(로봇 Nav2 인형 층은 min range 0·persistence 0 이어야 함)')
    elif getattr(a, 'doll_memory', True):
        print('주의: 인형 기억을 켤 수 없음(odom 구독 없음) → 예전 방식으로 보냄')
    if getattr(a, 'predict_moving', False) and getattr(a, 'odom_buffer', None) is not None:
        from .moving import MovingConfig, MovingTracker
        motion = MovingTracker(MovingConfig(horizon_s=a.predict_horizon_s))
        print(f'움직이는 인형 예측: {a.odom_topic} 로 로봇 움직임을 빼고 {a.predict_horizon_s:g} s 뒤 자리까지 보냄')
    a.obstacle_sink = ObstacleSink(cam, cfg, pub, require_robot_time=bool(a.robot),
                                   motion=motion, pose_lookup=getattr(a, 'odom_buffer', None), memory=memory)
    a.yolo = True       # 시험 중에는 YOLO 상자를 기본으로 보여 줌(y 로 끌 수 있고, 꺼도 장애물은 계속 보냄)
    from .nav2_doll import check_overrides
    nav2 = check_overrides(a.nav2_file, a.obstacle_topic, cfg.max_range_m)
    if nav2['status'] != 'same':
        print(f'주의: Nav2 덮어쓰기 파일 확인 = {nav2["status"]} ({nav2["detail"]}). 로봇에 넘긴 파일과 같은지 볼 것')
    a.obstacle_sink.meta = {
        'camera': Path(a.camera).expanduser().name, 'model': '/'.join(Path(a.model).expanduser().parts[-2:]),
        'topic': a.obstacle_topic, 'frame': 'base_footprint', 'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
        'source': f'robot {a.robot}' if a.robot else f'local {a.source}', 'nav2': nav2,
        'torch_threads': getattr(a, 'torch_threads', 0) or 'default'}
    blind = blind_distance(cam)
    print(f'인형 장애물 보내기: {a.obstacle_topic} (base_footprint, 렌즈 {a.cam_x_cm:g} cm 앞, '
          f'{a.max_range_cm:g} cm 안, 신뢰도 {a.obstacle_conf:g} 이상, 카메라 사각 = 렌즈 앞 '
          f'{"?" if blind is None else f"{blind * 100:.0f}"} cm)')


def cmd_web(a):
    from .session import ViewSession
    from .web import serve
    if a.port < 1024 or a.port > 65535:
        sys.exit('--port 1024~65535')
    if a.bind not in ('127.0.0.1', 'localhost'):
        print('주의: --bind', a.bind, '→ 같은 망의 누구나 버튼을 누를 수 있음(인증 없음)')
    cam = load_camera(a.camera)
    node = make_ros_sinks(a, cam)
    src = open_source(a)
    try:
        serve(ViewSession(a, cam, src), src, a.bind, a.port)
    finally:
        if node is not None:
            from .ros_obstacles import close_ros_node
            if getattr(a, 'demo_link', None) is not None:
                a.demo_link.close()
            if getattr(a, 'ros_spinner', None) is not None:
                a.ros_spinner.close()
            close_ros_node(node)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='roport_follow.tool', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def add_source(p):
        p.add_argument('--robot', help='로봇 IP(SSH 로 카메라만 씀)')
        p.add_argument('--source', help='로봇 대신: USB 카메라 번호·동영상·사진 폴더')
        p.add_argument('--key', default='~/.ssh/pinky_ed25519')
        p.add_argument('--fps', type=float, default=10.0)
        p.add_argument('--quality', type=int, default=80)
        p.add_argument('--cam-exposure-us', type=int, default=0,
                       help='로봇 카메라 노출 고정(µs, 0 = 자동). 짧으면 회전 중 번짐이 줄고 영상이 어두워짐')
        p.add_argument('--cam-gain', type=float, default=0.0, help='노출 고정 때 아날로그 게인(0 = 자동)')
        p.add_argument('--no-rotate', action='store_true', help='로봇 영상 180° 회전 안 함')

    v = sub.add_parser('view', help='실시간 화면·거리 격자·YOLO·카드·사진 저장·측정 기록')
    add_source(v)
    for args, kw in VIEW_ARGS:
        v.add_argument(*args, **kw)
    v.set_defaults(func=cmd_view)

    ci = sub.add_parser('calib-intrinsics', help='체스보드 사진 폴더 → 렌즈 보정')
    ci.add_argument('--images', required=True)
    ci.add_argument('--camera', default=DEFAULT_CAMERA)
    ci.add_argument('--free-k3', action='store_true', help='세 번째 왜곡 계수도 계산(사진이 화면 전체에 고르게 있을 때만)')
    ci.set_defaults(func=cmd_calib_intrinsics)

    cf = sub.add_parser('calib-floor', help='바닥에 둔 체스보드 사진 → 카메라 높이·숙임')
    cf.add_argument('--image', required=True)
    cf.add_argument('--camera', default=DEFAULT_CAMERA)
    cf.add_argument('--board-thickness-mm', type=float, default=0.0)
    cf.add_argument('--dry-run', action='store_true')
    cf.set_defaults(func=cmd_calib_floor)

    we = sub.add_parser('web', help='브라우저 화면(실시간 영상·체스보드 검출 상태·버튼)')
    add_source(we)
    for args, kw in VIEW_ARGS:
        we.add_argument(*args, **kw)
    we.add_argument('--bind', default='127.0.0.1', help='기본 노트북 안에서만. 0.0.0.0 은 같은 망 누구나 조작 가능')
    we.add_argument('--port', type=int, default=8090)
    we.set_defaults(func=cmd_web)

    la = sub.add_parser('latency', help='화면 시계 마커로 로봇 → 노트북 영상 지연 측정')
    add_source(la)
    la.add_argument('--seconds', type=float, default=20.0)
    la.set_defaults(func=cmd_latency)

    a = ap.parse_args(argv)
    a.func(a)


if __name__ == '__main__':
    main()
