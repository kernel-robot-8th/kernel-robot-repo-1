"""로봇에서 실행: 노트북 시험 화면 버튼 → TrialDriver(roport_pinky). 관제 MQTT 수신기 자리를 대신하는 시험용.

관제(roport-control-v2)에 추종 명령이 생기기 전까지만 쓴다. 최종 구조는 고객 폰·패드 → 서비스 서버 → 관제 → 로봇.
TrialDriver·trial_guard(구동권 문지기)는 그대로 쓰고, 관제 키트 runner 와 같은 순서로 부른다
(명령 검사 validate_action → dispatch, 0.1 s 마다 heartbeat → drain_results).

  ROS 입력  demo/cmd   std_msgs/String JSON {"op": ..., "seq": n, ...}
            op = ping | route {"route": [...], "speed_limit_mps"} | switch_nav2 | switch_line(N12 에서 라인 복귀)
                 | navigate {"goal_id", "x", "y", "yaw", "position_tolerance_m"(선택)} (map 좌표 m·rad, Nav2 모드)
                 | follow {"card_id", "standoff_m", "max_duration_s", "lost_after_s", "turn_first"} | stop
                   turn_first = true 면 지금 자리에서 180° 돈 뒤 추종을 시작한다(ARR 에서 마커를 보고 선 카트가
                   뒤에 있는 고객 쪽으로 돌아섬).
                 | alert {"on": bool} (탑승 시간 알림 켜기·끄기. 주행 명령이 아님)
  ROS 출력  demo/state std_msgs/String JSON 2 Hz: 드라이버 관측·진행 중 명령·최근 결과·콘솔 연결
            smartcart/boarding_alert std_msgs/Bool 2 Hz → roport_line_nav status_indicator(smartcart:=true)가
            LCD·LED·부저로 알림. 언제 켤지(탑승 시각 기준)는 명령을 보내는 쪽이 정한다.
안전: 콘솔 신호(ping 또는 명령)가 console_timeout_s(1 s) 넘게 없으면 heartbeat 를 멈추고(구동권이 1 s 뒤 닫힘)
      진행 중 명령이 있으면 safe_stop('control_link_lost'). 모터 bringup·Nav2 실행은 하지 않는다(사람이 함).

  실행(로봇, ROS·trial.launch 이미 실행 중):
  ros2 run roport_follow demo_runner --map ~/roport_follow/maps_v16/airport_map.yaml \\
      --layout ~/roport_follow/roport_airport_v16.yaml --start-node N08
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import threading
import time
import uuid
from pathlib import Path

OPS = ('ping', 'route', 'switch_nav2', 'switch_line', 'navigate', 'follow', 'stop', 'alert')
PATIENT_BT = Path(__file__).parent / 'bt' / 'navigate_patient.xml'
# Nav2 목표 도착 판정(드라이버). Nav2 goal checker(xy 0.08 m)보다 넉넉히: 시험 목표 좌표가 실측 전 후보값이라
NAV_POSITION_TOL_M = 0.12
# 명령에 position_tolerance_m 을 주면 이 범위 안에서 그 값을 쓴다. 목표가 인형·고객 둘레에 걸리면 플래너가
# planner_tolerance(0.3 m) 안의 가까운 자리로 경로를 끝낸다(11-22 d660: ARR 목표에서 12.5 cm 에 서서 Nav2 는 성공인데
# 0.12 기준으로 nav2_goal_not_reached 실패) → ARR·Gate 처럼 고객이 있는 목표만 보내는 쪽이 넓힌다.
NAV_POSITION_TOL_RANGE_M = (0.05, 0.30)
NAV_YAW_TOL_RAD = 0.35
# 추종 전 돌아서기(turn_first): 같은 자리·반대 방향 Nav2 목표. 돈 뒤 카메라 영상이 노트북을 거쳐 카드 위치로
# 돌아오기까지(실측 0.3~0.5 s + 6 fps) 기다렸다가 추종을 보낸다. 드라이버는 방금 본 카드로만 추종을 시작한다.
TURN_GOAL_ID = 'demo_TURN'
TURN_SETTLE_S = 1.5
HEADING = {'E': 0.0, 'N': math.pi / 2, 'W': math.pi, 'S': -math.pi / 2}


def map_version(path):
    """관제와 같은 방식: sha256(지도 yaml + 이미지)[:16]."""
    import yaml
    p = Path(path).expanduser().resolve()
    img = p.parent / yaml.safe_load(p.read_text())['image']
    return hashlib.sha256(p.read_bytes() + img.read_bytes()).hexdigest()[:16]


def make_profile(map_path, layout_path):
    import yaml
    frame = yaml.safe_load(Path(layout_path).expanduser().read_text())['frame']
    return {'map_id': Path(map_path).expanduser().stem, 'map_version': map_version(map_path),
            'frame_yaw_rad': math.radians(float(frame.get('yaw_deg', 0.0)))}


class DemoRunner:
    """ROS 없는 판정부. driver = TrialDriver 와 같은 hook(observe·validate_action·dispatch·safe_stop·
    drain_results·heartbeat)을 가진 객체, graph = roport_line_nav Graph(경로 시작 방향 계산)."""

    def __init__(self, driver, profile, graph, *, console_timeout_s=1.0, now=time.monotonic):
        self.driver, self.profile, self.graph = driver, profile, graph
        self.timeout, self.now = float(console_timeout_s), now
        self.console_at = None
        self.active = None              # 진행 중 명령 dict
        self.events = collections.deque(maxlen=20)
        self.link_stopped = False
        self.boarding = False           # 탑승 시간 알림(표시·소리만, 주행과 무관)
        self.after_turn = None          # 돌아선 뒤 보낼 follow 명령
        self.turned_at = None
        self.lock = threading.Lock()

    # ---- 명령 만들기(관제 trial-v1 과 같은 모양)
    def _base(self, kind, payload):
        return {'type': kind, 'command_id': str(uuid.uuid4()), 'payload': payload}

    def _route(self, msg):
        route = [str(n) for n in msg.get('route') or []]
        if len(route) < 2 or any(n not in self.graph.nodes for n in route):
            raise ValueError('invalid_route')
        speed = float(msg.get('speed_limit_mps', 0.06))
        if not (0.01 <= speed <= 0.2):
            raise ValueError('invalid_speed')
        heading = HEADING[self.graph.heading(route[0], route[1])] + self.profile['frame_yaw_rad']
        loading = self._loading_heading(route[0])
        if loading is not None:
            # 상차 대기(N01·N11·N21·N22, 반시계 90° 돈 자세)에서 출발: 드라이버는 지금 자세를 출발 방향으로 받고
            # load_done 으로 진입선으로 돌린 뒤 출발한다(관제 trial-v1 과 같음). 경로 방향을 넣으면 loading_heading_mismatch.
            heading = loading
        return self._base('route.requested', {
            'graph_version': self.graph.version, 'start_node': route[0], 'route': route,
            'start_heading_rad': heading, 'start_maneuver': 'none', 'speed_limit_mps': speed})

    def _loading_heading(self, start):
        logic = getattr(self.driver, 'logic', None)
        st = getattr(logic, 'st', None)
        if not st or st.get('state') != 'loading' or not st.get('route') or st['route'][-1] != start:
            return None
        return float(self.driver.observe()['heading_rad'])

    def _navigate(self, msg):
        goal_id = str(msg.get('goal_id') or '')
        x, y, yaw = (float(msg[k]) for k in ('x', 'y', 'yaw'))
        if not goal_id or not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('invalid_goal')
        tol = float(msg.get('position_tolerance_m', NAV_POSITION_TOL_M))
        if not (NAV_POSITION_TOL_RANGE_M[0] <= tol <= NAV_POSITION_TOL_RANGE_M[1]):
            raise ValueError('invalid_position_tolerance')
        return self._base('navigate.requested', {
            'goal_id': goal_id, 'goal_version': 1, 'frame_id': 'map',
            'pose': {'x': x, 'y': y, 'yaw': math.atan2(math.sin(yaw), math.cos(yaw))},
            'position_tolerance_m': tol, 'yaw_tolerance_rad': NAV_YAW_TOL_RAD, **self._maps()})

    def _maps(self):
        return {'map_id': self.profile['map_id'], 'map_version': self.profile['map_version']}

    def build(self, msg):
        op = msg.get('op')
        if op == 'route':
            return self._route(msg)
        if op == 'switch_nav2':
            return self._base('mode_switch.requested', {'to_mode': 'nav2', **self._maps()})
        if op == 'switch_line':
            return self._base('mode_switch.requested', {'to_mode': 'line', **self._maps()})
        if op == 'navigate':
            return self._navigate(msg)
        if op == 'follow':
            p = {'card_id': int(msg['card_id']), 'standoff_m': float(msg.get('standoff_m', 0.5)), **self._maps()}
            if msg.get('max_duration_s') is not None:
                p['max_duration_s'] = float(msg['max_duration_s'])
            if msg.get('lost_after_s') is not None:
                p['lost_after_s'] = float(msg['lost_after_s'])
            return self._base('follow.requested', p)
        raise ValueError('unknown_op')

    def _turn(self):
        """지금 자리에서 반대 방향을 보는 Nav2 목표. 위치를 모르면(AMCL 준비 전) 거절."""
        snap = self.driver.nav.snapshot() if hasattr(self.driver, 'nav') else None
        pose = (snap or {}).get('pose')
        if not pose:
            raise ValueError('turn_pose_unavailable')
        return self._navigate({'goal_id': TURN_GOAL_ID, 'x': pose['x'], 'y': pose['y'], 'yaw': pose['yaw'] + math.pi})

    def _event(self, kind, **kw):
        self.events.append({'t': round(time.time(), 2), 'kind': kind, **kw})

    # ---- 입력(콘솔 메시지): ROS 콜백 스레드에서 불러도 됨
    def handle(self, msg):
        with self.lock:
            op = msg.get('op')
            if op not in OPS:
                self._event('rejected', op=op, reason='unknown_op')
                return
            if op == 'alert':
                # 표시 전용: 주행 연결의 살아 있음 신호로 세지 않는다(alert 만 계속 와도 연결 유실 정지가 동작해야 함)
                if not isinstance(msg.get('on'), bool):
                    self._event('rejected', op=op, reason='invalid_alert')
                    return
                self.boarding = msg['on']
                self._event('alert', on=self.boarding)
                return
            if self.console_at is not None and not self.console_alive():
                self._link_lost()           # 만료된 뒤(다음 tick 전)에 온 신호: 끊김 처리를 먼저 한다
            self.console_at = self.now()
            self.link_stopped = False
            if op == 'ping':
                return
            if op == 'stop':
                self.after_turn = self.turned_at = None
                if self.active is not None:
                    self.driver.safe_stop('operator_stop')
                    self._event('stopped', op=self.active['type'], reason='operator_stop')
                    self.active = None
                else:
                    self.driver.safe_stop('operator_stop')
                    self._event('stopped', reason='operator_stop')
                return
            self.after_turn = self.turned_at = None     # 새 명령은 결과와 상관없이 기다리던 추종을 버림
            try:
                cmd = self.build(msg)
                follow = None
                if op == 'follow' and msg.get('turn_first') is True:
                    follow, cmd = cmd, self._turn()
                self.driver.validate_action(cmd)
            except (ValueError, KeyError, TypeError) as e:
                self._event('rejected', op=op, reason=str(e) or type(e).__name__)
                return
            self.after_turn, self.turned_at = follow, None
            self._send(cmd)

    def _send(self, cmd):
        self.active = cmd
        self.driver.dispatch(cmd)
        self._event('accepted', op=cmd['type'], command_id=cmd['command_id'])

    def _link_lost(self):
        """콘솔 신호가 끊김(lock 안에서 부름): 기다리던 추종을 버리고 진행 중 명령을 멈춘다."""
        if self.after_turn is not None:     # 돈 뒤 기다리던 추종: 끊기면 새 구동을 내지 않음
            self._event('stopped', op='follow.requested', reason='control_link_lost')
            self.after_turn = self.turned_at = None
        if self.active is not None and not self.link_stopped:
            # 노트북 끊김: 구동권 갱신을 멈추고(guard 가 1 s 뒤 닫음) 진행 중 명령도 멈춘다
            self.driver.safe_stop('control_link_lost')
            self._event('stopped', op=self.active['type'], reason='control_link_lost')
            self.active, self.link_stopped = None, True

    def console_alive(self):
        return self.console_at is not None and self.now() - self.console_at <= self.timeout

    # ---- 0.1 s 마다(주 루프)
    def tick(self):
        with self.lock:
            alive = self.console_alive()
            if alive:
                self.driver.heartbeat()
            else:
                self._link_lost()
            if self.boarding and not self.console_alive():  # 알림을 켠 화면이 끊김: 끌 방법이 없으니 끈다
                self.boarding = False
                self._event('alert', on=False, reason='control_link_lost')
            for original, phase, details in self.driver.drain_results():
                self._event(phase, op=original.get('type'), **{k: v for k, v in details.items()
                                                                if isinstance(v, (str, int, float, bool, type(None)))})
                if self.active is not None and original.get('command_id') == self.active.get('command_id') \
                        and phase in ('completed', 'failed'):
                    self.active = None
                    if self.after_turn is not None:
                        if phase == 'completed':
                            self.turned_at = self.now()
                        else:
                            self.after_turn = None
            if self.after_turn is not None and self.turned_at is not None and alive and not self.link_stopped \
                    and self.now() - self.turned_at >= TURN_SETTLE_S and self.active is None:
                cmd, self.after_turn, self.turned_at = self.after_turn, None, None
                try:
                    self.driver.validate_action(cmd)
                except (ValueError, KeyError, TypeError) as e:
                    self._event('rejected', op='follow', reason=str(e) or type(e).__name__)
                else:
                    self._send(cmd)

    def state(self):
        with self.lock:
            try:
                obs = self.driver.observe()
            except Exception as e:  # noqa: BLE001 — 화면 표시용
                obs = {'fault': f'observe_failed:{type(e).__name__}'}
            return {'observed': {k: v for k, v in obs.items() if isinstance(v, (str, int, float, bool, type(None)))},
                    'active': None if self.active is None else {'type': self.active['type'],
                                                                'command_id': self.active['command_id']},
                    'pending': 'follow_after_turn' if self.after_turn is not None else None,
                    'console_alive': self.console_alive(), 'boarding': self.boarding, 'events': list(self.events),
                    'map_version': self.profile['map_version'], 'stamp': time.time()}


def apply_start_mode(driver, mode):
    """시험 편의(11-21): 로봇을 N12 에 손으로 놓고 Nav2 모드로 바로 시작한다(① 라인 주행·② 전환을 건너뜀).
    드라이버의 전환 조건(라인으로 N12 에 도착해 handoff 상태)을 거치지 않으므로 시험용 실행기에서만 쓴다.
    사람이 로봇을 N12 에 놓고 그 자리로 처음 위치(/initialpose)를 넣어야 한다. 관제 경로에는 없는 동작이다."""
    if mode == 'line':
        return
    if mode != 'nav2':
        raise ValueError('start mode must be line or nav2')
    with driver.lock:
        driver.mode = 'nav2'
        driver.logic.node = None


class FineAlign:
    """마커 없는 마지막 정밀 접근(시험용, 11-22). 드라이버(trial_nav2)의 게이트 마커 정밀 접근 단계를 그대로 쓴다:
    Nav2 가 목표 20 cm 뒤까지 간 뒤 멈춘 것을 확인하고, 목표까지 직선 경로를 GateDock 제어기(2 cm/s, 충돌 검사 유지)로
    보내 1.5 cm·0.08 rad 안에 들어왔는지 확인한다. 마커 대신 명령의 목표 좌표를 쓴다.

    왜: N12 복귀 자세에서 Nav2 가 일반 도착 허용 오차(8 cm) 안에서 멈춰, 라인 전환 조건(5 cm·가운데 IR 이 선 위)을
    못 맞추고 매번 7 cm 앞에 섰다(11-22 d660 두 번). roport_smartcart real_return 의 N12 정밀 맞춤(8 cm 안이면
    정확한 끝점까지 FollowPath 로 한 번 더)과 같은 생각이다. 드라이버의 GateDocking 과 같은 함수 이름을 가진다."""

    MAX_OFFSET_M = 0.30         # 정밀 접근을 시작할 때 목표까지 허용 거리(20 cm 뒤 + Nav2 도착 오차)
    MAX_YAW_RAD = 0.35
    TIMEOUT_S = 25.0

    def __init__(self, goals):
        self.goals = frozenset(goals)
        self.active, self.command, self.started = False, None, None

    def reset(self, command=None):
        self.active, self.command, self.started = False, command, None

    def enabled(self, goal_id):
        return goal_id in self.goals

    def begin(self, now):
        self.active, self.started = True, now

    def target(self, pose, now):
        if self.command is None:
            raise ValueError('fine_align_no_goal')
        if self.started is not None and now - self.started > self.TIMEOUT_S:
            raise ValueError('fine_align_timeout')
        goal = self.command['pose']
        target = {'x': float(goal['x']), 'y': float(goal['y']), 'yaw': float(goal['yaw'])}
        yaw_error = math.atan2(math.sin(target['yaw'] - pose['yaw']), math.cos(target['yaw'] - pose['yaw']))
        if math.hypot(target['x'] - pose['x'], target['y'] - pose['y']) > self.MAX_OFFSET_M or abs(yaw_error) > self.MAX_YAW_RAD:
            raise ValueError('fine_align_outside_envelope')
        return target


FINE_ALIGN_GOALS = ('demo_RET_N12',)


def install_fine_align(driver, log=print):
    """드라이버의 Nav2 어댑터에 FineAlign 을 끼운다(--fine-align 을 줬을 때만 부른다). 게이트 마커 정밀 접근이 이미
    켜져 있으면(ROPORT_GATE_DOCK_CONFIG) 건드리지 않는다. 제어기 GateDock·검사기 gate_dock_goal_checker 가 Nav2
    설정(params_file)에 없으면 20 cm 뒤에서 마지막 접근 요청이 거절되므로, 그 설정으로 띄운 경우에만 켠다."""
    nav = driver.nav
    if getattr(nav, 'docking', None) is not None:
        log('fine align: gate docking already configured; N12 fine align not installed')
        return False
    from nav2_msgs.action import FollowPath
    from rclpy.action import ActionClient
    nav.docking = FineAlign(FINE_ALIGN_GOALS)
    nav.dock_action = ActionClient(driver.node, FollowPath, 'follow_path')
    return True


def lift_nav_recovery_cap(driver):
    """시험 편의(11-22): Nav2 목표(navigate)의 누적 복구 상한(공용 MAX_RECOVERIES 2)을 쓰지 않는다.
    인형 사이를 지나는 동안 「앞에 충돌 → 1 s 대기 → 다시」가 여러 번 생겨 2번으로는 출발 14 s 만에 끝났다(d660).
    추종 goal 과 같은 기준만 남는다: 5 cm 이동 없이 30 s(RECOVERY_TIMEOUT_S)면 멈춤. 관제 경로에는 없는 동작이다.
    Nav2 목표는 추종과 같은 방식의 트리(bt/navigate_patient.xml)로 보낸다: 막히면 1 s 기다렸다 계속 다시 계획하고
    비용 지도를 비우지 않는다. 기본 트리는 2번 다시 해 보고 끝났고(11-22: 출발 자리 14 cm 앞 인형, 13 s), 비울 때
    카메라로만 보이는 인형이 지도에서 사라진다. 추종은 자기 트리를 명시해 보내므로 영향 없다."""
    import functools
    extra = {'behavior_tree': str(PATIENT_BT)} if PATIENT_BT.is_file() else {}
    driver.nav.start = functools.partial(driver.nav.start, max_recoveries=None, **extra)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', required=True, help='Nav2 지도 yaml(trial.launch 와 같은 파일)')
    ap.add_argument('--layout', required=True, help='레이아웃 yaml(노드 그래프·frame)')
    ap.add_argument('--start-node', required=True, help='로봇을 놓은 노드(예: N08)')
    ap.add_argument('--role', default='smartcart')
    ap.add_argument('--console-timeout-s', type=float, default=1.0)
    ap.add_argument('--start-mode', choices=('line', 'nav2'), default='line',
                    help='nav2 = 로봇을 N12 에 손으로 놓고 Nav2 모드로 바로 시작(시험 편의, ①·② 건너뜀)')
    ap.add_argument('--fine-align', action='store_true',
                    help='N12 복귀 자세(demo_RET_N12)에서 직선 정밀 접근을 켠다. Nav2 설정에 GateDock 제어기·'
                         'gate_dock_goal_checker 가 있어야 한다(roport_pinky config/nav2_follow_real.yaml). 기본 끔')
    ap.add_argument('--keep-nav-recovery-cap', action='store_true',
                    help='Nav2 목표의 누적 복구 상한(2번)을 그대로 둠. 기본은 시험용으로 상한 없음(30 s 무진행이면 멈춤)')
    a = ap.parse_args(argv)
    from std_msgs.msg import Bool, String

    from roport_line_nav.graph import Graph
    from roport_pinky.trial_driver import TrialDriver

    profile = make_profile(a.map, a.layout)
    graph = Graph.load(str(Path(a.layout).expanduser()))
    driver = TrialDriver(profile, a.role, layout_file=str(Path(a.layout).expanduser()),
                         start_node=a.start_node, map_path=str(Path(a.map).expanduser()))
    apply_start_mode(driver, a.start_mode)
    if not a.keep_nav_recovery_cap:
        lift_nav_recovery_cap(driver)
    if a.fine_align:
        install_fine_align(driver, log=lambda text: print(text, flush=True))
    runner = DemoRunner(driver, profile, graph, console_timeout_s=a.console_timeout_s)
    node = driver.node                      # 드라이버의 spin 스레드가 콜백을 돌린다

    def on_cmd(msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            runner.handle(data)

    node.create_subscription(String, 'demo/cmd', on_cmd, 10)
    pub = node.create_publisher(String, 'demo/state', 5)
    boarding_pub = node.create_publisher(Bool, 'smartcart/boarding_alert', 1)
    print(f'demo_runner: map {profile["map_id"]} {profile["map_version"]}, start {a.start_node}. '
          '시험용(관제 대신). 콘솔 신호가 끊기면 멈춤.'
          + (' [Nav2 모드로 바로 시작: 로봇이 N12 에 있고 처음 위치를 넣었는지 확인]' if a.start_mode == 'nav2' else ''),
          flush=True)
    last = 0.0
    try:
        while True:
            runner.tick()
            if time.monotonic() - last >= 0.5:
                last = time.monotonic()
                state = runner.state()
                pub.publish(String(data=json.dumps(state, ensure_ascii=False)))
                boarding_pub.publish(Bool(data=bool(state['boarding'])))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            driver.safe_stop('operator_stop')
        finally:
            driver.close()


if __name__ == '__main__':
    main()
