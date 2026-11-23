"""로봇에서 실행: 노트북 시험 화면 버튼 → TrialDriver(roport_pinky). 관제 MQTT 수신기 자리를 대신하는 시험용.

관제(roport-control-v2)에 추종 명령이 생기기 전까지만 쓴다. 최종 구조는 고객 폰·패드 → 서비스 서버 → 관제 → 로봇.
TrialDriver·trial_guard(구동권 문지기)는 그대로 쓰고, 관제 키트 runner 와 같은 순서로 부른다
(명령 검사 validate_action → dispatch, 0.1 s 마다 heartbeat → drain_results).

  ROS 입력  demo/cmd   std_msgs/String JSON {"op": ..., "seq": n, ...}
            op = ping | route {"route": [...], "speed_limit_mps"} | switch_nav2 | switch_line(N12 에서 라인 복귀)
                 | navigate {"goal_id", "x", "y", "yaw"} (map 좌표 m·rad, Nav2 모드)
                 | follow {"card_id", "standoff_m", "max_duration_s", "lost_after_s"} | stop
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
NAV_YAW_TOL_RAD = 0.35
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
        return self._base('navigate.requested', {
            'goal_id': goal_id, 'goal_version': 1, 'frame_id': 'map',
            'pose': {'x': x, 'y': y, 'yaw': math.atan2(math.sin(yaw), math.cos(yaw))},
            'position_tolerance_m': NAV_POSITION_TOL_M, 'yaw_tolerance_rad': NAV_YAW_TOL_RAD, **self._maps()})

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

    def _event(self, kind, **kw):
        self.events.append({'t': round(time.time(), 2), 'kind': kind, **kw})

    # ---- 입력(콘솔 메시지): ROS 콜백 스레드에서 불러도 됨
    def handle(self, msg):
        with self.lock:
            op = msg.get('op')
            if op not in OPS:
                self._event('rejected', op=op, reason='unknown_op')
                return
            self.console_at = self.now()
            self.link_stopped = False
            if op == 'ping':
                return
            if op == 'alert':
                if not isinstance(msg.get('on'), bool):
                    self._event('rejected', op=op, reason='invalid_alert')
                    return
                self.boarding = msg['on']
                self._event('alert', on=self.boarding)
                return
            if op == 'stop':
                if self.active is not None:
                    self.driver.safe_stop('operator_stop')
                    self._event('stopped', op=self.active['type'], reason='operator_stop')
                    self.active = None
                else:
                    self.driver.safe_stop('operator_stop')
                    self._event('stopped', reason='operator_stop')
                return
            try:
                cmd = self.build(msg)
                self.driver.validate_action(cmd)
            except (ValueError, KeyError, TypeError) as e:
                self._event('rejected', op=op, reason=str(e) or type(e).__name__)
                return
            self.active = cmd
            self.driver.dispatch(cmd)
            self._event('accepted', op=cmd['type'], command_id=cmd['command_id'])

    def console_alive(self):
        return self.console_at is not None and self.now() - self.console_at <= self.timeout

    # ---- 0.1 s 마다(주 루프)
    def tick(self):
        with self.lock:
            if self.console_alive():
                self.driver.heartbeat()
            elif self.active is not None and not self.link_stopped:
                # 노트북 끊김: 구동권 갱신을 멈추고(guard 가 1 s 뒤 닫음) 진행 중 명령도 멈춘다
                self.driver.safe_stop('control_link_lost')
                self._event('stopped', op=self.active['type'], reason='control_link_lost')
                self.active, self.link_stopped = None, True
            if self.boarding and not self.console_alive():  # 알림을 켠 화면이 끊김: 끌 방법이 없으니 끈다
                self.boarding = False
                self._event('alert', on=False, reason='control_link_lost')
            for original, phase, details in self.driver.drain_results():
                self._event(phase, op=original.get('type'), **{k: v for k, v in details.items()
                                                                if isinstance(v, (str, int, float, bool, type(None)))})
                if self.active is not None and original.get('command_id') == self.active.get('command_id') \
                        and phase in ('completed', 'failed'):
                    self.active = None

    def state(self):
        with self.lock:
            try:
                obs = self.driver.observe()
            except Exception as e:  # noqa: BLE001 — 화면 표시용
                obs = {'fault': f'observe_failed:{type(e).__name__}'}
            return {'observed': {k: v for k, v in obs.items() if isinstance(v, (str, int, float, bool, type(None)))},
                    'active': None if self.active is None else {'type': self.active['type'],
                                                                'command_id': self.active['command_id']},
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
