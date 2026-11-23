"""노트북 쪽: 시험 화면 버튼 → 로봇 demo_runner(demo/cmd), 로봇 상태(demo/state) 받기. 시험용(관제 대신).

- 버튼 입력은 여기서 먼저 검사한다(허용 op·노드 이름·카드 35~49·간격·시간·속도 범위). 로봇 TrialDriver 가 다시 검사한다.
- 살아 있음 신호(ping): 브라우저가 최근 browser_timeout_s(3 s) 안에 화면 상태를 물어봤을 때만 5 Hz 로 보낸다.
  브라우저 창을 닫거나 노트북이 멈추면 ping 이 끊기고, 로봇 demo_runner 는 1 s 뒤 구동권을 놓고 멈춘다.
"""
from __future__ import annotations

import json
import math
import re
import threading
import time

NODE = re.compile(r'^[A-Z][0-9A-Z]{1,3}$')
GOAL_ID = re.compile(r'^[A-Za-z0-9_]{1,24}$')
# Nav2 목표 범위: 시연 공간 지도(원점 -1.654, -2.094, 넓이 4.60 x 2.66 m) 안
MAP_X, MAP_Y = (-1.654, 2.946), (-2.094, 0.566)
CARD_IDS = range(35, 50)


def _num(body, key, default, lo, hi):
    v = body.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
        raise ValueError(f'{key} must be {lo}..{hi}')
    return float(v)


def demo_command(body):
    """화면 요청(dict) → demo/cmd 메시지(dict). 틀리면 ValueError."""
    op = body.get('op')
    if op == 'route':
        route = body.get('route')
        if not isinstance(route, list) or not 2 <= len(route) <= 12 or not all(
                isinstance(n, str) and NODE.match(n) for n in route):
            raise ValueError('route must be 2..12 node ids like N08')
        return {'op': 'route', 'route': route, 'speed_limit_mps': _num(body, 'speed_limit_mps', 0.06, 0.01, 0.2)}
    if op == 'alert':                        # 탑승 시간 알림 켜기·끄기(표시·소리만)
        if not isinstance(body.get('on'), bool):
            raise ValueError('on must be true or false')
        return {'op': 'alert', 'on': body['on']}
    if op in ('switch_nav2', 'switch_line', 'stop'):
        return {'op': op}
    if op == 'navigate':
        goal_id = body.get('goal_id')
        if not isinstance(goal_id, str) or not GOAL_ID.match(goal_id):
            raise ValueError('goal_id must be 1..24 letters, digits or _')
        cmd = {'op': 'navigate', 'goal_id': goal_id, 'x': _num(body, 'x', None, *MAP_X),
               'y': _num(body, 'y', None, *MAP_Y), 'yaw': _num(body, 'yaw', None, -math.pi, math.pi)}
        if 'position_tolerance_m' in body:       # 도착 허용(로봇 기본 0.12 m, 고객이 있는 목표만 0.30 까지)
            cmd['position_tolerance_m'] = _num(body, 'position_tolerance_m', None, 0.05, 0.30)
        return cmd
    if op == 'follow':
        card = body.get('card_id')
        if isinstance(card, bool) or not isinstance(card, int) or card not in CARD_IDS:
            raise ValueError('card_id must be 35..49')
        cmd = {'op': 'follow', 'card_id': card, 'standoff_m': _num(body, 'standoff_m', 0.5, 0.2, 1.2),
               # 관제 기본 명령 마감 300 s 와 맞춤
               'max_duration_s': _num(body, 'max_duration_s', 120, 1, 300),
               # 놓침 기준: 기본 1 s(SR-22), 4 s 까지는 명시했을 때만(시연값). 노트북 --follow-lost-s 와 같은 값으로
               'lost_after_s': _num(body, 'lost_after_s', 1.0, 1.0, 4.0)}
        turn = body.get('turn_first', False)
        if not isinstance(turn, bool):
            raise ValueError('turn_first must be true or false')
        if turn:                                     # 제자리 180° 돈 뒤 추종(ARR 에서 고객 쪽으로 돌아섬)
            cmd['turn_first'] = True
        return cmd
    raise ValueError('op must be route, switch_nav2, switch_line, navigate, follow, stop or alert')


class DemoLink:
    # 11-21 실로봇: 노트북(YOLO 4.6 코어)·브라우저가 바빠 화면 읽기가 1 s 넘게 밀려 ping 이 끊기고 구동권이 잠김
    # → 3 s. 로봇 쪽 demo_runner --console-timeout-s 2.0 과 함께 쓴다(노트북이 꺼지면 최대 약 3~4 s 안 정지).
    def __init__(self, node=None, publish=None, browser_timeout_s=3.0, ping_hz=5.0):
        self.publish = publish
        self.browser_timeout_s = float(browser_timeout_s)
        self.browser_at = None
        self.robot, self.robot_at = None, None
        self.sent = 0
        self.seq = 0
        self.lock = threading.Lock()
        self._stop = threading.Event()
        if node is not None:
            from std_msgs.msg import String
            pub = node.create_publisher(String, 'demo/cmd', 10)
            self.publish = lambda msg: pub.publish(String(data=json.dumps(msg)))
            node.create_subscription(String, 'demo/state', self._on_state, 5)
            threading.Thread(target=self._ping_loop, args=(1.0 / ping_hz,), daemon=True).start()

    def _on_state(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.robot, self.robot_at = data, time.time()

    def browser_seen(self):
        self.browser_at = time.time()

    def browser_alive(self):
        return self.browser_at is not None and time.time() - self.browser_at <= self.browser_timeout_s

    def _emit(self, msg):
        with self.lock:
            self.seq += 1
            msg = dict(msg, seq=self.seq)
            self.publish(msg)
            if msg['op'] != 'ping':
                self.sent += 1
            return msg

    def _ping_loop(self, period):
        while not self._stop.wait(period):
            if self.browser_alive():
                self._emit({'op': 'ping'})

    def send(self, body):
        msg = demo_command(body)
        self.browser_seen()          # 버튼을 누른 순간도 살아 있음
        return self._emit(msg)

    def close(self):
        self._stop.set()

    def info(self):
        with self.lock:
            age = None if self.robot_at is None else round(time.time() - self.robot_at, 2)
            return {'robot': self.robot, 'robot_age_s': age, 'sent': self.sent,
                    'browser_alive': self.browser_alive()}
