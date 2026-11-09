"""시험용 2D 시뮬레이터: 실측 노드 그래프의 테이프 위에서 Pinky 자세와 IR 3채널 값을 계산한다.

Gazebo 에는 IR 센서가 없어서 경로 주행 상태 기계를 이 모델로 검사한다(실로봇 검증을 대신하지 않음).
단위 cm·rad. 로봇 = 바퀴 축 중심 자세, IR 3개는 축 앞 ir_ahead_cm, 좌우 간격 ir_spacing_cm.
테이프 = 연결 구간(폭 line_width) + 정차 가로선(bar_length x line_width).
IR 값 = 카펫 - 덮인 비율 * (카펫 - 테이프), 센서 자국은 반지름 0.4 cm 9점.
명령이 None 이면 마지막 명령을 유지한다(pinky_bringup 에 cmd_vel 타임아웃이 없는 것과 같게).
cmd_delay_s: 명령이 바퀴에 반영되기까지 걸리는 시간(구동 지연 흉내, 기본 0 = 바로 반영).
  실로봇 d660 0.06 m/s 기록(2026-11-06)에서 전진 → 회전 전환 뒤 0.9 cm 더 간 것(0.03 m/s 는 0.5 cm)에 맞춰
  0.15~0.2 s 로 시험한다. odom 은 실제 움직임을 그대로 따른다(odom 지연은 흉내 내지 않음).
"""

import math
from collections import deque

from roport_pinky_line.line_sensor import Calibration, LineSensor

D660 = Calibration(carpet=(2308, 2376, 2233), tape=(299, 388, 281),
                   threshold=(1300, 1380, 1260), hysteresis=100)
DIRV = {'E': (1, 0), 'N': (0, 1), 'W': (-1, 0), 'S': (0, -1)}


def tape_rects(graph, skip=()):
    h = graph.line_width / 2
    half_bar = graph.bar_length / 2
    rects = []
    done = set()
    for n in graph.nodes.values():
        for o in n.links:
            key = tuple(sorted((n.id, o)))
            if key in done or key in skip:
                continue
            done.add(key)
            m = graph.nodes[o]
            rects.append((min(n.x, m.x) - h, min(n.y, m.y) - h,
                          max(n.x, m.x) + h, max(n.y, m.y) + h))
        if n.bar == 'x':
            rects.append((n.x - half_bar, n.y - h, n.x + half_bar, n.y + h))
        elif n.bar == 'y':
            rects.append((n.x - h, n.y - half_bar, n.x + h, n.y + half_bar))
    return rects


class Sim:
    def __init__(self, graph, cal=D660, ir_ahead=3.0, ir_spacing=2.0, skip=(), dt=0.05,
                 tape_graph=None, odom_scale=1.0, odom_yaw_scale=1.0, cmd_delay_s=0.0):
        """tape_graph: 실제 테이프 위치(없으면 graph). odom_*_scale: 엔코더 오차 흉내.
        cmd_delay_s: 구동 지연(s). 0 이면 받은 명령을 같은 tick 에 반영(기존 동작)."""
        if cmd_delay_s < 0:
            raise ValueError('cmd_delay_s must be >= 0')
        self.graph = graph
        self.cal = cal
        self.rects = tape_rects(tape_graph or graph, skip)
        self.odom_scale = odom_scale
        self.odom_yaw_scale = odom_yaw_scale
        self.odom = (0.0, 0.0, 0.0)
        self.ir_ahead = ir_ahead
        self.ir_spacing = ir_spacing
        self.dt = dt
        self.sensor = LineSensor(cal)
        self.x = self.y = self.yaw = 0.0
        self.t = 0.0
        self.cmd = (0.0, 0.0)          # 지금 바퀴에 걸린 명령
        self.cmd_delay_s = cmd_delay_s
        self._pending = deque()         # (반영 시각, 명령)
        self.trace = []

    def place(self, node_id, heading, sensor_offset=0.0):
        """센서 중심이 node_id 에서 heading 쪽으로 sensor_offset cm 에 오게 둔다."""
        n = self.graph.nodes[node_id]
        dx, dy = DIRV[heading]
        self.yaw = math.atan2(dy, dx)
        self.x = n.x + dx * (sensor_offset - self.ir_ahead)
        self.y = n.y + dy * (sensor_offset - self.ir_ahead)
        self.odom = (self.x, self.y, self.yaw)

    def _covered(self, px, py):
        r = 0.4
        pts = [(px + r * math.cos(a), py + r * math.sin(a))
               for a in (k * math.pi / 4 for k in range(8))] + [(px, py)]
        inside = sum(1 for qx, qy in pts
                     if any(x0 <= qx <= x1 and y0 <= qy <= y1 for x0, y0, x1, y1 in self.rects))
        return inside / len(pts)

    def sensor_points(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ax, ay = self.x + self.ir_ahead * c, self.y + self.ir_ahead * s
        return [(ax - lat * s, ay + lat * c) for lat in (self.ir_spacing, 0.0, -self.ir_spacing)]

    def raw(self):
        out = []
        for i, (px, py) in enumerate(self.sensor_points()):
            f = self._covered(px, py)
            carpet, tape = self.cal.carpet[i], self.cal.tape[i]
            out.append(int(round(carpet - f * (carpet - tape))))
        return out

    def pose(self):
        """odom 자세(실제 자세와 엔코더 오차만큼 다름). 실행부는 이것만 본다."""
        return self.odom

    def tick(self, executor):
        reading = self.sensor.update(self.raw())
        cmd = executor.step(reading, self.pose(), self.t)
        if cmd is not None:
            if self.cmd_delay_s <= 0:
                self.cmd = (cmd.linear, cmd.angular)
            else:
                self._pending.append((self.t + self.cmd_delay_s, (cmd.linear, cmd.angular)))
        while self._pending and self._pending[0][0] <= self.t + 1e-9:
            self.cmd = self._pending.popleft()[1]
        v, w = self.cmd
        v_cm = v * 100.0
        mid = self.yaw + w * self.dt / 2
        self.x += v_cm * math.cos(mid) * self.dt
        self.y += v_cm * math.sin(mid) * self.dt
        self.yaw += w * self.dt
        ox, oy, oyaw = self.odom
        omid = oyaw + w * self.odom_yaw_scale * self.dt / 2
        self.odom = (ox + v_cm * self.odom_scale * math.cos(omid) * self.dt,
                     oy + v_cm * self.odom_scale * math.sin(omid) * self.dt,
                     oyaw + w * self.odom_yaw_scale * self.dt)
        self.t += self.dt
        self.trace.append((self.t, executor.state))
        return reading

    def run(self, executor, until, max_s=400.0, on_loading=None):
        while self.t < max_s:
            self.tick(executor)
            if executor.state == 'loading' and on_loading is not None:
                if on_loading(self):
                    executor.load_done()
            if until(executor):
                return True
        return False
