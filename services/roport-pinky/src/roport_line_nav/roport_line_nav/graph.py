"""라인 노드 그래프(layout yaml 의 nodes 절) 읽기와 경로 계획. 순수 Python.

그래프 원본은 roport_gz_world/layout/roport_airport.yaml 하나다(번호를 코드에 적지 않는다).
좌표는 실측 cm(테이프 중심선), x = 길이 방향, y = 창가 방향. 연결은 모두 x 또는 y 축과 나란하다.
방향은 E(+x)·N(+y)·W(-x)·S(-y), 회전은 진행 방향 기준 LEFT(반시계)·RIGHT(시계)·STRAIGHT.
"""

import math
from dataclasses import dataclass, field

import yaml

DIRS = {'E': (1, 0), 'N': (0, 1), 'W': (-1, 0), 'S': (0, -1)}
ORDER = ('E', 'N', 'W', 'S')      # 반시계 순서
STRAIGHT, LEFT, RIGHT, BACK = 'straight', 'left', 'right', 'back'

# 도착 동작 (운영 위치 → 동작). 2026-11-03 사용자 결정
#   load   : P4(N01)·P7(N11) 상차. 반시계 90° → 상차 완료까지 대기 → 반시계 90°
#   park   : P1·P2(스마트카트)·P5·P6(물류) 대기·충전. 180° → 후진해 IR 3개가 가로선을 보면 정지
#   handoff: N12(nav2_switch). 정지 후 Nav2 전환 가능 보고
#   stop   : 그 밖의 가로선 노드. 정지만
# 노드에 arrival: load|park|stop 을 적으면 운영 위치보다 먼저 쓴다(예: 02 인도장 N21·N22 = load, 2026-11-05)
LOAD, PARK, HANDOFF, STOP = 'load', 'park', 'handoff', 'stop'
LOAD_PLACES = ('P4', 'P7')
PARK_PLACES = ('P1', 'P2', 'P5', 'P6')


class GraphError(ValueError):
    pass


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float
    links: tuple
    role: str = ''
    place: str = ''
    bar: str = ''            # 정차 가로선 방향 'x'·'y', 없으면 ''
    nav2_switch: bool = False
    arrival: str = ''        # 도착 동작 직접 지정(LOAD·PARK·STOP), 없으면 운영 위치로 정함


def turn_of(heading, new_heading):
    """진행 방향 heading 에서 new_heading 으로 갈 때의 회전."""
    d = (ORDER.index(new_heading) - ORDER.index(heading)) % 4
    return (STRAIGHT, LEFT, BACK, RIGHT)[d]


@dataclass
class Graph:
    nodes: dict
    version: str = ''
    bar_length: float = 12.0
    line_width: float = 2.4

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data):
        spec = data.get('nodes') or {}
        items = spec.get('items') or []
        if not items:
            raise GraphError('layout has no nodes.items')
        nodes = {}
        for it in items:
            nid = str(it['id'])
            if nid in nodes:
                raise GraphError(f'duplicate node id {nid}')
            x, y = (float(v) for v in it['at'])
            arrival = str(it.get('arrival', ''))
            if arrival not in ('', LOAD, PARK, STOP):
                raise GraphError(f'{nid}: arrival must be {LOAD}, {PARK} or {STOP}')
            nodes[nid] = Node(nid, x, y, tuple(str(v) for v in it.get('links') or ()),
                              str(it.get('role', '')), str(it.get('place', '')),
                              str(it.get('bar', '')), bool(it.get('nav2_switch', False)), arrival)
        width = float((data.get('defaults') or {}).get('line_width', 2.4))
        g = cls(nodes, str(spec.get('graph_version', '')),
                float(spec.get('bar_length', 12)), width)
        g._check()
        return g

    def _check(self):
        for n in self.nodes.values():
            for other in n.links:
                if other not in self.nodes:
                    raise GraphError(f'{n.id} links unknown node {other}')
                if n.id not in self.nodes[other].links:
                    raise GraphError(f'link {n.id}-{other} is not symmetric')
                self.heading(n.id, other)  # 축과 나란한지

    def heading(self, a, b):
        """a → b 진행 방향(E·N·W·S)."""
        na, nb = self.nodes[a], self.nodes[b]
        dx, dy = nb.x - na.x, nb.y - na.y
        if abs(dx) > 1e-6 and abs(dy) > 1e-6:
            raise GraphError(f'link {a}-{b} is not axis-aligned')
        if abs(dx) > abs(dy):
            return 'E' if dx > 0 else 'W'
        if abs(dy) > 1e-6:
            return 'N' if dy > 0 else 'S'
        raise GraphError(f'link {a}-{b} has zero length')

    def length(self, a, b):
        na, nb = self.nodes[a], self.nodes[b]
        return math.hypot(nb.x - na.x, nb.y - na.y)

    def arrival_action(self, nid):
        n = self.nodes[nid]
        if n.nav2_switch:
            return HANDOFF
        if n.arrival:
            return n.arrival
        if n.place in LOAD_PLACES:
            return LOAD
        if n.place in PARK_PLACES:
            return PARK
        return STOP

    def side_links(self, frm, to):
        """frm → to 로 도착했을 때 to 에서 옆으로 난 선의 방향(LEFT·RIGHT)."""
        h = self.heading(frm, to)
        sides = set()
        for other in self.nodes[to].links:
            if other == frm:
                continue
            t = turn_of(h, self.heading(to, other))
            if t in (LEFT, RIGHT):
                sides.add(t)
        return frozenset(sides)


@dataclass(frozen=True)
class Leg:
    """경로의 한 구간 frm → to 와 to 에서 할 일."""

    frm: str
    to: str
    heading: str
    length: float              # 노드 중심 사이 cm
    action: str                # straight·left·right(중간) 또는 load·park·handoff·stop(마지막)
    detect: str                # 'bar'(세 채널 모두) 또는 'side'(sides 쪽 채널)
    sides: frozenset = field(default_factory=frozenset)


def plan_route(graph, route, *, allow_junction_stop=False):
    """노드 목록 → Leg 목록. 기본은 가로선 도착, 명시한 경우만 교차점 정차 허용."""
    route = [str(r) for r in route]
    if len(route) < 2:
        raise GraphError('route needs at least 2 nodes')
    for nid in route:
        if nid not in graph.nodes:
            raise GraphError(f'unknown node {nid}')
    legs = []
    for i in range(len(route) - 1):
        a, b = route[i], route[i + 1]
        if b not in graph.nodes[a].links:
            raise GraphError(f'{a} and {b} are not linked')
        h = graph.heading(a, b)
        last = i == len(route) - 2
        if last:
            if not graph.nodes[b].bar and not allow_junction_stop:
                raise GraphError(f'route must end at a stop node with a bar: {b}')
            action = graph.arrival_action(b) if graph.nodes[b].bar else STOP
        else:
            action = turn_of(h, graph.heading(b, route[i + 2]))
            if action == BACK:
                raise GraphError(f'U-turn at {b} is not supported')
        node_b = graph.nodes[b]
        if node_b.bar:
            if not last:
                raise GraphError(f'{b} is a stop node; it can only be the last node')
            detect, sides = 'bar', frozenset()
        else:
            sides = graph.side_links(a, b)
            if not sides:
                raise GraphError(f'{b} has no side line to detect from {a}')
            detect = 'side'
        legs.append(Leg(a, b, h, graph.length(a, b), action, detect, sides))
    return legs
