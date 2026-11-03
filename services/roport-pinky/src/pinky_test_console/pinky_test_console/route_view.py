"""경로 주행 화면(mode:=route)·테스트 관제(mode:=fleet) 공용: 명령 검사와 노드 그래프 표시 데이터.

ROS·Flask 없이 테스트한다. 경로 자체의 연결 검사는 로봇의 line_nav 노드(roport_line_nav)가 한다.
"""

import heapq
import math
import re

import yaml

NODE_ID = re.compile(r'^[A-Z][A-Z0-9]{0,7}$')
MAX_ROUTE = 64
COMMANDS = ('start', 'stop', 'load_done')

# 테스트 관제의 출발 위치(대기·충전 칸). 운영 위치 → 노드
START_PLACES = ('P1', 'P2', 'P5', 'P6')

# Map 페이지(Confluence 9338886) 경로 예시. 화면의 빠른 선택용
PRESETS = [
    {'name': '물류 대기 P5 → 창고 상차 P4', 'route': ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']},
    {'name': '창고 상차 P4 → 인도장 P7', 'route': ['N01', 'N18', 'N07', 'N19', 'N11']},
    {'name': '인도장 P7 → 물류 대기 P6', 'route': ['N11', 'N19', 'N07', 'N05', 'N15', 'N17']},
    {'name': '스마트카트 P1 → 인도장 P7', 'route': ['H1', 'N09', 'N19', 'N11']},
    {'name': '인도장 P7 → N12 (Nav2 전환)', 'route': ['N11', 'N19', 'N09', 'N08', 'N12']},
    {'name': '짧은 시험: N14 → P5', 'route': ['N14', 'N16']},
    {'name': '짧은 시험: N18 → P4 상차', 'route': ['N18', 'N01']},
]


class RouteCommandError(ValueError):
    pass


def parse_route_command(data):
    """{cmd, route?, start_offset_cm?} → line_nav/command 로 보낼 dict. 잘못되면 RouteCommandError."""
    if not isinstance(data, dict):
        raise RouteCommandError('JSON object required')
    cmd = data.get('cmd')
    if cmd not in COMMANDS:
        raise RouteCommandError(f"cmd must be one of {', '.join(COMMANDS)}")
    if cmd != 'start':
        return {'cmd': cmd}
    route = data.get('route')
    if not isinstance(route, list) or not 2 <= len(route) <= MAX_ROUTE:
        raise RouteCommandError(f'route must be a list of 2..{MAX_ROUTE} node ids')
    if not all(isinstance(r, str) and NODE_ID.match(r) for r in route):
        raise RouteCommandError('route has an invalid node id')
    out = {'cmd': 'start', 'route': list(route)}
    offset = data.get('start_offset_cm')
    if offset is not None:
        if isinstance(offset, bool) or not isinstance(offset, (int, float)) \
                or not math.isfinite(offset) or not 0.0 <= offset <= 50.0:
            raise RouteCommandError('start_offset_cm must be a number in 0..50')
        out['start_offset_cm'] = float(offset)
    return out


def graph_view(path):
    """배치 yaml 의 노드 그래프 → 화면용 dict. 파일이 없거나 형식이 틀리면 RouteCommandError."""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except OSError as e:
        raise RouteCommandError(f'layout not readable: {e.strerror}') from None
    spec = data.get('nodes') or {}
    items = spec.get('items') or []
    if not items:
        raise RouteCommandError('layout has no nodes.items')
    nodes = []
    for it in items:
        x, y = (float(v) for v in it['at'])
        nodes.append({
            'id': str(it['id']), 'x': x, 'y': y,
            'links': [str(v) for v in it.get('links') or []],
            'role': str(it.get('role', '')), 'place': str(it.get('place', '')),
            'bar': str(it.get('bar', '')), 'nav2_switch': bool(it.get('nav2_switch', False)),
        })
    ids = {n['id'] for n in nodes}
    presets = [p for p in PRESETS if all(r in ids for r in p['route'])]
    starts = [{'place': n['place'], 'node': n['id']} for n in nodes if n['place'] in START_PLACES]
    starts.sort(key=lambda s: s['place'])
    return {'version': str(spec.get('graph_version', '')), 'nodes': nodes, 'presets': presets,
            'starts': starts, 'goals': [n['id'] for n in nodes if n['bar']]}


def plan_to(graph, start, goal):
    """graph_view 결과에서 start → goal 최단 경로(노드 id 목록, 길이 = 선 중심 거리).

    목적지는 가로선이 있는 정차 노드만(line_nav 경로 규칙). 경로가 없으면 RouteCommandError.
    """
    nodes = {n['id']: n for n in graph['nodes']}
    for nid in (start, goal):
        if not isinstance(nid, str) or not NODE_ID.match(nid) or nid not in nodes:
            raise RouteCommandError(f'unknown node {nid!r}')
    if not nodes[goal]['bar']:
        raise RouteCommandError(f'{goal} is not a stop node (choose a node with a stop bar)')
    if start == goal:
        raise RouteCommandError('start and goal are the same node')
    dist = {start: 0.0}
    prev = {}
    heap = [(0.0, start)]
    while heap:
        d, nid = heapq.heappop(heap)
        if nid == goal:
            break
        if d > dist.get(nid, math.inf):
            continue
        a = nodes[nid]
        for other in a['links']:
            if other not in nodes:
                continue
            b = nodes[other]
            nd = d + math.hypot(b['x'] - a['x'], b['y'] - a['y'])
            if nd < dist.get(other, math.inf):
                dist[other] = nd
                prev[other] = nid
                heapq.heappush(heap, (nd, other))
    if goal not in dist:
        raise RouteCommandError(f'no route from {start} to {goal}')
    route = [goal]
    while route[-1] != start:
        route.append(prev[route[-1]])
    route.reverse()
    # 가로선 노드(막다른 칸)는 경로 중간에 올 수 없다(line_nav 규칙). 최단 경로에서는 생기지 않는다.
    if any(nodes[r]['bar'] for r in route[1:-1]):
        raise RouteCommandError('route passes through a stop node')
    return route
