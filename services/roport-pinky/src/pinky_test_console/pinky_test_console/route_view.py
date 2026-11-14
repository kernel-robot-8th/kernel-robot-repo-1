"""경로 주행 화면(mode:=route)·테스트 관제(mode:=fleet) 공용: 명령 검사와 노드 그래프 표시 데이터.

ROS·Flask 없이 테스트한다. 경로 자체의 연결 검사는 로봇의 line_nav 노드(roport_line_nav)가 한다.
"""

import heapq
import math
import os
import re

import yaml

NODE_ID = re.compile(r'^[A-Z][A-Z0-9]{0,7}$')
MAX_ROUTE = 64
COMMANDS = ('start', 'stop', 'load_done', 'speed', 'dock', 'pause', 'resume', 'back')
SPEED_CAPS = {'base_speed': 0.15, 'rotate_speed': 1.5}   # 형식 검사용(실제 상한은 line_nav 설정)
# 후진 양보(FFT1-172, 계약 05_reverse_contract.md 3·4절): speed 명령의 reverse_speed 범위 m/s
REVERSE_SPEED_RANGE = (0.01, 0.03)
REVERSE_SPEED_DEFAULT = 0.02

# 정밀 도킹(FFT1-112, 계약 01_dock_contract.md 1·2절): 상차(LOAD) 노드별 ① 선 따라 ② 반시계 회전 ③ 직진.
# 대상 = 경로 끝 동작이 LOAD 인 노드(운영 위치 P4·P7 또는 arrival: load). 지금 그래프 v14 기준.
# test_route_view 가 배치 yaml 의 LOAD 노드와 같은지 확인한다.
DOCK_NODES = ('N01', 'N11', 'N21', 'N22')
LOAD_PLACES = ('P4', 'P7')       # roport_line_nav graph.LOAD_PLACES 와 같게
DOCK_LIMITS = {'advance_cm': (-6.0, 6.0), 'rotate_deg': (60.0, 120.0), 'move_cm': (-6.0, 6.0)}
DOCK_DEFAULT = {'advance_cm': 0.0, 'rotate_deg': 90.0, 'move_cm': 0.0}   # = 지금 동작
# 교차선 위 출발(계약 3절): 로봇이 보는 방향
FACINGS = ('E', 'N', 'W', 'S')

# 테스트 관제의 출발 위치(대기·충전 칸). 운영 위치 → 노드
START_PLACES = ('P1', 'P2', 'P5', 'P6')
# 로봇 화면 「복귀」 버튼의 목적지(2026-11-07 사용자: P5 기준)
HOME_PLACE = 'P5'

# Map 페이지(Confluence 9338886) 경로 예시. 화면의 빠른 선택용
PRESETS = [
    {'name': '물류 대기 P5 → 창고 상차 P4', 'route': ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']},
    {'name': 'P5 → P4 상차 → P5 왕복',
     'route': ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N10', 'N02', 'N14', 'N16']},
    {'name': 'P5 → P4 상차 → P7 상차 (한 번에)',
     'route': ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11']},
    {'name': 'P5 → P7 바로 (P4 안 거침)', 'route': ['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11']},
    {'name': 'P5 → P7 상차 → P5 왕복',
     'route': ['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11',
               'N19', 'N23', 'N07', 'N05', 'N15', 'N14', 'N16']},
    {'name': '창고 상차 P4 → 인도장 P7', 'route': ['N01', 'N18', 'N07', 'N23', 'N19', 'N11']},
    {'name': 'P5 → 02 인도장 왼쪽 N21', 'route': ['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N21']},
    {'name': 'P5 → 02 인도장 오른쪽 N22',
     'route': ['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N24', 'N22']},
    {'name': '인도장 P7 → 물류 대기 P6', 'route': ['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N17']},
    {'name': '스마트카트 P1 → 인도장 P7', 'route': ['H1', 'N09', 'N24', 'N19', 'N11']},
    {'name': '인도장 P7 → N12 (Nav2 전환)', 'route': ['N11', 'N19', 'N24', 'N09', 'N08', 'N12']},
    {'name': '짧은 시험: P5 → P6 (N14 T자·N15)', 'route': ['N16', 'N14', 'N15', 'N17']},
    {'name': '짧은 시험: P6 → P5 (N15 T자·N14)', 'route': ['N17', 'N15', 'N14', 'N16']},
    {'name': '짧은 시험: N14 → P5', 'route': ['N14', 'N16']},
    {'name': '짧은 시험: N18 → P4 상차', 'route': ['N18', 'N01']},
]


# 로봇 로그 파일 이름(노트북으로 가져올 때 이 형식만 받음)
ROUTE_LOG_RE = re.compile(r'^route_\d{8}_\d{6}_[A-Z0-9]{1,8}-[A-Z0-9]{1,8}\.csv$')
ROS_LOG_DIR_RE = re.compile(r'^\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}-\d{6}-[A-Za-z0-9_-]{1,40}$')


class RouteCommandError(ValueError):
    pass


def _list(path, pattern, dirs):
    root = os.path.expanduser(path)
    try:
        names = os.listdir(root)
    except OSError:
        return []
    out = []
    for n in names:
        full = os.path.join(root, n)
        if not pattern.match(n) or os.path.isdir(full) != dirs:
            continue
        st = os.stat(full)
        out.append({'name': n, 'size': None if dirs else st.st_size, 'mtime': round(st.st_mtime)})
    return sorted(out, key=lambda e: e['name'], reverse=True)


def list_route_logs(path):
    """roport_line_nav 경로 상세 기록 CSV 목록(새것부터)."""
    return _list(path, ROUTE_LOG_RE, dirs=False)


def list_ros_log_dirs(path, limit=10):
    """~/.ros/log 의 ros2 launch 실행 폴더(새것부터 limit 개)."""
    return _list(path, ROS_LOG_DIR_RE, dirs=True)[:limit]


def parse_route_command(data):
    """{cmd, route?, start_offset_cm?, start_cross?, start_facing?} 또는 dock 명령
    → line_nav/command 로 보낼 dict(알려진 키만). 잘못되면 RouteCommandError."""
    if not isinstance(data, dict):
        raise RouteCommandError('JSON object required')
    cmd = data.get('cmd')
    if cmd not in COMMANDS:
        raise RouteCommandError(f"cmd must be one of {', '.join(COMMANDS)}")
    if cmd == 'speed':
        out = {'cmd': 'speed'}
        for k, cap in SPEED_CAPS.items():
            v = data.get(k)
            if v is None:
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
                    or not 0.0 < v <= cap:
                raise RouteCommandError(f'{k} must be a number in (0, {cap}]')
            out[k] = float(v)
        rv = data.get('reverse_speed')
        if rv is not None:
            lo, hi = REVERSE_SPEED_RANGE
            if isinstance(rv, bool) or not isinstance(rv, (int, float)) or not math.isfinite(rv) \
                    or not lo - 1e-9 <= rv <= hi + 1e-9:
                raise RouteCommandError(f'reverse_speed must be a number in {lo:g}..{hi:g}')
            out['reverse_speed'] = float(rv)
        if len(out) == 1:
            raise RouteCommandError('speed needs base_speed, rotate_speed and/or reverse_speed')
        return out
    if cmd == 'back':
        to = data.get('to')
        if not isinstance(to, str) or not NODE_ID.match(to):
            raise RouteCommandError('back needs "to": a node id')
        return {'cmd': 'back', 'to': to}
    if cmd == 'dock':
        return _parse_dock(data)
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
    cross, facing = data.get('start_cross'), data.get('start_facing')
    if cross is not None and not isinstance(cross, bool):
        raise RouteCommandError('start_cross must be true or false')
    if cross:
        # 교차선 위 출발: route[0] 이 가로선 노드면 line_nav 가 거부한다(그래프는 로봇 쪽에 있음)
        if facing not in FACINGS:
            raise RouteCommandError(f"start_cross needs start_facing one of {', '.join(FACINGS)}")
        if offset is not None:
            raise RouteCommandError('start_offset_cm cannot be used with start_cross')
        out['start_cross'] = True
        out['start_facing'] = facing
    elif facing is not None:
        raise RouteCommandError('start_facing is only allowed with start_cross: true')
    return out


def _dock_number(data, key):
    v = data.get(key)
    lo, hi = DOCK_LIMITS[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
            or not lo <= v <= hi:
        raise RouteCommandError(f'{key} must be a number in {lo:g}..{hi:g}')
    return float(v)


def _parse_dock(data):
    """{cmd: dock, node, advance_cm, rotate_deg, move_cm} 또는 {cmd: dock, node, reset: true}.

    범위 밖·유한값 아님·LOAD 노드 아님은 거부. 주행 중 거부와 저장은 line_nav 가 한다.
    """
    node = data.get('node')
    if node not in DOCK_NODES:
        raise RouteCommandError(f"dock node must be one of {', '.join(DOCK_NODES)}")
    reset = data.get('reset')
    if reset is not None and not isinstance(reset, bool):
        raise RouteCommandError('reset must be true or false')
    if reset:
        if any(k in data for k in DOCK_LIMITS):
            raise RouteCommandError('reset cannot be combined with dock values')
        return {'cmd': 'dock', 'node': node, 'reset': True}
    out = {'cmd': 'dock', 'node': node}
    for k in DOCK_LIMITS:
        out[k] = _dock_number(data, k)
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
            'arrival': str(it.get('arrival', '')),
        })
    ids = {n['id'] for n in nodes}
    presets = [p for p in PRESETS if all(r in ids for r in p['route'])]
    starts = [{'place': n['place'], 'node': n['id']} for n in nodes if n['place'] in START_PLACES]
    starts.sort(key=lambda s: s['place'])
    out = {'version': str(spec.get('graph_version', '')), 'nodes': nodes, 'presets': presets,
           'starts': starts, 'goals': [n['id'] for n in nodes if n['bar']],
           'dock_nodes': [n['id'] for n in nodes if is_load_node(n)],
           'bar_length': float(spec.get('bar_length', 12.0)),
           'line_width': float((data.get('defaults') or {}).get('line_width', 2.4))}
    out.update(_layout_view(data))
    return out


def is_load_node(n):
    """graph_view 노드가 경로 끝 상차(LOAD) 노드인지. roport_line_nav graph.arrival_action 과 같은 규칙."""
    if not n['bar'] or n['nav2_switch']:
        return False
    if n['arrival']:
        return n['arrival'] == 'load'
    return n['place'] in LOAD_PLACES


def _layout_view(data):
    """전체 지도 표시용: 벽·물체·마커·03 구역(Nav2)·실측 ↔ map 좌표 변환. 실측 cm."""
    walls = [{'name': str(w.get('name', '')), 'from': [float(v) for v in w['from']],
              'to': [float(v) for v in w['to']]} for w in data.get('walls') or []]
    objects = [{'name': str(o.get('name', '')), 'min': [float(v) for v in o['min']],
                'max': [float(v) for v in o['max']], 'height': o.get('height'),
                'provisional': bool(o.get('provisional', False))}
               for o in data.get('objects') or []]
    markers = [{'name': str(m.get('name', '')), 'center': [float(v) for v in m['center']],
                'floor': bool(m.get('floor', False))}
               for m in (data.get('markers') or {}).get('items') or []]
    xs = [p[0] for w in walls for p in (w['from'], w['to'])] or [0.0, 444.5]
    ys = [p[1] for w in walls for p in (w['from'], w['to'])] or [0.0, 221.0]
    room = [min(xs), min(ys), max(xs), max(ys)]
    zones = []
    by = {w['name']: w for w in walls}
    if 'zone03_bottom' in by and 'zone03_left' in by:
        zones.append({'name': '03 공용 내부공간 (Nav2)', 'x0': by['zone03_left']['from'][0],
                      'y0': by['zone03_bottom']['from'][1], 'x1': room[2], 'y1': room[3]})
    return {'walls': walls, 'objects': objects, 'markers': markers, 'room': room,
            'zones': zones, 'frame': data.get('frame') or {}}


def plan_via(graph, start, goal, via=None):
    """start → (via 정차 노드 경유) → goal. via 가 없으면 plan_to 와 같음."""
    if not via:
        return plan_to(graph, start, goal)
    if via in (start, goal):
        raise RouteCommandError('via must differ from start and goal')
    first = plan_to(graph, start, via)
    return first + plan_to(graph, via, goal)[1:]


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


def plan_return(graph, state, goal=None, start=None):
    """「복귀」 계획. state = line_nav/state(as_dict). → {'route', 'load_done_first', 'msg'}.

    위치는 끝난 경로의 도착 노드(arrived·handoff), 경로 끝 상차 대기(loading → 상차 완료 뒤 출발),
    또는 위치를 모를 때(idle·stopped·error) 사용자가 지정한 start 노드만 믿는다. 주행 중·경유 상차
    중에는 거부한다(RouteCommandError). goal 기본 = HOME_PLACE(P5) 노드.
    """
    nodes = {n['id']: n for n in graph['nodes']}
    if goal is None:
        goal = next((n['id'] for n in graph['nodes'] if n.get('place') == HOME_PLACE), None)
        if goal is None:
            raise RouteCommandError(f'{HOME_PLACE} not in the node graph')
    st = (state or {}).get('state')
    route = [str(r) for r in (state or {}).get('route') or []]
    leg = (state or {}).get('leg') or {}
    load_first = False
    if st in ('arrived', 'handoff') and route:
        cur = route[-1]
    elif st == 'loading':
        if not route or leg.get('to') != route[-1] or (state or {}).get('index') != len(route) - 2:
            raise RouteCommandError('경유 상차 중 — 「상차 완료」로 경로를 마치거나, 정지 후 지금 노드를 지정')
        cur, load_first = route[-1], True
    elif st in (None, 'idle', 'stopped', 'error'):
        if not start:
            raise RouteCommandError(f'위치 모름({st or "상태 없음"}) — 지도에서 지금 서 있는 노드를 눌러 지정')
        if start not in nodes:
            raise RouteCommandError(f'unknown node {start!r}')
        cur = start
    else:
        raise RouteCommandError(f'주행 중({st}) — 정지한 뒤 다시')
    if cur == goal:
        return {'route': [], 'load_done_first': load_first, 'msg': f'이미 {HOME_PLACE}({goal})'}
    path = plan_to(graph, cur, goal)
    msg = f'{HOME_PLACE}({goal})로 복귀: ' + ' → '.join(path)
    if load_first:
        msg = '상차 완료 처리 뒤 ' + msg
    return {'route': path, 'load_done_first': load_first, 'msg': msg}
