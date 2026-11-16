import json
import os

import pytest

from pinky_test_console.route_view import (PRESETS, RouteCommandError, graph_view,
                                           list_ros_log_dirs, list_route_logs,
                                           parse_route_command, plan_to)

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')


def test_parse_ok():
    assert parse_route_command({'cmd': 'stop'}) == {'cmd': 'stop'}
    assert parse_route_command({'cmd': 'load_done', 'route': 'ignored'}) == {'cmd': 'load_done'}
    assert parse_route_command({'cmd': 'start', 'route': ['N14', 'N16'], 'start_offset_cm': 3}) \
        == {'cmd': 'start', 'route': ['N14', 'N16'], 'start_offset_cm': 3.0}


@pytest.mark.parametrize('data', [
    [], {'cmd': 'go'}, {'cmd': 'start'}, {'cmd': 'start', 'route': ['N14']},
    {'cmd': 'start', 'route': ['N14', 'n16']}, {'cmd': 'start', 'route': ['N14', 'N16; ls']},
    {'cmd': 'start', 'route': ['N14'] * 65},
    {'cmd': 'start', 'route': ['N14', 'N16'], 'start_offset_cm': -1},
    {'cmd': 'start', 'route': ['N14', 'N16'], 'start_offset_cm': float('nan')},
])
def test_parse_rejects(data):
    with pytest.raises(RouteCommandError):
        parse_route_command(data)


def test_graph_view_from_layout():
    g = graph_view(LAYOUT)
    ids = {n['id'] for n in g['nodes']}
    assert {'N01', 'N11', 'N12', 'H1'} <= ids
    assert g['version'].startswith('line-v14')
    assert len(g['presets']) == len(PRESETS)       # 예시 경로의 노드가 모두 배치에 있음
    n12 = next(n for n in g['nodes'] if n['id'] == 'N12')
    assert n12['nav2_switch'] and n12['bar'] == 'y'


def test_graph_view_missing(tmp_path):
    with pytest.raises(RouteCommandError):
        graph_view(str(tmp_path / 'nope.yaml'))
    (tmp_path / 'empty.yaml').write_text('nodes: {}\n')
    with pytest.raises(RouteCommandError):
        graph_view(str(tmp_path / 'empty.yaml'))


def test_starts_and_goals():
    g = graph_view(LAYOUT)
    assert [(s['place'], s['node']) for s in g['starts']] == [
        ('P1', 'H1'), ('P2', 'H2'), ('P5', 'N16'), ('P6', 'N17')]
    assert set(g['goals']) == {'N01', 'N11', 'N12', 'N16', 'N17', 'H1', 'H2', 'N21', 'N22'}


@pytest.mark.parametrize('start, goal, route', [
    ('N16', 'N01', ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']),
    ('N17', 'N11', ['N17', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11']),
    ('H1', 'N11', ['H1', 'N09', 'N24', 'N19', 'N11']),
    ('H2', 'N12', ['H2', 'N08', 'N12']),
    ('N01', 'N17', ['N01', 'N18', 'N07', 'N05', 'N15', 'N17']),
])
def test_plan_to_shortest(start, goal, route):
    assert plan_to(graph_view(LAYOUT), start, goal) == route


@pytest.mark.parametrize('start, goal', [
    ('N16', 'N14'),            # 목적지가 정차 노드가 아님
    ('N16', 'N16'), ('N99', 'N01'), ('N16', 'x;y'),
])
def test_plan_to_rejects(start, goal):
    with pytest.raises(RouteCommandError):
        plan_to(graph_view(LAYOUT), start, goal)


def test_list_logs(tmp_path):
    for n in ('route_20261104_090000_N14-N16.csv', 'route_20261104_100000_N16-N01.csv',
              'notes.txt', 'route_bad.csv'):
        (tmp_path / n).write_text('x')
    (tmp_path / '2026-11-04-09-00-00-123456-raspi-1').mkdir()
    (tmp_path / 'latest').mkdir()
    assert [f['name'] for f in list_route_logs(str(tmp_path))] == [
        'route_20261104_100000_N16-N01.csv', 'route_20261104_090000_N14-N16.csv']
    assert [d['name'] for d in list_ros_log_dirs(str(tmp_path))] == [
        '2026-11-04-09-00-00-123456-raspi-1']
    assert list_route_logs(str(tmp_path / 'missing')) == []


def test_layout_for_full_map():
    g = graph_view(LAYOUT)
    assert g['room'] == [0.0, 0.0, 444.5, 221.0]
    assert {w['name'] for w in g['walls']} >= {'zone03_bottom', 'partition_warehouse'}
    assert any(o['name'] == 'box_03' for o in g['objects'])
    assert {'G1', 'G2', 'G3', 'ARR'} <= {m['name'] for m in g['markers']}
    assert g['zones'][0]['y0'] == 94.0 and g['frame']['yaw_deg'] == 0.10


def test_parse_speed():
    assert parse_route_command({'cmd': 'speed', 'base_speed': 0.05}) == \
        {'cmd': 'speed', 'base_speed': 0.05}
    assert parse_route_command({'cmd': 'speed', 'base_speed': 0.04, 'rotate_speed': 0.5,
                                'x': 1}) == {'cmd': 'speed', 'base_speed': 0.04,
                                             'rotate_speed': 0.5}
    for bad in ({'cmd': 'speed'}, {'cmd': 'speed', 'base_speed': 0.2},
                {'cmd': 'speed', 'base_speed': 0}, {'cmd': 'speed', 'rotate_speed': 2},
                {'cmd': 'speed', 'base_speed': True}, {'cmd': 'speed', 'base_speed': 'x'},
                {'cmd': 'speed', 'base_speed': float('inf')}):
        with pytest.raises(RouteCommandError):
            parse_route_command(bad)


def test_plan_via():
    from pinky_test_console.route_view import plan_via
    g = graph_view(LAYOUT)
    assert plan_via(g, 'N16', 'N11', 'N01') == \
        ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11']
    assert plan_via(g, 'N16', 'N11') == plan_to(g, 'N16', 'N11')
    for start, goal, via in (('N16', 'N11', 'N16'), ('N16', 'N11', 'N11'),
                             ('N16', 'N11', 'N19')):          # 경유는 정차 노드만
        with pytest.raises(RouteCommandError):
            plan_via(g, start, goal, via)


def test_presets_are_valid_routes():
    """경로 예시가 line_nav 경로 규칙(경유 정차 포함)을 지킴."""
    import sys
    src = os.path.join(os.path.dirname(__file__), '..', '..')
    sys.path[:0] = [os.path.join(src, 'roport_line_nav'), os.path.join(src, 'roport_pinky_line')]
    from roport_line_nav.executor import RouteExecutor
    from roport_line_nav.graph import Graph
    ex = RouteExecutor(Graph.load(LAYOUT))
    names = [p['name'] for p in PRESETS]
    assert 'P5 → P4 상차 → P7 상차 (한 번에)' in names and 'P5 → P7 바로 (P4 안 거침)' in names
    for p in PRESETS:
        legs, _ = ex._plan_segments(p['route'])
        assert len(legs) == len(p['route']) - 1, p['name']


def _ld(state, route, index=None, to=None, **kw):
    leg = {'to': to or (route[-1] if route else None)}
    return {'state': state, 'route': route, 'index': len(route) - 2 if index is None else index,
            'leg': leg, **kw}


def test_round_trip_preset():
    p = next(p for p in PRESETS if p['name'] == 'P5 → P4 상차 → P5 왕복')
    assert p['route'][0] == p['route'][-1] == 'N16' and 'N01' in p['route']


def test_plan_return():
    from pinky_test_console.route_view import plan_return
    g = graph_view(LAYOUT)
    r = plan_return(g, _ld('arrived', ['N11', 'N19', 'N24', 'N09', 'N08', 'N12']))
    assert r['route'][0] == 'N12' and r['route'][-1] == 'N16' and not r['load_done_first']
    r = plan_return(g, _ld('loading', ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']))   # 경로 끝 상차
    assert r['load_done_first'] and r['route'][0] == 'N01' and r['route'][-1] == 'N16'
    assert '상차 완료' in r['msg']
    with pytest.raises(RouteCommandError, match='경유 상차'):                  # 경유 상차 중
        plan_return(g, _ld('loading', ['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07',
                                       'N23', 'N19', 'N11'], index=4, to='N01'))
    with pytest.raises(RouteCommandError, match='주행 중'):
        plan_return(g, _ld('follow', ['N16', 'N14']))
    for st in ('stopped', 'error', 'idle', None):
        with pytest.raises(RouteCommandError, match='위치 모름'):
            plan_return(g, None if st is None else _ld(st, []))
        r = plan_return(g, None if st is None else _ld(st, []), start='N11')   # 지도에서 지정
        assert r['route'][0] == 'N11' and r['route'][-1] == 'N16'
    assert plan_return(g, _ld('arrived', ['N14', 'N16']))['route'] == []     # 이미 P5
    with pytest.raises(RouteCommandError):
        plan_return(g, _ld('stopped', []), start='N99')


def _console_fake(state):
    """ConsoleNode 의 route_return·route_command·emergency_stop 을 그대로 쓰는 가짜 노드."""
    import threading
    import time
    ws = pytest.importorskip('pinky_test_console.web_server')

    class Log:
        def warn(self, *_):
            pass

        def info(self, *_):
            pass

    class Pub:
        def __init__(self, sent):
            self.sent = sent

        def publish(self, msg):
            self.sent.append(json.loads(msg.data))

    class Guard:
        def stop(self):
            return False

    class Fake:
        route_command = ws.ConsoleNode.route_command
        emergency_stop = ws.ConsoleNode.emergency_stop
        route_return = ws.ConsoleNode.route_return

        def __init__(self):
            self.lock = threading.Lock()
            self._return_lock = threading.RLock()
            self._return_gen = 0
            self.route_state = (state, time.monotonic())
            self.sent = []
            self.route_cmd_pub = Pub(self.sent)
            self.teleop_guard = Guard()
            self._g = graph_view(LAYOUT)

        def _require_web_drive(self):
            pass

        def route_graph(self):
            return self._g

        def publish_twist(self, *_):
            pass

        def cancel_goal(self, **_):
            return False

        def get_logger(self):
            return Log()

        def set_state(self, st):
            with self.lock:
                self.route_state = (st, time.monotonic())

    return Fake()


ROUTE_P4 = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']
LOADING_P4 = {'state': 'loading', 'route': ROUTE_P4, 'index': 4, 'leg': {'to': 'N01'}, 'event_seq': 10}


def _wait(cond, sec=3.0):
    import time
    t_end = time.monotonic() + sec
    while not cond() and time.monotonic() < t_end:
        time.sleep(0.05)


def test_console_route_return_sends_load_done_then_start():
    """로봇 콘솔 route_return: 경로 끝 상차 대기 → load_done, 도착(arrived) 뒤 복귀 start."""
    fake = _console_fake(LOADING_P4)
    res = fake.route_return()
    assert res['success'] and res['route'][0] == 'N01' and fake.sent == [{'cmd': 'load_done'}]
    fake.set_state({'state': 'arrived', 'route': ROUTE_P4, 'event_seq': 13})   # 상차 완료 → 회전 → 도착
    _wait(lambda: len(fake.sent) >= 2)
    assert fake.sent[1] == {'cmd': 'start', 'route': res['route']}
    fake2 = _console_fake({'state': 'arrived', 'route': ['N11', 'N19', 'N24', 'N09', 'N08', 'N12']})
    res = fake2.route_return()
    assert fake2.sent == [{'cmd': 'start', 'route': res['route']}] and res['route'][-1] == 'N16'


@pytest.mark.parametrize('how', ['route_stop', 'estop', 'other_command'])
def test_console_return_cancelled_by_stop(how):
    """PR #43 리뷰 P1: 상차 완료를 보낸 뒤 도착 전에 정지(경로 정지·비상 정지·다른 명령)하면
    RouteExecutor.stop() 이 arrived 를 그대로 둬도 복귀 start 가 나가지 않는다(load_done → stop 뒤 끝)."""
    import time
    fake = _console_fake(LOADING_P4)
    fake.route_return()
    if how == 'route_stop':
        fake.route_command({'cmd': 'stop'})
    elif how == 'estop':
        fake.emergency_stop()
    else:
        fake.route_command({'cmd': 'speed', 'base_speed': 0.05})
    fake.set_state({'state': 'arrived', 'route': ROUTE_P4, 'event_seq': 13})   # 정지해도 arrived 유지
    time.sleep(0.6)
    assert fake.sent[0] == {'cmd': 'load_done'}
    assert {'cmd': 'start', 'route': ['N01', 'N18', 'N10', 'N02', 'N14', 'N16']} not in fake.sent, fake.sent
    assert all(c['cmd'] != 'start' for c in fake.sent), fake.sent


def test_console_return_start_then_stop_keeps_order():
    """복귀 start 가 먼저 나갔으면 그 뒤 정지는 그대로 나가 로봇이 멈춤(순서 load_done → start → stop)."""
    fake = _console_fake(LOADING_P4)
    fake.route_return()
    fake.set_state({'state': 'arrived', 'route': ROUTE_P4, 'event_seq': 13})
    _wait(lambda: len(fake.sent) >= 2)
    fake.route_command({'cmd': 'stop'})
    assert [c['cmd'] for c in fake.sent] == ['load_done', 'start', 'stop']


def test_console_return_newer_return_replaces_older():
    """복귀를 두 번 누르면 앞의 대기 복귀는 취소되고 마지막 것만 출발."""
    fake = _console_fake(LOADING_P4)
    fake.route_return()
    fake.route_return()
    fake.set_state({'state': 'arrived', 'route': ROUTE_P4, 'event_seq': 13})
    _wait(lambda: sum(c['cmd'] == 'start' for c in fake.sent) >= 1)
    import time
    time.sleep(0.4)
    assert sum(c['cmd'] == 'start' for c in fake.sent) == 1, fake.sent


# ---------------- 정밀 도킹(FFT1-112) ----------------
def test_parse_dock_ok():
    assert parse_route_command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1.5,
                                'rotate_deg': 96, 'move_cm': -2.0, 'extra': 'dropped'}) == {
        'cmd': 'dock', 'node': 'N11', 'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': -2.0}
    for node in ('N01', 'N21', 'N22'):
        assert parse_route_command({'cmd': 'dock', 'node': node, 'advance_cm': -6,
                                    'rotate_deg': 60, 'move_cm': 6})['node'] == node
    assert parse_route_command({'cmd': 'dock', 'node': 'N01', 'reset': True}) == {
        'cmd': 'dock', 'node': 'N01', 'reset': True}


@pytest.mark.parametrize('data', [
    {'cmd': 'dock', 'node': 'N12', 'advance_cm': 0, 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N16', 'advance_cm': 0, 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N07', 'reset': True},
    {'cmd': 'dock', 'advance_cm': 0, 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': 6.01, 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': 0, 'rotate_deg': 59.9, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': 0, 'rotate_deg': 120.1, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': 0, 'rotate_deg': 90, 'move_cm': -6.5},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': float('nan'), 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': 0, 'rotate_deg': float('inf'), 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': True, 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'advance_cm': '1', 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'rotate_deg': 90, 'move_cm': 0},
    {'cmd': 'dock', 'node': 'N11', 'reset': 'yes'},
    {'cmd': 'dock', 'node': 'N11', 'reset': True, 'advance_cm': 1},
])
def test_parse_dock_rejects(data):
    with pytest.raises(RouteCommandError):
        parse_route_command(data)


def test_parse_start_cross():
    assert parse_route_command({'cmd': 'start', 'route': ['N07', 'N18', 'N01'],
                                'start_cross': True, 'start_facing': 'S'}) == {
        'cmd': 'start', 'route': ['N07', 'N18', 'N01'], 'start_cross': True, 'start_facing': 'S'}
    # start_cross: false 는 지금과 같음(키를 보내지 않음)
    assert parse_route_command({'cmd': 'start', 'route': ['N14', 'N16'],
                                'start_cross': False}) == {'cmd': 'start', 'route': ['N14', 'N16']}


@pytest.mark.parametrize('extra', [
    {'start_cross': True},                                  # 방향 없음
    {'start_cross': True, 'start_facing': 'SE'},
    {'start_cross': True, 'start_facing': 's'},
    {'start_cross': 1, 'start_facing': 'S'},
    {'start_cross': 'true', 'start_facing': 'S'},
    {'start_facing': 'S'},                                  # start_cross 없이
    {'start_cross': False, 'start_facing': 'S'},
    {'start_cross': True, 'start_facing': 'S', 'start_offset_cm': 3},
])
def test_parse_start_cross_rejects(extra):
    with pytest.raises(RouteCommandError):
        parse_route_command(dict({'cmd': 'start', 'route': ['N07', 'N18', 'N01']}, **extra))


def test_dock_nodes_match_layout_load_nodes():
    """route_view.DOCK_NODES(명령 검사) = 배치의 LOAD 노드(line_nav 규칙: arrival: load 또는 P4·P7)."""
    from pinky_test_console.route_view import DOCK_NODES
    g = graph_view(LAYOUT)
    assert set(g['dock_nodes']) == set(DOCK_NODES)
    assert g['bar_length'] == 12.0 and g['line_width'] == 2.4
