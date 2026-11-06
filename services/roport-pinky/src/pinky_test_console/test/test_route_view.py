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
