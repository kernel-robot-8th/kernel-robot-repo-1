import os

import pytest

from pinky_test_console.route_view import (PRESETS, RouteCommandError, graph_view,
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
    assert g['version'].startswith('line-v13')
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
    assert set(g['goals']) == {'N01', 'N11', 'N12', 'N16', 'N17', 'H1', 'H2'}


@pytest.mark.parametrize('start, goal, route', [
    ('N16', 'N01', ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']),
    ('N17', 'N11', ['N17', 'N15', 'N05', 'N07', 'N19', 'N11']),
    ('H1', 'N11', ['H1', 'N09', 'N19', 'N11']),
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
