import os

import pytest

from roport_line_nav.graph import (BACK, HANDOFF, LEFT, LOAD, PARK, RIGHT, STRAIGHT, Graph,
                                   GraphError, plan_route, turn_of)

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def test_turn_of():
    assert turn_of('E', 'N') == LEFT
    assert turn_of('E', 'S') == RIGHT
    assert turn_of('S', 'S') == STRAIGHT
    assert turn_of('W', 'E') == BACK


def test_layout_graph_loads(graph):
    assert graph.version.startswith('line-v14')
    assert {'N01', 'N11', 'N12', 'N16', 'N17', 'H1', 'H2'} <= set(graph.nodes)
    assert graph.arrival_action('N01') == LOAD and graph.arrival_action('N11') == LOAD
    for p in ('N16', 'N17', 'H1', 'H2'):
        assert graph.arrival_action(p) == PARK, p
    assert graph.arrival_action('N12') == HANDOFF
    # 02 인도장 새 정차 노드(운영 위치 없음)는 arrival: load 로 N11과 같은 동작
    assert graph.arrival_action('N21') == LOAD and graph.arrival_action('N22') == LOAD


def test_plan_example_route(graph):
    # 물류 대기(P5) → 창고 상차 (Map 페이지 경로 예시)
    legs = plan_route(graph, ['N16', 'N14', 'N02', 'N10', 'N18', 'N01'])
    assert [lg.action for lg in legs] == [RIGHT, LEFT, LEFT, RIGHT, LOAD]
    assert [lg.heading for lg in legs] == ['S', 'W', 'S', 'E', 'S']
    assert legs[0].detect == 'side' and legs[0].sides == {LEFT, RIGHT}   # T자를 줄기 쪽에서
    assert legs[1].sides == {LEFT}                                         # N02 모서리
    assert legs[-1].detect == 'bar'


def test_plan_straight_through_branch(graph):
    legs = plan_route(graph, ['N10', 'N18', 'N07', 'N23', 'N19', 'N11'])
    assert [lg.action for lg in legs] == [STRAIGHT, STRAIGHT, STRAIGHT, RIGHT, LOAD]
    assert legs[0].sides == {RIGHT}       # N18: P4 분기가 오른쪽(통로 쪽)


def test_junction_stop_is_opt_in_and_keeps_bar_arrivals(graph):
    from roport_line_nav.graph import STOP
    with pytest.raises(GraphError, match='must end at a stop node'):
        plan_route(graph, ['N01', 'N18'])
    leg, = plan_route(graph, ['N01', 'N18'], allow_junction_stop=True)
    assert leg.action == STOP and leg.detect == 'side'
    assert plan_route(graph, ['N18', 'N01'], allow_junction_stop=True)[-1].action == LOAD


def test_arrival_override_is_checked():
    data = {'nodes': {'items': [
        {'id': 'A', 'at': [0, 0], 'links': ['B']},
        {'id': 'B', 'at': [0, -40], 'links': ['A'], 'bar': 'x', 'arrival': 'dance'}]}}
    with pytest.raises(GraphError, match='arrival must be'):
        Graph.from_dict(data)


@pytest.mark.parametrize('route, msg', [
    (['N16'], 'at least 2'),
    (['N16', 'N99'], 'unknown'),
    (['N16', 'N02'], 'not linked'),
    (['N14', 'N02', 'N14', 'N16'], 'U-turn'),
    (['N16', 'N14', 'N02'], 'must end at a stop node'),
])
def test_plan_rejects(graph, route, msg):
    with pytest.raises(GraphError, match=msg):
        plan_route(graph, route)


def test_bad_graphs_rejected():
    base = {'nodes': {'items': [
        {'id': 'A', 'at': [0, 0], 'links': ['B']},
        {'id': 'B', 'at': [10, 0], 'links': ['A']}]}}
    Graph.from_dict(base)
    asym = {'nodes': {'items': [
        {'id': 'A', 'at': [0, 0], 'links': ['B']}, {'id': 'B', 'at': [10, 0], 'links': []}]}}
    with pytest.raises(GraphError, match='symmetric'):
        Graph.from_dict(asym)
    diag = {'nodes': {'items': [
        {'id': 'A', 'at': [0, 0], 'links': ['B']}, {'id': 'B', 'at': [10, 5], 'links': ['A']}]}}
    with pytest.raises(GraphError, match='axis-aligned'):
        Graph.from_dict(diag)
    with pytest.raises(GraphError, match='no nodes'):
        Graph.from_dict({})
