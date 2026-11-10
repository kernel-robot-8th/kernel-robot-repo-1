"""IR 라인 유실 복구(FFT1-163): 테이프 끊김을 GAP(방향 유지 직진)·SEARCH(후진 후 좌우 찾기)로 넘거나 안전 정지.

2D 시뮬(test/sim.py, gaps=)로만 확인한다(실로봇 미검증). 끊김 = 지정 구간 테이프를 잘라 낸 것.
"""

import math
import os

import pytest
from sim import Sim

from roport_line_nav.executor import (ADVANCE, ARRIVED, ERROR, FOLLOW, LOADING, NavConfig,
                                      ROTATE, RouteExecutor, STOPPED)
from roport_line_nav.graph import Graph

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')
SPEEDS = (0.03, 0.06, 0.08)
DELAYS = (0.0, 0.15, 0.2)
TO_N01 = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']
# 끊김 위치: (이름, 경로, 시작 노드·방향·교차선 출발, 끊긴 구간 a, b, a 에서 시작 cm)
PLACES = {
    'N05-N07 mid': (['N05', 'N07', 'N18', 'N01'], ('N05', 'S', True), ('N05', 'N07', 40.0)),
    'N16-N14': (['N16', 'N14', 'N15', 'N17'], ('N16', 'S', False), ('N16', 'N14', 6.0)),
    'N02-N10 front': (TO_N01, ('N16', 'S', False), ('N02', 'N10', 12.0)),   # 모서리 회전 직후
}
EVENTS = ('gap', 'gap_window', 'gap_node', 'gap_recovered', 'search', 'search_found',
          'search_failed')


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def _cfg(speed, **kw):
    return NavConfig(base_speed=speed, max_linear=max(0.06, speed), **kw)


def run(graph, place, length, speed, delay, gaps=None, on_tick=None, **cfg):
    route, (start, facing, cross), (a, b, st) = PLACES[place]
    sim = Sim(graph, cmd_delay_s=delay,
              gaps=gaps if gaps is not None else ([(a, b, st, length)] if length else []))
    sim.place(start, facing, 0.0)
    ex = RouteExecutor(graph, _cfg(speed, **cfg))
    if cross:
        ex.start(route, start_cross=True, start_facing=facing)
    else:
        ex.start(route)
    events, seen = [], ex.event_seq
    while ex.active and sim.t < 600:
        sim.tick(ex)
        if on_tick is not None:
            on_tick(sim, ex)
        if ex.state == LOADING:
            ex.load_done()
        new = ex.event_seq - seen
        if new:
            events.extend(list(ex.events)[-new:])
            seen = ex.event_seq
    return sim, ex, events


def _kinds(events):
    return [e['event'] for e in events if e['event'] in EVENTS]


# ---------------------------------------------------------------- 끊김 없음 = 예전과 같음
@pytest.mark.parametrize('route, start', [
    (['N16', 'N14', 'N02', 'N10', 'N18', 'N01', 'N18', 'N07', 'N23', 'N19', 'N11'], ('N16', 'S')),
    (['N11', 'N19', 'N23', 'N07', 'N05', 'N15', 'N17'], ('N11', 'N')),
    (['H1', 'N09', 'N24', 'N19', 'N11'], ('H1', 'N')),
])
def test_no_gap_identical_to_disabled(graph, route, start):
    """테이프가 끊기지 않으면 복구 켬(기본)과 끔(gap_max_cm 0, 예전 동작)의 궤적·이벤트가 같다."""
    runs = []
    for gap_max in (8.0, 0.0):
        sim = Sim(graph, cmd_delay_s=0.2)
        sim.place(start[0], start[1], 0.0)
        ex = RouteExecutor(graph, NavConfig(gap_max_cm=gap_max))
        ex.set_speed(0.06, 0.6)
        ex.start(route)
        poses = []
        while ex.active and sim.t < 900:
            sim.tick(ex)
            poses.append((round(sim.x, 6), round(sim.y, 6), round(sim.yaw, 6)))
            if ex.state == LOADING:
                ex.load_done()
        runs.append((ex.state, poses, [(e['event'], e['detail']) for e in ex.events]))
    assert runs[0] == runs[1] and runs[0][0] == ARRIVED


# ---------------------------------------------------------------- 3·5 cm 는 끝까지
@pytest.mark.parametrize('delay', DELAYS)
@pytest.mark.parametrize('speed', SPEEDS)
@pytest.mark.parametrize('length', [2.0, 3.0, 5.0])
@pytest.mark.parametrize('place', sorted(PLACES))
def test_short_gap_completes(graph, place, length, speed, delay):
    sim, ex, events = run(graph, place, length, speed, delay)
    assert ex.state == ARRIVED, (ex.reason, _kinds(events))
    assert _kinds(events) == ['gap', 'gap_recovered']          # 감지 창 전에서 복구
    rec = [e for e in events if e['event'] == 'gap_recovered'][0]
    assert float(rec['detail'].split()[0]) <= ex.cfg.gap_max_cm


def test_gap_keeps_speed_and_heading(graph):
    """GAP 중 명령: 선속도 = approach_speed(0.08 m/s 주행이어도), 방향 유지, 노드 판정 없음."""
    seen = []

    def watch(sim, ex):
        if ex._gap is not None:
            seen.append((sim.cmd, ex.as_dict()))

    sim, ex, events = run(graph, 'N05-N07 mid', 5.0, 0.08, 0.0, on_tick=watch)
    assert ex.state == ARRIVED
    assert seen
    for cmd, st in seen[1:]:
        assert cmd[0] == pytest.approx(ex.approach_speed) and abs(cmd[1]) <= ex.cfg.max_angular
        assert st['state'] == FOLLOW and st['recovery']['mode'] == 'gap'
    gaps = [st['recovery']['gap_cm'] for _, st in seen]
    assert gaps == sorted(gaps)
    prog = [st['leg_progress'] for _, st in seen]                 # 화면 추정도 직진
    assert prog == sorted(prog) and prog[-1] > prog[0]
    assert ex.as_dict()['recovery'] is None


# ---------------------------------------------------------------- 8 cm: 결과 기록, 10 cm+: 정지
EIGHT_CM = {   # 시뮬 결과(2026-11-09): 위치마다 기대 결과
    'N05-N07 mid': {'recovered'},
    'N16-N14': {'recovered', 'stop_window'},    # 짧은 구간: 끊김 끝이 감지 창 안 — 창 안 직진·복구, 못 하면 정지
    'N02-N10 front': {'recovered', 'stop_gap'},      # 회전 직후: 9번 중 1번(0.08·0.15 s) GAP 초과 정지
}


def _outcome(ex, kinds):
    if ex.state == ARRIVED:
        return 'recovered' if 'search_found' not in kinds else 'recovered_search'
    assert ex.state == ERROR and 'line_lost' in ex.reason, ex.reason
    return 'stop_window' if 'window' in ex.reason else 'stop_gap'


@pytest.mark.parametrize('place', sorted(PLACES))
def test_eight_cm_gap_outcome(graph, place):
    outcomes = []
    for speed in SPEEDS:
        for delay in DELAYS:
            sim, ex, events = run(graph, place, 8.0, speed, delay)
            outcomes.append(_outcome(ex, _kinds(events)))
    assert set(outcomes) <= EIGHT_CM[place], outcomes
    if place == 'N05-N07 mid':
        assert outcomes.count('recovered') == 9


@pytest.mark.parametrize('length', [10.0, 14.0])
@pytest.mark.parametrize('speed, delay', [(0.03, 0.0), (0.06, 0.15), (0.08, 0.2), (0.03, 0.2)])
def test_long_gap_stops_without_search_by_default(graph, length, speed, delay):
    """기본(search_deg 0): 10 cm 이상은 GAP 8 cm 뒤 후진·회전 없이 정지."""
    sim, ex, events = run(graph, 'N05-N07 mid', length, speed, delay)
    assert ex.state == ERROR and 'exceeded 8 cm on N05→N07' in ex.reason, ex.reason
    assert _kinds(events) == ['gap']
    i = [e['event'] for e in events].index('gap')
    assert not [e for e in events[i:] if e['event'] in ('advance', 'rotate', 'reverse')]
    for _ in range(6):
        sim.tick(ex)
    assert sim.cmd == (0.0, 0.0)


@pytest.mark.parametrize('length', [10.0, 14.0])
@pytest.mark.parametrize('speed, delay', [(0.03, 0.0), (0.06, 0.15), (0.08, 0.2), (0.03, 0.2)])
def test_long_gap_search_fails_and_stops(graph, length, speed, delay):
    """SEARCH 를 켜면(search_deg 20): 10 cm 이상은 GAP 8 cm → SEARCH(후진 → 좌 → 우 → 원래 방향) → 정지.
    선을 다시 잡아도 두 번째 GAP 에서 정지. 2D 시뮬에서 SEARCH 로 끊김을 넘은 경우는 없음(그래서 기본 끔)."""
    sim, ex, events = run(graph, 'N05-N07 mid', length, speed, delay, search_deg=20.0)
    kinds = _kinds(events)
    assert ex.state == ERROR and 'gap/search failed on N05→N07' in ex.reason, (ex.reason, kinds)
    assert kinds[:2] == ['gap', 'search'] and kinds.count('search') == 1
    for _ in range(6):
        sim.tick(ex)
    assert sim.cmd == (0.0, 0.0)
    n07 = graph.nodes['N07']
    assert sim.y > n07.y + 20                                   # 노드에 다가가지 않음


def test_search_sequence_returns_to_lost_point(graph):
    """SEARCH: 멈춤 → 놓친 지점까지 후진 → 좌 20° → 우 40° → 좌 20° 순서, 회전은 놓친 지점에서."""
    trace = []

    def watch(sim, ex):
        if ex._recovery is not None:
            trace.append((ex.state, ex._prim.get('dir'), ex._prim.get('phase'), sim.x, sim.y,
                          sim.yaw, ex._recovery['d']))

    lost = {}

    def watch2(sim, ex):
        if ex._gap is not None and 'pose' not in lost:
            lost['pose'] = ex._gap['pose']
        watch(sim, ex)

    sim, ex, events = run(graph, 'N05-N07 mid', 14.0, 0.03, 0.0, on_tick=watch2, search_deg=20.0)
    assert ex.state == ERROR
    rot = [e['detail'] for e in events if e['event'] == 'rotate']
    assert rot[-3:] == ["(1.0, 0.0, 20.0, 'search')", "(-1.0, 0.0, 40.0, 'search')",
                        "(1.0, 0.0, 20.0, 'search')"]
    rotating = [t for t in trace if t[0] == ROTATE]
    assert rotating
    lx, ly = lost['pose']
    for t in rotating:                                           # 축이 놓친 지점 ±0.6 cm
        assert math.hypot(t[3] - lx, t[4] - ly) < 0.6, (t, lost)
    back = [t for t in trace if t[0] == ADVANCE and t[2] == 'search']
    assert back
    yaws = [math.degrees(t[5]) + 90.0 for t in rotating]         # 남쪽(−90°) 기준 ±20° 안
    assert max(yaws) <= 21.5 and min(yaws) >= -21.5
    assert ex.as_dict()['recovery'] is None


# ---------------------------------------------------------------- 방향 오차
@pytest.mark.parametrize('length', [3.0, 5.0])
@pytest.mark.parametrize('deg', [-5.0, -3.0, 3.0, 5.0])
def test_gap_with_heading_error(graph, length, deg):
    """끊김에서 실제 방향이 3~5° 틀어짐(odom 은 모름 = 방향 유지 기준 오차): 그래도 건넘."""
    done = {}

    def bump(sim, ex):
        if ex._gap is not None and not done:
            sim.yaw += math.radians(deg)
            done['ok'] = True

    sim, ex, events = run(graph, 'N05-N07 mid', length, 0.06, 0.15, on_tick=bump)
    assert done and ex.state == ARRIVED, (ex.reason, _kinds(events))


# ---------------------------------------------------------------- 노드 근처: 감지 창 안 GAP
# 감지 창 안에서는 직진을 유지하며 그 노드의 기존 판정(가로선 = IR 3개, 옆선·T자 규칙, node_confirm)으로만 감지.
# 진입로가 끊겨도 노드 테이프에 닿으면 도착, 노드 테이프가 없으면 max_ok·gap_max_cm 에서 정지(SEARCH 없음).
APPROACH = {   # 이름: (경로, 출발 노드·방향, 끊긴 구간 a → 노드 b)
    'N11 load': (['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11'], ('N16', 'S'), ('N19', 'N11')),
    'N01 load': (TO_N01, ('N16', 'S'), ('N18', 'N01')),
    'N16 park': (['N17', 'N15', 'N14', 'N16'], ('N17', 'S'), ('N14', 'N16')),
    'N10 corner': (TO_N01, ('N16', 'S'), ('N02', 'N10')),
    'N14 tee': (['N16', 'N14', 'N15', 'N17'], ('N16', 'S'), ('N16', 'N14')),
    'N15 pass': (['N16', 'N14', 'N15', 'N05', 'N07', 'N23', 'N19', 'N11'], ('N16', 'S'), ('N14', 'N15')),
}
APPROACH_END_CM = 1.5      # 끊김 끝 = 노드 중심 1.5 cm 앞(노드 테이프 ±1.2 cm 는 그대로)


def run_approach(graph, name, length, speed, delay, end_cm=APPROACH_END_CM):
    route, (start, facing), (a, b) = APPROACH[name]
    cut = (a, b, graph.length(a, b) - end_cm - length, length)
    sim = Sim(graph, cmd_delay_s=delay, gaps=[cut])
    sim.place(start, facing, 0.0)
    ex = RouteExecutor(graph, _cfg(speed))
    ex.start(route)
    events, seen = [], ex.event_seq
    while ex.active and sim.t < 600:
        sim.tick(ex)
        if ex.state == LOADING:
            ex.load_done()
        new = ex.event_seq - seen
        if new:
            events.extend(list(ex.events)[-new:])
            seen = ex.event_seq
    return sim, ex, events


def _node_found(events, node):
    return [e for e in events if e['event'] == 'node' and e['detail'].startswith(node + ' ')]


@pytest.mark.parametrize('delay', DELAYS)
@pytest.mark.parametrize('speed', SPEEDS)
@pytest.mark.parametrize('length', [3.0, 5.0])
@pytest.mark.parametrize('name', ['N10 corner', 'N14 tee', 'N15 pass'])
def test_approach_gap_turn_and_pass_nodes(graph, name, length, speed, delay):
    """회전·T자·통과 노드 진입로 3·5 cm 끊김(노드 테이프 유지): 창 안 직진으로 노드를 평소 판정으로 감지·완주.
    예외 1건: N15 통과 5 cm·0.08 m/s·0.2 s 는 GAP 방향 −4° 로 통과한 뒤 짧은 다음 구간 끝 N05 회전에서
    선을 못 찾아 정지(안전 정지, 2D 시뮬)."""
    sim, ex, events = run_approach(graph, name, length, speed, delay)
    node = APPROACH[name][2][1]
    kinds = _kinds(events)
    assert kinds[:2] == ['gap', 'gap_window'], kinds
    assert _node_found(events, node), kinds
    if (name, length, speed, delay) == ('N15 pass', 5.0, 0.08, 0.2):
        assert ex.state in (ARRIVED, ERROR) and (ex.state == ARRIVED or 'line_not_found' in ex.reason)
        return
    assert ex.state == ARRIVED, (ex.reason, kinds)


@pytest.mark.parametrize('delay', DELAYS)
@pytest.mark.parametrize('speed', SPEEDS)
@pytest.mark.parametrize('name', ['N11 load', 'N01 load', 'N16 park'])
def test_approach_gap_stop_nodes(graph, name, speed, delay):
    """정차 노드(상차·대기 칸) 진입로 끊김: 3 cm(노드 앞 1.5~4.5 cm)는 가로선 감지·도착 동작까지 완주.
    5 cm(1.5~6.5 cm)는 가로선은 감지하지만, 도착 뒤 진입선 찾기 회전이 노드에서 약 7 cm 지점 테이프를 봐야
    해서 끊긴 범위에 걸리면 line_not_found 로 정지(판정 완화 안 함). 다른 선을 잡고 가지 않는다."""
    node = APPROACH[name][2][1]
    sim, ex, events = run_approach(graph, name, 3.0, speed, delay)
    assert ex.state == ARRIVED, (ex.reason, _kinds(events))
    assert _kinds(events)[:2] == ['gap', 'gap_window'] and _node_found(events, node)
    sim, ex, events = run_approach(graph, name, 5.0, speed, delay)
    assert _node_found(events, node), _kinds(events)
    i = events.index(_node_found(events, node)[-1])             # 가로선 감지 뒤 도착 동작 시작
    assert ('loading' if 'load' in name else 'rotate') in [e['event'] for e in events[i:]]
    assert ex.state == ARRIVED or (ex.state == ERROR and 'line_not_found' in ex.reason), ex.reason


@pytest.mark.parametrize('lost_at, label', [(85.0, 'lost in window'), (78.0, 'gap into window')])
@pytest.mark.parametrize('speed, delay', [(0.03, 0.0), (0.06, 0.2), (0.08, 0.15)])
def test_corner_tape_missing_stops(graph, lost_at, label, speed, delay):
    """N10 모서리 테이프가 없음(N02→N10 끝 + N10→N18 처음 6 cm): 창 안에서 놓치든 GAP 이 창에 들어가든
    직진하며 노드 판정만 → 노드 테이프가 없으니 정지. N10 으로 오판하거나 돌아서 다른 선을 잡지 않는다."""
    n10 = graph.nodes['N10']
    gaps = [('N02', 'N10', lost_at, 20.0), ('N10', 'N18', -2.0, 8.0)]
    sim, ex, events = run(graph, 'N02-N10 front', 0, speed, delay, gaps=gaps)
    assert ex.state == ERROR and ex.reason == \
        'line_lost: gap into N10 window, node not found on N02→N10', ex.reason
    assert 'gap_window' in _kinds(events) and 'search' not in _kinds(events)
    assert ex.legs[ex.index].to == 'N10'
    assert not _node_found(events, 'N10')
    j = events.index([e for e in events if e['event'] == 'leg' and 'N02→N10' in e['detail']][0])
    assert not [e for e in events[j:] if e['event'] in ('rotate', 'search', 'advance')]
    for _ in range(6):
        sim.tick(ex)
    assert sim.cmd == (0.0, 0.0)
    assert sim.y > n10.y - 6.0                                   # gap_max_cm 안에서 멈춤
    assert abs(math.degrees(sim.yaw) + 90.0) < 20                # 남쪽 그대로(돌지 않음)


@pytest.mark.parametrize('speed, delay', [(0.03, 0.0), (0.06, 0.2), (0.08, 0.15)])
def test_tee_tape_missing_stops(graph, speed, delay):
    """N14 T자 테이프가 없음(N16→N14 끝 6 cm + 좌우 갈래 처음 5 cm): 노드를 건너뛰지 않고 정지."""
    gaps = [('N16', 'N14', 19.0, 10.0), ('N14', 'N02', -2.0, 7.0), ('N14', 'N15', -2.0, 7.0)]
    sim, ex, events = run(graph, 'N16-N14', 0, speed, delay, gaps=gaps)
    assert ex.state == ERROR and 'gap into N14 window, node not found' in ex.reason, ex.reason
    assert not _node_found(events, 'N14') and 'search' not in _kinds(events)
    assert not [e for e in events if e['event'] in ('rotate', 'advance')]


def test_lost_in_window_message_only_when_enabled(graph):
    """복구를 끄면(gap_max_cm 0) 예전과 같은 메시지 'line_lost on A→B'."""
    gaps = [('N02', 'N10', 85.0, 20.0), ('N10', 'N18', -2.0, 8.0)]
    _, ex, _ = run(graph, 'N02-N10 front', 0, 0.03, 0.0, gaps=gaps, gap_max_cm=0.0)
    assert ex.state == ERROR and ex.reason == 'line_lost on N02→N10'
    _, ex, _ = run(graph, 'N05-N07 mid', 5.0, 0.03, 0.0, gap_max_cm=0.0)
    assert ex.state == ERROR and ex.reason == 'line_lost on N05→N07'      # 끄면 끊김을 못 건넘


def test_suppress_zone_lost_unchanged(graph):
    """구간 시작 suppress_cm 안의 유실은 예전처럼 대기 후 'line_lost on'(GAP 안 함)."""
    gaps = [('N16', 'N14', 1.0, 6.0)]
    _, ex, events = run(graph, 'N16-N14', 0, 0.03, 0.0, gaps=gaps)
    assert ex.state == ERROR and ex.reason == 'line_lost on N16→N14'
    assert 'gap' not in _kinds(events)


# ---------------------------------------------------------------- 정지 명령·설정
def test_stop_during_gap(graph):
    state = {}

    def stop_in_gap(sim, ex):
        if ex._gap is not None and 'stopped' not in state:
            state['stopped'] = True
            ex.stop()

    sim, ex, events = run(graph, 'N05-N07 mid', 5.0, 0.06, 0.15, on_tick=stop_in_gap)
    assert ex.state == STOPPED and state
    assert ex.as_dict()['recovery'] is None
    for _ in range(6):                                          # 정지 뒤 0 한 번(구동 지연 뒤 바퀴에 걸림)
        sim.tick(ex)
    assert sim.cmd == (0.0, 0.0)
    reading = sim.sensor.update(sim.raw())
    assert ex.step(reading, sim.pose(), sim.t) is None          # 그 뒤로는 발행 안 함


def test_stop_during_search(graph):
    state = {}

    def stop_in_search(sim, ex):
        if ex._recovery is not None and ex.state == ROTATE and 'stopped' not in state:
            state['stopped'] = True
            ex.stop()

    sim, ex, _ = run(graph, 'N05-N07 mid', 14.0, 0.03, 0.0, on_tick=stop_in_search,
                     search_deg=20.0)
    assert ex.state == STOPPED and state
    for _ in range(6):
        sim.tick(ex)
    assert sim.cmd == (0.0, 0.0)


def test_gap_config_limits():
    for bad in ({'gap_max_cm': -1.0}, {'gap_max_cm': 16.0}, {'gap_max_cm': float('nan')},
                {'gap_heading_window_cm': 0.2}, {'search_deg': -1.0}, {'search_deg': 50.0},
                {'gap_confirm': 0}, {'ir_spacing_cm': 0.1}, {'ir_spacing_cm': float('inf')}):
        with pytest.raises(ValueError):
            NavConfig(**bad)
    c = NavConfig()
    assert (c.gap_max_cm, c.gap_heading_window_cm, c.gap_confirm, c.search_deg,
            c.ir_spacing_cm) == (8.0, 4.0, 2, 0.0, 2.0)


def test_sim_gap_cuts_only_that_segment(graph):
    """시뮬 끊김: N05→N07 40~45 cm 테이프만 없음(가로선·다른 구간은 그대로)."""
    sim = Sim(graph, gaps=[('N05', 'N07', 40.0, 5.0)])
    n05 = graph.nodes['N05']
    assert sim._covered(n05.x, n05.y - 42.5) == 0.0
    assert sim._covered(n05.x, n05.y - 38.0) == 1.0 and sim._covered(n05.x, n05.y - 47.0) == 1.0
    full = Sim(graph)
    assert len(sim.rects) == len(full.rects) + 1


# ---------------------------------------------------------------- 검토 반영(H1·M2·L3)
def _reading(on, error=0.0):
    from roport_pinky_line.line_sensor import LineReading
    n = tuple(1.0 if o else 0.0 for o in on)
    return LineReading(raw=(0, 0, 0), n=n, on_line=tuple(on), error=error, lost=not any(on),
                       node=all(on))


LOST = (False, False, False)
CENTER = (False, True, False)


def _to_gap(graph, **cfg):
    """N05→N07(88.9 cm) 가짜 입력: 선 위로 20 cm 간 뒤 유실 → GAP. → (실행부, 자세, 시각)."""
    ex = RouteExecutor(graph, NavConfig(**cfg))
    ex.start(['N05', 'N07', 'N18', 'N01'], start_cross=True, start_facing='S')
    n05 = graph.nodes['N05']
    x, y, yaw, t = n05.x, n05.y + 3.0, -math.pi / 2, 0.0
    while ex.state != FOLLOW:                      # 교차선 출발 전진
        ex.step(_reading(CENTER), (x, y, yaw), t)
        y, t = y - 0.15, t + 0.05
    while ex.as_dict()['traveled_cm'] < 20.0:
        ex.step(_reading(CENTER), (x, y, yaw), t)
        y, t = y - 0.15, t + 0.05
    cmd = ex.step(_reading(LOST), (x, y, yaw), t)
    assert ex._gap is not None and cmd.linear > 0
    return ex, (x, y, yaw), t


def test_gap_stops_when_odom_frozen(graph):
    """H1: GAP 중 odom 메시지는 오는데 값이 그대로(바퀴 막힘 등) → lost_timeout(1 s) 뒤 정지, 0 한 번 뒤 무발행."""
    ex, pose, t = _to_gap(graph)
    cmds = []
    for k in range(1, 60):
        cmds.append(ex.step(_reading(LOST), pose, t + 0.05 * k))
        if not ex.active:
            break
    assert ex.state == ERROR and 'gap no progress' in ex.reason, ex.reason
    assert 0.05 * len(cmds) <= ex.cfg.lost_timeout + 0.2          # 약 1 s 안
    assert cmds[-1].linear == 0.0 and cmds[-1].angular == 0.0
    assert ex.step(_reading(LOST), pose, t + 4.0) is None


def test_gap_total_timeout(graph):
    """H1: 아주 느리게라도 움직이면(진행 있음) 시간 상한 2 + 2·gap_max/min_speed(약 12.7 s)에서 정지."""
    ex, (x, y, yaw), t = _to_gap(graph)
    limit = 2.0 + 2.0 * ex.cfg.gap_max_cm / (ex.cfg.min_speed * 100.0)
    k = 0
    while ex.active and k < 1000:
        k += 1
        y -= 0.006                                  # 0.12 cm/s: 진행 검사(1 s 0.1 cm)는 통과
        ex.step(_reading(LOST), (x, y, yaw), t + 0.05 * k)
    assert ex.state == ERROR and 'gap timeout' in ex.reason, ex.reason
    assert abs(0.05 * k - limit) <= 0.1


@pytest.mark.parametrize('side', [(True, False, False), (False, False, True)])
def test_gap_recovers_on_side_ir(graph, side):
    """M2: 옆으로 밀려 놓친 뒤 옆 IR 이 선을 다시 봄 → 추종 복귀(PD 가 그쪽으로 조향)."""
    ex, (x, y, yaw), t = _to_gap(graph)
    err = -1.0 if side[0] else 1.0                   # 왼쪽 IR = 선이 왼쪽(오차 −)
    cmd = None
    for k in range(1, 4):
        y -= 0.15
        cmd = ex.step(_reading(side, err), (x, y, yaw), t + 0.05 * k)
    assert ex._gap is None and ex.state == FOLLOW
    assert any(e['event'] == 'gap_recovered' for e in ex.events)
    assert cmd.linear > 0 and (cmd.angular > 0) == (err < 0)     # 선 쪽으로 돌림(반시계 = 왼쪽)


def test_single_lost_sample_recovers_quickly(graph):
    """M2: 1샘플 유실 → GAP 시작 → 다음 샘플부터 선 위면 gap_confirm(2) 샘플 뒤 추종(0.3 cm 안)."""
    ex, (x, y, yaw), t = _to_gap(graph)
    for k in range(1, 3):
        y -= 0.15
        ex.step(_reading(CENTER), (x, y, yaw), t + 0.05 * k)
    assert ex._gap is None and ex.state == FOLLOW
    rec = [e for e in ex.events if e['event'] == 'gap_recovered'][-1]
    assert float(rec['detail'].split()[0]) <= 0.31


def test_lost_after_max_ok_is_node_missed(graph):
    """L3: 예상 거리(max_ok)를 넘은 샘플에서 처음 유실 → GAP 명령 없이 바로 node_missed(예전과 같음)."""
    ex = RouteExecutor(graph)
    ex.start(['N05', 'N07', 'N18', 'N01'], start_cross=True, start_facing='S')
    n05 = graph.nodes['N05']
    x, y, yaw, t = n05.x, n05.y + 3.0, -math.pi / 2, 0.0
    while ex.state != FOLLOW:
        ex.step(_reading(CENTER), (x, y, yaw), t)
        y, t = y - 0.15, t + 0.05
    max_ok = ex.as_dict()['window_cm'][1]
    ex._max_ok = 30.0                              # 짧은 창으로 바꿔 빠르게 확인
    while ex._odo - ex._ref_odo + 0.15 <= 30.0:    # 다음 샘플이 처음으로 max_ok 를 넘게
        ex.step(_reading(CENTER), (x, y, yaw), t)
        y, t = y - 0.15, t + 0.05
    assert ex.state == FOLLOW
    cmd = ex.step(_reading(LOST), (x, y, yaw), t)  # max_ok 를 넘은 첫 샘플이 유실
    assert max_ok > 30.0 and ex._odo - ex._ref_odo > 30.0
    assert ex.state == ERROR and 'node_missed' in ex.reason, ex.reason
    assert cmd.linear == 0.0 and not [e for e in ex.events if e['event'] == 'gap']
