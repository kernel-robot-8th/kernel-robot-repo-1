"""일시정지·이어 가기·끝 기억(FFT1-172). 2D 시뮬(test/sim.py)과 합성 입력으로만 확인한다(실로봇 미검증).

수동 후진 back{to}·장애물 양보 후진과 그 후진 상태 피드백 제어는 2026-11-15 사용자 결정으로 없앴다.
장애물 정지는 test_obstacle.py 에 있다.

예시 경로 = N07(교차선 출발, 동쪽) → N23(직진 통과) → N19(우회전) → N11(상차, P7).
"""

import os

import pytest
from sim import Sim

from roport_line_nav.executor import (ARRIVED, FOLLOW, LOADING, PAUSED, ROTATE, NavConfig,
                                      RouteExecutor)
from roport_line_nav.graph import Graph

LAYOUT = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                      'roport_airport.yaml')
ROUTE = ['N07', 'N23', 'N19', 'N11']


@pytest.fixture(scope='module')
def graph():
    return Graph.load(LAYOUT)


def _start(graph, delay=0.0, odom=1.0, **cfg):
    sim = Sim(graph, cmd_delay_s=delay, odom_scale=odom)
    sim.place('N07', 'E', 0.0)
    ex = RouteExecutor(graph, NavConfig(**cfg))
    ex.start(ROUTE, start_cross=True, start_facing='E')
    return sim, ex


def _at(idx, cm, state=FOLLOW):
    return lambda e: e.state == state and e.index == idx and e.as_dict()['traveled_cm'] >= cm


def _reading(on, error=0.0):
    from roport_pinky_line.line_sensor import LineReading
    n = tuple(1.0 if o else 0.0 for o in on)
    return LineReading(raw=(0, 0, 0), n=n, on_line=tuple(on), error=error, lost=not any(on),
                       node=all(on))


# ---------------------------------------------------------------- 일시정지·이어 가기
def test_pause_resume_keeps_leg_and_completes(graph):
    sim, ex = _start(graph, delay=0.15)
    assert sim.run(ex, _at(1, 20.0))
    before = ex.as_dict()
    ex.pause()
    cmd = ex.step(sim.sensor.update(sim.raw()), sim.pose(), sim.t)
    assert ex.state == PAUSED and (cmd.linear, cmd.angular) == (0.0, 0.0)
    st = ex.as_dict()
    assert st['pause'] == {'from': FOLLOW, 'settled': False, 'manual': True}
    assert st['pause_reason'] == 'manual'
    for _ in range(200):                     # 10 s 멈춤(시간 제한 없음)
        sim.tick(ex)
    st = ex.as_dict()
    assert st['state'] == PAUSED and st['pause']['settled'] is True
    assert st['window_cm'] is None and st['leg'] == before['leg']
    assert any(e['event'] == 'paused_settled' for e in ex.events)
    assert ex.pause() == 'already paused'
    ex.resume()
    st = ex.as_dict()
    assert ex.state == FOLLOW and st['window_cm'] == before['window_cm']
    assert st['traveled_cm'] >= before['traveled_cm'] and st['pause_reason'] is None
    assert sim.run(ex, lambda e: e.state == LOADING)
    ex.load_done()
    sim.run(ex, lambda e: not e.active)
    assert ex.state == ARRIVED, ex.as_dict()


def test_pause_during_rotate_then_resume(graph):
    sim, ex = _start(graph)
    assert sim.run(ex, lambda e: e.state == ROTATE and e.as_dict()['rotated_deg'] > 30)
    ex.pause()
    for _ in range(40):
        sim.tick(ex)
    assert sim.cmd == (0.0, 0.0)
    ex.resume()
    assert ex.state == ROTATE
    assert sim.run(ex, lambda e: e.state == LOADING)


def test_pause_resume_shifts_timers(graph):
    """멈춘 시간은 라인 놓침 시간 등에 세지 않는다."""
    ex = RouteExecutor(graph)
    ex.start(['N14', 'N16'])
    ex.step(_reading((False, True, False)), (0.0, 0.0, 0.0), 0.0)
    ex.pause()
    ex.step(_reading((False, True, False)), (0.0, 0.0, 0.0), 60.0)
    ex.resume()
    assert ex._lost_since is None
    ex.step(_reading((False, False, False)), (0.0, 0.0, 0.0), 60.05)
    assert ex.state == FOLLOW


def test_pause_resume_rejections(graph):
    ex = RouteExecutor(graph)
    with pytest.raises(ValueError):
        ex.pause()
    with pytest.raises(ValueError):
        ex.resume()
    ex.start(['N14', 'N16'])
    with pytest.raises(ValueError, match='not paused'):
        ex.resume()


def test_back_command_removed(graph):
    """수동 후진은 없앰(2026-11-15): 공개 API·상태 필드가 없다."""
    ex = RouteExecutor(graph)
    for name in ('back', 'back_targets', 'back_why'):
        assert not hasattr(ex, name)
    st = ex.as_dict()
    assert 'back_targets' not in st and 'back_why' not in st


def test_reverse_follow_removed(graph):
    """양보 후진·후진 상태 피드백을 없앰(2026-11-15): 설정·속도 키·상태 필드가 없다. 주차 후진 reverse_speed 는 그대로."""
    ex = RouteExecutor(graph)
    assert ex.cfg.reverse_speed == 0.02
    for name in ('reverse_follow_speed', 'reverse_wn', 'reverse_zeta', 'reverse_lost_cm',
                 'reverse_max_heading_deg', 'yield_cm', 'approach_window_s', 'approach_min_cm',
                 'yield_max_cm', 'rear_clear_cm'):
        assert not hasattr(ex.cfg, name)
        with pytest.raises(TypeError):
            NavConfig(**{name: 1.0})
    with pytest.raises(TypeError):
        ex.set_speed(reverse_speed=0.03)
    sp = ex.as_dict()['speed']
    assert not {'reverse_follow_speed', 'reverse_follow_min', 'reverse_follow_max'} & set(sp)
    assert 'reverse' not in ex.as_dict()
    for name in ('_reverse_omega', '_rv_speed', '_psi_of', '_yield_step'):
        assert not hasattr(ex, name)
    ex.set_speed(base_speed=0.04)
    assert ex.events[-1]['detail'] == '0.040 m/s, 0.40 rad/s'      # 기존 문구 그대로


def test_no_pause_route_unchanged(graph):
    """pause 를 쓰지 않으면 기존 경로 주행과 같다(이벤트 종류에 새 이벤트 없음)."""
    sim, ex = _start(graph, delay=0.15)
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    assert ex.state == ARRIVED
    kinds = {e['event'] for e in ex.events}
    assert not kinds & {'pause', 'resume', 'paused_settled', 'obstacle_stop', 'obstacle_clear',
                        'obstacle_warn'}
    st = ex.as_dict()
    assert st['pause'] is None and st['pause_reason'] is None


# ---------------------------------------------------------------- 끝 기억(이어서 출발)
def _to_end(graph, route):
    sim = Sim(graph)
    sim.place(route[0], graph.heading(route[0], route[1]), 0.0)
    ex = RouteExecutor(graph)
    ex.start(route)
    sim.run(ex, lambda e: not e.active, max_s=300.0)
    assert sim.run(ex, lambda e: e.settled, max_s=sim.t + 5.0)
    return sim, ex


def _to_n11(graph, delay=0.0):
    """N07 → N11 상차 → 되돌아 섬(북쪽). → 끝 기억 있는 상태."""
    sim, ex = _start(graph, delay=delay)
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    assert ex.state == ARRIVED
    for _ in range(10):
        sim.tick(ex)
    return sim, ex


def test_end_report_after_forward_arrivals(graph):
    """end 보고: 주행 중 null, 상차·handoff 뒤 node·facing(passed 는 back 을 없애 늘 [])."""
    sim, ex = _start(graph)
    assert ex.as_dict()['end'] is None
    sim.run(ex, lambda e: not e.active, on_loading=lambda s: True)
    for _ in range(10):
        sim.tick(ex)
    assert ex.as_dict()['end'] == {'node': 'N11', 'facing': 'N', 'moved': False, 'passed': []}
    ex.stop()
    assert ex.as_dict()['end'] is None
    sim, ex = _to_end(graph, ['N09', 'N08', 'N12'])
    e = ex.as_dict()['end']
    assert e['node'] == 'N12' and e['passed'] == []


@pytest.mark.parametrize('how', ('odom_3cm', 'odom_lost'))
def test_moved_end_memory_dropped_on_straight_start(graph, how):
    """결정 1(2026-11-12): 끝난 뒤 odom 이 2 cm 넘게 바뀌었거나 odom 을 모르면 같은 노드 직진 출발이라도 끝 기억을 버리고
    손으로 놓은 출발과 같게(거절 안 함): 비율 창만, gate_early 없음."""
    sim, ex = _to_n11(graph, delay=0.15)
    if how == 'odom_3cm':                 # 로봇은 그대로, odom 만 3 cm 바뀜(미끄러짐·조종 흉내)
        ox, oy, oyaw = sim.odom
        sim.odom = (ox, oy + 3.0, oyaw)
        for _ in range(10):
            sim.tick(ex)
        assert ex.as_dict()['end']['moved'] is True
    else:
        ex.step(sim.sensor.update(sim.raw()), None, sim.t)
        assert ex.as_dict()['end']['moved'] is None
    ex.start(['N11', 'N19', 'N23', 'N21'])
    kinds = [e['event'] for e in ex.events]
    assert 'end_dropped' in kinds and ex._end is None
    c = ex.cfg
    assert ex._min_ok == pytest.approx(max(0.0, c.gate_min * ex._expected - c.gate_slack_cm))


def test_unmoved_end_memory_kept_on_straight_start(graph):
    """대조: 끝난 자세 그대로면 끝 기억(s0, gate_early)을 쓰고 end_dropped 없음."""
    sim, ex = _to_n11(graph, delay=0.15)
    ex.start(['N11', 'N19', 'N23', 'N21'])
    assert 'end_dropped' not in [e['event'] for e in ex.events]
    assert ex._min_ok == pytest.approx(ex._expected - ex.cfg.gate_early_cm)
    sim.run(ex, lambda e: not e.active, max_s=sim.t + 300.0, on_loading=lambda s: True)
    assert ex.state == ARRIVED, ex.reason


# ---------------------------------------------------------------- 실로봇 정지 확인 잡음(2026-11-12)
def _jitter_hold(ex, sim, seconds, yaw_tick=0.0018, x_tick=0.017, every=4, drift_cm_s=0.0):
    """일시정지 뒤 실로봇처럼 odom 이 가끔 1틱씩 튀는 입력(20 Hz)을 넣는다. → settled 기록."""
    x, y, yaw = sim.pose()
    t = sim.t
    seen = []
    for n in range(int(seconds * 20)):
        t += 0.05
        j = 1 if (n // every) % 2 else 0          # every 샘플마다 1틱 올라갔다 내려옴
        p = (x + j * x_tick + drift_cm_s * (n * 0.05), y, yaw + j * yaw_tick)
        ex.step(_reading((False, True, False)), p, t, pose_stamp=t)
        seen.append(ex.settled)
    return seen


def _paused_on_leg(graph):
    sim, ex = _start(graph)
    assert sim.run(ex, _at(2, 15.0), max_s=200.0)
    ex.pause()
    return sim, ex


def test_settled_survives_encoder_tick_jitter(graph):
    """실로봇: 서 있어도 엔코더 1틱(yaw 0.0018 rad = 메시지 간 0.036 rad/s)이 튀면 예전엔 정지 확인이 풀렸다.
    창(0.3 s) 안 자세 변화로 보면 정지 유지(paused_settled 표시가 이 정지 확인을 씀)."""
    sim, ex = _paused_on_leg(graph)
    seen = _jitter_hold(ex, sim, 3.0)
    assert all(seen[10:]), seen
    assert ex.as_dict()['pause']['settled'] is True


def test_settled_false_when_really_creeping(graph):
    """창 안에서 실제로 0.5 cm/s 로 밀리면 정지로 보지 않는다."""
    sim, ex = _paused_on_leg(graph)
    seen = _jitter_hold(ex, sim, 2.0, drift_cm_s=0.5)
    assert not any(seen[10:])
