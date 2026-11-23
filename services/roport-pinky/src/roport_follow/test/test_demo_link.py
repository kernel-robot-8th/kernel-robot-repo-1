"""노트북 쪽 로봇 명령 연결(demo_link): 입력 검사, 살아 있음 신호는 브라우저가 볼 때만."""
import time

import pytest

from roport_follow.demo_link import DemoLink, demo_command


def test_demo_command_defaults_and_ranges():
    assert demo_command({'op': 'route', 'route': ['N08', 'N12']}) == {
        'op': 'route', 'route': ['N08', 'N12'], 'speed_limit_mps': 0.06}
    assert demo_command({'op': 'follow', 'card_id': 36, 'standoff_m': 0.7}) == {
        'op': 'follow', 'card_id': 36, 'standoff_m': 0.7, 'max_duration_s': 120.0, 'lost_after_s': 1.0}
    assert demo_command({'op': 'stop'}) == {'op': 'stop'}
    for bad in ({'op': 'follow', 'card_id': True}, {'op': 'follow', 'card_id': 35, 'standoff_m': 0.1},
                {'op': 'route', 'route': ['N08']}, {'op': 'route', 'route': 'N08,N12'},
                {'op': 'follow', 'card_id': 35, 'standoff_m': float('nan')}, {}):
        with pytest.raises(ValueError):
            demo_command(bad)


def test_ping_only_while_browser_watching_and_seq_increases():
    sent = []
    link = DemoLink(publish=sent.append, browser_timeout_s=0.2)
    assert not link.browser_alive()
    link.send({'op': 'stop'})                     # 버튼 누름 = 살아 있음
    assert link.browser_alive() and sent[-1]['op'] == 'stop' and sent[-1]['seq'] == 1
    link._emit({'op': 'ping'})
    assert sent[-1]['seq'] == 2 and link.info()['sent'] == 1
    time.sleep(0.25)
    assert not link.browser_alive()


def test_info_reports_robot_state_age():
    link = DemoLink(publish=lambda m: None)
    assert link.info()['robot'] is None and link.info()['robot_age_s'] is None

    class Msg:
        data = '{"observed": {"mode": "nav2"}, "active": null}'
    link._on_state(Msg())
    info = link.info()
    assert info['robot']['observed']['mode'] == 'nav2' and info['robot_age_s'] < 1


def test_navigate_command_is_checked_against_map_bounds():
    from roport_follow.demo_link import demo_command
    m = demo_command({'op': 'navigate', 'goal_id': 'demo_ARR', 'x': 2.62, 'y': -0.02, 'yaw': 0.0})
    assert m == {'op': 'navigate', 'goal_id': 'demo_ARR', 'x': 2.62, 'y': -0.02, 'yaw': 0.0}
    for bad in ({'goal_id': 'bad id', 'x': 0, 'y': 0, 'yaw': 0}, {'goal_id': 'g', 'x': 5.0, 'y': 0, 'yaw': 0},
                {'goal_id': 'g', 'x': 0, 'y': 0}, {'goal_id': 'g', 'x': 0, 'y': 0, 'yaw': 4.0},
                {'goal_id': 'g', 'x': True, 'y': 0, 'yaw': 0}):
        with pytest.raises(ValueError):
            demo_command({'op': 'navigate', **bad})


def test_follow_lost_after_is_explicit_and_bounded():
    from roport_follow.demo_link import demo_command
    assert demo_command({'op': 'follow', 'card_id': 35, 'lost_after_s': 4})['lost_after_s'] == 4.0
    for bad in (0.5, 8, True):
        with pytest.raises(ValueError):
            demo_command({'op': 'follow', 'card_id': 35, 'lost_after_s': bad})


def test_alert_is_on_or_off_only():
    from roport_follow.demo_link import demo_command
    assert demo_command({'op': 'alert', 'on': True}) == {'op': 'alert', 'on': True}
    assert demo_command({'op': 'alert', 'on': False, 'title': 'x'}) == {'op': 'alert', 'on': False}
    for bad in ({'op': 'alert'}, {'op': 'alert', 'on': 1}):
        with pytest.raises(ValueError):
            demo_command(bad)


def test_follow_turn_first_is_passed_only_when_true():
    from roport_follow.demo_link import demo_command
    assert demo_command({'op': 'follow', 'card_id': 35, 'turn_first': True})['turn_first'] is True
    assert 'turn_first' not in demo_command({'op': 'follow', 'card_id': 35, 'turn_first': False})
    with pytest.raises(ValueError):
        demo_command({'op': 'follow', 'card_id': 35, 'turn_first': 'yes'})


def test_navigate_position_tolerance_is_optional_and_bounded():
    from roport_follow.demo_link import demo_command
    base = {'op': 'navigate', 'goal_id': 'demo_ARR', 'x': 2.6, 'y': 0.0, 'yaw': 0.0}
    assert 'position_tolerance_m' not in demo_command(base)
    assert demo_command({**base, 'position_tolerance_m': 0.3})['position_tolerance_m'] == 0.3
    with pytest.raises(ValueError):
        demo_command({**base, 'position_tolerance_m': 0.5})
