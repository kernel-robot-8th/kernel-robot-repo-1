"""노트북 쪽 로봇 명령 연결(demo_link): 입력 검사, 살아 있음 신호는 브라우저가 볼 때만."""
import time

import pytest

from roport_follow.demo_link import DemoLink, demo_command


def test_demo_command_defaults_and_ranges():
    assert demo_command({'op': 'route', 'route': ['N08', 'N12']}) == {
        'op': 'route', 'route': ['N08', 'N12'], 'speed_limit_mps': 0.06}
    assert demo_command({'op': 'follow', 'card_id': 36, 'standoff_m': 0.7}) == {
        'op': 'follow', 'card_id': 36, 'standoff_m': 0.7, 'max_duration_s': 120.0}
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
