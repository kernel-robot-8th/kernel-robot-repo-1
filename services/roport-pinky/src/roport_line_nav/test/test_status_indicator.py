"""status_indicator: 상태 → LCD 문구·LED 색 판정, 점멸, LCD 그림, 가짜 set_led 서비스 연동."""

import json
import threading
import time

import pytest

from roport_line_nav.status_indicator import STYLES, Blinker, Status, classify, render


def st(state, action='', frm='N19', to='N11', reason=''):
    return {'state': state, 'reason': reason,
            'leg': {'from': frm, 'to': to, 'action': action} if action else None}


@pytest.mark.parametrize('state, name', [
    (None, 'NO_SIGNAL'),
    (st('idle'), 'READY'),
    (st('follow', 'straight'), 'MOVING'),
    (st('rotate', 'right'), 'MOVING'),
    (st('pass', 'straight'), 'MOVING'),
    (st('follow', 'load'), 'DOCKING'),
    (st('rotate', 'load'), 'DOCKING'),
    (st('loading', 'load'), 'DOCKED'),
    (st('arrived', 'load'), 'READY'),
    (st('follow', 'park', 'N14', 'N16'), 'PARKING'),
    (st('reverse', 'park', 'N14', 'N16'), 'PARKING'),
    (st('arrived', 'park', 'N14', 'N16'), 'PARKED'),
    (st('handoff', 'handoff', 'N08', 'N12'), 'HANDOFF'),
    (st('stopped', reason='stop command'), 'STOPPED'),
    (st('error', 'left', reason='line_not_found after 121°'), 'ERROR'),
])
def test_classify(state, name):
    assert classify(state).name == name


def test_styles_match_request():
    # 2026-11-07 사용자 요청: 파란 바탕 흰 글자, MOVING 파랑 점멸, DOCKING 주황 점멸
    assert STYLES['MOVING'][1] == STYLES['DOCKING'][1] == STYLES['READY'][1]
    assert STYLES['MOVING'][2] == (0, 0, 255) and STYLES['MOVING'][3] > 0
    r, g, b = STYLES['DOCKING'][2]
    assert r == 255 and 60 <= g <= 140 and b == 0 and STYLES['DOCKING'][3] > 0
    assert classify(st('error', 'left', reason='x')).detail == 'x'
    assert classify(st('follow', 'straight')).detail == 'N19 -> N11'
    assert max(Status('READY').led) < 100           # READY 는 어둡게


def test_blinker():
    b = Blinker()
    b.reset(1.0, 10.0)
    assert b.on(10.0) and b.on(10.4) and not b.on(10.6) and b.on(11.1)
    b.reset(0.0, 0.0)
    assert b.on(5.0)
    b.reset(None, 0.0)
    assert not b.on(5.0)


def test_render_blue_background_white_text():
    img = render(Status('MOVING', 'N19 -> N11'))
    assert img.size == (320, 240)
    assert img.getpixel((2, 2)) == STYLES['MOVING'][1]
    assert (255, 255, 255) in set(img.getdata())       # 흰 글자가 그려짐
    err = render(Status('ERROR', 'line_not_found after 121°'))
    assert err.getpixel((2, 2)) == STYLES['ERROR'][1]


def test_node_blinks_led_through_set_led_service():
    """격리 도메인: 가짜 set_led 서버 + line_nav/state 발행 → fill(파랑)/clear 번갈아."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    pinky = pytest.importorskip('pinky_interfaces.srv')
    from roport_line_nav.status_indicator import make_node

    ctx = Context()
    rclpy.init(context=ctx, domain_id=193)
    calls = []
    try:
        peer = rclpy.create_node('fake_led', context=ctx, namespace='/status_pytest')

        def set_led(req, res):
            calls.append((req.command, (req.r, req.g, req.b), time.monotonic()))
            res.success = True
            return res

        peer.create_service(pinky.SetLed, 'set_led', set_led)
        pub = peer.create_publisher(String, 'line_nav/state', 10)
        node = make_node(context=ctx, namespace='/status_pytest',
                         parameter_overrides=[Parameter('lcd', value=False)])
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(peer)
        ex.add_node(node)
        stop = threading.Event()
        def spin():
            while not stop.is_set():
                ex.spin_once(timeout_sec=0.02)

        th = threading.Thread(target=spin)
        th.start()
        try:
            t_end = time.monotonic() + 3.5
            while time.monotonic() < t_end:
                pub.publish(String(data=json.dumps(st('follow', 'straight'))))
                time.sleep(0.1)
        finally:
            stop.set()
            th.join()
        assert node.status.name == 'MOVING'
        kinds = [c[0] for c in calls]
        assert kinds.count('fill') >= 2 and kinds.count('clear') >= 1, kinds
        assert all(c[1] == (0, 0, 255) for c in calls if c[0] == 'fill')
        for a, b in zip(kinds, kinds[1:]):             # 같은 명령을 연달아 보내지 않음
            assert a != b
        node.destroy_node()
        peer.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)


def test_battery_bar():
    from roport_line_nav.status_indicator import battery_text
    assert battery_text(None) == '--' and battery_text(77.6) == '78%' and battery_text(120) == '100%'
    plain = render(Status('MOVING'))
    with_batt = render(Status('MOVING'), battery=15)
    assert plain.size == with_batt.size == (320, 240)
    bottom = [with_batt.getpixel((x, 210)) for x in range(320)]
    assert any(p != STYLES['MOVING'][1] for p in bottom)      # 맨 아래에 배터리 그림
    assert all(plain.getpixel((x, 210)) == STYLES['MOVING'][1] for x in range(320))
    assert render(Status('READY'), battery=None).size == (320, 240)
