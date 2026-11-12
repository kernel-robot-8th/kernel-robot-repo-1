"""LineNavNode 를 실제 rclpy 로 띄워 명령·정지·감시를 확인한다(로봇·센서 없이).

안전: 별도 ROS_DOMAIN_ID(context) + 테스트 전용 namespace 를 써서
같은 네트워크/기기의 실제 로봇 /cmd_vel 에는 발행되지 않게 한다.
"""

import importlib.util
import json
from pathlib import Path
import threading
import time

import pytest

_NEEDED = ('rclpy', 'geometry_msgs', 'std_msgs', 'nav_msgs', 'roport_pinky_line')
_MISSING = [m for m in _NEEDED if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(bool(_MISSING), reason=f'missing: {_MISSING}')

if not _MISSING:
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String, UInt16MultiArray

    from roport_line_nav.line_nav_node import parse_command

TEST_DOMAIN_ID = 197  # 라인 추종 테스트 198, 콘솔 199 와 겹치지 않게
NS = '/roport_line_nav_pytest'
ROOT = Path(__file__).resolve().parents[2]
LAYOUT = ROOT / 'roport_gz_world' / 'layout' / 'roport_airport.yaml'
CONFIG = ROOT / 'roport_pinky_line' / 'config' / 'robots.yaml'
CENTER = [2308, 388, 2233]     # pinky_d660: 가운데만 테이프
ROUTE = ['N16', 'N14', 'N02', 'N10', 'N18', 'N01']


class Harness:
    def __init__(self, log_dir, extra=(), dock_file=None):
        from roport_line_nav.line_nav_node import LineNavNode
        self.ctx = Context()
        rclpy.init(context=self.ctx, domain_id=TEST_DOMAIN_ID)
        # 홈 ~/roport_dock.yaml 을 읽거나 쓰지 않게 테스트 임시 경로
        self.dock_file = str(dock_file or Path(log_dir).parent / 'roport_dock.yaml')
        self.node = LineNavNode(
            context=self.ctx, namespace=NS,
            parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('layout_file', value=str(LAYOUT)),
                Parameter('log_dir', value=str(log_dir)),     # 홈 ~/roport_logs 에 쓰지 않게
                Parameter('dock_file', value=self.dock_file),
                *extra,
            ])
        self.probe = rclpy.create_node('probe', context=self.ctx, namespace=NS)
        self.cmds = []
        self.states = []
        self.probe.create_subscription(
            Twist, 'cmd_vel', lambda m: self.cmds.append((m.linear.x, m.angular.z)), 10)
        self.probe.create_subscription(
            String, 'line_nav/state', lambda m: self.states.append(json.loads(m.data)), 10)
        self.ir_pub = self.probe.create_publisher(UInt16MultiArray, 'ir_sensor/range', 10)
        self.odom_pub = self.probe.create_publisher(Odometry, 'odom', 10)
        self.cmd_pub = self.probe.create_publisher(String, 'line_nav/command', 10)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        self.ex.add_node(self.probe)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        self.wait(lambda: (self.node.cmd_pub.get_subscription_count() > 0
                           and self.cmd_pub.get_subscription_count() > 0
                           and self.ir_pub.get_subscription_count() > 0
                           and self.odom_pub.get_subscription_count() > 0))

    @staticmethod
    def wait(cond, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not cond() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert cond()

    def feed(self, count=4, ir=CENTER, odom=True):
        for _ in range(count):
            if odom:
                self.odom_pub.publish(Odometry())
            self.ir_pub.publish(UInt16MultiArray(data=ir))
            time.sleep(0.05)

    def command(self, data):
        self.cmd_pub.publish(String(data=json.dumps(data)))

    def close(self):
        self.ex.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.probe.destroy_node()
        rclpy.shutdown(context=self.ctx)


@pytest.fixture
def ros(tmp_path):
    h = Harness(tmp_path / 'logs')
    yield h
    h.close()


def test_topics_are_isolated(ros):
    assert ros.node.cmd_pub.topic_name == f'{NS}/cmd_vel'


def test_idle_never_publishes(ros):
    ros.feed(6)
    assert ros.cmds == []
    ros.wait(lambda: ros.states and ros.states[-1]['state'] == 'idle')
    assert ros.states[-1]['graph_version'].startswith('line-v14')


def test_start_moves_then_stop_publishes_zero(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(4)
    assert any(v > 0 for v, _ in ros.cmds)
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')
    ros.wait(lambda: ros.cmds and ros.cmds[-1] == (0.0, 0.0))
    n = len(ros.cmds)
    ros.feed(4)
    assert len(ros.cmds) == n          # 정지 뒤에는 발행하지 않음(다른 주행 소스를 가리지 않게)


def test_bad_command_rejected_and_reported(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ['N16', 'N02']})
    ros.wait(lambda: ros.states and ros.states[-1]['last_command']
             and ros.states[-1]['last_command']['ok'] is False)
    assert 'not linked' in ros.states[-1]['last_command']['detail']
    assert ros.node.ex.state == 'idle' and ros.cmds == []


def test_ir_timeout_stops(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(2)
    ros.wait(lambda: ros.node.ex.state == 'error', timeout=2.0)   # IR 이 끊기면 0.3 s
    assert ros.node.ex.reason == 'no_ir'
    ros.wait(lambda: ros.cmds[-1] == (0.0, 0.0))


def test_no_odom_stops(ros):
    ros.feed(3, odom=False)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'error')
    assert ros.node.ex.reason == 'no_odom'


@pytest.mark.parametrize('text', [
    'not json', '[]', '{"cmd": 1}', '{"cmd": "fly"}',
    '{"cmd": "start", "route": "N16"}',
    '{"cmd": "start", "route": ["N16", "n14; rm -rf"]}',
    '{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": 99}',
    '{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": true}',
    '{"cmd": "speed"}', '{"cmd": "speed", "base_speed": "fast"}',
    '{"cmd": "speed", "base_speed": NaN}', '{"cmd": "speed", "rotate_speed": true}',
    '{"cmd": "dock"}', '{"cmd": "dock", "node": "n11"}',
    '{"cmd": "dock", "node": "N11", "advance_cm": 1, "rotate_deg": 90}',
    '{"cmd": "dock", "node": "N11", "advance_cm": "1", "rotate_deg": 90, "move_cm": 0}',
    '{"cmd": "dock", "node": "N11", "advance_cm": true, "rotate_deg": 90, "move_cm": 0}',
    '{"cmd": "dock", "node": "N11", "advance_cm": NaN, "rotate_deg": 90, "move_cm": 0}',
    '{"cmd": "dock", "node": "N11", "reset": "yes"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": 1, "start_facing": "S"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": true}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": true, "start_facing": "south"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_facing": "S"}',
    '{"cmd": "start", "route": ["N07", "N18"], "start_cross": true, "start_facing": "S",'
    ' "start_offset_cm": 3}',
    '{"cmd": "back"}', '{"cmd": "back", "to": "n23"}', '{"cmd": "back", "to": 23}',
    '{"cmd": "speed", "reverse_speed": "slow"}', '{"cmd": "speed", "reverse_speed": true}',
])
def test_parse_command_rejects(text):
    with pytest.raises(ValueError):
        parse_command(text)


def test_parse_command_ok():
    assert parse_command('{"cmd": "stop"}') == ('stop', {})
    cmd, args = parse_command('{"cmd": "start", "route": ["N16", "N14"], "start_offset_cm": 7}')
    assert cmd == 'start' and args == {'route': ['N16', 'N14'], 'start_offset_cm': 7.0,
                                       'start_cross': False, 'start_facing': None}
    cmd, args = parse_command('{"cmd": "start", "route": ["N07", "N18", "N01"], '
                              '"start_cross": true, "start_facing": "S"}')
    assert args['start_cross'] is True and args['start_facing'] == 'S'
    assert parse_command('{"cmd": "dock", "node": "N11", "advance_cm": 1.5, "rotate_deg": 96, '
                         '"move_cm": -2}') == ('dock', {'node': 'N11', 'reset': False,
                                                        'advance_cm': 1.5, 'rotate_deg': 96.0,
                                                        'move_cm': -2.0})
    assert parse_command('{"cmd": "dock", "node": "N11", "reset": true}') == \
        ('dock', {'node': 'N11', 'reset': True})
    assert parse_command('{"cmd": "pause"}') == ('pause', {})
    assert parse_command('{"cmd": "resume"}') == ('resume', {})
    assert parse_command('{"cmd": "back", "to": "N23"}') == ('back', {'to': 'N23'})
    assert parse_command('{"cmd": "speed", "reverse_speed": 0.03}') == \
        ('speed', {'reverse_speed': 0.03})


def test_speed_command(ros):
    """직선·회전 속도 바꾸기(상한 안만), 상태에 보고."""
    ros.feed(3)
    ros.command({'cmd': 'speed', 'base_speed': 0.05, 'rotate_speed': 0.5})
    ros.wait(lambda: ros.states and ros.states[-1]['speed']['base_speed'] == 0.05)
    assert ros.states[-1]['speed']['rotate_speed'] == 0.5
    # 노드 앞 감속은 별도 파라미터(approach_speed·approach_lead_cm): 직선 속도를 바꿔도 그대로
    assert ros.states[-1]['speed']['approach_speed'] == 0.03
    assert ros.states[-1]['speed']['approach_lead_cm'] == 5.0
    ros.command({'cmd': 'speed', 'base_speed': 0.2})
    ros.wait(lambda: ros.states and ros.states[-1]['last_command']['cmd'] == 'speed'
             and not ros.states[-1]['last_command']['ok'])
    assert ros.node.ex.cfg.base_speed == 0.05
    assert ros.cmds == []                       # 속도 바꾸기만으로는 움직이지 않음


def test_approach_parameters(tmp_path):
    """approach_speed·approach_lead_cm 파라미터: 범위 안이면 적용, 밖이면 노드가 뜨지 않음."""
    h = Harness(tmp_path / 'logs', extra=[Parameter('base_speed', value=0.05),
                                          Parameter('approach_speed', value=0.02),
                                          Parameter('approach_lead_cm', value=8.0)])
    try:
        assert h.node.ex.cfg.approach_lead_cm == 8.0
        assert h.node.ex.approach_speed == 0.02 and h.node.ex.cfg.base_speed == 0.05
    finally:
        h.close()
    from roport_line_nav.line_nav_node import LineNavNode
    ctx = Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    try:
        with pytest.raises(ValueError, match='approach_speed'):
            LineNavNode(context=ctx, namespace=NS, parameter_overrides=[
                Parameter('robot', value='pinky_d660'),
                Parameter('calibration_file', value=str(CONFIG)),
                Parameter('layout_file', value=str(LAYOUT)),
                Parameter('log_dir', value=str(tmp_path / 'logs2')),
                Parameter('dock_file', value=str(tmp_path / 'dock2.yaml')),
                Parameter('approach_speed', value=0.2)])     # 하드 상한 0.15 초과
    finally:
        rclpy.shutdown(context=ctx)


# ---------------------------------------------------------------- 정밀 도킹(FFT1-112)
def _last(ros, cmd):
    ros.wait(lambda: ros.states and (ros.states[-1]['last_command'] or {}).get('cmd') == cmd)
    return ros.states[-1]['last_command']


def test_dock_command_saves_and_reloads(tmp_path):
    import yaml
    h = Harness(tmp_path / 'logs')
    try:
        assert h.node.ex.dock.as_dict() == {}            # 파일 없음 = 기본
        h.feed(2)
        h.command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1.5, 'rotate_deg': 96,
                   'move_cm': 2})
        last = _last(h, 'dock')
        assert last['ok'], last
        h.wait(lambda: h.states[-1]['dock'].get('N11'))
        assert h.states[-1]['dock'] == {
            'N11': {'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': 2.0}}
        h.command({'cmd': 'dock', 'node': 'N01', 'advance_cm': -2, 'rotate_deg': 84,
                   'move_cm': -3})
        h.wait(lambda: 'N01' in h.states[-1]['dock'])
        data = yaml.safe_load(Path(h.dock_file).read_text())
        assert data['version'] == 1 and set(data['nodes']) == {'N01', 'N11'}
        h.command({'cmd': 'dock', 'node': 'N01', 'reset': True})
        h.wait(lambda: 'N01' not in h.states[-1]['dock'])
        assert h.cmds == []                              # 값 바꾸기만으로는 움직이지 않음
    finally:
        h.close()
    h = Harness(tmp_path / 'logs')                       # 다시 켜면 파일에서 읽음
    try:
        assert h.node.ex.dock.as_dict() == {
            'N11': {'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': 2.0}}
    finally:
        h.close()


def test_dock_command_rejected(ros):
    ros.feed(3)
    for bad in ({'node': 'N11', 'advance_cm': 7, 'rotate_deg': 90, 'move_cm': 0},
                {'node': 'N11', 'advance_cm': 0, 'rotate_deg': 130, 'move_cm': 0},
                {'node': 'N16', 'advance_cm': 0, 'rotate_deg': 90, 'move_cm': 0}):
        n = len(ros.states)
        ros.command({'cmd': 'dock', **bad})
        ros.wait(lambda: len(ros.states) > n and ros.states[-1]['last_command']['cmd'] == 'dock')
        assert ros.states[-1]['last_command']['ok'] is False
    assert not Path(ros.dock_file).exists() and ros.node.ex.dock.as_dict() == {}
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1, 'rotate_deg': 90, 'move_cm': 0})
    ros.wait(lambda: ros.states[-1]['last_command']['cmd'] == 'dock')
    last = ros.states[-1]['last_command']
    assert last['ok'] is False and 'while driving' in last['detail']
    assert ros.node.ex.state == 'follow' and not Path(ros.dock_file).exists()
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')


def test_broken_dock_file_uses_defaults(tmp_path):
    bad = tmp_path / 'roport_dock.yaml'
    bad.write_text('version: 1\nnodes:\n  N11: {advance_cm: 99, rotate_deg: 90, move_cm: 0}\n')
    h = Harness(tmp_path / 'logs', dock_file=bad)
    try:
        assert h.node.ex.dock.as_dict() == {}
        h.feed(2)
        h.command({'cmd': 'dock', 'node': 'N11', 'advance_cm': 1, 'rotate_deg': 90, 'move_cm': 0})
        assert _last(h, 'dock')['ok']                    # 깨진 파일은 새 값으로 원자적 교체
        assert 'advance_cm: 1.0' in bad.read_text()
    finally:
        h.close()


def test_start_cross_command(ros):
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ['N07', 'N18', 'N01'], 'start_cross': True,
                 'start_facing': 'S'})
    ros.wait(lambda: ros.node.ex.state == 'advance')
    ros.feed(2)
    assert any(v > 0 for v, _ in ros.cmds)
    ros.command({'cmd': 'stop'})
    ros.wait(lambda: ros.node.ex.state == 'stopped')
    ros.command({'cmd': 'start', 'route': ROUTE, 'start_cross': True, 'start_facing': 'S'})
    last = _last(ros, 'start')
    assert last['ok'] is False and 'bar' in last['detail']


# ---------------------------------------------------------------- 라인 유실 복구(FFT1-163)
def test_gap_parameters_and_state(tmp_path):
    h = Harness(tmp_path / 'logs', extra=[Parameter('gap_max_cm', value=5.0),
                                          Parameter('gap_confirm', value=3),
                                          Parameter('search_deg', value=15.0),
                                          Parameter('side_rise_only', value=False),
                                          Parameter('side_rise_off_samples', value=3)])
    try:
        c = h.node.ex.cfg
        assert (c.gap_max_cm, c.gap_confirm, c.search_deg) == (5.0, 3, 15.0)
        assert (c.side_rise_only, c.side_rise_off_samples, c.side_rise_tee) == (False, 3, False)
        h.feed(2)
        h.wait(lambda: h.states and 'recovery' in h.states[-1])
        assert h.states[-1]['recovery'] is None
    finally:
        h.close()


def test_pause_resume_back_commands(ros):
    """pause → 0 속도·paused, resume → 주행, 정지 확인 뒤 back(출발 노드, 가까우면 odom) → arrived."""
    ros.feed(3)
    ros.command({'cmd': 'start', 'route': ROUTE})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(2)
    ros.command({'cmd': 'back', 'to': 'N16'})          # 멈추지 않았으면 거부
    ros.wait(lambda: ros.states and (ros.states[-1]['last_command'] or {}).get('cmd') == 'back')
    assert ros.states[-1]['last_command']['ok'] is False and ros.node.ex.state == 'follow'
    ros.command({'cmd': 'pause'})
    ros.wait(lambda: ros.node.ex.state == 'paused')
    ros.feed(10)                                        # odom 이 그대로 → 0.3 s 뒤 정지 확인
    ros.wait(lambda: ros.states and (ros.states[-1].get('pause') or {}).get('settled'))
    assert ros.cmds[-1] == (0.0, 0.0)
    ros.command({'cmd': 'resume'})
    ros.wait(lambda: ros.node.ex.state == 'follow')
    ros.feed(2)
    assert ros.cmds[-1][0] > 0
    ros.command({'cmd': 'pause'})
    ros.feed(10)
    ros.wait(lambda: ros.node.ex.settled)
    ros.command({'cmd': 'speed', 'reverse_speed': 0.03})
    ros.wait(lambda: ros.states and ros.states[-1]['speed']['reverse_follow_speed'] == 0.03)
    assert 'reverse 0.030' in ros.states[-1]['last_command']['detail']
    ros.command({'cmd': 'back', 'to': 'N16'})
    ros.wait(lambda: ros.node.ex.state == 'arrived')
    st = ros.states[-1]
    assert st['route'] == ['N16'] and st['reverse']['phase'] == 'done'
    assert st['last_command'] == {'cmd': 'back', 'ok': True, 'detail': 'back to N16'}
