"""Flask 라우트 테스트 (ROS·로봇 없이 가짜 노드로)."""

import importlib.util
import os

import pytest

from pinky_test_console.map_store import map_base_path, MapNameError
from pinky_test_console.teleop import TeleopGuard

# 모듈 단위 importorskip 은 이 환경에서 디렉터리 전체 수집을 멈추게 해서 skipif 로 처리
pytestmark = pytest.mark.skipif(importlib.util.find_spec('flask') is None,
                                reason='python3-flask not installed')


def create_app(node, web_dir):
    from pinky_test_console.web_app import create_app as _create_app
    return _create_app(node, web_dir)


WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


class FakeNode:
    def __init__(self, navigating=False):
        self.calls = []
        self.guard = TeleopGuard()
        self.navigating = navigating

    def get_info(self):
        return {'mode': 'slam'}

    def get_state_snapshot(self, versions, layers):
        self.calls.append(('state', versions, layers))
        return {'map': None}

    def get_status(self):
        return {'memory': None}

    def is_navigating(self):
        return self.navigating

    def list_maps(self):
        return {'map_dir': '/x', 'maps': []}

    def send_goal(self, x, y, yaw):
        self.calls.append(('goal', x, y, yaw))
        return True

    def set_initial_pose(self, x, y, yaw):
        self.calls.append(('init', x, y, yaw))
        return True

    def cancel_goal(self):
        self.calls.append(('cancel',))
        self.navigating = False
        return True

    def teleop(self, lin, ang):
        nav_cancelled = False
        if self.navigating:
            nav_cancelled = self.cancel_goal()
        lin, ang = self.guard.update(lin, ang)
        self.calls.append(('teleop', lin, ang))
        return {'linear': lin, 'angular': ang, 'nav_cancelled': nav_cancelled}

    def teleop_stop(self):
        self.calls.append(('teleop_stop',))
        self.guard.stop()

    def emergency_stop(self):
        self.calls.append(('estop',))
        return {'nav_cancelled': False}

    def slam_reset(self):
        return True

    def slam_save_map(self, name, overwrite):
        map_base_path('/x', name)  # 실제 노드와 같은 검사
        if name == 'exists' and not overwrite:
            return {'success': False, 'exists': True}
        self.calls.append(('save', name, overwrite))
        return {'success': True, 'name': name}


@pytest.fixture
def node():
    return FakeNode()


@pytest.fixture
def client(node):
    app = create_app(node, WEB_DIR)
    app.testing = True
    return app.test_client()


def test_index_and_assets(client):
    for path in ('/', '/app.js', '/style.css', '/index.html'):
        r = client.get(path)
        assert r.status_code == 200, path


def test_only_whitelisted_static_files(client):
    # 원본은 스크립트 폴더 전체가 static 이라 서버 코드까지 내려받을 수 있었다
    for path in ('/web_server.py', '/../package.xml', '/web/../setup.py', '/%2e%2e/setup.py'):
        assert client.get(path).status_code == 404, path


def test_security_headers(client):
    r = client.get('/')
    assert "script-src 'self'" in r.headers['Content-Security-Policy']
    assert r.headers['X-Frame-Options'] == 'DENY'
    assert client.get('/api/status').headers['Cache-Control'] == 'no-store'


def test_html_has_no_external_or_inline_code():
    html = open(os.path.join(WEB_DIR, 'index.html')).read()
    assert 'http://' not in html.split('-->', 1)[1]
    assert 'https://' not in html.split('-->', 1)[1]
    assert '<script>' not in html and 'style=' not in html and 'onclick' not in html


def test_post_requires_json(client, node):
    r = client.post('/api/teleop', data='linear=1&angular=1',
                    content_type='application/x-www-form-urlencoded')
    assert r.status_code == 400
    r = client.post('/api/estop', data='{}', content_type='text/plain')
    assert r.status_code == 400
    assert ('estop',) not in node.calls


def test_teleop_clamped(client, node):
    r = client.post('/api/teleop', json={'linear': 5.0, 'angular': -9.0})
    assert r.status_code == 200
    d = r.get_json()
    assert d['linear'] == pytest.approx(0.10)
    assert d['angular'] == pytest.approx(-0.6)


@pytest.mark.parametrize('body', [
    {'linear': 'fast', 'angular': 0},
    {'linear': True, 'angular': 0},
    {'linear': 0.1},
    {'linear': None, 'angular': 0},
    [],
])
def test_teleop_bad_input_stops(client, node, body):
    node.guard.update(0.1, 0)
    r = client.post('/api/teleop', json=body)
    assert r.status_code == 400
    assert not node.guard.active
    assert not any(c[0] == 'teleop' for c in node.calls)


def test_teleop_nan_rejected(client, node):
    r = client.post('/api/teleop', data='{"linear": NaN, "angular": 0}',
                    content_type='application/json')
    assert r.status_code == 400


def test_teleop_cancels_nav_first():
    node = FakeNode(navigating=True)
    client = create_app(node, WEB_DIR).test_client()
    d = client.post('/api/teleop', json={'linear': 0.05, 'angular': 0}).get_json()
    assert d['nav_cancelled'] is True
    names = [c[0] for c in node.calls]
    assert names.index('cancel') < names.index('teleop')


def test_teleop_stop_and_estop(client, node):
    assert client.post('/api/teleop/stop', json={}).get_json()['success']
    assert client.post('/api/estop', json={}).get_json()['success']
    assert ('teleop_stop',) in node.calls and ('estop',) in node.calls


def test_goal_validation(client, node):
    assert client.post('/api/goal', json={'x': 1, 'y': 2}).get_json()['success']
    assert ('goal', 1.0, 2.0, 0.0) in node.calls
    assert client.post('/api/goal', json={'x': 'a', 'y': 2}).status_code == 400
    assert client.post('/api/initialpose', json={'y': 2}).status_code == 400


def test_state_versions_parsed(client, node):
    client.get('/api/state?layers=map,local&map_v=3&global_v=x&local_v=7')
    _, versions, layers = node.calls[-1]
    assert versions == {'map': 3, 'global': -1, 'local': 7}
    assert layers == {'map', 'local'}


def test_save_map_rejects_injection(client, node):
    r = client.post('/api/slam/save_map', json={'name': 'x; rm -rf ~'})
    assert r.status_code == 400
    assert not any(c[0] == 'save' for c in node.calls)


def test_save_map_conflict_then_overwrite(client, node):
    assert client.post('/api/slam/save_map', json={'name': 'exists'}).status_code == 409
    r = client.post('/api/slam/save_map', json={'name': 'exists', 'overwrite': True})
    assert r.status_code == 200
    assert ('save', 'exists', True) in node.calls


def test_map_name_error_type():
    with pytest.raises(MapNameError):
        map_base_path('/x', '../../etc/passwd')
