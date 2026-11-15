"""Flask 라우트 테스트 (ROS·로봇 없이 가짜 노드로)."""

import importlib.util
import os

import pytest

from pinky_test_console.ir_record import validate_filename, validate_label
from pinky_test_console.map_store import map_base_path, MapNameError
from pinky_test_console.teleop import TeleopGuard

# 모듈 단위 importorskip 은 이 환경에서 디렉터리 전체 수집을 멈추게 해서 skipif 로 처리
pytestmark = pytest.mark.skipif(importlib.util.find_spec('flask') is None,
                                reason='python3-flask not installed')


def create_app(node, web_dir, **kwargs):
    from pinky_test_console.web_app import create_app as _create_app
    return _create_app(node, web_dir, **kwargs)


WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


class FakeNode:
    def __init__(self, navigating=False):
        self.calls = []
        self.guard = TeleopGuard()
        self.navigating = navigating
        self.ir_dir = '/nonexistent-ir'
        self.jpeg = None
        self.records_path = None

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

    def ir_snapshot(self):
        return {'latest': [1, 2, 3]}

    def ir_start(self, label, duration):
        validate_label(label)  # 실제 노드와 같은 검사
        self.calls.append(('ir_start', label, duration))
        return {'label': label}

    def ir_cancel(self):
        return True

    def ir_records(self):
        return {'ir_dir': self.ir_dir, 'records': [], 'suggestion': None}

    def ir_record_path(self, name):
        return os.path.join(self.ir_dir, validate_filename(name))

    def marker_snapshot(self):
        return {'fps': 9.5, 'detections': [], 'checklist': [], 'error': None}

    def marker_jpeg(self):
        return self.jpeg

    def marker_reset(self):
        self.calls.append(('marker_reset',))
        return True

    def marker_record_start(self, distance_cm, duration, marker_id=None, note=''):
        from pinky_test_console.marker_detect import MarkerError
        bad_id = marker_id is not None and not isinstance(marker_id, int)
        if not 1 <= distance_cm <= 500 or bad_id:
            raise MarkerError('bad')
        self.calls.append(('marker_record', distance_cm, duration, marker_id, note))
        return {'distance_cm': distance_cm}

    def marker_records(self):
        return {'path': self.records_path, 'records': []}

    def marker_records_path(self):
        return self.records_path

    def route_snapshot(self):
        return {'mode': 'route', 'state': {'state': 'idle'}, 'stale': False}

    def route_graph(self):
        return {'version': 'line-v13', 'nodes': [], 'presets': []}

    def route_command(self, cmd):
        self.calls.append(('route', cmd))
        return True

    def route_return(self, goal=None, start=None):
        self.calls.append(('return', goal, start))
        return {'success': True, 'msg': 'P5(N16)로 복귀', 'route': ['N14', 'N16']}

    def ros_log_snapshot(self, after=0):
        rows = [{'seq': 1, 'level': 'INFO', 'node': 'line_nav', 'msg': 'hi'}]
        return {'entries': [r for r in rows if r['seq'] > after], 'last': 1}

    def log_files(self):
        return {'route_logs': [], 'ros_logs': []}

    # AMCL
    NODES = [{'id': 'N12', 'x_cm': 378.3, 'y_cm': 73.2, 'yaw_deg': 0.0, 'nav2_switch': True}]
    amcl_path = None

    def amcl_snapshot(self, map_version):
        self.calls.append(('amcl', map_version))
        return {'map': None, 'score': None, 'nodes': self.NODES}

    def amcl_node(self, node_id):
        from pinky_test_console.amcl_check import AmclCheckError
        for n in self.NODES:
            if n['id'] == node_id:
                return n
        raise AmclCheckError('unknown node')

    def amcl_initial_pose_cm(self, x, y, yaw):
        self.calls.append(('amcl_init', x, y, yaw))
        return True

    def amcl_nomotion_update(self):
        self.calls.append(('nomotion',))
        return True

    def amcl_record(self, label, truth):
        from pinky_test_console.amcl_check import validate_label
        validate_label(label)
        self.calls.append(('amcl_record', label, truth))
        return {'label': label}

    def amcl_records_path(self):
        return self.amcl_path


@pytest.fixture
def node():
    return FakeNode()


@pytest.fixture
def client(node):
    app = create_app(node, WEB_DIR)
    app.testing = True
    return app.test_client()


def test_index_and_assets(client):
    for path in ('/', '/app.js', '/style.css', '/index.html', '/ir.html', '/ir.js',
                 '/marker.html', '/marker.js', '/amcl.html', '/amcl.js', '/route.html',
                 '/route.js', '/routemap.js', '/fleet.html', '/fleet.js', '/settings.html',
                 '/settings.js', '/test.html', '/test.js', '/dock.html', '/dock.js',
                 '/dock_core.js'):
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


@pytest.mark.parametrize('page', ['index.html', 'ir.html', 'marker.html', 'amcl.html',
                                  'route.html', 'fleet.html', 'settings.html', 'test.html',
                                  'dock.html'])
def test_html_has_no_external_or_inline_code(page):
    html = open(os.path.join(WEB_DIR, page)).read()
    assert 'http://' not in html.split('-->', 1)[1]
    assert 'https://' not in html.split('-->', 1)[1]
    assert '<script>' not in html and 'style=' not in html and 'onclick' not in html


def test_ir_api(client, node, tmp_path):
    assert client.get('/api/ir').get_json() == {'latest': [1, 2, 3]}
    r = client.post('/api/ir/record', json={'label': 'carpet', 'duration': 10})
    assert r.status_code == 200 and node.calls[-1] == ('ir_start', 'carpet', 10.0)
    for body in ({'label': 'a b', 'duration': 5}, {'label': 'x', 'duration': 'ten'},
                 {'label': 'x'}):
        assert client.post('/api/ir/record', json=body).status_code == 400, body
    assert client.post('/api/ir/record', data='label=x',
                       content_type='application/x-www-form-urlencoded').status_code == 400
    assert client.post('/api/ir/cancel', json={}).get_json() == {'success': True}
    assert client.get('/api/ir/records').get_json()['records'] == []


def test_ir_record_download(client, node, tmp_path):
    node.ir_dir = str(tmp_path)
    (tmp_path / '20261028_120000_carpet.csv').write_text('t_sec,ir0,ir1,ir2\n')
    (tmp_path / 'secret.txt').write_text('no')
    r = client.get('/api/ir/records/20261028_120000_carpet.csv')
    assert r.status_code == 200 and r.data.startswith(b't_sec')
    assert 'attachment' in r.headers['Content-Disposition']
    for name in ('secret.txt', '..%2Fsecret.txt', '20261028_120000_missing.csv'):
        assert client.get('/api/ir/records/' + name).status_code == 404, name


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


def test_marker_api(client, node):
    assert client.get('/api/marker').get_json()['fps'] == 9.5
    r = client.get('/api/marker/frame.jpg')
    assert r.status_code == 204          # 아직 영상 없음
    node.jpeg = b'\xff\xd8fake'
    r = client.get('/api/marker/frame.jpg')
    assert r.status_code == 200 and r.mimetype == 'image/jpeg' and r.data == node.jpeg
    assert r.headers['Cache-Control'] == 'no-store'
    assert client.post('/api/marker/reset', data='x').status_code == 400
    assert client.post('/api/marker/reset', json={}).get_json() == {'success': True}
    assert node.calls[-1] == ('marker_reset',)


def test_marker_record_api(client, node, tmp_path):
    r = client.post('/api/marker/record', json={'distance_cm': 58, 'marker_id': 3, 'note': 'P5'})
    assert r.status_code == 200 and node.calls[-1] == ('marker_record', 58.0, 5.0, 3, 'P5')
    for body in ({'distance_cm': 0}, {'distance_cm': 'x'}, {'distance_cm': 30, 'marker_id': 'a'}):
        assert client.post('/api/marker/record', json=body).status_code == 400, body
    assert client.get('/api/marker/records').get_json()['records'] == []
    assert client.get('/api/marker/records.csv').status_code == 404
    node.records_path = str(tmp_path / 'range_log.csv')
    (tmp_path / 'range_log.csv').write_text('time,distance_cm\n')
    r = client.get('/api/marker/records.csv')
    assert r.status_code == 200 and 'attachment' in r.headers['Content-Disposition']


def test_amcl_snapshot_passes_map_version(client, node):
    assert client.get('/api/amcl?map_v=3').get_json()['nodes'][0]['id'] == 'N12'
    assert node.calls[-1] == ('amcl', 3)
    client.get('/api/amcl?map_v=x')
    assert node.calls[-1] == ('amcl', -1)


def test_amcl_initialpose_by_node_and_cm(client, node):
    assert client.post('/api/amcl/initialpose', json={'node': 'N12'}).status_code == 200
    assert node.calls[-1] == ('amcl_init', 378.3, 73.2, 0.0)
    client.post('/api/amcl/initialpose', json={'node': 'N12', 'yaw_deg': 180})
    assert node.calls[-1] == ('amcl_init', 378.3, 73.2, 180.0)
    client.post('/api/amcl/initialpose', json={'x_cm': 170, 'y_cm': 150.5, 'yaw_deg': -90})
    assert node.calls[-1] == ('amcl_init', 170.0, 150.5, -90.0)
    n = len(node.calls)
    for body in ({'node': 'N99'}, {'x_cm': 1e6, 'y_cm': 0}, {'x_cm': 'a', 'y_cm': 0},
                 {'x_cm': 1, 'y_cm': 1, 'yaw_deg': 720}, {'y_cm': 1}):
        assert client.post('/api/amcl/initialpose', json=body).status_code == 400, body
    assert len(node.calls) == n   # 잘못된 입력은 초기 위치를 보내지 않음


def test_amcl_nomotion_and_record(client, node, tmp_path):
    assert client.post('/api/amcl/nomotion', json={}).get_json() == {'success': True}
    truth = {'x_cm': 378.3, 'y_cm': 73.2}
    r = client.post('/api/amcl/record', json={'label': 'N12', 'truth': truth})
    assert r.status_code == 200 and node.calls[-1] == ('amcl_record', 'N12', (378.3, 73.2, None))
    client.post('/api/amcl/record', json={'label': 'G1'})
    assert node.calls[-1] == ('amcl_record', 'G1', None)
    for body in ({'label': 'a b'}, {'label': 'x', 'truth': [1, 2]},
                 {'label': 'x', 'truth': {'x_cm': 1}}):
        assert client.post('/api/amcl/record', json=body).status_code == 400, body
    assert client.get('/api/amcl/records.csv').status_code == 404
    node.amcl_path = str(tmp_path / 'amcl_20261103.csv')
    (tmp_path / 'amcl_20261103.csv').write_text('time,label\n')
    r = client.get('/api/amcl/records.csv')
    assert r.status_code == 200 and 'attachment' in r.headers['Content-Disposition']


def test_route_api(client, node):
    assert client.get('/api/route').get_json()['state']['state'] == 'idle'
    assert client.get('/api/route/graph').get_json()['version'] == 'line-v13'
    r = client.post('/api/route/command', json={'cmd': 'start', 'route': ['N14', 'N16']})
    assert r.status_code == 200 and node.calls[-1] == ('route', {'cmd': 'start',
                                                                 'route': ['N14', 'N16']})
    for body in ({'cmd': 'fly'}, {'cmd': 'start', 'route': ['N14']},
                 {'cmd': 'start', 'route': ['N14', 'x; rm']}):
        assert client.post('/api/route/command', json=body).status_code == 400, body
    assert client.post('/api/route/command', data='cmd=stop',
                       content_type='application/x-www-form-urlencoded').status_code == 400


def test_fleet_routes_absent_without_fleet(client):
    assert client.get('/api/fleet').status_code == 404
    assert client.post('/api/fleet/job', json={}).status_code == 404


@pytest.fixture
def fleet_client(node, tmp_path):
    from test_fleet import FakeClient, FakeRunner, GOOD
    from pinky_test_console.fleet import Commands, FleetManager, FleetStore
    store = FleetStore(str(tmp_path / 'fleet.json'))
    store.upsert(GOOD)
    mgr = FleetManager(store, Commands('/repo', '/k'), client=FakeClient(), runner=FakeRunner())
    app = create_app(node, WEB_DIR, fleet=mgr, allowed_hosts={'127.0.0.1:8080'})
    app.testing = True
    return app.test_client(), mgr


def test_fleet_host_check(fleet_client):
    c, _ = fleet_client
    assert c.get('/api/fleet', headers={'Host': 'evil.example'}).status_code == 403
    assert c.get('/api/fleet', headers={'Host': '127.0.0.1:8080'}).status_code == 200


def test_fleet_api(fleet_client):
    c, mgr = fleet_client
    h = {'Host': '127.0.0.1:8080'}
    snap = c.get('/api/fleet', headers=h).get_json()
    assert snap['robots'][0]['name'] == 'pinky1'
    r = c.post('/api/fleet/robots', headers=h, json={'name': 'pinky2', 'ip': '192.168.4.22',
                                                     'role': 'smartcart', 'domain': 16,
                                                     'calib': 'pinky_764e'})
    assert r.status_code == 200
    assert c.post('/api/fleet/robots', headers=h, json={'name': 'X'}).status_code == 400
    r = c.post('/api/fleet/job', headers=h, json={'name': 'pinky2', 'action': 'set_domain'})
    assert r.status_code == 200 and r.get_json()['job']['action'] == 'set_domain'
    assert c.post('/api/fleet/job', headers=h,
                  json={'name': 'pinky1', 'action': 'deploy', 'ref': 'main'}).status_code == 400
    r = c.post('/api/fleet/route', headers=h, json={'name': 'pinky1', 'cmd': 'stop'})
    assert r.status_code == 200 and r.get_json()['result'] == {'success': True}
    assert c.post('/api/fleet/route', headers=h,
                  json={'name': 'pinky1', 'cmd': 'start', 'route': ['N1']}).status_code == 400
    assert c.post('/api/fleet/stop_all', headers=h, json={}).get_json()['success']
    mgr.client.fail = True
    r = c.post('/api/fleet/route', headers=h, json={'name': 'pinky1', 'cmd': 'stop'})
    assert r.status_code == 502
    assert c.post('/api/fleet/robots/delete', headers=h,
                  json={'name': 'pinky2'}).status_code == 200


def test_route_plan_api(client, node):
    from pinky_test_console.route_view import graph_view
    layout = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                          'roport_airport.yaml')
    node.route_graph = lambda: graph_view(layout)
    r = client.get('/api/route/plan?from=N16&to=N01')
    assert r.status_code == 200 and r.get_json()['route'][0] == 'N16'
    assert client.get('/api/route/plan?from=N16&to=N14').status_code == 400
    assert client.get('/api/route/plan?from=N16').status_code == 400
    r = client.get('/api/route/plan?from=N16&to=N11&via=N01')
    assert r.status_code == 200 and r.get_json()['route'][5] == 'N01'
    assert client.get('/api/route/plan?from=N16&to=N11&via=N19').status_code == 400


def test_fleet_logs_api(fleet_client):
    c, mgr = fleet_client
    h = {'Host': '127.0.0.1:8080'}
    mgr.log.add('pinky1', 'fleet', 'command', '{"cmd": "stop"}')
    data = c.get('/api/fleet/logs?after=0', headers=h).get_json()
    assert data['last'] >= 1 and data['entries'][-1]['event'] == 'command'
    assert c.get('/api/fleet/logs?after=x', headers=h).status_code == 400
    r = c.get('/api/fleet/logs.csv?robot=pinky1', headers=h)
    assert r.status_code == 200 and 'attachment' in r.headers['Content-Disposition']
    assert r.data.decode().startswith('seq,time,robot')


def test_robot_log_api(client):
    assert client.get('/api/logs?after=0').get_json()['entries'][0]['node'] == 'line_nav'
    assert client.get('/api/logs?after=1').get_json()['entries'] == []
    assert client.get('/api/logs?after=x').status_code == 400
    assert client.get('/api/logs/files').get_json() == {'route_logs': [], 'ros_logs': []}


def test_fleet_files_proxy(fleet_client):
    c, mgr = fleet_client
    h = {'Host': '127.0.0.1:8080'}
    r = c.get('/api/fleet/files?name=pinky1', headers=h)
    assert r.status_code == 200 and r.get_json()['success']
    assert c.get('/api/fleet/files?name=nobody', headers=h).status_code == 400


def test_fleet_index_and_frame_headers(fleet_client):
    c, _ = fleet_client
    h = {'Host': '127.0.0.1:8080'}
    r = c.get('/', headers=h)
    assert r.status_code == 302 and r.headers['Location'].endswith('test.html')
    r = c.get('/test.html', headers=h)
    assert r.headers['X-Frame-Options'] == 'SAMEORIGIN'      # 노트북 탭 iframe 허용
    assert "frame-ancestors 'self'" in r.headers['Content-Security-Policy']


def test_robot_proxy(fleet_client):
    c, mgr = fleet_client
    h = {'Host': '127.0.0.1:8080'}
    calls = []

    def forward(ip, sub, query, method, body=None, content_type=None):
        calls.append((ip, sub, query, method, body, content_type))
        if sub.startswith('api/'):
            return 200, 'application/json', None, b'{"ok": true}'
        return 200, 'text/html; charset=utf-8', None, b'<html></html>'

    mgr.proxy.forward = forward
    assert c.get('/robot/pinky1', headers=h).status_code in (301, 302, 308)
    r = c.get('/robot/pinky1/route.html', headers=h)
    assert r.status_code == 200 and r.headers['X-Frame-Options'] == 'SAMEORIGIN'
    r = c.get('/robot/pinky1/api/ir?after=3', headers=h)
    assert r.get_json() == {'ok': True} and r.headers['Cache-Control'] == 'no-store'
    assert calls[-1][:4] == ('192.168.4.21', 'api/ir', 'after=3', 'GET')
    r = c.post('/robot/pinky1/api/route/command', headers=h, json={'cmd': 'stop'})
    assert r.status_code == 200 and calls[-1][3:] == ('POST', b'{"cmd": "stop"}',
                                                      'application/json')
    n = len(calls)
    # 로봇 화면 파일·api/ 밖, 폼 POST, 미등록 로봇은 로봇까지 가지 않음
    assert c.get('/robot/pinky1/fleet.html', headers=h).status_code == 404
    assert c.get('/robot/pinky1/api/../setup.py', headers=h).status_code == 404
    assert c.post('/robot/pinky1/api/route/command', headers=h,
                  data='cmd=stop').status_code == 400
    assert c.get('/robot/nobody/route.html', headers=h).status_code == 400
    assert c.get('/robot/pinky1/route.html', headers={'Host': 'evil'}).status_code == 403
    assert len(calls) == n

    def down(*a, **k):
        raise OSError('refused')
    mgr.proxy.forward = down
    assert c.get('/robot/pinky1/api/route', headers=h).status_code == 502


def test_robot_proxy_absent_without_fleet(client):
    assert client.get('/robot/pinky1/route.html').status_code == 404


def test_fleet_return_api(fleet_client, node):
    from pinky_test_console.route_view import graph_view
    c, mgr = fleet_client
    h = {'Host': '127.0.0.1:8080'}
    layout = os.path.join(os.path.dirname(__file__), '..', '..', 'roport_gz_world', 'layout',
                          'roport_airport.yaml')
    node.route_graph = lambda: graph_view(layout)
    r = c.post('/api/fleet/return', headers=h, json={})
    assert r.status_code == 200 and not r.get_json()['result']['pinky1']['ok']
    assert c.post('/api/fleet/return', headers=h, json={'name': 3}).status_code == 400
    assert c.post('/api/fleet/return', headers=h, json={'name': 'nobody'}).status_code == 400
    assert c.post('/api/fleet/return', headers=h, data='x').status_code == 400


def test_route_return_endpoint():
    node = FakeNode()
    client = create_app(node, WEB_DIR).test_client()
    res = client.post('/api/route/return', json={'start': 'N14'})
    assert res.status_code == 200 and res.get_json()['success']
    assert node.calls[-1] == ('return', None, 'N14')
    assert client.post('/api/route/return', json={}).status_code == 200
    assert node.calls[-1] == ('return', None, None)
    assert client.post('/api/route/return', json={'start': 'x;y'}).status_code == 400
    assert client.post('/api/route/return', json={'goal': 5}).status_code == 400


# ---------------- 정밀 도킹 (mode:=fleet) ----------------
@pytest.fixture
def dock_client(node, tmp_path):
    import json as _json
    from test_dock_camera import CALIB, CAM, FakeRunner
    from test_fleet import FakeClient, GOOD
    from test_fleet import FakeRunner as FleetRunner
    from pinky_test_console.dock_camera import DockCameras, DockResults
    from pinky_test_console.fleet import Commands, FleetManager, FleetStore
    store = FleetStore(str(tmp_path / 'fleet.json'))
    store.upsert(GOOD)
    mgr = FleetManager(store, Commands('/repo', '/k'), client=FakeClient(), runner=FleetRunner())
    dock_dir = tmp_path / 'roport_dock'
    dock_dir.mkdir()
    (dock_dir / 'cameras.json').write_text(_json.dumps([CAM]))
    (dock_dir / CAM['calib']).write_text(_json.dumps(CALIB))
    runner = FakeRunner()
    cams = DockCameras(str(dock_dir), runner=runner, idle_sec=0.05)
    results = DockResults(str(tmp_path / 'roport_fleet_logs' / 'dock'), home=str(tmp_path))
    app = create_app(node, WEB_DIR, fleet=mgr, allowed_hosts={'127.0.0.1:8080'},
                     dock=(cams, results))
    app.testing = True
    return app.test_client(), mgr, runner, tmp_path


H = {'Host': '127.0.0.1:8080'}


def test_dock_routes_absent_without_fleet(client):
    for path in ('/api/dock/cameras', '/api/dock/camera/omx12/snapshot.jpg',
                 '/api/dock/calib/omx12', '/api/dock/results'):
        assert client.get(path).status_code == 404, path
    assert client.post('/api/dock/result', json={}).status_code == 404


def test_dock_cameras_snapshot_calib(dock_client):
    c, _, runner, _ = dock_client
    d = c.get('/api/dock/cameras', headers=H).get_json()
    assert [x['id'] for x in d['cameras']] == ['omx12'] and d['cameras'][0]['calib_found']
    r = c.get('/api/dock/camera/omx12/snapshot.jpg', headers=H)
    assert r.status_code == 200 and r.mimetype == 'image/jpeg' and r.data[:2] == b'\xff\xd8'
    assert r.headers['Cache-Control'] == 'no-store'
    assert c.get('/api/dock/camera/nope/snapshot.jpg', headers=H).status_code == 404
    assert c.get('/api/dock/camera/omx12/snapshot.jpg',
                 headers={'Host': 'evil.example'}).status_code == 403
    runner.run_result = (1, b'', b'/dev/video4: Device or resource busy')
    r = c.get('/api/dock/camera/omx12/snapshot.jpg', headers=H)
    assert r.status_code == 409 and '사용 중' in r.get_json()['msg']
    cal = c.get('/api/dock/calib/omx12', headers=H).get_json()
    assert cal['node'] == 'N11' and len(cal['floor_to_img']) == 3


@pytest.mark.parametrize('site,code', [
    ('cross-site', 403), ('same-site', 403), ('same-origin', 200), ('none', 200), (None, 200)])
def test_dock_camera_sec_fetch_site(dock_client, site, code):
    """다른 사이트의 <img> 가 ssh·카메라를 열지 못함. 헤더 없는 클라이언트(curl)는 허용."""
    c, _, runner, _ = dock_client
    headers = dict(H, **({'Sec-Fetch-Site': site} if site else {}))
    r = c.get('/api/dock/camera/omx12/snapshot.jpg', headers=headers)
    assert r.status_code == code
    assert len(runner.runs) == (1 if code == 200 else 0)      # 403 이면 ssh 를 실행하지 않음
    if code == 403:
        assert c.get('/api/dock/camera/omx12/stream', headers=headers).status_code == 403
        assert runner.popens == []


def test_dock_result_save_failure_is_clear_error(dock_client, monkeypatch):
    from test_dock_camera import RESULT
    c, _, _, _ = dock_client

    def exists(src, dst):
        raise FileExistsError(17, 'File exists')
    monkeypatch.setattr('pinky_test_console.dock_camera.os.link', exists)
    r = c.post('/api/dock/result', headers=H, json=RESULT)
    assert r.status_code == 409 and '이미 있음' in r.get_json()['msg']


def test_dock_stream_busy_409(dock_client):
    import threading
    c, _, runner, _ = dock_client
    runner.next_stderr = b'Device or resource busy'
    threading.Timer(0.05, lambda: runner.procs[0].finish(1)).start()
    r = c.get('/api/dock/camera/omx12/stream', headers=H)
    assert r.status_code == 409


def test_dock_stream_mjpeg(dock_client):
    import threading
    from test_dock_camera import JPEG, mpjpeg
    c, _, runner, _ = dock_client
    threading.Timer(0.05, lambda: runner.procs[0].stdout.put(mpjpeg([JPEG, JPEG]))).start()
    r = c.get('/api/dock/camera/omx12/stream', headers=H, buffered=False)
    assert r.status_code == 200 and r.mimetype == 'multipart/x-mixed-replace'
    first = next(r.response)
    assert first.startswith(b'--dockframe\r\nContent-Type: image/jpeg') and JPEG in first
    r.close()


def test_dock_result_api(dock_client):
    from test_dock_camera import RESULT
    c, _, _, tmp = dock_client
    r = c.post('/api/dock/result', headers=H, json=RESULT)
    assert r.status_code == 200, r.get_json()
    name = r.get_json()['file']
    assert (tmp / 'roport_fleet_logs' / 'dock' / name).is_file()
    assert c.get('/api/dock/results', headers=H).get_json()['results'][0]['file'] == name
    assert c.post('/api/dock/result', headers=H,
                  json=dict(RESULT, node='../../x')).status_code == 400
    assert c.post('/api/dock/result', headers=H, data='x').status_code == 400


def test_dock_command_via_fleet_route(dock_client):
    c, mgr, _, _ = dock_client
    body = {'name': 'pinky1', 'cmd': 'dock', 'node': 'N11', 'advance_cm': 1.5,
            'rotate_deg': 96.0, 'move_cm': 2.0}
    r = c.post('/api/fleet/route', headers=H, json=body)
    assert r.status_code == 200
    assert mgr.client.calls[-1][1:] == ('/api/route/command', {
        'cmd': 'dock', 'node': 'N11', 'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': 2.0})
    assert c.post('/api/fleet/route', headers=H,
                  json=dict(body, advance_cm=7)).status_code == 400
    r = c.post('/api/fleet/route', headers=H, json={
        'name': 'pinky1', 'cmd': 'start', 'route': ['N07', 'N23', 'N19', 'N11'],
        'start_cross': True, 'start_facing': 'S'})
    assert r.status_code == 200
    assert mgr.client.calls[-1][2]['start_facing'] == 'S'


def test_robot_console_route_command_accepts_dock(client, node):
    r = client.post('/api/route/command', json={'cmd': 'dock', 'node': 'N01', 'reset': True})
    assert r.status_code == 200 and node.calls[-1] == ('route', {'cmd': 'dock', 'node': 'N01',
                                                                 'reset': True})
    bad = {'cmd': 'dock', 'node': 'N12', 'reset': True}
    assert client.post('/api/route/command', json=bad).status_code == 400
