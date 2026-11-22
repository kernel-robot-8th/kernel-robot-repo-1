"""웹 화면 API: 허용 키만·다른 사이트 요청 거부·클릭 범위·JSON 만(카메라·YOLO 없음)."""
import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from roport_follow.web import make_handler


class FakeSession:
    def __init__(self):
        self.keys, self.clicks, self.typed = [], [], ''

    def key(self, k):
        self.keys.append(k)
        return f'ok {k}'

    def click(self, u, v):
        self.clicks.append((u, v))
        return (0.3, 0.0)

    def set_typed(self, value):
        self.typed = ''.join(c for c in str(value) if c.isdigit())[:4]
        return self.typed

    def state(self):
        return {'fps': 0.0}

    paused, logs = False, []

    def obstacle_pause(self, paused):
        self.paused = paused
        return paused

    def test_log(self, step, note=''):
        if not isinstance(step, int) or isinstance(step, bool) or not 0 <= step <= 9:
            raise ValueError('step must be an integer 0..9')
        self.logs.append((step, note))
        return '/tmp/x_test1.csv'


    demo, demo_sent = None, []

    def demo_send(self, body):
        from roport_follow.demo_link import demo_command
        msg = demo_command(body)
        self.demo_sent.append(msg)
        return msg


class FakeWorker:
    size = (640, 480)
    seq, jpeg, stop_flag = 0, None, True
    cond = threading.Condition()


@pytest.fixture
def server():
    ses = FakeSession()
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(ses, FakeWorker()))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield ses, f'http://127.0.0.1:{httpd.server_address[1]}'
    httpd.shutdown()
    httpd.server_close()


def _post(base, path, body, headers=None, raw=None):
    data = raw if raw is not None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, method='POST',
                                 headers={'Content-Type': 'application/json', **(headers or {})})
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_allowed_key_reaches_session(server):
    ses, base = server
    assert _post(base, '/api/key', {'key': 'b'}) == (200, {'result': 'ok b'})
    assert _post(base, '/api/key', {'key': 'enter'})[0] == 200
    assert _post(base, '/api/key', {'key': 'r'})[0] == 200
    assert ses.keys == ['b', 'enter', 'r']


@pytest.mark.parametrize('key', ['q', 'rm -rf /', '', None, 5, 'yy', 'rr'])
def test_other_keys_rejected(server, key):
    ses, base = server
    assert _post(base, '/api/key', {'key': key})[0] == 400
    assert ses.keys == []


def test_cross_site_post_rejected(server):
    ses, base = server
    assert _post(base, '/api/key', {'key': 's'}, {'Sec-Fetch-Site': 'cross-site'})[0] == 403
    assert ses.keys == []


def test_non_json_and_big_body_rejected(server):
    ses, base = server
    req = urllib.request.Request(base + '/api/key', data=b'key=s', method='POST',
                                 headers={'Content-Type': 'application/x-www-form-urlencoded'})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req, timeout=3)
    assert e.value.code == 415
    assert _post(base, '/api/typed', None, raw=b'{"value":"' + b'1' * 2000 + b'"}')[0] == 413
    assert _post(base, '/api/key', None, raw=b'[1,2]')[0] == 400


def test_click_inside_image_only(server):
    ses, base = server
    assert _post(base, '/api/click', {'u': 320, 'v': 400}) == (200, {'ground_cm': [30.0, 0.0]})
    for bad in ({'u': -1, 'v': 10}, {'u': 640, 'v': 10}, {'u': 'a', 'v': 1}, {'v': 3}):
        assert _post(base, '/api/click', bad)[0] == 400
    assert ses.clicks == [(320, 400)]


def test_typed_keeps_digits_only(server):
    ses, base = server
    assert _post(base, '/api/typed', {'value': '6x0;rm'}) == (200, {'typed': '60'})


def test_unknown_paths_404(server):
    _, base = server
    assert _post(base, '/api/../etc', {})[0] == 404
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(base + '/../../etc/passwd', timeout=3)
    assert e.value.code == 404


def test_obstacle_pause_needs_boolean(server):
    ses, base = server
    assert _post(base, '/api/obstacles', {'pause': True}) == (200, {'paused': True}) and ses.paused is True
    assert _post(base, '/api/obstacles', {'pause': 'yes'})[0] == 400


def test_test_log_validates_step(server):
    ses, base = server
    assert _post(base, '/api/test_log', {'step': 3, 'note': 'min 18cm'})[0] == 200
    assert ses.logs[-1] == (3, 'min 18cm')
    assert _post(base, '/api/test_log', {'step': '3; rm'})[0] == 400
    assert _post(base, '/api/test_log', {'step': 3}, headers={'Sec-Fetch-Site': 'cross-site'})[0] == 403


def test_demo_command_validated_before_robot(server):
    ses, base = server
    code, body = _post(base, '/api/demo', {'op': 'follow', 'card_id': 35})
    assert code == 200 and body['sent']['card_id'] == 35 and body['sent']['max_duration_s'] == 120
    for bad in ({'op': 'follow', 'card_id': 7}, {'op': 'route', 'route': ['N08; rm', 'N12']},
                {'op': 'route', 'route': ['N08', 'N12'], 'speed_limit_mps': 1.0}, {'op': 'dance'},
                {'op': 'follow', 'card_id': 35, 'max_duration_s': 1000}):
        code, body = _post(base, '/api/demo', bad)
        assert code == 400 and 'error' in body
    code, _ = _post(base, '/api/demo', {'op': 'stop'}, headers={'Sec-Fetch-Site': 'cross-site'})
    assert code == 403
