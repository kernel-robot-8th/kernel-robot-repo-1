"""Loopback shopping HTTP contract; no device or real camera is opened."""
from http.client import HTTPConnection
import json
from types import SimpleNamespace
from threading import Condition

import pytest

from roport_smartcart.shopping_http import serve


@pytest.fixture
def web(tmp_path):
    (tmp_path / 'index.html').write_text('pad')

    class Shop:
        commands = []
        camera_session = 'owned-session-only'

        def status(self, key):
            return {'ownsCamera': key == self.camera_session}

        def image(self, frame, key):
            return b'synthetic-jpeg' if key == self.camera_session else None

        def command(self, action, body):
            self.commands.append((action, body))
            return {'success': True}

    shop = Shop()
    server = serve(shop, tmp_path, 0)
    port = server.server_port
    host = f'127.0.0.1:{port}'

    def request(method, path, headers=None, body=None):
        conn = HTTPConnection('127.0.0.1', port, timeout=2)
        try:
            conn.request(method, path, body=body, headers={'Host': host, **(headers or {})})
            res = conn.getresponse()
            return res.status, res.read(), res.getheader('Content-Type'), res.getheader('Cache-Control')
        finally:
            conn.close()

    try:
        yield shop, request, host
    finally:
        server.shutdown()
        server.server_close()


@pytest.mark.parametrize('path', ['/shopping/status', '/shopping/image', '/shopping/image?frame=frozen'])
def test_get_rejects_wrong_host_and_other_origins(web, path):
    _, request, host = web
    assert request('GET', path, {'Host': 'other.invalid'})[0] == 403
    assert request('GET', path, {'Origin': 'http://other.invalid'})[0] == 403
    assert request('GET', path, {'Sec-Fetch-Site': 'same-site'})[0] == 403
    status, _, _, _ = request('GET', path, {'Origin': 'http://' + host})
    assert status == (200 if path == '/shopping/status' else 404)


@pytest.mark.parametrize('path', ['/shopping/image', '/shopping/image?frame=frozen'])
def test_only_owner_can_read_live_and_frozen_jpeg(web, path):
    shop, request, _ = web
    assert request('GET', path)[0] == 404
    assert request('GET', path, {'X-Shopping-Session': 'someone-else'})[0] == 404
    assert request('GET', path, {'X-Shopping-Session': shop.camera_session}) == (
        200, b'synthetic-jpeg', 'image/jpeg', 'no-store',
    )


def test_post_checks_origin_host_content_type_and_size_before_dispatch(web):
    shop, request, host = web
    good = {'Origin': 'http://' + host, 'Content-Type': 'application/json'}
    uri = '/shopping/stop'
    assert request('POST', uri, {**good, 'Host': 'other.invalid'}, '{}')[0] == 403
    assert request('POST', uri, {**good, 'Origin': 'http://other.invalid'}, '{}')[0] == 403
    assert request('POST', uri, {**good, 'Content-Type': 'text/plain'}, '{}')[0] == 403
    assert request('POST', uri, good, '')[0] == 409
    assert request('POST', uri, good, json.dumps({'extra': 'a' * 2048}))[0] == 409
    assert shop.commands == []
    assert request('POST', uri, good, '{}')[0] == 200
    assert shop.commands == [('stop', {})]


def test_shop_image_checks_owner_and_expiry_before_returning_frozen_frame(monkeypatch):
    pytest.importorskip('cv2')
    pytest.importorskip('rclpy')
    from roport_smartcart import shopping_follow as node

    shop = SimpleNamespace(lock=Condition(), owner='owned-session-only', touch=10.0,
                           frozen=('frame', (9.0, None, None, b'frozen-jpeg')), packet=None)
    monkeypatch.setattr(node.time, 'monotonic', lambda: 11.0)
    assert node.Shopping.image(shop, 'frame', None) is None
    assert node.Shopping.image(shop, 'frame', 'someone-else') is None
    assert node.Shopping.image(shop, 'frame', shop.owner) == b'frozen-jpeg'
    monkeypatch.setattr(node.time, 'monotonic', lambda: 12.0)
    assert node.Shopping.image(shop, 'frame', shop.owner) is None
