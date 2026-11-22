"""Serve the built pad and its cart-specific follow API on one loopback origin."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from threading import Thread
from urllib.parse import parse_qs, urlsplit


def serve(shop, dist, port):
    if not (dist/'index.html').is_file():
        raise ValueError('build @roport/pad and supply its dist directory')

    class Handler(SimpleHTTPRequestHandler):
        def reply(self, status, data, kind='application/json'):
            raw = json.dumps(data, allow_nan=False).encode() if kind=='application/json' else data
            self.send_response(status)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def loopback_host(self):
            host = self.headers.get('Host', '')
            if host in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'):
                return host
            return None

        def do_GET(self):
            host = self.loopback_host()
            url = urlsplit(self.path)
            origin = self.headers.get('Origin')
            if (not host or (url.path.startswith('/shopping/') and (
                    (origin is not None and origin != 'http://' + host)
                    or self.headers.get('Sec-Fetch-Site') in ('cross-site', 'same-site')))):
                self.reply(403, {})
                return
            session = self.headers.get('X-Shopping-Session')
            if url.path == '/shopping/status':
                self.reply(200, shop.status(session))
            elif url.path == '/shopping/image':
                frame = parse_qs(url.query).get('frame', [None])[0]
                jpeg = shop.image(frame, session)
                self.reply(200, jpeg, 'image/jpeg') if jpeg else self.reply(404, {})
            else:
                super().do_GET()

        def do_POST(self):
            host = self.loopback_host()
            if (not host or self.headers.get('Origin') != 'http://' + host
                    or self.headers.get_content_type() != 'application/json'):
                self.reply(403, {})
                return
            if not self.path.startswith('/shopping/'):
                self.reply(404, {})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048:
                    raise ValueError('잘못된 요청입니다')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError('잘못된 요청입니다')
                self.reply(200, shop.command(self.path[10:], body))
            except (ValueError, TypeError, UnicodeError) as exc:
                self.reply(409, {'error': str(exc)})

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', port), partial(Handler, directory=str(dist)))
    Thread(target=server.serve_forever, daemon=True).start()
    return server
