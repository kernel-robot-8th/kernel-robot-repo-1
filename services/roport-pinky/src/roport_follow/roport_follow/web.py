"""0단계 웹 화면: 브라우저에서 실시간 영상(겹치기 포함)·체스보드 검출 상태·버튼(키와 같은 동작).

  $PY -m roport_follow.tool web --robot <로봇 IP>      → http://127.0.0.1:8090/
기본은 노트북 안(127.0.0.1)에서만 연다. --bind 0.0.0.0 은 같은 망의 누구나 버튼을 누를 수 있어 권하지 않는다.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

STATIC = Path(__file__).with_name('web_static')
ALLOWED_KEYS = set('0123456789yabcfhxwsrv') | {'enter', 'backspace'}
MAX_BODY = 1024


class Worker(threading.Thread):
    """세션을 돌려 최신 겹친 영상을 JPEG 로 들고 있는다."""

    def __init__(self, ses, quality=80):
        super().__init__(daemon=True)
        self.ses, self.quality = ses, int(quality)
        self.cond = threading.Condition()
        self.jpeg, self.seq, self.size = None, 0, (640, 480)
        self.stop_flag = False

    def run(self):
        import cv2
        while not self.stop_flag:
            try:
                img = self.ses.step()
            except Exception as e:  # noqa: BLE001 - 화면에 알리고 계속
                self.ses.note(f'error: {e}', f'처리 오류: {e}')
                img = None
            if img is None:
                time.sleep(0.03)
                continue
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                with self.cond:
                    self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                    self.size = (img.shape[1], img.shape[0])
                    self.cond.notify_all()
            time.sleep(0.01)


def make_handler(ses, worker):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'roport-follow'

        def log_message(self, fmt, *args):   # 접속 기록은 터미널에 안 남김
            pass

        def _send(self, code, body, ctype='application/json; charset=utf-8'):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/index.html'):
                self._send(200, (STATIC / 'index.html').read_bytes(), 'text/html; charset=utf-8')
            elif path == '/api/state':
                if getattr(ses, 'demo', None) is not None:
                    ses.demo.browser_seen()          # 브라우저가 보고 있을 때만 로봇에 살아 있음 신호
                st = ses.state()
                st['image_size'] = list(worker.size)
                self._send(200, st)
            elif path == '/map.png' and getattr(ses, 'loc_view', None) is not None:
                self._send(200, ses.loc_view.png, 'image/png')
            elif path == '/stream.mjpg':
                self._stream()
            else:
                self._send(404, {'error': 'not found'})

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = -1
            try:
                while True:
                    with worker.cond:
                        worker.cond.wait_for(lambda: worker.seq != last or worker.stop_flag, timeout=2.0)
                        if worker.stop_flag:
                            return
                        if worker.seq == last or worker.jpeg is None:
                            continue
                        jpg, last = worker.jpeg, worker.seq
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpg))
                    self.wfile.write(jpg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                return

        def do_POST(self):
            # 다른 사이트에서 보낸 요청(CSRF) 막기: 같은 출처이거나 헤더 없음(curl)만, JSON 만
            if self.headers.get('Sec-Fetch-Site', 'same-origin') not in ('same-origin', 'none'):
                return self._send(403, {'error': 'cross-site request'})
            if not self.headers.get('Content-Type', '').startswith('application/json'):
                return self._send(415, {'error': 'json only'})
            try:
                n = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                n = -1
            if not 0 <= n <= MAX_BODY:
                return self._send(413, {'error': 'body too large'})
            try:
                body = json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                return self._send(400, {'error': 'bad json'})
            if not isinstance(body, dict):
                return self._send(400, {'error': 'object expected'})
            path = self.path.split('?', 1)[0]
            if path == '/api/key':
                k = body.get('key')
                if k not in ALLOWED_KEYS:
                    return self._send(400, {'error': 'key not allowed'})
                return self._send(200, {'result': ses.key(k)})
            if path == '/api/click':
                try:
                    u, v = int(body['u']), int(body['v'])
                except (KeyError, TypeError, ValueError):
                    return self._send(400, {'error': 'u, v integers required'})
                w, h = worker.size
                if not (0 <= u < w and 0 <= v < h):
                    return self._send(400, {'error': 'outside image'})
                g = ses.click(u, v)
                return self._send(200, {'ground_cm': None if g is None else [round(g[0] * 100, 1), round(g[1] * 100, 1)]})
            if path == '/api/obstacles':          # 시험 1: 장애물 보내기 일시 멈춤/다시
                if not isinstance(body.get('pause'), bool):
                    return self._send(400, {'error': 'pause must be true or false'})
                try:
                    return self._send(200, {'paused': ses.obstacle_pause(body['pause'])})
                except ValueError as e:
                    return self._send(409, {'error': str(e)})
            if path == '/api/test_log':           # 시험 1: 단계 기록 CSV
                try:
                    return self._send(200, {'file': ses.test_log(body.get('step'), body.get('note', ''))})
                except ValueError as e:
                    return self._send(400, {'error': str(e)})
            if path == '/api/demo':               # 시험용: 버튼 → 로봇 demo_runner(관제 대신)
                try:
                    return self._send(200, {'sent': ses.demo_send(body)})
                except ValueError as e:
                    return self._send(400, {'error': str(e)})
            if path == '/api/boarding_alert':     # 패드의 탑승 시간 알림 → 로봇 표시(주행 명령 아님)
                boarding = getattr(ses, 'boarding', None)
                if boarding is None:
                    return self._send(404, {'error': 'boarding alert off'})
                try:
                    return self._send(200, {'on': boarding.set(body.get('on'))})
                except ValueError as e:
                    return self._send(400, {'error': str(e)})
            if path == '/api/typed':
                return self._send(200, {'typed': ses.set_typed(body.get('value', ''))})
            return self._send(404, {'error': 'not found'})

    return Handler


def serve(ses, src, bind='127.0.0.1', port=8090):
    worker = Worker(ses)
    worker.start()
    httpd = ThreadingHTTPServer((bind, port), make_handler(ses, worker))
    httpd.daemon_threads = True
    print(f'웹 화면: http://{"127.0.0.1" if bind in ("0.0.0.0", "") else bind}:{port}/  (Ctrl+C 로 끝)')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop_flag = True
        ses.close()
        with worker.cond:
            worker.cond.notify_all()
        httpd.server_close()
        src.close()
