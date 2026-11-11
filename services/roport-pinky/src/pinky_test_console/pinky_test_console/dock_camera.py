"""정밀 도킹 탭(mode:=fleet, 노트북)의 탑뷰 카메라 중계·보정·결과 기록. Flask·ROS 없이 테스트한다.

카메라는 OMX 팀 PC(omen, gpu = team16-pc)에 꽂혀 있고 OMX 프로그램과 같이 쓴다. 노트북이 SSH 로 그 PC 의
ffmpeg 를 실행해 MJPEG 를 받는다(계약 01_dock_contract.md 5절).
  - SSH 는 ~/.ssh/config 별칭 + 키(BatchMode, 비밀번호 없음), StrictHostKeyChecking=yes(처음 한 번은
    사용자가 직접 접속해 호스트 키를 등록).
  - 원격 명령은 셸 문자열을 만들지 않고 검사한 토큰(host·device·size 정규식)과 고정 문자열만 넘긴다.
  - 장치가 사용 중이면(Device or resource busy) 409 「사용 중」. 다른 프로세스는 절대 끄지 않는다.
    이 서버가 띄운 ssh 프로세스만 끈다(보는 사람이 없으면 몇 초 뒤, 서버 종료 때 모두).
  - 같은 카메라의 실시간·사진 요청은 ssh ffmpeg 하나를 같이 쓴다(장치를 두 번 열지 않음).
"""

import datetime
import json
import math
import os
import re
import subprocess
import tempfile
import threading
import time
from dataclasses import asdict, dataclass

from .route_view import DOCK_NODES, NODE_ID

DEFAULT_DOCK_DIR = '~/roport_dock'
CAMERAS_FILE = 'cameras.json'
CAMERA_ID = re.compile(r'^[a-z0-9][a-z0-9_-]{0,31}$')
HOST = re.compile(r'^[A-Za-z0-9._-]+$')         # ~/.ssh/config 별칭
DEVICE = re.compile(r'^/dev/video[0-9]+$')
SIZE = re.compile(r'^[0-9]{3,4}x[0-9]{3,4}$')
CALIB_NAME = re.compile(r'^calib_[A-Za-z0-9_]{1,60}\.json$')
ROBOT_NAME = re.compile(r'^[a-z][a-z0-9_-]{0,15}$')   # fleet.NAME 과 같게
LABEL_MAX = 60
SSH_OPTIONS = ('-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=6',
               '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2')
FFMPEG_INPUT = ('ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'v4l2',
                '-input_format', 'mjpeg')
STREAM_FPS = 5
SNAPSHOT_TIMEOUT = 15.0        # s, 사진 1장(ssh 접속 + 장치 열기 포함)
FIRST_FRAME_TIMEOUT = 12.0     # s, 실시간 첫 장
IDLE_STOP_SEC = 3.0            # 보는 사람이 없어진 뒤 ssh ffmpeg 를 끄기까지
MAX_VIEWERS = 4                # 카메라당 동시 실시간 연결
MAX_FRAME = 4 * 1024 * 1024    # JPEG 1장 상한
FRESH_SEC = 2.0                # 실시간 중이면 이보다 새 장을 사진으로 돌려줌
BOUNDARY = 'dockframe'
POSE_LIMIT_CM = 1000.0
RESULT_KEEP = 50

# 계약 5절 기본값(cameras.json 이 없을 때). host 는 사용자 ~/.ssh/config 별칭
DEFAULT_CAMERAS = (
    {'id': 'omx12', 'label': 'OMX1·2 탑뷰', 'host': 'omen', 'device': '/dev/video4',
     'size': '1280x720', 'node': 'N11', 'calib': 'calib_N11_20261107_184517.json'},
    {'id': 'omx3', 'label': 'OMX3 탑뷰', 'host': 'gpu', 'device': '/dev/video0',
     'size': '1280x720', 'node': 'N01', 'calib': 'calib_N01_20261107_190226.json'},
)


class DockError(ValueError):
    """화면에 보여 줄 오류. status = HTTP 상태 코드."""

    status = 400

    def __init__(self, msg, status=None):
        super().__init__(msg)
        if status is not None:
            self.status = status


class CameraBusy(DockError):
    status = 409


@dataclass(frozen=True)
class Camera:
    id: str
    label: str
    host: str
    device: str
    size: str
    node: str
    calib: str = ''


def validate_camera(data):
    if not isinstance(data, dict):
        raise DockError('camera must be an object')
    cid, host, device, size = (data.get(k) for k in ('id', 'host', 'device', 'size'))
    if not isinstance(cid, str) or not CAMERA_ID.match(cid):
        raise DockError('camera id: lowercase letters/digits/-/_ (max 32)')
    # 앞이 '-' 이면 ssh 가 옵션으로 읽는다(-oProxyCommand=…): 정규식에 더해 막는다
    if not isinstance(host, str) or not HOST.match(host) or host[0] in '-.' or len(host) > 64:
        raise DockError(f'{cid}: host must be an ~/.ssh/config alias (letters, digits, . _ -)')
    if not isinstance(device, str) or not DEVICE.match(device):
        raise DockError(f'{cid}: device must look like /dev/video0')
    if not isinstance(size, str) or not SIZE.match(size):
        raise DockError(f'{cid}: size must look like 1280x720')
    node = data.get('node')
    if node not in DOCK_NODES:
        raise DockError(f"{cid}: node must be one of {', '.join(DOCK_NODES)}")
    label = data.get('label', cid)
    if not isinstance(label, str) or not label.strip() or len(label) > LABEL_MAX:
        raise DockError(f'{cid}: label must be 1..{LABEL_MAX} characters')
    calib = data.get('calib', '') or ''
    if not isinstance(calib, str) or (calib and not CALIB_NAME.match(calib)):
        raise DockError(f'{cid}: calib must be a file name like calib_N11_20261107_184517.json')
    return Camera(cid, label.strip(), host, device, size, node, calib)


def load_cameras(dock_dir):
    """dock_dir/cameras.json([...] 또는 {"cameras": [...]}) → [Camera]. 없으면 기본 2대."""
    path = os.path.join(dock_dir, CAMERAS_FILE)
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
    except FileNotFoundError:
        return [validate_camera(c) for c in DEFAULT_CAMERAS], None
    except (OSError, ValueError) as e:
        raise DockError(f'{path} unreadable: {e}') from None
    items = data.get('cameras') if isinstance(data, dict) else data
    if not isinstance(items, list) or not items:
        raise DockError(f'{path}: needs a list of cameras')
    cams = [validate_camera(c) for c in items]
    if len({c.id for c in cams}) != len(cams):
        raise DockError(f'{path}: camera ids must be unique')
    return cams, path


def ssh_argv(cam, remote):
    """ssh 인자 목록. remote = 검사한 토큰 목록(ssh 가 공백으로 이어 원격 셸에 넘긴다)."""
    return ['ssh', *SSH_OPTIONS, '--', cam.host, *remote]


def snapshot_command(cam):
    return ssh_argv(cam, [*FFMPEG_INPUT, '-video_size', cam.size, '-i', cam.device,
                          '-frames:v', '1', '-c:v', 'mjpeg', '-q:v', '3',
                          '-f', 'image2pipe', '-'])


def stream_command(cam, fps=STREAM_FPS):
    if isinstance(fps, bool) or not isinstance(fps, int) or not 1 <= fps <= 15:
        raise DockError('fps must be 1..15')
    return ssh_argv(cam, [*FFMPEG_INPUT, '-video_size', cam.size, '-i', cam.device, '-an',
                          '-vf', f'fps={fps}', '-c:v', 'mjpeg', '-q:v', '5',
                          '-f', 'mpjpeg', '-'])


def classify_failure(code, stderr):
    """ssh/ffmpeg 실패 → DockError(409 사용 중 / 502 연결·장치 / 504 시간 초과)."""
    text = (stderr or b'').decode('utf-8', 'replace') if isinstance(stderr, bytes) else str(stderr)
    tail = ' '.join(text.strip().splitlines()[-2:])[:300]
    if 'Device or resource busy' in text:
        return CameraBusy('사용 중 — 다른 프로그램(OMX 등)이 카메라를 쓰고 있음. 그 프로그램은 끄지 않음')
    if code == 124:
        return DockError('시간 초과(카메라 PC 응답 없음)', 504)
    if 'Host key verification failed' in text:
        return DockError('호스트 키 미등록: 터미널에서 한 번 ssh <별칭> 으로 접속해 키를 확인·등록', 502)
    if code == 255:
        return DockError(f'카메라 PC SSH 연결 실패: {tail or "exit 255"}', 502)
    if code == 127 or 'command not found' in text:
        return DockError('카메라 PC 에 ffmpeg 가 없음', 502)
    return DockError(f'카메라 읽기 실패(code {code}): {tail}', 502)


def chunk_reader(stream):
    """있는 만큼 돌려주는 read(n). Popen(bufsize=0) 의 stdout 은 FileIO 라 read1 이 없고 read(n) 이 그 역할."""
    return getattr(stream, 'read1', None) or stream.read


def iter_mpjpeg(read, max_frame=MAX_FRAME):
    """ffmpeg -f mpjpeg 출력(--경계, 헤더, Content-length, JPEG)에서 JPEG 를 하나씩 낸다.

    read(n) = 있는 만큼 돌려주는 함수(chunk_reader: FileIO.read 또는 BufferedReader.read1). EOF 면 끝. 형식이 틀리면 ValueError.
    """
    buf = b''
    while True:
        i = buf.find(b'\r\n\r\n')
        while i < 0:
            if len(buf) > 8192:
                raise ValueError('mpjpeg header too long')
            chunk = read(65536)
            if not chunk:
                return
            buf += chunk
            i = buf.find(b'\r\n\r\n')
        head, buf = buf[:i].decode('latin-1'), buf[i + 4:]
        m = re.search(r'(?im)^content-length:\s*(\d{1,9})\s*$', head)
        if not m:
            raise ValueError('mpjpeg part without Content-length')
        n = int(m.group(1))
        if n > max_frame:
            raise ValueError('frame too large')
        while len(buf) < n:
            chunk = read(65536)
            if not chunk:
                return
            buf += chunk
        yield buf[:n]
        buf = buf[n:]


def multipart_part(jpeg):
    return (f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n'
            .encode('ascii') + jpeg + b'\r\n')


class ProcRunner:
    """subprocess(셸 없음). 테스트에서는 가짜 객체로 바꾼다."""

    def run(self, argv, timeout):
        """→ (code, stdout, stderr). 시간 초과면 code 124."""
        proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
        try:
            out, err = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, err = proc.communicate()
            return 124, b'', err
        return proc.returncode, out, err

    def popen(self, argv):
        return subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, bufsize=0)


def _stop_proc(proc):
    """이 서버가 띄운 ssh 프로세스만 끈다. ssh 가 끊기면 원격 ffmpeg 는 쓰기 실패로 끝난다."""
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)
    except OSError:
        pass


class _Relay:
    """카메라 1대의 ssh ffmpeg(mpjpeg) 1개를 여러 화면이 같이 본다."""

    def __init__(self, cam, runner, idle_sec):
        self.cam = cam
        self.runner = runner
        self.idle_sec = idle_sec
        self.cond = threading.Condition()
        self.proc = None
        self.frame = None
        self.frame_t = 0.0
        self.seq = 0
        self.viewers = 0
        self.error = None          # 마지막 실패(DockError)
        self.started = None
        self._gen = 0              # 프로세스 세대(늦게 끝난 reader 가 새 상태를 덮지 않게)
        self.closed = False

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def acquire(self):
        with self.cond:
            if self.closed:
                raise DockError('server is shutting down', 503)
            if self.viewers >= MAX_VIEWERS:
                raise DockError(f'too many viewers (max {MAX_VIEWERS})', 429)
            self.viewers += 1
            if not self.running():
                self._start_locked()

    def release(self):
        with self.cond:
            self.viewers = max(0, self.viewers - 1)
            if self.viewers:
                return
            gen = self._gen
        t = threading.Timer(self.idle_sec, self._stop_if_idle, args=(gen,))
        t.daemon = True
        t.start()

    def _stop_if_idle(self, gen):
        with self.cond:
            if self.viewers or gen != self._gen:
                return
            proc, self.proc = self.proc, None
        _stop_proc(proc)

    def _start_locked(self):
        self._gen += 1
        self.error = None
        self.frame = None
        self.started = time.time()
        try:
            proc = self.runner.popen(stream_command(self.cam))
        except OSError as e:
            self.error = DockError(f'ssh not runnable: {e}', 502)
            self.cond.notify_all()
            return
        self.proc = proc
        gen = self._gen
        err_buf = []
        threading.Thread(target=self._drain, args=(proc, err_buf), daemon=True).start()
        threading.Thread(target=self._read, args=(proc, gen, err_buf), daemon=True).start()

    @staticmethod
    def _drain(proc, err_buf):
        try:
            for line in iter(proc.stderr.readline, b''):
                err_buf.append(line)
                del err_buf[:-20]
        except (OSError, ValueError):
            pass

    def _read(self, proc, gen, err_buf):
        bad = None
        try:
            for jpeg in iter_mpjpeg(chunk_reader(proc.stdout)):
                with self.cond:
                    if gen != self._gen:
                        break
                    self.frame, self.frame_t = jpeg, time.time()
                    self.seq += 1
                    self.cond.notify_all()
        except (OSError, ValueError) as e:
            bad = e
        _stop_proc(proc)
        try:
            code = proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            code = None
        time.sleep(0.05)    # stderr 마지막 줄
        with self.cond:
            if gen != self._gen:
                return
            if bad is not None:
                self.error = DockError(f'stream format error: {bad}', 502)
            elif code not in (0, -15, None) or self.frame is None:
                self.error = classify_failure(code, b''.join(err_buf))
            self.proc = None
            self.cond.notify_all()

    def wait_frame(self, after_seq, timeout):
        """after_seq 보다 새 장 → (seq, jpeg). 프로세스가 실패로 끝나면 DockError."""
        deadline = time.monotonic() + timeout
        with self.cond:
            while True:
                if self.frame is not None and self.seq > after_seq:
                    return self.seq, self.frame
                if self.proc is None and self.error is not None:
                    raise self.error
                if self.proc is None or self.closed:
                    raise DockError('stream ended', 502)
                left = deadline - time.monotonic()
                if left <= 0:
                    raise DockError('no frame from camera (timeout)', 504)
                self.cond.wait(left)

    def status(self):
        with self.cond:
            return {'streaming': self.running(), 'viewers': self.viewers,
                    'frame_age_sec': None if self.frame is None
                    else round(time.time() - self.frame_t, 1),
                    'error': None if self.error is None else str(self.error),
                    'busy': isinstance(self.error, CameraBusy)}

    def close(self):
        with self.cond:
            self.closed = True
            self._gen += 1
            proc, self.proc = self.proc, None
            self.cond.notify_all()
        _stop_proc(proc)


class DockCameras:
    """카메라 목록(dock_dir/cameras.json)·실시간 중계·사진·보정 파일."""

    def __init__(self, dock_dir=DEFAULT_DOCK_DIR, runner=None, idle_sec=IDLE_STOP_SEC):
        self.dock_dir = os.path.abspath(os.path.expanduser(dock_dir))
        self.runner = runner or ProcRunner()
        self.config_error = None
        self.config_file = None
        try:
            cams, self.config_file = load_cameras(self.dock_dir)
        except DockError as e:     # 화면에 보여 주고 기본값으로
            self.config_error = str(e)
            cams = [validate_camera(c) for c in DEFAULT_CAMERAS]
        self.cameras = {c.id: c for c in cams}
        self._relays = {c.id: _Relay(c, self.runner, idle_sec) for c in cams}
        self._snap_locks = {c.id: threading.Lock() for c in cams}

    def get(self, cam_id):
        cam = self.cameras.get(cam_id) if isinstance(cam_id, str) else None
        if cam is None:
            raise DockError(f'unknown camera {cam_id!r}', 404)
        return cam

    def listing(self):
        return {'dock_dir': self.dock_dir, 'config_file': self.config_file,
                'config_error': self.config_error,
                'cameras': [dict(asdict(c), status=self._relays[c.id].status(),
                                 calib_found=self._calib_path(c) is not None)
                            for c in self.cameras.values()]}

    # ---------------- 사진 ----------------
    def snapshot(self, cam_id):
        """JPEG 1장. 실시간 중이면 그 장(장치를 두 번 열지 않음), 아니면 ssh ffmpeg 1장."""
        cam = self.get(cam_id)
        relay = self._relays[cam.id]
        if relay.running():
            relay.acquire()
            try:
                with relay.cond:
                    fresh = relay.frame is not None and time.time() - relay.frame_t < FRESH_SEC
                    seq = relay.seq - 1 if fresh else relay.seq
                return relay.wait_frame(seq, SNAPSHOT_TIMEOUT)[1]
            finally:
                relay.release()
        lock = self._snap_locks[cam.id]
        if not lock.acquire(timeout=SNAPSHOT_TIMEOUT):
            raise CameraBusy('사진 요청이 이미 진행 중')
        try:
            try:
                code, out, err = self.runner.run(snapshot_command(cam), SNAPSHOT_TIMEOUT)
            except OSError as e:
                raise DockError(f'ssh not runnable: {e}', 502) from None
        finally:
            lock.release()
        if code != 0 or not out.startswith(b'\xff\xd8'):
            raise classify_failure(code if code != 0 else 1, err or b'no JPEG data')
        if len(out) > MAX_FRAME:
            raise DockError('frame too large', 502)
        return out

    # ---------------- 실시간 ----------------
    def open_stream(self, cam_id):
        """첫 장을 기다렸다가 multipart 조각 생성기를 돌려준다. 실패는 여기서 DockError."""
        cam = self.get(cam_id)
        relay = self._relays[cam.id]
        relay.acquire()
        try:
            seq, first = relay.wait_frame(0 if relay.frame is None else relay.seq - 1,
                                          FIRST_FRAME_TIMEOUT)
        except BaseException:
            relay.release()
            raise

        def gen():
            nonlocal seq
            try:
                yield multipart_part(first)
                while True:
                    seq, jpeg = relay.wait_frame(seq, FIRST_FRAME_TIMEOUT)
                    yield multipart_part(jpeg)
            except DockError:
                return
            finally:            # 화면을 닫으면(연결 끊김) 여기로 온다
                relay.release()
        return gen()

    # ---------------- 보정 ----------------
    def _calib_path(self, cam):
        if not cam.calib:
            return None
        path = os.path.realpath(os.path.join(self.dock_dir, cam.calib))
        if os.path.dirname(path) != os.path.realpath(self.dock_dir) or not os.path.isfile(path):
            return None
        return path

    def calib(self, cam_id):
        cam = self.get(cam_id)
        path = self._calib_path(cam)
        if path is None:
            raise DockError(f'calibration file not found in {self.dock_dir}: {cam.calib or "-"} '
                            '(화면에서 보정 JSON 을 직접 불러와도 됨)', 404)
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            raise DockError(f'{cam.calib} unreadable: {e}', 500) from None
        out = validate_calib(data)
        if out['node'] != cam.node:
            raise DockError(f"{cam.calib}: node {out['node']} != camera node {cam.node}", 500)
        out['file'] = cam.calib
        return out

    def close(self):
        for r in self._relays.values():
            r.close()


def validate_calib(data):
    """{node, floor_to_img 3x3, source?} → 검사한 dict."""
    if not isinstance(data, dict):
        raise DockError('calibration must be an object')
    node = data.get('node')
    if node not in DOCK_NODES:
        raise DockError(f"calibration node must be one of {', '.join(DOCK_NODES)}")
    m = data.get('floor_to_img')
    ok = isinstance(m, list) and len(m) == 3 and all(
        isinstance(r, list) and len(r) == 3 and all(_finite(v) for v in r) for r in m)
    if not ok:
        raise DockError('floor_to_img must be a 3x3 list of numbers')
    src = data.get('source', '')
    out = {'node': node, 'floor_to_img': [[float(v) for v in r] for r in m],
           'source': str(src)[:300] if isinstance(src, str) else ''}
    if _finite(data.get('px_per_cm')):
        out['px_per_cm'] = float(data['px_per_cm'])
    return out


def _finite(v):
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)


# ---------------- 결과 기록 ----------------
POSE_KEYS = ('x_cm', 'y_cm', 'heading_deg')
PLAN_KEYS = ('advance_cm', 'rotate_deg', 'move_cm')
ERROR_KEYS = ('along_cm', 'left_cm', 'dtheta_deg')
RESULT_FIELDS = {'node', 'robot', 'camera', 'route', 'applied', 'target', 'predicted', 'measured',
                 'error', 'state', 'note', 'calib_source'}


def _obj(data, key, keys, limit):
    v = data.get(key)
    if v is None:
        return None
    if not isinstance(v, dict) or set(v) - set(keys):
        raise DockError(f'{key} must be an object with {", ".join(keys)}')
    out = {}
    for k in keys:
        x = v.get(k)
        if not _finite(x) or abs(x) > limit:
            raise DockError(f'{key}.{k} must be a finite number within ±{limit:g}')
        out[k] = round(float(x), 3)
    return out


def validate_result(data):
    if not isinstance(data, dict):
        raise DockError('JSON object required')
    extra = set(data) - RESULT_FIELDS
    if extra:
        raise DockError(f'unknown fields: {", ".join(sorted(extra))}')
    node = data.get('node')
    if node not in DOCK_NODES:
        raise DockError(f"node must be one of {', '.join(DOCK_NODES)}")
    out = {'node': node}
    for key, pat in (('robot', ROBOT_NAME), ('camera', CAMERA_ID)):
        v = data.get(key)
        if v is not None:
            if not isinstance(v, str) or not pat.match(v):
                raise DockError(f'invalid {key}')
            out[key] = v
    route = data.get('route')
    if route is not None:
        if not isinstance(route, list) or len(route) > 64 or not all(
                isinstance(r, str) and NODE_ID.match(r) for r in route):
            raise DockError('route must be a list of node ids')
        out['route'] = list(route)
    out['applied'] = _obj(data, 'applied', PLAN_KEYS, 360.0)
    for key in ('target', 'predicted', 'measured'):
        out[key] = _obj(data, key, POSE_KEYS, POSE_LIMIT_CM)
    if out['measured'] is None or out['target'] is None:
        raise DockError('target and measured poses are required')
    out['error'] = _obj(data, 'error', ERROR_KEYS, 360.0)
    for key, n in (('state', 32), ('note', 500), ('calib_source', 300)):
        v = data.get(key)
        if v is not None:
            if not isinstance(v, str) or len(v) > n:
                raise DockError(f'{key} must be text up to {n} characters')
            out[key] = v
    return out


class DockResults:
    """결과 JSON 을 홈 안의 폴더(기본 ~/roport_fleet_logs/dock)에 1건 1파일로. 파일 이름은 서버가 정한다."""

    NAME = re.compile(r'^dock_(N[0-9]{2})_\d{8}_\d{6}_\d{6}\.json$')

    def __init__(self, root, home=None):
        home = os.path.realpath(os.path.expanduser(home or '~'))
        self.root = os.path.realpath(os.path.abspath(os.path.expanduser(root)))
        self.error = None
        if os.path.commonpath([home, self.root]) != home or self.root == home:
            self.error = f'result folder must be inside {home}: {self.root}'

    def save(self, data, now=None):
        rec = validate_result(data)
        if self.error:
            raise DockError(self.error, 500)
        now = now or datetime.datetime.now()
        rec = {'saved_at': now.isoformat(timespec='seconds'), **rec}
        name = f"dock_{rec['node']}_{now.strftime('%Y%m%d_%H%M%S_%f')}.json"
        path = os.path.join(self.root, name)
        try:
            os.makedirs(self.root, mode=0o700, exist_ok=True)
            fd, tmp = tempfile.mkstemp(dir=self.root, prefix='.dock-')
        except OSError as e:
            raise DockError(f'결과 저장 실패({self.root}): {e.strerror or e}', 500) from None
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                json.dump(rec, f, ensure_ascii=False, indent=1)
            os.chmod(tmp, 0o600)
            os.link(tmp, path)        # 같은 이름이 있으면 실패(덮어쓰지 않음)
        except FileExistsError:
            raise DockError(f'같은 이름의 결과 파일이 이미 있음({name}) — 다시 「결과 기록」',
                            409) from None
        except OSError as e:
            raise DockError(f'결과 저장 실패({name}): {e.strerror or e}', 500) from None
        finally:
            try:
                os.unlink(tmp)
            except OSError:
                pass
        return {'file': name, 'path': path, 'record': rec}

    def recent(self, limit=RESULT_KEEP):
        try:
            names = sorted((n for n in os.listdir(self.root) if self.NAME.match(n)), reverse=True)
        except OSError:
            names = []
        out = []
        for n in names[:limit]:
            try:
                with open(os.path.join(self.root, n), encoding='utf-8') as f:
                    rec = json.load(f)
            except (OSError, ValueError):
                continue
            if isinstance(rec, dict):
                out.append({'file': n, **{k: rec.get(k) for k in
                                          ('saved_at', 'node', 'robot', 'applied', 'error',
                                           'note')}})
        return {'dir': self.root, 'error': self.error, 'results': out}
