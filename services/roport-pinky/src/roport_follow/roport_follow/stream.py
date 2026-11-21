"""노트북 쪽 영상 입력: 로봇 SSH MPJPEG, USB 카메라, 동영상·사진 폴더(시험용)."""
from __future__ import annotations

import re
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

IP_RE = re.compile(r'^\d{1,3}(\.\d{1,3}){3}$')
MAX_FRAME = 4 * 1024 * 1024
ROBOT_SCRIPT = Path(__file__).with_name('robot_cam_stream.py')


def iter_mpjpeg(read, max_frame=MAX_FRAME):
    """MPJPEG(경계·머리·Content-Length·JPEG) → (headers dict, jpeg bytes). EOF 면 끝, 형식 오류는 ValueError.

    pinky_test_console/dock_camera.py 의 같은 이름 함수와 같은 방식(머리 값도 돌려주도록 확장).
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
        headers = {}
        for line in head.splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                headers[k.strip().lower()] = v.strip()
        n = headers.get('content-length', '')
        if not n.isdigit():
            raise ValueError('mpjpeg part without Content-Length')
        n = int(n)
        if n > max_frame:
            raise ValueError('frame too large')
        while len(buf) < n:
            chunk = read(65536)
            if not chunk:
                return
            buf += chunk
        yield headers, buf[:n]
        buf = buf[n:]


class Frame:
    __slots__ = ('image', 'robot_time', 'recv_time', 'seq')

    def __init__(self, image, robot_time, recv_time, seq):
        self.image, self.robot_time, self.recv_time, self.seq = image, robot_time, recv_time, seq


class RobotStream:
    """SSH 로 robot_cam_stream.py 를 실행해 최신 프레임만 유지(밀린 프레임은 버림)."""

    def __init__(self, ip, key='~/.ssh/pinky_ed25519', user='pinky', fps=10.0, quality=80,
                 rotate=True, known_hosts='~/.ssh/known_hosts', exposure_us=0, gain=0.0):
        if not IP_RE.match(ip):
            raise ValueError(f'not an IPv4 address: {ip}')
        if not re.fullmatch(r'[a-z_][a-z0-9_-]{0,31}', user):
            raise ValueError('bad ssh user')
        args = ['--fps', f'{float(fps):g}', '--quality', str(int(quality))]
        if not rotate:
            args.append('--no-rotate')
        if exposure_us:
            args += ['--exposure-us', str(int(exposure_us))]
        if gain:
            args += ['--gain', f'{float(gain):g}']
        cmd = ['ssh', '-i', str(Path(key).expanduser()), '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
               '-o', f'UserKnownHostsFile={Path(known_hosts).expanduser()}',
               '-o', 'StrictHostKeyChecking=accept-new', '-o', 'ServerAliveInterval=5',
               f'{user}@{ip}', 'python3 -u - ' + ' '.join(args)]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0)
        self.proc.stdin.write(ROBOT_SCRIPT.read_bytes())
        self.proc.stdin.close()
        self._lock = threading.Lock()
        self._latest = None
        self._seq = 0
        self.error = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        import cv2
        try:
            for headers, jpg in iter_mpjpeg(self.proc.stdout.read):
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                try:
                    rt = float(headers.get('x-robot-time', 'nan'))
                except ValueError:
                    rt = float('nan')
                self._seq += 1
                with self._lock:
                    self._latest = Frame(img, rt, time.time(), self._seq)
        except Exception as e:  # noqa: BLE001 - 상태로 보고
            self.error = str(e)
        err = self.proc.stderr.read().decode('utf-8', 'replace').strip()
        if self.proc.poll() is None:
            self.proc.wait(timeout=2)
        if not self.error:
            self.error = err[-400:] or f'robot stream ended (code {self.proc.returncode})'

    def latest(self):
        with self._lock:
            return self._latest

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class LocalStream:
    """USB 카메라 번호, 동영상 파일, 사진 폴더(시험·재생용). 로봇 시각은 없음(NaN)."""

    def __init__(self, source, fps=10.0):
        import cv2
        self.files = None
        p = Path(str(source)).expanduser()
        if p.is_dir():
            self.files = sorted(f for f in p.iterdir() if f.suffix.lower() in ('.jpg', '.jpeg', '.png'))
            if not self.files:
                raise ValueError(f'no images in {p}')
            self.idx = 0
        else:
            self.cap = cv2.VideoCapture(int(source) if str(source).isdigit() else str(p))
            if not self.cap.isOpened():
                raise ValueError(f'cannot open {source}')
        self.period, self.last, self.seq, self.frame, self.error = 1.0 / fps, 0.0, 0, None, None

    def latest(self):
        import cv2
        now = time.time()
        if self.frame is not None and now - self.last < self.period:
            return self.frame
        if self.files is not None:
            img = cv2.imread(str(self.files[self.idx]))
        else:
            ok, img = self.cap.read()
            if not ok:
                self.error = 'end of video'
                return self.frame
        self.last, self.seq = now, self.seq + 1
        self.frame = Frame(img, float('nan'), now, self.seq)
        return self.frame

    def step(self, delta):
        """사진 폴더에서 앞뒤로 넘김."""
        if self.files is not None:
            self.idx = (self.idx + delta) % len(self.files)
            self.frame = None

    def close(self):
        if self.files is None:
            self.cap.release()
