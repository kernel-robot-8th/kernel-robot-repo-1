"""정밀 도킹 카메라 중계·보정·결과 기록 테스트(실제 ssh·카메라 없이 가짜 프로세스)."""

import io
import json
import os
import queue
import shutil
import subprocess
import threading
import time

import pytest

from pinky_test_console import dock_camera as dc

CAM = {'id': 'omx12', 'label': 'OMX1·2 탑뷰', 'host': 'omen', 'device': '/dev/video4',
       'size': '1280x720', 'node': 'N11', 'calib': 'calib_N11_20261107_184517.json'}
JPEG = b'\xff\xd8' + b'x' * 100 + b'\xff\xd9'
CALIB = {'node': 'N11', 'floor_to_img': [[90.7, -4.4, -13054.9], [6.36, -67.3, 3741.9],
                                         [0.0314, -0.0013, 1.0]], 'source': 'test'}


def mpjpeg(frames):
    out = b''
    for f in frames:
        out += (b'--ffmpeg\r\nContent-type: image/jpeg\r\nContent-length: %d\r\n\r\n' % len(f)
                + f + b'\r\n')
    return out


# ---------------- 카메라 설정 검사 ----------------
def test_validate_camera_ok():
    c = dc.validate_camera(CAM)
    assert (c.host, c.device, c.size, c.node) == ('omen', '/dev/video4', '1280x720', 'N11')


@pytest.mark.parametrize('key,value', [
    ('host', 'omen; rm -rf ~'), ('host', '-oProxyCommand=touch /tmp/x'), ('host', 'a b'),
    ('host', '$(id)'), ('host', ''), ('host', '.hidden'), ('host', 'x' * 65), ('host', None),
    ('device', '/dev/video4; ls'), ('device', '/dev/sda'), ('device', '/dev/video'),
    ('device', '../dev/video0'),
    ('size', '1280x720 -f'), ('size', '12x7'), ('size', '1280*720'),
    ('id', 'Bad ID'), ('id', '../x'), ('node', 'N12'), ('node', 'N07'),
    ('calib', '../calib_N11.json'), ('calib', 'calib_N11.json; ls'), ('label', ''),
])
def test_validate_camera_rejects_injection(key, value):
    with pytest.raises(dc.DockError):
        dc.validate_camera(dict(CAM, **{key: value}))


def test_load_cameras_default_and_file(tmp_path):
    cams, path = dc.load_cameras(str(tmp_path))
    assert path is None and [c.id for c in cams] == ['omx12', 'omx3']
    assert {c.node for c in cams} == {'N11', 'N01'}
    (tmp_path / 'cameras.json').write_text(json.dumps({'cameras': [CAM]}))
    cams, path = dc.load_cameras(str(tmp_path))
    assert path.endswith('cameras.json') and cams[0].id == 'omx12'
    (tmp_path / 'cameras.json').write_text(json.dumps([CAM, CAM]))
    with pytest.raises(dc.DockError):
        dc.load_cameras(str(tmp_path))
    (tmp_path / 'cameras.json').write_text(json.dumps([dict(CAM, host='a;b')]))
    with pytest.raises(dc.DockError):
        dc.load_cameras(str(tmp_path))
    # 깨진 파일: 서버는 기본값으로 뜨고 오류를 화면에 알린다
    cams = dc.DockCameras(str(tmp_path), runner=FakeRunner())
    assert cams.config_error and set(cams.cameras) == {'omx12', 'omx3'}


# ---------------- ssh 명령 ----------------
def test_ssh_commands_are_argument_lists():
    cam = dc.validate_camera(CAM)
    snap = dc.snapshot_command(cam)
    assert snap[:1] == ['ssh']
    i = snap.index('--')
    opts = snap[1:i]
    for o in ('BatchMode=yes', 'StrictHostKeyChecking=yes', 'ConnectTimeout=6'):
        assert o in opts and opts[opts.index(o) - 1] == '-o'
    assert snap[i + 1] == 'omen'
    remote = snap[i + 2:]
    assert remote[:7] == ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'v4l2',
                          '-input_format'] and remote[7] == 'mjpeg'
    assert remote[remote.index('-video_size') + 1] == '1280x720'
    assert remote[remote.index('-i') + 1] == '/dev/video4'
    assert remote[remote.index('-frames:v') + 1] == '1' and remote[-1] == '-'
    stream = dc.stream_command(cam)
    r = stream[stream.index('--') + 2:]
    assert r[r.index('-f', 8) + 1] == 'mpjpeg' and r[-1] == '-'
    # 원격 셸로 가는 토큰에는 셸 특수 문자가 없다
    for tok in remote + r:
        assert not set(tok) & set(';|&$`\'"<>(){}*?!~ \\\n'), tok
    assert not any(t in ('pkill', 'kill', 'fuser', 'killall') for t in snap + stream)
    with pytest.raises(dc.DockError):
        dc.stream_command(cam, fps=60)


def test_classify_failure():
    busy = dc.classify_failure(1, b'[video4linux2] Cannot open video device /dev/video4: '
                                  b'Device or resource busy\n')
    assert isinstance(busy, dc.CameraBusy) and busy.status == 409
    assert dc.classify_failure(255, b'ssh: connect to host omen port 22: No route').status == 502
    assert '호스트 키' in str(dc.classify_failure(255, b'Host key verification failed.'))
    assert dc.classify_failure(124, b'').status == 504
    assert dc.classify_failure(127, b'bash: ffmpeg: command not found').status == 502


# ---------------- mpjpeg ----------------
def test_iter_mpjpeg():
    frames = [JPEG, JPEG + b'y', b'\xff\xd8\r\n\r\n\xff\xd9']
    data = io.BytesIO(mpjpeg(frames))
    assert list(dc.iter_mpjpeg(lambda n: data.read(7))) == frames   # 조각나서 와도
    with pytest.raises(ValueError):
        list(dc.iter_mpjpeg(io.BytesIO(b'--x\r\nfoo: 1\r\n\r\nabc').read))
    with pytest.raises(ValueError):
        list(dc.iter_mpjpeg(io.BytesIO(mpjpeg([JPEG])).read, max_frame=10))


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='ffmpeg not installed')
def test_iter_mpjpeg_real_ffmpeg_format():
    """노트북 ffmpeg 의 실제 mpjpeg 출력 형식(카메라 대신 lavfi 시험 영상, 장치 열지 않음)."""
    out = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
                          'testsrc=size=160x120:rate=5', '-frames:v', '3', '-c:v', 'mjpeg',
                          '-f', 'mpjpeg', '-'], capture_output=True, timeout=30, check=True)
    frames = list(dc.iter_mpjpeg(io.BytesIO(out.stdout).read))
    assert len(frames) == 3 and all(f[:2] == b'\xff\xd8' and f[-2:] == b'\xff\xd9' for f in frames)


# ---------------- 가짜 프로세스 ----------------
class FakeStdout:
    """실시간 ffmpeg 출력 흉내: put() 한 만큼 read 가 돌려주고, close 면 EOF.
    실제 Popen(bufsize=0) stdout(FileIO)처럼 read1 은 없다(2026-11-09 실사용 AttributeError)."""

    def __init__(self):
        self.q = queue.Queue()

    def put(self, data):
        self.q.put(data)

    def read(self, n=-1):
        return self.q.get()


class FakeProc:
    def __init__(self, stderr=b''):
        self.stdout = FakeStdout()
        self.stderr = io.BytesIO(stderr)
        self.code = None
        self.terminated = False

    def poll(self):
        return self.code

    def finish(self, code):
        self.code = code
        self.stdout.put(b'')

    def terminate(self):
        self.terminated = True
        if self.code is None:
            self.finish(-15)

    def kill(self):
        self.terminate()

    def wait(self, timeout=None):
        return self.code


class FakeRunner:
    def __init__(self, run_result=(0, JPEG, b'')):
        self.run_result = run_result
        self.runs, self.popens, self.procs = [], [], []
        self.next_stderr = b''

    def run(self, argv, timeout):
        self.runs.append((argv, timeout))
        return self.run_result

    def popen(self, argv):
        self.popens.append(argv)
        p = FakeProc(self.next_stderr)
        self.procs.append(p)
        return p


def make(tmp_path, runner=None, idle=0.05):
    (tmp_path / 'cameras.json').write_text(json.dumps([CAM]))
    return dc.DockCameras(str(tmp_path), runner=runner or FakeRunner(), idle_sec=idle)


def test_snapshot_one_shot(tmp_path):
    r = FakeRunner()
    cams = make(tmp_path, r)
    assert cams.snapshot('omx12') == JPEG
    assert r.runs[0][0] == dc.snapshot_command(cams.get('omx12'))
    assert r.runs[0][1] <= 15.0
    with pytest.raises(dc.DockError) as e:
        cams.snapshot('nope')
    assert e.value.status == 404


def test_snapshot_busy_is_409(tmp_path):
    r = FakeRunner((1, b'', b'Cannot open video device /dev/video4: Device or resource busy'))
    with pytest.raises(dc.CameraBusy) as e:
        make(tmp_path, r).snapshot('omx12')
    assert e.value.status == 409 and len(r.runs) == 1       # 다시 시도·다른 프로세스 정리 없음


def test_snapshot_not_jpeg(tmp_path):
    with pytest.raises(dc.DockError) as e:
        make(tmp_path, FakeRunner((0, b'garbage', b''))).snapshot('omx12')
    assert e.value.status == 502


def test_stream_shared_and_stopped_when_idle(tmp_path):
    r = FakeRunner()
    cams = make(tmp_path, r)
    feeder = threading.Thread(target=lambda: (time.sleep(0.05),
                                              r.procs[0].stdout.put(mpjpeg([JPEG]))))
    feeder.start()
    g1 = cams.open_stream('omx12')
    assert next(g1).endswith(JPEG + b'\r\n')
    g2 = cams.open_stream('omx12')                    # 두 번째 화면: 같은 ssh 프로세스
    assert len(r.popens) == 1 and next(g2).startswith(b'--' + dc.BOUNDARY.encode())
    st = cams.listing()['cameras'][0]['status']
    assert st['streaming'] and st['viewers'] == 2
    # 실시간 중 사진 = 실시간 장(장치를 다시 열지 않음)
    assert cams.snapshot('omx12') == JPEG and r.runs == []
    g1.close()
    g2.close()
    deadline = time.time() + 2
    while not r.procs[0].terminated and time.time() < deadline:
        time.sleep(0.02)
    assert r.procs[0].terminated                       # 보는 사람이 없으면 이 서버의 ssh 만 끔
    assert cams.listing()['cameras'][0]['status']['viewers'] == 0
    feeder.join()


def test_stream_busy_is_409(tmp_path):
    r = FakeRunner()
    r.next_stderr = b'[video4linux2] ioctl(VIDIOC_STREAMON): Device or resource busy\n'
    cams = make(tmp_path, r)
    threading.Timer(0.05, lambda: r.procs[0].finish(1)).start()
    with pytest.raises(dc.CameraBusy):
        cams.open_stream('omx12')
    st = cams.listing()['cameras'][0]['status']
    assert st['busy'] and st['viewers'] == 0 and '사용 중' in st['error']


def test_close_stops_all(tmp_path):
    r = FakeRunner()
    cams = make(tmp_path, r, idle=60)
    threading.Timer(0.05, lambda: r.procs[0].stdout.put(mpjpeg([JPEG]))).start()
    g = cams.open_stream('omx12')
    next(g)
    cams.close()
    assert r.procs[0].terminated
    with pytest.raises(dc.DockError):
        cams.open_stream('omx12')


# ---------------- 보정 ----------------
def test_calib(tmp_path):
    cams = make(tmp_path)
    with pytest.raises(dc.DockError) as e:
        cams.calib('omx12')
    assert e.value.status == 404
    (tmp_path / CAM['calib']).write_text(json.dumps(CALIB))
    c = cams.calib('omx12')
    assert c['node'] == 'N11' and len(c['floor_to_img']) == 3 and c['file'] == CAM['calib']
    (tmp_path / CAM['calib']).write_text(json.dumps(dict(CALIB, node='N01')))
    with pytest.raises(dc.DockError):
        cams.calib('omx12')
    for bad in ({'node': 'N11', 'floor_to_img': [[1, 2], [3, 4]]},
                {'node': 'N11', 'floor_to_img': [[1, 2, 3], [4, 5, 6], [7, 8, float('nan')]]},
                {'node': 'X', 'floor_to_img': CALIB['floor_to_img']}):
        with pytest.raises(dc.DockError):
            dc.validate_calib(bad)


def test_calib_symlink_outside_dir_ignored(tmp_path):
    outside = tmp_path / 'outside.json'
    outside.write_text(json.dumps(CALIB))
    d = tmp_path / 'dock'
    d.mkdir()
    os.symlink(outside, d / CAM['calib'])
    (d / 'cameras.json').write_text(json.dumps([CAM]))
    with pytest.raises(dc.DockError):
        dc.DockCameras(str(d), runner=FakeRunner()).calib('omx12')


# ---------------- 결과 기록 ----------------
RESULT = {'node': 'N11', 'robot': 'pinky1', 'camera': 'omx12',
          'route': ['N07', 'N23', 'N19', 'N11'],
          'applied': {'advance_cm': 1.5, 'rotate_deg': 96.0, 'move_cm': 2.0},
          'target': {'x_cm': 209.4, 'y_cm': 39.0, 'heading_deg': 6.0},
          'measured': {'x_cm': 209.9, 'y_cm': 38.6, 'heading_deg': 4.0},
          'error': {'along_cm': 0.4, 'left_cm': 0.5, 'dtheta_deg': -2.0}, 'note': '시험'}


def test_result_saved_inside_home(tmp_path):
    res = dc.DockResults(str(tmp_path / 'logs' / 'dock'), home=str(tmp_path))
    out = res.save(RESULT)
    assert os.path.dirname(out['path']) == str(tmp_path / 'logs' / 'dock')
    assert dc.DockResults.NAME.match(out['file'])
    saved = json.load(open(out['path'], encoding='utf-8'))
    assert saved['measured'] == RESULT['measured'] and saved['note'] == '시험'
    assert oct(os.stat(out['path']).st_mode & 0o777) == '0o600'
    assert res.recent()['results'][0]['file'] == out['file']


def test_result_dir_outside_home_refused(tmp_path):
    home = tmp_path / 'home'
    home.mkdir()
    for root in (str(tmp_path / 'elsewhere'), str(home / '..' / 'x'), str(home)):
        res = dc.DockResults(root, home=str(home))
        assert res.error
        with pytest.raises(dc.DockError):
            res.save(RESULT)
    os.symlink(tmp_path / 'elsewhere', home / 'link')     # 홈 안 symlink 로 밖에 쓰기도 막음
    assert dc.DockResults(str(home / 'link' / 'dock'), home=str(home)).error


def test_result_save_errors(tmp_path, monkeypatch):
    res = dc.DockResults(str(tmp_path / 'd'), home=str(tmp_path))

    def exists(src, dst):
        raise FileExistsError(17, 'File exists')
    monkeypatch.setattr(dc.os, 'link', exists)
    with pytest.raises(dc.DockError) as e:
        res.save(RESULT)
    assert e.value.status == 409 and '이미 있음' in str(e.value)

    def full(src, dst):
        raise OSError(28, 'No space left on device')
    monkeypatch.setattr(dc.os, 'link', full)
    with pytest.raises(dc.DockError) as e:
        res.save(RESULT)
    assert e.value.status == 500 and 'No space left' in str(e.value)
    assert os.listdir(tmp_path / 'd') == []                    # 임시 파일도 남지 않음

    (tmp_path / 'blocked').write_text('file, not a folder')
    with pytest.raises(dc.DockError) as e:
        dc.DockResults(str(tmp_path / 'blocked' / 'dock'), home=str(tmp_path)).save(RESULT)
    assert e.value.status == 500


@pytest.mark.parametrize('patch', [
    {'node': '../../etc/passwd'}, {'node': 'N12'}, {'robot': '../x'}, {'camera': 'a/b'},
    {'measured': None}, {'measured': {'x_cm': 1, 'y_cm': 2}},
    {'measured': {'x_cm': float('inf'), 'y_cm': 2, 'heading_deg': 0}},
    {'target': {'x_cm': 1, 'y_cm': 2, 'heading_deg': 0, 'path': '/etc'}},
    {'route': ['N07', '../x']}, {'note': 'x' * 501}, {'file': 'evil.json'},
])
def test_result_rejects(tmp_path, patch):
    data = dict(RESULT, **patch)
    data = {k: v for k, v in data.items() if v is not None}
    with pytest.raises(dc.DockError):
        dc.DockResults(str(tmp_path / 'd'), home=str(tmp_path)).save(data)
    assert not (tmp_path / 'd').exists() or not os.listdir(tmp_path / 'd')


def test_stream_reader_works_with_unbuffered_popen():
    """Popen(bufsize=0) 의 stdout 은 FileIO(read1 없음) — chunk_reader 로 실제 mpjpeg 프레임을 읽는다."""
    import shutil
    if shutil.which('ffmpeg') is None:
        pytest.skip('ffmpeg 없음')
    proc = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
                             'testsrc=size=160x120:rate=10', '-frames:v', '3', '-c:v', 'mjpeg',
                             '-f', 'mpjpeg', '-'], stdin=subprocess.DEVNULL,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    try:
        assert not hasattr(proc.stdout, 'read1')
        frames = list(dc.iter_mpjpeg(dc.chunk_reader(proc.stdout)))
    finally:
        proc.wait(timeout=10)
    assert len(frames) == 3 and all(f[:2] == b'\xff\xd8' for f in frames)
