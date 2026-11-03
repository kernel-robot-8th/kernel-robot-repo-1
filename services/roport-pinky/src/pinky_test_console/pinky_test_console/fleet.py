"""노트북용 Pinky 설정·테스트 관제(mode:=fleet). Flask·ROS 없이 테스트할 수 있게 순수 Python.

관제 서버(roport-control-server) Pinky Settings 화면과 같은 기능을 먼저 시험하는 도구다. MQTT 는 쓰지 않는다.
  - 등록: 이름(pinky1~), IP, 역할(물류·스마트카트), ROS_DOMAIN_ID, IR 기준값 이름(pinky_d660 등)
  - 설치: 노트북 저장소의 브랜치(업데이트) 또는 origin/main(릴리즈)을 git archive → ssh → 로봇 ~/pinky_test_ws/src
  - 빌드: 로봇에서 colcon build (패키지 고정 목록)
  - ROS_DOMAIN_ID: 로봇 ~/.bashrc 에 표시(# roport-domain)가 붙은 export 한 줄을 넣는다(이전 표시 줄은 지움)
  - 상태·경로 명령: 로봇 테스트 콘솔(mode:=route, http://<IP>:8080)의 /api/route 를 HTTP 로 부른다
보안: SSH 는 키 파일(BatchMode, 비밀번호 없음)만, 명령은 인자 목록으로 실행(셸 문자열에 사용자 입력을
그대로 넣지 않음), IP·이름·브랜치·도메인은 검사한다. 이 서버는 기본 127.0.0.1 에만 연다(launch_plan).
"""

import ipaddress
import json
import os
import re
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections import deque
from dataclasses import asdict, dataclass

from .route_view import parse_route_command

NAME = re.compile(r'^[a-z][a-z0-9_-]{0,15}$')
CALIB = re.compile(r'^pinky_[0-9a-z]{2,8}$')
REF = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._/-]{0,99}$')
ROLES = ('logistics', 'smartcart')
DOMAIN_MAX = 101                      # ROS 2 기본 도메인 범위 0~101
PACKAGES = ('pinky_test_console', 'roport_pinky_line', 'roport_line_nav', 'roport_gz_world')
SRC_PREFIX = 'services/roport-pinky/src'
REMOTE_WS = '~/pinky_test_ws'
DOMAIN_MARK = '# roport-domain'
RELEASE_REF = 'origin/main'
ROBOT_PATHS = ('/api/route', '/api/status', '/api/route/command', '/api/estop')
MAX_RESPONSE = 256 * 1024
TIMEOUTS = {'check': 15, 'read_domain': 15, 'set_domain': 15, 'deploy': 180, 'build': 900}
ACTIONS = tuple(TIMEOUTS)


class FleetError(ValueError):
    pass


@dataclass(frozen=True)
class Robot:
    name: str
    ip: str
    role: str
    domain: int
    calib: str


def validate_robot(data):
    if not isinstance(data, dict):
        raise FleetError('robot must be an object')
    name = data.get('name')
    if not isinstance(name, str) or not NAME.match(name):
        raise FleetError('name: lowercase letters/digits/-/_ (max 16), e.g. pinky1')
    try:
        ip = str(ipaddress.IPv4Address(str(data.get('ip', '')).strip()))
    except ValueError:
        raise FleetError('ip must be an IPv4 address') from None
    role = data.get('role')
    if role not in ROLES:
        raise FleetError(f"role must be one of {', '.join(ROLES)}")
    domain = data.get('domain')
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= DOMAIN_MAX:
        raise FleetError(f'domain must be an integer 0..{DOMAIN_MAX}')
    calib = data.get('calib')
    if not isinstance(calib, str) or not CALIB.match(calib):
        raise FleetError('calib must look like pinky_d660 (IR calibration name)')
    return Robot(name, ip, role, domain, calib)


class FleetStore:
    """등록 목록 JSON 파일(기본 ~/.roport/fleet.json). 비밀값은 저장하지 않는다."""

    def __init__(self, path):
        self.path = os.path.abspath(os.path.expanduser(path))
        self._lock = threading.Lock()
        self._robots = {}
        self._load()

    def _load(self):
        try:
            with open(self.path, encoding='utf-8') as f:
                data = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, json.JSONDecodeError) as e:
            raise FleetError(f'fleet file unreadable: {self.path}: {e}') from None
        for item in data.get('robots', []) if isinstance(data, dict) else []:
            r = validate_robot(item)
            self._robots[r.name] = r

    def _save(self):
        os.makedirs(os.path.dirname(self.path), mode=0o700, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path), prefix='.fleet-')
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                json.dump({'robots': [asdict(r) for r in self.list()]}, f, ensure_ascii=False,
                          indent=2)
            os.chmod(tmp, 0o600)
            os.replace(tmp, self.path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def list(self):
        return sorted(self._robots.values(), key=lambda r: r.name)

    def get(self, name):
        r = self._robots.get(name)
        if r is None:
            raise FleetError(f'unknown robot {name!r}')
        return r

    def upsert(self, data):
        r = validate_robot(data)
        with self._lock:
            for other in self._robots.values():
                if other.ip == r.ip and other.name != r.name:
                    raise FleetError(f'ip {r.ip} is already registered as {other.name}')
            self._robots[r.name] = r
            self._save()
        return r

    def remove(self, name):
        with self._lock:
            if self._robots.pop(name, None) is None:
                raise FleetError(f'unknown robot {name!r}')
            self._save()


class Commands:
    """실행할 명령의 인자 목록(셸 없음). 원격 명령 문자열에는 검사한 정수·고정 문자열만 넣는다."""

    def __init__(self, repo, ssh_key, user='pinky'):
        self.repo = os.path.abspath(os.path.expanduser(repo))
        self.ssh_key = os.path.abspath(os.path.expanduser(ssh_key))
        if not re.match(r'^[a-z_][a-z0-9_-]{0,31}$', user):
            raise FleetError('invalid ssh user')
        self.user = user

    def ssh(self, ip, remote):
        ipaddress.IPv4Address(ip)  # 검사된 값만 들어온다는 것을 한 번 더 확인
        return ['ssh', '-i', self.ssh_key, '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                '-o', 'StrictHostKeyChecking=accept-new', '-o', 'ConnectTimeout=5',
                f'{self.user}@{ip}', remote]

    def check(self, ip):
        return self.ssh(ip, 'echo ok; hostname')

    def read_domain(self, ip):
        return self.ssh(ip, "grep -n 'ROS_DOMAIN_ID' ~/.bashrc || true")

    def set_domain(self, ip, domain):
        if isinstance(domain, bool) or not isinstance(domain, int) \
                or not 0 <= domain <= DOMAIN_MAX:
            raise FleetError(f'domain must be an integer 0..{DOMAIN_MAX}')
        return self.ssh(ip, (
            f"sed -i '/{DOMAIN_MARK}$/d' ~/.bashrc && "
            f"echo 'export ROS_DOMAIN_ID={domain}  {DOMAIN_MARK}' >> ~/.bashrc && "
            "grep -n 'ROS_DOMAIN_ID' ~/.bashrc"))

    def fetch(self):
        return ['git', '-C', self.repo, 'fetch', '--quiet', '--prune', 'origin']

    def verify_ref(self, ref):
        return ['git', '-C', self.repo, 'rev-parse', '--verify', '--quiet', f'{ref}^{{commit}}']

    def archive(self, ref):
        return (['git', '-C', self.repo, 'archive', '--format=tar', ref]
                + [f'{SRC_PREFIX}/{p}' for p in PACKAGES])

    def untar(self, ip):
        return self.ssh(ip, f'mkdir -p {REMOTE_WS}/src && '
                            f'tar xf - --strip-components=3 -C {REMOTE_WS}/src')

    def build(self, ip):
        pkgs = ' '.join(PACKAGES)
        return self.ssh(ip, f"bash -lc 'cd {REMOTE_WS} && source /opt/ros/jazzy/setup.bash && "
                            f"colcon build --packages-select {pkgs}'")

    def branches(self):
        return ['git', '-C', self.repo, 'for-each-ref', '--format=%(refname:short)',
                'refs/remotes/origin']


def check_ref(ref):
    if not isinstance(ref, str) or not REF.match(ref) or '..' in ref or ref.endswith('/'):
        raise FleetError('ref must be a branch like origin/feature/minjee92/line-nav')
    if not ref.startswith('origin/'):
        raise FleetError('ref must be a remote branch (origin/...)')
    return ref


class Runner:
    """subprocess 실행(셸 없음). 테스트에서는 가짜 객체로 바꾼다."""

    def run(self, argv, timeout, log):
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL)
        return self._wait(proc, timeout, log)

    def pipe(self, argv1, argv2, timeout, log):
        """argv1 | argv2 (git archive | ssh tar)."""
        p1 = subprocess.Popen(argv1, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              stdin=subprocess.DEVNULL)
        p2 = subprocess.Popen(argv2, stdin=p1.stdout, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT)
        p1.stdout.close()
        code2 = self._wait(p2, timeout, log)
        try:
            _, err = p1.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            p1.kill()
            _, err = p1.communicate()
        for line in (err or b'').decode('utf-8', 'replace').splitlines():
            log(line)
        return code2 if p1.returncode == 0 else (p1.returncode or 1)

    @staticmethod
    def _wait(proc, timeout, log):
        try:
            out, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
            log(f'timeout after {timeout} s')
            return 124
        for line in (out or b'').decode('utf-8', 'replace').splitlines():
            log(line)
        return proc.returncode


class RobotClient:
    """로봇 테스트 콘솔 HTTP(JSON). 허용 경로만 부른다."""

    def __init__(self, port=8080, timeout=1.5):
        self.port = int(port)
        self.timeout = timeout

    def _url(self, ip, path):
        ipaddress.IPv4Address(ip)
        if path not in ROBOT_PATHS:
            raise FleetError(f'path not allowed: {path}')
        return f'http://{ip}:{self.port}{path}'

    def request(self, ip, path, body=None):
        data = None if body is None else json.dumps(body).encode('utf-8')
        req = urllib.request.Request(self._url(ip, path), data=data, method='GET' if body is None
                                     else 'POST', headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:  # noqa: S310 - http 고정
                raw = resp.read(MAX_RESPONSE + 1)
        except urllib.error.HTTPError as e:
            raw = e.read(MAX_RESPONSE + 1)
            try:
                return {'http_status': e.code, **json.loads(raw.decode('utf-8'))}
            except (ValueError, UnicodeDecodeError):
                return {'http_status': e.code, 'success': False, 'msg': e.reason}
        if len(raw) > MAX_RESPONSE:
            raise FleetError('robot response too large')
        return json.loads(raw.decode('utf-8'))


class FleetManager:
    def __init__(self, store, commands, client=None, runner=None, poll_sec=1.0):
        self.store = store
        self.cmd = commands
        self.client = client or RobotClient()
        self.runner = runner or Runner()
        self.poll_sec = poll_sec
        self._jobs = {}
        self._order = deque(maxlen=30)
        self._busy = set()
        self._status = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._poller = None

    # ---------------- 상태 폴링 ----------------
    def start_polling(self):
        if self._poller is None:
            self._poller = threading.Thread(target=self._poll_loop, daemon=True)
            self._poller.start()

    def close(self):
        self._stop.set()

    def _poll_loop(self):
        while not self._stop.is_set():
            for r in self.store.list():
                self.poll_once(r)
            self._stop.wait(self.poll_sec)

    def poll_once(self, robot):
        entry = {'t': time.time()}
        try:
            entry['route'] = self.client.request(robot.ip, '/api/route')
            entry['ok'] = True
        except (OSError, ValueError) as e:
            entry.update(ok=False, error=f'console not reachable: {e}')
        with self._lock:
            self._status[robot.name] = entry
        return entry

    def snapshot(self):
        now = time.time()
        with self._lock:
            status = dict(self._status)
            jobs = [self._job_view(self._jobs[j]) for j in self._order]
            busy = set(self._busy)
        robots = []
        for r in self.store.list():
            st = status.get(r.name)
            robots.append(dict(asdict(r), busy=r.name in busy, status=None if st is None else
                               dict(st, age_sec=round(now - st['t'], 1))))
        return {'robots': robots, 'jobs': jobs, 'packages': list(PACKAGES),
                'release_ref': RELEASE_REF, 'fleet_file': self.store.path}

    # ---------------- 경로 명령 ----------------
    def route_command(self, name, data):
        cmd = parse_route_command(data)
        robot = self.store.get(name)
        return self.client.request(robot.ip, '/api/route/command', cmd)

    def stop_all(self):
        out = {}
        for r in self.store.list():
            res = {}
            for path, body in (('/api/route/command', {'cmd': 'stop'}), ('/api/estop', {})):
                try:
                    res[path] = self.client.request(r.ip, path, body).get('success', False)
                except (OSError, ValueError) as e:
                    res[path] = f'failed: {e}'
            out[r.name] = res
        return out

    # ---------------- 작업(SSH·git) ----------------
    def branches(self):
        lines = []
        code = self.runner.run(self.cmd.branches(), 15, lines.append)
        if code != 0:
            raise FleetError('git for-each-ref failed: ' + ' '.join(lines[-3:]))
        return sorted(b for b in lines if b.startswith('origin/') and b != 'origin/HEAD'
                      and REF.match(b))

    def start_job(self, name, action, params=None, wait=False):
        params = params or {}
        robot = self.store.get(name)
        if action not in ACTIONS:
            raise FleetError(f"action must be one of {', '.join(ACTIONS)}")
        steps = self._steps(robot, action, params)   # 입력 검사는 시작 전에
        with self._lock:
            if robot.name in self._busy:
                raise FleetError(f'{robot.name} already has a running job')
            job = {'id': uuid.uuid4().hex[:12], 'robot': robot.name, 'action': action,
                   'params': {k: v for k, v in params.items() if k in ('ref', 'domain')},
                   'state': 'running', 'code': None, 'log': deque(maxlen=300),
                   'started': time.time(), 'ended': None}
            self._jobs[job['id']] = job
            if len(self._order) == self._order.maxlen:
                self._jobs.pop(self._order[0], None)
            self._order.append(job['id'])
            self._busy.add(robot.name)
        t = threading.Thread(target=self._run_job, args=(job, steps, TIMEOUTS[action]),
                             daemon=True)
        t.start()
        if wait:
            t.join()
        return self._job_view(job)

    def _steps(self, robot, action, params):
        c = self.cmd
        if action == 'check':
            return [('run', c.check(robot.ip))]
        if action == 'read_domain':
            return [('run', c.read_domain(robot.ip))]
        if action == 'set_domain':
            domain = params.get('domain', robot.domain)
            return [('run', c.set_domain(robot.ip, domain))]
        if action == 'build':
            return [('run', c.build(robot.ip))]
        ref = RELEASE_REF if params.get('release') else check_ref(params.get('ref'))
        params['ref'] = ref
        return [('run', c.fetch()), ('run', c.verify_ref(ref)),
                ('pipe', c.archive(ref), c.untar(robot.ip))]

    def _run_job(self, job, steps, timeout):
        def log(line):
            job['log'].append(line[:500])
        code = 0
        try:
            for step in steps:
                log('$ ' + ' '.join(step[1]) + (' | ' + ' '.join(step[2]) if step[0] == 'pipe'
                                                else ''))
                if step[0] == 'pipe':
                    code = self.runner.pipe(step[1], step[2], timeout, log)
                else:
                    code = self.runner.run(step[1], timeout, log)
                if code != 0:
                    break
        except OSError as e:   # ssh/git 이 없음 등
            log(f'error: {e}')
            code = 127
        job['code'] = code
        job['state'] = 'ok' if code == 0 else 'failed'
        job['ended'] = time.time()
        with self._lock:
            self._busy.discard(job['robot'])

    @staticmethod
    def _job_view(job):
        return {k: (list(v) if k == 'log' else v) for k, v in job.items()}

    def jobs(self):
        with self._lock:
            return [self._job_view(self._jobs[j]) for j in self._order]
