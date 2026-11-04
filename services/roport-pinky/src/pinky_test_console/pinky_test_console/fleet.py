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

import csv
import datetime
import io
import ipaddress
import json
import os
import re
import subprocess
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from collections import deque
from dataclasses import asdict, dataclass

from .route_view import ROS_LOG_DIR_RE, ROUTE_LOG_RE, parse_route_command

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
ROBOT_PATHS = ('/api/route', '/api/status', '/api/route/command', '/api/estop', '/api/logs',
               '/api/logs/files')
MAX_RESPONSE = 256 * 1024
TIMEOUTS = {'check': 15, 'read_domain': 15, 'set_domain': 15, 'deploy': 180, 'build': 900,
            'fetch_log': 60, 'fetch_roslog': 120, 'console_start': 40, 'console_stop': 25}
ACTIONS = tuple(TIMEOUTS)
RMW_CHOICES = ('', 'rmw_fastrtps_cpp', 'rmw_cyclonedds_cpp')
# 원격으로 켤 수 있는 로봇 콘솔 모드(bringup 은 켜지도 끄지도 않음). 마커는 부착·시험 뒤에 추가
CONSOLE_MODES = ('test', 'ir')
CONSOLE_PATTERN = '[r]os2 launch pinky_test_console'   # [r] = pgrep 이 자기 셸을 잡지 않게
LOG_FIELDS = ('seq', 'time', 'robot', 'source', 'event', 'detail')
LOG_KEEP = 3000          # 화면·다운로드용 메모리 기록 줄 수(파일에는 모두 남김)


def _csv_cell(value):
    """표 계산 프로그램이 수식으로 읽지 않게(CSV injection) =,+,-,@ 로 시작하면 ' 를 붙인다."""
    text = '' if value is None else str(value)
    return "'" + text if text[:1] in ('=', '+', '-', '@', '\t', '\r') else text


class FleetLog:
    """테스트 관제 기록: 로봇 이벤트·상태 변화·콘솔 연결·명령·작업. 메모리 + 날짜별 CSV 파일."""

    def __init__(self, log_dir=None, keep=LOG_KEEP):
        self._lock = threading.Lock()
        self._rows = deque(maxlen=keep)
        self._seq = 0
        self.log_dir = None if log_dir is None else os.path.abspath(os.path.expanduser(log_dir))
        self.file_error = None

    def path(self, now=None):
        if self.log_dir is None:
            return None
        day = (now or datetime.datetime.now()).strftime('%Y%m%d')
        return os.path.join(self.log_dir, f'fleet_{day}.csv')

    def add(self, robot, source, event, detail=''):
        now = datetime.datetime.now()
        with self._lock:
            self._seq += 1
            row = {'seq': self._seq, 'time': now.isoformat(timespec='milliseconds'),
                   'robot': robot, 'source': source, 'event': event, 'detail': str(detail)[:500]}
            self._rows.append(row)
            self._write(row, now)
        return row

    def _write(self, row, now):
        path = self.path(now)
        if path is None or self.file_error:
            return
        try:
            os.makedirs(self.log_dir, mode=0o700, exist_ok=True)
            new = not os.path.exists(path)
            with open(path, 'a', encoding='utf-8', newline='') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(LOG_FIELDS)
                w.writerow([_csv_cell(row[k]) for k in LOG_FIELDS])
        except OSError as e:
            self.file_error = f'log file not writable: {e}'

    def after(self, seq, limit=500):
        with self._lock:
            rows = [r for r in self._rows if r['seq'] > seq]
            last = self._seq
        return rows[-limit:], last

    def csv_text(self, robot=None):
        out = io.StringIO()
        w = csv.writer(out)
        w.writerow(LOG_FIELDS)
        with self._lock:
            rows = list(self._rows)
        for r in rows:
            if robot is None or r['robot'] == robot:
                w.writerow([_csv_cell(r[k]) for k in LOG_FIELDS])
        return out.getvalue()


class FleetError(ValueError):
    pass


@dataclass(frozen=True)
class Robot:
    name: str
    ip: str
    role: str
    domain: int
    calib: str
    rmw: str = ''      # 원격 콘솔 실행 때 RMW_IMPLEMENTATION (6422 = rmw_fastrtps_cpp)


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
    rmw = data.get('rmw', '') or ''
    if rmw not in RMW_CHOICES:
        raise FleetError(f"rmw must be one of {', '.join(c or '(기본)' for c in RMW_CHOICES)}")
    return Robot(name, ip, role, domain, calib, rmw)


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

    def scp_from(self, ip, remote_path, dest_dir):
        """로봇 → 노트북 파일 복사. remote_path 는 홈 기준 상대 경로(검사한 이름만)."""
        ipaddress.IPv4Address(ip)
        return ['scp', '-i', self.ssh_key, '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
                '-o', 'StrictHostKeyChecking=accept-new', '-o', 'ConnectTimeout=5',
                f'{self.user}@{ip}:{remote_path}', dest_dir + os.sep]

    def tar_roslog(self, ip, dirname):
        if not ROS_LOG_DIR_RE.match(dirname):
            raise FleetError('invalid ROS log folder name')
        return self.ssh(ip, f'tar czf - -C ~/.ros/log {dirname}')

    def console_stop(self, ip):
        # SIGINT → launch 가 콘솔·line_nav 를 정상 종료(0 속도 발행). 10 s 안에 안 끝나면 TERM
        p = CONSOLE_PATTERN
        return self.ssh(ip, (
            f"pkill -INT -f '{p}'; for i in $(seq 20); do pgrep -f '{p}' >/dev/null || "
            f"exit 0; sleep 0.5; done; pkill -TERM -f '{p}'; sleep 1; "
            f"pgrep -f '{p}' >/dev/null && echo 'console still running' && exit 1; exit 0"))

    def console_start(self, ip, mode, calib, rmw=''):
        if mode not in CONSOLE_MODES:
            raise FleetError(f"mode must be one of {', '.join(CONSOLE_MODES)}")
        if not CALIB.match(calib) or rmw not in RMW_CHOICES:
            raise FleetError('invalid robot calibration or rmw')
        p = CONSOLE_PATTERN
        env = f'RMW_IMPLEMENTATION={rmw} ' if rmw else ''
        # 패키지 이름은 변수로 넘긴다: 이 원격 셸 명령줄에 「ros2 launch pinky_test_console」이 그대로
        # 있으면 pgrep 이 셸 자신을 실행 중인 콘솔로 잡는다
        launch = f'ros2 launch "$PKG" console.launch.py mode:={mode} robot:={calib}'
        # bash -i: 로봇 ~/.bashrc(ROS·Pinky 워크스페이스·ROS_DOMAIN_ID)를 읽고 실행. bringup 은 건드리지 않음
        return self.ssh(ip, (
            f"pgrep -f '{p}' >/dev/null && echo 'console already running (stop first)' && exit 1; "
            # 「mkdir … && nohup … &」은 && 목록 전체가 서브셸로 백그라운드가 되어 ssh 출력을 붙잡는다
            "mkdir -p ~/roport_logs; "
            f"PKG=pinky_test_console nohup bash -ic '{env}exec {launch}' "
            f"> ~/roport_logs/console_{mode}.log 2>&1 < /dev/null & sleep 4; "
            f"pgrep -fa '{p}' || (tail -n 30 ~/roport_logs/console_{mode}.log; exit 1)"))


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

    def run_to_file(self, argv, path, timeout, log):
        """argv 의 표준 출력을 파일로(ssh tar → .tar.gz). 실패하면 파일을 지운다."""
        with open(path, 'wb') as out:
            proc = subprocess.Popen(argv, stdout=out, stderr=subprocess.PIPE,
                                    stdin=subprocess.DEVNULL)
            try:
                _, err = proc.communicate(timeout=timeout)
            except subprocess.TimeoutExpired:
                proc.kill()
                _, err = proc.communicate()
                log(f'timeout after {timeout} s')
                code = 124
            else:
                code = proc.returncode
        for line in (err or b'').decode('utf-8', 'replace').splitlines():
            log(line)
        if code != 0 and os.path.exists(path):
            os.unlink(path)
        return code

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

    def request(self, ip, path, body=None, query=None):
        data = None if body is None else json.dumps(body).encode('utf-8')
        url = self._url(ip, path)
        if query:
            url += '?' + urllib.parse.urlencode(query)
        req = urllib.request.Request(url, data=data, method='GET' if body is None
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


class RobotProxy:
    """노트북 통합 화면(test.html)에서 로봇 콘솔 페이지·API 를 같은 출처로 쓰려는 중계.

    등록된 로봇의 콘솔(http://<IP>:<port>)로만, 로봇 화면 파일과 api/ 경로만 넘긴다.
    """

    PAGES = {'', 'index.html', 'app.js', 'style.css', 'ir.html', 'ir.js', 'marker.html',
             'marker.js', 'amcl.html', 'amcl.js', 'route.html', 'route.js', 'routemap.js'}
    API = re.compile(r'^api/[A-Za-z0-9_./-]{1,100}$')
    MAX_BODY = 8 * 1024 * 1024

    def __init__(self, port=8080, timeout=3.0):
        self.port = int(port)
        self.timeout = timeout

    def allowed(self, sub):
        return sub in self.PAGES or (bool(self.API.match(sub)) and '..' not in sub)

    def forward(self, ip, sub, query, method, body=None, content_type=None):
        """→ (status, content_type, content_disposition, body). 연결 실패는 OSError."""
        ipaddress.IPv4Address(ip)
        if not self.allowed(sub) or method not in ('GET', 'POST'):
            raise FleetError('path not allowed')
        url = f'http://{ip}:{self.port}/{sub}' + (f'?{query}' if query else '')
        headers = {'Content-Type': content_type} if content_type else {}
        req = urllib.request.Request(url, data=body if method == 'POST' else None,
                                     method=method, headers=headers)
        try:
            resp = urllib.request.urlopen(req, timeout=self.timeout)  # noqa: S310 - http 고정
        except urllib.error.HTTPError as e:
            resp = e
        with resp:
            data = resp.read(self.MAX_BODY + 1)
            if len(data) > self.MAX_BODY:
                raise FleetError('robot response too large')
            return (resp.status if hasattr(resp, 'status') else resp.code,
                    resp.headers.get('Content-Type', 'application/octet-stream'),
                    resp.headers.get('Content-Disposition'), data)


class FleetManager:
    def __init__(self, store, commands, client=None, runner=None, poll_sec=1.0, log=None):
        self.log = log or FleetLog()
        self.proxy = RobotProxy(getattr(client, 'port', 8080))
        self._seen = {}          # 로봇별 이미 기록한 line_nav 이벤트 키
        self._ros_seq = {}       # 로봇별 이미 가져온 /rosout seq
        self._last = {}          # 로봇별 (콘솔 연결, state, reason)
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
        self._log_poll(robot.name, entry)
        if entry['ok']:
            self._poll_ros_logs(robot)
        return entry

    def _poll_ros_logs(self, robot):
        """로봇 콘솔 /api/logs(/rosout)의 새 줄을 기록에 넣는다(source = pinky)."""
        after = self._ros_seq.get(robot.name, 0)
        try:
            data = self.client.request(robot.ip, '/api/logs', query={'after': after})
        except (OSError, ValueError):
            return
        last = data.get('last', 0) if isinstance(data, dict) else 0
        if not isinstance(last, int):
            return
        if last < after:                 # 로봇 콘솔이 다시 시작됨
            self._ros_seq[robot.name] = 0
            return
        for e in data.get('entries') or []:
            if isinstance(e, dict) and isinstance(e.get('seq'), int) and e['seq'] > after:
                self.log.add(robot.name, 'pinky', f"{e.get('level', '')} {e.get('node', '')}",
                             e.get('msg', ''))
        self._ros_seq[robot.name] = last

    def robot_files(self, name):
        robot = self.store.get(name)
        return self.client.request(robot.ip, '/api/logs/files')

    def _log_poll(self, name, entry):
        """새 line_nav 이벤트·state 변화·콘솔 연결 변화만 기록한다."""
        prev = self._last.get(name, (None, None, None))
        if entry['ok'] != prev[0]:
            self.log.add(name, 'console', 'reachable' if entry['ok'] else 'unreachable',
                         '' if entry['ok'] else entry.get('error', ''))
        st = (entry.get('route') or {}).get('state') if entry['ok'] else None
        state = reason = None
        if isinstance(st, dict):
            state, reason = st.get('state'), st.get('reason') or ''
            seen = self._seen.setdefault(name, (set(), deque()))
            for ev in st.get('events') or []:
                if not isinstance(ev, dict):
                    continue
                key = (ev.get('t'), ev.get('event'), ev.get('detail'))
                if key in seen[0]:
                    continue
                seen[0].add(key)
                seen[1].append(key)
                if len(seen[1]) > 500:
                    seen[0].discard(seen[1].popleft())
                self.log.add(name, 'robot', ev.get('event'),
                             f"{ev.get('detail', '')} (robot t={ev.get('t')})")
            if state != prev[1] or reason != prev[2]:
                self.log.add(name, 'state', state, reason)
        self._last[name] = (entry['ok'], state if st else prev[1], reason if st else prev[2])

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
                'release_ref': RELEASE_REF, 'fleet_file': self.store.path,
                'log_file': self.log.path(), 'log_error': self.log.file_error}

    # ---------------- 경로 명령 ----------------
    def route_command(self, name, data):
        cmd = parse_route_command(data)
        robot = self.store.get(name)
        self.log.add(name, 'fleet', 'command', json.dumps(cmd, ensure_ascii=False))
        try:
            res = self.client.request(robot.ip, '/api/route/command', cmd)
        except (OSError, ValueError) as e:
            self.log.add(name, 'fleet', 'command_failed', str(e))
            raise
        self.log.add(name, 'fleet', 'command_result',
                     'ok' if res.get('success') else res.get('msg', res))
        return res

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
            self.log.add(r.name, 'fleet', 'stop_all', json.dumps(res, ensure_ascii=False))
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
                   'params': {k: v for k, v in params.items()
                              if k in ('ref', 'domain', 'file', 'dir', 'mode')},
                   'state': 'running', 'code': None, 'log': deque(maxlen=300),
                   'started': time.time(), 'ended': None}
            self._jobs[job['id']] = job
            if len(self._order) == self._order.maxlen:
                self._jobs.pop(self._order[0], None)
            self._order.append(job['id'])
            self._busy.add(robot.name)
        self.log.add(robot.name, 'job', f'{action} start',
                     json.dumps(job['params'], ensure_ascii=False))
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
        if action == 'fetch_log':
            name = params.get('file')
            if not isinstance(name, str) or not ROUTE_LOG_RE.match(name):
                raise FleetError('file must be a route log name (route_YYYYmmdd_HHMMSS_A-B.csv)')
            dest = self._robot_log_dir(robot.name)
            return [('run', c.scp_from(robot.ip, f'roport_logs/{name}', dest))]
        if action == 'fetch_roslog':
            name = params.get('dir')
            if not isinstance(name, str) or not ROS_LOG_DIR_RE.match(name):
                raise FleetError('dir must be a ~/.ros/log folder name')
            dest = os.path.join(self._robot_log_dir(robot.name), f'roslog_{name}.tar.gz')
            return [('to_file', c.tar_roslog(robot.ip, name), dest)]
        if action == 'console_stop':
            return [('run', c.console_stop(robot.ip))]
        if action == 'console_start':
            mode = params.get('mode', 'test')
            return [('run', c.console_start(robot.ip, mode, robot.calib, robot.rmw))]
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
                                                else ' > ' + step[2] if step[0] == 'to_file'
                                                else ''))
                if step[0] == 'pipe':
                    code = self.runner.pipe(step[1], step[2], timeout, log)
                elif step[0] == 'to_file':
                    code = self.runner.run_to_file(step[1], step[2], timeout, log)
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
        self.log.add(job['robot'], 'job', f"{job['action']} {job['state']}", f'code={code}')
        with self._lock:
            self._busy.discard(job['robot'])

    def _robot_log_dir(self, name):
        """노트북 쪽 로봇별 로그 폴더(fleet_log_dir/<로봇>/). fleet_log_dir 이 없으면 ~/roport_fleet_logs."""
        root = self.log.log_dir or os.path.abspath(os.path.expanduser('~/roport_fleet_logs'))
        path = os.path.join(root, name)
        os.makedirs(path, mode=0o700, exist_ok=True)
        return path

    @staticmethod
    def _job_view(job):
        return {k: (list(v) if k == 'log' else v) for k, v in job.items()}

    def jobs(self):
        with self._lock:
            return [self._job_view(self._jobs[j]) for j in self._order]
