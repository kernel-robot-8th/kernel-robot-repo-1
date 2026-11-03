"""mode:=fleet(노트북) Pinky 설정·테스트 관제. 실제 ssh·git·HTTP 없이 가짜 Runner·Client 로 검사한다."""

import json
import os
import stat

import pytest

from pinky_test_console import fleet
from pinky_test_console.fleet import (Commands, FleetError, FleetManager, FleetStore, check_ref,
                                      validate_robot)
from pinky_test_console.route_view import RouteCommandError

GOOD = {'name': 'pinky1', 'ip': '192.168.4.21', 'role': 'logistics', 'domain': 14,
        'calib': 'pinky_d660'}


class FakeRunner:
    def __init__(self, codes=None, out=None):
        self.calls = []
        self.codes = codes or {}
        self.out = out or []

    def run(self, argv, timeout, log):
        self.calls.append(('run', argv, timeout))
        for line in self.out:
            log(line)
        return self.codes.get(argv[0] if argv[0] != 'git' else argv[3], 0)

    def pipe(self, argv1, argv2, timeout, log):
        self.calls.append(('pipe', argv1, argv2, timeout))
        return 0


class FakeClient:
    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def request(self, ip, path, body=None):
        self.calls.append((ip, path, body))
        if self.fail:
            raise OSError('connection refused')
        if path == '/api/route':
            return {'state': {'state': 'idle'}, 'stale': False}
        return {'success': True}


@pytest.fixture
def store(tmp_path):
    return FleetStore(str(tmp_path / 'fleet.json'))


@pytest.fixture
def mgr(store):
    store.upsert(GOOD)
    return FleetManager(store, Commands('/repo', '/home/u/.ssh/pinky_ed25519'),
                        client=FakeClient(), runner=FakeRunner())


@pytest.mark.parametrize('key, value', [
    ('name', 'Pinky 1'), ('name', 'a' * 17), ('ip', '192.168.4.300'), ('ip', 'pinky.local'),
    ('ip', '1.2.3.4; reboot'), ('role', 'omx'), ('domain', 102), ('domain', '14'),
    ('domain', True), ('calib', 'd660'), ('calib', 'pinky_d660;x'),
])
def test_validate_robot_rejects(key, value):
    with pytest.raises(FleetError):
        validate_robot(dict(GOOD, **{key: value}))


def test_store_roundtrip_and_permissions(tmp_path, store):
    store.upsert(GOOD)
    store.upsert(dict(GOOD, name='pinky2', ip='192.168.4.22', role='smartcart', domain=16))
    with pytest.raises(FleetError, match='already registered'):
        store.upsert(dict(GOOD, name='pinky3'))           # 같은 IP 다른 이름
    again = FleetStore(store.path)
    assert [r.name for r in again.list()] == ['pinky1', 'pinky2']
    assert stat.S_IMODE(os.stat(store.path).st_mode) == 0o600
    data = json.load(open(store.path))
    assert set(data['robots'][0]) == {'name', 'ip', 'role', 'domain', 'calib'}  # 비밀값 없음
    again.remove('pinky2')
    with pytest.raises(FleetError):
        again.remove('pinky2')


def test_ssh_commands_have_no_user_text():
    c = Commands('/repo', '/k')
    argv = c.set_domain('192.168.4.21', 16)
    assert argv[:2] == ['ssh', '-i'] and 'BatchMode=yes' in argv
    assert argv[-2] == 'pinky@192.168.4.21'
    assert "export ROS_DOMAIN_ID=16  # roport-domain" in argv[-1]
    assert "sed -i '/# roport-domain$/d' ~/.bashrc" in argv[-1]
    for bad in (102, -1, '16; rm -rf ~', True):
        with pytest.raises(FleetError):
            c.set_domain('192.168.4.21', bad)
    with pytest.raises(ValueError):
        c.ssh('1.2.3.4 -oProxyCommand=x', 'true')
    assert c.archive('origin/main')[-4:] == [f'services/roport-pinky/src/{p}'
                                             for p in fleet.PACKAGES]
    assert 'colcon build --packages-select ' + ' '.join(fleet.PACKAGES) in c.build('10.0.0.1')[-1]


@pytest.mark.parametrize('ref', ['main', 'origin/../x', 'origin/a b', '-origin/x', 'origin/',
                                 'origin/x;ls', None, 'origin/' + 'a' * 120])
def test_check_ref_rejects(ref):
    with pytest.raises(FleetError):
        check_ref(ref)


def test_deploy_job_steps(mgr):
    job = mgr.start_job('pinky1', 'deploy', {'ref': 'origin/feature/minjee92/line-nav'},
                        wait=True)
    assert job['state'] == 'ok' and job['params']['ref'] == 'origin/feature/minjee92/line-nav'
    kinds = [(c[0], c[1][3] if c[1][0] == 'git' else c[1][0]) for c in mgr.runner.calls]
    assert kinds == [('run', 'fetch'), ('run', 'rev-parse'), ('pipe', 'archive')]
    pipe = mgr.runner.calls[-1]
    assert pipe[2][-1].startswith('mkdir -p ~/pinky_test_ws/src && tar xf -')
    release = mgr.start_job('pinky1', 'deploy', {'release': True}, wait=True)
    assert release['params']['ref'] == 'origin/main'


def test_job_failure_stops_and_reports(store):
    store.upsert(GOOD)
    m = FleetManager(store, Commands('/repo', '/k'), client=FakeClient(),
                     runner=FakeRunner(codes={'rev-parse': 1}))
    job = m.start_job('pinky1', 'deploy', {'ref': 'origin/nope'}, wait=True)
    assert job['state'] == 'failed' and job['code'] == 1
    assert [c[0] for c in m.runner.calls] == ['run', 'run']      # archive 까지 가지 않음


def test_job_rejects_bad_input_before_running(mgr):
    for action, params in (('reboot', {}), ('deploy', {'ref': 'main'}),
                           ('set_domain', {'domain': 500})):
        with pytest.raises(FleetError):
            mgr.start_job('pinky1', action, params)
    with pytest.raises(FleetError, match='unknown robot'):
        mgr.start_job('pinky9', 'check')
    assert mgr.runner.calls == []


def test_one_job_per_robot(mgr):
    mgr._busy.add('pinky1')
    with pytest.raises(FleetError, match='running job'):
        mgr.start_job('pinky1', 'check')


def test_route_command_validated_and_proxied(mgr):
    assert mgr.route_command('pinky1', {'cmd': 'start', 'route': ['N14', 'N16']}) == \
        {'success': True}
    assert mgr.client.calls[-1] == ('192.168.4.21', '/api/route/command',
                                    {'cmd': 'start', 'route': ['N14', 'N16']})
    with pytest.raises(RouteCommandError):
        mgr.route_command('pinky1', {'cmd': 'start', 'route': ['N14']})


def test_poll_and_snapshot(mgr):
    entry = mgr.poll_once(mgr.store.get('pinky1'))
    assert entry['ok'] and entry['route']['state']['state'] == 'idle'
    snap = mgr.snapshot()
    assert snap['robots'][0]['status']['ok'] and snap['release_ref'] == 'origin/main'
    mgr.client.fail = True
    assert mgr.poll_once(mgr.store.get('pinky1'))['ok'] is False


def test_stop_all_reports_each_robot(mgr):
    out = mgr.stop_all()
    assert out['pinky1'] == {'/api/route/command': True, '/api/estop': True}
    mgr.client.fail = True
    assert 'failed' in mgr.stop_all()['pinky1']['/api/estop']


def test_branches_filtered(store):
    store.upsert(GOOD)
    runner = FakeRunner(out=['origin/HEAD', 'origin/main', 'origin/feature/minjee92/line-nav',
                             'origin/bad;name'])
    m = FleetManager(store, Commands('/repo', '/k'), client=FakeClient(), runner=runner)
    assert m.branches() == ['origin/feature/minjee92/line-nav', 'origin/main']


def test_robot_client_allows_only_known_paths():
    c = fleet.RobotClient()
    with pytest.raises(FleetError):
        c._url('192.168.4.21', '/api/teleop')
    with pytest.raises(ValueError):
        c._url('evil.example', '/api/route')
    assert c._url('192.168.4.21', '/api/route') == 'http://192.168.4.21:8080/api/route'


class EventClient(FakeClient):
    def __init__(self):
        super().__init__()
        self.state = {'state': 'follow', 'reason': '',
                      'events': [{'t': 1.0, 'event': 'start', 'detail': 'N14 → N16'}]}

    def request(self, ip, path, body=None):
        if path == '/api/route' and not self.fail:
            self.calls.append((ip, path, body))
            return {'state': self.state, 'stale': False}
        return super().request(ip, path, body)


def test_log_collects_events_once_and_state_changes(store, tmp_path):
    store.upsert(GOOD)
    client = EventClient()
    m = FleetManager(store, Commands('/repo', '/k'), client=client, runner=FakeRunner(),
                     log=fleet.FleetLog(str(tmp_path / 'logs')))
    r = store.get('pinky1')
    m.poll_once(r)
    m.poll_once(r)                                   # 같은 이벤트는 다시 기록하지 않음
    client.state = dict(client.state, state='error', reason='node_missed: N16',
                        events=client.state['events'] + [{'t': 9.5, 'event': 'error',
                                                          'detail': 'node_missed'}])
    m.poll_once(r)
    client.fail = True
    m.poll_once(r)
    rows, last = m.log.after(0)
    kinds = [(x['source'], x['event']) for x in rows]
    assert kinds == [('console', 'reachable'), ('robot', 'start'), ('state', 'follow'),
                     ('robot', 'error'), ('state', 'error'), ('console', 'unreachable')]
    assert last == len(rows) and m.log.after(last)[0] == []
    path = m.log.path()
    assert path.startswith(str(tmp_path / 'logs')) and open(path).read().count('\n') == 7


def test_log_records_commands_and_jobs(mgr):
    mgr.route_command('pinky1', {'cmd': 'stop'})
    mgr.start_job('pinky1', 'check', wait=True)
    events = [(x['source'], x['event']) for x in mgr.log.after(0)[0]]
    assert ('fleet', 'command') in events and ('fleet', 'command_result') in events
    assert ('job', 'check start') in events and ('job', 'check ok') in events


def test_log_csv_escapes_formulas():
    log = fleet.FleetLog(None)
    log.add('pinky1', 'robot', '=HYPERLINK("x")', '+1')
    log.add('pinky2', 'robot', 'ok', '-2')
    text = log.csv_text()
    assert "'=HYPERLINK" in text and "'+1" in text
    only = log.csv_text('pinky2')
    assert 'pinky1' not in only and "'-2" in only
