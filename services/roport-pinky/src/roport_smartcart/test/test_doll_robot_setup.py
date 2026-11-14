"""Setup actions never start motors during a read-only connection check."""

import json
import subprocess
import sys
from types import SimpleNamespace

import pytest

from roport_smartcart.doll_robot_setup import REMOTE_SCRIPT, RobotSetup


@pytest.mark.parametrize('action', ['check', 'sensors', 'bringup', 'stop_all'])
def test_remote_lifecycle_and_stop_order(action, monkeypatch, capsys):
    names = ('bringup', 'camera', 'adc', 'guard')
    active = {name: action == 'stop_all' for name in names}
    changes, scope = [], {}

    def run(command, **_kwargs):
        if command[0] == 'pgrep':
            name = next(n for n, pattern in scope['patterns'].items() if pattern == command[2])
            return SimpleNamespace(returncode=0 if active[name] else 1)
        name = command[command.index('-t') + 1][5:] if '-t' in command else command[command.index('-s') + 1][5:]
        if command[1] == 'has-session':
            return SimpleNamespace(returncode=0 if active[name] else 1)
        changes.append((command[1], name))
        active[name] = command[1] == 'new-session'
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, 'run', run)
    monkeypatch.setattr('time.sleep', lambda _seconds: None)
    monkeypatch.setattr('os.path.isfile', lambda _path: True)
    monkeypatch.setattr(sys, 'argv', ['setup', action])
    exec(REMOTE_SCRIPT, scope)
    result = json.loads(capsys.readouterr().out)
    assert result['ok']
    if action == 'check':
        assert not changes and not any(result['nodes'].values())
    elif action == 'sensors':
        assert result['nodes']['adc'] and result['nodes']['camera'] and result['nodes']['guard']
        assert not result['nodes']['bringup']
    elif action == 'bringup':
        assert all(result['nodes'].values())
        exec(REMOTE_SCRIPT, scope)
        assert len(changes) == 4  # repeated start does not duplicate processes
    else:
        assert [name for _, name in changes] == list(names)
        assert not any(result['nodes'].values())


def test_ssh_failure_and_pending_shutdown(monkeypatch):
    setup = RobotSetup('pinky@example.com')
    calls = []

    def run(command, **kwargs):
        calls.append(command[-1])
        assert 'BatchMode=yes' in command and 'StrictHostKeyChecking=yes' in command
        assert kwargs['timeout'] == 25
        if len(calls) == 1:
            setup.pending_stop = True
        return SimpleNamespace(stdout=json.dumps({'ok': True, 'nodes': {}}))

    monkeypatch.setattr(subprocess, 'run', run)
    setup._work('sensors')
    assert calls == ['python3 - sensors', 'python3 - stop_all']
    assert not setup.snapshot()['busy']
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: (_ for _ in ()).throw(subprocess.TimeoutExpired('ssh', 25)))
    setup._work('check')
    assert not setup.snapshot()['connected']
    assert not setup.submit('arbitrary_shell_command')
    with pytest.raises(ValueError):
        RobotSetup('-oProxyCommand=bad')
