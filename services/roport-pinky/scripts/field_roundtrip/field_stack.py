"""Refuse foreign drive stacks; reuse only a field-owned guard, never kill one."""
from pathlib import Path
import re
import subprocess


def owned_guard(pid, root, panes, proc=Path('/proc')):
    """Require matching field/domain environment and a dedicated guard tmux ancestor."""
    try:
        env = dict(value.split('=', 1) for value in (proc/str(pid)/'environ').read_bytes().decode().split('\0') if '=' in value)
        if (env.get('ROPORT_FIELD_CONFIG_DIR') != str(Path(root).resolve())
                or env.get('ROS_DOMAIN_ID') != '13'):
            return False
        allowed = {int(parent) for name, parent in panes
                   if re.fullmatch(r'(h1g3_ui_guard_|field_guard_)[0-9]+', name)}
        for _ in range(32):
            if pid in allowed:
                return True
            status = (proc/str(pid)/'status').read_text()
            pid = int(next(line.split()[1] for line in status.splitlines() if line.startswith('PPid:')))
            if pid <= 1:
                break
    except (OSError, UnicodeError, ValueError, StopIteration):
        pass
    return False


def compatible_guard(root):
    for pattern in ('[r]oport_line_nav', '[t]rial_driver', '[t]rial_control'):
        result = subprocess.run(['pgrep', '-f', pattern], stdout=subprocess.DEVNULL)
        if result.returncode not in (0, 1):
            raise RuntimeError('Cannot inspect incompatible drive stacks')
        if result.returncode == 0:
            raise RuntimeError('Incompatible drive stack; no process was stopped')
    found = subprocess.run(['pgrep', '-f', '[r]oport_pinky/trial_guard'], capture_output=True, text=True)
    if found.returncode not in (0, 1):
        raise RuntimeError('Cannot inspect drive guard ownership')
    guards = found.stdout.split()
    if not guards:
        return False
    panes = subprocess.run(['tmux', 'list-panes', '-a', '-F', '#{session_name} #{pane_pid}'],
                           capture_output=True, text=True, timeout=2)
    rows = [row.split() for row in panes.stdout.splitlines()]
    if (len(guards) != 1 or panes.returncode or any(len(row) != 2 for row in rows)
            or not guards[0].isdigit() or not owned_guard(int(guards[0]), root, rows)):
        raise RuntimeError('Incompatible or unowned drive guard; no process was stopped')
    return True
