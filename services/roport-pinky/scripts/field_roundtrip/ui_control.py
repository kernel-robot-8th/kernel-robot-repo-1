"""Robot-local, fail-closed UI lease. No ROS or process discovery side effects.

An unguessable per-launch id is never reused. Stop writes a permanent tombstone
under the same flock as launch, so even a delayed SSH Start cannot revive it.
Heartbeat challenges expire on the robot's monotonic clock; late packets cannot
revive an expired lease. This is not a replacement for a physical emergency stop.
"""
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import re
import threading
import time
import uuid

TTL = 3.0


class ControlLost(RuntimeError):
    pass


class Control:
    def __init__(self, root, token, clock=time.monotonic, boot=None):
        if not isinstance(token, str) or not re.fullmatch(r'[0-9a-f]{32}', token):
            raise ControlLost('Invalid UI control identity')
        self.root = Path(root)/'ui-control'
        self.root.mkdir(exist_ok=True)
        self.token, self.clock = token, clock
        self.boot = boot or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.path = self.root/(token+'.json')

    @contextmanager
    def locked(self):
        with (self.root/'control.lock').open('a') as file:
            fcntl.flock(file, fcntl.LOCK_EX)
            yield

    def write(self, data):
        temp = self.root/(self.token+'.tmp')
        temp.write_text(json.dumps(data))
        os.replace(temp, self.path)

    def valid(self):
        try:
            data = json.loads(self.path.read_text())
            now = self.clock()
            deadline = data['deadline']
            issued = data['issued']
            if (data['token'] != self.token or data['boot'] != self.boot
                    or data['stopped'] is not False
                    or type(deadline) not in (int, float) or type(issued) not in (int, float)
                    or not all(math.isfinite(v) for v in (now, issued, deadline))
                    or not issued <= now < deadline or deadline-issued > TTL
                    or not re.fullmatch('[0-9a-f]{32}', data['challenge'])):
                raise ValueError('invalid or expired')
            return data
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise ControlLost('UI control missing, stopped, expired or invalid') from exc

    @contextmanager
    def launch(self):
        with self.locked():
            if self.path.exists():
                raise ControlLost('UI Start already used or cancelled')
            now = self.clock()
            data = dict(token=self.token, boot=self.boot, stopped=False,
                        issued=now, deadline=now+TTL, challenge=uuid.uuid4().hex)
            self.write(data)
            try:
                yield data['challenge']
                self.valid()
            except BaseException:
                self.write(dict(token=self.token, stopped=True))
                raise

    def heartbeat(self, challenge):
        with self.locked():
            data = self.valid()
            if challenge != data['challenge']:
                raise ControlLost('Stale UI heartbeat challenge')
            now = self.clock()
            if not data['issued'] <= now < data['deadline']:
                raise ControlLost('UI heartbeat expired before renewal')
            data.update(issued=now, deadline=now+TTL, challenge=uuid.uuid4().hex)
            self.write(data)
            return data['challenge']

    def stop(self):
        with self.locked():
            self.write(dict(token=self.token, stopped=True))


class LeaseMonitor:
    """Check every drive lease, plus revoke independently of a blocked main loop."""
    def __init__(self, control, runner):
        self.control, self.runner = control, runner
        self.original = runner.lease
        self.lock = threading.RLock()
        self.failed = None
        self.done = threading.Event()
        runner.lease = self.lease

    def check(self):
        with self.lock:
            if self.failed is not None:
                raise ControlLost(self.failed)
            try:
                self.control.valid()
            except ControlLost as exc:
                self.failed = str(exc)
                self.runner.owner = 'none'
                self.original('none')
                raise

    def lease(self, owner):
        with self.lock:
            if owner != 'none':
                self.check()
            self.original(owner)

    def watch(self):
        while not self.done.wait(.05):
            try:
                self.check()
            except ControlLost:
                # Keep revoking even while a pending action acknowledgement blocks cleanup.
                with self.lock:
                    self.original('none')

    def start(self):
        self.check()
        self.thread = threading.Thread(target=self.watch, daemon=True)
        self.thread.start()

    def close(self):
        self.done.set()
        self.thread.join(timeout=1)
