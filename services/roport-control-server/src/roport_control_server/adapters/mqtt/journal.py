"""Small, exclusive local trial journal. Not the operational/shared database."""

import fcntl
import json
import os
import tempfile
from copy import deepcopy
from pathlib import Path


class Journal:
    """One Linux process owns a path; save before publishing or invoking hardware."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = self.path.with_suffix(self.path.suffix + ".lock").open("a")
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.data = json.loads(self.path.read_text()) if self.path.exists() else {}
            if not isinstance(self.data, dict):
                raise TypeError("invalid trial journal")
        except BaseException:
            self._lock.close()
            raise
        self.failed = False

    def save(self, data):
        if self.failed:
            raise OSError("journal failed; restart and reconcile")
        raw = json.dumps(data, allow_nan=False, ensure_ascii=False).encode()
        if len(raw) > 4 * 1024 * 1024:
            raise ValueError("trial journal full; archive after reconciliation")
        name = None
        try:
            with tempfile.NamedTemporaryFile(dir=self.path.parent, delete=False) as f:
                name = f.name
                f.write(raw)
                f.flush()
                os.fsync(f.fileno())
            os.replace(name, self.path)
            fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            self.data = deepcopy(data)
        except BaseException:
            self.failed = True
            raise
        finally:
            if name and os.path.exists(name):
                os.unlink(name)

    def close(self):
        self._lock.close()
