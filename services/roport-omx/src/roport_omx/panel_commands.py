"""Provisional receiver-side command ledger for the panel's actual device owner.

The panel must embed this at its process-start boundary. Wrapping a remote legacy
HTTP start with this class does not cancel that remote request.
"""

import json
import sqlite3
from datetime import UTC, datetime
from threading import RLock


class CommandConflict(ValueError):
    """A valid request conflicts with a recorded command or the current device owner."""


class PanelCommands:
    def __init__(self, path, *, clock=lambda: datetime.now(UTC)):
        self.clock, self.lock = clock, RLock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute(
            "CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, body TEXT, state TEXT, observation TEXT, stop_requested_at TEXT)"
        )
        # No attempt is replayed after restart, even if it had only been queued.
        self.db.execute(
            "UPDATE commands SET state='unknown' WHERE state IN ('pending','running','stopping')"
        )
        self.db.commit()

    def close(self):
        self.db.close()

    def get(self, command_id):
        with self.lock:
            row = self.db.execute(
                "SELECT * FROM commands WHERE id=?", (command_id,)
            ).fetchone()
            if row is None:
                return {"command_id": command_id, "state": "not_found"}
            return {
                "command_id": row["id"],
                "state": row["state"],
                "observation": json.loads(row["observation"])
                if row["observation"]
                else None,
            }

    def submit(self, request):
        if (
            not isinstance(request, dict)
            or set(request) != {"command_id", "expires_at", "payload"}
            or not isinstance(request["command_id"], str)
            or not request["command_id"].strip()
            or not isinstance(request["payload"], dict)
        ):
            raise ValueError("command_id, expires_at and payload required")
        expiry = datetime.fromisoformat(request["expires_at"])
        if expiry.tzinfo is None:
            raise ValueError("expires_at requires timezone")
        body = json.dumps(request, sort_keys=True, allow_nan=False)
        with self.lock:
            row = self.db.execute(
                "SELECT * FROM commands WHERE id=?", (request["command_id"],)
            ).fetchone()
            if row:
                if row["body"] is not None and row["body"] != body:
                    raise CommandConflict("command_id reused")
                return self.get(request["command_id"])
            if self.db.execute(
                "SELECT 1 FROM commands WHERE state IN ('pending','running','stopping','unknown')"
            ).fetchone():
                raise CommandConflict("panel busy or requires reconciliation")
            state = "expired" if expiry <= self.clock() else "pending"
            with self.db:
                self.db.execute(
                    "INSERT INTO commands (id,body,state) VALUES (?,?,?)",
                    (request["command_id"], body, state),
                )
            return self.get(request["command_id"])

    def start(self, command_id, begin):
        """begin creates the device process locally and returns; it must not queue remote work."""
        with self.lock:
            row = self.db.execute(
                "SELECT * FROM commands WHERE id=?", (command_id,)
            ).fetchone()
            if row is None or row["state"] != "pending":
                return self.get(command_id)
            request = json.loads(row["body"])
            state = (
                "expired"
                if datetime.fromisoformat(request["expires_at"]) <= self.clock()
                else "running"
            )
            with self.db:
                self.db.execute(
                    "UPDATE commands SET state=? WHERE id=?", (state, command_id)
                )
            if state == "running":
                # Keep local process creation atomic with cancel: unlocking permits a late start after stop.
                try:
                    begin(request["payload"])
                except Exception:
                    with self.db:
                        self.db.execute(
                            "UPDATE commands SET state='unknown' WHERE id=?",
                            (command_id,),
                        )
                    raise
            return self.get(command_id)

    def cancel(self, command_id, stop):
        if not isinstance(command_id, str) or not command_id.strip():
            raise ValueError("command_id required")
        with self.lock:
            state = self.get(command_id)["state"]
            if state in {"cancelled", "expired", "completed", "failed"}:
                return self.get(command_id)
            with self.db:
                self.db.execute(
                    "INSERT INTO commands (id,state) VALUES (?,'cancelled') ON CONFLICT(id) DO UPDATE SET state=excluded.state",
                    (command_id,),
                )
                if state in {"running", "stopping", "unknown"}:
                    self.db.execute(
                        "UPDATE commands SET state='stopping', stop_requested_at=CASE WHEN ?='stopping' THEN stop_requested_at ELSE ? END WHERE id=?",
                        (state, self.clock().isoformat(), command_id),
                    )
            if state in {"running", "stopping", "unknown"}:
                stop()  # Request only; confirmation is a separate physical observation.
            return self.get(command_id)

    def confirm_stopped(self, command_id, sample):
        """The device owner supplies a measured motor stop after cancellation."""
        with self.lock, self.db:
            row = self.db.execute(
                "SELECT * FROM commands WHERE id=?", (command_id,)
            ).fetchone()
            if row is None or row["state"] != "stopping":
                return
            observed = datetime.fromisoformat(sample["observed_at"])
            since = datetime.fromisoformat(row["stop_requested_at"])
            if (
                sample.get("stopped") is not True
                or not (since <= observed <= self.clock())
                or (self.clock() - observed).total_seconds() > 3
            ):
                raise ValueError("fresh post-cancel motor stop required")
            self.db.execute(
                "UPDATE commands SET state='cancelled' WHERE id=?", (command_id,)
            )

    def finish(self, command_id, observation):
        if observation.get("state") not in {"finished", "failed"}:
            raise ValueError("terminal panel observation required")
        with self.lock, self.db:
            self.db.execute(
                "UPDATE commands SET state=?,observation=? WHERE id=? AND state='running'",
                (
                    "completed" if observation["state"] == "finished" else "failed",
                    json.dumps(observation, allow_nan=False),
                    command_id,
                ),
            )
