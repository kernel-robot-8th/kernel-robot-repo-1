"""Service requests and executor state in the control-owned PostgreSQL database."""

import argparse
import os
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from threading import RLock
from uuid import uuid4

import psycopg
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb

SCHEMA = Path(__file__).with_name("002_service.sql")
TERMINAL = {"COMPLETED", "FAILED", "REJECTED"}
STATES = {"QUEUED", "WAITING_MANIFEST", "RUNNING", "HOLD"} | TERMINAL


def migrate_service(dsn):
    """Explicit installation; construction and API startup never migrate."""
    with psycopg.connect(dsn) as conn:
        conn.execute("SELECT pg_advisory_xact_lock(71519002)")
        conn.execute(SCHEMA.read_text())


class OperationalStore:
    """One executor owns the snapshot; API reads go through its existing boundary."""

    def __init__(self, dsn):
        self._lock = RLock()
        self._conn = psycopg.connect(dsn, autocommit=True, row_factory=dict_row)
        if not self._conn.execute(
            "SELECT pg_try_advisory_lock(71519003) AS acquired"
        ).fetchone()["acquired"]:
            self._conn.close()
            raise RuntimeError("another operational executor owns this database")
        try:
            self.data = self._conn.execute(
                "SELECT data FROM control_operational_state WHERE singleton"
            ).fetchone()["data"]
        except Exception:
            self.close()
            raise

    def save(self, data):
        with self._lock:
            self._conn.execute(
                "UPDATE control_operational_state SET data=%s, updated_at=now() WHERE singleton",
                (Jsonb(data),),
            )
            self.data = deepcopy(data)

    def close(self):
        self._conn.close()


class ServiceTaskStore:
    def __init__(self, dsn):
        self.dsn = dsn

    def _connection(self):
        return psycopg.connect(self.dsn, row_factory=dict_row)

    @staticmethod
    def _event(conn, task, kind):
        message = {
            "schema_version": "1.0",
            "message_id": str(uuid4()),
            "type": kind,
            "site_id": task["site_id"],
            "source": "control-01",
            "target": "service-01",
            "sent_at": datetime.now(UTC).isoformat(),
            "task_id": task["task_id"],
            "payload": {
                "reservation_id": task["reservation_id"],
                "order_id": task["order_id"],
                "task_revision": task["revision"],
                "state": task["state"],
                "progress": task["progress"],
            },
        }
        conn.execute(
            "INSERT INTO control_service_outbox(message_id,message) VALUES (%s,%s)",
            (message["message_id"], Jsonb(message)),
        )

    def accept(self, message):
        """Commit request, task and acceptance before transport ACK; never starts motion."""
        if message.get("type") != "task.requested":
            raise ValueError("expected task.requested")
        payload = message.get("payload")
        if not isinstance(payload, dict):
            raise TypeError("payload must be an object")
        for key in ("message_id", "site_id"):
            if not isinstance(message.get(key), str) or not message[key]:
                raise ValueError(f"{key} required")
        for key in ("order_id", "reservation_id", "pickup_point_id"):
            if not isinstance(payload.get(key), str) or not payload[key]:
                raise ValueError(f"{key} required")
        if payload.get("operation") not in {"prepare_order", "customer_service"}:
            raise ValueError("unknown operation")
        if (
            type(payload.get("order_revision")) is not int
            or payload["order_revision"] < 0
        ):
            raise ValueError("invalid order revision")
        try:
            pickup_at = datetime.fromisoformat(payload["pickup_at"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("pickup_at must be an ISO timestamp") from exc
        if pickup_at.tzinfo is None:
            raise ValueError("pickup_at requires timezone")
        packages = payload.get("packages")
        if not isinstance(packages, list) or any(
            not isinstance(p, dict)
            or not isinstance(p.get("package_id"), str)
            or not p["package_id"]
            or type(p.get("quantity")) is not int
            or p["quantity"] != 1
            for p in packages
        ):
            raise ValueError("invalid package manifest")
        if len({p["package_id"] for p in packages}) != len(packages):
            raise ValueError("duplicate package in manifest")
        with self._connection() as conn:
            # Serializes retransmits and the one-active-order rule together.
            conn.execute("SELECT pg_advisory_xact_lock(71519004)")
            previous = conn.execute(
                "SELECT * FROM control_service_inbox WHERE message_id=%s",
                (message["message_id"],),
            ).fetchone()
            if previous:
                if previous["message"] != message:
                    raise ValueError("message_id reused with different content")
                return conn.execute(
                    "SELECT * FROM control_service_tasks WHERE task_id=%s",
                    (previous["task_id"],),
                ).fetchone()
            task = conn.execute(
                "SELECT * FROM control_service_tasks WHERE reservation_id=%s",
                (payload["reservation_id"],),
            ).fetchone()
            if task:
                if task["request"] != payload or task["site_id"] != message["site_id"]:
                    raise ValueError("reservation already has a different request")
            else:
                existing = conn.execute(
                    "SELECT 1 FROM control_service_tasks WHERE order_id=%s AND state NOT IN ('COMPLETED','FAILED','REJECTED')",
                    (payload["order_id"],),
                ).fetchone()
                if existing:
                    raise ValueError("order already has an active task")
                task = conn.execute(
                    """INSERT INTO control_service_tasks
                    (task_id,site_id,order_id,reservation_id,request,state)
                    VALUES (%s,%s,%s,%s,%s,%s) RETURNING *""",
                    (
                        str(uuid4()),
                        message["site_id"],
                        payload["order_id"],
                        payload["reservation_id"],
                        Jsonb(payload),
                        "QUEUED" if packages else "WAITING_MANIFEST",
                    ),
                ).fetchone()
                self._event(conn, task, "task.accepted")
            conn.execute(
                "INSERT INTO control_service_inbox(message_id,message,task_id) VALUES (%s,%s,%s)",
                (message["message_id"], Jsonb(message), task["task_id"]),
            )
            return task

    def accept_receipt(self, message):
        """Persist first-receipt eligibility; ACK means stored, not arrival verified.

        A false ``arrival_verified`` is durable: replay cannot turn an early or
        legacy receipt into departure permission when the robot arrives later.
        """
        if message.get("type") != "handover.receipt_recorded":
            raise ValueError("expected handover receipt")
        payload = message.get("payload", {})
        if not isinstance(payload, dict):
            raise TypeError("receipt payload must be an object")
        if not isinstance(payload.get("receipt_id"), str) or not payload["receipt_id"]:
            raise ValueError("receipt_id required")
        with self._connection() as conn:
            task = conn.execute(
                "SELECT * FROM control_service_tasks WHERE task_id=%s FOR UPDATE",
                (message.get("task_id"),),
            ).fetchone()
            if not task or (
                task["site_id"],
                task["reservation_id"],
                task["order_id"],
            ) != (
                message.get("site_id"),
                payload.get("reservation_id"),
                payload.get("order_id"),
            ):
                raise ValueError("receipt does not match task")
            packages = payload.get("package_ids")
            manifest = {p["package_id"] for p in task["request"]["packages"]}
            if (
                not isinstance(packages, list)
                or not packages
                or any(
                    not isinstance(package, str) or package not in manifest
                    for package in packages
                )
                or len(set(packages)) != len(packages)
            ):
                raise ValueError("receipt packages do not match task manifest")
            previous = conn.execute(
                "SELECT * FROM control_service_receipts WHERE receipt_id=%s",
                (payload["receipt_id"],),
            ).fetchone()
            if previous:
                if (
                    previous["task_id"] != task["task_id"]
                    or previous["message"]["payload"] != payload
                ):
                    raise ValueError("conflicting receipt")
                return previous
            snapshot = conn.execute(
                "SELECT data FROM control_operational_state WHERE singleton FOR SHARE"
            ).fetchone()["data"]
            children = snapshot.get("tasks", {})
            arrival_verified = all(
                children.get(f"{task['task_id']}:{package}", {}).get("stage")
                == "await_receipt"
                and children.get(f"{task['task_id']}:{package}", {}).get("status")
                == "AWAITING_RECEIPT"
                for package in packages
            )
            return conn.execute(
                "INSERT INTO control_service_receipts(task_id,receipt_id,message,arrival_verified) VALUES (%s,%s,%s,%s) RETURNING *",
                (
                    task["task_id"],
                    payload["receipt_id"],
                    Jsonb(message),
                    arrival_verified,
                ),
            ).fetchone()

    def receipts(self):
        with self._connection() as conn:
            return conn.execute(
                "SELECT * FROM control_service_receipts WHERE arrival_verified ORDER BY recorded_at"
            ).fetchall()

    def pending_tasks(self):
        with self._connection() as conn:
            return conn.execute(
                "SELECT * FROM control_service_tasks WHERE state NOT IN ('COMPLETED','FAILED','REJECTED') ORDER BY updated_at,task_id"
            ).fetchall()

    def update(self, task_id, state, progress=None):
        if state not in STATES:
            raise ValueError("unknown service task state")
        progress = progress or {}
        with self._connection() as conn:
            task = conn.execute(
                "SELECT * FROM control_service_tasks WHERE task_id=%s FOR UPDATE",
                (task_id,),
            ).fetchone()
            if not task:
                raise ValueError("unknown task")
            if task["state"] == state and task["progress"] == progress:
                return task
            if task["state"] in TERMINAL:
                raise ValueError("terminal task cannot be reopened")
            if not task["request"]["packages"] and state not in {
                "HOLD",
                "FAILED",
                "REJECTED",
            }:
                raise ValueError("package manifest is not ready")
            task = conn.execute(
                """UPDATE control_service_tasks SET state=%s,progress=%s,
                revision=revision+1,updated_at=now() WHERE task_id=%s RETURNING *""",
                (state, Jsonb(progress), task_id),
            ).fetchone()
            kind = {
                "COMPLETED": "task.completed",
                "FAILED": "task.failed",
                "REJECTED": "task.rejected",
            }.get(state, "task.progress")
            self._event(conn, task, kind)
            return task

    def outbox(self):
        with self._connection() as conn:
            return [
                r["message"]
                for r in conn.execute(
                    "SELECT message FROM control_service_outbox WHERE acknowledged_at IS NULL ORDER BY created_at,message_id"
                ).fetchall()
            ]

    def acknowledge(self, message_id):
        with self._connection() as conn:
            conn.execute(
                "UPDATE control_service_outbox SET acknowledged_at=now() WHERE message_id=%s AND acknowledged_at IS NULL",
                (message_id,),
            )


def main():
    parser = argparse.ArgumentParser(
        description="Install control service/operational tables explicitly"
    )
    parser.parse_args()
    dsn = os.environ.get("ROPORT_DATABASE_URL")
    if not dsn:
        parser.error("ROPORT_DATABASE_URL is required")
    migrate_service(dsn)
    print("Control service and operational tables ready.")


if __name__ == "__main__":
    main()
