"""Small transactional registry. No robot I/O; no memory fallback on DB failure."""

from contextlib import contextmanager
from uuid import uuid4

import psycopg
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb

from ...contracts.robots import (
    PINKY_SLOTS,
    RegistryError,
    RegistryRequest,
    ReleaseEvidence,
    unknown_observation,
)

SELECT_SLOTS = """
SELECT s.robot_id, s.revision, s.kind, s.host_id, s.arm_id,
       b.binding_id, b.device_id, host(b.ip) AS ip, b.role, b.ros_domain_id,
       b.calibration, b.desired_revision, b.excluded, b.created_at
FROM control_robot_slots s LEFT JOIN control_robot_bindings b
ON b.robot_id=s.robot_id AND b.closed_at IS NULL
"""


class RegistryStore:
    def __init__(self, dsn: str):
        self._dsn = dsn

    @contextmanager
    def connection(self):
        try:
            with psycopg.connect(
                self._dsn, connect_timeout=2, row_factory=dict_row
            ) as conn:
                conn.execute("SET statement_timeout='3s'; SET lock_timeout='2s'")
                yield conn
        except psycopg.errors.UniqueViolation:
            raise RegistryError("duplicate_device_domain_or_address", 409) from None
        except psycopg.Error:
            raise RegistryError("registry_unavailable", 503) from None

    @staticmethod
    def metadata(conn, *, lock=False):
        row = conn.execute(
            "SELECT * FROM control_registry_meta WHERE singleton"
            + (" FOR UPDATE" if lock else "")
        ).fetchone()
        if not row or row["schema_version"] != 1:
            raise RegistryError("registry_schema_mismatch", 503)
        return row

    def snapshot(self):
        with self.connection() as conn:
            conn.execute("SET TRANSACTION ISOLATION LEVEL REPEATABLE READ READ ONLY")
            meta = self.metadata(conn)
            rows = conn.execute(
                SELECT_SLOTS + " ORDER BY s.kind DESC, s.robot_id"
            ).fetchall()
            return {
                "schema_version": 1,
                "registry_id": str(meta["registry_id"]),
                "revision": meta["revision"],
                "development": meta["development"],
                "available": True,
                "robots": [self.view(row) for row in rows],
            }

    @staticmethod
    def view(row):
        bound = row["binding_id"] is not None
        reasons = (
            ["runtime_unverified", "executor_not_connected"]
            if bound
            else ["unregistered" if row["kind"] == "pinky" else "omx_not_connected"]
        )
        if row["excluded"]:
            reasons.insert(0, "excluded")
        return {
            "robot_id": row["robot_id"],
            "kind": row["kind"],
            "host_id": row["host_id"],
            "arm_id": row["arm_id"],
            "revision": row["revision"],
            "binding_id": str(row["binding_id"]) if bound else None,
            "device_id": row["device_id"],
            "registered": bound,
            "excluded": bool(row["excluded"]),
            "desired_config_revision": row["desired_revision"],
            "desired_config": {
                k: row[k] for k in ("ip", "role", "ros_domain_id", "calibration")
            }
            if bound
            else None,
            "file_config": None,
            "observation": unknown_observation(),
            "display": {
                "freshness": "unknown",
                "config_match": "unknown",
                "ready": False,
                "reasons": reasons,
            },
        }

    def change(
        self, robot_id: str, action: str, data: dict, *, development: bool = False
    ):
        if robot_id not in PINKY_SLOTS:
            raise RegistryError("unknown_pinky_slot", 404)
        request = RegistryRequest.parse(action, data)
        # No browser-provided 'verified' flag can enable real registrations.
        if not development:
            raise RegistryError("operating_writes_disabled", 403)
        with self.connection() as conn:
            # Six fixed slots: one short lock makes revisions, uniqueness and retry receipts atomic.
            meta = self.metadata(conn, lock=True)
            if not meta["development"]:
                raise RegistryError("development_database_required", 403)
            previous = conn.execute(
                "SELECT * FROM control_registry_changes WHERE request_id=%s",
                (request.request_id,),
            ).fetchone()
            if previous:
                if (previous["robot_id"], previous["action"], previous["payload"]) != (
                    robot_id,
                    action,
                    request.payload(),
                ):
                    raise RegistryError("request_id_reused", 409)
                return previous["result"]
            row = conn.execute(
                SELECT_SLOTS + " WHERE s.robot_id=%s", (robot_id,)
            ).fetchone()
            if row["revision"] != request.expected_revision:
                raise RegistryError("revision_conflict", 409)
            binding = str(row["binding_id"]) if row["binding_id"] else None
            if action == "registration":
                if binding:
                    raise RegistryError("slot_occupied", 409)
                if not request.device_id.startswith("sim-"):
                    raise RegistryError("synthetic_device_required", 403)
                binding = str(uuid4())
                config = request.config
                conn.execute(
                    """INSERT INTO control_robot_bindings
                    (binding_id, robot_id, device_id, ip, role, ros_domain_id, calibration)
                    VALUES (%s,%s,%s,%s,%s,%s,%s)""",
                    (
                        binding,
                        robot_id,
                        request.device_id,
                        config.ip,
                        config.role,
                        config.ros_domain_id,
                        config.calibration,
                    ),
                )
            else:
                if binding is None or binding != request.binding_id:
                    raise RegistryError("binding_conflict", 409)
                if not row["device_id"].startswith("sim-"):
                    raise RegistryError("synthetic_device_required", 403)
                if action == "config":
                    config = request.config
                    conn.execute(
                        """UPDATE control_robot_bindings SET ip=%s, role=%s, ros_domain_id=%s,
                        calibration=%s, desired_revision=desired_revision+1 WHERE binding_id=%s""",
                        (
                            config.ip,
                            config.role,
                            config.ros_domain_id,
                            config.calibration,
                            binding,
                        ),
                    )
                elif action == "exclude":
                    conn.execute(
                        "UPDATE control_robot_bindings SET excluded=%s WHERE binding_id=%s",
                        (request.excluded, binding),
                    )
                else:
                    # Synthetic registrations cannot own physical work. This is not a real release proof.
                    evidence = ReleaseEvidence(
                        online=True,
                        stopped=True,
                        active_task=False,
                        pending_command=False,
                        occupied_resource=False,
                        loaded=False,
                        maintenance=False,
                    )
                    if reasons := evidence.reasons(row["excluded"]):
                        raise RegistryError(reasons[0], 409)
                    conn.execute(
                        "UPDATE control_robot_bindings SET closed_at=now() WHERE binding_id=%s",
                        (binding,),
                    )
            conn.execute(
                "UPDATE control_robot_slots SET revision=revision+1 WHERE robot_id=%s",
                (robot_id,),
            )
            conn.execute("UPDATE control_registry_meta SET revision=revision+1")
            result = {
                "request_id": request.request_id,
                "robot_id": robot_id,
                "binding_id": binding,
                "revision": row["revision"] + 1,
                "registry_revision": meta["revision"] + 1,
            }
            conn.execute(
                """INSERT INTO control_registry_changes(request_id,robot_id,action,payload,result,actor)
                VALUES (%s,%s,%s,%s,%s,'local-development')""",
                (
                    request.request_id,
                    robot_id,
                    action,
                    Jsonb(request.payload()),
                    Jsonb(result),
                ),
            )
            return result

    def history(self, robot_id: str, *, limit: int = 50):
        if (
            robot_id not in PINKY_SLOTS
            or type(limit) is not int
            or not 1 <= limit <= 100
        ):
            raise RegistryError("invalid_history_query")
        with self.connection() as conn:
            rows = conn.execute(
                """SELECT request_id, action, result, created_at, actor FROM control_registry_changes
                WHERE robot_id=%s ORDER BY created_at DESC, request_id LIMIT %s""",
                (robot_id, limit),
            ).fetchall()
            return [
                {
                    **r,
                    "request_id": str(r["request_id"]),
                    "created_at": r["created_at"].isoformat(),
                }
                for r in rows
            ]
