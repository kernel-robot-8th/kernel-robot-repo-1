"""Read-only checks of the DB contract used by the current control/service apps.

Credentials come from the apps' existing environment variables. This does not
run migrations, acquire the Fleet owner lock, seed data or prove a write succeeds.
"""

import argparse
import json
import os

import psycopg
from psycopg.conninfo import conninfo_to_dict
from psycopg.rows import dict_row

# Required consumer fields, not a second migration definition. Unused future
# service tables (admins/notifications/package_items/request_results) are outside it.
TABLES = {
    "control": {
        "control_registry_meta": (
            "singleton schema_version registry_id development revision",
            "SELECT UPDATE",
        ),
        "control_robot_slots": (
            "robot_id revision kind host_id arm_id",
            "SELECT UPDATE",
        ),
        "control_robot_bindings": (
            "binding_id robot_id device_id ip role ros_domain_id calibration desired_revision excluded created_at closed_at",
            "SELECT INSERT UPDATE",
        ),
        "control_registry_changes": (
            "request_id robot_id action payload result actor created_at",
            "SELECT INSERT",
        ),
        "control_operational_state": ("singleton data updated_at", "SELECT UPDATE"),
        "control_service_tasks": (
            "task_id site_id order_id reservation_id request state revision progress updated_at dispatch_locked",
            "SELECT INSERT UPDATE",
        ),
        "control_service_inbox": ("message_id message task_id", "SELECT INSERT"),
        "control_service_outbox": (
            "message_id message acknowledged_at created_at seq",
            "SELECT INSERT UPDATE",
        ),
        "control_service_receipts": (
            "task_id receipt_id message recorded_at arrival_verified",
            "SELECT INSERT",
        ),
        "control_service_quarantine": (
            "site_id message_id digest reason recorded_at",
            "SELECT INSERT",
        ),
    },
    "service": {
        "members": ("member_id login_id display_name", "SELECT"),
        "orders": (
            "order_id order_no member_id site_id revision business_status ordered_at",
            "SELECT UPDATE",
        ),
        "order_items": ("item_id order_id product_name quantity", "SELECT"),
        "pickup_locations": (
            "location_id site_id name control_point_ref is_active",
            "SELECT",
        ),
        "flights": (
            "flight_id flight_no service_date boarding_start boarding_end departure_at gate_code revision",
            "SELECT",
        ),
        "packages": (
            "package_id order_id package_revision manifest_locked_at inactive_at",
            "SELECT",
        ),
        "package_markers": ("package_id marker empty_box_returned_at", "SELECT"),
        "pickups": (
            "pickup_id reservation_no order_id location_id flight_id flight_revision flight_snapshot pickup_at channel status revision created_at updated_at",
            "SELECT INSERT UPDATE",
        ),
        "qr_credentials": (
            "credential_id token_hash pickup_id issued_at expires_at revoked_at updated_at",
            "SELECT INSERT UPDATE",
        ),
        "verifications": (
            "verification_id verification_request_id pickup_id method result verified_at actor_ref credential_id reason_code",
            "SELECT INSERT",
        ),
        "handover_approvals": (
            "approval_id pickup_id order_id verification_id task_ref cart_ref order_revision manifest_hash expires_at revoked_at consumed_at",
            "SELECT INSERT UPDATE",
        ),
        "receipts": (
            "receipt_id order_id pickup_id approval_id confirmation_request_id confirmed_by channel received_at order_revision follow_card",
            "SELECT INSERT",
        ),
        "receipt_packages": (
            "receipt_id order_id package_id package_revision",
            "SELECT INSERT",
        ),
        "control_task_views": (
            "task_ref pickup_id site_id task_revision state progress_payload observed_at synced_at",
            "SELECT INSERT UPDATE",
        ),
        "control_link_inbox": (
            "message_id site_id message received_at",
            "SELECT INSERT",
        ),
        "control_link_outbox": (
            "message_id site_id source_ref message acknowledged_at created_at",
            "SELECT INSERT UPDATE",
        ),
        "flyway_schema_history": ("installed_rank version success", "SELECT"),
    },
}

# The unique identities used for replay and ON CONFLICT must be real DB keys.
UNIQUE = {
    "control_registry_meta": ["singleton"],
    "control_robot_slots": ["robot_id"],
    "control_robot_bindings": ["binding_id"],
    "control_registry_changes": ["request_id"],
    "control_operational_state": ["singleton"],
    "control_service_tasks": ["task_id", "reservation_id"],
    "control_service_inbox": ["message_id"],
    "control_service_outbox": ["message_id", "seq"],
    "control_service_receipts": ["receipt_id"],
    "control_service_quarantine": ["site_id message_id digest"],
    "pickups": ["pickup_id", "reservation_no"],
    "verifications": ["verification_id", "verification_request_id"],
    "receipts": ["receipt_id", "confirmation_request_id", "approval_id"],
    "control_task_views": ["task_ref"],
    "control_link_inbox": ["message_id"],
    "control_link_outbox": ["message_id", "source_ref"],
}
TYPES = {
    "control_operational_state.data": "jsonb",
    "control_service_tasks.request": "jsonb",
    "control_service_tasks.progress": "jsonb",
    "control_service_tasks.dispatch_locked": "bool",
    "control_service_outbox.seq": "int8",
    "control_service_receipts.arrival_verified": "bool",
    "control_robot_bindings.ip": "inet",
    "control_task_views.progress_payload": "jsonb",
    "receipts.follow_card": "int4",
}


def connection_settings(app, env):
    name = "ROPORT_DATABASE_URL" if app == "control" else "SERVICE_DB_URL"
    dsn = env.get(name, "")
    if not dsn:
        raise ValueError(f"{name} is required")
    if app == "service":
        if not dsn.startswith("jdbc:postgresql://"):
            raise ValueError("SERVICE_DB_URL must be a PostgreSQL JDBC URL")
        dsn = dsn.removeprefix("jdbc:")
    try:
        settings = conninfo_to_dict(dsn)
    except psycopg.Error:
        raise ValueError("invalid PostgreSQL connection settings") from None
    if app == "service":
        # Match application.properties rather than inheriting the shell's PGUSER.
        settings["user"] = env.get("SERVICE_DB_USER", "postgres")
        settings["password"] = env.get("SERVICE_DB_PASSWORD", "")
    settings.update(
        connect_timeout=5,
        options=settings.get("options", "")
        + " -c default_transaction_read_only=on -c statement_timeout=5000",
    )
    return settings


def inspect_database(conn, app):
    # Set before the first query: even future accidental DML fails in this check.
    conn.read_only = True
    info = conn.execute(
        "SELECT current_setting('transaction_read_only') AS read_only, "
        "current_setting('server_version_num')::int AS version, "
        "has_database_privilege(current_user,current_database(),'CONNECT') AS connect"
    ).fetchone()
    if info["read_only"] != "on":
        raise ValueError("read-only transaction was not established")
    issues, checked = [], {}
    if not info["connect"]:
        issues.append("database CONNECT privilege missing")
    for table, (required, privileges) in TABLES[app].items():
        relation = conn.execute(
            "SELECT c.oid, c.relrowsecurity AS rls, "
            "has_schema_privilege(current_user,n.oid,'USAGE') AS schema_usage "
            "FROM pg_class c JOIN pg_namespace n ON n.oid=c.relnamespace "
            "WHERE c.oid=to_regclass(%s)",
            (table,),
        ).fetchone()
        if not relation:
            issues.append(f"{table}: missing table in app search_path")
            checked[table] = {"exists": False}
            continue
        columns = {
            row["name"]: row["type"]
            for row in conn.execute(
                "SELECT a.attname AS name,t.typname AS type FROM pg_attribute a "
                "JOIN pg_type t ON t.oid=a.atttypid "
                "WHERE a.attrelid=%s AND a.attnum>0 AND NOT a.attisdropped",
                (relation["oid"],),
            ).fetchall()
        }
        missing = sorted(set(required.split()) - columns.keys())
        if missing:
            issues.append(f"{table}: missing columns {', '.join(missing)}")
        for column, actual in columns.items():
            expected = TYPES.get(f"{table}.{column}")
            if expected and actual != expected:
                issues.append(f"{table}.{column}: expected {expected}, found {actual}")
        grants = {}
        # PostgreSQL's comma-separated privilege argument means ANY, not ALL.
        for privilege in privileges.split():
            grants[privilege] = conn.execute(
                "SELECT has_table_privilege(current_user,%s,%s) AS granted",
                (relation["oid"], privilege),
            ).fetchone()["granted"]
            if not grants[privilege]:
                issues.append(f"{table}: {privilege} privilege missing")
        if not relation["schema_usage"]:
            issues.append(f"{table}: schema USAGE privilege missing")
        if relation["rls"]:
            issues.append(f"{table}: row-level policy needs separate validation")
        keys = conn.execute(
            "SELECT ARRAY(SELECT a.attname FROM unnest(i.indkey) WITH ORDINALITY k(attnum,pos) "
            "JOIN pg_attribute a ON a.attrelid=i.indrelid AND a.attnum=k.attnum "
            "WHERE k.pos<=i.indnkeyatts ORDER BY k.pos) AS columns "
            "FROM pg_index i WHERE i.indrelid=%s AND i.indisunique "
            "AND i.indisvalid AND i.indpred IS NULL AND i.indexprs IS NULL",
            (relation["oid"],),
        ).fetchall()
        for key in UNIQUE.get(table, []):
            if not any(set(row["columns"]) == set(key.split()) for row in keys):
                issues.append(f"{table}: unique key ({key}) missing")
        checked[table] = {
            "exists": True,
            "missing_columns": missing,
            "privileges": grants,
        }
    if app == "control":
        if (
            checked["control_service_outbox"].get("exists")
            and "seq" not in checked["control_service_outbox"]["missing_columns"]
        ):
            sequence = conn.execute(
                "SELECT pg_get_serial_sequence('control_service_outbox','seq') AS name, "
                "(SELECT atthasdef OR attidentity<>'' FROM pg_attribute "
                "WHERE attrelid=to_regclass('control_service_outbox') "
                "AND attname='seq') AS default_present"
            ).fetchone()
            if (
                not sequence["name"]
                or not sequence["default_present"]
                or not conn.execute(
                    "SELECT has_sequence_privilege(current_user,%s,'USAGE,UPDATE') AS granted",
                    (sequence["name"],),
                ).fetchone()["granted"]
            ):
                issues.append(
                    "control_service_outbox.seq: sequence nextval privilege/default missing"
                )
    elif (
        checked["flyway_schema_history"].get("exists")
        and not checked["flyway_schema_history"]["missing_columns"]
        and checked["flyway_schema_history"]["privileges"]["SELECT"]
    ):
        migrations = conn.execute(
            "SELECT version,success FROM flyway_schema_history ORDER BY installed_rank"
        ).fetchall()
        done = {row["version"] for row in migrations if row["success"]}
        missing = set(map(str, range(1, 6))) - done
        if missing or any(not row["success"] for row in migrations):
            issues.append(
                "service Flyway V1–V5 not all applied successfully; migration review required"
            )
    return {
        "app": app,
        "read_only": True,
        "server_major": info["version"] // 10000,
        "compatible_for_checked_contract": not issues,
        "issues": issues,
        "tables": checked,
        "limits": [
            "No writes or migrations attempted; runtime write success is unverified.",
            "Partial indexes, triggers, Flyway checksums and existing business data need separate validation.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", required=True, choices=TABLES)
    args = parser.parse_args()
    try:
        settings = connection_settings(args.app, os.environ)
        with psycopg.connect(**settings, row_factory=dict_row) as conn:
            report = inspect_database(conn, args.app)
    except (psycopg.Error, ValueError):
        # libpq errors can include the DSN, username or password: never echo them.
        print(
            json.dumps(
                {
                    "app": args.app,
                    "error": "DB check failed; verify app environment, connectivity and SELECT access. No migrations or data changes attempted.",
                }
            )
        )
        return 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["compatible_for_checked_contract"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
