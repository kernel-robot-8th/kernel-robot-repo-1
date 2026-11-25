"""DB readiness inspection never seeds data, migrates or prints credentials."""

import json
import sys
from pathlib import Path

import psycopg
import pytest
from psycopg.rows import dict_row

from roport_control_server.adapters.db.service_tasks import migrate_service

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import check_database as check


class Rows:
    def __init__(self, *rows):
        self.rows = list(rows)

    def fetchone(self):
        return self.rows[0] if self.rows else None

    def fetchall(self):
        return self.rows


class Database:
    def __init__(self, app):
        self.app = app
        self.read_only = False
        self.queries = []
        self.missing = set()
        self.denied = set()
        self.columns = {}
        self.keys = {}
        self.versions = list(map(str, range(1, 6)))
        self.sequence_usage = True
        self.sequence_default = True

    def execute(self, query, params=()):
        assert self.read_only, "first statement must already be read-only"
        assert query.startswith("SELECT "), "inspection must not use DDL/DML"
        self.queries.append((query, params))
        if "current_setting('transaction_read_only')" in query:
            return Rows({"read_only": "on", "version": 160015, "connect": True})
        if "FROM pg_class" in query:
            return (
                Rows()
                if params[0] in self.missing
                else Rows({"oid": params[0], "rls": False, "schema_usage": True})
            )
        if "pg_get_serial_sequence" in query:
            return Rows(
                {
                    "name": "public.control_service_outbox_seq_seq",
                    "default_present": self.sequence_default,
                }
            )
        if "FROM pg_attribute" in query:
            table = params[0]
            columns = self.columns.get(table, check.TABLES[self.app][table][0].split())
            return Rows(
                *[
                    {"name": name, "type": check.TYPES.get(f"{table}.{name}", "text")}
                    for name in columns
                ]
            )
        if "has_table_privilege" in query:
            return Rows({"granted": tuple(params) not in self.denied})
        if "FROM pg_index" in query:
            keys = self.keys.get(params[0], check.UNIQUE.get(params[0], []))
            return Rows(*[{"columns": key.split()} for key in keys])
        if "has_sequence_privilege" in query:
            return Rows({"granted": self.sequence_usage})
        if "FROM flyway_schema_history" in query:
            return Rows(
                *[{"version": version, "success": True} for version in self.versions]
            )
        raise AssertionError(query)


@pytest.mark.parametrize("app", ["control", "service"])
def test_inspection_is_read_only_and_checks_the_existing_contract(app):
    db = Database(app)
    result = check.inspect_database(db, app)
    assert result["read_only"] and result["compatible_for_checked_contract"]
    assert result["issues"] == []
    assert set(result["tables"]) == set(check.TABLES[app])
    assert "No writes" in result["limits"][0]


def test_old_service_schema_identifies_follow_column_and_v5_gap():
    db = Database("service")
    db.columns["receipts"] = check.TABLES["service"]["receipts"][0].split()[:-1]
    db.versions.remove("5")
    result = check.inspect_database(db, "service")
    assert not result["compatible_for_checked_contract"]
    assert "receipts: missing columns follow_card" in result["issues"]
    assert any("V1–V5" in issue for issue in result["issues"])


def test_verification_replay_requires_the_on_conflict_unique_key():
    db = Database("service")
    db.keys["verifications"] = ["verification_id"]
    result = check.inspect_database(db, "service")
    assert not result["compatible_for_checked_contract"]
    assert (
        "verifications: unique key (verification_request_id) missing"
        in result["issues"]
    )


def test_each_dml_privilege_is_checked_separately():
    db = Database("control")
    db.denied.add(("control_service_outbox", "INSERT"))
    result = check.inspect_database(db, "control")
    assert result["tables"]["control_service_outbox"]["privileges"] == {
        "SELECT": True,
        "INSERT": False,
        "UPDATE": True,
    }
    assert "control_service_outbox: INSERT privilege missing" in result["issues"]
    assert all(
        "," not in params[1]
        for query, params in db.queries
        if "has_table_privilege" in query
    )


def test_missing_table_does_not_try_to_read_its_rows_or_columns():
    db = Database("control")
    db.missing.add("control_service_outbox")
    result = check.inspect_database(db, "control")
    assert result["tables"]["control_service_outbox"] == {"exists": False}
    assert not any("pg_get_serial_sequence" in query for query, _ in db.queries)


def test_sequence_grant_and_replay_key_are_not_assumed_from_table_grants():
    db = Database("control")
    db.sequence_usage = False
    db.keys["control_service_inbox"] = []
    result = check.inspect_database(db, "control")
    assert any("sequence nextval" in issue for issue in result["issues"])
    assert "control_service_inbox: unique key (message_id) missing" in result["issues"]


def test_jdbc_app_settings_and_existing_search_path_are_preserved():
    settings = check.connection_settings(
        "service",
        {
            "SERVICE_DB_URL": "jdbc:postgresql://127.0.0.1/example_test?options=-c%20search_path%3Dexample",
            "SERVICE_DB_USER": "example_app",
            "SERVICE_DB_PASSWORD": "REPLACE_ME",
        },
    )
    assert settings["user"] == "example_app" and settings["password"] == "REPLACE_ME"
    assert settings["dbname"] == "example_test" and settings["connect_timeout"] == 5
    assert settings["options"].startswith("-c search_path=example ")
    assert "default_transaction_read_only=on" in settings["options"]


def test_existing_sequence_without_a_column_default_is_not_ready():
    db = Database("control")
    db.sequence_default = False
    result = check.inspect_database(db, "control")
    assert not result["compatible_for_checked_contract"]
    assert any("sequence nextval" in issue for issue in result["issues"])


def test_service_defaults_match_spring_instead_of_shell_pguser():
    settings = check.connection_settings(
        "service",
        {
            "SERVICE_DB_URL": "jdbc:postgresql://127.0.0.1/example_test",
            "PGUSER": "other_account",
            "PGPASSWORD": "REPLACE_ME",
        },
    )
    assert settings["user"] == "postgres" and settings["password"] == ""


@pytest.mark.parametrize(
    "env", [{}, {"ROPORT_DATABASE_URL": "invalid conninfo REPLACE_ME"}]
)
def test_invalid_configuration_is_rejected_without_connecting(env):
    with pytest.raises(ValueError) as error:
        check.connection_settings("control", env)
    assert "REPLACE_ME" not in str(error.value)


def test_cli_connection_failure_never_prints_libpq_details(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["check_database.py", "--app", "control"])
    monkeypatch.setenv(
        "ROPORT_DATABASE_URL", "postgresql://example_app@127.0.0.1/example_test"
    )

    def fail(**settings):
        assert "default_transaction_read_only=on" in settings["options"]
        raise psycopg.OperationalError("sensitive connection detail DO_NOT_PRINT")

    monkeypatch.setattr(check.psycopg, "connect", fail)
    assert check.main() == 2
    output = capsys.readouterr()
    assert "DO_NOT_PRINT" not in output.out + output.err
    assert "postgresql://" not in output.out + output.err
    assert "error" in json.loads(output.out)


def test_real_control_schema_in_an_isolated_test_namespace(registry_db):
    _, dsn = registry_db
    migrate_service(dsn)
    with psycopg.connect(
        **check.connection_settings("control", {"ROPORT_DATABASE_URL": dsn}),
        row_factory=dict_row,
    ) as conn:
        result = check.inspect_database(conn, "control")
        assert result["compatible_for_checked_contract"], result["issues"]
        assert conn.execute("SHOW transaction_read_only").fetchone() == {
            "transaction_read_only": "on"
        }
