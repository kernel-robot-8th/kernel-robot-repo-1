"""Isolated PostgreSQL fixtures. Never drop or clear the caller's tables."""

import os
from uuid import uuid4

import psycopg
import pytest
from psycopg import sql
from psycopg.conninfo import make_conninfo

from roport_control_server.adapters.db.migrate import migrate
from roport_control_server.adapters.db.registry import RegistryStore


@pytest.fixture
def registry_db():
    dsn = os.environ.get("ROPORT_TEST_DATABASE_URL")
    if not dsn:
        if os.environ.get("REQUIRE_REGISTRY_TESTS") == "1":
            pytest.fail("ROPORT_TEST_DATABASE_URL required")
        pytest.skip("isolated PostgreSQL required (set REQUIRE_REGISTRY_TESTS=1 in CI)")
    schema = "registry_test_" + uuid4().hex
    with psycopg.connect(dsn, autocommit=True) as admin:
        admin.execute(sql.SQL("CREATE SCHEMA {}").format(sql.Identifier(schema)))
        scoped = make_conninfo(dsn, options=f"-c search_path={schema}")
        try:
            migrate(scoped, development=True)
            yield RegistryStore(scoped), scoped
        finally:
            admin.execute(
                sql.SQL("DROP SCHEMA {} CASCADE").format(sql.Identifier(schema))
            )


def registration(index=1, **changes):
    return {
        "request_id": str(uuid4()),
        "expected_revision": 0,
        "device_id": f"sim-unit-{index}",
        "config": {
            "ip": f"192.0.2.{10 + index}",
            "role": "logistics" if index != 3 else "smartcart",
            "ros_domain_id": index + 13,
            "calibration": f"pinky_000{index}",
        },
        **changes,
    }


def mutation(result, **changes):
    return {
        "request_id": str(uuid4()),
        "expected_revision": result["revision"],
        "binding_id": result["binding_id"],
        **changes,
    }
