"""Real API/socket/registration writer in an isolated PostgreSQL test schema."""

import os
from contextlib import contextmanager
from uuid import uuid4

import psycopg
from fleet_web_fixture import owner as fleet_owner
from psycopg import sql
from psycopg.conninfo import make_conninfo

from roport_control_server.adapters.db.migrate import migrate
from roport_control_server.adapters.db.service_tasks import (
    OperationalStore,
    migrate_service,
)


@contextmanager
def owner(folder, mode="simulation"):
    dsn = os.environ["ROPORT_TEST_DATABASE_URL"]
    schema = "registry_web_" + uuid4().hex
    with psycopg.connect(dsn, autocommit=True) as admin:
        admin.execute(sql.SQL("CREATE SCHEMA {}").format(sql.Identifier(schema)))
        scoped = make_conninfo(dsn, options=f"-c search_path={schema}")
        try:
            migrate(scoped, development=True)
            migrate_service(scoped)
            store = OperationalStore(scoped)
            try:
                with fleet_owner(folder, mode, store=store, registry=True) as fixture:
                    yield fixture
            finally:
                store.close()
        finally:
            admin.execute(
                sql.SQL("DROP SCHEMA {} CASCADE").format(sql.Identifier(schema))
            )
