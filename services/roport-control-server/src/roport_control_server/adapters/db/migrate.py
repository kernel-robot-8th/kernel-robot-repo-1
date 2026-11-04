"""Explicit, atomic schema installation. API startup never runs migrations."""

import argparse
import os
from pathlib import Path
from uuid import uuid4

from .registry import RegistryStore


def migrate(dsn: str, *, development: bool = False):
    store = RegistryStore(dsn)
    with store.connection() as conn:
        conn.execute("SELECT pg_advisory_xact_lock(71519001)")
        exists = conn.execute(
            "SELECT to_regclass('control_registry_meta') AS name"
        ).fetchone()["name"]
        if exists:
            meta = store.metadata(conn)
            if meta["development"] != development:
                raise ValueError(
                    "existing database mode differs; use a separate database"
                )
            return
        conn.execute(Path(__file__).with_name("001_registry.sql").read_text())
        conn.execute(
            "INSERT INTO control_registry_meta (schema_version, registry_id, development) VALUES (1, %s, %s)",
            (uuid4(), development),
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--development",
        action="store_true",
        help="mark an isolated database for synthetic local writes",
    )
    args = parser.parse_args()
    dsn = os.environ.get("ROPORT_DATABASE_URL")
    if not dsn:
        parser.error("ROPORT_DATABASE_URL is required")
    try:
        migrate(dsn, development=args.development)
    except (ValueError, OSError):
        parser.exit(
            1,
            "Registry migration failed; verify connectivity, permissions and schema mode.\n",
        )
    print(f"Registry schema 1 ready (development={args.development}).")


if __name__ == "__main__":
    main()
