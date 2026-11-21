"""Isolated local service/control test databases and Java service; never uses an existing DB."""

import getpass
import os
import shutil
import socket
import subprocess
import tempfile
import time
import urllib.request
from contextlib import contextmanager, nullcontext
from datetime import UTC, datetime, timedelta
from pathlib import Path
from uuid import uuid4

import psycopg

REPO = Path(__file__).resolve().parents[3]


def free_port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


@contextmanager
def databases(pg_bin, output, *, keep=False):
    with (
        nullcontext(output / "database")
        if keep
        else tempfile.TemporaryDirectory(prefix="rp-customer-e2e-")
    ) as folder:
        root = Path(folder)
        root.mkdir(parents=True, exist_ok=True)
        port = free_port()
        subprocess.run(
            [
                str(pg_bin / "initdb"),
                "-D",
                str(root / "data"),
                "-A",
                "trust",
                "--no-locale",
                "--encoding=UTF8",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        subprocess.run(
            [
                str(pg_bin / "pg_ctl"),
                "-D",
                str(root / "data"),
                "-l",
                str(root / "postgres.log"),
                "-o",
                f"-k '' -h 127.0.0.1 -p {port}",
                "-w",
                "start",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        try:
            for name in ("service_test", "control_test"):
                subprocess.run(
                    [
                        str(pg_bin / "createdb"),
                        "-h",
                        "127.0.0.1",
                        "-p",
                        str(port),
                        name,
                    ],
                    check=True,
                )
            yield port
        finally:
            subprocess.run(
                [
                    str(pg_bin / "pg_ctl"),
                    "-D",
                    str(root / "data"),
                    "-w",
                    "stop",
                    "-m",
                    "fast",
                ],
                check=True,
                stdout=subprocess.DEVNULL,
            )
            shutil.copyfile(root / "postgres.log", output / "postgres.log")


@contextmanager
def java_service(pg_port, output):
    project = REPO / "services/roport-service-server"
    with (output / "java-build.log").open("w") as log:
        subprocess.run(
            ["./gradlew", "--no-daemon", "bootJar"],
            cwd=project,
            check=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    port, token = free_port(), uuid4().hex
    env = {
        k: v
        for k, v in os.environ.items()
        if not k.startswith(("SERVICE_", "SERVER_", "SPRING_"))
    }
    env.update(
        SERVICE_DB_URL=f"jdbc:postgresql://127.0.0.1:{pg_port}/service_test",
        SERVICE_DB_USER=getpass.getuser(),
        SERVICE_DB_PASSWORD="",
        SERVICE_CONTROL_ENABLED="true",
        SERVICE_CONTROL_SITE="E2E",
        SERVICE_CONTROL_TOKEN=token,
        SERVICE_DEV_LOGIN="true",
        SERVICE_DEV_CART="true",
        SERVER_ADDRESS="127.0.0.1",
        SERVER_PORT=str(port),
    )
    with (output / "java-service.log").open("w") as log:
        process = subprocess.Popen(
            [
                "java",
                "-jar",
                str(project / "build/libs/roport-service-server-0.1.0-SNAPSHOT.jar"),
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        url = f"http://127.0.0.1:{port}"
        try:
            deadline = time.monotonic() + 30
            while True:
                if process.poll() is not None:
                    raise RuntimeError("Java service exited; see java-service.log")
                try:
                    with urllib.request.urlopen(
                        url + "/actuator/health", timeout=1
                    ) as response:
                        if response.status == 200:
                            break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Java service startup timeout") from None
                    time.sleep(0.1)
            yield url, token
        finally:
            process.terminate()
            process.wait(10)


def seed_customer(dsn, package, *, prefix="e2e", marker=None):
    """Seed account/order/catalog only; shelf and task progress come from robot events."""
    now = datetime.now(UTC)
    pickup = datetime.fromtimestamp(
        ((int((now + timedelta(hours=1)).timestamp()) // 1800) + 1) * 1800, UTC
    )
    boarding = now + timedelta(hours=4)
    with psycopg.connect(dsn) as db:
        db.execute(
            "INSERT INTO members(member_id,login_id,display_name) VALUES(%s,%s,'시험 고객') ON CONFLICT DO NOTHING",
            (prefix + "-member", prefix + "-login"),
        )
        db.execute(
            "INSERT INTO orders(order_id,order_no,member_id,site_id,revision,business_status,ordered_at) VALUES(%s,%s,%s,'E2E',0,'CONFIRMED',now())",
            (prefix + "-order", prefix + "-order", prefix + "-member"),
        )
        db.execute(
            "INSERT INTO order_items(item_id,order_id,product_name,quantity) VALUES(%s,%s,'가상 시험 물품',1)",
            (prefix + "-item", prefix + "-order"),
        )
        db.execute(
            "INSERT INTO pickup_locations(location_id,site_id,name,control_point_ref,is_active) VALUES('e2e-location','E2E','공용공간 시험 위치','PUBLIC_SPACE_TRIAL',true) ON CONFLICT DO NOTHING"
        )
        db.execute(
            "INSERT INTO flights(flight_id,flight_no,service_date,boarding_start,boarding_end,departure_at,gate_code,revision) VALUES('e2e-flight','e2e-flight',CURRENT_DATE,%s,%s,%s,'G1',0) ON CONFLICT DO NOTHING",
            (boarding, boarding, boarding),
        )
        db.execute(
            "INSERT INTO packages(package_id,order_id,package_revision,label_ref,manifest_locked_at) VALUES(%s,%s,0,'test-only',now())",
            (package, prefix + "-order"),
        )
        if marker is not None:
            db.execute(
                "INSERT INTO package_markers(package_id,marker) VALUES(%s,%s)",
                (package, marker),
            )
    return {
        "orderId": prefix + "-order",
        "locationId": "e2e-location",
        "flightId": "e2e-flight",
        "pickupAt": pickup.isoformat(),
    }
