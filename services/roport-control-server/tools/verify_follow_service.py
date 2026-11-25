"""Explicit loopback service/WSS/DB/MQTT follow acceptance test; never uses hardware.

Creates fresh databases and runs the current Java service and virtual Fleet launcher.
Only customer/catalog data is seeded. Arrival, registration, stopping and return
must come from actual MQTT endpoints and the control owner, never DB state edits.
"""

import argparse
import getpass
import json
import os
import secrets
import shutil
import signal
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from contextlib import ExitStack
from http.cookiejar import CookieJar
from pathlib import Path
from uuid import uuid4

import psycopg
from local_service import databases, free_port, java_service, seed_customer
from run_virtual_fleet import SERVICE, fetch, send

from roport_control_server.adapters.db.service_tasks import migrate_service


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=str) + "\n")


def exercise(
    output, pg_port, service_url, service_token, preview_token, port, timeout, mosquitto
):
    service_dsn = f"postgresql://{getpass.getuser()}@127.0.0.1:{pg_port}/service_test"
    control_dsn = f"postgresql://{getpass.getuser()}@127.0.0.1:{pg_port}/control_test"
    migrate_service(control_dsn)
    customer_request = seed_customer(service_dsn, "package-small-1", marker="15")
    connection = output / "service-env.json"
    write_json(
        connection,
        {
            "ROPORT_DATABASE_URL": control_dsn,
            "ROPORT_SERVICE_WSS_URL": service_url.replace("http://", "ws://")
            + "/ws/v1/sites/E2E/control",
            "ROPORT_SERVICE_TOKEN": service_token,
            "ROPORT_SERVICE_SITE": "E2E",
            "ROPORT_FOLLOW_PREVIEW_TOKEN": preview_token,
        },
    )
    control_url = f"http://127.0.0.1:{port}"
    env = {
        k: v
        for k, v in os.environ.items()
        if not k.startswith(("ROPORT_", "SERVICE_", "SERVER_"))
    }
    env["PINKY_SIM_TICKS_PER_FRAME"] = "1"
    env["ROPORT_BOARDING_ALERT_TEST"] = "1"
    evidence = {
        "hardware_connected": False,
        "simulation": True,
        "speed_limit_mps": 0.08,
        "steps": [],
    }
    customer = urllib.request.build_opener(
        urllib.request.HTTPCookieProcessor(CookieJar())
    )

    def http(path, body=None, *, cart=False, raw=False):
        headers = {"Content-Type": "application/json"}
        if cart:
            headers["X-Cart-Id"] = "pinky3"
        request = urllib.request.Request(
            service_url + path,
            data=None if body is None else json.dumps(body).encode(),
            headers=headers,
        )
        with customer.open(request, timeout=5) as response:
            if raw:
                return response.headers.get_content_type(), response.read()
            return json.load(response)

    def commands():
        journal = json.loads((output / "fleet/pinky3.json").read_text())
        return [r["command"] for r in journal["records"].values()]

    with (output / "fleet-launcher.log").open("w") as log:
        process = subprocess.Popen(
            [
                sys.executable,
                "tools/run_virtual_fleet.py",
                "--output",
                str(output / "fleet"),
                "--port",
                str(port),
                "--mosquitto",
                mosquitto,
                "--service-env",
                str(connection),
            ],
            cwd=SERVICE,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + timeout

        def wait(label, predicate, *, seconds=None):
            until = min(deadline, time.monotonic() + seconds) if seconds else deadline
            last = None
            while time.monotonic() < until:
                if process.poll() is not None:
                    raise RuntimeError(
                        "virtual launcher exited; see fleet-launcher.log"
                    )
                try:
                    last = fetch(control_url)
                except (OSError, ValueError):
                    time.sleep(0.2)
                    continue
                if last.get("fault"):
                    raise RuntimeError(f"Fleet HOLD during {label}: {last['fault']}")
                result = predicate(last)
                if result:
                    evidence["steps"].append(label)
                    print(label, flush=True)
                    return result
                time.sleep(0.2)
            write_json(output / "timeout-snapshot.json", last)
            raise TimeoutError(label)

        def stable_stop(stage, receipt, seconds=2):
            before = [c["command_id"] for c in commands()]
            first_pose = fetch(control_url)["poses"]["pinky3"]["pose"]
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                state = fetch(control_url)
                assert state["robots"]["pinky3"]["state"]["stopped"] is True
                assert state["poses"]["pinky3"]["pose"] == first_pose
                assert [c["command_id"] for c in commands()] == before
                fresh = http("/cart-api/follow/" + receipt, cart=True)
                assert fresh["stage"] == stage and not fresh["stale"], fresh
                time.sleep(0.2)
            evidence["steps"].append(
                f"{stage}: repeated GET caused no motion or new MQTT command"
            )

        try:
            state = wait(
                "seven virtual endpoints ready",
                lambda s: (
                    s if s.get("available") and len(s.get("robots", {})) == 7 else None
                ),
                seconds=45,
            )
            key = (output / "fleet/ui-key.txt").read_text().strip()
            assert state["enabled"] is False
            send(
                control_url,
                key,
                state,
                "submit",
                request={
                    "request_id": "follow-stock",
                    "operation": "inbound",
                    "package_id": "package-small-1",
                    "marker": "15",
                    "goal": None,
                },
            )
            assert not fetch(control_url)["commands"]
            send(control_url, key, state, "enable")
            wait(
                "inbound completed through MQTT",
                lambda s: (
                    s["tasks"].get("follow-stock", {}).get("status") == "COMPLETE"
                ),
            )

            http("/api/session", {"loginId": "e2e-login"})
            query = urllib.parse.urlencode(
                {k: customer_request[k] for k in ("flightId", "locationId")}
            )
            customer_request["pickupAt"] = http(
                "/api/orders/" + customer_request["orderId"] + "/pickup-times?" + query
            )["times"][0]
            reservation = http("/api/pickups", customer_request)
            child = wait(
                "customer reservation reached cart arrival",
                lambda s: next(
                    (
                        k
                        for k, t in s["tasks"].items()
                        if t["request"]["operation"] == "outbound"
                        and t["stage"] == "await_receipt"
                    ),
                    None,
                ),
            )
            verified = http(
                "/cart-api/verifications",
                {
                    "verificationRequestId": str(uuid4()),
                    "method": "RESERVATION_NO",
                    "value": reservation["reservationNo"],
                },
                cart=True,
            )
            assert verified["result"] == "VERIFIED"
            wait(
                "appearance capability reached service",
                lambda _: (
                    "appearance"
                    in http(
                        "/cart-api/follow/capabilities?verificationId="
                        + verified["verificationId"],
                        cart=True,
                    )["modes"]
                ),
            )
            confirmation = {
                "verificationId": verified["verificationId"],
                "confirmationRequestId": str(uuid4()),
                "followMode": "appearance",
            }
            receipt = http("/cart-api/receipts", confirmation, cart=True)["receiptId"]
            assert (
                http("/cart-api/receipts", confirmation, cart=True)["receiptId"]
                == receipt
            )
            follow_path = "/cart-api/follow/" + receipt

            def follow_where(predicate):
                value = http(follow_path, cart=True)
                return value if not value["stale"] and predicate(value) else None

            def action(operation, **extra):
                current = wait(
                    operation + " available",
                    lambda _: follow_where(lambda f: operation in f["allowedActions"]),
                )
                body = {
                    "requestId": str(uuid4()),
                    "expectedRevision": current["revision"],
                    "operation": operation,
                    **extra,
                }
                accepted = http(follow_path + "/actions", body, cart=True)
                assert accepted == {"accepted": True, "requestId": body["requestId"]}
                assert http(follow_path + "/actions", body, cart=True) == accepted
                return body["requestId"]

            wait(
                "appearance receipt waits for registration",
                lambda _: follow_where(lambda f: f["stage"] == "follow_register"),
            )
            stable_stop("follow_register", receipt)
            capture_id = action("capture")
            captured = wait(
                "capture result reached service",
                lambda _: follow_where(
                    lambda f: (
                        f.get("registration", {}).get("frameRef")
                        and "select" in f["allowedActions"]
                    )
                ),
            )
            registration = captured["registration"]
            media_type, image = http(registration["imagePath"], cart=True, raw=True)
            assert media_type in {"image/png", "image/jpeg"}
            assert image.startswith(
                b"\x89PNG\r\n\x1a\n" if media_type == "image/png" else b"\xff\xd8\xff"
            )
            evidence["frame"] = {"content_type": media_type, "bytes": len(image)}
            select_id = action(
                "select",
                frameRef=registration["frameRef"],
                candidateId=registration["candidates"][0]["candidateId"],
            )
            selected = wait(
                "selected target waits for operator approval",
                lambda _: follow_where(
                    lambda f: (
                        f["stage"] == "follow_pending"
                        and f["registration"]["registered"]
                    )
                ),
            )
            assert "targetRef" not in selected["registration"]
            assert "frameRef" not in selected["registration"]
            stable_stop("follow_pending", receipt)
            assert not [c for c in commands() if c["type"] == "follow_target_command"]
            send(
                control_url,
                key,
                fetch(control_url),
                "follow",
                request_id=child,
                operation="approve",
            )
            active = wait(
                "approved appearance follow is active",
                lambda _: follow_where(lambda f: f["stage"] == "following"),
            )
            assert "boardingAlert" not in active
            wait(
                "robot reports appearance follow active",
                lambda s: (
                    s["robots"]["pinky3"]["state"].get("follow", {}).get("active")
                ),
            )
            wait(
                "control boarding alert reached the service API",
                lambda _: follow_where(lambda f: f.get("boardingAlert") is True),
                seconds=30,
            )
            pause_id = action("pause")
            paused = wait(
                "pad pause receives stopped confirmation",
                lambda _: follow_where(lambda f: f["stage"] == "follow_paused"),
            )
            assert "boardingAlert" not in paused
            stable_stop("follow_paused", receipt)
            resume_id = action("resume")
            wait(
                "pad resume becomes following",
                lambda _: follow_where(lambda f: f["stage"] == "following"),
            )
            return_id = action("return")
            wait(
                "return completes parking through MQTT",
                lambda s: s["tasks"][child]["status"] == "COMPLETE",
            )
            finished = wait(
                "service receives final parking state",
                lambda _: follow_where(
                    lambda f: f["stage"] == "done" and f["status"] == "COMPLETED"
                ),
            )
            all_commands = commands()
            write_json(output / "cart-commands.json", all_commands)
            write_json(output / "control-snapshot.json", fetch(control_url))
            targets = [c for c in all_commands if c["type"] == "follow_target_command"]
            assert len(targets) == 2
            assert all(c["payload"]["follow_mode"] == "appearance" for c in targets)
            stops = [
                c
                for c in all_commands
                if c["type"] == "robot_stop_command"
                and c["payload"]["reason"] == "follow_pause"
            ]
            assert len(stops) == 2, (
                "pause and return each require one robot_stop_command"
            )
            registrations = [
                c for c in all_commands if c["type"] == "follow_registration_command"
            ]
            assert [c["payload"]["operation"] for c in registrations] == [
                "capture",
                "select",
            ]
            with psycopg.connect(service_dsn) as db:
                assert db.execute("SELECT count(*) FROM receipts").fetchone()[0] == 1
                assert (
                    db.execute(
                        "SELECT business_status FROM orders WHERE order_id='e2e-order'"
                    ).fetchone()[0]
                    == "RECEIVED"
                )
                for request in (capture_id, select_id, pause_id, resume_id, return_id):
                    assert (
                        db.execute(
                            "SELECT count(*) FROM control_link_outbox WHERE source_ref=%s",
                            ("follow:" + request,),
                        ).fetchone()[0]
                        == 1
                    )
            evidence.update(
                passed=True,
                final_follow=finished,
                receipt_id=receipt,
                command_types=[c["type"] for c in all_commands],
                checks={
                    "duplicate_receipts": False,
                    "duplicate_actions": False,
                    "approval_required": True,
                    "stop_confirmed_before_return": True,
                    "http_frame_relay": True,
                },
            )
            write_json(output / "result.json", evidence)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--pg-bin", required=True, type=Path)
    parser.add_argument(
        "--mosquitto",
        default=os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto"),
    )
    parser.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args()
    if not args.mosquitto or not (args.pg_bin / "initdb").is_file():
        parser.error("PostgreSQL bin directory and Mosquitto are required")
    if not 30 <= args.timeout <= 3600:
        parser.error("timeout must be between 30 and 3600 seconds")
    output = args.output.resolve()
    if len(str(output / "fleet/f").encode()) >= 100:
        parser.error("use a shorter output path for the Unix socket")
    os.umask(0o077)
    output.mkdir(parents=True, exist_ok=False)

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        with ExitStack() as stack:
            pg_port = stack.enter_context(databases(args.pg_bin, output, keep=True))
            port, preview_token = free_port(), secrets.token_urlsafe(32)
            service_url, service_token = stack.enter_context(
                java_service(
                    pg_port,
                    output,
                    preview_url=f"http://127.0.0.1:{port}",
                    preview_token=preview_token,
                )
            )
            exercise(
                output,
                pg_port,
                service_url,
                service_token,
                preview_token,
                port,
                args.timeout,
                args.mosquitto,
            )
        print("PASS: " + str(output / "result.json"), flush=True)
    except BaseException as exc:
        write_json(
            output / "failure.json", {"type": type(exc).__name__, "detail": str(exc)}
        )
        raise


if __name__ == "__main__":
    main()
