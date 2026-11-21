"""Customer web + pad + service WSS + Fleet + MQTT robots, all on loopback.

Each run owns two fresh PostgreSQL databases. Logs and database files survive exit.
Only sensor/motor/Nav2 I/O and OMX motion are virtual. Ctrl+C stops this run only.
"""

import argparse
import getpass
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

from local_service import databases, free_port, java_service, seed_customer
from run_virtual_fleet import SERVICE, fetch, send

from roport_control_server.adapters.db.service_tasks import migrate_service


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--port", type=int, default=8000, help="control web port")
    parser.add_argument("--pg-bin", type=Path, default=os.environ.get("PG_BIN"))
    args = parser.parse_args()
    if not args.pg_bin or not (args.pg_bin / "initdb").is_file():
        parser.error("set PG_BIN to the PostgreSQL bin directory")
    if not (os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")):
        parser.error("Mosquitto required")
    os.umask(0o077)
    root = (
        args.output
        or Path.home()
        / "roport-trial/service"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    ).resolve()
    root.mkdir(parents=True, exist_ok=False)
    web = SERVICE.parent / "roport-web"
    env = {
        k: v
        for k, v in os.environ.items()
        if not k.startswith(("ROPORT_", "SERVICE_", "SERVER_", "VITE_"))
    }
    processes = []

    def interrupted(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    with ExitStack() as stack:

        def stop_processes():
            for _, process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(20)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()

        def start(name, command, cwd, extra=None):
            log = stack.enter_context((root / (name + ".log")).open("w"))
            process = subprocess.Popen(
                command,
                cwd=cwd,
                env={**env, **(extra or {})},
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
            processes.append((name, process))

        # Registered after the DB/service contexts so clients stop before their servers.
        pg = stack.enter_context(databases(args.pg_bin, root, keep=True))
        service_url, token = stack.enter_context(java_service(pg, root))
        dsn = f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/service_test"
        small = seed_customer(dsn, "package-small-1", marker="15")
        large = seed_customer(dsn, "package-large-1", prefix="large", marker="17")
        migrate_service(f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/control_test")
        connection = root / "service-env.json"
        connection.write_text(
            json.dumps(
                {
                    "ROPORT_DATABASE_URL": f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/control_test",
                    "ROPORT_SERVICE_WSS_URL": service_url.replace("http://", "ws://")
                    + "/ws/v1/sites/E2E/control",
                    "ROPORT_SERVICE_TOKEN": token,
                    "ROPORT_SERVICE_SITE": "E2E",
                }
            )
        )
        stack.callback(stop_processes)
        start(
            "fleet",
            [
                sys.executable,
                "tools/run_virtual_fleet.py",
                "--output",
                str(root / "fleet"),
                "--port",
                str(args.port),
                "--service-env",
                str(connection),
            ],
            SERVICE,
        )
        urls = {}
        for app in ("service", "pad"):
            port = free_port()
            urls[app] = f"http://127.0.0.1:{port}"
            start(
                app + "-web",
                [
                    "npm",
                    "run",
                    "dev",
                    "--workspace",
                    "@roport/" + app,
                    "--",
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(port),
                    "--strictPort",
                ],
                web,
                {"ROPORT_SERVICE_HTTP_URL": service_url, "VITE_PAD_API": "server"},
            )
        control_url = f"http://127.0.0.1:{args.port}"
        info = {
            "service_url": service_url,
            "customer_url": urls["service"],
            "pad_url": urls["pad"],
            "control_url": control_url,
            "key_file": str(root / "fleet/ui-key.txt"),
            "run_dir": str(root),
            "postgres_port": pg,
            "accounts": ["e2e-login", "large-login"],
            "reservations": [small, large],
            "simulation": True,
        }
        (root / "run.json").write_text(json.dumps(info, indent=2))
        deadline = time.monotonic() + 45
        while True:
            for name, p in processes:
                if p.poll() is not None:
                    raise RuntimeError(f"{name} stopped: see {root / (name + '.log')}")
            try:
                state = fetch(control_url)
                ready = (
                    state.get("available")
                    and len(state.get("robots", {})) == 7
                    and not state.get("fault")
                )
                for url in urls.values():
                    with urllib.request.urlopen(url, timeout=1):
                        pass
                if ready:
                    break
            except (OSError, ValueError):
                pass
            if time.monotonic() > deadline:
                raise RuntimeError(f"UI startup timeout: {root}")
            time.sleep(0.2)
        key = Path(info["key_file"]).read_text().strip()
        # Packing fixture ends at a real inbound request; never seed shelf/task completion.
        for package, marker in (("package-small-1", "15"), ("package-large-1", "17")):
            send(
                control_url,
                key,
                state,
                "submit",
                request={
                    "request_id": "stock-" + package,
                    "operation": "inbound",
                    "package_id": package,
                    "marker": marker,
                    "goal": None,
                },
            )
        print(
            f"고객 웹: {urls['service']}\n계정: e2e-login (소형), large-login (대형)\n카트 패드: {urls['pad']}\n관제: {control_url}/ui/?mode=fleet\n기록: {root}\n시험 입고 2건이 대기 중입니다. 관제 화면에서 조작 키를 입력하고 ‘요청 처리 시작’을 누르세요.\n선반 보관 완료 후 고객 웹에서 예약하세요.\n시험에서는 예약 시각을 기다리지 않고 출고합니다. 종료: Ctrl+C\n조작 키: {key}",
            flush=True,
        )
        while True:
            for name, p in processes:
                if p.poll() is not None:
                    raise RuntimeError(f"{name} stopped: see {root / (name + '.log')}")
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
