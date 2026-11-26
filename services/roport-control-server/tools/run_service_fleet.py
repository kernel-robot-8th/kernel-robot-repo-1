"""Customer web, pad and service link for virtual or explicitly configured field robots.

Each run owns two fresh PostgreSQL databases. Logs and database files survive exit.
The field mode uses an existing broker and external Pinky endpoints.
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
import urllib.request
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

from local_service import databases, free_port, java_service, seed_customer
from run_virtual_fleet import SERVICE, fetch, send

from roport_control_server.adapters.db.service_tasks import migrate_service


def field_settings(bundle, mock_omx, *, shelf_slot=None):
    from check_field import bundle_integrity, settings

    from roport_control_server.executor.fleet import TARGETS

    targets = (
        ["pinky1", "pinky2"] if mock_omx else [r for r in TARGETS if r != "pinky3"]
    )
    if shelf_slot:
        targets.remove("omx3")
    config, issues = settings(bundle, targets)
    if bundle_integrity(bundle)["damaged_files"]:
        issues.append("bundle files missing or modified")
    if config.get("initial_roles") != {
        "pinky1": "smartcart",
        "pinky2": "logistics",
        "pinky3": None,
    }:
        issues.append("use a two-cart field bundle")
    if issues:
        raise ValueError("; ".join(issues))
    if mock_omx:
        config["omx"] = {
            r: {"model_version": "sim-model", "calibration_version": "sim-cal"}
            for r in config["omx"]
        }
    if shelf_slot:
        config["omx"]["omx3"] = {
            "model_version": "sim-model",
            "calibration_version": "sim-cal",
        }
        config["trial_shelf_slots"] = [shelf_slot]
    config["service_policy"] = {
        "dispatch_lead_s": 86400
    }  # This isolated trial's reservation.
    return config


def start_field(
    start,
    root,
    bundle,
    config,
    port,
    broker_port,
    mock_omx,
    service_env,
    preview_env=None,
):
    folder = root / "fleet"
    folder.mkdir()
    key = secrets.token_urlsafe(24)
    (folder / "ui-key.txt").write_text(key + "\n")
    (folder / "fleet.json").write_text(json.dumps(config, indent=2))
    mqtt = {k: v for k, v in os.environ.items() if k.startswith("ROPORT_MQTT_")}
    mqtt.update(ROPORT_MQTT_HOST="127.0.0.1", ROPORT_MQTT_PORT=str(broker_port))
    shelf_trial = bool(config.get("trial_shelf_slots"))
    if mock_omx or shelf_trial:
        from dataclasses import replace

        from check_field import listen

        from roport_control_server.adapters.mqtt.trial_wire import mqtt_config

        targets = ["omx3"] if shelf_trial else list(config["omx"])
        connection = replace(
            mqtt_config("probe-" + secrets.token_hex(6)),
            host="127.0.0.1",
            port=broker_port,
        )
        report = listen(config, targets, connection, 2)
        occupied = [t for t, state in report["targets"].items() if state.get("samples")]
        if occupied or report["rejected_states"]:
            raise ValueError(
                f"mock OMX target already publishing or unexpected state: {occupied}"
            )
        omx = SERVICE.parent / "roport-omx"
        start(
            "mock-omx",
            [
                sys.executable,
                str(omx / "scripts/field_host.py"),
                "--mock",
                "--control",
                config["control_id"],
                "--journal",
                str(folder / "omx.json"),
                "--package-size",
                "large" if shelf_trial else "small",
                *(["--mock-targets", "omx3"] if shelf_trial else []),
            ],
            SERVICE,
            {**mqtt, "PYTHONPATH": str(omx / "src")},
        )
    sock = str(folder / "f")
    start(
        "owner",
        [
            sys.executable,
            "tools/run_field_owner.py",
            "--config",
            str(bundle),
            "--socket",
            sock,
            "--hardware-authorized",
            "--shelf-clear-confirmed",
            *(["--real-shelf", config["trial_shelf_slots"][0]] if shelf_trial else []),
            *(["--mock-omx"] if mock_omx else []),
        ],
        SERVICE,
        {**mqtt, **service_env},
    )
    map_file = next(
        p for p in (bundle / "maps").glob("*.yaml") if p.name != "layout.yaml"
    )
    start(
        "api",
        [
            sys.executable,
            "-m",
            "uvicorn",
            "roport_control_server.adapters.api.app:app",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        SERVICE,
        {
            "ROPORT_FLEET_SOCKET": sock,
            "ROPORT_TRIAL_TOKEN": key,
            "ROPORT_MAP": str(map_file),
            "ROPORT_MAP_LAYOUT": str(bundle / "maps/layout.yaml"),
            **(preview_env or {}),
        },
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--registry",
        action="store_true",
        help="start the virtual trial with empty shared robot slots",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--port", type=int, default=8000, help="control web port")
    parser.add_argument("--pg-bin", type=Path, default=os.environ.get("PG_BIN"))
    parser.add_argument(
        "--field-bundle",
        type=Path,
        help="verified two-cart field bundle",
    )
    parser.add_argument("--hardware-authorized", action="store_true")
    parser.add_argument(
        "--shelf-clear-confirmed",
        action="store_true",
        help="operator confirmed the trial shelf slots and arms are empty",
    )
    field_mode = parser.add_mutually_exclusive_group()
    field_mode.add_argument(
        "--real-shelf",
        choices=("1b", "2b"),
        nargs="?",
        const="2b",
        metavar="SLOT",
        help="cooperative B shelf (default: upper 2b); simulate only OMX3",
    )
    field_mode.add_argument(
        "--mock-omx",
        action="store_true",
        help="use only on a dedicated trial broker without physical OMX",
    )
    parser.add_argument(
        "--broker-port", type=int, help="existing trial broker on this control PC"
    )
    args = parser.parse_args()
    field = args.field_bundle is not None
    if field and args.registry:
        parser.error("--registry here is for the isolated virtual trial only")
    if field != args.hardware_authorized or (
        not field and (args.mock_omx or args.real_shelf or args.broker_port)
    ):
        parser.error(
            "field mode requires --field-bundle, --hardware-authorized and --broker-port together"
        )
    if field:
        if not args.shelf_clear_confirmed:
            parser.error(
                "check that trial slots and arms are empty, then pass --shelf-clear-confirmed"
            )
        if not args.broker_port or not 1 <= args.broker_port <= 65535:
            parser.error("specify the dedicated field broker port")
        args.field_bundle = args.field_bundle.resolve()
        try:
            field_config = field_settings(
                args.field_bundle, args.mock_omx, shelf_slot=args.real_shelf
            )
        except (KeyError, ValueError, OSError) as exc:
            parser.error(str(exc))
    if not args.pg_bin or not (args.pg_bin / "initdb").is_file():
        parser.error("set PG_BIN to the PostgreSQL bin directory")
    if not field and not (os.environ.get("MOSQUITTO_BIN") or shutil.which("mosquitto")):
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
        if k == "ROPORT_BOARDING_ALERT_TEST"
        or not k.startswith(("ROPORT_", "SERVICE_", "SERVER_", "VITE_"))
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
        preview_token = secrets.token_urlsafe(32)
        service_url, token = stack.enter_context(
            java_service(
                pg,
                root,
                preview_url=f"http://127.0.0.1:{args.port}",
                preview_token=preview_token,
            )
        )
        dsn = f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/service_test"
        accounts, reservations, stock = [], [], []
        if not args.real_shelf:
            accounts.append("e2e-login")
            stock.append(("package-small-1", "15"))
            reservations.append(seed_customer(dsn, "package-small-1", marker="15"))
        if not field or args.real_shelf:
            accounts.append("large-login")
            marker = "16" if args.real_shelf else "17"
            stock.append(("package-large-1", marker))
            reservations.append(
                seed_customer(dsn, "package-large-1", prefix="large", marker=marker)
            )
        migrate_service(f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/control_test")
        if args.registry:
            from roport_control_server.adapters.db.migrate import migrate

            migrate(
                f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/control_test",
                development=True,
            )
        connection = root / "service-env.json"
        connection.write_text(
            json.dumps(
                {
                    "ROPORT_DATABASE_URL": f"postgresql://{getpass.getuser()}@127.0.0.1:{pg}/control_test",
                    "ROPORT_SERVICE_WSS_URL": service_url.replace("http://", "ws://")
                    + "/ws/v1/sites/E2E/control",
                    "ROPORT_SERVICE_TOKEN": token,
                    "ROPORT_SERVICE_SITE": "E2E",
                    "ROPORT_FOLLOW_PREVIEW_TOKEN": preview_token,
                }
            )
        )
        stack.callback(stop_processes)
        if field:
            start_field(
                start,
                root,
                args.field_bundle,
                field_config,
                args.port,
                args.broker_port,
                args.mock_omx,
                json.loads(connection.read_text()),
                {
                    "ROPORT_FOLLOW_PREVIEW_TOKEN": preview_token,
                    **{
                        k: os.environ[k]
                        for k in (
                            "ROPORT_FOLLOW_FRAME_SOURCES",
                            "ROPORT_FOLLOW_FRAME_TOKEN",
                            "ROPORT_FOLLOW_FRAME_CA_FILE",
                        )
                        if k in os.environ
                    },
                },
            )
        else:
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
                    *(["--registry"] if args.registry else []),
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
                {
                    "ROPORT_SERVICE_HTTP_URL": service_url,
                    "VITE_PAD_API": "server",
                    "VITE_PAD_CART_ID": "pinky1" if field else "pinky3",
                    "VITE_PAD_FOLLOW": "1"
                    if not field
                    or field_config.get("verified_follow_robot") == "pinky1"
                    else "0",
                },
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
            "accounts": accounts,
            "reservations": reservations,
            "simulation": not field,
            "registry": args.registry,
            "real_shelf_slot": args.real_shelf,
            "mock_omx": args.mock_omx if field else True,
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
                    and (field or len(state.get("robots", {})) == 7)
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
        for package, marker in stock:
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
        if args.registry:
            print(
                "공유 등록 시험: Pinky 설정에서 기체와 역할을 등록한 뒤 요청 처리를 시작하세요."
            )
        if field:
            omx_label = "모의 OMX" if args.mock_omx else "실물 OMX"
            if args.real_shelf:
                omx_label = f"OMX3 모의 / 선반 양팔 실물 {args.real_shelf}"
            print(
                f"실물 시험 · {omx_label} · 시작 전 pinky2/물류/N17, pinky1/스마트카트/H1의 기체 관측을 확인하세요."
            )
        print(
            f"고객 웹: {urls['service']}\n계정: {', '.join(accounts)}\n카트 패드: {urls['pad']}\n관제: {control_url}/ui/?mode=fleet\n기록: {root}\n시험 입고 {len(stock)}건이 대기 중입니다. 관제 화면에서 조작 키를 입력하고 ‘요청 처리 시작’을 누르세요.\n선반 보관 완료 후 고객 웹에서 예약하세요.\n시험에서는 예약 시각을 기다리지 않고 출고합니다. 종료: Ctrl+C\n조작 키: {key}",
            flush=True,
        )
        while True:
            for name, p in processes.copy():
                if p.poll() is not None:
                    if field and name in {"service-web", "pad-web"}:
                        print(
                            f"{name} stopped; Fleet owner remains active",
                            file=sys.stderr,
                            flush=True,
                        )
                        processes.remove((name, p))
                        continue
                    raise RuntimeError(f"{name} stopped: see {root / (name + '.log')}")
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
