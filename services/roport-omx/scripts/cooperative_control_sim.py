"""Test-only control sender for the shelf-01 Host: status → task → events → event.ack.

Stands in until the control server sends roport-omx-task-v1. Events are appended (fsync) to --log
before event.ack; that file is a test record, not the operational store. Moves nothing by itself:
with the simulated Host nothing moves at all.

  uv run python scripts/cooperative_control_sim.py --host BROKER_HOST inbound
"""

import argparse
import json
import os
import sys
from datetime import UTC, datetime, timedelta
from pathlib import Path
from queue import Empty
from time import monotonic
from uuid import uuid4

from roport_omx.cooperative import PROFILE, WORKCELL
from roport_omx.mqtt_link.messages import decode, topic
from roport_omx.mqtt_link.mqtt import MqttConfig, MqttTransport

FINAL = {"completed", "failed", "rejected"}


def envelope(args, kind: str, payload: dict, ids: dict) -> dict:
    now = datetime.now(UTC)
    return {
        "schema_version": "1.0",
        "profile": PROFILE,
        "message_id": str(uuid4()),
        "type": kind,
        "site_id": args.site,
        "source": args.control,
        "target": WORKCELL,
        "sent_at": now.isoformat(timespec="milliseconds"),
        **(
            {
                "expires_at": (now + timedelta(seconds=10)).isoformat(
                    timespec="milliseconds"
                )
            }
            if kind.endswith(".requested")
            else {}
        ),
        **ids,
        "payload": payload,
    }


def new_ids(step: str) -> dict:
    return {
        "task_id": "sim-task",
        "step_id": step,
        "command_id": str(uuid4()),
        "execution_id": str(uuid4()),
    }


def wait_events(args, transport, command: dict, log_file):
    """Yield events of this command until a final one; ack each after it is written."""
    deadline = monotonic() + args.timeout
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise TimeoutError(
                "no final event; result unknown, reconcile with Host status"
            )
        try:
            delivery = transport.receive(timeout=remaining)
        except Empty:
            continue
        event = decode(delivery.payload)
        transport.ack(delivery)
        if (
            delivery.retained
            or event["command_id"] != command["command_id"]
            or event["source"] != WORKCELL
        ):
            continue
        log_file.write(json.dumps(event, ensure_ascii=False) + "\n")
        log_file.flush()
        os.fsync(log_file.fileno())
        ack_ids = {
            k: event[k] for k in ("task_id", "step_id", "command_id", "execution_id")
        }
        ack = envelope(
            args,
            "event.ack",
            {"acked_message_id": event["message_id"], "status": "stored"},
            ack_ids,
        )
        transport.publish(topic(args.site, "workcell", WORKCELL, "ack"), ack)
        yield event
        if event["type"].rsplit(".", 1)[1] in FINAL:
            return


def show(event: dict) -> None:
    p = event["payload"]
    extra = {
        k: p[k]
        for k in ("error_code", "step", "departure_ready", "armed", "ready", "hold")
        if k in p
    }
    print(
        f"{event['type']:<28} rev={p['phase_revision']} phase={p['phase']} {json.dumps(extra, ensure_ascii=False)}",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("operation", choices=["status", "inbound", "outbound"])
    parser.add_argument(
        "--host", default=os.environ.get("ROPORT_MQTT_HOST", "127.0.0.1")
    )
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("ROPORT_MQTT_PORT", "1883"))
    )
    parser.add_argument("--site", default="test")
    parser.add_argument("--control", default="control-test")
    parser.add_argument("--binding", default="test-binding")
    parser.add_argument("--slot", default="slot-A1")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--log", type=Path, default=Path("cooperative_sim_events.jsonl")
    )
    args = parser.parse_args()

    transport = MqttTransport(
        MqttConfig(args.host, f"roport-sim-control-{uuid4().hex[:8]}", port=args.port),
        [topic(args.site, "workcell", WORKCELL, "event")],
    )
    cmd_topic = topic(args.site, "workcell", WORKCELL, "cmd")
    try:
        transport.start()
        with args.log.open("a") as log_file:
            status = envelope(
                args,
                "status.requested",
                {"binding_id": args.binding},
                new_ids("status"),
            )
            transport.publish(cmd_topic, status)
            boot_id = None
            for event in wait_events(args, transport, status, log_file):
                show(event)
                boot_id = event["payload"].get("boot_id")
            if args.operation == "status" or not event["type"].endswith(".completed"):
                return
            operation = args.operation.upper()
            payload = {
                "binding_id": args.binding,
                "expected_boot_id": boot_id,
                "operation": operation,
                "workcell_id": WORKCELL,
                "manipulation_mode": "COOPERATIVE",
                "assigned_arm_id": None,
                "robot_ids": ["omx1", "omx2"],
                "package_id": "sim-package-001",
                "quantity": 1,
                "slot_id": args.slot,
                "pinky_id": "pinky3" if operation == "OUTBOUND" else "pinky1",
                "dock_session_id": "sim-dock-001",
                "resource_token": "sim-grant-001",
                "model_version": "sim-model",
                "calibration_version": "sim-cal",
            }
            task = envelope(
                args,
                f"shelf_{args.operation}.requested",
                payload,
                new_ids(f"shelf-{args.operation}"),
            )
            transport.publish(cmd_topic, task)
            for event in wait_events(args, transport, task, log_file):
                show(event)
    except (ConnectionError, TimeoutError) as exc:
        print(f"stopped: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        transport.stop()


if __name__ == "__main__":
    main()
