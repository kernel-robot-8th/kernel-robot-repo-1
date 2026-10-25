#!/usr/bin/env python3
"""Read-only OMX v0.6.1 bus probe. Never calls LeRobot.connect()."""

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path

# OmxFollower v0.6.1: XL430 body IDs 11–13, XL330 wrist/gripper IDs 14–16.
MOTORS = {11: 1060, 12: 1060, 13: 1060, 14: 1200, 15: 1200, 16: 1200}


def probe(sdk, device, baudrate):
    """Only PING and READ packets; closing the port does not change torque."""
    port = sdk.PortHandler(device)
    packet = sdk.PacketHandler(2.0)
    rows = []
    try:
        if not port.openPort():
            raise RuntimeError(f"Cannot open port: {device}")
        if not port.setBaudRate(baudrate):
            raise RuntimeError(f"Cannot set host baudrate: {baudrate}")
        for motor_id, expected_model in MOTORS.items():
            row = {"id": motor_id, "expected_model": expected_model, "ok": False}
            rows.append(row)
            try:
                model, comm, error = packet.ping(port, motor_id)
                if comm != sdk.COMM_SUCCESS or error:
                    raise RuntimeError(
                        f"PING failed: comm={comm}, packet_error={error}"
                    )
                row["model"] = model
                if model != expected_model:
                    raise RuntimeError(
                        "Model differs from the vendored OMX motor configuration"
                    )
                for field, address, reader in (
                    ("position_counts", 132, packet.read4ByteTxRx),
                    ("torque_enabled", 64, packet.read1ByteTxRx),
                    ("hardware_error", 70, packet.read1ByteTxRx),
                ):
                    value, comm, error = reader(port, motor_id, address)
                    if comm != sdk.COMM_SUCCESS or error:
                        raise RuntimeError(
                            f"READ {field} failed: comm={comm}, packet_error={error}"
                        )
                    if field == "position_counts" and value >= 2**31:
                        value -= 2**32
                    row[field] = value
                row["ok"] = row["hardware_error"] == 0
            except Exception as exc:  # noqa: BLE001 - report each failed motor, then continue probing
                row["error"] = str(exc)
    finally:
        if port.is_open:
            port.closePort()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        required=True,
        help="Prefer /dev/serial/by-id/...; stop other bus users first",
    )
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.baudrate <= 0:
        parser.error("--baudrate must be positive")
    report = {
        "scope": "omx_ping_and_register_read_only",
        "time_utc": datetime.now(UTC).isoformat(),
        "port": args.port,
        "baudrate": args.baudrate,
        "motion_verified": False,
        "ok": False,
    }
    try:
        import dynamixel_sdk

        report["motors"] = probe(dynamixel_sdk, args.port, args.baudrate)
        report["ok"] = all(row["ok"] for row in report["motors"])
    except Exception as exc:  # noqa: BLE001 - CLI failures must produce a diagnostic report
        report["error"] = str(exc)
    text = json.dumps(report, indent=2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text + "\n")
    print(text)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
