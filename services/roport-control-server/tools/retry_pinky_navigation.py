"""Local operator reconciliation for one failed Nav2 execution; no MQTT owner."""

import argparse
import json
import socket
import time


def rpc(path, payload):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as channel:
        channel.settimeout(2)
        channel.connect(path)
        channel.sendall(json.dumps(payload, allow_nan=False).encode())
        reply = json.loads(channel.recv(65536))
    if reply.get("ok") is not True:
        raise ValueError(reply.get("error", "local owner did not confirm retry"))
    return reply["snapshot"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--execution-id", required=True)
    parser.add_argument(
        "--field-confirmed",
        action="store_true",
        required=True,
        help="operator checked stopped robot, cargo, cleared route and same goal",
    )
    args = parser.parse_args()
    try:
        current = rpc(args.socket, {"action": "snapshot"})
        result = rpc(
            args.socket,
            {
                "action": "retry_navigation",
                "session_id": current["session_id"],
                "task_id": current["state"]["task_id"],
                "phase": current["state"]["phase"],
                "expires_at": time.time() + 1,
                "failed_execution_id": args.execution_id,
                "field_confirmed": args.field_confirmed,
            },
        )
    except (ValueError, OSError) as exc:
        parser.exit(
            1,
            f"Retry not confirmed: {exc}. Inspect owner state before another attempt.\n",
        )
    print(
        json.dumps(
            {"phase": result["state"]["phase"], "fault": result["state"]["fault"]}
        )
    )


if __name__ == "__main__":
    main()
