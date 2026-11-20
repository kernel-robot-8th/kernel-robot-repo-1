"""Confirm one restarted Pinky's arrival through the local Fleet owner socket."""

import argparse
import time

from retry_pinky_navigation import rpc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--robot", required=True)
    parser.add_argument("--command-id", required=True)
    parser.add_argument("--boot-version", required=True)
    parser.add_argument("--note", required=True)
    args = parser.parse_args()
    try:
        current = rpc(args.socket, {"action": "snapshot"})
        rpc(
            args.socket,
            {
                "action": "confirm_arrival",
                "session_id": current["session_id"],
                "expires_at": time.time() + 1,
                "robot": args.robot,
                "command_id": args.command_id,
                "boot_version": args.boot_version,
                "note": args.note,
            },
        )
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Arrival confirmation failed: {exc}\n")
    print(f"Arrival confirmed for {args.robot}")


if __name__ == "__main__":
    main()
