"""Request release of the currently held shelf command from the owner's account."""

import argparse
import time
from uuid import uuid4

from retry_pinky_navigation import rpc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--hold-command-id", required=True)
    for item in ("arms", "box", "pinky"):
        parser.add_argument(
            "--" + item + "-confirmed", required=True, action="store_true"
        )
    args = parser.parse_args()
    request_id = str(uuid4())
    try:
        snapshot = rpc(args.socket, {"action": "snapshot"})
        rpc(
            args.socket,
            {
                "action": "release_workcell",
                "session_id": snapshot["session_id"],
                "expires_at": time.time() + 1,
                "request_id": request_id,
                "hold_command_id": args.hold_command_id,
                "confirmed": {
                    item: getattr(args, item + "_confirmed")
                    for item in ("arms", "box", "pinky")
                },
            },
        )
        print("요청 ID", request_id, flush=True)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            status = rpc(args.socket, {"action": "snapshot"})["host_releases"][
                request_id
            ]["status"]
            if status == "completed":
                print("Host가 일반 HOLD를 해제했습니다. 운행은 별도로 승인하세요.")
                return
            if status != "pending":
                parser.exit(
                    1, f"Host 해제 결과 {status}. 요청 ID로 기록을 확인하세요.\n"
                )
            time.sleep(0.2)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, f"해제 결과를 확인할 수 없습니다. 요청 {request_id}: {exc}\n")
    parser.exit(1, f"해제 결과 미확인. 요청 {request_id}의 기록을 확인하세요.\n")


if __name__ == "__main__":
    main()
