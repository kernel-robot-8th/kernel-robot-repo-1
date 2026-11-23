"""shelf-01 direct 실행부(--executor direct)의 armed를 이 PC에서 켜고 끈다. Panel 화면의 armed 스위치를 대신한다.

armed가 켜져 있을 때만 관제 명령으로 팔이 움직인다. 켜기 전에 사람이 현장에서 두 팔·상자·Pinky를 눈으로 확인한다.
중지·이상 종료·HOLD가 생기면 Host가 스스로 끈다. 다시 켜려면 다시 확인한다.

  uv run python scripts/shelf_direct_arm.py --config ~/.config/roport/shelf-direct.json status
  uv run python scripts/shelf_direct_arm.py --config ~/.config/roport/shelf-direct.json on --checked
  uv run python scripts/shelf_direct_arm.py --config ~/.config/roport/shelf-direct.json off
"""

import argparse
import getpass
import json
import os
import sys
import time
import uuid
from pathlib import Path

from roport_omx.shelf_direct.local_panel import DirectConfig

WAIT_S = 5.0  # Host가 1 s마다 상태를 읽는다


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("action", choices=["on", "off", "status"])
    parser.add_argument(
        "--checked",
        action="store_true",
        help="on: 두 팔·상자·Pinky를 현장에서 확인했다",
    )
    args = parser.parse_args()
    d = DirectConfig.load(args.config).dir()
    state_path = d / "arm-state.json"
    if args.action == "status":
        print(
            state_path.read_text()
            if state_path.exists()
            else "Host 상태 파일 없음 (Host가 돌고 있나요?)"
        )
        return 0
    if args.action == "on" and not args.checked:
        parser.error(
            "on은 --checked와 함께: 두 팔·상자·Pinky를 현장에서 확인한 뒤 켭니다"
        )
    request_id = uuid.uuid4().hex
    req = {
        "request_id": request_id,
        "armed": args.action == "on",
        "by": getpass.getuser(),
        "requested_at": time.time(),
    }
    tmp = d / "arm-request.json.tmp"
    tmp.write_text(json.dumps(req))
    os.replace(tmp, d / "arm-request.json")
    end = time.monotonic() + WAIT_S
    while time.monotonic() < end:
        try:
            state = json.loads(state_path.read_text())
        except (OSError, ValueError):
            state = {}
        if state.get("request_id") == request_id:
            print(json.dumps(state, ensure_ascii=False))
            return 1 if state.get("refused") else 0
        time.sleep(0.2)
    print(
        f"Host가 {WAIT_S:g} s 안에 요청을 읽지 않았습니다 (Host가 --executor direct로 돌고 있나요?)",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    sys.exit(main())
