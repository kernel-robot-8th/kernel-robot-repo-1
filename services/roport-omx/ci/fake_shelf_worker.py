"""시험용 가짜 shelf_direct worker: status 파일 형식과 신호(SIGINT = 중지)만 흉내 낸다. 장치 없음.

FAKE_WORKER_MODE: ok | refused | crash | wait_stop
FAKE_WORKER_RESULT: 끝날 때 쓸 last_result(JSON)
"""

import json
import os
import signal
import sys
import time
from pathlib import Path

cmd, req = sys.argv[1], json.loads(Path(sys.argv[2]).read_text())
MODE = os.environ.get("FAKE_WORKER_MODE", "ok")
if cmd == "home":
    Path(os.environ["FAKE_WORKER_HOME_LOG"]).write_text(json.dumps(req["arms"]))
    time.sleep(0.3)
    sys.exit(0)

status = Path(req["status_path"])
STOP = {"req": False}
signal.signal(signal.SIGINT, lambda *_: STOP.__setitem__("req", True))


def publish(phase, running=True, **extra):
    tmp = status.with_suffix(".tmp")
    tmp.write_text(
        json.dumps(
            {"repo_id": req["repo_id"], "phase": phase, "running": running, **extra}
        )
    )
    os.replace(tmp, status)


publish("starting")
if MODE == "refused":
    publish(
        "refused",
        running=False,
        refused="tray_check",
        tray_check={"tray_check": "not_empty"},
    )
    sys.exit(4)
publish(
    "eval", last_result={"repo_id": req["repo_id"], "stage": "eval", "final": False}
)
if MODE == "crash":
    time.sleep(0.2)
    os._exit(9)
end = time.time() + (30 if MODE == "wait_stop" else 0.5)
while time.time() < end and not STOP["req"]:
    time.sleep(0.02)
result = json.loads(os.environ.get("FAKE_WORKER_RESULT") or "{}")
if STOP["req"]:
    result = dict(result, exit_ok=False, stopped_by="중지")
result = dict(result, repo_id=req["repo_id"])
publish(
    "done",
    running=False,
    last_result=result,
    start_poses={"left": {"shoulder_pan": 1.0}},
)
