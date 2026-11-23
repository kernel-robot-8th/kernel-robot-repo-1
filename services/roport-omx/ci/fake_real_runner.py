"""시험용 가짜 real_runner.py: 정렬기의 CLI·파일·exit code 계약만 흉내 낸다(장치·카메라 없음).

FAKE_RR_MODE: ok(기본) | tray_bad | hang | mismatch | silent | stop_wait | no_route
"""

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

MODE = os.environ.get("FAKE_RR_MODE", "ok")
T0 = time.time()
STOP = {"req": False}
signal.signal(signal.SIGTERM, lambda *_: STOP.__setitem__("req", True))


def write(path, data):
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(data))
    os.replace(tmp, path)


def fresh(path):
    try:
        return Path(path).stat().st_mtime > T0
    except OSError:
        return False


ap = argparse.ArgumentParser()
for flag in (
    "--box",
    "--slot",
    "--step",
    "--result",
    "--tray-ref",
    "--pinky",
    "--home-pose",
    "--handover-wait",
    "--repo-id",
    "--handover-dir",
):
    ap.add_argument(flag)
ap.add_argument("--auto", action="store_true")
ap.add_argument("--release", action="store_true")
a = ap.parse_args()
with Path(os.environ["FAKE_RR_LOG"]).open("a") as f:
    f.write(
        json.dumps(
            {
                "argv": sys.argv[1:],
                "cwd": os.getcwd(),
                "panel": os.environ.get("OMX_PANEL_URL"),
                "threads": os.environ.get("OMP_NUM_THREADS"),
            }
        )
        + "\n"
    )

if MODE == "hang":
    time.sleep(60)
if a.release:
    sys.exit(3 if MODE == "no_route" else 0)
if a.step == "tray_check":
    ok = MODE != "tray_bad"
    if ok:
        write(
            a.tray_ref,
            {"kind": "tray_pre_mission", "reference_at": "2026-11-23T21:00:00+09:00"},
        )
    write(
        a.result,
        {
            "result": {
                "tray_check": "ok" if ok else "not_empty",
                "tray_reference": "pre_mission" if ok else None,
                "reference_at": "2026-11-23T21:00:00+09:00" if ok else None,
            }
        },
    )
    sys.exit(0)
if a.step == "pinky_read":
    write(
        a.result,
        {
            "result": {
                "on_pinky": True,
                "on_shelf": False,
                "tray_reference": "pre_mission",
                "reference_at": "2026-11-23T21:00:00+09:00" if a.tray_ref else None,
            },
            "log": "runs/p.json",
        },
    )
    sys.exit(0)


def align_and_exit(extra=None):
    placement = {
        "on_shelf": True,
        "shelf": "ok",
        "slot": a.slot,
        "box": int(a.box),
        "source": "align",
    }
    write(str(a.result).replace(".json", ".placement.json"), placement)
    if MODE == "stop_wait":
        while not STOP["req"]:
            time.sleep(0.02)
        write(
            a.result,
            {
                "box": int(a.box),
                "result": {"stopped": True},
                "early_exit": True,
                "log": "runs/a.json",
            },
        )
        sys.exit(2)
    result = {
        "shelf": "ok",
        "on_shelf": True,
        "align": "aligned",
        "rest_target": "home",
        "home_pose": {"left_shoulder_pan": 0.0},
        "home_given": a.home_pose is not None,
        **(extra or {}),
    }
    write(
        a.result,
        {
            "box": int(a.box),
            "slot": a.slot,
            "result": result,
            "success": None,
            "log": "runs/a.json",
        },
    )
    sys.exit(0)


if a.step == "shelf_read":
    write(a.result, {"result": {"shelf": "ok", "on_shelf": True}, "log": "runs/r.json"})
    sys.exit(0)
if a.handover_wait:
    d = Path(a.handover_dir)
    state = None
    while state is None:
        write(d / "handover_ready.json", {"pid": os.getpid(), "ts": time.time()})
        done = d / "handover_eval_done"
        if fresh(done) and json.loads(done.read_text()).get("hold_exit") is not True:
            state = "not_placed"
        elif fresh(d / "handover_stop") or STOP["req"]:
            state = "stopped"
        elif fresh(done):
            ep = (
                json.loads(Path(a.handover_wait).read_text())
                if fresh(a.handover_wait)
                else {}
            )
            ok = ep.get("repo_id") == a.repo_id and ep.get("end_reason") == "placed"
            state = (
                ("regs_mismatch" if MODE == "mismatch" else "taken")
                if ok
                else "not_placed"
            )
        time.sleep(0.05)
    (d / "handover_ready.json").unlink(missing_ok=True)
    if MODE == "silent" and state == "taken":
        time.sleep(60)
    write(d / "handover_state.json", {"state": state, "ts": time.time()})
    if state == "taken":
        align_and_exit({"handover_align": "from_handover"})
    write(
        a.result,
        {
            "box": int(a.box),
            "result": {"stopped": state == "stopped"},
            "early_exit": True,
        },
    )
    sys.exit({"stopped": 2, "regs_mismatch": 3}.get(state, 0))
align_and_exit()
