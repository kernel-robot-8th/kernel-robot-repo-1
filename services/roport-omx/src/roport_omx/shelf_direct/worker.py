"""장비 쪽 작업 프로세스(il3 venv: lerobot·OpenCV·numpy). Host(core 환경)의 LocalPanel이 띄운다.

  python -m roport_omx.shelf_direct.worker mission <request.json>   미션 한 건. 진행·결과는 request의 status 파일
  python -m roport_omx.shelf_direct.worker arms <request.json>      양팔 관절 읽기(토크 변경 없음). stdout JSON
  python -m roport_omx.shelf_direct.worker home <request.json>      처음 자세로 복귀. stdout JSON

request.json: {"repo_id", "body", "status_path", "robot": {"ports", "cameras", "calibration_id", "calibration_dir"},
"inference": {"server": [host, port], "key_file", "timeout_s", "attempts"}, "aligner": {"cmd", "work_dir", "cwd"},
"model_home", "arms" (home: {side: {"port", "joints"}})}
"""

from __future__ import annotations

import json
import logging
import sys
import threading
from pathlib import Path

from .aligner import Aligner, AlignerConfig
from .contract import SIDES
from .devices import read_arm, return_home
from .infer import BiUdpInferClient, read_key
from .judge import PlaceWatch
from .lift import make_lifter
from .mission import FILTER_ALPHA, Mission, install_signals
from .policy import ActionFilter, BiChunkPolicy
from .robot import BiOmxRobot, RobotConfig
from .runner import RunSettings, ShelfRunner

log = logging.getLogger("shelf_direct.worker")


def robot_config(req: dict) -> RobotConfig:
    r = req["robot"]
    return RobotConfig(
        ports=r["ports"],
        cameras=r.get("cameras") or {},
        calibration_id=r.get("calibration_id", "omx_follower_arm"),
        calibration_dir=r.get("calibration_dir"),
    )


def read_both(req: dict) -> dict:
    r = req["robot"]
    out, errors = {}, []

    def one(side: str) -> None:
        try:
            out[side] = read_arm(
                r["ports"][side],
                r.get("calibration_id", "omx_follower_arm"),
                r.get("calibration_dir"),
            )
        except Exception as exc:  # noqa: BLE001 - 읽지 못한 팔은 없는 것으로
            errors.append(f"{side}: {exc}")

    threads = [threading.Thread(target=one, args=(s,)) for s in SIDES]
    for t in threads:
        t.start()
    for t in threads:
        t.join(8.0)
    if errors or set(out) != set(SIDES):
        raise RuntimeError("; ".join(errors) or "arm read timed out")
    return out


def make_runner_factory(req: dict):
    inf = req["inference"]

    def make(body: dict, plan: dict, aligner: Aligner):
        client = BiUdpInferClient(
            tuple(inf["server"]),
            read_key(inf["key_file"]),
            body["model_path"],
            chunk=int(inf.get("chunk", 100)),
            timeout_s=float(inf.get("timeout_s", 1.5)),
            attempts=int(inf.get("attempts", 3)),
        )
        client.load()
        level = body.get("output_filter")
        policy = BiChunkPolicy(
            client,
            actions_per_chunk=min(100, int(body.get("actions_per_chunk") or 100)),
            threshold=float(body.get("chunk_size_threshold") or 0.25),
            aggregate=body.get("aggregate_fn") or "latest_only",
            action_filter=ActionFilter(FILTER_ALPHA[level]) if level else None,
        )
        robot = BiOmxRobot(robot_config(req), lifter=make_lifter(0.04), log=log.info)
        watch = None
        if body.get("shelf_align") and (
            body.get("end_on_place") or body.get("place_shadow")
        ):
            watch = PlaceWatch(
                [int(body["target_marker"])], shadow=not plan["handover"]
            )
        runner = ShelfRunner(
            robot,
            policy,
            RunSettings(
                episode_s=float(body.get("episode_s") or 180.0),
                end_on_return=bool(body.get("end_on_return", True)),
                fast_release=bool(body.get("fast_release")),
            ),
            end_checks=[watch.end_check] if watch else [],
            hold_ready=aligner.handover_ready if plan["handover"] else (lambda: False),
            on_phase=lambda phase, info: log.info("phase %s %s", phase, info),
        )
        if watch is not None:
            watch.reset()
        return runner, watch

    return make


def main(argv: list[str]) -> int:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    cmd, req = argv[0], json.loads(Path(argv[1]).read_text())
    if cmd == "arms":
        print(json.dumps(read_both(req)))
        return 0
    if cmd == "home":
        done = return_home(req["arms"], lifter=make_lifter(0.04), log=log.info)
        print(json.dumps(done))
        return 0 if all(done.values()) else 1
    if cmd != "mission":
        raise SystemExit(f"unknown command {cmd}")
    a = req["aligner"]
    aligner = Aligner(
        AlignerConfig(
            cmd=tuple(a["cmd"]), work_dir=Path(a["work_dir"]), cwd=Path(a["cwd"])
        )
    )
    mission = Mission(
        req["body"],
        req["repo_id"],
        make_runner=make_runner_factory(req),
        aligner=aligner,
        read_arms=lambda: read_both(req),
        return_home=lambda arms: return_home(
            arms, lifter=make_lifter(0.04), log=log.info
        ),
        model_home=req.get("model_home"),
        ports=req["robot"]["ports"],
        status_path=Path(req["status_path"]),
    )
    install_signals(mission)
    try:
        return mission.run()
    finally:
        aligner.kill()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
