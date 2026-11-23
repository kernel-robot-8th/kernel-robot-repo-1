"""미션 한 건(평가 1회 + 이어지는 정렬·확인)의 흐름. Panel(omx_web.py v3.21.1)의 source=mqtt 경로를 옮겼다.

worker 프로세스(il3 venv)에서 돈다. 진행과 결과는 Panel /api/status의 last_result와 같은 형식으로 status 파일에
남기고, Host 쪽 LocalPanel이 그 파일을 읽어 PanelExecutor에 그대로 넘긴다(현장 v2 판정 규칙을 고치지 않고 쓴다).

흐름
1. 출고(pinky_check): 쟁반 확인. 수락되지 않으면 phase "refused"(Panel의 400 tray_check)로 끝, 팔은 움직이지 않음.
2. 입고 + end_on_place: 정렬기 standby 시작(넘겨받기 대기).
3. 평가(ShelfRunner): 놓음 판정(placed) 또는 복귀 감지(returned)로 끝. 넘겨받기 준비면 토크를 켠 채 포트만 닫음.
4. 입고: 넘겨받았으면(taken) 정렬기가 이어서 정렬. 아니면 평가가 정상 종료됐을 때 정렬(shelf_align).
   넘겨받기 실패(failed·stopped)면 처음 자세로 복귀(fallback home), regs_mismatch·stopped·hold면 HOLD.
5. 출고: Pinky 위 확인(pinky_read). 비상 정지면 건너뜀.
6. 정렬 뒤 제자리 확인(rest_check): 버스에서 양팔을 다시 읽어 기준 자세와 5 넘게 다른 관절.
정지(SIGINT/SIGTERM): 평가 중이면 runner를 멈추고 착지, 정렬 중이면 정렬기를 멈춘 뒤 기록한 길로 되돌림(--release).
"""

from __future__ import annotations

import json
import logging
import os
import signal
import threading
import time
from collections.abc import Callable, Mapping
from datetime import datetime
from pathlib import Path

from .aligner import Aligner, rest_check, rest_reference
from .contract import SIDES, STATE_KEYS
from .runner import RunResult

log = logging.getLogger(__name__)
ARM_NAMES = {"left": "OMX 1", "right": "OMX 2"}
GRIP_OPEN, GRIP_CLOSED = 57.0, 52.0
FLAG_SIGMA, MIN_STD = 3.0, 0.5
FILTER_ALPHA = {"약": 0.4, "중": 0.25, "강": 0.14}
STOP_NAMES = {"stop": "중지", "freeze": "비상 정지"}
ERROR_CODES = {
    "refused": "ROBOT_NOT_READY",
    "PolicyUnavailable": "INFERENCE_UNAVAILABLE",
    "SafeStartRefused": "SAFE_START_REFUSED",
}


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _iso_ms() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def arm_states(state: Mapping[str, float] | None, home: Mapping | None) -> dict:
    """Panel arm_states: 팔마다 관절, 그리퍼, 모델 홈(3σ, 최소 0.5, 그리퍼 제외)에 있는가. 모르면 None."""
    out = {}
    for side, name in ARM_NAMES.items():
        pre = f"{side}_"
        joints = {
            k[len(pre) : -4]: v
            for k, v in (state or {}).items()
            if k.startswith(pre) and k.endswith(".pos")
        }
        g = joints.get("gripper")
        grip = (
            None
            if g is None
            else "open"
            if g >= GRIP_OPEN
            else "closed"
            if g < GRIP_CLOSED
            else "half"
        )
        at_home, far = None, []
        hj = {
            k: v
            for k, v in ((home or {}).get("joints") or {}).items()
            if k.startswith(pre) and not k.endswith("gripper")
        }
        if hj and joints:
            std = (home or {}).get("std") or {}
            cur = {k: (state or {}).get(k + ".pos") for k in hj}
            if all(c is not None for c in cur.values()):
                far = [
                    k[len(pre) :]
                    for k, v in hj.items()
                    if abs(cur[k] - v) > FLAG_SIGMA * max(std.get(k, 0.0), MIN_STD)
                ]
                at_home = not far
        out[name] = {
            "side": side,
            "joints": joints or None,
            "gripper": g,
            "gripper_state": grip,
            "gripper_open": None if grip is None else grip == "open",
            "at_home": at_home,
            "far_joints": far,
        }
    return out


class Mission:
    def __init__(
        self,
        body: Mapping,  # PanelExecutor.start가 보낸 /api/eval/start 본문
        repo_id: str,
        *,
        make_runner: Callable,  # (body, plan, aligner) -> (ShelfRunner, PlaceWatch | None)
        aligner: Aligner,
        read_arms: Callable[[], dict],  # 버스에서 양팔 {side: {joint: v}}
        return_home: Callable[
            [dict], dict
        ],  # {side: {"port","joints"}} -> {side: bool}
        model_home: Mapping | None,
        ports: Mapping[str, str],
        status_path: Path,
    ):
        self.body, self.repo_id = dict(body), repo_id
        self.make_runner, self.aligner = make_runner, aligner
        self.read_arms, self.return_home_fn = read_arms, return_home
        self.home, self.ports = model_home, dict(ports)
        self.status_path = Path(status_path)
        self.stop_event = threading.Event()
        self.stop_kind: str | None = None
        self.phase = "starting"
        self.result: dict = {}
        self.box = int(body["target_marker"])
        self.slot = str(body.get("slot") or "2B").upper()
        self.inbound = bool(body.get("shelf_align"))
        self.outbound = bool(body.get("pinky_check"))
        self.handover = (
            self.inbound
            and bool(body.get("end_on_place"))
            and bool(body.get("end_on_return", True))
        )
        self.start_poses: dict = {}
        self.watch = None

    # ------------------------------------------------------------ 상태 파일
    def _publish(self, phase: str | None = None, running: bool = True, **extra) -> None:
        if phase:
            self.phase = phase
        data = {
            "pid": os.getpid(),
            "repo_id": self.repo_id,
            "phase": self.phase,
            "running": running,
            "last_result": self.result or None,
            "at": time.time(),
            **extra,
        }
        tmp = self.status_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False))
        os.replace(tmp, self.status_path)

    def request_stop(self, kind: str = "stop") -> None:
        if self.stop_kind is None:
            self.stop_kind = kind
        self.stop_event.set()
        if self.aligner.running() and self.aligner.kind in {
            "align",
            "shelf_read",
            "pinky_read",
            "tray_check",
        }:
            threading.Thread(target=self.aligner.stop, daemon=True).start()

    # ------------------------------------------------------------ 전체 흐름
    def run(self) -> int:
        pre = None
        if self.outbound:
            self._publish("tray_check")
            pre = self.aligner.tray_check(
                self.box, self.slot, self.body.get("pinky_id")
            )
            if not pre["accepted"]:
                self._publish(
                    "refused", running=False, refused="tray_check", tray_check=pre
                )
                return 4
        if self.handover:
            self.aligner.start_standby(self.box, self.slot, self.repo_id, self.home)
        started = time.time()
        try:  # 모델 올리기(GPU LOAD)·장치 구성 실패는 평가 오류로 끝낸다(팔은 움직이지 않음)
            runner, watch = self.make_runner(
                self.body, {"handover": self.handover}, self.aligner
            )
        except Exception as exc:
            log.exception("run setup failed")
            if self.handover:
                self.aligner.handover_stop()
                self.aligner.stop()
            res = RunResult("error", error=f"PolicyUnavailable: {exc}"[:200])
            self._eval_result(res, started, None)
            self.result.update(stage="done", final=True)
            self._publish("done", running=False)
            return 1
        self.watch = watch
        self._publish("eval")
        res = runner.run(self.body["task"], self.stop_event)
        self.start_poses = res.start_poses
        self._eval_result(res, started, watch)
        end = self._episode_end(res, watch)
        self.aligner.write_episode_end(end)
        if self.handover:
            self._after_handover(res)
        elif self.inbound:
            self._align_after_eval()
        if self.outbound and self.result.get("stopped_by") not in {
            "비상 정지",
            "강제 종료",
        }:
            self._publish("pinky")
            self.result["pinky"] = self.aligner.pinky_read(
                self.box, self.slot, self.body.get("pinky_id"), pre
            )
        if self.result.get("stage") != "hold":
            self.result.update(stage="done", final=True)
        self._publish("done", running=False, start_poses=self.start_poses)
        return 0

    def _eval_result(self, res: RunResult, started: float, watch) -> None:
        stopped = (
            STOP_NAMES.get(self.stop_kind)
            if self.stop_kind
            else ("비상 정지" if res.end_reason == "frozen" else None)
        )
        err = None
        if res.end_reason in {"error", "refused"}:
            name = (
                (res.error or "").split(":", 1)[0]
                if res.end_reason == "error"
                else "refused"
            )
            err = {
                "code": ERROR_CODES.get(name, "RUNTIME_ERROR"),
                "name": name,
                "message": (res.error or "")[:200],
            }
        state = (
            dict(zip(STATE_KEYS, res.last_state, strict=True)) if res.last_state else {}
        )
        arms = arm_states(state, self.home) if state else {}
        checks = [a["at_home"] for a in arms.values()]
        self.result = {
            "kind": "eval",
            "source": "mqtt",
            "repo_id": self.repo_id,
            "recorded": False,
            "started_at": datetime.fromtimestamp(started).isoformat(timespec="seconds"),
            "ended_at": _now(),
            "returncode": 0 if err is None else 1,
            "exit_ok": err is None and stopped is None,
            "stopped_by": stopped,
            "err": err,
            "target_marker": self.box,
            "target_drawn": False,
            "last_episode": {
                "episode": 0,
                "at": _now(),
                "aborted": stopped is not None,
                "returned": res.end_reason == "returned",
                "elapsed_s": res.elapsed_s,
                "end_reason": self._end_reason(res),
                "judged_at": (
                    (watch.placed() or {}).get("judged_at") if watch else None
                ),
                "loop_ms": res.loop_ms,
            },
            "arms": arms or None,
            "home_check": "unknown"
            if not checks or None in checks
            else "ok"
            if all(checks)
            else "far",
            "landing": res.landing,
            "stage": "eval",
            "final": False,
        }

    @staticmethod
    def _end_reason(res: RunResult) -> str:
        return {"frozen": "stopped", "error": "stopped", "refused": "stopped"}.get(
            res.end_reason, res.end_reason
        )

    def _episode_end(self, res: RunResult, watch) -> dict:
        hit = (watch.placed() if watch else None) or {}
        state = (
            dict(zip(STATE_KEYS, res.last_state, strict=True)) if res.last_state else {}
        )
        return {
            "repo_id": self.repo_id,
            "episode": 0,
            "at": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            "aborted": self.stop_kind is not None,
            "returned": res.end_reason == "returned",
            "elapsed_s": res.elapsed_s,
            "end_reason": self._end_reason(res),
            "judged_at": hit.get("judged_at"),
            "judge": {k: v for k, v in hit.items() if k != "judged_at"} or None,
            "loop_ms": res.loop_ms,
            "judge_watch": watch.summary() if watch else None,
            "state_ts": res.state_at,
            "state_at": datetime.fromtimestamp(res.state_at).isoformat(
                timespec="milliseconds"
            )
            if res.state_at
            else None,
            "state": state,
        }

    # ------------------------------------------------------------ 입고: 넘겨받기
    def _after_handover(self, res: RunResult) -> None:
        hit = (self.watch.placed() if self.watch else None) or {}
        summary = self.watch.summary() if self.watch else {}
        seen = {
            "mode": "handover",
            "end_reason": self._end_reason(res),
            "judged_at": hit.get("judged_at"),
            "judge": {
                k: hit[k]
                for k in ("box", "version", "judged_ts", "tip_y", "marker_seen")
                if k in hit
            }
            or None,
            "judge_watch": {
                k: summary.get(k) for k in ("version", "updates", "errors", "hit")
            }
            or None,
            "loop_ms": res.loop_ms,
            "off_reason": None,
            "state": None,
            "fallback": None,
        }
        self.result["handover"] = seen
        hold_exit = res.landing == "hold"
        self.aligner.eval_done(hold_exit, self._end_reason(res))
        if not hold_exit:
            seen["state"] = "not_placed" if res.end_reason != "placed" else "not_ready"
            self.result.update(
                stage="align",
                final=False,
                align={"ran": False, "ok": None, "box": self.box, "slot": self.slot},
            )
            if self.aligner.wait(5.0) is None:
                self.aligner.kill()
            self._align_after_eval()
            return
        self.result["placement"] = self._judge_placement(hit)
        self.result.update(
            stage="align",
            final=False,
            align={
                "ran": False,
                "ok": None,
                "box": self.box,
                "slot": self.slot,
                "handover": True,
            },
        )
        seen["state"] = "waiting"
        self._publish("handover")
        st = self.aligner.wait_taken()
        seen.update(state=st["state"], reason=st.get("reason"), state_at=_iso_ms())
        if st["state"] == "taken":
            self._publish("align")
            self._align_ended(self.aligner.wait(), handover=True)
            return
        skipped = {
            "ran": False,
            "ok": False,
            "box": self.box,
            "slot": self.slot,
            "skipped": f"넘겨받지 못함 ({st['state']})",
        }
        self.result["align"] = skipped
        if st["state"] in {"regs_mismatch", "stopped"}:
            if self.aligner.wait(5.0) is None:
                self.aligner.kill()
            seen["fallback"] = "hold"
            self.result.update(stage="hold", final=False)
            return
        if self.aligner.wait(5.0) is None:
            self.aligner.kill()
        seen["fallback"] = "home"
        self._publish("home")
        ok = self._home_now()
        seen["fallback_ok"] = ok

    def _judge_placement(self, judge: Mapping) -> dict:
        """놓음 판정 순간의 근거(Panel judge_evidence)."""
        return {
            "on_shelf": True,
            "shelf": "ok",
            "slot": self.slot,
            "box": self.box,
            **judge_evidence(judge),
            "source": "eval_judge",
            "judge_version": judge.get("version"),
            "reported_at": _iso_ms(),
        }

    # ------------------------------------------------------------ 입고: 정렬
    def _align_after_eval(self) -> None:
        if not self.result.get("exit_ok") or self.stop_event.is_set():
            return
        self._publish("align")
        self.result.update(stage="align", final=False)
        self.aligner.start_align(self.box, self.slot, self.home)
        self._align_ended(self.aligner.wait(), handover=False)

    def _align_ended(self, rc: int | None, handover: bool) -> None:
        stopped = STOP_NAMES.get(self.stop_kind) if self.stop_kind else None
        align = self.aligner.align_result(rc, stopped_by=stopped)
        align.update(box=self.box, slot=self.slot)
        placement = self.aligner.placement("align")
        if placement is not None:
            prev = self.result.get("placement")
            if isinstance(prev, dict) and prev.get("source") == "eval_judge":
                placement["first"] = {
                    k: prev.get(k)
                    for k in ("source", "on_shelf", "observed_at", "reported_at")
                }
            self.result["placement"] = placement
        self.result["align"] = align
        if handover and align.get("handover_align") == "hold":
            self.result.update(stage="hold", final=False)
            return
        if stopped is not None:
            if stopped == "비상 정지":
                self.result.update(stage="hold", final=False)
                return
            self._publish("release")
            self.aligner.start_release(self.box)
            rc2 = self.aligner.wait(120.0)
            align["released"] = rc2 == 0
            if rc2 != 0:
                self.result.update(stage="hold", final=False)
            return
        before, ref = rest_reference(align, self.aligner_episode_state())
        time.sleep(0.5)
        try:
            after = self.read_arms()
        except Exception as exc:  # noqa: BLE001 - 읽지 못하면 unknown(완료로 보지 않음)
            log.warning("rest read failed: %s", exc)
            after = None
        align["rest"] = rest_check(before, after)
        align["rest_ref"] = ref

    def aligner_episode_state(self) -> dict | None:
        try:
            return json.loads(
                (self.aligner.c.dir() / "episode_end.json").read_text()
            ).get("state")
        except (OSError, ValueError):
            return None

    # ------------------------------------------------------------ 복귀
    def _home_now(self) -> bool:
        arms = {
            side: {"port": self.ports[side], "joints": self.start_poses.get(side)}
            for side in SIDES
        }
        try:
            done = self.return_home_fn(arms)
        except Exception as exc:  # noqa: BLE001 - 복귀 실패는 결과로 남긴다
            log.warning("home return failed: %s", exc)
            return False
        return all(done.get(side) for side in SIDES)


def judge_evidence(judge: Mapping) -> dict:
    """Panel judge_evidence: 판정에 쓴 프레임 시각, 두 손끝이 상자 앞면·선반 가장자리보다 3 cm 뒤인가."""
    frame_ts = judge.get("frame_ts")
    ty = judge.get("tip_y") if isinstance(judge.get("tip_y"), list) else []
    y_out = judge.get("y_out")
    joints = judge.get("joints") or {}
    return {
        "observed_at": judge.get("frame_at") or judge.get("judged_at"),
        "observed_ts": frame_ts or judge.get("judged_ts"),
        "observed_src": "front",
        "observed_clock": "capture" if frame_ts else "control_step",
        "judged_at": judge.get("judged_at"),
        "judged_ts": judge.get("judged_ts"),
        "tips_clear": bool(max(ty) < y_out)
        if len(ty) == 2 and y_out is not None
        else None,
        "tip_y": dict(zip(SIDES, ty, strict=True)) if len(ty) == 2 else None,
        "y_out": y_out,
        "gripper": {s: joints.get(f"{s}_gripper") for s in SIDES},
    }


def install_signals(mission: Mission) -> None:
    """Panel과 같은 신호: SIGINT/SIGTERM = 중지(평가는 착지, 정렬은 멈춘 뒤 되돌림)."""
    signal.signal(signal.SIGINT, lambda *_: mission.request_stop("stop"))
    signal.signal(signal.SIGTERM, lambda *_: mission.request_stop("stop"))
