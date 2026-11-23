"""선반 정렬기(real_runner.py) 실행·감독: Panel(omx_web.py v3.21.1)이 하던 쪽 계약을 그대로 옮겼다.

정렬기는 고치지 않고 경로로 실행한다. Panel 서버에는 연결하지 않는다: OMX_PANEL_URL을 닿지 않는 주소로 주면
정렬기는 /api/status를 빈 값으로 보고 카메라를 직접 연다(preview 없음). /api/snapshot도 preview일 때만 쓴다.
정렬기가 import하는 Panel 모듈(omx_safe_start·omx_bimanual·omx_config)은 Panel 폴더에서 그대로 읽는다.

단계 (모두 --auto):
- tray_check (출고 시작 전, 동기 20 s): 빈 쟁반 기준 사진(tray_pre_*.json)을 남긴다. ok + pre_mission일 때만 시작.
- standby (입고, 평가와 동시): --handover-wait로 기다리다 놓음 판정 뒤 팔을 토크를 켠 채 넘겨받아 정렬한다.
  파일: 정렬기 → handover_ready.json(0.5 s 심박), handover_state.json(taken/regs_mismatch/stopped/not_placed).
        실행부 → episode_end.json, handover_eval_done({hold_exit, end_reason}), handover_stop.
- align (입고, 넘겨받기 없이 평가가 정상 종료된 뒤): --step shelf_align.
- shelf_read (팔을 움직이지 않고 선반만 읽기), pinky_read (출고 뒤, 20 s), release (정렬 중단 뒤 되돌아가기).
exit code: 0 끝(정렬 안 됨 포함), 1 오류, 2 정지·유지(토크 켬), 3 되돌아갈 길 없음·레지스터 불일치.
"""

from __future__ import annotations

import json
import logging
import os
import re
import signal
import subprocess
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

log = logging.getLogger(__name__)
DEAD_PANEL_URL = "http://127.0.0.1:1"  # 닿지 않는 주소: 정렬기가 카메라를 직접 연다
STOP_WAIT_S = 5.0
TAKEN_WAIT_S = 8.0
READ_TIMEOUT_S = 20.0
HEARTBEAT_S = 1.0
REST_TOL = 5.0


@dataclass(frozen=True)
class AlignerConfig:
    cmd: Sequence[str] = (
        "~/venv/omx_sim/bin/python",
        "~/il_ws/src/omx_sim_align/real_runner.py",
    )
    work_dir: Path = Path("~/.local/state/roport/shelf-direct")  # 결과·넘겨받기 파일
    cwd: Path = Path("~/il_ws/src/lerobot_v0.6.1")  # Panel과 같은 실행 위치
    slot: str = "2B"
    read_timeout_s: float = READ_TIMEOUT_S
    taken_wait_s: float = TAKEN_WAIT_S
    panel_url: str = DEAD_PANEL_URL
    extra_env: Mapping[str, str] = field(default_factory=dict)
    extra_args: Sequence[str] = ()  # 모든 실행 뒤에 붙인다(시험: --dry-run 등)

    def command(self) -> list[str]:
        return [os.path.expanduser(c) for c in self.cmd]

    def dir(self) -> Path:
        d = Path(os.path.expanduser(self.work_dir))
        d.mkdir(parents=True, exist_ok=True)
        return d


def _iso_ms() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _write_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    os.replace(tmp, path)


def _read_json(path: Path) -> dict | None:
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def pinky_number(pinky_id: str | None) -> str | None:
    m = re.search(r"pinky(\d+)", str(pinky_id or ""), re.IGNORECASE)
    return m.group(1) if m else None


def normalize_slot(slot: str | None, default: str) -> str:
    s = str(slot or "").upper()
    return s if re.fullmatch(r"[1-9][A-Z]", s) else default


class Aligner:
    """한 실행의 정렬기 프로세스들. 한 번에 하나의 프로세스만 돈다(Panel의 robot job과 같음)."""

    def __init__(
        self,
        config: AlignerConfig = AlignerConfig(),  # noqa: B008 - frozen dataclass
        *,
        popen: Callable = subprocess.Popen,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self.c, self.popen, self.clock, self.sleep = config, popen, clock, sleep
        self.proc = None
        self.kind: str | None = None
        self.stop_requested = False
        self.eval_done_at: float | None = None

    # ------------------------------------------------------------ 공통
    def _env(self, standby: bool = False) -> dict:
        env = dict(os.environ)
        env.update(self.c.extra_env)
        env.update({"PYTHONUNBUFFERED": "1", "OMX_PANEL_URL": self.c.panel_url})
        if standby:  # 평가와 함께 도는 동안 CPU를 덜 쓴다(Panel Standby)
            env.update(
                {
                    "OMP_NUM_THREADS": "1",
                    "MKL_NUM_THREADS": "1",
                    "OPENBLAS_NUM_THREADS": "1",
                }
            )
        return env

    def _launch(
        self,
        kind: str,
        args: list[str],
        *,
        standby: bool = False,
        cwd: str | None = None,
    ):
        if self.running():
            raise RuntimeError(f"aligner {self.kind} still running")
        d = self.c.dir()
        logf = open(d / f"{time.strftime('%Y%m%d_%H%M%S')}_{kind}.log", "w")  # noqa: SIM115 - 프로세스가 쓴다
        kwargs = {
            "cwd": cwd or os.path.expanduser(str(self.c.cwd)),
            "env": self._env(standby),
            "stdin": subprocess.DEVNULL,
            "stdout": logf,
            "stderr": subprocess.STDOUT,
            "start_new_session": True,
        }
        if standby:
            kwargs["preexec_fn"] = lambda: os.nice(10)
        try:
            self.proc = self.popen(
                self.c.command() + args + list(self.c.extra_args), **kwargs
            )
        finally:
            logf.close()  # 자식이 복사본을 가진다
        self.kind, self.stop_requested = kind, False
        log.info("aligner %s started (pid %s)", kind, self.proc.pid)
        return self.proc

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def wait(self, timeout: float | None = None) -> int | None:
        if self.proc is None:
            return None
        end = None if timeout is None else self.clock() + timeout
        while self.proc.poll() is None:
            if end is not None and self.clock() >= end:
                return None
            self.sleep(0.05)
        return self.proc.returncode

    def stop(self) -> int | None:
        """정지 요청: 정렬기는 제자리에서 멈추고 토크를 켠 채 exit 2. 5 s 안에 안 끝나면 강제 종료."""
        if not self.running():
            return self.proc.returncode if self.proc else None
        self.stop_requested = True
        os.kill(self.proc.pid, signal.SIGTERM)
        rc = self.wait(STOP_WAIT_S)
        if rc is None:
            self.kill()
            rc = self.wait(2.0)
        return rc

    def kill(self) -> None:
        if self.running():
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(2.0)  # 좀비를 남기지 않는다
            except subprocess.TimeoutExpired:
                log.warning("aligner pid %s did not exit after SIGKILL", self.proc.pid)

    def _box_args(
        self, box: int, slot: str | None, step: str, result: Path
    ) -> list[str]:
        return [
            "--box", str(int(box)), "--slot", normalize_slot(slot, self.c.slot),
            "--step", step, "--auto", "--result", str(result),
        ]  # fmt: skip

    def _sync_read(
        self, kind: str, args: list[str], cwd: str | None = None
    ) -> tuple[int | None, bool]:
        """카메라만 쓰는 단계: read_timeout_s 안에 끝나지 않으면 강제 종료. (rc, timed_out)"""
        self._launch(kind, args, cwd=cwd)
        rc = self.wait(self.c.read_timeout_s)
        if rc is None:
            self.kill()
            self.wait(2.0)
            return None, True
        return rc, False

    # ------------------------------------------------------------ 출고: 쟁반 확인
    def tray_check(self, box: int, slot: str | None, pinky_id: str | None) -> dict:
        """출고 시작 전. 돌려준 dict의 accepted가 True일 때만 평가를 시작한다(Panel이 400으로 거절하던 조건)."""
        d = self.c.dir()
        result = d / "tray_check_result.json"
        result.unlink(missing_ok=True)
        ref = d / f"tray_pre_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
        args = self._box_args(box, slot, "tray_check", result) + [
            "--tray-ref",
            str(ref),
        ]
        if (n := pinky_number(pinky_id)) is not None:
            args += ["--pinky", n]
        started = self.clock()
        rc, timed_out = self._sync_read(
            "tray_check", args, cwd=os.path.expanduser("~")
        )  # Panel과 같음
        r = ((_read_json(result) or {}).get("result") or {}) if not timed_out else {}
        status = r.get("tray_check") or ("timeout" if timed_out else "no_result")
        reference_at = r.get("reference_at") or r.get("tray_reference_at")
        out = {
            k: r.get(k)
            for k in (
                "tray_reference", "observed_at", "not_empty_why", "tray_empty_verified",
                "observed_ts", "observed_src", "box_on_tray_ids", "tray_changed", "photo_top",
            )
        }  # fmt: skip
        out.update(
            tray_check=status,
            tray_ref_file=str(ref),
            reference_at=reference_at,
            returncode=rc,
            checked_at=_iso_ms(),
            elapsed_s=round(self.clock() - started, 1),
        )
        out["accepted"] = bool(
            status == "ok" and r.get("tray_reference") == "pre_mission" and reference_at
        )
        return out

    # ------------------------------------------------------------ 출고: Pinky 위 확인
    def pinky_read(
        self, box: int, slot: str | None, pinky_id: str | None, pre: dict | None
    ) -> dict:
        d = self.c.dir()
        result = d / "pinky_result.json"
        result.unlink(missing_ok=True)
        args = self._box_args(box, slot, "pinky_read", result)
        if (n := pinky_number(pinky_id)) is not None:
            args += ["--pinky", n]
        if pre and pre.get("tray_ref_file"):
            args += ["--tray-ref", pre["tray_ref_file"]]
        rc, timed_out = self._sync_read("pinky_read", args)
        raw = _read_json(result) if not timed_out else None
        r = (raw or {}).get("result") or {}
        out = {
            k: v
            for k, v in r.items()
            if k not in ("ran", "box", "returncode", "skipped", "pinky_id", "pinky_no")
        }
        out.update(
            ran=True, returncode=rc, log=(raw or {}).get("log"), reported_at=_iso_ms()
        )
        if timed_out:
            out.update(on_pinky=None, on_shelf=None, skipped="시간 초과")
        elif raw is None:
            out["skipped"] = f"결과 없음 (종료 코드 {rc})"
        elif raw.get("early_exit"):
            out["skipped"] = "실행기가 중간에 끝남"
        if pre is not None:
            used_at = r.get("reference_at") or r.get("tray_reference_at")
            out["pre_mission"] = pre
            out["reference_at"] = used_at
            out["pre_mission_match"] = bool(
                r.get("tray_reference") == "pre_mission"
                and pre.get("reference_at")
                and used_at == pre.get("reference_at")
            )
        return out

    # ------------------------------------------------------------ 입고: 정렬
    def _home_arg(self, home: Mapping | None) -> list[str]:
        if not home or not isinstance(home.get("joints"), Mapping):
            return []
        path = self.c.dir() / "align_home_pose.json"
        _write_json(path, dict(home))
        return ["--home-pose", str(path)]

    def _clear_align_files(self) -> Path:
        d = self.c.dir()
        result = d / "align_result.json"
        for p in (
            result,
            d / "align_result.placement.json",
            d / "align_home_pose.json",
        ):
            p.unlink(missing_ok=True)
        return result

    def start_standby(
        self, box: int, slot: str | None, repo_id: str, home: Mapping | None
    ) -> None:
        """평가 시작과 함께. 정렬기는 넘겨받기를 기다리며 handover_ready.json 심박을 쓴다."""
        d = self.c.dir()
        for name in (
            "handover_stop",
            "handover_eval_done",
            "handover_state.json",
            "episode_end.json",
        ):
            (d / name).unlink(missing_ok=True)
        result = self._clear_align_files()
        args = self._box_args(box, slot, "shelf_align", result) + [
            "--handover-wait", str(d / "episode_end.json"),
            "--repo-id", repo_id, "--handover-dir", str(d),
        ] + self._home_arg(home)  # fmt: skip
        self.eval_done_at = None
        self._launch("standby", args, standby=True)

    def handover_ready(self) -> bool:
        """놓음 판정 순간 넘겨받을 수 있는가: 심박 파일이 1 s 안이고 그 프로세스가 살아 있다(omx_run과 같은 규칙)."""
        if self.kind != "standby" or not self.running():
            return False
        hb = _read_json(self.c.dir() / "handover_ready.json")
        if not hb:
            return False
        try:
            os.kill(int(hb["pid"]), 0)
        except (KeyError, TypeError, ValueError, ProcessLookupError, PermissionError):
            return False
        return time.time() - float(hb.get("ts", 0)) <= HEARTBEAT_S

    def write_episode_end(self, data: Mapping) -> None:
        _write_json(self.c.dir() / "episode_end.json", dict(data))

    def eval_done(self, hold_exit: bool, end_reason: str) -> None:
        """평가가 끝났다고 알린다. hold_exit가 아니면 handover_stop도 써서 기다림을 끝낸다."""
        d = self.c.dir()
        _write_json(
            d / "handover_eval_done",
            {
                "at": _iso_ms(),
                "ts": time.time(),
                "hold_exit": bool(hold_exit),
                "end_reason": end_reason,
            },
        )
        self.eval_done_at = (d / "handover_eval_done").stat().st_mtime
        if not hold_exit:
            self.handover_stop()

    def handover_stop(self) -> None:
        (self.c.dir() / "handover_stop").write_text(_iso_ms())

    def wait_taken(self) -> dict:
        """eval_done 뒤 taken_wait_s 안에 넘겨받기 결과. {"state": taken|regs_mismatch|stopped|not_placed|failed, ...}"""
        path = self.c.dir() / "handover_state.json"
        end = self.clock() + self.c.taken_wait_s
        while self.clock() < end:
            st = self._fresh_state(path)
            if st is not None:
                return st
            if not self.running():
                st = self._fresh_state(path)
                return st or {
                    "state": "failed",
                    "reason": f"aligner exited ({self.proc.returncode})",
                }
            self.sleep(0.05)
        self.handover_stop()
        if self.running():
            os.kill(self.proc.pid, signal.SIGTERM)
        return {
            "state": "failed",
            "reason": f"not taken within {self.c.taken_wait_s:g}s",
        }

    def _fresh_state(self, path: Path) -> dict | None:
        try:
            if path.stat().st_mtime < (self.eval_done_at or 0):
                return None
        except OSError:
            return None
        st = _read_json(path)
        return st if st and st.get("state") else None

    def start_align(self, box: int, slot: str | None, home: Mapping | None) -> None:
        """넘겨받기 없이 평가가 정상 종료된 뒤(팔은 이미 착지)."""
        result = self._clear_align_files()
        self._launch(
            "align",
            self._box_args(box, slot, "shelf_align", result) + self._home_arg(home),
        )

    def start_shelf_read(self, box: int, slot: str | None) -> None:
        """팔을 움직이지 않고 상자가 칸에 있는지만 읽는다(armed가 꺼졌거나 정렬을 시작하지 못한 경우)."""
        result = self._clear_align_files()
        self._launch("shelf_read", self._box_args(box, slot, "shelf_read", result))

    def start_release(self, box: int) -> None:
        """정렬이 중단된 팔을 기록한 경로로 되돌려 내려놓는다. rc 3 = 되돌아갈 길 없음(사람이 확인)."""
        self._launch("release", ["--box", str(int(box)), "--auto", "--release"])

    # ------------------------------------------------------------ 결과 해석
    def align_result(self, rc: int | None, stopped_by: str | None = None) -> dict:
        """Panel align_ended와 같은 last_result.align. rest(제자리 확인)는 실행부가 버스를 읽어 rest_check로 채운다."""
        raw = _read_json(self.c.dir() / "align_result.json") or {}
        r = raw.get("result") if isinstance(raw.get("result"), dict) else {}
        align = {
            "ran": True,
            "ok": bool(
                rc == 0
                and stopped_by is None
                and r.get("shelf") == "ok"
                and r.get("align") == "aligned"
            ),
            "returncode": rc,
            "stopped_by": stopped_by,
            "shelf": r.get("shelf"),
            "align": r.get("align"),
            "log": raw.get("log"),
            "ended_at": _iso_ms(),
            "released": (rc == 0) if stopped_by is None else None,
            "rest": None,
            "early_exit": bool(raw.get("early_exit")),
        }
        for k, v in r.items():
            align.setdefault(k, v)
        align.setdefault("on_shelf", None)
        align["reported_at"] = _iso_ms()
        return align

    def placement(self, source: str) -> dict | None:
        """정렬 전에 쓰는 .placement.json(없으면 결과 파일에서). source: align | shelf_read."""
        d = self.c.dir()
        data = _read_json(d / "align_result.placement.json")
        if data is not None:
            return dict(data, source=source, reported_at=_iso_ms())
        r = (_read_json(d / "align_result.json") or {}).get("result") or {}
        if r.get("on_shelf") is None:
            return None
        keep = (
            "on_shelf",
            "shelf",
            "slot",
            "box",
            "observed_at",
            "observed_ts",
            "observed_src",
        )
        return {
            **{k: r.get(k) for k in keep},
            "source": source,
            "from_result": True,
            "reported_at": _iso_ms(),
        }


def rest_reference(
    align: Mapping, episode_state: Mapping | None
) -> tuple[dict | None, str | None]:
    """제자리 확인 기준 자세(12개 이름): 정렬기 홈 → 정렬기 시작 자세 → 평가 끝 자세 (Panel 순서)."""
    if align.get("rest_target") == "home" and isinstance(
        align.get("home_pose"), Mapping
    ):
        return dict(align["home_pose"]), "home"
    if isinstance(align.get("start_pose"), Mapping):
        return dict(align["start_pose"]), "runner"
    if episode_state:
        return {k.removesuffix(".pos"): v for k, v in episode_state.items()}, "eval_end"
    return None, None


def rest_check(
    before: Mapping | None, after: Mapping[str, Mapping[str, float]] | None
) -> dict:
    """정렬 뒤 팔이 기준 자세에서 5 넘게 벗어난 관절(그리퍼 제외). after: {"left": {joint: v}, "right": ...}."""
    if not before or not after:
        return {"check": "unknown", "tolerance": REST_TOL, "arms": {}}
    arms, far_any = {}, False
    for side, name in (("left", "OMX 1"), ("right", "OMX 2")):
        joints = after.get(side)
        if not joints:
            return {"check": "unknown", "tolerance": REST_TOL, "arms": arms}
        far = [
            j for j, v in joints.items()
            if j != "gripper" and f"{side}_{j}" in before
            and abs(float(v) - float(before[f"{side}_{j}"])) > REST_TOL
        ]  # fmt: skip
        grip = joints.get("gripper")
        state = (
            None
            if grip is None
            else "open"
            if grip >= 57
            else "closed"
            if grip < 52
            else "half"
        )
        arms[name] = {
            "side": side,
            "joints": dict(joints),
            "gripper": grip,
            "gripper_state": state,
            "far_joints": far,
        }
        far_any = far_any or bool(far)
    return {"check": "far" if far_any else "ok", "tolerance": REST_TOL, "arms": arms}
