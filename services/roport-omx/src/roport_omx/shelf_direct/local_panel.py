"""Host 안에서 Panel HTTP API를 흉내 내는 로컬 구현. PanelExecutor의 _call이 HTTP 대신 이것을 부른다.

현장 v2 판정 규칙(panel_executor.PanelExecutor: 완료 근거·정지·HOLD·복구)을 고치지 않고 그대로 쓰기 위해,
그 코드가 읽는 응답 형식을 같은 의미로 만든다. 로봇 구동은 il3 venv의 worker 프로세스가 한다(core 환경에
lerobot을 섞지 않는다, roport-omx AGENTS.md).

- GET  /api/status        armed, job(running, info.repo_id/phase), last_result(worker status 파일), gpu_link(UDP 점검)
- GET  /api/arms?model=   worker arms로 버스 읽기(로봇 job이 없을 때만, 10 s 캐시) + 모델 홈 비교
- GET  /api/train/runs    경로표 모델 중 GPU 서버가 아는 것(LOAD _info_only)
- POST /api/eval/start    armed·장치 잠금 확인 → worker mission. 쟁반 확인 거절은 400 {"refused": "tray_check"}
- POST /api/stop          armed 끔 + worker에 SIGINT(평가는 착지, 정렬은 멈춘 뒤 되돌림)
- POST /api/home/return   armed·HOLD 확인 → worker home(실행 시작 자세로)
armed는 사람이 켠다. 지금은 이 PC의 로컬 명령(scripts/shelf_direct_arm.py → state_dir/arm-request.json)으로
켜고 끄며, 관제 허가는 다음 단계에서 연결한다. 정지·오류 결과·HOLD·부팅 때 꺼진다(Panel과 같음).
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import tempfile
import threading
import time
import urllib.parse
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from ..infer_link.client import InferenceClient, InferenceError, InferenceTimeout
from ..panel_executor import PanelRefused
from .devices import RobotBusy, RobotLock
from .infer import read_key
from .mission import arm_states

log = logging.getLogger(__name__)
VERSION = "3.21.1"  # 같은 의미로 응답하는 Panel 버전(PanelExecutor 버전 검사용)
ARMS_TTL_S = 10.0
HOME_TTL_S = 300.0
LINK_EVERY_S = 5.0
START_WAIT_S = 60.0  # 쟁반 확인(20 s)까지 기다린다
ARM_REQUEST_MAX_AGE_S = 60.0
SRC = Path(__file__).resolve().parents[2]  # roport_omx 패키지가 있는 src


@dataclass(frozen=True)
class DirectConfig:
    python: str = "~/venv/il3/bin/python"  # lerobot·OpenCV가 있는 venv
    ports: Mapping[str, str] = field(
        default_factory=lambda: {
            "left": "/dev/omx_follower_2",
            "right": "/dev/omx_follower_3",
        }
    )
    cameras: Mapping[str, str | int] = field(default_factory=dict)
    calibration_id: str = "omx_follower_arm"
    calibration_dir: str | None = None
    udp_server: tuple[str, int] = ("127.0.0.1", 5600)
    udp_key_file: str = "~/.config/roport/omx-udp.key"
    aligner_cmd: Sequence[str] = (
        "~/venv/omx_sim/bin/python",
        "~/il_ws/src/omx_sim_align/real_runner.py",
    )
    aligner_cwd: str = "~/il_ws/src/lerobot_v0.6.1"
    state_dir: str = "~/.local/state/roport/shelf-direct"
    lock_path: str = "~/omx_data/robot.lock"  # Panel shared_root와 같은 파일

    @classmethod
    def load(cls, path: str | Path) -> DirectConfig:
        raw = json.loads(Path(os.path.expanduser(str(path))).read_text())
        if "udp_server" in raw:
            host, port = raw["udp_server"]
            raw["udp_server"] = (str(host), int(port))
        return cls(**raw)

    def dir(self) -> Path:
        d = Path(os.path.expanduser(self.state_dir))
        d.mkdir(parents=True, exist_ok=True)
        return d


class LocalPanel:
    def __init__(
        self,
        config: DirectConfig,
        models: Sequence[str],
        *,
        spawn: Callable
        | None = None,  # (args, **popen kwargs) -> Popen. 시험에서 바꾼다
        model_info: Callable[[str], dict] | None = None,
        link_check: Callable[[], None] | None = None,
        lock: RobotLock | None = None,
    ):
        self.c, self.models = config, list(models)
        self.spawn = spawn or subprocess.Popen
        self.model_info = model_info or self._udp_model_info
        self.link_check = link_check or self._udp_ping
        self.lock = lock or RobotLock(config.lock_path)
        self.mu = threading.RLock()
        self.armed = {"on": False, "since": None, "off_reason": "부팅"}
        self.last_arm_request: str | None = None
        self._write_arm_state()  # 부팅: 꺼짐
        self.job: dict | None = (
            None  # {"kind", "proc", "repo_id", "status_path", "started"}
        )
        self.last_status: dict = {}
        self.last_job_end = 0.0
        self.arms_cache: tuple[float, dict | None, str | None] = (0.0, None, None)
        self.homes: dict[str, tuple[float, dict | None]] = {}
        self.link = {"ok": None, "at": None, "stage": "starting", "error": None}
        self._stop = threading.Event()
        threading.Thread(target=self._link_loop, daemon=True).start()

    # ------------------------------------------------------------ armed
    def set_armed(self, on: bool, reason: str = "") -> dict:
        with self.mu:
            if on and not self.armed["on"]:
                self.armed = {
                    "on": True,
                    "since": datetime.now().isoformat(timespec="seconds"),
                    "off_reason": None,
                }
            elif not on and self.armed["on"]:
                self.armed = {
                    "on": False,
                    "since": None,
                    "off_reason": reason or "꺼짐",
                }
            self._write_arm_state()
            return dict(self.armed)

    def _arm_paths(self) -> tuple[Path, Path]:
        d = self.c.dir()
        return d / "arm-request.json", d / "arm-state.json"

    def poll_arm_request(self) -> None:
        """로컬 명령 파일을 한 번 읽고 지운다. 60 s 넘은 요청은 쓰지 않는다."""
        req_path, _ = self._arm_paths()
        try:
            req = json.loads(req_path.read_text())
        except FileNotFoundError:
            return
        except (OSError, ValueError):
            req = None
        req_path.unlink(missing_ok=True)
        if not isinstance(req, dict) or not isinstance(req.get("armed"), bool):
            return
        self.last_arm_request = req.get("request_id")
        if time.time() - float(req.get("requested_at") or 0) > ARM_REQUEST_MAX_AGE_S:
            self._write_arm_state(refused="request too old")
            return
        if req["armed"] and self.job is not None:
            self._write_arm_state(refused="robot job running")
            return
        self.set_armed(req["armed"], f"로컬 명령 ({req.get('by') or 'operator'})")

    def _write_arm_state(self, refused: str | None = None) -> None:
        _, state_path = self._arm_paths()
        data = dict(
            self.armed,
            request_id=self.last_arm_request,
            refused=refused,
            at=time.time(),
        )
        tmp = state_path.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(data, ensure_ascii=False))
            os.replace(tmp, state_path)
        except OSError as exc:  # 화면용 파일: 쓰지 못해도 Host는 계속
            log.warning("arm state not written: %s", exc)

    # ------------------------------------------------------------ 요청
    def request(self, method: str, path: str, body: dict | None = None) -> dict:
        if (
            self._stop.is_set()
        ):  # Host가 끝났다: 남은 스레드가 상태 폴더를 다시 만들지 않게
            raise PanelRefused("local panel closed", {"error": "closed"})
        route, _, query = path.partition("?")
        if method == "GET" and route == "/api/status":
            return self.status()
        if method == "GET" and route == "/api/arms":
            model = urllib.parse.parse_qs(query).get("model", [""])[0]
            return self.arms(model)
        if method == "GET" and route == "/api/train/runs":
            return {
                "runs": [{"model_path": m} for m in self.models if self._model_known(m)]
            }
        if method == "POST" and route == "/api/eval/start":
            return self.eval_start(body or {})
        if method == "POST" and route == "/api/stop":
            return self.stop()
        if method == "POST" and route == "/api/home/return":
            return self.home_return(body or {})
        raise PanelRefused(
            f"local panel {path}: not supported", {"error": "not supported"}
        )

    def status(self) -> dict:
        with self.mu:
            self._reap()
            self.poll_arm_request()
            job = self.job
            st = (
                self._read_status(job["status_path"])
                if job and job.get("status_path")
                else self.last_status
            )
            running = job is not None
            info = {}
            if job:
                info = {
                    "kind": job["kind"],
                    "repo_id": job.get("repo_id"),
                    "phase": (st or {}).get("phase"),
                }
            return {
                "version": VERSION,
                "direct": True,
                "armed": self.armed["on"],
                "armed_since": self.armed["since"],
                "armed_off_reason": self.armed["off_reason"],
                "job": {"running": running, "info": info},
                "last_result": (st or {}).get("last_result"),
                "gpu_link": {"infer": {"transport": "udp", "udp": dict(self.link)}},
            }

    # ------------------------------------------------------------ 팔 읽기
    def arms(self, model: str) -> dict:
        out = {"source": None, "at": None, "arms": None, "error": None}
        with self.mu:
            self._reap()
            if self.job is not None:
                return out | {"source": "busy", "error": "robot job running"}
            t, current, error = self.arms_cache
            if self.last_job_end > t or time.time() - t >= ARMS_TTL_S:
                try:
                    current, error = self._run_worker("arms", {}), None
                except Exception as exc:  # noqa: BLE001 - 모르면 None(준비 안 됨)
                    current, error = None, str(exc)[:200]
                t = time.time()
                self.arms_cache = (t, current, error)
        out.update(
            source="bus",
            at=datetime.fromtimestamp(t).isoformat(timespec="seconds"),
            error=error,
        )
        if current is None:
            return out
        home = None
        if model:
            try:
                home = self._model_home(model)
            except Exception as exc:  # noqa: BLE001 - 홈을 모르면 at_home None
                out["error"] = f"model home unknown: {exc}"[:200]
        state = {
            f"{side}_{j}.pos": v
            for side, joints in current.items()
            for j, v in joints.items()
        }
        out["arms"] = arm_states(state, home)
        out["home_known"] = bool(home)
        return out

    # ------------------------------------------------------------ 시작·정지·복귀
    def eval_start(self, body: dict) -> dict:
        if body.get("source") != "mqtt":
            raise PanelRefused(
                "local panel runs control-tower evaluations only", {"error": "source"}
            )
        with self.mu:
            self._reap()
            self.poll_arm_request()
            if not self.armed["on"]:
                raise PanelRefused("armed off", {"error": "armed 꺼짐"})
            if self.job is not None:
                raise PanelRefused("robot job running", {"error": "busy"})
            try:
                self.lock.acquire("roport shelf-direct")
            except RobotBusy as exc:
                raise PanelRefused(str(exc), {"error": str(exc)}) from exc
            repo_id = f"direct_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
            status_path = self.c.dir() / f"{repo_id}.status.json"
            try:
                home = self._model_home(body["model_path"])
            except Exception as exc:  # noqa: BLE001 - 홈 없이도 평가는 지금 자세에서 시작(Panel과 같음)
                log.warning("model home unknown: %s", exc)
                home = None
            req = self._request(
                repo_id=repo_id,
                body=body,
                status_path=str(status_path),
                model_home=home,
            )
            proc = self._spawn_worker("mission", req)
            self.job = {
                "kind": "eval",
                "proc": proc,
                "repo_id": repo_id,
                "status_path": status_path,
                "started": time.time(),
            }
        end = time.monotonic() + START_WAIT_S
        while time.monotonic() < end:
            st = self._read_status(status_path) or {}
            if st.get("phase") == "refused":
                proc.wait(10)
                with self.mu:
                    self._reap()
                raise PanelRefused(
                    "tray check refused",
                    {"refused": "tray_check", "tray_check": st.get("tray_check")},
                )
            if (
                st.get("phase") not in (None, "starting", "tray_check")
                or proc.poll() is not None
            ):
                break
            time.sleep(0.1)
        return {"repo_id": repo_id}

    def stop(self) -> dict:
        with self.mu:
            self.set_armed(False, "중지")
            job = self.job
            if job is not None and job["proc"].poll() is None:
                os.kill(job["proc"].pid, signal.SIGINT)
        return {"ok": True}

    def home_return(self, body: dict) -> dict:
        with self.mu:
            self._reap()
            if body.get("source") == "mqtt" and not self.armed["on"]:
                raise PanelRefused(
                    "armed off", {"error": "armed 꺼짐", "refused": "armed"}
                )
            if self.job is not None:
                raise PanelRefused("robot job running", {"error": "busy"})
            st = self.last_status or {}
            if (st.get("last_result") or {}).get("stage") == "hold":
                raise PanelRefused(
                    "hold: a person checks first", {"error": "hold", "refused": "hold"}
                )
            poses = st.get("start_poses") or {}
            arms = {
                side: {"port": port, "joints": poses.get(side)}
                for side, port in self.c.ports.items()
            }
            try:
                self.lock.acquire("roport shelf-direct home")
            except RobotBusy as exc:
                raise PanelRefused(str(exc), {"error": str(exc)}) from exc
            proc = self._spawn_worker("home", self._request(arms=arms))
            self.job = {
                "kind": "home",
                "proc": proc,
                "repo_id": None,
                "status_path": None,
                "started": time.time(),
            }
        return {"ok": True}

    # ------------------------------------------------------------ worker
    def _request(self, **extra) -> dict:
        c = self.c
        return {
            "robot": {
                "ports": dict(c.ports),
                "cameras": dict(c.cameras),
                "calibration_id": c.calibration_id,
                "calibration_dir": c.calibration_dir,
            },
            "inference": {"server": list(c.udp_server), "key_file": c.udp_key_file},
            "aligner": {
                "cmd": list(c.aligner_cmd),
                "work_dir": str(c.dir() / "aligner"),
                "cwd": c.aligner_cwd,
            },
            **extra,
        }

    def _write_request(self, req: dict) -> Path:
        fd, path = tempfile.mkstemp(prefix="req_", suffix=".json", dir=self.c.dir())
        with os.fdopen(fd, "w") as f:
            json.dump(req, f, ensure_ascii=False)
        return Path(path)

    def _worker_args(self, cmd: str, req_path: Path) -> list[str]:
        return [
            os.path.expanduser(self.c.python),
            "-m",
            "roport_omx.shelf_direct.worker",
            cmd,
            str(req_path),
        ]

    def _worker_env(self) -> dict:
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(
            filter(None, [str(SRC), env.get("PYTHONPATH")])
        )
        return env

    def _spawn_worker(self, cmd: str, req: dict):
        req_path = self._write_request(req)
        name = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{cmd}.log"
        logf = open(self.c.dir() / name, "w")  # noqa: SIM115 - 자식 프로세스가 이어서 쓴다
        try:
            return self.spawn(
                self._worker_args(cmd, req_path),
                env=self._worker_env(),
                stdin=subprocess.DEVNULL,
                stdout=logf,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        finally:
            logf.close()

    def _run_worker(self, cmd: str, req: dict, timeout: float = 15.0) -> dict:
        req_path = self._write_request(self._request(**req))
        try:
            out = subprocess.run(
                self._worker_args(cmd, req_path),
                env=self._worker_env(),
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        finally:
            req_path.unlink(missing_ok=True)
        if out.returncode != 0:
            tail = (out.stderr or out.stdout).strip().splitlines()
            raise RuntimeError(
                tail[-1] if tail else f"worker {cmd} rc {out.returncode}"
            )
        return json.loads(out.stdout.strip().splitlines()[-1])

    def _reap(self) -> None:
        """끝난 job을 정리한다: 잠금 풀기, 결과 보존, 오류로 끝났으면 armed 끔(Panel과 같음)."""
        job = self.job
        if job is None or job["proc"].poll() is None:
            return
        rc = job["proc"].returncode
        if job.get("status_path"):
            st = self._read_status(job["status_path"]) or {}
            res = st.get("last_result") or {}
            if not st or st.get("running"):  # 상태를 남기지 못하고 끝남(강제 종료·충돌)
                res = dict(res or {}, repo_id=job["repo_id"], exit_ok=False, stage="done", final=True,
                           err={"code": "WORKER_EXITED", "name": f"rc {rc}"})  # fmt: skip
                st = dict(st, last_result=res, running=False)
            self.last_status = st
            if res.get("err") or res.get("stage") == "hold":
                self.set_armed(False, "이상 종료" if res.get("err") else "HOLD")
        self.lock.release()
        self.job, self.last_job_end = None, time.time()

    @staticmethod
    def _read_status(path) -> dict | None:
        try:
            return json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return None

    # ------------------------------------------------------------ GPU 서버
    def _client(self) -> InferenceClient:
        return InferenceClient(tuple(self.c.udp_server), read_key(self.c.udp_key_file))

    def _udp_model_info(self, model: str) -> dict:
        client = self._client()
        try:
            return client.load(model, {"_info_only": True}, timeout=10.0, attempts=2)
        finally:
            client.close()

    def _model_known(self, model: str) -> bool:
        try:
            self._model_home(model)
            return True
        except Exception:  # noqa: BLE001 - 모르면 모델 없음(MODEL_NOT_READY)
            return False

    def _model_home(self, model: str) -> dict | None:
        hit = self.homes.get(model)
        if hit and time.time() - hit[0] < HOME_TTL_S:
            return hit[1]
        info = self.model_info(model)
        home = info.get("home")
        self.homes[model] = (time.time(), home)
        return home

    def _udp_ping(self) -> None:
        client = self._client()
        try:
            client.ping(timeout=1.0)
        finally:
            client.close()

    def _link_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.link_check()
                self.link = {
                    "ok": True,
                    "at": time.time(),
                    "stage": "ok",
                    "error": None,
                }
            except (InferenceTimeout, InferenceError, OSError, ValueError) as exc:
                self.link = {
                    "ok": False,
                    "at": time.time(),
                    "stage": "ping",
                    "error": str(exc)[:200],
                }
            self._stop.wait(LINK_EVERY_S)

    def close(self) -> None:
        self._stop.set()
        with self.mu:
            if self.job is not None and self.job["proc"].poll() is None:
                os.kill(self.job["proc"].pid, signal.SIGINT)
