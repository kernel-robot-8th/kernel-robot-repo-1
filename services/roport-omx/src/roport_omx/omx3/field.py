"""field_host factory: ``--factory roport_omx.omx3.field:build`` → ({"omx3": FieldWarehouseExecutor}, None).

N01 상차는 정렬 센서를 쓰지 않는다(README 「Factory 정렬 센서」). 환경변수:
- ROPORT_OMX3_INFER_SERVER: GPU 추론 서버 ``host:port`` (필수)
- ROPORT_OMX3_MODEL: GPU PC의 체크포인트 경로(``.../pretrained_model``, 필수)
- ROPORT_OMX3_PACKAGE_SIZES: 상자 크기 JSON 파일 ``{"15": "small", ...}`` (필수, 학습한 마커 전부)
- ROPORT_OMX3_PORT, ROPORT_OMX3_CAMERA, ROPORT_OMX3_KEY_FILE: 기본값은 아래 상수

build()는 모델을 GPU 서버에 올린 뒤 팔을 연결한다(connect = 모터 모드·PID·토크 쓰기).
field_host의 --hardware-authorized 뒤에만, 사람이 팔 옆에 있을 때 부른다.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import threading
import time
from collections import deque
from datetime import UTC, datetime
from pathlib import Path

from ..field_executors import WarehouseExecutor
from .inference import ChunkPolicy
from .runner import Runner
from .settings import Omx3Settings

log = logging.getLogger(__name__)

DEFAULT_PORT = "/dev/omx_follower"
DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
DEFAULT_KEY = "~/.config/roport/omx-udp.key"
SIZES = frozenset({"small", "large"})  # 관제 select_slot이 받는 값

# 정지 판정. 2026-11-20 토크 켠 정지 상태 실측: 0.5초 창 관절 변화 최대 0.049(정규화 단위)
STOP_SPAN_S = 0.5  # 이 시간 이상 이어진 관측이어야 정지로 본다
STOP_RANGE = 0.3  # 그 동안 모든 관절의 (최대 - 최소)가 이 값 이하
STOP_WINDOW_S = 1.0  # 최근 이 시간의 관측만 남긴다

# 정지 확인: Runner가 끝난 뒤(완료·취소) executor가 직접 관측을 모으는 간격과 한도
FEED_S = 0.1
CONFIRM_S = 3.0  # 완료 뒤 이 시간 안에 정지가 확인되지 않으면 OMX3_STOP_UNCONFIRMED
CLOSE_LOG_S = 5.0  # 종료 때 Runner가 안 끝나면 이 간격으로 로그를 남기며 계속 기다린다
STOP_UNCONFIRMED = "OMX3_STOP_UNCONFIRMED"


class StopWatch:
    """Robot 감싸기: Runner·executor가 부르는 observe()의 관절 값으로 정지를 판정한다.

    정지 구간은 hold() 이후의 관측으로만 만든다(#149 리뷰). invalidate()(정지 요청)부터
    hold()가 끝날 때까지 들어온 관측은 버린다: abort 뒤 hold 전에 끝난 관측이 이전 표본과
    이어져 정지로 판정되지 않게 한다. 취소·실패·VERIFY는 모두 Runner의 hold()를 거친다.
    """

    def __init__(self, robot, clock=time.monotonic, now=lambda: datetime.now(UTC)):
        self.robot, self.clock, self.now = robot, clock, now
        self._samples: deque = deque()
        self._accepting = True
        self._lock = threading.Lock()

    def ready(self):
        return self.robot.ready()

    def send(self, action):
        self.robot.send(action)

    def invalidate(self):
        """정지 요청: 지금까지의 표본을 버리고, 다음 hold()까지 새 표본도 받지 않는다."""
        with self._lock:
            self._samples.clear()
            self._accepting = False

    def hold(self):
        # 실패한 hold 뒤에는 표본을 받지 않는다: 마지막 구동 목표가 남아 있을 수 있다(#149 리뷰)
        self.invalidate()
        self.robot.hold()
        with self._lock:
            self._samples.clear()
            self._accepting = True

    def observe(self):
        obs = self.robot.observe()
        t = self.clock()
        with self._lock:
            if self._accepting:
                self._samples.append((t, self.now(), tuple(obs.state)))
                while t - self._samples[0][0] > STOP_WINDOW_S:
                    self._samples.popleft()
        return obs

    def status(self):
        """WarehouseExecutor stop_status: observed_at은 마지막 관측 시각(지금 시각이 아님)."""
        with self._lock:
            samples = list(self._samples)
        if not samples:
            return {"stopped": False, "observed_at": None}
        span = samples[-1][0] - samples[0][0]
        joints = list(zip(*(s[2] for s in samples), strict=True))
        still = all(max(j) - min(j) <= STOP_RANGE for j in joints)
        return {
            "stopped": span >= STOP_SPAN_S and still,
            "observed_at": samples[-1][1].isoformat(),
        }


class FieldWarehouseExecutor(WarehouseExecutor):
    """WarehouseExecutor + 정지 확인 + 종료 경로(field_host.serve()가 끝날 때 부르는 close())."""

    def __init__(
        self,
        runner,
        *,
        watch,
        devices=(),
        clock=time.monotonic,
        sleep=time.sleep,
        **kwargs,
    ):
        super().__init__(runner, stop_status=watch.status, **kwargs)
        self.watch, self.devices = watch, devices  # devices: close()로 닫을 것
        self.clock, self.sleep = clock, sleep
        self._finished_at = None
        self._fed_at = None

    def start(self, task):
        self._finished_at = None
        super().start(task)

    def abort(self):
        self.watch.invalidate()  # 정지 근거는 이 요청 뒤의 hold부터 다시 모은다
        super().abort()

    def _feed(self):
        """Runner가 끝났으면 직접 관측해 정지 표본을 채운다(Runner가 버스를 놓은 뒤에만)."""
        if self.host.status()["active"] is not None:
            return
        now = self.clock()
        if self._fed_at is None or now - self._fed_at >= FEED_S:
            self._fed_at = now
            self.runner.robot.observe()

    def observe(self):
        if self.stop_requested_at is not None:
            try:
                self._feed()
            except Exception:  # 읽기 실패는 정지 미확인으로 남는다
                log.exception("OMX3 observe failed while confirming stop")
            return super().observe()
        result = super().observe()
        if result.get("state") != "finished" or result["arms"]["omx3"]["stopped"]:
            return result
        # 완료(VERIFY)는 hold 직후라 정지 표본이 아직 없다: 모일 때까지 완료를 보류한다
        if self._finished_at is None:
            self._finished_at = self.clock()
        if self.clock() - self._finished_at > CONFIRM_S:
            return {"state": "failed", "error": STOP_UNCONFIRMED}
        try:
            self._feed()
        except Exception:
            log.exception("OMX3 observe failed while confirming stop")
            return {"state": "failed", "error": STOP_UNCONFIRMED}
        result = super().observe()
        if result.get("state") == "finished" and result["arms"]["omx3"]["stopped"]:
            return result
        return {"state": "running", "step": "VERIFY"}

    def close(self, stop_s=CONFIRM_S):
        """Runner 취소 → 끝날 때까지 대기 → 정지 확인 → 연결 정리. 토크는 끄지 않는다.

        정지가 확인될 때까지 반환하지 않고 장치 소유권을 유지한다: 반환하면 프로세스가 끝나
        hold 없이 장치가 남는다. Runner가 안 끝나면 기다리고, 끝난 뒤 정지가 확인되지 않으면
        stop_s마다 hold를 다시 쓴다(Runner가 끝났으니 버스를 다투지 않는다). 그래도 멈추지
        않으면 현장에서 비상 정지·강제 종료로 인계한다.
        """
        # hold를 다시 쓰면 표본을 지우므로 재시도 간격이 판정 구간보다 길어야 확인할 수 있다
        stop_s = max(stop_s, STOP_SPAN_S + 0.2)
        try:
            self.abort()  # 실행 중이면 취소 신호, 아니면 바로 hold
        except Exception:  # hold 실패: 아래에서 다시 쓴다
            log.exception("OMX3 abort failed while closing")
        while True:
            self.host.wait(CLOSE_LOG_S)
            if self.host.status()["active"] is None:
                break
            log.error(
                "OMX3 runner still active; keeping device ownership until it stops"
            )
        deadline = self.clock() + stop_s
        while not self.observe().get("stopped"):
            if self.clock() >= deadline:
                log.error(
                    "OMX3 stop not confirmed; rewriting hold and keeping device ownership"
                )
                try:
                    self.watch.hold()
                except Exception:
                    log.exception("OMX3 hold failed while closing")
                deadline = self.clock() + stop_s
            self.sleep(FEED_S)
        log.info("OMX3 stop confirmed; closing devices")
        for device in self.devices:
            try:
                (getattr(device, "close", None) or device.disconnect)()
            except Exception:  # 나머지 장치도 닫는다
                log.exception("OMX3 device close failed")


def model_version(model: str) -> str:
    path = Path(model)
    return (path.parent if path.name == "pretrained_model" else path).name


def calibration_version(robot_id: str, path: str | Path) -> str:
    """캘리브레이션 파일 내용의 sha256 앞 8자리: 파일이 바뀌면 값도 바뀐다."""
    return f"{robot_id}-{hashlib.sha256(Path(path).read_bytes()).hexdigest()[:8]}"


def package_sizes(path: str | Path, marker_ids: dict[str, int]) -> dict[str, str]:
    """관제 선반 배정에 필요한 상자 크기. 빠진 상자가 있으면 상차 전에(시작할 때) 거절한다."""
    sizes = json.loads(Path(path).expanduser().read_text())
    if not isinstance(sizes, dict) or set(sizes) != set(marker_ids):
        raise ValueError(f"package sizes must list exactly {sorted(marker_ids)}")
    if any(v not in SIZES for v in sizes.values()):
        raise ValueError(f"package sizes must be one of {sorted(SIZES)}")
    return sizes


def build():
    env = os.environ
    host, _, port = env["ROPORT_OMX3_INFER_SERVER"].rpartition(":")
    model = env["ROPORT_OMX3_MODEL"]
    # 패널 "부드럽게 실행"(시간 앙상블, 기본 켜짐)과 같은 동작: 이 모델은 그 설정으로 평가했다
    settings = Omx3Settings(ensemble_coeff=0.01)
    # 관제 package_id(문자열) = 상자 ArUco 번호(관제와 합의)
    marker_ids = {str(i): i for i in sorted(settings.package_ids)}
    sizes = package_sizes(env["ROPORT_OMX3_PACKAGE_SIZES"], marker_ids)
    # lerobot·cv2는 장비 PC 환경에만 있다
    from .adapters.lerobot_robot import LeRobotOmx
    from .adapters.marker import ArucoTargetMarker
    from .adapters.udp_infer import UdpInferClient, read_key

    robot = LeRobotOmx(
        env.get("ROPORT_OMX3_PORT", DEFAULT_PORT),
        env.get("ROPORT_OMX3_CAMERA", DEFAULT_CAMERA),
    )
    calibration = calibration_version(robot.robot.id, robot.robot.calibration_fpath)
    client = UdpInferClient(
        (host, int(port)),
        read_key(env.get("ROPORT_OMX3_KEY_FILE", DEFAULT_KEY)),
        model,
    )
    try:
        client.load()  # 주행 전에 올린다(다른 Host의 추론을 막지 않게). 모터는 아직 안 쓴다
        robot.connect()  # 실패하면 스스로 연 것을 닫는다
    except BaseException:
        client.close()
        raise
    watch = StopWatch(robot)
    runner = Runner(watch, ChunkPolicy(client, settings), ArucoTargetMarker, settings)
    executor = FieldWarehouseExecutor(
        runner,
        watch=watch,
        devices=(client, robot),
        marker_ids=marker_ids,
        model_version=model_version(model),
        calibration_version=calibration,
        package_sizes=sizes,
    )
    return {"omx3": executor}, None
