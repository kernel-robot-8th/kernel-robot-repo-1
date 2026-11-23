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

# 종료(close): Runner는 매 주기(30 fps) 취소를 보지만 관측·추론 한 번이 최대 약 1초 걸린다
CLOSE_RUNNER_S = 5.0
CLOSE_STOP_S = 3.0
CLOSE_POLL_S = 0.1


class StopWatch:
    """Robot 감싸기: Runner·executor가 이미 부르는 observe()의 관절 값으로 정지를 판정한다.

    추가 버스 읽기를 하지 않는다(실행 중 모터 버스는 Runner가 소유).
    hold()에서 이전 표본을 버린다: 취소·실패·완료는 모두 hold()를 거치므로, 정지 구간은
    hold 이후의 관측으로만 만들어진다(정지 요청 전 표본으로 완료하지 않게, #149 리뷰).
    """

    def __init__(self, robot, clock=time.monotonic, now=lambda: datetime.now(UTC)):
        self.robot, self.clock, self.now = robot, clock, now
        self._samples: deque = deque()
        self._lock = threading.Lock()

    def ready(self):
        return self.robot.ready()

    def send(self, action):
        self.robot.send(action)

    def hold(self):
        try:
            self.robot.hold()
        finally:
            with self._lock:
                self._samples.clear()

    def observe(self):
        obs = self.robot.observe()
        t = self.clock()
        with self._lock:
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
    """WarehouseExecutor + 종료 경로: field_host.serve()가 끝날 때 부르는 close()."""

    def __init__(self, runner, *, devices=(), sleep=time.sleep, **kwargs):
        super().__init__(runner, **kwargs)
        self.devices, self.sleep = (
            devices,
            sleep,
        )  # close()로 닫을 것(추론 클라이언트·팔)

    def close(
        self,
        runner_s=CLOSE_RUNNER_S,
        stop_s=CLOSE_STOP_S,
        clock=time.monotonic,
    ):
        """Runner 취소 → 끝날 때까지 대기 → 정지 근거 확인 → 연결 정리. 토크는 끄지 않는다."""
        self.abort()  # 실행 중이면 취소 신호, 아니면 바로 hold
        self.host.wait(runner_s)  # Runner가 취소를 보고 hold한 뒤 끝난다
        if self.host.status()["active"] is not None:
            # 아직 Runner가 버스를 쓰는 중: 연결을 끊으면 hold를 끊을 수 있다. 프로세스 종료에 맡긴다
            log.error(
                "OMX3 runner still active after %.1fs; leaving devices open", runner_s
            )
            return
        deadline = clock() + stop_s
        while not self.observe().get("stopped"):
            if clock() >= deadline:
                log.warning("OMX3 stop not confirmed within %.1fs", stop_s)
                break
            try:
                self.runner.robot.observe()  # Runner가 끝났으니 버스를 읽어도 된다
            except Exception:  # 읽기 실패는 정지 미확인으로 남긴다
                log.exception("OMX3 observe failed while confirming stop")
                break
            self.sleep(CLOSE_POLL_S)
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
    settings = Omx3Settings()
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
        devices=(client, robot),
        marker_ids=marker_ids,
        model_version=model_version(model),
        calibration_version=calibration,
        package_sizes=sizes,
        stop_status=watch.status,
    )
    return {"omx3": executor}, None
