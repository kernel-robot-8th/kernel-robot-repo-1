"""field_host factory: ``--factory roport_omx.omx3.field:build`` → ({"omx3": WarehouseExecutor}, None).

N01 상차는 정렬 센서를 쓰지 않는다(README 「Factory 정렬 센서」). 환경변수:
- ROPORT_OMX3_INFER_SERVER: GPU 추론 서버 ``host:port`` (필수)
- ROPORT_OMX3_MODEL: GPU PC의 체크포인트 경로(``.../pretrained_model``, 필수)
- ROPORT_OMX3_PORT, ROPORT_OMX3_CAMERA, ROPORT_OMX3_KEY_FILE: 기본값은 아래 상수

build()는 모델을 GPU 서버에 올린 뒤 팔을 연결한다(connect = 모터 모드·PID·토크 쓰기).
field_host의 --hardware-authorized 뒤에만, 사람이 팔 옆에 있을 때 부른다.
"""

from __future__ import annotations

import hashlib
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

DEFAULT_PORT = "/dev/omx_follower"
DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
DEFAULT_KEY = "~/.config/roport/omx-udp.key"

# 정지 판정. 2026-11-20 토크 켠 정지 상태 실측: 0.5초 창 관절 변화 최대 0.049(정규화 단위)
STOP_SPAN_S = 0.5  # 이 시간 이상 이어진 관측이어야 정지로 본다
STOP_RANGE = 0.3  # 그 동안 모든 관절의 (최대 - 최소)가 이 값 이하
STOP_WINDOW_S = 1.0  # 최근 이 시간의 관측만 남긴다


class StopWatch:
    """Robot 감싸기: Runner·executor가 이미 부르는 observe()의 관절 값으로 정지를 판정한다.

    추가 버스 읽기를 하지 않는다(실행 중 모터 버스는 Runner가 소유).
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
        self.robot.hold()

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


def model_version(model: str) -> str:
    path = Path(model)
    return (path.parent if path.name == "pretrained_model" else path).name


def calibration_version(robot_id: str, path: str | Path) -> str:
    """캘리브레이션 파일 내용의 sha256 앞 8자리: 파일이 바뀌면 값도 바뀐다."""
    return f"{robot_id}-{hashlib.sha256(Path(path).read_bytes()).hexdigest()[:8]}"


def build():
    env = os.environ
    host, _, port = env["ROPORT_OMX3_INFER_SERVER"].rpartition(":")
    model = env["ROPORT_OMX3_MODEL"]
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
    settings = Omx3Settings()
    watch = StopWatch(robot)
    runner = Runner(watch, ChunkPolicy(client, settings), ArucoTargetMarker, settings)
    executor = WarehouseExecutor(
        runner,
        # 관제 package_id(문자열) = 상자 ArUco 번호(관제와 합의)
        marker_ids={str(i): i for i in sorted(settings.package_ids)},
        model_version=model_version(model),
        calibration_version=calibration,
        stop_status=watch.status,
    )
    return {"omx3": executor}, None
