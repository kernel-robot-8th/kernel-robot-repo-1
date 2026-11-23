"""ports.Robot 구현: OMX1(left)·OMX2(right) 두 대의 LeRobot OmxFollower + 카메라 4대.

Panel(omx_bimanual.BiOmxFollower + omx_safe_start)과 같은 장치 설정을 쓴다.
- 두 팔 모두 robot id "omx_follower_arm"(공장 보정, 같은 캘리브레이션 파일). 학습 데이터와 같은 관절값을 얻으려면
  Panel과 같은 id·calibration_dir이어야 한다.
- 연결 순서: preflight(읽기만) → 팔(connect(calibrate=False): 보정은 쓰지 않고 운전 모드·PID·토크는 쓴다)
  → 카메라. 한 팔 연결이 실패하면 이미 연결한 팔은 출력을 줄이고 토크를 끈다.
- 착지(soft)는 Panel처럼 왼팔 다음 오른팔 순서, 빠른 착지(fast)는 두 팔 동시.

connect()는 모터를 움직일 수 있다(토크 on). 현장 안전 확인과 사용자 지시 없이 부르지 않는다.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

from .contract import CAMERAS, JOINTS, SIDES, join, split
from .ports import LandMode, Observation, RobotNotReady
from .safety import DEFAULT_SAFETY, PWM_MAX, Mover, SafeStart, SafetySettings, preflight

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RobotConfig:
    ports: Mapping[str, str] = field(
        default_factory=lambda: {
            "left": "/dev/omx_follower_2",
            "right": "/dev/omx_follower_3",
        }
    )
    calibration_id: str = "omx_follower_arm"
    calibration_dir: str | None = None  # None: LeRobot 기본 캘리브레이션 폴더
    cameras: Mapping[str, str | int] = field(
        default_factory=dict
    )  # 이름 → /dev/video* 또는 index
    width: int = 640
    height: int = 480
    fps: int = 30
    fourcc: str = "MJPG"
    warmup_s: float = 3.0
    read_retries: int = 3

    def __post_init__(self):
        if set(self.ports) != set(SIDES):
            raise ValueError("ports needs left and right")
        missing = [c for c in CAMERAS if c not in self.cameras]
        if missing:
            raise ValueError(f"cameras missing: {missing}")


class LeRobotArm:
    """ports.ArmIO 구현: OmxFollower의 모터 버스."""

    def __init__(self, side: str, follower, read_retries: int = 3):
        self.side, self.follower, self.bus = side, follower, follower.bus
        self.read_retries = read_retries
        self.pwm_ratio = 1.0

    def read(self) -> dict[str, float]:
        for left in range(self.read_retries, -1, -1):
            try:
                return {
                    m: float(v)
                    for m, v in self.bus.sync_read("Present_Position").items()
                }
            except ConnectionError:
                if not left or not self.bus.is_connected:
                    raise
                time.sleep(0.005)
                ser = getattr(self.bus.port_handler, "ser", None)
                if ser is not None:
                    ser.reset_input_buffer()  # 깨진 패킷의 남은 바이트
        raise AssertionError("unreachable")

    def write_goals(self, goals: Mapping[str, float]) -> None:
        self.bus.sync_write("Goal_Position", {m: float(v) for m, v in goals.items()})

    def set_pwm(self, ratio: float) -> None:
        self.pwm_ratio = ratio
        for motor in self.bus.motors:
            try:
                self.bus.write("Goal_PWM", motor, int(PWM_MAX * ratio), normalize=False)
            except Exception as exc:  # noqa: BLE001 - Panel과 같이 그 모터만 건너뛴다
                log.debug("%s %s Goal_PWM: %s", self.side, motor, exc)

    def torque_off(self) -> None:
        self.bus.disable_torque()

    def reboot(self, joint: str) -> None:
        self.bus.packet_handler.reboot(self.bus.port_handler, self.bus.motors[joint].id)
        time.sleep(0.6)
        self.bus.write(
            "Goal_PWM", joint, int(PWM_MAX * self.pwm_ratio), normalize=False
        )
        self.bus.enable_torque(joint)

    def close(self, *, torque_off: bool) -> None:
        if self.bus.is_connected:
            self.bus.disconnect(disable_torque=torque_off)


def lerobot_arm(side: str, port: str, config: RobotConfig) -> LeRobotArm:
    from lerobot.robots.omx_follower import OmxFollower, OmxFollowerConfig

    follower = OmxFollower(
        OmxFollowerConfig(
            id=config.calibration_id,
            calibration_dir=config.calibration_dir,
            port=port,
            disable_torque_on_disconnect=False,  # 착지는 Mover가 한다
        )
    )
    follower.connect(calibrate=False)
    return LeRobotArm(side, follower, config.read_retries)


def lerobot_cameras(config: RobotConfig) -> dict:
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.cameras.utils import make_cameras_from_configs

    cams = make_cameras_from_configs(
        {
            name: OpenCVCameraConfig(
                index_or_path=config.cameras[name],
                fps=config.fps,
                width=config.width,
                height=config.height,
                fourcc=config.fourcc,
                warmup_s=config.warmup_s,
            )
            for name in CAMERAS
        }
    )
    for cam in cams.values():
        cam.connect()
    return cams


class BiOmxRobot:
    def __init__(
        self,
        config: RobotConfig,
        safety: SafetySettings = DEFAULT_SAFETY,
        *,
        arm_factory: Callable[[str, str, RobotConfig], object] = lerobot_arm,
        camera_factory: Callable[[RobotConfig], dict] = lerobot_cameras,
        preflight_fn: Callable[[Sequence[tuple[str, str]]], list[str]] = preflight,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        log: Callable[[str], None] = lambda _msg: None,
        lifter=None,
    ):
        self.config, self.safety = config, safety
        self.arm_factory, self.camera_factory = arm_factory, camera_factory
        self.preflight_fn, self.clock, self.sleep, self.log = (
            preflight_fn,
            clock,
            sleep,
            log,
        )
        self.lifter = lifter
        self.freeze, self.release = threading.Event(), threading.Event()
        self.arms: dict = {}
        self.cameras: dict = {}
        self.starts: dict[str, SafeStart] = {}

    # ------------------------------------------------------------ connect
    def connect(self) -> None:
        problems = self.preflight_fn(
            [(side, self.config.ports[side]) for side in SIDES]
        )
        if problems:
            raise RobotNotReady("; ".join(problems))
        try:
            for side in SIDES:
                self.arms[side] = self.arm_factory(
                    side, self.config.ports[side], self.config
                )
            for side in SIDES:
                ss = SafeStart(self.arms[side], self.safety, self.clock, self.log)
                ss.begin()  # 출력 제한
                self.starts[side] = ss
            self.cameras = self.camera_factory(self.config)
        except Exception as exc:
            for (
                arm
            ) in self.arms.values():  # 연결된 팔은 제자리에서 출력을 줄이고 토크 끔
                try:
                    self._mover(arm).fast_land()
                    arm.close(torque_off=True)
                except Exception as land_exc:  # noqa: BLE001 - 다른 팔도 내려야 한다
                    self.log(f"{arm.side}: stop after failed connect: {land_exc}")
            self.arms, self.starts = {}, {}
            if isinstance(exc, RobotNotReady):
                raise
            raise RobotNotReady(f"connect failed: {exc}") from exc

    def _mover(self, arm) -> Mover:
        return Mover(
            arm,
            self.safety,
            freeze=self.freeze,
            clock=self.clock,
            sleep=self.sleep,
            log=self.log,
            lifter=self.lifter,
        )

    # ------------------------------------------------------------ loop
    def observe(self) -> Observation:
        arms = {side: self.arms[side].read() for side in SIDES}
        images = {name: cam.read_latest() for name, cam in self.cameras.items()}
        return Observation(state=tuple(join(arms)), images=images, at=time.time())

    def send(self, action: Sequence[float]) -> None:
        parts = split(action)
        goals = {side: self.starts[side].filter(parts[side]) for side in SIDES}
        for side in SIDES:
            self.arms[side].write_goals(goals[side])

    def start_poses(self) -> dict[str, dict[str, float]]:
        return {
            s: dict(ss.home) for s, ss in self.starts.items() if ss.home is not None
        }

    def read_joints(self) -> dict[str, dict[str, float]]:
        """버스에서 바로 읽은 양팔 관절(정지 확인용)."""
        out = {}
        for side, arm in self.arms.items():
            values = arm.read()
            out[side] = {j: values[j] for j in JOINTS}
        return out

    # ------------------------------------------------------------ end
    def land(self, mode: LandMode) -> None:
        if mode == "hold" and not self.freeze.is_set():
            self.log("handover: ports closed with torque on")
            for arm in self.arms.values():
                arm.close(torque_off=False)
            return
        homes = self.start_poses()
        if mode == "fast" and not self.freeze.is_set():
            threads = [
                threading.Thread(target=self._land_one, args=(arm, "fast", None))
                for arm in self.arms.values()
            ]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        else:
            for side in SIDES:  # Panel과 같은 순서: 왼팔 다음 오른팔
                if side in self.arms:
                    self._land_one(self.arms[side], "soft", homes.get(side))
        self.release.clear()

    def _land_one(self, arm, mode: str, home) -> None:
        mover = self._mover(arm)
        try:
            if mode == "fast":
                mover.fast_land()
            else:
                mover.soft_land(home, self.release)
        except Exception as exc:  # noqa: BLE001 - 다른 팔은 계속 착지해야 한다
            self.log(f"{arm.side}: landing error ({exc})")
        finally:
            arm.close(torque_off=True)

    def disconnect(self) -> None:
        for cam in self.cameras.values():
            try:
                if cam.is_connected:
                    cam.disconnect()
            except Exception as exc:  # noqa: BLE001 - 나머지 카메라도 닫는다
                self.log(f"camera disconnect: {exc}")
        self.cameras = {}
        self.arms, self.starts = {}, {}
