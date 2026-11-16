"""ports.Robot 구현: 저장소 lerobot/(v0.6.1)의 OmxFollower + 정면 OpenCV 카메라.

학습 데이터와 같은 관절 값을 얻으려면 패널과 같은 robot_id·calibration_dir을 쓴다
(기본값 omx_follower_arm, LeRobot 기본 캘리브레이션 폴더. 패널 데이터 폴더의 사본과 같은 파일).
모델은 정면 카메라만 쓴다(시뮬 학습 데이터에 wrist 영상이 없음).

connect()는 캘리브레이션을 쓰지 않지만(calibrate=False) 모터 운전 모드·PID·토크를 쓴다.
현장 안전 확인과 사용자 지시 없이 부르지 않는다.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.omx_follower import OmxFollower, OmxFollowerConfig

from ..ports import Observation
from ..settings import JOINTS

GRIPPER = JOINTS.index("gripper")


class LeRobotOmx:
    def __init__(
        self,
        port: str,
        front_camera: str | int,
        robot_id: str = "omx_follower_arm",
        calibration_dir: str | Path | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ):
        front = OpenCVCameraConfig(
            index_or_path=front_camera,
            fps=fps,
            width=width,
            height=height,
            fourcc="MJPG",
        )
        self.robot = OmxFollower(
            OmxFollowerConfig(
                port=port,
                id=robot_id,
                calibration_dir=Path(calibration_dir) if calibration_dir else None,
                cameras={"front": front},
                # 프로세스가 끝나도 토크를 끄지 않는다: 물품을 쥔 채 팔이 떨어지지 않게(hold와 같은 원칙)
                disable_torque_on_disconnect=False,
            )
        )
        self._gripper_goal: float | None = None

    def connect(self) -> None:
        try:
            self.robot.connect(calibrate=False)
            # 파일과 모터 값이 다르면 학습과 다른 관절 값이 나온다
            if not self.robot.is_calibrated:
                raise RuntimeError(
                    f"calibration mismatch: {self.robot.calibration_fpath} (패널에서 확인)"
                )
        except BaseException:
            self.disconnect()  # 모터만 열리고 카메라가 실패한 경우도 닫아야 다시 연결된다(#91 리뷰)
            raise

    def disconnect(self) -> None:
        """열린 것만 각각 닫는다(부분 연결 포함). 토크는 끄지 않는다."""
        # OmxFollower.disconnect()는 버스와 카메라가 모두 연결돼야 불리므로 직접 닫는다
        if self.robot.bus.is_connected:
            self.robot.bus.disconnect(disable_torque=False)
        for cam in self.robot.cameras.values():
            if (
                cam.is_connected or cam.thread is not None
            ):  # OpenCVCamera.disconnect 조건
                cam.disconnect()

    def ready(self) -> str | None:
        return None if self.robot.is_connected else "not connected"

    def observe(self) -> Observation:
        # 시간 제한: 모터 버스는 패킷 timeout, 카메라는 500 ms보다 오래된 프레임이면 예외
        obs = self.robot.get_observation()
        return Observation([float(obs[f"{j}.pos"]) for j in JOINTS], obs["front"])

    def send(self, action: Sequence[float]) -> None:
        if len(action) != len(JOINTS):
            raise ValueError(f"action은 {len(JOINTS)}개여야 합니다: {len(action)}")
        self.robot.send_action(
            {f"{j}.pos": float(v) for j, v in zip(JOINTS, action, strict=True)}
        )
        self._gripper_goal = float(action[GRIPPER])

    def hold(self) -> None:
        """관절은 현재 위치를 목표로. 그리퍼는 마지막 목표를 유지한다.

        그리퍼는 전류 제한 위치 제어라, 물품에 막힌 현재 위치를 목표로 주면 쥐는 힘이 풀린다.
        """
        goal = self.robot.bus.sync_read("Present_Position")
        if self._gripper_goal is not None:
            goal["gripper"] = self._gripper_goal
        self.robot.bus.sync_write("Goal_Position", goal)
