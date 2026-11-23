"""장치 소유 잠금, 토크를 건드리지 않는 관절 읽기, 처음 자세로 복귀(Panel omx_return_home.py).

- RobotLock: Panel과 같은 <shared_root>/robot.lock flock. Panel의 학습·녹화·평가 job과 이 실행부가 같은 팔과
  시리얼 포트를 동시에 쓰지 않게 한다. 다른 로봇 프로그램(lerobot-teleoperate/record, omx_run 등)이 돌면 거절.
- read_arm: 버스만 열어 Present_Position을 읽는다(운전 모드·토크 쓰기 없음). Panel omx_home.read_follower와 같다.
- return_home: 강제 종료로 토크가 켜진 채 남은 팔을, 버스만 다시 열어 지금 자리에서 출력 30%로 잡은 뒤
  처음 자세로 동기 이동 → 그리퍼 열기 → 손목 정리 → 출력 감쇠 → 토크 끔. 범위·거리 검사에 걸리면 움직이지 않는다.
lerobot은 함수 안에서 불러온다(il3 venv의 worker 프로세스에서만 쓴다).
"""

from __future__ import annotations

import fcntl
import os
import re
import subprocess
import threading
import time
from collections.abc import Mapping
from pathlib import Path

from .contract import JOINTS
from .safety import DEFAULT_SAFETY, Mover, SafetySettings

ROBOT_PROGRAMS = re.compile(
    r"lerobot-(teleoperate|record)|omx_(run|record|teleop|return_home)\.py"
)


class RobotBusy(RuntimeError):
    pass


class RobotLock:
    def __init__(self, path: str | Path):
        self.path = Path(os.path.expanduser(str(path)))
        self._fd: int | None = None

    def acquire(self, owner: str) -> None:
        busy = other_robot_processes()
        if busy:
            raise RobotBusy(f"robot program running: {busy[0]}")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o664)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            raise RobotBusy(f"{self.path} held by another program") from None
        self._fd = fd
        (self.path.parent / "robot.owner").write_text(f"{owner} pid {os.getpid()}\n")

    def release(self) -> None:
        if self._fd is not None:
            fcntl.flock(self._fd, fcntl.LOCK_UN)
            os.close(self._fd)
            self._fd = None

    @property
    def held(self) -> bool:
        return self._fd is not None


def other_robot_processes() -> list[str]:
    out = subprocess.run(
        ["ps", "-eo", "args"], capture_output=True, text=True, timeout=5, check=False
    ).stdout
    return [line.strip() for line in out.splitlines() if ROBOT_PROGRAMS.search(line)]


def _follower(port: str, calibration_id: str, calibration_dir: str | None):
    from lerobot.robots.omx_follower import OmxFollower, OmxFollowerConfig

    return OmxFollower(
        OmxFollowerConfig(id=calibration_id, calibration_dir=calibration_dir, port=port)
    )


def read_arm(
    port: str,
    calibration_id: str = "omx_follower_arm",
    calibration_dir: str | None = None,
) -> dict:
    robot = _follower(port, calibration_id, calibration_dir)
    robot.bus.connect()
    try:
        pos = robot.bus.sync_read("Present_Position")
    finally:
        robot.bus.disconnect(disable_torque=False)
    return {m: round(float(v), 2) for m, v in pos.items() if m in JOINTS}


def return_home(
    arms: Mapping[
        str, Mapping
    ],  # {"left": {"port", "joints"}, "right": ...} (실행 시작 자세)
    *,
    calibration_id: str = "omx_follower_arm",
    calibration_dir: str | None = None,
    settings: SafetySettings = DEFAULT_SAFETY,
    lifter=None,
    log=print,
) -> dict[str, bool]:
    """두 팔을 동시에 처음 자세로. {side: 돌아갔는가}."""
    from .robot import LeRobotArm

    done: dict[str, bool] = {}

    def one(side: str, entry: Mapping) -> None:
        robot = _follower(entry["port"], calibration_id, calibration_dir)
        robot.bus.connect()  # 포트만 연다: 토크·목표는 끝난 실행이 남긴 그대로
        arm = LeRobotArm(side, robot)
        try:
            present = arm.read()
            arm.set_pwm(settings.pwm_ratio)
            arm.write_goals(present)  # 옛 목표로 튀지 않게 지금 자리부터
            robot.bus.enable_torque()
            time.sleep(0.2)
            mover = Mover(arm, settings, log=log, lifter=lifter)
            done[side] = mover.soft_land(entry.get("joints"), threading.Event())
        except Exception as exc:  # noqa: BLE001 - 다른 팔은 계속 돌아가야 한다
            log(f"{side}: return home failed ({exc})")
            done[side] = False
        finally:
            arm.close(torque_off=False)

    threads = [threading.Thread(target=one, args=item) for item in arms.items()]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return done
