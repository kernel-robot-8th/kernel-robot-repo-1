"""안전 시작·착지: Panel omx_safe_start.py의 평가 경로를 옮겼다(LeRobot 클래스 패치 없이, ArmIO 위에서).

현장 설정(Panel 3.21, home_pose 끔, 저장된 시작·휴식 자세 없음)에서 평가 한 번은 이렇게 움직인다.
- 시작 전 점검(preflight): 별도 프로세스로 모터 12개의 원시 tick을 읽는다. 무응답·0~4095 밖이면 거절.
- 연결 직후 출력 30% 제한.
- 첫 목표 직전: 실측 자세를 "처음 자세"로 저장. 회전 관절(shoulder_pan, wrist_roll)이 범위 밖이면 재부팅,
  다른 관절이 범위 밖이거나 첫 목표와 90 넘게 다르면 거절.
- gripper 단계: 그리퍼를 10 u/s로 47(닫힘)까지.
- approach 단계: 정책 목표를 향해 15 u/s(그리퍼 10). 목표가 시작 때보다 3 넘게 바뀌면 출력 100%,
  60 u/s로 따라감. 2 이내로 0.3초 머물거나 25초가 지나면 normal(제한 없음, 출력 100%).
- 착지(soft): 처음 자세로 동기 이동(12 u/s, 출력 30%) → 그리퍼를 처음 값으로 엶 → 손목 롤 0,
  그리퍼 47까지 닫음(걸리면 멈춤) → 3초에 걸쳐 출력을 줄이고 토크 끔.
  들기(lift, 손끝을 먼저 위로)는 lifter를 넘겼을 때만 한다(P2에서 연결).
- 빠른 착지(fast): 0.5초에 걸쳐 출력을 줄이고 토크 끔. 넘겨받기(hold): 토크를 켠 채 포트만 닫음.
- 비상 정지(freeze): 이동을 멈추고 release가 올 때까지 토크를 켠 채 기다린 뒤 착지를 이어 간다.
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from .contract import ARM_JOINTS, JOINTS
from .ports import ArmIO, SafeStartRefused

ROTATION_JOINTS = ("shoulder_pan", "wrist_roll")  # 중력 부하 없음: 재부팅해도 안전
PWM_MAX = 885  # X 시리즈 Goal_PWM 최대


@dataclass(frozen=True)
class SafetySettings:
    approach_speed: float = 15.0
    gripper_speed: float = 10.0
    gripper_close: float = 47.0
    pwm_ratio: float = 0.3
    catch_up_speed: float = 60.0
    target_move: float = 3.0  # Panel leader_move: 목표가 이만큼 바뀌면 따라잡기
    max_gap: float = 90.0
    home_max_gap: float = 150.0
    tolerance: float = 2.0
    near_hold_s: float = 0.3
    timeout_s: float = 25.0
    stop_speed: float = 12.0
    stop_ramp_s: float = 3.0
    fast_ramp_s: float = 0.5
    stop_wrist_roll: float = 0.0
    gripper_stall: float = 4.0
    rate_hz: float = 30.0
    open_gripper_on_return: bool = True  # Panel OMX_HOME_GRIPPER=open


DEFAULT_SAFETY = SafetySettings()


def in_range(value: float) -> bool:
    return -100.0 <= value <= 100.0


def _step(current: float, target: float, max_delta: float) -> float:
    d = target - current
    return target if abs(d) <= max_delta else current + max_delta * (1 if d > 0 else -1)


# ---------------------------------------------------------------- preflight
PROBE_S = 8.0
PROBE = """
import json, sys
from dynamixel_sdk import PacketHandler, PortHandler
p = PortHandler(sys.argv[1])
if not (p.openPort() and p.setBaudRate(1000000)):
    sys.exit(3)
h, first, out = PacketHandler(2.0), int(sys.argv[2]), []
for i in range(first, first + 6):
    v, comm, _ = h.read4ByteTxRx(p, i, 132)
    out.append(None if comm else v - 2**32 if v >= 2**31 else v)
p.closePort()
print(json.dumps(out))
"""


def preflight(
    arms: Sequence[tuple[str, str]],
    *,
    first_id: int = 11,
    python: str = sys.executable,
    timeout_s: float = PROBE_S,
    popen=subprocess.Popen,
) -> list[str]:
    """(side, port)마다 Present_Position 원시 tick을 읽기만 한다(토크·설정 쓰기 없음). 문제 목록을 돌려준다.

    LeRobot은 범위 밖 값을 잘라 읽으므로 한 바퀴 넘은 모터는 원시 tick으로만 보인다.
    응답 없는 보드가 연결을 막지 않게 팔마다 별도 프로세스로 읽는다.
    """
    procs = [
        (
            side,
            port,
            popen(
                [python, "-c", PROBE, port, str(first_id)],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            ),
        )
        for side, port in arms
    ]
    problems, end = [], time.monotonic() + timeout_s
    try:
        while time.monotonic() < end and any(p.poll() is None for _, _, p in procs):
            time.sleep(0.05)
        for side, port, p in procs:
            if p.poll() is None:
                problems.append(f"{side}: board not answering ({port})")
                continue
            if p.returncode:
                problems.append(f"{side}: cannot open port ({port})")
                continue
            try:
                ticks = json.loads(p.stdout.read())
            except ValueError:
                problems.append(f"{side}: unreadable probe output ({port})")
                continue
            for name, tick in zip(JOINTS, ticks, strict=False):
                if tick is None:
                    problems.append(f"{side} {name}: motor not answering")
                elif not 0 <= tick <= 4095:
                    problems.append(f"{side} {name}: past one turn (tick {tick})")
    finally:
        for _, _, p in procs:
            if p.poll() is None:
                p.kill()
    return problems


# ---------------------------------------------------------------- safe start
class SafeStart:
    """한 팔의 첫 목표들을 저속·저출력 순서로 바꾼다. phase: gripper → approach → normal."""

    def __init__(
        self,
        arm: ArmIO,
        settings: SafetySettings = DEFAULT_SAFETY,
        clock: Callable[[], float] = time.monotonic,
        log: Callable[[str], None] = lambda _msg: None,
    ):
        self.arm, self.s, self.clock, self.log = arm, settings, clock, log
        self.phase = "gripper"
        self.home: dict[str, float] | None = None  # 처음 자세(첫 목표 직전 실측)
        self._cmd: dict[str, float] | None = None
        self._t = self._t0 = 0.0
        self._base: dict[str, float] | None = None
        self._driving = False
        self._near: float | None = None

    def begin(self) -> None:
        self.arm.set_pwm(self.s.pwm_ratio)

    def filter(self, target: Mapping[str, float]) -> dict[str, float]:
        """정책 목표 → 이번 주기에 실제로 보낼 목표."""
        if self.phase == "normal":
            return dict(target)
        now = self.clock()
        if self._cmd is None:
            self._first(target, now)
        dt = min(max(now - self._t, 0.0), 0.2)
        self._t = now
        cmd, s = self._cmd, self.s
        if self.phase == "gripper":
            cmd["gripper"] = _step(
                cmd["gripper"], s.gripper_close, s.gripper_speed * dt
            )
            if abs(cmd["gripper"] - s.gripper_close) <= 1e-6:
                self.phase = "approach"
                self._base = dict(target)
                self.log("approach: moving slowly to the policy's target")
            return dict(cmd)
        base = self._base or target
        if not self._driving and any(
            abs(target[m] - base.get(m, target[m])) > s.target_move for m in ARM_JOINTS
        ):
            self._driving = True
            self.arm.set_pwm(1.0)
            self.log("approach: target moving, full output catch-up")
        for m, goal in target.items():
            if self._driving:
                speed = s.catch_up_speed
            else:
                speed = s.gripper_speed if m == "gripper" else s.approach_speed
            cmd[m] = _step(cmd[m], goal, speed * dt)
        far = max((abs(cmd[m] - g) for m, g in target.items()), default=0.0)
        self._near = (self._near or now) if far <= s.tolerance else None
        if (self._near is not None and now - self._near > s.near_hold_s) or (
            now - self._t0 > s.timeout_s
        ):
            self.phase = "normal"
            self.arm.set_pwm(1.0)
            self.log("normal control" if far <= s.tolerance else "approach timed out")
            return dict(target)
        return dict(cmd)

    def _first(self, target: Mapping[str, float], now: float) -> None:
        present = self.arm.read()
        bad = [m for m in ARM_JOINTS if not in_range(present[m])]
        for m in bad:
            if m not in ROTATION_JOINTS:
                raise SafeStartRefused(
                    f"{self.arm.side} {m} out of range ({present[m]:.1f})"
                )
            self.log(f"{m} read past one turn ({present[m]:.1f}); rebooting it")
            self.arm.reboot(m)
        if bad:
            present = self.arm.read()
            still = [m for m in ARM_JOINTS if not in_range(present[m])]
            if still:
                raise SafeStartRefused(f"{self.arm.side} {still} still out of range")
        gaps = {m: abs(target[m] - present[m]) for m in ARM_JOINTS if m in target}
        worst = max(gaps, key=gaps.get) if gaps else None
        if worst and gaps[worst] > self.s.max_gap:
            raise SafeStartRefused(
                f"{self.arm.side} {worst} first target {gaps[worst]:.0f} away"
            )
        self._cmd = {m: float(v) for m, v in present.items()}
        self.home = dict(self._cmd)
        self._t = self._t0 = now


# ---------------------------------------------------------------- landing
class Mover:
    """처음 자세로 돌아가기·손목 정리·출력 줄이기. freeze가 서 있으면 이동을 멈춘다."""

    def __init__(
        self,
        arm: ArmIO,
        settings: SafetySettings = DEFAULT_SAFETY,
        *,
        freeze: threading.Event | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        log: Callable[[str], None] = lambda _msg: None,
        lifter=None,  # (present, side) -> list[dict] 손끝을 먼저 드는 경유점(P2)
    ):
        self.arm, self.s = arm, settings
        self.freeze = freeze or threading.Event()
        self.clock, self.sleep, self.log, self.lifter = clock, sleep, log, lifter

    def _move(self, start: dict, goal: dict, speed: float, settle: bool = True) -> bool:
        """동기 이동: 가장 먼 관절이 speed로, 모두 같이 도착(관절 공간 직선)."""
        span = max((abs(goal[m] - start[m]) for m in goal), default=0.0)
        dur = span / speed if speed > 0 else 0.0
        done, t = 0.0, self.clock()
        t0 = t
        while self.clock() - t0 < self.s.timeout_s:
            if self.freeze.is_set():
                return False
            self.sleep(1 / self.s.rate_hz)
            now = self.clock()
            dt, t = min(now - t, 0.2), now
            done = 1.0 if dur <= 0 else min(1.0, done + dt / dur)
            self.arm.write_goals(
                {m: start[m] + (goal[m] - start[m]) * done for m in goal}
            )
            if done >= 1.0:
                if settle:
                    self.sleep(0.5)
                return True
        return False

    def go_home(self, home: Mapping[str, float]) -> bool:
        present = self.arm.read()
        bad = [
            m for m in ARM_JOINTS if not in_range(present[m]) or not in_range(home[m])
        ]
        if bad:
            self.log(f"{self.arm.side}: {bad} out of range, not moving home")
            return False
        gaps = {m: abs(present[m] - home[m]) for m in ARM_JOINTS}
        worst = max(gaps, key=gaps.get)
        if gaps[worst] > self.s.home_max_gap:
            self.log(f"{self.arm.side}: {worst} too far from home, not moving")
            return False
        self.arm.set_pwm(self.s.pwm_ratio)
        pos = {m: present[m] for m in ARM_JOINTS}
        for i, p in enumerate(
            self.lifter(present, self.arm.side) if self.lifter else []
        ):
            if not self._move(pos, p, self.s.stop_speed, settle=False):
                return False
            pos = dict(p)
        arrived = self._move(pos, {m: home[m] for m in ARM_JOINTS}, self.s.stop_speed)
        if arrived and self.s.open_gripper_on_return and "gripper" in home:
            now = self.arm.read()["gripper"]
            if home["gripper"] >= now - 1.0:  # 여는 방향만(쥔 물건을 조이지 않는다)
                self._move(
                    {"gripper": now}, {"gripper": home["gripper"]}, self.s.gripper_speed
                )
        return arrived

    def park(self) -> None:
        """손목 롤을 stop_wrist_roll로, 그리퍼를 닫힘까지(걸리면 그 자리에서 멈춤)."""
        present = self.arm.read()
        if not in_range(present["wrist_roll"]):
            return
        if (
            self._steps(
                "wrist_roll",
                present["wrist_roll"],
                self.s.stop_wrist_roll,
                self.s.stop_speed,
            )
            and present["gripper"] > self.s.gripper_close
        ):
            self._steps(
                "gripper",
                present["gripper"],
                self.s.gripper_close,
                self.s.gripper_speed,
                stall=self.s.gripper_stall,
            )

    def _steps(self, motor, cmd, goal, speed, stall=None) -> bool:
        t = t0 = self.clock()
        while self.clock() - t0 < self.s.timeout_s:
            if self.freeze.is_set():
                return False
            self.sleep(1 / self.s.rate_hz)
            now = self.clock()
            dt, t = min(now - t, 0.2), now
            cmd = _step(cmd, goal, speed * dt)
            if stall is not None:
                at = self.arm.read()[motor]
                if abs(cmd - at) > stall:
                    self.arm.write_goals({motor: at})
                    return True
            self.arm.write_goals({motor: cmd})
            if abs(cmd - goal) <= 1e-6:
                self.sleep(0.5)
                return True
        return False

    def fade_off(self, start: float, ramp_s: float, steps: int) -> None:
        for k in range(1, steps + 1):
            self.arm.set_pwm(start * (1 - k / steps))
            self.sleep(ramp_s / steps)
        self.arm.torque_off()
        self.arm.set_pwm(1.0)  # 다음 프로그램을 위해 기본 출력으로(RAM 값, 토크는 꺼짐)

    def soft_land(
        self, home: Mapping[str, float] | None, release: threading.Event
    ) -> bool:
        """처음 자세 → park → 출력 줄이고 토크 끔. 돌아갔으면 True. freeze 중에는 release를 기다린다."""
        self.arm.set_pwm(self.s.pwm_ratio)
        self._hold_while_frozen(release)
        arrived = False
        if home:
            arrived = self.go_home(home)
            while (
                self.freeze.is_set()
            ):  # 돌아가는 중 비상 정지: 제자리에서 기다린 뒤 이어서
                self._hold_while_frozen(release)
                arrived = self.go_home(home)
        if arrived:
            try:
                self.park()
            except Exception as exc:  # noqa: BLE001 - 출력은 그래도 줄여야 한다
                self.log(f"{self.arm.side}: park failed ({exc})")
        self.fade_off(self.s.pwm_ratio, self.s.stop_ramp_s, 15)
        return arrived

    def _hold_while_frozen(self, release: threading.Event) -> None:
        if self.freeze.is_set():
            self.log(f"{self.arm.side}: frozen, holding with torque on until release")
            release.wait()
            self.freeze.clear()

    def fast_land(self) -> None:
        self.fade_off(1.0, self.s.fast_ramp_s, 5)
