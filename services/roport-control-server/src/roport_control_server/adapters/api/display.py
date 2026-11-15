"""Per-viewer synthetic display state; never connects to or commands hardware."""

import math
import time
from dataclasses import asdict
from uuid import uuid4

from roport_control_server.contracts.snapshot import Snapshot, TaskRow

from .maps import pixel_to_map

ROBOTS = tuple(
    {"id": f"{kind}{i}", "kind": kind, "name": f"{name} {i:02}"}
    for kind, name in (("pinky", "Pinky"), ("omx", "OMX"))
    for i in range(1, 4)
)


class Display:
    """A browser-local demonstration, not the future DB-backed runtime."""

    def __init__(self, demo=False, map_geometry=None):
        self.demo = demo
        self.map_geometry = map_geometry
        self.session_id = str(uuid4())
        self.revision = 0
        self.playing = True
        self.position = 0.0
        self.last_tick = time.monotonic()
        self.dropped = set()
        self.samples = {}

    def command(self, data):
        if not self.demo or not isinstance(data, dict):
            raise ValueError("only demo playback controls are available")
        action, value = data.get("action"), data.get("value")
        if action == "play" and set(data) == {"action", "value"}:
            if type(value) is not bool:
                raise ValueError("play requires boolean value")
            self.playing = value
        elif action == "seek" and set(data) == {"action", "value"}:
            if (
                type(value) not in (float, int)
                or not math.isfinite(value)
                or not 0 <= value <= 24
            ):
                raise ValueError("seek requires finite seconds in [0, 24]")
            self.position = value
        elif action == "drop" and set(data) == {"action", "value", "robot_id"}:
            robot = data["robot_id"]
            if robot not in [r["id"] for r in ROBOTS] or type(value) is not bool:
                raise ValueError("drop requires example robot and boolean value")
            if value:
                self.dropped.add(robot)
            else:
                self.dropped.discard(robot)
        else:
            raise ValueError("unknown demo control")

    def snapshot(self):
        now = time.monotonic()
        if self.demo and self.playing:
            self.position = (self.position + min(now - self.last_tick, 0.2)) % 24
        self.last_tick = now
        self.revision += 1
        rows = []
        for index, robot in enumerate(ROBOTS):
            robot_id = robot["id"]
            if self.demo and robot_id not in self.dropped:
                t = self.position
                if robot["kind"] == "pinky":
                    phase = (t / 24 + index / 3) * math.tau
                    pose = [
                        2.3 + 1.3 * math.cos(phase),
                        1.3 + 0.65 * math.sin(phase),
                        0,
                        math.atan2(0.65 * math.cos(phase), -1.3 * math.sin(phase)),
                    ]
                    joints = {"l_wheel_joint": t * 3, "r_wheel_joint": t * 3}
                    mode, velocity = "주행 · 모의", 0.2
                else:
                    pose = [0.7 + (index - 3) * 1.45, 2.15, 0, -math.pi / 2]
                    joints = {
                        f"joint{i}": 0.35 * math.sin(t * 0.5 + i) for i in range(1, 6)
                    }
                    joints["gripper_joint_1"] = 0.3 + 0.2 * math.sin(t)
                    mode, velocity = "관절 표시 · 모의", None
                if self.map_geometry:
                    # Synthetic animation stays in the selected map; it is not a drivable route.
                    m = self.map_geometry
                    pose[:2] = pixel_to_map(
                        pose[0] / 4.6 * m["width"] - 0.5,
                        m["height"] - pose[1] / 2.65 * m["height"] - 0.5,
                        height=m["height"],
                        resolution=m["resolution"],
                        origin=m["origin"],
                    )
                    pose[3] = math.atan2(
                        math.sin(pose[3] + m["origin"][2]),
                        math.cos(pose[3] + m["origin"][2]),
                    )
                self.samples[robot_id] = (
                    now,
                    {
                        "pose": pose,
                        "joints": joints,
                        "mode": mode,
                        "velocity": velocity,
                    },
                )
            received, sample = self.samples.get(
                robot_id,
                (
                    None,
                    {"pose": None, "joints": {}, "mode": "미수신", "velocity": None},
                ),
            )
            age = None if received is None else max(0, now - received)
            rows.append(
                dict(sample, robot_id=robot_id, age=age, stale=age is None or age >= 2)
            )
        task = Snapshot(
            self.session_id,
            self.revision,
            (
                TaskRow("DEMO-001", "Pinky 1", "고정 경로 운송 (모의)", 40, "진행"),
                TaskRow("DEMO-002", "OMX 1·2", "양팔 협업 준비 (모의)", 0, "대기"),
                TaskRow("DEMO-003", "OMX 3", "카트 상차 완료 (모의)", 100, "완료"),
            )
            if self.demo
            else (),
        )
        return dict(
            asdict(task),
            robots=rows,
            demo=self.demo,
            demo_time=self.position,
            playing=self.playing,
            dropped=sorted(self.dropped),
            source="모의 상태 수신" if self.demo else "OFFLINE · 실행부 미연결",
            task_source="모의 작업 · 동작과 독립된 표시 예시"
            if self.demo
            else "실행부 미연결",
            message="읽기 전용 · URDF 표시 모델 · 실기 보정 미검증",
        )
