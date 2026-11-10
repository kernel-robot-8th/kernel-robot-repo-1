"""Virtual sensors/motors around the team's unchanged Pinky trial and line driver.

Only the I/O adapters are simulated. Nav2 uses an obstacle-free kinematic stand-in,
not ROS navigation. Run via tools/run_virtual_trial.py (isolated loopback only).
"""

import math
import sys
import threading
import time
from datetime import UTC, datetime
from pathlib import Path

PINKY = Path(__file__).resolve().parents[2] / "roport-pinky"
LAYOUT = PINKY / "src/roport_gz_world/layout/roport_airport.yaml"
for package in ("roport_pinky", "roport_line_nav", "roport_pinky_line"):
    sys.path.insert(0, str(PINKY / "src" / package))
sys.path.insert(0, str(PINKY / "src/roport_line_nav/test"))

from roport_line_nav.executor import RouteExecutor
from roport_line_nav.graph import Graph
from roport_pinky.trial_driver import TrialDriver, TrialLogic
from sim import Sim


class SimulatedNav:
    """Explicitly simulated, obstacle-free motion; real completion checks stay in TrialDriver."""

    def __init__(self, pose, ready=True):
        self.pose = pose
        self.ready = ready
        self.goal = self.result = None
        self.navigation = None
        self.stopped = True
        self.observed_at = time.time()

    def snapshot(self):
        return {
            "pose": dict(self.pose),
            "nav2_ready": self.ready,
            "localization_ready": self.ready and time.time() - self.observed_at <= 0.3,
            "stopped": self.stopped,
            "observed_at": datetime.fromtimestamp(self.observed_at, UTC).isoformat(),
        }

    def diagnostic(self):
        return dict(self.navigation) if self.navigation else None

    def start(self, payload):
        self.navigation = {
            "phase": "navigating",
            "goal_id": payload["goal_id"],
            "target_pose": dict(payload["pose"]),
            "current_pose": dict(self.pose),
            "recoveries": 0,
            "max_recoveries": 2,
            "recovery_elapsed_s": 0.0,
            "recovery_timeout_s": 30.0,
            "nav2_error_code": 0,
            "action_status": 0,
            "error_code": None,
        }
        self.goal = payload["pose"]
        self.result = None
        self.stopped = False

    def cancel(self):
        self.goal = None
        self.stopped = True

    def tick(self, dt):
        self.observed_at = time.time()
        if self.navigation is not None:
            self.navigation["current_pose"] = dict(self.pose)
        if self.goal is None:
            return
        dx, dy = (self.goal[k] - self.pose[k] for k in ("x", "y"))
        distance = math.hypot(dx, dy)
        if distance > 0.001:
            fraction = min(1, 0.03 * dt / distance)
            self.pose["x"] += dx * fraction
            self.pose["y"] += dy * fraction
        else:
            error = math.atan2(
                math.sin(self.goal["yaw"] - self.pose["yaw"]),
                math.cos(self.goal["yaw"] - self.pose["yaw"]),
            )
            self.pose["yaw"] += max(-0.4 * dt, min(0.4 * dt, error))
            if abs(error) <= 0.01:
                self.goal = None
                self.stopped = True
                self.result = ("SUCCEEDED", time.monotonic())


class VirtualPinky(TrialDriver):
    """Reuse observe/validation/dispatch/result/heartbeat; replace ROS I/O only."""

    def __init__(self, profile, role, *, nav2=True, fault="none", threaded=True):
        if profile["mode"] != "simulation":
            raise ValueError("virtual Pinky requires a simulation profile")
        if fault not in {"none", "line-loss", "odom-stale"}:
            raise ValueError("unknown virtual fault")
        self.lock = threading.RLock()
        self.graph = Graph.load(str(LAYOUT))
        if self.graph.version != profile["graph_version"]:
            raise ValueError("virtual layout graph version mismatch")
        key = (
            "to_delivery"
            if profile["scenario"] == "single_pinky"
            else "to_loading"
            if role == "logistics"
            else "cart_to_delivery"
        )
        route = profile["routes"][key]
        start = route["start_node"]
        heading = self.graph.heading(start, self.graph.nodes[start].links[0])
        self.sim = Sim(self.graph)
        self.sim.place(start, heading)
        self.ex = RouteExecutor(self.graph)
        self.mode, self.nav_command = "line", None
        self.pose_source = "simulation"
        self.halted, self.runner_seen = None, time.monotonic()
        self.guard = {"owner": "none", "blocked": False}
        self.lease_at = None
        self.last = None
        self.fault = fault
        self.motion_time = 0.0
        self.logic = TrialLogic(
            self.graph, profile, send=self._send, zero=self._zero, start_node=start
        )
        self.nav = SimulatedNav(self.map_pose(), ready=nav2)
        self._feed()
        self.done = threading.Event()
        self.thread = None
        if threaded:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def map_pose(self):
        import yaml

        # Read the frame once; never infer metre coordinates from centimetre nodes.
        if not hasattr(self, "frame"):
            self.frame = yaml.safe_load(LAYOUT.read_text())["frame"]
        yaw = math.radians(self.frame["yaw_deg"])
        x, y = self.sim.x / 100, self.sim.y / 100
        return {
            "x": self.frame["x_m"] + math.cos(yaw) * x - math.sin(yaw) * y,
            "y": self.frame["y_m"] + math.sin(yaw) * x + math.cos(yaw) * y,
            "yaw": math.atan2(
                math.sin(self.sim.yaw + yaw), math.cos(self.sim.yaw + yaw)
            ),
        }

    def _send(self, body):
        try:
            {
                "speed": lambda: self.ex.set_speed(body["base_speed"]),
                "start": lambda: self.ex.start(body["route"]),
                "load_done": self.ex.load_done,
                "stop": self.ex.stop,
            }[body["cmd"]]()
            self.last = {"cmd": body["cmd"], "ok": True, "detail": ""}
        except ValueError as exc:
            self.last = {"cmd": body["cmd"], "ok": False, "detail": str(exc)}

    def _guard_fault(self):
        # shortcut: in-process lease model; independent ROS guard/process death needs hardware/HIL.
        if self.lease_at is not None and time.monotonic() - self.lease_at > 1:
            self.guard.update(blocked=True, owner="none")
        return self.halted or ("drive_lease_expired" if self.guard["blocked"] else None)

    def _lease(self, owner):
        if not self._guard_fault():
            self.guard["owner"] = owner
            self.lease_at = time.monotonic()

    def _feed(self):
        state = self.ex.as_dict()
        state.update(graph_version=self.graph.version, last_command=self.last)
        self.logic.on_state(state)
        if not (self.fault == "odom-stale" and self.motion_time >= 5):
            self.logic.on_odom(*self.sim.cmd)

    def tick(self):
        with self.lock:
            if self._guard_fault():
                self.sim.cmd = (0.0, 0.0)
                self.ex.stop()
                self.nav.cancel()
            if self.logic.active:
                self.motion_time += self.sim.dt
            if self.fault == "line-loss" and self.motion_time >= 5:
                self.sim.rects = []
            if self.mode == "line":
                self.sim.tick(self.ex)
                self.nav.pose = self.map_pose()
                self.nav.observed_at = time.time()
            else:
                self.sim.cmd = (0.0, 0.0)
                self.sim.t += self.sim.dt
                self.nav.tick(self.sim.dt)
            self._feed()
            self.sim.trace.clear()  # Persistent MQTT journals hold progress; avoid unbounded tick history.

    def _run(self):
        try:
            next_tick = time.monotonic() + self.sim.dt
            while not self.done.wait(max(0, next_tick - time.monotonic())):
                if time.monotonic() - next_tick > 0.25:
                    self.halted = "virtual_clock_overrun"
                self.tick()
                next_tick = (
                    time.monotonic() + self.sim.dt
                    if self.halted
                    else next_tick + self.sim.dt
                )
        except Exception:
            with self.lock:
                self.halted = "virtual_sensor_loop_failed"
                self.sim.cmd = (0.0, 0.0)
            raise

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=2)
        self.safe_stop("virtual_shutdown")


def main():
    import argparse
    import json

    from roport_control_server.adapters.mqtt.trial_runner import run, select_robot
    from roport_control_server.adapters.mqtt.trial_wire import mqtt_config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--role", required=True)
    parser.add_argument("--journal", required=True)
    parser.add_argument("--nav2", choices=("simulated", "off"), default="simulated")
    parser.add_argument(
        "--fault", choices=("none", "line-loss", "odom-stale"), default="none"
    )
    args = parser.parse_args()
    profile = json.loads(Path(args.config).read_text())
    robot = select_robot(profile, args.role)
    connection = mqtt_config(f"{profile['site_id']}-{robot['robot_id']}")
    if connection.host != "127.0.0.1" or profile["mode"] != "simulation":
        parser.error("virtual Pinky requires simulation + 127.0.0.1 broker")
    driver = VirtualPinky(
        profile, args.role, nav2=args.nav2 == "simulated", fault=args.fault
    )
    run(profile, args.role, connection, args.journal, driver)


if __name__ == "__main__":
    main()
