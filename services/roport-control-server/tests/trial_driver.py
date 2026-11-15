"""Synthetic ROS-hook substitute for the published runner's subprocess tests."""

import time
from copy import deepcopy
from datetime import UTC, datetime
from itertools import pairwise


class Driver:
    def __init__(self, profile, role):
        assert profile["mode"] == "simulation"
        step = (
            "to_delivery"
            if profile["scenario"] == "single_pinky"
            else "to_loading"
            if role == "logistics"
            else "cart_to_delivery"
        )
        self.state = {
            "node_id": profile["routes"][step]["start_node"],
            "heading_rad": profile["routes"][step]["start_heading_rad"],
            "mode": "line",
            "stopped": True,
            "line_enabled": False,
            "nav2_ready": False,
            "localization_ready": False,
            "drive_owner": "none",
            "graph_version": profile["graph_version"],
            "map_id": profile["map_id"],
            "map_version": profile["map_version"],
            "fault": None,
        }
        self.headings = {
            r["start_node"]: r["start_heading_rad"] for r in profile["routes"].values()
        }
        self.nodes = profile["nodes"]
        self.pending = None

    def observe(self):
        return {**self.state, "observed_at": datetime.now(UTC).isoformat()}

    def validate_action(self, command):
        if command["type"] == "route.requested":
            path = command["payload"]["route"]
            if any(b not in self.nodes[a] for a, b in pairwise(path)):
                raise ValueError("unknown_edge")

    def dispatch(self, command):
        self.pending = deepcopy(command)
        self.seq = 0
        self.due = time.monotonic() + 0.05

    def safe_stop(self, reason):
        self.pending = None
        self.state.update(stopped=True, line_enabled=False, drive_owner="none")

    def drain_results(self):
        cmd = self.pending
        if not cmd or time.monotonic() < self.due:
            return
        p = cmd["payload"]
        if cmd["type"] == "route.requested":
            path, i = p["route"], self.seq
            last = i == len(path) - 1
            self.state.update(
                heading_rad=self.headings.get(path[i], self.state["heading_rad"]),
                node_id=path[i],
                stopped=last,
                line_enabled=not last,
                drive_owner="line",
            )
            self.seq += 1
            self.due = time.monotonic() + 0.05
            if last:
                self.pending = None
            yield (
                cmd,
                "completed" if last else "progress",
                {
                    "seq": i,
                    "total": len(path),
                    "prev_node": path[i - 1] if i else None,
                    "next_node": path[i + 1] if not last else None,
                },
            )
        elif cmd["type"] == "mode_switch.requested":
            self.state.update(
                mode="nav2",
                stopped=True,
                line_enabled=False,
                drive_owner="nav2",
                nav2_ready=True,
                localization_ready=True,
            )
            self.pending = None
            yield cmd, "completed", {}
        else:
            self.pending = None
            yield (
                cmd,
                "completed",
                {
                    "goal_id": p["goal_id"],
                    "goal_version": p["goal_version"],
                    "frame_id": "map",
                    "pose": p["pose"],
                    "nav2_result": "SUCCEEDED",
                },
            )


def create_driver(profile, role):
    return Driver(profile, role)
