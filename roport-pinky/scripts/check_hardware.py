#!/usr/bin/env python3
"""Subscribe to an already-running Pinky. Does not launch nodes or publish commands."""

import argparse
import json
import math
import os
import time
from datetime import UTC, datetime
from functools import partial
from pathlib import Path


def validate(kind, msg):
    """Validate the message contract of the vendored Pinky bringup and lidar."""
    # JointState.frame_id is optional and the upstream bringup leaves it empty.
    if kind != "joint_states" and not msg.header.frame_id:
        raise ValueError("Missing frame_id")
    if kind == "odom":
        pose = msg.pose.pose
        q = pose.orientation
        values = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            q.x,
            q.y,
            q.z,
            q.w,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ]
        if not msg.child_frame_id or msg.child_frame_id == msg.header.frame_id:
            raise ValueError("Invalid odom parent/child frames")
        if not math.isclose(
            sum(v * v for v in (q.x, q.y, q.z, q.w)), 1.0, abs_tol=0.01
        ):
            raise ValueError("Invalid orientation quaternion")
    elif kind == "joint_states":
        if not {"left_wheel_joint", "right_wheel_joint"}.issubset(msg.name):
            raise ValueError("Missing wheel joint names")
        if len(set(msg.name)) != len(msg.name) or len(msg.position) != len(msg.name):
            raise ValueError("Invalid joint names/position lengths")
        if msg.velocity and len(msg.velocity) != len(msg.name):
            raise ValueError("Invalid joint velocity length")
        values = list(msg.position) + list(msg.velocity)
    elif kind == "scan":
        if not (0 <= msg.range_min < msg.range_max and math.isfinite(msg.range_max)):
            raise ValueError("Invalid scan range limits")
        if (
            not msg.ranges
            or not math.isfinite(msg.angle_increment)
            or msg.angle_increment <= 0
        ):
            raise ValueError("Empty scan or invalid angle_increment")
        # +inf is a valid no-return ray; NaN and -inf are not.
        if any(math.isnan(v) or v == -math.inf or v < 0 for v in msg.ranges):
            raise ValueError("Invalid scan range values")
        values = []
    else:
        raise ValueError(f"Unknown topic kind: {kind}")
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite state values")


class Samples:
    def __init__(self, required):
        self.required = required
        self.topics = {
            kind: {"received": 0, "samples": 0, "last_stamp_ns": None, "error": None}
            for kind in ("odom", "joint_states", "scan")
        }

    def receive(self, kind, msg):
        row = self.topics[kind]
        row["received"] += 1
        try:
            validate(kind, msg)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= 0 or (
                row["last_stamp_ns"] is not None and stamp <= row["last_stamp_ns"]
            ):
                raise ValueError("Missing or non-advancing timestamp")
            row["samples"] += 1
            row["last_stamp_ns"] = stamp
            row["frame_id"] = msg.header.frame_id
        except ValueError as exc:
            row["error"] = str(exc)

    @property
    def complete(self):
        return all(row["samples"] >= self.required for row in self.topics.values())

    @property
    def ok(self):
        return self.complete and all(
            row["error"] is None for row in self.topics.values()
        )


def observe(namespace, timeout, samples):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, LaserScan

    state = Samples(samples)
    rclpy.init(args=[])
    node = rclpy.create_node("roport_read_only_check", namespace=namespace)
    try:
        for kind, message_type in (
            ("odom", Odometry),
            ("joint_states", JointState),
            ("scan", LaserScan),
        ):
            node.create_subscription(
                message_type,
                kind,
                partial(state.receive, kind),
                qos_profile_sensor_data,
            )
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not state.complete:
            rclpy.spin_once(
                node, timeout_sec=min(0.1, max(0, deadline - time.monotonic()))
            )
        return {
            "ok": state.ok,
            "topics": state.topics,
            "clock_publishers": node.count_publishers("/clock"),
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="/")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or args.samples < 2:
        parser.error(
            "--timeout must be positive and finite; --samples must be at least 2"
        )
    report = {
        "scope": "ros_topic_receive_only",
        "time_utc": datetime.now(UTC).isoformat(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "namespace": args.namespace,
        "timeout_seconds": args.timeout,
        "required_samples": args.samples,
        "hardware_source_verified": False,
        "motion_verified": False,
        "ok": False,
    }
    try:
        report.update(observe(args.namespace, args.timeout, args.samples))
    except Exception as exc:  # noqa: BLE001 - CLI failures must produce a diagnostic report
        report["error"] = str(exc)
    text = json.dumps(report, indent=2)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text + "\n")
    print(text)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
