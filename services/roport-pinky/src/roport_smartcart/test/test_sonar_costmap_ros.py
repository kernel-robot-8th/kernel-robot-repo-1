"""Actual Nav2 controller must recover after its first low scan arrives late."""

import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import pytest


@pytest.mark.parametrize(
    "marking_range,has_max_range_obstacles", [(0.94, False), (0.95, True)]
)
def test_delayed_low_scan_recovers_controller_without_disabling_freshness(
    monkeypatch, tmp_path, marking_range, has_max_range_obstacles
):
    import rclpy
    from ament_index_python.packages import (
        get_package_prefix,
        get_package_share_directory,
    )
    from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry, OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster
    from roport_smartcart.profiles import build_params, _clock_mode

    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    ctx = Context()
    rclpy.init(context=ctx, domain_id=184)
    n = rclpy.create_node("low_scan_probe", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(n)
    tf = StaticTransformBroadcaster(n)
    t = TransformStamped()
    t.header.frame_id = "test_world"
    t.child_frame_id = "test_base"
    t.transform.rotation.w = 1.0
    tf.sendTransform(t)
    source = n.create_publisher(LaserScan, "/test_low_scan", 10)
    odom = n.create_publisher(Odometry, "odom", 10)
    commands = []
    maps = []
    n.create_subscription(Twist, "cmd_vel", lambda m: commands.append(m.linear.x), 10)
    n.create_subscription(
        OccupancyGrid, "local_costmap/costmap", lambda m: maps.append(m), 10
    )
    p = build_params(
        str(
            Path(get_package_share_directory("pinky_navigation"))
            / "params/nav2_params.yaml"
        ),
        str(
            Path(get_package_share_directory("roport_smartcart"))
            / "config/nav2_overrides.yaml"
        ),
        "sim",
    )
    _clock_mode(p, False)
    c = p["local_costmap"]["local_costmap"]["ros__parameters"]
    c.update(
        global_frame="test_world",
        robot_base_frame="test_base",
        plugins=["sonar_layer"],
        width=2,
        height=2,
        track_unknown_space=False,
        always_send_full_costmap=True,
        publish_frequency=10.0,
    )
    c["sonar_layer"]["sonar_proxy"]["topic"] = "/test_low_scan"
    c["sonar_layer"]["sonar_proxy"]["obstacle_max_range"] = marking_range
    config = tmp_path / "params.yaml"
    config.write_text(yaml.safe_dump(p))
    log = (tmp_path / "controller.log").open("w")
    child = subprocess.Popen(
        [
            str(
                Path(get_package_prefix("nav2_controller"))
                / "lib/nav2_controller/controller_server"
            ),
            "--ros-args",
            "--params-file",
            str(config),
        ],
        env={**os.environ, "ROS_DOMAIN_ID": "184"},
        stdout=log,
        stderr=log,
    )
    handle = None
    try:
        lifecycle = n.create_client(ChangeState, "controller_server/change_state")
        assert lifecycle.wait_for_service(timeout_sec=15)
        for transition in (1, 3):
            req = ChangeState.Request()
            req.transition.id = transition
            f = lifecycle.call_async(req)
            ex.spin_until_future_complete(f, timeout_sec=6)
            assert f.done() and f.result().success
        # Miss more than the configured .5 s before publishing the first observation.
        start = time.monotonic()
        while time.monotonic() - start < 1.0:
            ex.spin_once(timeout_sec=0.02)
        client = ActionClient(n, FollowPath, "follow_path")
        assert client.wait_for_server(timeout_sec=5)
        goal = FollowPath.Goal()
        goal.path.header.frame_id = "test_world"
        goal.controller_id = "FollowPath"
        for x in (0.0, 0.2, 0.4):
            pose = PoseStamped()
            pose.header.frame_id = "test_world"
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)
        started = time.monotonic()
        future = None
        while time.monotonic() - started < 6:
            now = n.get_clock().now().to_msg()
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = "test_base"
            scan.angle_min = -0.13
            scan.angle_max = 0.13
            scan.angle_increment = 0.0325
            scan.range_min = 0.02
            scan.range_max = 0.95
            scan.ranges = [float("inf")] * 9
            source.publish(scan)
            msg = Odometry()
            msg.header.stamp = now
            msg.header.frame_id = "test_world"
            msg.child_frame_id = "test_base"
            msg.pose.pose.orientation.w = 1.0
            odom.publish(msg)
            if future is None and time.monotonic() - started > 1.0:
                future = client.send_goal_async(goal)
            ex.spin_once(timeout_sec=0.02)
            if any(v > 0.01 for v in commands) and maps:
                break
        assert any(v > 0.01 for v in commands), (
            tmp_path / "controller.log"
        ).read_text()
        assert maps and (100 in maps[-1].data) is has_max_range_obstacles
        assert c["sonar_layer"]["sonar_proxy"]["expected_update_rate"] == 0.5
        if future and future.done():
            handle = future.result()
    finally:
        if handle and handle.accepted:
            f = handle.cancel_goal_async()
            ex.spin_until_future_complete(f, timeout_sec=2)
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=3)
        log.close()
        ex.shutdown()
        n.destroy_node()
        rclpy.shutdown(context=ctx)
