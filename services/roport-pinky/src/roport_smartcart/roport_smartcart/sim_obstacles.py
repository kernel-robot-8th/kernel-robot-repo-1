"""Gazebo-only 6 cm fixed/moving boxes; never publishes robot velocity."""

import json
import math
import os


def obstacle_sdf(name, x, y, height):
    if name not in ("nav_crossing", "line_crossing"):
        raise ValueError("unknown test actor")
    if not all(math.isfinite(v) for v in (x, y, height)) or not 0.02 <= height <= 0.2:
        raise ValueError("finite coordinates and height 0.02..0.20 m required")
    return f'''<sdf version="1.9"><model name="{name}">
      <pose>{x} {y} {height / 2} 0 0 0</pose><link name="body">
      <gravity>false</gravity><inertial><mass>0.1</mass><inertia>
      <ixx>0.00016</ixx><iyy>0.00016</iyy><izz>0.00006</izz></inertia></inertial>
      <collision name="collision"><geometry><box><size>0.06 0.06 {height}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.06 0.06 {height}</size></box></geometry>
      <material><diffuse>1 0.2 0.05 1</diffuse></material></visual>
      <sensor name="contacts" type="contact"><always_on>true</always_on><update_rate>50</update_rate>
      <contact><collision>collision</collision></contact><topic>/sim/obstacles/{name}/contacts</topic></sensor>
      </link><plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <topic>/sim/obstacles/{name}/cmd_vel</topic></plugin>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>20</update_frequency>
      </plugin></model></sdf>'''


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "198":
        raise SystemExit("Gazebo domain 198 only")
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    for key in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER"):
        os.environ.pop(key, None)
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav2_msgs.msg import CollisionMonitorState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node(
        "sim_obstacles", parameter_overrides=[Parameter("use_sim_time", value=True)]
    )
    bay_x = float(node.declare_parameter("bay_x", 1.544).value)
    stationary = bool(node.declare_parameter("stationary_line", False).value)
    stationary_nav = bool(node.declare_parameter("stationary_nav", False).value)
    poses, seen = {}, {}
    triggered = None
    direction = 1.0
    publishers = {
        name: node.create_publisher(Twist, f"sim/obstacles/{name}/cmd_vel", 10)
        for name in ("nav_crossing", "line_crossing")
    }
    monitor = [0, 0.0]
    state = {}
    state_seen = None

    def observe(name, msg):
        for t in msg.transforms:
            if t.child_frame_id == name:
                poses[name] = t.transform.translation
                seen[name] = node.get_clock().now().nanoseconds / 1e9

    def status(msg):
        nonlocal state_seen
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                state.clear()
                state.update(data)
                state_seen = node.get_clock().now().nanoseconds / 1e9
        except ValueError:
            pass

    def tick():
        nonlocal direction, triggered
        now = node.get_clock().now().nanoseconds / 1e9
        for name, pub in publishers.items():
            cmd = Twist()
            p = poses.get(name)
            if (
                p is not None
                and now - seen[name] < 0.5
                and state_seen is not None
                and now - state_seen < 1.0
                and state.get("state") in ("NAVIGATING", "WAIT_UNLOAD", "RETURNING")
            ):
                if name == "nav_crossing":
                    if p.y > -0.25:
                        direction = -1.0
                    if p.y < -0.85:
                        direction = 1.0
                    if not stationary_nav:
                        cmd.linear.y = direction * 0.04
                else:
                    robot = poses.get("pinky")
                    if (
                        triggered is None
                        and robot
                        and abs(robot.x - bay_x) < 0.12
                        and abs(robot.y + 1.284) < 0.08
                        and state.get("state") == "RETURNING"
                        and monitor[0] == CollisionMonitorState.STOP
                        and now - monitor[1] < 0.5
                    ):
                        triggered = now
                        node.get_logger().info(
                            "LINE ACTOR: safety stop observed; hold 3 seconds, then cross the tape"
                        )
                    if (
                        not stationary
                        and triggered is not None
                        and now - triggered > 3
                        and p.x < bay_x + 0.30
                    ):
                        cmd.linear.x = 0.05
            pub.publish(cmd)

    for name in publishers:
        node.create_subscription(
            TFMessage,
            f"model/{name}/pose",
            lambda m, k=name: observe(k, m),
            qos_profile_sensor_data,
        )
    node.create_subscription(
        TFMessage,
        "sim/gazebo_poses",
        lambda m: observe("pinky", m),
        qos_profile_sensor_data,
    )
    node.create_subscription(String, "smartcart/sim/state", status, 10)

    def safety_state(message):
        monitor[:] = [message.action_type, node.get_clock().now().nanoseconds / 1e9]

    node.create_subscription(
        CollisionMonitorState, "collision_monitor_state", safety_state, 10
    )
    node.create_timer(0.05, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for pub in publishers.values():
                pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
