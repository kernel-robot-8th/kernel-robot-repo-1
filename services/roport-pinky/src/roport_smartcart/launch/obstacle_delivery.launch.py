"""Nav2-only or combined Gazebo obstacle fixtures; never sends delivery goals."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roport_smartcart.sim_obstacles import obstacle_sdf


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    bay = arg("bay")
    if bay not in ("H1", "H2"):
        raise ValueError("bay must be H1 or H2")
    scenario = arg("scenario")
    if scenario not in ("combined", "nav-static", "nav-moving"):
        raise ValueError("scenario must be combined, nav-static or nav-moving")
    bay_x = 1.544 if bay == "H1" else 1.874
    share = Path(get_package_share_directory("roport_smartcart"))
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/simulation.launch.py")),
            launch_arguments={"gui": arg("gui")}.items(),
        )
    ]
    bridges = []
    actors = [
        ("nav_crossing", 0.95, -0.55, float(arg("height"))),
    ]
    if scenario == "combined":
        actors.append(("line_crossing", bay_x, -1.48, 0.10))
    for name, x, y, height in actors:
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-world",
                    "roport_airport",
                    "-name",
                    name,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    str(height / 2),
                    "-string",
                    obstacle_sdf(name, x, y, height),
                ],
                parameters=[{"use_sim_time": True}],
            )
        )
        bridges.extend(
            [
                f"/model/{name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                f"/sim/obstacles/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/world/roport_airport/model/{name}/link/body/sensor/contacts/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            ]
        )
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=bridges,
            remappings=[
                (
                    f"/world/roport_airport/model/{name}/link/body/sensor/contacts/contact",
                    f"/sim/obstacles/{name}/contacts",
                )
                for name, _, _, _ in actors
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="roport_smartcart",
            executable="sim_obstacles",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "bay_x": bay_x,
                    "stationary_line": arg("stationary_line") == "true",
                    "stationary_nav": scenario == "nav-static",
                }
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("bay", default_value="H1"),
            DeclareLaunchArgument("height", default_value="0.125"),
            DeclareLaunchArgument("stationary_line", default_value="false"),
            DeclareLaunchArgument("scenario", default_value="combined"),
            OpaqueFunction(function=setup),
        ]
    )
