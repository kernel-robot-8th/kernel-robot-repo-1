"""Low-box fixtures and bounded stop/wait on a blocked tape path."""

import xml.etree.ElementTree as ET
import pytest
from roport_smartcart.sim_obstacles import obstacle_sdf
from roport_smartcart.sim_return_h1 import line_blocked_since


@pytest.mark.parametrize("height", [0.10, 0.125])
def test_actor_has_real_geometry_motion_pose_and_contact_sensor(height):
    model = ET.fromstring(obstacle_sdf("nav_crossing", 0.95, -0.55, height)).find(
        "model"
    )
    assert [
        float(v) for v in model.findtext("link/collision/geometry/box/size").split()
    ] == [0.06, 0.06, height]
    assert float(model.findtext("pose").split()[2]) == height / 2
    assert model.find("link/sensor/contact/collision").text == "collision"
    plugins = {p.get("name") for p in model.findall("plugin")}
    assert {
        "gz::sim::systems::VelocityControl",
        "gz::sim::systems::PosePublisher",
    } <= plugins


@pytest.mark.parametrize("height", [-1.0, 0.0, 0.3, float("nan"), float("inf")])
def test_actor_rejects_unbounded_or_invalid_geometry(height):
    with pytest.raises(ValueError):
        obstacle_sdf("nav_crossing", 0.0, 0.0, height)


def test_stopped_robot_waits_briefly_but_fails_instead_of_leaving_the_tape():
    since = line_blocked_since(None, (0.06, 0.0), (0.0, 0.0), 10.0)
    assert since == 10.0
    assert line_blocked_since(since, (0.06, 0.0), (0.0, 0.0), 21.99) == 10.0
    with pytest.raises(RuntimeError, match="line_motion_blocked"):
        line_blocked_since(since, (0.06, 0.0), (0.0, 0.0), 22.0)
    assert line_blocked_since(since, (0.02, 0.0), (0.006, 0.0), 11.0) is None
    assert line_blocked_since(since, (0.0, 0.0), (0.0, 0.0), 11.0) is None
    assert line_blocked_since(since, (0.0, 0.45), (0.0, 0.135), 11.0) is None


def test_obstacle_fixture_cannot_start_in_physical_domain(monkeypatch):
    from roport_smartcart.sim_obstacles import main

    monkeypatch.setenv("ROS_DOMAIN_ID", "13")
    with pytest.raises(SystemExit):
        main()


def test_spawn_passes_explicit_world_pose_and_bridges_real_contact_topics(
    monkeypatch, tmp_path
):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch_ros.actions import Node

    path = Path(__file__).parents[1] / "launch/obstacle_delivery.launch.py"
    spec = importlib.util.spec_from_file_location("obstacle_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(
        bay="H1", gui="false", height="0.125", stationary_line="false"
    )
    actions = module.setup(context)
    creates = [
        a for a in actions if isinstance(a, Node) and a.node_executable == "create"
    ]
    assert len(creates) == 2
    for a in creates:
        args = ["".join(p.perform(context) for p in arg) for arg in a.cmd[1:13]]
        values = dict(zip(args[::2], args[1::2]))
        model = ET.fromstring(values["-string"]).find("model")
        assert list(map(float, model.findtext("pose").split()[:3])) == [
            float(values["-x"]),
            float(values["-y"]),
            float(values["-z"]),
        ]
    bridge = next(
        a
        for a in actions
        if isinstance(a, Node) and a.node_executable == "parameter_bridge"
    )
    args = ["".join(p.perform(context) for p in arg) for arg in bridge.cmd[1:7]]
    assert any(
        "/world/roport_airport/model/nav_crossing/link/body/sensor/contacts/contact@"
        in a
        for a in args
    )
