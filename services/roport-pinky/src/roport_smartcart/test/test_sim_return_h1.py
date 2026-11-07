"""The geometric IR fixture must observe tape, not invent all-clear readings."""
import math
from types import SimpleNamespace

import pytest

from roport_smartcart.sim_return_h1 import ir_values, wrap


@pytest.mark.parametrize('angle', [0., math.pi/2, -.8])
def test_ir_follows_rotated_tape_and_distinguishes_outside(angle):
    # Sensor ahead at 3 cm, footprint radius 4 mm, tape width 24 mm.
    line = SimpleNamespace(cx=.5*math.cos(angle), cy=.5*math.sin(angle),
                           yaw=angle, sx=2., sy=.024)
    cal = SimpleNamespace(carpet=(2300, 2400, 2200), tape=(300, 400, 200))
    assert ir_values([line], (0., 0., angle), cal) == [2300, 400, 2200]
    assert ir_values([line], (-.1*math.sin(angle), .1*math.cos(angle), angle), cal) == list(cal.carpet)


def test_handoff_pose_puts_centre_ir_on_n12_and_h1_route_exists():
    from pathlib import Path
    import yaml
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph, plan_route
    package = Path(__file__).parents[1]
    path = package.parent/'roport_gz_world/layout/roport_airport.yaml'
    layout = Layout(yaml.safe_load(path.read_text()))
    graph = Graph.load(path)
    node = graph.nodes['N12']
    x, y = layout.frame.to_world(node.x+3., node.y)
    yaw = math.pi+math.radians(layout.frame.yaw_deg)
    assert (x+.03*math.cos(yaw), y+.03*math.sin(yaw)) == pytest.approx(
        layout.frame.to_world(node.x, node.y))
    legs = plan_route(graph, ['N12', 'N08', 'N09', 'H1'])
    assert [leg.to for leg in legs] == ['N08', 'N09', 'H1']


def test_wrap_handles_half_turn_without_direction_jump():
    assert wrap(2*math.pi+.1) == pytest.approx(.1)
    assert abs(wrap(3*math.pi)) == pytest.approx(math.pi)


def test_return_rejects_physical_domain_before_creating_ros_node(monkeypatch):
    from roport_smartcart.sim_return_h1 import main
    monkeypatch.setenv('ROS_DOMAIN_ID', '13')
    monkeypatch.setattr('sys.argv', ['sim_return_h1'])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2


def test_resume_near_handoff_keeps_progress_instead_of_turning_back():
    from roport_smartcart.sim_return_h1 import return_waypoints
    n12 = (2.264, -1.284)
    assert return_waypoints((2.369, -1.274), n12) == []
    assert return_waypoints((.3, -.3), n12) == ['return_entry']
    assert return_waypoints((2.264, -.9), n12) == ['return_entry']
