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
    assert return_waypoints((2.369, -1.274), n12, public_space=False) == []
    assert return_waypoints((.3, -.3), n12, public_space=True) == ['return_entry']
    assert return_waypoints((2.264, -.9), n12, public_space=True) == ['return_entry']


@pytest.mark.parametrize('offset,expected', [(-.02, [300, 2400, 2200]),
                                            (.02, [2300, 2400, 200])])
def test_ir_left_and_right_channel_order(offset, expected):
    line = SimpleNamespace(cx=.5, cy=0., yaw=0., sx=2., sy=.024)
    cal = SimpleNamespace(carpet=(2300, 2400, 2200), tape=(300, 400, 200))
    assert ir_values([line], (0., offset, 0.), cal) == expected


def test_layout_boundaries_and_intermediate_line_nodes_block_nav2():
    from pathlib import Path
    import yaml
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph
    from roport_smartcart.sim_return_h1 import in_public_space, return_waypoints
    path = Path(__file__).parents[1].parent/'roport_gz_world/layout/roport_airport.yaml'
    data = yaml.safe_load(path.read_text())
    # Use different boundaries and a rotated world frame to catch hard-coded coordinates.
    for wall in data['walls']:
        if wall['name'] == 'zone03_left':
            wall['from'][0] = wall['to'][0] = 140
        elif wall['name'] == 'zone03_bottom':
            wall['from'][1] = wall['to'][1] = 96
    data['frame']['yaw_deg'] = 27
    layout = Layout(data)
    assert in_public_space(layout, layout.frame.to_world(141, 97))
    assert not in_public_space(layout, layout.frame.to_world(139, 97))
    assert not in_public_space(layout, layout.frame.to_world(141, 95))
    graph = Graph.from_dict(data)
    n12 = graph.nodes['N12']
    n12_world = layout.frame.to_world(n12.x+3., n12.y)
    for name in ('N09', 'N11', 'H2'):
        node = graph.nodes[name]
        position = layout.frame.to_world(node.x, node.y)
        assert not in_public_space(layout, position)
        with pytest.raises(RuntimeError, match='intermediate line nodes'):
            return_waypoints(position, n12_world, public_space=False)


def test_repeated_shutdown_signals_do_not_interrupt_stop_burst():
    import os
    import subprocess
    import sys
    program = r"""
import os, signal
from roport_smartcart.sim_return_h1 import ignore_shutdown_signals
ignore_shutdown_signals()
stops = []
for index in range(10):
    stops.append((0., 0.))
    os.kill(os.getpid(), (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)[index % 3])
assert len(stops) == 10
print('stopped and cleaned up')
"""
    result = subprocess.run([sys.executable, '-c', program],
                            env=os.environ.copy(), capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'stopped and cleaned up'
