"""Calibrated node clearance is generic across names, dense graphs and frames."""

from copy import deepcopy
from itertools import pairwise

import pytest
from test_fleet import fleet as fleet  # noqa: PLC0414
from test_pinky_trial import profile as profile  # noqa: PLC0414

from roport_control_server.executor.fleet import FleetControl
from roport_control_server.executor.plans import shortest_route


class MemoryStore:
    def __init__(self):
        self.data = {}

    def save(self, data):
        self.data = deepcopy(data)


def control_for(positions, route, *, clearance=0.35):
    nodes = {node: [] for node in route}
    for a, b in pairwise(route):
        nodes[a].append(b)
        nodes[b].append(a)
    config = {
        "nodes": nodes,
        "graph_version": "clearance-test",
        "map_version": "clearance-test",
        "command_timeout_s": 30,
    }
    if clearance is not None:
        config.update(node_positions=positions, node_clearance_m=clearance)
    control = FleetControl(config, MemoryStore(), clock=lambda: 1)
    control.data["roles"] = {
        "pinky1": "logistics",
        "pinky2": None,
        "pinky3": None,
    }
    return control


def route_resources(control, route, index):
    return control.command_resources(
        {"robot": "pinky1", "parking": "test-parking"},
        ("pinky1", "line_route_command", {"route": route}),
        index,
    )


def activate(control, route, index):
    control.data["tasks"]["request"] = {
        "robot": "pinky1",
        "parking": "test-parking",
        "stage": "custom",
        "status": "RUNNING",
    }
    control.data["commands"]["command"] = {
        "task": "request",
        "message": {
            "target": "pinky1",
            "type": "line_route_command",
            "payload": {"route": route},
        },
        "participants": ["pinky1"],
        "sent_at": 1,
        "route_index": index,
    }
    control.data["resources"] = control.reservations(control.data)
    control.data["enabled"] = True
    control.heard["pinky1"] = 1


def test_dense_route_keeps_older_node_and_holds_for_its_occupant():
    route = ["start", "junction", "next", "goal"]
    control = control_for(
        dict(zip(route, [(0, 0), (0.1, 0), (0.2, 0), (1, 0)], strict=True)),
        route,
    )
    assert shortest_route(control.config["nodes"], "start", "goal") == route
    activate(control, route, 2)
    assert "node:start" in control.data["resources"]
    assert control.edge("junction", "next") in control.data["resources"]
    control.observed["pinky2"] = {"state": {"node": "start", "mode": "line"}}
    control.heard["pinky2"] = 1
    control.check_active()
    assert control.data["fault"] == "route occupied: start"


@pytest.mark.parametrize(
    "route",
    [
        ["N17", "N15", "N05"],
        ["N16", "N14", "N02"],
        ["H1", "N09", "N08", "H2"],
    ],
)
def test_current_map_short_runs_keep_older_nearby_nodes(fleet, route):
    control = control_for(fleet.config["node_positions"], route)
    retained = route_resources(control, route, len(route) - 1)
    assert "node:" + route[0] in retained
    assert control.edge(route[-2], route[-1]) in retained


@pytest.mark.parametrize(
    "names", [("alpha", "beta", "gamma"), ("zone-1", "junction-8", "dock-4")]
)
def test_long_incoming_edge_releases_start_for_any_node_names(names):
    control = control_for(
        dict(zip(names, [(0, 0), (0.5, 0), (1, 0)], strict=True)), names
    )
    retained = route_resources(control, names, 1)
    assert "node:" + names[0] not in retained
    assert "node:" + names[1] in retained
    assert control.edge(names[0], names[1]) in retained


def test_hyphenated_edge_names_do_not_alias_other_edges():
    first = FleetControl.edge("A-B", "C")
    second = FleetControl.edge("A", "B-C")
    assert first != second
    assert first == FleetControl.edge("C", "A-B")
    assert second == FleetControl.edge("B-C", "A")
    # Existing map resource keys remain compatible with saved checkpoints.
    assert FleetControl.edge("N23", "N07") == "edge:N07-N23"


@pytest.mark.parametrize("offset", [0.0, 100.0, -100.0])
def test_exact_threshold_is_invariant_to_coordinate_translation(offset):
    route = ["A", "B", "C"]
    control = control_for(
        {"A": (offset, 0), "B": (offset + 0.35, 0), "C": (offset + 1, 0)},
        route,
    )
    retained = route_resources(control, route, 1)
    assert "node:A" not in retained
    assert control.edge("A", "B") in retained


def test_tolerance_does_not_release_an_actually_short_segment():
    route = ["A", "B", "C"]
    control = control_for({"A": (0, 0), "B": (0.349999, 0), "C": (1, 0)}, route)
    assert "node:A" in route_resources(control, route, 1)


def test_unconfigured_clearance_preserves_existing_incoming_segment():
    route = ["A", "B", "C"]
    control = control_for(
        {"A": (0, 0), "B": (2, 0), "C": (3, 0)}, route, clearance=None
    )
    retained = route_resources(control, route, 1)
    assert "node:A" in retained
    assert control.edge("A", "B") in retained


def test_literal_escape_text_does_not_alias_hyphenated_edge_names():
    assert FleetControl.edge("A-B", "C") != FleetControl.edge("A%2DB", "C")
    assert FleetControl.edge("A%B", "C") != FleetControl.edge("A%25B", "C")


@pytest.mark.parametrize(
    ("points", "release_index"),
    [
        ([(0, 0), (0.5, 0), (0.5, 0.5), (0.1, 0.1), (1, 1)], 4),
        ([(0, 0), (0.5, 0), (0.5, 0.5), (-0.5, -0.5), (-0.5, -1)], 3),
    ],
)
def test_future_return_or_crossing_keeps_old_node_without_reacquisition(
    points, release_index
):
    route = ["A", "B", "C", "D", "E"]
    control = control_for(dict(zip(route, points, strict=True)), route)
    activate(control, route, 0)
    previous = set(control.data["resources"])
    for index, node in enumerate(route[1:], 1):
        control.route_progress("command", node)
        current = set(control.data["resources"])
        assert current <= previous, "progress cannot reacquire released resources"
        assert ("node:A" not in current) is (index >= release_index)
        assert control.edge(route[index - 1], node) in current
        previous = current
    final = deepcopy(control.data)
    control.route_progress("command", "B")
    assert control.data == final, "late progress cannot restore old reservations"
