from itertools import pairwise

import pytest

from roport_control_server.executor.fleet_traffic import (
    clear_prefix,
    edge_key,
    route_resources,
    validate_clearance,
)


def test_edge_reservations_are_shared_in_both_directions_without_aliases():
    assert edge_key("N23", "N07") == edge_key("N07", "N23") == "edge:N07-N23"
    assert edge_key("A-B", "C") != edge_key("A", "B-C")
    assert edge_key("A-B", "C") != edge_key("A%2DB", "C")
    assert edge_key("A%B", "C") != edge_key("A%25B", "C")


@pytest.mark.parametrize("offset", [0.0, 100.0, -100.0])
def test_exact_clearance_releases_old_node_but_keeps_incoming_edge(offset):
    route = ("start", "junction", "goal")
    positions = {
        "start": (offset, 0),
        "junction": (offset + 0.35, 0),
        "goal": (offset + 1, 0),
    }
    resources = route_resources(route, 1, positions=positions, clearance=0.35)
    assert resources == {
        edge_key("start", "junction"),
        edge_key("junction", "goal"),
        "node:junction",
        "node:goal",
    }


@pytest.mark.parametrize("clearance", [None, 0.35])
def test_short_incoming_segment_preserves_its_start(clearance):
    route = ["A", "B", "C"]
    positions = {"A": (0, 0), "B": (0.349999, 0), "C": (1, 0)}
    resources = route_resources(route, 1, positions=positions, clearance=clearance)
    assert {"node:A", edge_key("A", "B")} <= resources


@pytest.mark.parametrize(
    ("points", "released_at"),
    [
        ([(0, 0), (0.5, 0), (0.5, 0.5), (0.1, 0.1), (1, 1)], 4),
        ([(0, 0), (0.5, 0), (0.5, 0.5), (-0.5, -0.5), (-0.5, -1)], 3),
    ],
)
def test_future_return_or_crossing_keeps_old_node_without_reacquisition(
    points, released_at
):
    route = ["A", "B", "C", "D", "E"]
    positions = dict(zip(route, points, strict=True))
    previous = route_resources(route, 0, positions=positions, clearance=0.35)
    for index, (a, b) in enumerate(pairwise(route), 1):
        current = route_resources(route, index, positions=positions, clearance=0.35)
        assert current <= previous
        assert ("node:A" not in current) is (index >= released_at)
        assert edge_key(a, b) in current
        previous = current


@pytest.mark.parametrize("clearance", [False, 0, -1, float("inf"), float("nan")])
def test_invalid_clearance_is_rejected(clearance):
    with pytest.raises(ValueError, match="positive calibrated distance"):
        validate_clearance(["A"], {"A": (0, 0)}, clearance)


@pytest.mark.parametrize("positions", [{}, {"A": (0, float("nan"))}, {"A": (0,)}])
def test_calibration_must_cover_every_node_with_finite_coordinates(positions):
    with pytest.raises(ValueError, match="finite"):
        validate_clearance(["A"], positions, 0.35)


def test_valid_clearance_accepts_coordinates_and_uncalibrated_fallback():
    validate_clearance(["A"], {"A": (0, 0)}, 0.35)
    validate_clearance(["A"], None, None)


def test_follower_can_take_released_short_route_while_leader_is_still_active():
    nodes = {
        "N05": ["N07", "N15"],
        "N07": ["N05", "N18", "N23"],
        "N18": ["N07", "N01"],
        "N01": ["N18", "detour"],
        "N23": ["N07", "N21"],
        "N21": ["N23"],
        "N15": ["N05", "detour"],
        "detour": ["N15", "N01"],
    }
    leader = ["N07", "N23", "N21"]
    positions = {"N07": (0, 0), "N23": (1, 0), "N21": (2, 0)}

    def follower(occupied, moving, resources):
        return clear_prefix(
            nodes,
            "N05",
            "N18",
            occupied=occupied,
            moving_nodes=moving,
            reserved=resources,
        )

    assert follower({"N07"}, {"N07"}, route_resources(leader)) == ["N05"]
    assert follower(
        {"N23"},
        {"N23"},
        route_resources(leader, 1, positions=positions, clearance=0.35),
    ) == ["N05", "N07", "N18"]
    assert follower({"N07"}, set(), set()) == ["N05", "N15", "detour", "N01", "N18"]


def test_reserved_reverse_edge_blocks_prefix_even_if_nodes_were_released():
    nodes = {"A": ["B"], "B": ["A", "C"], "C": ["B"]}
    assert clear_prefix(
        nodes,
        "A",
        "C",
        occupied=set(),
        moving_nodes=set(),
        reserved={edge_key("B", "A")},
    ) == ["A"]
    with pytest.raises(ValueError, match="start node occupied"):
        clear_prefix(
            nodes, "A", "C", occupied=set(), moving_nodes=set(), reserved={"node:A"}
        )
