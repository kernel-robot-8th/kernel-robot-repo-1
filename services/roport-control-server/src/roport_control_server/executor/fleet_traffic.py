"""Line-route reservations and available prefixes for Fleet dispatch."""

import math
from itertools import pairwise

from .plans import shortest_route


def validate_clearance(nodes, positions, clearance):
    if clearance is not None:
        if (
            type(clearance) not in (int, float)
            or not math.isfinite(clearance)
            or clearance <= 0
        ):
            raise ValueError("node_clearance_m requires a positive calibrated distance")
        if not isinstance(positions, dict):
            raise ValueError("node_positions requires a node-to-position mapping")
        for node in nodes:
            point = positions.get(node)
            if (
                not isinstance(point, (list, tuple))
                or len(point) != 2
                or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in point
                )
            ):
                raise ValueError(
                    f"node_positions requires a finite [x, y] in meters: {node}"
                )


def edge_key(a, b):
    """Both directions reserve the same edge, including escaped node names."""
    return "edge:" + "-".join(
        node.replace("%", "%25").replace("-", "%2D") for node in sorted((a, b))
    )


def _distance_to_segment(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    squared_length = dx * dx + dy * dy
    if not squared_length:
        return math.dist(point, start)
    fraction = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / squared_length
    fraction = max(0, min(1, fraction))
    return math.hypot(
        point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy
    )


def route_resources(route, route_index=0, *, positions=None, clearance=None):
    """Return the nodes and edges still held after confirmed route progress."""
    incoming = route[max(0, route_index - 1) :]
    resources = {edge_key(a, b) for a, b in pairwise(incoming)}
    if not route_index or clearance is None:
        return resources | {"node:" + node for node in incoming}

    remaining = route[route_index:]
    segments = list(pairwise(remaining)) or [(remaining[0], remaining[0])]
    protected = set(remaining)
    # Check the entire remaining path so an old node is never reserved again.
    for node in route[:route_index]:
        distance = min(
            _distance_to_segment(positions[node], positions[a], positions[b])
            for a, b in segments
        )
        if distance < clearance and not math.isclose(
            distance, clearance, rel_tol=0, abs_tol=1e-9
        ):
            protected.add(node)
    return resources | {"node:" + node for node in protected}


def clear_prefix(nodes, start, goal, *, occupied, moving_nodes, reserved):
    """Prefer the short route past moving robots and detour around idle ones."""
    try:
        path = shortest_route(nodes, start, goal, occupied - moving_nodes - {goal})
    except ValueError:
        path = shortest_route(nodes, start, goal)

    if start in occupied or "node:" + start in reserved:
        raise ValueError("start node occupied")
    for index, (a, b) in enumerate(pairwise(path), 1):
        if b in occupied or "node:" + b in reserved or edge_key(a, b) in reserved:
            return path[:index]
    return path
