"""Check paired joint observations and targets against an ordered reference run."""

import json
from pathlib import Path

from .act_support import Rejected, finite, vector

# Widest corridor tolerance studied offline; a wider one stops rejecting poses 10 units off.
MAX_CORRIDOR_TOLERANCE = 10


def _corridor(config):
    """Reviewed poses for a supervised trial: (points, tolerance) in normalized units."""
    import hashlib
    import io

    import numpy as np

    if not isinstance(config, dict) or set(config) != {"path", "sha256", "tolerance"}:
        raise Rejected("explicit corridor configuration required")
    path = Path(config["path"])
    if not path.is_absolute():
        raise Rejected("absolute corridor path required")
    tolerance = config["tolerance"]
    if not finite(tolerance) or not 0 < tolerance <= MAX_CORRIDOR_TOLERANCE:
        raise Rejected("corridor tolerance must be positive and at most 10")
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != config["sha256"]:
        raise Rejected("corridor point file changed")
    points = np.load(io.BytesIO(raw), allow_pickle=False)
    if points.ndim != 2 or points.shape[1] != 6 or not len(points):
        raise Rejected("corridor requires six-joint reviewed poses")
    if not np.isfinite(points).all():
        raise Rejected("corridor requires finite reviewed poses")
    return points.astype(np.float64), tolerance


class DemonstrationGuard:
    def __init__(self, config):
        if not isinstance(config, dict) or set(config) - {"corridor"} != {
            "path",
            "state_tolerance",
            "action_tolerance",
            "lookahead_frames",
        }:
            raise Rejected("explicit demonstration guard configuration required")
        self.corridor = _corridor(config["corridor"]) if "corridor" in config else None
        path = Path(config["path"])
        if not path.is_absolute():
            raise Rejected("absolute demonstration reference path required")
        self.state_tolerance = vector(config["state_tolerance"])
        self.action_tolerance = vector(config["action_tolerance"])
        if any(x <= 0 for x in (*self.state_tolerance, *self.action_tolerance)):
            raise Rejected("positive demonstration guard tolerances required")
        self.lookahead = config["lookahead_frames"]
        if type(self.lookahead) is not int or not 1 <= self.lookahead <= 100:
            raise Rejected("demonstration lookahead must be between 1 and 100 frames")
        reference = json.loads(path.read_text())
        if not isinstance(reference, list) or not reference:
            raise Rejected("demonstration reference must contain ordered frames")
        self.frames = []
        for frame in reference:
            if not isinstance(frame, dict) or set(frame) != {"state", "action"}:
                raise Rejected(
                    "demonstration frame must pair the observed state and action"
                )
            self.frames.append((vector(frame["state"]), vector(frame["action"])))
        self.possible_frames = {0}

    def check(self, state, action, *, advance=True):
        state, action = vector(state), vector(action)
        if self.corridor is not None:
            # Supervised trial: stay near any reviewed pose; order and timing are not checked.
            points, tolerance = self.corridor
            for which, pose in (("state", state), ("action", action)):
                distance = float(abs(points - pose).max(axis=1).min())
                if not distance <= tolerance:
                    detail = {
                        "unit": "lerobot_normalized",
                        "which": which,
                        "distance": distance,
                        "tolerance": tolerance,
                        "state": state,
                        "action": action,
                    }
                    raise Rejected(
                        "ACT target leaves the reviewed demonstration corridor: "
                        + json.dumps(detail, allow_nan=False)
                    )
            return
        windows = [
            range(start, min(len(self.frames), start + self.lookahead))
            for start in self.possible_frames
        ]
        candidates = {index for window in windows for index in window}
        matches = set()
        for index in candidates:
            observed, commanded = self.frames[index]
            if all(
                finite(actual) and abs(actual - reference) <= tolerance
                for actual, reference, tolerance in zip(
                    (*state, *action),
                    (*observed, *commanded),
                    (*self.state_tolerance, *self.action_tolerance),
                    strict=True,
                )
            ):
                matches.add(index)
        if not matches:
            state_matches = {
                i
                for i in candidates
                if all(
                    abs(a - b) <= t
                    for a, b, t in zip(
                        state, self.frames[i][0], self.state_tolerance, strict=True
                    )
                )
            }
            # Diagnose the target at a compatible pose, not a later task phase.
            nearest = min(
                state_matches or candidates,
                key=lambda i: (
                    max(
                        abs(a - b) / t
                        for a, b, t in zip(
                            (*state, *action),
                            (*self.frames[i][0], *self.frames[i][1]),
                            (*self.state_tolerance, *self.action_tolerance),
                            strict=True,
                        )
                    ),
                    i,
                ),
            )
            observed, commanded = self.frames[nearest]
            detail = {
                "unit": "lerobot_normalized",
                "reference_frame": nearest,
                "state_matching_frames": len(state_matches),
                "state": state,
                "action": action,
                "state_delta": [a - b for a, b in zip(state, observed, strict=True)],
                "action_delta": [a - b for a, b in zip(action, commanded, strict=True)],
                "state_tolerance": self.state_tolerance,
                "action_tolerance": self.action_tolerance,
            }
            raise Rejected(
                "ACT target leaves the reviewed demonstration path: "
                + json.dumps(detail, allow_nan=False)
            )
        if advance:
            reachable = set()
            for window in windows:
                matched = False
                for index in window:
                    if index in matches:
                        reachable.add(index)
                        matched = True
                    elif matched:
                        # A repeated pose after an excursion is not the same pause.
                        break
            self.possible_frames = reachable
