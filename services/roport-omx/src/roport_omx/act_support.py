"""Validation helpers and the evaluation Profile shared by the local ACT runtime modules.

Copied from the deployed panel runtime (its dedicated.py, everything before DedicatedHost).
The DedicatedHost state machine is not part of this module; task/omx keeps its own dedicated.py.
"""

import json
import math
from copy import deepcopy
from datetime import datetime

PROFILE_VERSION = "omx.dedicated/1"
PHASES = ("GRASPING", "TRANSFERRING", "RELEASING", "RETURNING")
WAITS = ("WAIT_GRASP", "WAIT_SEATED", "WAIT_RELEASE", "WAIT_SAFE")
# Supervisor verdict kind required per WAIT (grasp/seated/release/safe), same order.
VERDICTS = ("GRASPED", "SEATED", "RELEASED", "SAFE")
BINDINGS = (
    "execution_id",
    "operation",
    "assigned_arm_id",
    "phase",
    "phase_revision",
    "policy_id",
    "model_version",
    "policy_bundle_version",
    "feature_schema_version",
    "calibration_version",
    "request_id",
    "observation_seq",
    "captured_at",
    "expires_at",
)


class Rejected(ValueError):
    """Input was rejected; an active execution is held on unsafe input."""

    def __init__(self, reason, code=None):
        super().__init__(reason)
        self.code = code


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def vector(value):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 6
        or not all(map(finite, value))
    ):
        raise Rejected("expected six finite positions")
    return tuple(value)


def timestamp(value):
    if not isinstance(value, str):
        raise Rejected("timestamp must be offset ISO8601")
    try:
        result = datetime.fromisoformat(value)
    except ValueError as exc:
        raise Rejected("invalid timestamp") from exc
    if result.tzinfo is None:
        raise Rejected("timestamp needs UTC offset")
    return result.timestamp()


def encoded(value):
    try:
        return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))
    except (ValueError, TypeError) as exc:
        raise Rejected("input must be finite JSON") from exc


def nonempty(value):
    return isinstance(value, str) and bool(value.strip())


def bounds(value):
    if not isinstance(value, list) or len(value) != 6:
        raise Rejected("six joint bounds required")
    for pair in value:
        if not isinstance(pair, list) or len(pair) != 2 or not all(map(finite, pair)):
            raise Rejected("invalid joint bounds")
        if pair[0] > pair[1]:
            raise Rejected("reversed joint bounds")
    return value


def inside(positions, limits):
    return all(lo <= pos <= hi for pos, (lo, hi) in zip(positions, limits, strict=True))


class Profile:
    """Explicit profile: missing physical and timing values have no defaults."""

    @classmethod
    def from_dict(cls, data):
        return cls(data)

    def __init__(self, data):
        if not isinstance(data, dict):
            raise Rejected("profile must be an object")
        self.data = deepcopy(data)
        d = self.data
        encoded(d)
        for name in (
            "workcell_id",
            "site_id",
            "control_id",
            "profile_version",
            "policy_bundle_version",
            "feature_schema_version",
            "calibration_version",
            "workcell_profile_version",
        ):
            if not nonempty(d.get(name)):
                raise Rejected(f"profile requires {name}")
        if d["profile_version"] != PROFILE_VERSION or d["workcell_id"] != "shelf-01":
            raise Rejected("unsupported workcell/profile")
        roles = d.get("roles")
        if not isinstance(roles, dict) or set(roles) != {"INBOUND", "OUTBOUND"}:
            raise Rejected("two operation roles required")
        if not all(map(nonempty, roles.values())) or len(set(roles.values())) != 2:
            raise Rejected("distinct arms required")
        for name in ("slot_ids", "package_ids"):
            if (
                not isinstance(d.get(name), list)
                or not d[name]
                or not all(map(nonempty, d[name]))
            ):
                raise Rejected(f"profile requires {name}")
        if "slot_levels" in d:
            levels = d["slot_levels"]
            if (
                not isinstance(levels, dict)
                or set(levels) != set(d["slot_ids"])
                or any(
                    type(level) is not int or level not in (1, 2)
                    for level in levels.values()
                )
            ):
                raise Rejected(
                    "slot_levels must map every configured slot to level 1 or 2"
                )
        names = d.get("feature_names")
        if (
            not isinstance(names, list)
            or len(names) != 6
            or not all(map(nonempty, names))
            or len(set(names)) != 6
        ):
            raise Rejected("six ordered unique feature names required")
        if type(d.get("use_degrees")) is not bool:
            raise Rejected("use_degrees must be boolean")
        expected_units = ["degrees" if d["use_degrees"] else "range_m100_100"] * 5 + [
            "range_0_100"
        ]
        if d.get("feature_units") != expected_units:
            raise Rejected(
                "feature units must preserve OMX joint/gripper normalization"
            )
        directions = d.get("joint_directions")
        if (
            not isinstance(directions, list)
            or len(directions) != 6
            or any(type(x) is not int or x not in (-1, 1) for x in directions)
        ):
            raise Rejected("six explicit joint directions required")
        bounds(d.get("joint_bounds"))
        if not d["use_degrees"] and any(
            lo < -100 or hi > 100 for lo, hi in d["joint_bounds"][:5]
        ):
            raise Rejected("arm joint bounds must use range_m100_100")
        if any(lo < 0 or hi > 100 for lo, hi in [d["joint_bounds"][-1]]):
            raise Rejected("gripper bounds must use range_0_100")
        step = vector(d.get("max_step"))
        if any(x <= 0 for x in step):
            raise Rejected("positive max_step required")
        tolerance = vector(d.get("safe_tolerance"))
        if any(x < 0 for x in tolerance):
            raise Rejected("nonnegative safe tolerance required")
        safe = d.get("safe_positions")
        if not isinstance(safe, dict) or set(safe) != set(roles.values()):
            raise Rejected("safe positions required for both arms")
        for pos in safe.values():
            if not inside(vector(pos), d["joint_bounds"]):
                raise Rejected("safe positions out of bounds")
        timing = d.get("timing")
        required = {
            "clock_skew",
            "permit_max_age",
            "permit_lease",
            "observation_max_age",
            "wait_timeout",
            "inference_timeout",
            "phase_timeout",
        }
        if not isinstance(timing, dict) or set(timing) != required:
            raise Rejected("explicit timing fields required")
        if any(not finite(x) or x <= 0 for x in timing.values()):
            raise Rejected("positive finite timing values required")
        policies = d.get("policies")
        if not isinstance(policies, dict) or set(policies) != set(roles):
            raise Rejected("policies required for both operations")
        for operation in roles:
            if set(policies[operation]) != set(PHASES):
                raise Rejected("four phase policies required")
            if len({p.get("policy_id") for p in policies[operation].values()}) != 4:
                raise Rejected("four separate phase policy registrations required")
            for policy in policies[operation].values():
                if not all(
                    nonempty(policy.get(k)) for k in ("policy_id", "model_version")
                ):
                    raise Rejected("policy/model identity required")
                for name in ("entry_bounds", "exit_bounds"):
                    for pair, physical in zip(
                        bounds(policy.get(name)), d["joint_bounds"], strict=True
                    ):
                        if pair[0] < physical[0] or pair[1] > physical[1]:
                            raise Rejected(
                                "phase envelope exceeds physical joint bounds"
                            )
