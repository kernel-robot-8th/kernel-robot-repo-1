"""Validate joint and timing limits for a local full-episode ACT evaluation."""

from copy import deepcopy

from .act_support import (
    Profile,
    Rejected,
    bounds,
    encoded,
    finite,
    inside,
    nonempty,
    vector,
)

OMX_FEATURES = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)


def validated_host_profile(profile, operation):
    host = profile.get("host_profile")
    if not isinstance(host, dict) or host.get("profile_version") != "omx.local-act/1":
        return Profile(host).data
    if operation != "evaluate" or not nonempty(profile.get("act_checkpoint")):
        raise Rejected(
            "local ACT profile requires a full-episode checkpoint evaluation"
        )

    host = deepcopy(host)
    encoded(host)
    if not nonempty(host.get("calibration_version")):
        raise Rejected("local ACT profile requires a calibration version")
    if (
        host.get("feature_names") != list(OMX_FEATURES)
        or host.get("use_degrees") is not False
        or host.get("feature_units") != ["range_m100_100"] * 5 + ["range_0_100"]
    ):
        raise Rejected("local ACT requires the exact normalized OMX feature order")
    directions = host.get("joint_directions")
    if (
        not isinstance(directions, list)
        or len(directions) != len(OMX_FEATURES)
        or any(type(value) is not int or value not in (-1, 1) for value in directions)
    ):
        raise Rejected("six explicit joint directions required")

    roles = host.get("roles")
    if (
        not isinstance(roles, dict)
        or set(roles) != {"INBOUND", "OUTBOUND"}
        or not all(map(nonempty, roles.values()))
        or len(set(roles.values())) != 2
    ):
        raise Rejected(
            "local ACT requires distinct inbound and outbound arm identities"
        )
    limits = bounds(host.get("joint_bounds"))
    if any(lo < -100 or hi > 100 for lo, hi in limits[:5]) or not all(
        0 <= value <= 100 for value in limits[-1]
    ):
        raise Rejected("local ACT joint bounds exceed normalized OMX limits")
    if any(value <= 0 for value in vector(host.get("max_step"))):
        raise Rejected("positive max_step required")
    if any(value < 0 for value in vector(host.get("safe_tolerance"))):
        raise Rejected("nonnegative safe tolerance required")
    safe = host.get("safe_positions")
    if not isinstance(safe, dict) or set(safe) != set(roles.values()):
        raise Rejected("safe positions required for both arms")
    if any(not inside(vector(positions), limits) for positions in safe.values()):
        raise Rejected("safe positions out of bounds")

    timing = host.get("timing")
    if (
        not isinstance(timing, dict)
        or set(timing) != {"clock_skew", "observation_max_age", "inference_timeout"}
        or any(not finite(value) or value <= 0 for value in timing.values())
    ):
        raise Rejected(
            "explicit positive clock, observation and inference timing required"
        )
    return host
