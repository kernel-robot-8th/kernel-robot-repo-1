"""Re-register the waiting arm's safe pose from the panel.

The panel refuses to start while the arm that is not working is away from its registered safe pose
(it must stay clear of the working arm). Arms drift after power cycles, so the operator can pin the
waiting arm's current pose instead of editing files. Only a still, folded, in-bounds pose is accepted.
"""

import json
import math
import os
import tempfile
import time
from datetime import UTC, datetime
from pathlib import Path

from .act_support import Rejected, inside, vector

# A folded arm keeps its shoulder lift low and its elbow bent; an extended arm is never a "safe" rest pose.
FOLDED_LIFT_MAX = -55.0
FOLDED_ELBOW_MIN = 45.0
# Largest accepted difference per joint from the previous registration (normalized units): a sanity
# limit against a bad read, not a safety limit.
MAX_SAFE_CHANGE = 25.0
STILL_TOLERANCE = 0.3
STILL_WAIT_S = 1.0


def register_safe_position(profile, inspect, pause=time.sleep):
    """Pin the measured pose of the waiting arm as its safe pose for this profile and save it.

    `inspect()` returns a report with `measured`: every arm's position. It must raise for torque on,
    motor errors or unreadable motors, and those errors are not swallowed here.
    """
    safe = profile["host_profile"]["safe_positions"]
    waiting = [arm for arm in safe if arm != profile.get("arm_id")]
    if profile.get("arm_id") not in safe or len(waiting) != 1:
        raise Rejected("대기 팔을 하나로 정할 수 없어 등록할 수 없습니다.")
    arm = waiting[0]
    first = vector(inspect()["measured"][arm])
    pause(STILL_WAIT_S)
    pose = vector(inspect()["measured"][arm])
    if max(abs(a - b) for a, b in zip(first, pose, strict=True)) > STILL_TOLERANCE:
        raise Rejected(
            "대기 팔이 움직이고 있습니다. 팔을 받쳐 가만히 둔 뒤 다시 시도하세요."
        )
    if pose[1] > FOLDED_LIFT_MAX or pose[2] < FOLDED_ELBOW_MIN:
        raise Rejected(
            f"대기 팔이 접힌 자세가 아닙니다 (어깨들기 {pose[1]:.1f}은 {FOLDED_LIFT_MAX:g} 이하, "
            f"팔꿈치 {pose[2]:.1f}은 {FOLDED_ELBOW_MIN:g} 이상이어야 합니다). 팔을 접은 뒤 다시 시도하세요."
        )
    if not inside(pose, profile["host_profile"]["joint_bounds"]):
        raise Rejected(
            "대기 팔 자세가 이 프로파일의 관절 범위 밖이라 등록할 수 없습니다."
        )
    previous = list(safe[arm])
    change = max(abs(a - b) for a, b in zip(pose, previous, strict=True))
    if change > MAX_SAFE_CHANGE:
        raise Rejected(
            f"기존 등록값과 차이가 너무 큽니다 ({change:.1f} > {MAX_SAFE_CHANGE:g}). "
            "측정이 잘못됐을 수 있어 등록하지 않습니다."
        )
    # Save first: a failed save must not leave a pose in memory that no restart would reproduce.
    persisted = save_safe_pose(
        profile.get("safe_pose_file"),
        arm,
        pose,
        safe_pose_basis(profile["host_profile"]),
    )
    safe[arm] = list(pose)
    return {
        "registered": True,
        "arm": arm,
        "pose": list(pose),
        "previous": previous,
        "persisted": persisted,
        "max_change": change,
    }


def propagate_safe_pose(profiles, arm, pose):
    """Give every evaluation profile the same safe pose for `arm`, where the pose fits its joint bounds."""
    applied, skipped = [], []
    for profile_id, profile in profiles.items():
        host = profile.get("host_profile") if isinstance(profile, dict) else None
        if not isinstance(host, dict) or arm not in host.get("safe_positions", {}):
            continue
        if inside(vector(pose), host["joint_bounds"]):
            host["safe_positions"][arm] = list(pose)
            applied.append(profile_id)
        else:
            skipped.append(profile_id)
    return {"applied": applied, "skipped": skipped}


def safe_pose_basis(host_profile):
    """What a saved pose is only valid for: the same calibration and the same joint units."""
    return {
        "calibration_version": host_profile.get("calibration_version"),
        "use_degrees": bool(host_profile.get("use_degrees")),
    }


def load_safe_poses(path, basis=None):
    """Saved poses as {arm: [6 numbers]}; a missing or damaged file, or a bad entry, is ignored.

    With `basis` (see safe_pose_basis) only poses saved under the same calibration and units are used.
    """
    try:
        data = json.loads(Path(path).read_text())
    except (TypeError, OSError, ValueError):
        return {}
    poses = {}
    for arm, entry in data.items() if isinstance(data, dict) else ():
        pose = entry.get("pose") if isinstance(entry, dict) else None
        if basis is not None and (
            not isinstance(entry, dict) or entry.get("basis") != basis
        ):
            continue
        if (
            isinstance(pose, list)
            and len(pose) == 6
            and all(type(x) in (int, float) and math.isfinite(x) for x in pose)
        ):
            poses[arm] = [float(x) for x in pose]
    return poses


def save_safe_pose(path, arm, pose, basis=None):
    """Keep the registration across panel restarts. Returns False when no file is configured."""
    if path is None:
        return False
    path = Path(path)
    current = {}
    try:
        raw = json.loads(path.read_text())
        current = raw if isinstance(raw, dict) else {}
    except (OSError, ValueError):
        pass
    current[arm] = {
        "pose": [float(x) for x in pose],
        "registered_at": datetime.now(UTC).isoformat(timespec="seconds"),
    }
    if basis is not None:
        current[arm]["basis"] = basis
    descriptor, temporary = tempfile.mkstemp(
        dir=path.parent, prefix=path.name, suffix=".tmp"
    )
    try:
        with os.fdopen(descriptor, "w") as handle:
            json.dump(current, handle, indent=1)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise
    return True


def apply_saved_safe_poses(host_profile, saved):
    """At startup: use a saved pose only for an arm the profile knows and only if it fits the bounds."""
    applied = []
    for arm, pose in saved.items():
        if arm in host_profile["safe_positions"] and inside(
            vector(pose), host_profile["joint_bounds"]
        ):
            host_profile["safe_positions"][arm] = list(pose)
            applied.append(arm)
    return applied
