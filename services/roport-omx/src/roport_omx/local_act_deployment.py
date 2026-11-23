"""Trusted local composition for the OMX1 outbound floor-one trial."""

import hashlib
import json
from pathlib import Path

from .act_support import Rejected, inside, vector
from .demonstration_guard import DemonstrationGuard
from .lerobot_runtime import OMX_FEATURES, DeviceLock
from .safe_pose import register_safe_position
from .start_alignment import StartAlignment


def start_report(profile, positions):
    if set(positions) != set(profile["host_profile"]["safe_positions"]):
        raise Rejected("both measured arms required for start inspection")
    positions = {arm: vector(values) for arm, values in positions.items()}
    guard = DemonstrationGuard(profile["demonstration_guard"])
    state = positions[profile["arm_id"]]
    index = min(
        range(min(guard.lookahead, len(guard.frames))),
        key=lambda i: max(
            abs(a - b) / t
            for a, b, t in zip(
                state, guard.frames[i][0], guard.state_tolerance, strict=True
            )
        ),
    )
    target = guard.frames[index][0]
    delta = [a - b for a, b in zip(state, target, strict=True)]
    inactive = all(
        abs(a - b) <= t
        for arm, values in positions.items()
        if arm != profile["arm_id"]
        for a, b, t in zip(
            values,
            profile["host_profile"]["safe_positions"][arm],
            profile["host_profile"]["safe_tolerance"],
            strict=True,
        )
    )
    report = {
        "ready": inactive
        and all(abs(x) <= t for x, t in zip(delta, guard.state_tolerance, strict=True)),
        "unit": "lerobot_normalized",
        "current": state,
        "reference": target,
        "delta": delta,
        "inactive_arm_ready": inactive,
        "measured": {arm: list(values) for arm, values in positions.items()},
        "note": "시작 자세 검사만 수행했습니다. 모델 후보와 영상은 실행 직전에 다시 검사합니다.",
    }
    if "start_alignment" in profile:
        plan = StartAlignment(
            profile["start_alignment"], target_tolerance=guard.state_tolerance
        )
        report["alignment_ready"] = inactive and all(
            abs(a - b) <= t
            for a, b, t in zip(state, plan.start, plan.start_tolerance, strict=True)
        )
        # For adjusting the arm by hand: the pose the alignment move expects to start from.
        report["alignment_start"] = list(plan.start)
        report["alignment_start_delta"] = [
            a - b for a, b in zip(state, plan.start, strict=True)
        ]
        report["alignment_start_tolerance"] = list(plan.start_tolerance)
    return report


# A re-registered start may not be farther than this (normalized units) from the demonstration
# start, per joint: the arm joints sweep the arm through space, wrist roll and gripper do not.
START_JOINTS = ("어깨회전", "어깨들기", "팔꿈치", "손목굽힘", "손목회전", "그리퍼")
MAX_START_TO_TARGET = (12, 12, 12, 12, 25, 15)


def register_alignment_start(profile, report):
    """Pin the measured working-arm pose as this session's alignment start.

    In memory only: the next job builds its alignment plan from the profile, a panel restart
    returns to the deployed settings. Refuses unsafe poses and changes nothing when it does.
    """
    if report.get("inactive_arm_ready") is not True:
        raise Rejected(
            "비작업 팔이 안전 자세에 있지 않아 등록할 수 없습니다. 대기 팔을 접은 자세로 두고 "
            "‘대기 팔 안전 자세 등록’으로 지금 자세를 등록하거나, 등록된 자세로 옮기세요."
        )
    pose = vector(report["current"])
    alignment = profile["start_alignment"]
    if not inside(pose, profile["host_profile"]["joint_bounds"]):
        raise Rejected(
            "현재 자세가 시연에서 사용한 관절 범위 밖이라 등록할 수 없습니다."
        )
    distances = [abs(a - b) for a, b in zip(pose, alignment["target"], strict=True)]
    for name, distance, limit in zip(
        START_JOINTS, distances, MAX_START_TO_TARGET, strict=True
    ):
        if distance > limit:
            raise Rejected(
                f"현재 자세가 시연 시작 자세에서 너무 멉니다 ({name} {distance:.1f} > {limit}). "
                "팔을 접힌 대기 자세 근처로 되돌린 뒤 다시 시도하세요."
            )
    far = max(distances)
    if far / min(alignment["max_rate"]) >= alignment["timeout_s"]:
        raise Rejected(
            "현재 자세에서 정렬이 제한 시간 안에 끝나지 않아 등록할 수 없습니다."
        )
    previous = list(alignment["start"])
    alignment["start"] = list(pose)
    return {
        "registered": True,
        "persisted": False,
        "start": list(pose),
        "previous": previous,
        "max_distance_to_target": far,
    }


def inspect_start(profile):
    """Only SDK PING/READ packets under the same exclusive lock as device jobs."""
    import dynamixel_sdk as sdk

    positions = {}
    lock = DeviceLock(profile["shared_lock_path"]).acquire()
    try:
        for name, config in profile["follower_configs"].items():
            entry = profile["calibration_files"][name]
            raw = Path(entry["path"]).read_bytes()
            if hashlib.sha256(raw).hexdigest() != entry["sha256"]:
                raise Rejected("calibration hash changed")
            calibration = json.loads(raw)
            port, packet = sdk.PortHandler(config.port), sdk.PacketHandler(2.0)
            values = []
            try:
                if not port.openPort() or not port.setBaudRate(1_000_000):
                    raise Rejected("cannot open follower for read-only inspection")
                for feature in OMX_FEATURES:
                    joint = feature.removesuffix(".pos")
                    c = calibration[joint]
                    model, comm, error = packet.ping(port, c["id"])
                    if (
                        comm != sdk.COMM_SUCCESS
                        or error
                        or model != (1060 if c["id"] <= 13 else 1200)
                    ):
                        raise Rejected("motor identity/read failure")
                    row = {}
                    for field, address, reader in (
                        ("drive_mode", 10, packet.read1ByteTxRx),
                        ("torque", 64, packet.read1ByteTxRx),
                        ("error", 70, packet.read1ByteTxRx),
                        ("watchdog", 98, packet.read1ByteTxRx),
                        ("position", 132, packet.read4ByteTxRx),
                        ("homing_offset", 20, packet.read4ByteTxRx),
                        ("range_max", 48, packet.read4ByteTxRx),
                        ("range_min", 52, packet.read4ByteTxRx),
                    ):
                        value, comm, error = reader(port, c["id"], address)
                        if comm != sdk.COMM_SUCCESS or error:
                            raise Rejected(f"motor read failed: {joint}/{field}")
                        row[field] = (
                            value - 2**32
                            if field in ("position", "homing_offset") and value >= 2**31
                            else value
                        )
                    if row["torque"] or row["error"] or not 0 <= row["watchdog"] <= 127:
                        raise Rejected(
                            "토크/모터 오류 상태를 현장에서 확인하세요. 검사에서는 모터 설정을 변경하지 않습니다."
                        )
                    if c["drive_mode"] != 0 or any(
                        row[k] != c[k]
                        for k in (
                            "drive_mode",
                            "homing_offset",
                            "range_min",
                            "range_max",
                        )
                    ):
                        raise Rejected("motor calibration differs from pinned file")
                    if not c["range_min"] <= row["position"] <= c["range_max"]:
                        raise Rejected("position outside encoder calibration")
                    ratio = (row["position"] - c["range_min"]) / (
                        c["range_max"] - c["range_min"]
                    )
                    values.append(
                        ratio * 100 if joint == "gripper" else ratio * 200 - 100
                    )
                positions[name] = values
            finally:
                if port.is_open:
                    port.closePort()
        return start_report(profile, positions)
    finally:
        lock.release()


def build_outbound_floor1(config, settings):
    """Settings are a private, reviewed deployment file; never accepted via HTTP."""
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.robots.omx_follower.config_omx_follower import OmxFollowerConfig
    from lerobot.teleoperators.omx_leader.config_omx_leader import OmxLeaderConfig

    source = (
        config["profiles"].get("omx1_outbound")
        or config["profiles"]["omx1_outbound_floor1"]
    )
    active = source["manual_teleop"]["pairs"][0]
    inactive = (
        config["profiles"].get("omx2_outbound")
        or config["profiles"]["omx2_outbound_floor1"]
    )["manual_teleop"]["pairs"][0]
    files, followers = {}, {}
    host = settings["host_profile"]
    for name, device in {
        "omx1": active["follower"],
        "omx2": inactive["follower"],
        "leader": active["leader"],
    }.items():
        files[name] = {
            "path": device["calibration_file"],
            "sha256": device["sha256"],
            "calibration_version": host["calibration_version"],
        }
        args = {
            "port": device["port"],
            "id": device["id"],
            "calibration_dir": Path(device["calibration_file"]).parent,
        }
        if name == "leader":
            leader = OmxLeaderConfig(**args)
        else:
            cameras = (
                {
                    key: OpenCVCameraConfig(
                        index_or_path=Path(value["path"]),
                        width=value["width"],
                        height=value["height"],
                        fps=value["fps"],
                        fourcc=value["fourcc"],
                    )
                    for key, value in source["record_cameras"].items()
                }
                if name == "omx1"
                else {}
            )
            followers[name] = OmxFollowerConfig(
                **args,
                cameras=cameras,
                use_degrees=False,
                disable_torque_on_disconnect=False,
                max_relative_target=dict(
                    zip(
                        (key.removesuffix(".pos") for key in OMX_FEATURES),
                        host["max_step"],
                        strict=True,
                    )
                ),
            )

    profile = dict(
        settings,
        operations=["evaluate"],
        device_mode="hardware",
        authorization={"supervisor": True, "hardware": True},
        operation="OUTBOUND",
        arm_id="omx1",
        shelf_level=1,
        shared_lock_path=source["shared_lock_path"],
        preview_lock_path=source["preview_lock_path"],
        preview_cameras=source["preview_cameras"],
        follower_configs=followers,
        leader_config=leader,
        calibration_files=files,
        camera_mapping={
            f"observation.images.{key}": key for key in source["record_cameras"]
        },
        camera_devices={
            key: value["path"] for key, value in source["record_cameras"].items()
        },
        dataset_profile={
            "cameras": {
                f"observation.images.{key}": {
                    "shape": [value["height"], value["width"], 3],
                    "fps": value["fps"],
                }
                for key, value in source["record_cameras"].items()
            }
        },
        capture_metadata=lambda sample: sample,
        supervise_local_act=lambda sample: "run",
    )
    if "start_alignment" in profile:
        guard = DemonstrationGuard(profile["demonstration_guard"])
        plan = StartAlignment(
            profile["start_alignment"], target_tolerance=guard.state_tolerance
        )
        velocity = profile.get("start_alignment_motor_velocity")
        if (
            type(velocity) is not int
            or not 1 <= velocity <= 32767
            or not inside(plan.start, host["joint_bounds"])
            or not inside(plan.target, host["joint_bounds"])
            or plan.target != guard.frames[0][0]
            or any(
                tolerance > limit
                for tolerance, limit in zip(
                    plan.start_tolerance, guard.state_tolerance, strict=True
                )
            )
        ):
            raise Rejected(
                "alignment requires bounded poses, demo target and motor velocity"
            )
        profile["alignment_start_report"] = lambda positions: start_report(
            profile, positions
        )
        # Read-only inspection first (needs torque off and a quiet arm), then pin that pose.
        profile["register_alignment_start"] = lambda: register_alignment_start(
            profile, inspect_start(profile)
        )

    def verify(devices):
        positions = {
            name: [values[key.removesuffix(".pos")] for key in OMX_FEATURES]
            for name, arm in devices.arms.items()
            for values in [arm.bus.sync_read("Present_Position")]
        }
        report = start_report(profile, positions)
        if devices.profile.get("_start_alignment") is not None:
            if not report.get("alignment_ready"):
                raise Rejected(
                    "등록한 정렬 출발 자세 또는 비작업 팔 자세가 일치하지 않습니다."
                )
            return True
        if not report["ready"]:
            raise Rejected(
                "시연 시작 자세 불일치 (정규화 단위): "
                + json.dumps(
                    {
                        key: report[key]
                        for key in (
                            "current",
                            "reference",
                            "delta",
                            "inactive_arm_ready",
                        )
                    }
                )
            )
        return True

    profile["startup_verify"] = verify
    profile["evaluation_preflight"] = lambda: inspect_start(profile)
    # Waiting arm: read twice, then pin its still, folded pose as the safe pose and keep it across restarts.
    profile["register_safe_position"] = lambda: register_safe_position(
        profile, lambda: inspect_start(profile)
    )
    from .panel_act import PanelAct

    # Imports/weight loading must not starve the live panel heartbeat during startup.
    profile["act_policy"] = PanelAct(Path(profile["act_checkpoint"]))
    return profile
