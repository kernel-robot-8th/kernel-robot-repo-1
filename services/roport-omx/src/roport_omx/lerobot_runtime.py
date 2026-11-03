"""Supervised LeRobot recording/evaluation; imports never open hardware."""

import hashlib
import json
import os
import time
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from threading import Event

from .dedicated import (
    PHASES,
    WAITS,
    Profile,
    Rejected,
    finite,
    inside,
    nonempty,
    timestamp,
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
_RECOVERY_LOCKS = {}


class DeviceLock:
    """Advisory deployment lock, shared with the external panel and Host."""

    def __init__(self, path):
        self.path = Path(path)
        if not self.path.is_absolute():
            raise Rejected("shared device lock must be an absolute deployment path")
        self.file = None

    def acquire(self, *, recovery=False):
        try:
            import fcntl

            self.file = self.path.open("a+")
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.file.seek(0)
            if self.file.read().strip() and not recovery:
                raise Rejected("device lock requires physical reconciliation")
        except (ImportError, OSError) as exc:
            self.release()
            raise Rejected("shared device lock unavailable or already owned") from exc
        except Rejected:
            self.release()
            raise
        return self

    def mark_unresolved(self):
        self._write('{"state":"RECONCILING"}')

    def confirm_safe(self):
        self._write("")

    def _write(self, value):
        self.file.seek(0)
        self.file.truncate()
        self.file.write(value)
        self.file.flush()
        os.fsync(self.file.fileno())

    def release(self):
        if self.file is not None:
            self.file.close()
            self.file = None


def _callback(profile, key):
    callback = profile.get(key)
    if not callable(callback):
        raise Rejected(f"trusted profile requires {key} callback")
    return callback


def _validated(request, profile, operation):
    if not isinstance(request, dict) or request.get("operation") != operation:
        raise Rejected("workflow operation mismatch")
    if request.get("schema_version") != "omx.workflow/1":
        raise Rejected("workflow schema mismatch")
    if not all(nonempty(request.get(key)) for key in ("request_id", "profile_id")):
        raise Rejected("workflow request/profile identity required")
    if not isinstance(request.get("parameters"), dict):
        raise Rejected("workflow parameters must be an object")
    if request["parameters"]:
        raise Rejected("device job parameters are selected by trusted profile")
    hp = Profile(profile.get("host_profile")).data
    mode = profile.get("device_mode")
    auth = profile.get("authorization", {})
    if mode not in ("fake", "hardware") or auth.get("supervisor") is not True:
        raise Rejected("explicit device mode and supervisor authorization required")
    if mode == "hardware" and (
        auth.get("hardware") is not True or hp.get("synthetic_fixture") is True
    ):
        raise Rejected("trusted hardware authorization and field profile required")
    if (
        profile.get("operation") not in hp["roles"]
        or profile.get("arm_id") != hp["roles"][profile["operation"]]
    ):
        raise Rejected("operation/assigned arm mismatch")
    for key in ("fps", "max_frames"):
        if type(profile.get(key)) is not int or profile[key] <= 0:
            raise Rejected(f"explicit positive {key} required")
    if profile.get("_recovery_lock") is not None:
        raise Rejected("physical reconciliation required before another job")
    return hp


def _positions(action, hp):
    if not isinstance(action, dict) or set(action) != set(hp["feature_names"]):
        raise Rejected("exact six action features required")
    values = vector([action[key] for key in hp["feature_names"]])
    if not inside(values, hp["joint_bounds"]):
        raise Rejected("action outside calibrated joint bounds")
    return values


def _capture(devices, hp, profile):
    sample = devices.capture()
    wall_clock = profile.get("wall_clock", time.time)
    if any(
        camera not in sample.get("observation", {})
        for camera in profile.get("camera_mapping", {}).values()
    ):
        raise Rejected("required current camera observations missing")
    arms = sample.get("arms", {})
    if set(arms) != set(hp["safe_positions"]):
        raise Rejected("both measured arm observations required")
    for arm in arms.values():
        if not inside(vector(arm["positions"]), hp["joint_bounds"]):
            raise Rejected("measured arm outside joint bounds")
        age = wall_clock() - timestamp(arm["captured_at"])
        if (
            not -hp["timing"]["clock_skew"]
            <= age
            <= hp["timing"]["observation_max_age"]
        ):
            raise Rejected("stale or future measured arm observation")
    return sample


def _physical_stop(profile, devices, reason):
    try:
        evidence = devices.stop(reason)
        confirmed = (
            isinstance(evidence, dict)
            and evidence.get("confirmed") is True
            and evidence.get("supported") is True
            and evidence.get("manual_recovery_required") is False
            and bool(evidence.get("evidence"))
        )
    except Exception as exc:  # noqa: BLE001 -- Device hook failure never confirms physical safety.
        evidence, confirmed = {"error": str(exc)}, False
    return {"confirmed": confirmed, "evidence": evidence}


def reconcile_devices(profile, devices):
    """Explicit supervisor recovery, never a side effect of submitting another job."""
    path = str(Path(profile["shared_lock_path"]).resolve())
    lock = _RECOVERY_LOCKS.get(path) or DeviceLock(path).acquire(recovery=True)
    result = _physical_stop(profile, devices, "supervised reconciliation")
    if result["confirmed"]:
        lock.confirm_safe()
        lock.release()
        _RECOVERY_LOCKS.pop(path, None)
        profile.pop("_recovery_lock", None)
    else:
        _RECOVERY_LOCKS[path] = lock
    return result


def _run(request, profile, devices, operation, body, *, prepare=None):
    hp = _validated(request, profile, operation)
    lock = DeviceLock(profile["shared_lock_path"]).acquire()
    event = profile.get("stop_event")
    if event is None:
        event = Event()
    started, result = False, {}
    try:
        if event.is_set():
            return {"state": "FAILED", "error": "stopped before device startup"}
        if prepare is not None:
            prepare()
        lock.mark_unresolved()
        started = True  # A partially failed start still needs verified physical stop.
        devices.start(profile, operation)
        result = body(hp, event)
    except Exception as exc:  # noqa: BLE001 -- All adapter failures must enter physical recovery.
        result = {"state": "FAILED", "error": str(exc)}
    finally:
        if started:
            stop = _physical_stop(profile, devices, result.get("error", "job finished"))
            result["physical_stop"] = stop
            if not stop["confirmed"]:
                result["state"] = "RECONCILING"
                profile["_recovery_lock"] = lock
                _RECOVERY_LOCKS[str(lock.path.resolve())] = lock
            else:
                lock.confirm_safe()
                lock.release()
        else:
            lock.release()
    return result


def record(request, profile, devices):
    """Record applied actions; success requires a finalized, inspected dataset."""
    factory = _callback(profile, "writer_factory")
    writer = None
    body_started = False

    def prepare():
        nonlocal writer
        writer = factory(request, profile)
        if not all(
            callable(getattr(writer, method, None))
            for method in ("add_frame", "finish", "abort")
        ):
            raise Rejected("dataset writer requires add_frame/finish/abort")

    def body(hp, stop_event):
        nonlocal body_started
        body_started = True
        last_seq = {}
        try:
            for index in range(profile["max_frames"]):
                if stop_event.is_set():
                    raise Rejected("recording stopped; dataset incomplete")
                sample = _capture(devices, hp, profile)
                active = sample["arms"][profile["arm_id"]]
                for arm_id, arm in sample["arms"].items():
                    seq = arm["observation_seq"]
                    if type(seq) is not int or seq <= last_seq.get(arm_id, -1):
                        raise Rejected("measured observation sequence did not advance")
                    last_seq[arm_id] = seq
                    if arm_id != profile["arm_id"] and any(
                        abs(x - y) > tolerance
                        for x, y, tolerance in zip(
                            arm["positions"],
                            hp["safe_positions"][arm_id],
                            hp["safe_tolerance"],
                            strict=True,
                        )
                    ):
                        raise Rejected("inactive arm left safe position")
                target = devices.read_action()
                values = _positions(target, hp)
                previous = tuple(active["positions"])
                if any(
                    abs(a - b) > step
                    for a, b, step in zip(values, previous, hp["max_step"], strict=True)
                ):
                    raise Rejected("leader action exceeds calibrated maximum step")
                if stop_event.is_set():
                    raise Rejected("recording stopped before action")
                actual = devices.send_action(target)
                _positions(actual, hp)
                writer.add_frame(
                    {
                        "index": index,
                        "sample": sample,
                        "requested_action": deepcopy(target),
                        "actual_action": deepcopy(actual),
                        "action_ns": time.time_ns(),
                    }
                )
                if stop_event.wait(1 / profile["fps"]):
                    raise Rejected("recording stopped; dataset incomplete")
            manifest = writer.finish()
            if not isinstance(manifest, dict) or manifest.get("complete") is not True:
                raise Rejected("dataset writer did not finalize a complete manifest")
            return {
                "state": "SUCCEEDED",
                "frames": profile["max_frames"],
                "dataset": manifest,
            }
        except Exception:
            writer.abort()
            raise

    result = _run(request, profile, devices, "record", body, prepare=prepare)
    if (
        writer is not None
        and not body_started
        and callable(getattr(writer, "abort", None))
    ):
        writer.abort()
    return result


def evaluate(request, profile, host, devices):
    """Supervised evaluation via the existing Host, never direct AI motor commands."""
    exchange = _callback(profile, "exchange")
    supervise = _callback(profile, "supervise")
    context = _callback(profile, "execution_context")
    expected = Profile(profile.get("host_profile")).data
    if host.profile != expected or host.state != "READY":
        raise Rejected("evaluation requires matching, inactive ready Host")

    def body(hp, stop_event):
        applied = []
        try:
            for _ in range(profile["max_frames"]):
                if stop_event.is_set():
                    raise Rejected("supervisor stop requested")
                sample = _capture(devices, hp, profile)
                if profile["device_mode"] == "hardware" and set(
                    sample["arms"][profile["arm_id"]].get("images", {})
                ) != set(profile["camera_mapping"].values()):
                    raise Rejected(
                        "complete current camera image manifest required for evaluation"
                    )
                for arm_id, arm in sample["arms"].items():
                    host.observe(arm_id, **arm)
                update = supervise(host, sample)
                if update.get("permit") is not None:
                    host.update_permit(update["permit"])
                if not host.command:
                    execution = context()
                    if (
                        execution["command"]["payload"]["assigned_arm_id"]
                        != profile["arm_id"]
                        or execution["command"]["payload"]["operation"]
                        != profile["operation"]
                    ):
                        raise Rejected(
                            "evaluation execution differs from trusted arm/operation"
                        )
                    host.update_permit(execution["permit"])
                    host.accept_command(execution["command"])
                if host.tick() in ("HOLD", "RECONCILING"):
                    raise Rejected(host.reason or "Host held execution")
                if not host.reset_acknowledged:
                    host.acknowledge_reset(exchange(host.reset_request()))
                if host.state in WAITS:
                    if update.get("evidence") is not None:
                        host.confirm(update["evidence"])
                elif host.state in PHASES:
                    if update.get("phase_complete") is True:
                        host.phase_boundary()
                    else:
                        if not host.queue:
                            candidate = exchange(host.infer_request())
                            host.validate_candidate(candidate)
                        action = host.next_action()
                        if action is not None:
                            if stop_event.is_set():
                                raise Rejected("supervisor stop before action")
                            actual = devices.send_action(
                                dict(zip(hp["feature_names"], action, strict=True))
                            )
                            _positions(actual, hp)
                            applied.append(
                                {
                                    "actual_action": deepcopy(actual),
                                    "action_ns": time.time_ns(),
                                    "observation_seq": sample["arms"][
                                        profile["arm_id"]
                                    ]["observation_seq"],
                                }
                            )
                if host.state == "SAFE_RETURNED":
                    return {
                        "state": "SUCCEEDED",
                        "host": host.status(),
                        "actions": applied,
                    }
                stop_event.wait(1 / profile["fps"])
            raise Rejected(
                "evaluation frame budget exhausted before verified safe return"
            )
        except Exception as exc:  # noqa: BLE001 -- Preserve all applied actions before physical recovery.
            host.hold(str(exc))
            return {
                "state": "FAILED",
                "error": str(exc),
                "host": host.status(),
                "actions": applied,
            }

    result = _run(request, profile, devices, "evaluate", body)
    if result["state"] != "SUCCEEDED":
        host.hold(result.get("error", "physical stop confirmation required"))
    return result


class LeRobotDevices:
    """Lazy v0.6.1 OMX leader/follower adapter; field safety hooks are mandatory."""

    def __init__(self):
        self.arms, self.leader = {}, None
        self.profile = None
        self.seq = 0

    def start(self, profile, operation):
        if (
            profile.get("device_mode") != "hardware"
            or profile.get("authorization", {}).get("hardware") is not True
            or profile.get("authorization", {}).get("supervisor") is not True
            or operation not in ("record", "evaluate")
        ):
            raise Rejected("LeRobot adapters require explicit hardware authorization")
        hp = Profile(profile.get("host_profile")).data
        if (
            hp.get("synthetic_fixture") is True
            or tuple(hp["feature_names"]) != OMX_FEATURES
        ):
            raise Rejected("field OMX feature/calibration profile required")
        self.profile = profile
        _callback(profile, "physical_stop")
        _callback(profile, "capture_metadata")
        verify = _callback(profile, "startup_verify")
        if operation == "record" and hp["use_degrees"]:
            raise Rejected(
                "pinned OMX leader emits range_m100_100; degree recording needs a verified conversion"
            )
        configs = profile.get("follower_configs", {})
        calibration = profile.get("calibration_files", {})
        if set(configs) != set(hp["safe_positions"]) or "leader_config" not in profile:
            raise Rejected("explicit configs for both followers and leader required")
        mapping = profile.get("camera_mapping", {})
        cameras = profile.get("dataset_profile", {}).get("cameras", {})
        active_cameras = configs[profile["arm_id"]].cameras
        if (
            not mapping
            or set(mapping) != set(cameras)
            or set(mapping.values()) != set(active_cameras)
        ):
            raise Rejected("explicit complete wire/dataset camera mapping required")
        for key, name in mapping.items():
            camera, expected = active_cameras[name], cameras[key]
            if (
                expected.get("shape") != [camera.height, camera.width, 3]
                or expected.get("fps") != camera.fps
            ):
                raise Rejected(
                    "camera HWC shape/FPS differs from dataset/model profile"
                )
        all_configs = {**configs, "leader": profile["leader_config"]}
        if set(calibration) != set(all_configs):
            raise Rejected("verified calibration files required for every device")
        ports = []
        for name, config in all_configs.items():
            entry = calibration[name]
            path = Path(entry["path"])
            if (
                entry.get("calibration_version") != hp["calibration_version"]
                or not path.is_absolute()
                or hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]
            ):
                raise Rejected("calibration identity mismatch")
            if str(config.calibration_dir / f"{config.id}.json") != str(path):
                raise Rejected("device config does not select verified calibration")
            ports.append(config.port)
            if name != "leader" and (
                config.use_degrees != hp["use_degrees"]
                or config.max_relative_target is None
            ):
                raise Rejected("follower units/relative safety limit mismatch")
            if name != "leader":
                for feature, maximum in zip(OMX_FEATURES, hp["max_step"], strict=True):
                    relative = config.max_relative_target
                    value = (
                        relative.get(feature.removesuffix(".pos"))
                        if isinstance(relative, dict)
                        else relative
                    )
                    if not finite(value) or not 0 < value <= maximum:
                        raise Rejected(
                            "driver relative limit exceeds validated Host step limit"
                        )
        if len(set(ports)) != len(ports):
            raise Rejected("each serial port requires one owner")
        # Imports and constructors happen only after every trusted gate above.
        from lerobot.robots.omx_follower.omx_follower import OmxFollower
        from lerobot.teleoperators.omx_leader.omx_leader import OmxLeader

        self.arms = {key: OmxFollower(config) for key, config in configs.items()}
        self.leader = OmxLeader(profile["leader_config"])
        for arm in self.arms.values():
            arm.connect(calibrate=False)
        if operation == "record":
            self.leader.connect(calibrate=False)
        if verify(self) is not True:
            raise Rejected("field startup verifier did not authorize execution")

    def capture(self):
        self.seq += 1
        arms, observations, windows = {}, {}, {}
        for name, arm in self.arms.items():
            started_ns = time.time_ns()
            observations[name] = arm.get_observation()
            windows[name] = {"started_ns": started_ns, "finished_ns": time.time_ns()}
            arms[name] = {
                "positions": [observations[name][key] for key in OMX_FEATURES],
                "observation_seq": self.seq,
                "captured_at": datetime.fromtimestamp(
                    started_ns / 1e9, UTC
                ).isoformat(),
            }
        observation = observations[self.profile["arm_id"]]
        transform = _callback(self.profile, "capture_metadata")
        # Camera capture timestamps/encoding must come from the verified camera adapter.
        return transform(
            {"arms": arms, "observation": observation, "acquisition_windows": windows}
        )

    def read_action(self):
        return self.leader.get_action()

    def send_action(self, action):
        return self.arms[self.profile["arm_id"]].send_action(action)

    def stop(self, reason):
        evidence = _callback(self.profile, "physical_stop")(self, reason)
        if isinstance(evidence, dict) and evidence.get("confirmed") is True:
            devices = [
                *self.arms.values(),
                *([self.leader] if self.leader is not None else []),
            ]
            released = all(
                getattr(getattr(device, "bus", None), "is_connected", True) is False
                and all(
                    camera.is_connected is False
                    for camera in getattr(device, "cameras", {}).values()
                )
                for device in devices
            )
            evidence = {**evidence, "ownership_released": released}
            if not released:
                evidence.update(confirmed=False, manual_recovery_required=True)
        return evidence


class LeRobotDatasetWriter:
    """Use pinned dataset APIs; final success includes metadata and video inspection."""

    def __init__(self, profile):
        self.profile = profile
        self.frames = []
        self.closed = False
        self.annotate = _callback(profile, "annotate_frame")
        self.manifest_factory = _callback(profile, "manifest_factory")
        self.dataset = None
        root = Path(profile["dataset_path"])
        if not root.is_absolute() or root.exists():
            raise Rejected("new explicit dataset output path required")
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.dataset = LeRobotDataset.create(
            repo_id=profile["repo_id"],
            root=root,
            fps=profile["fps"],
            features=profile["dataset_features"],
            robot_type="omx_follower",
            use_videos=True,
        )

    def add_frame(self, frame):
        import numpy as np

        names = self.profile["host_profile"]["feature_names"]
        sample = frame["sample"]
        observation = sample["observation"]
        payload = {
            "observation.state": np.asarray(
                [observation[key] for key in names], dtype=np.float32
            ),
            "action": np.asarray(
                [frame["actual_action"][key] for key in names], dtype=np.float32
            ),
            "task": self.profile["task"],
        }
        for dataset_key, camera in self.profile["camera_mapping"].items():
            payload[dataset_key] = observation[camera]
        self.dataset.add_frame(payload)
        self.frames.append(self.annotate(deepcopy(frame)))

    def finish(self):
        self.dataset.save_episode(parallel_encoding=False)
        self.dataset.finalize()
        self.closed = True
        manifest = self.manifest_factory(self.dataset, self.frames)
        root = Path(self.profile["dataset_path"])
        if not isinstance(manifest, dict) or manifest.get("complete") is not True:
            raise Rejected("complete recording manifest required")
        (root / "manifest.json").write_text(json.dumps(manifest, allow_nan=False))
        from .learning import inspect_dataset

        return inspect_dataset(root, self.profile["dataset_profile"])

    def abort(self):
        if self.dataset is not None and not self.closed:
            self.dataset.clear_episode_buffer(delete_images=False)
            self.dataset.finalize()
            self.closed = True
