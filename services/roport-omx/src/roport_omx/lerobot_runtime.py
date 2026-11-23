"""Supervised LeRobot recording/evaluation; imports never open hardware."""

import hashlib
import json
import os
import time
from copy import deepcopy
from datetime import UTC, datetime
from pathlib import Path
from threading import Event

from .act_support import (
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
from .local_act_profile import OMX_FEATURES, validated_host_profile
from .run_trace import RunTrace
from .start_alignment import StartAlignment

_RECOVERY_LOCKS = {}
# Max ACT overshoot past a normalized-limit joint bound that is clipped back to it.
EDGE_CLIP_MAX_OVERSHOOT = 10.0


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

    def mark_unresolved(self, request_id=None):
        # The marker names the run that left the devices unresolved; recovery must present the same run.
        marker = {"state": "RECONCILING"}
        if request_id is not None:
            marker["request_id"] = request_id
        self._write(json.dumps(marker, separators=(",", ":")))

    def recorded_request_id(self):
        """The run named in the unresolved marker; None for a marker without one (or no marker)."""
        self.file.seek(0)
        try:
            marker = json.loads(self.file.read() or "null")
        except ValueError:
            return None
        return marker.get("request_id") if isinstance(marker, dict) else None

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
    parameters = request["parameters"]
    if parameters and not (
        operation == "evaluate"
        and "act_checkpoint" in profile
        and profile.get("device_mode") == "hardware"
        and isinstance(profile.get("start_alignment"), dict)
        and set(parameters) == {"prepare"}
        and parameters["prepare"] is True
    ):
        raise Rejected("device job parameters are selected by trusted profile")
    hp = validated_host_profile(profile, operation)
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


def _edge_clipped(action, hp):
    """Clip small overshoots only where the bound is the unit's normalized limit.

    Limits come from the validated feature unit names (act_support.py), e.g.
    range_m100_100 -> (-100, 100); LeRobot clips motor writes to the same range.
    Everything else out of bounds (incl. non-finite) is left for _positions to reject.
    """
    clipped = list(action)
    if len(clipped) != len(hp["feature_names"]):
        raise Rejected("exact six action features required")
    for j, (unit, (lo, hi)) in enumerate(
        zip(hp["feature_units"], hp["joint_bounds"], strict=True)
    ):
        if not unit.startswith("range_") or not finite(clipped[j]):
            continue
        n_lo, n_hi = (float(x) for x in unit[6:].replace("m", "-").split("_"))
        if lo == n_lo and 0 < lo - clipped[j] <= EDGE_CLIP_MAX_OVERSHOOT:
            clipped[j] = lo
        elif hi == n_hi and 0 < clipped[j] - hi <= EDGE_CLIP_MAX_OVERSHOOT:
            clipped[j] = hi
    return clipped


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
    _check_observation_age(sample, hp, wall_clock)
    return sample


def _check_observation_age(sample, hp, wall_clock):
    for arm in sample["arms"].values():
        age = wall_clock() - timestamp(arm["captured_at"])
        if (
            not -hp["timing"]["clock_skew"]
            <= age
            <= hp["timing"]["observation_max_age"]
        ):
            raise Rejected("stale or future measured arm observation")


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


def _seed_evaluation_hold(bus):
    """Seed a stationary torque-off hold, allowing one raw encoder count of jitter."""
    off = dict.fromkeys(bus.motors, 0)
    if not off:
        raise Rejected("initial hold requires configured motors")

    def read(register):
        values = bus.sync_read(register, normalize=False)
        if (
            not isinstance(values, dict)
            or set(values) != set(off)
            or any(type(value) is not int for value in values.values())
        ):
            raise Rejected("initial hold requires complete raw encoder readings")
        return dict(values)

    if read("Torque_Enable") != off:
        raise Rejected("initial hold requires all active motor torque off")
    hold = read("Present_Position")
    bus.sync_write("Goal_Position", dict(hold), normalize=False)
    goal = read("Goal_Position")
    current = read("Present_Position")
    if read("Torque_Enable") != off:
        raise Rejected("motor torque changed during initial hold inspection")
    # Torque-off hardware readback follows the observed adjacent encoder counts.
    if any(
        max(hold[key], goal[key], current[key])
        - min(hold[key], goal[key], current[key])
        > 1
        for key in off
    ):
        raise Rejected(
            "initial hold position readback mismatch (raw encoder counts): "
            + json.dumps({"written": hold, "readback": goal, "current": current})
        )


def _write_profile_velocities(bus, values):
    bus.sync_write("Profile_Velocity", values, normalize=False)
    if bus.sync_read("Profile_Velocity", normalize=False) != values:
        raise Rejected("DYNAMIXEL profile velocity readback mismatch")


def _pinned_follower_bus(profile):
    """Unconnected bus for the working follower, after the pinned calibration file is verified."""
    arm_id = profile["arm_id"]
    config = profile["follower_configs"][arm_id]
    calibration = profile["calibration_files"][arm_id]
    path = Path(calibration["path"])
    if (
        path != config.calibration_dir / f"{config.id}.json"
        or hashlib.sha256(path.read_bytes()).hexdigest() != calibration["sha256"]
    ):
        raise Rejected("calibration changed before alignment recovery")
    from lerobot.robots.omx_follower.omx_follower import OmxFollower

    return OmxFollower(config).bus


def _release_follower_torque(profile):
    """Operator-confirmed release: torque off, then clear the tripped Bus Watchdog.

    The panel refuses the next start while a Bus Watchdog value is outside 0..127, and the
    X430 motors reject the profile-velocity restore until it is cleared. Fails closed: a
    readback that is not all zero raises and the recovery lock stays held.
    """
    bus = _pinned_follower_bus(profile)
    try:
        bus.connect()
        off = dict.fromkeys(bus.motors, 0)
        for register, message in (
            ("Torque_Enable", "motor torque did not turn off"),
            ("Bus_Watchdog", "bus watchdog did not clear"),
        ):
            bus.sync_write(register, off, normalize=False)
            if bus.sync_read(register, normalize=False) != off:
                raise Rejected(f"{message}; keep supporting the arm")
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def _recover_alignment_velocity(profile, saved):
    if (
        not isinstance(saved, dict)
        or set(saved) != {feature.removesuffix(".pos") for feature in OMX_FEATURES}
        or any(
            type(value) is not int or not 0 <= value <= 32767
            for value in saved.values()
        )
    ):
        raise Rejected("original alignment motor velocities could not be verified")
    bus = _pinned_follower_bus(profile)
    try:
        bus.connect()
        off = dict.fromkeys(bus.motors, 0)
        if (
            not bus.is_calibrated
            or bus.sync_read("Drive_Mode", normalize=False) != off
            or bus.sync_read("Torque_Enable", normalize=False) != off
        ):
            raise Rejected(
                "support the aligned arm and verify all its motor torque is off before recovery"
            )
        _write_profile_velocities(bus, saved)
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def reconcile_evaluation(profile, job, confirmation):
    """Release an evaluation lock after onsite confirmation and adapter cleanup."""
    stop = ((job.get("result") or {}).get("physical_stop") or {}).get("evidence") or {}
    if (
        not isinstance(confirmation, dict)
        or confirmation.get("stopped_and_supported") is not True
        or stop.get("ownership_released") is not True
    ):
        raise Rejected("onsite stop and released evaluation devices must be confirmed")
    if (
        "release_torque" in confirmation
        and type(confirmation["release_torque"]) is not bool
    ):
        raise Rejected("release_torque must be true or false")
    path = str(Path(profile["shared_lock_path"]).resolve())
    held = _RECOVERY_LOCKS.get(path)
    lock = held or DeviceLock(path).acquire(recovery=True)
    recorded = lock.recorded_request_id()
    if recorded is not None and (job.get("result") or {}).get("request_id") != recorded:
        if held is None:
            lock.release()  # the marker stays: the run it names is still unresolved
        raise Rejected("recovery evidence belongs to a different evaluation")
    _RECOVERY_LOCKS[path] = lock
    stop_evidence = stop.get("evidence") or {}
    restored_velocity = stop_evidence.get("alignment_velocity_restore_pending") is True
    released = confirmation.get("release_torque") is True
    if released:
        _release_follower_torque(profile)
    if restored_velocity:
        _recover_alignment_velocity(
            profile, stop_evidence.get("alignment_original_profile_velocity")
        )
    lock.confirm_safe()
    lock.release()
    _RECOVERY_LOCKS.pop(path, None)
    profile.pop("_recovery_lock", None)
    return {
        "confirmed": True,
        "evidence": {
            "stopped_and_supported": True,
            "ownership_released": True,
            **({"alignment_velocity_restored": True} if restored_velocity else {}),
            **({"torque_released": True} if released else {}),
        },
    }


def _run(request, profile, devices, operation, body, *, prepare=None):
    hp = _validated(request, profile, operation)
    lock = DeviceLock(profile["shared_lock_path"]).acquire()
    event = profile.get("stop_event")
    if event is None:
        event = Event()
    started, result = False, {}
    try:
        _check_before_start(event)
        if prepare is not None:
            prepare()
            _check_before_start(event)
        lock.mark_unresolved(request["request_id"])
        started = True  # A partially failed start still needs verified physical stop.
        devices.start(profile, operation)
        result = body(hp, event)
    except Exception as exc:  # noqa: BLE001 -- All adapter failures must enter physical recovery.
        result = {"state": "FAILED", "error": str(exc)}
    finally:
        if started and getattr(event, "reason", None):
            result.update(state="FAILED", error=event.reason)
        if started:
            result["request_id"] = request["request_id"]
            stop = _physical_stop(profile, devices, result.get("error", "job finished"))
            result["physical_stop"] = stop
            if not stop["confirmed"]:
                result["state"] = "RECONCILING"
                profile["_recovery_lock"] = lock
                _RECOVERY_LOCKS[str(lock.path.resolve())] = lock
            else:
                lock.confirm_safe()
                lock.release()
            if isinstance(devices, LeRobotDevices) and operation == "evaluate":
                result["frames"] = devices.applied_actions
                result["alignment_frames"] = devices.applied_alignment_actions
                if devices.alignment_fault is not None:
                    result["alignment_fault"] = devices.alignment_fault
        else:
            lock.release()
    return result


def _check_before_start(event):
    if event.is_set():
        raise Rejected(
            getattr(event, "reason", None) or "stopped before device startup"
        )


def record(request, profile, devices):
    """Record applied actions; success requires a finalized, inspected dataset.

    Known limit (review of the migration): writer.finish() and writer.abort() run before the physical
    stop in _run(), so a stalled writer delays the stop. The host executors do not call record().
    """
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
                # read_action() may have been slow: the measured sample must still be current.
                _check_observation_age(sample, hp, profile.get("wall_clock", time.time))
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


def _align_local_act(profile, devices, plan, capture, stop_event):
    arm_id = profile["arm_id"]
    period = 1 / profile["fps"]
    max_gap = min(
        profile["bus_watchdog_ticks"] * 0.01,
        profile["host_profile"]["timing"]["observation_max_age"],
    )
    previous_cycle = time.monotonic()
    plan.begin(capture()["arms"][arm_id]["positions"], previous_cycle)
    devices.begin_alignment()

    def measured_cycle():
        nonlocal previous_cycle
        sample = capture()
        devices.check_alignment_health()
        now = time.monotonic()
        if not 0 <= now - previous_cycle < max_gap:
            raise Rejected("start alignment measurement missed its deadline")
        previous_cycle = now
        _check_before_start(stop_event)
        return sample, now

    while not plan.done:
        sample, now = measured_cycle()
        try:
            values = plan.step(sample["arms"][arm_id]["positions"], now)
        except Rejected:
            if isinstance(devices, LeRobotDevices):
                devices.capture_alignment_fault()
            raise
        if values is not None:
            devices.send_alignment(dict(zip(OMX_FEATURES, values, strict=True)))
        stop_event.wait(max(0, period - (time.monotonic() - now)))

    with stop_event.phase_guard:
        _check_before_start(stop_event)
        stop_event.alignment_phase = "WAITING_MODEL_START"
    # The operator's time to press start is its own setting; the alignment timeout only bounds settling.
    wait_s = profile.get("model_start_wait_s", plan.timeout_s)
    wait_deadline = time.monotonic() + wait_s
    report_start = _callback(profile, "alignment_start_report")
    while True:
        sample, now = measured_cycle()
        if now >= wait_deadline:
            raise Rejected(
                f"aligned arm timed out waiting for explicit model start after {wait_s:g} s"
            )
        plan.check_hold(sample["arms"][arm_id]["positions"])
        with stop_event.phase_guard:
            _check_before_start(stop_event)
            if stop_event.continue_model.is_set():
                break
        stop_event.wait(max(0, period - (time.monotonic() - now)))

    report = report_start(
        {name: arm["positions"] for name, arm in sample["arms"].items()}
    )
    if not report["ready"]:
        raise Rejected("aligned pose failed the demonstration start check")
    devices.finish_alignment()
    with stop_event.phase_guard:
        _check_before_start(stop_event)
        stop_event.alignment_phase = "MODEL_RUNNING"


def evaluate_local_act(request, profile, devices):
    """Run a full-episode policy under a deployment-provided field supervisor."""
    from .panel_act import PanelAct

    supervise = _callback(profile, "supervise_local_act")
    guard = None
    if profile.get("device_mode") == "hardware" or "demonstration_guard" in profile:
        from .demonstration_guard import DemonstrationGuard

        guard = DemonstrationGuard(profile.get("demonstration_guard"))
    level = profile.get("shelf_level")
    if type(level) is not int or level not in (1, 2):
        raise Rejected("trusted evaluation profile requires shelf level 1 or 2")
    checkpoint = Path(profile["act_checkpoint"])
    if not checkpoint.is_absolute():
        raise Rejected("absolute local ACT checkpoint required")
    policy = None
    plan = None
    parameters = request.get("parameters") if isinstance(request, dict) else None
    if isinstance(parameters, dict) and parameters.get("prepare") is True:
        _validated(request, profile, "evaluate")
        if guard is None:
            raise Rejected("alignment requires demonstration start limits")
        if "model_start_wait_s" in profile:
            wait = profile["model_start_wait_s"]
            if type(wait) is bool or not finite(wait) or not 10 <= wait <= 900:
                raise Rejected("model start wait must be 10 to 900 seconds")
        plan = StartAlignment(
            profile["start_alignment"], target_tolerance=guard.state_tolerance
        )
        if plan.target != guard.frames[0][0] or any(
            tolerance > limit
            for tolerance, limit in zip(
                plan.start_tolerance, guard.state_tolerance, strict=True
            )
        ):
            raise Rejected(
                "alignment target and tolerance must match demonstration start"
            )
        _callback(profile, "alignment_start_report")
        event = profile.get("stop_event")
        if not all(
            getattr(event, name, None) is not None
            for name in ("phase_guard", "continue_model")
        ):
            raise Rejected("start alignment requires panel supervision")

    def prepare():
        nonlocal policy
        policy = profile.get("act_policy") or PanelAct(checkpoint)
        policy.reset()

    def body(hp, stop_event):
        names = hp["feature_names"]
        wall_clock = profile.get("wall_clock", time.time)
        previous_seq = {}

        def capture():
            _check_before_start(stop_event)
            sample = _capture(devices, hp, profile)
            for arm_id, measured in sample["arms"].items():
                if arm_id != profile["arm_id"] and any(
                    abs(position - safe) > tolerance
                    for position, safe, tolerance in zip(
                        measured["positions"],
                        hp["safe_positions"][arm_id],
                        hp["safe_tolerance"],
                        strict=True,
                    )
                ):
                    raise Rejected("inactive arm outside calibrated safe pose")
                seq = measured["observation_seq"]
                if type(seq) is not int or seq <= previous_seq.get(arm_id, -1):
                    raise Rejected("fresh increasing observation required")
                previous_seq[arm_id] = seq
            return sample

        if plan is not None:
            _align_local_act(profile, devices, plan, capture, stop_event)
        trace = RunTrace.open(
            profile.get("trace_dir")
        )  # diagnostics only; None when off
        try:
            return drive(hp, stop_event, names, wall_clock, capture, trace)
        finally:
            if trace is not None:
                trace.close()

    def drive(hp, stop_event, names, wall_clock, capture, trace):
        deadline = time.monotonic() + profile["max_frames"] / profile["fps"]
        for frame in range(profile["max_frames"]):
            cycle_started = time.monotonic()
            if cycle_started >= deadline:
                raise Rejected("evaluation duration reached; confirm outcome on site")
            sample = capture()
            arm = sample["arms"][profile["arm_id"]]
            decision = supervise(sample)
            if decision == "complete":
                if frame == 0:
                    raise Rejected("completion requires a started evaluation")
                return {"state": "SUCCEEDED", "frames": frame}
            if decision != "run":
                raise Rejected("field supervisor did not authorize motion")
            observation = {
                name: sample["observation"][camera]
                for name, camera in profile["camera_mapping"].items()
            }
            observation["observation.state"] = arm["positions"]
            started = time.monotonic()
            action = _edge_clipped(
                policy.select_action(observation, level).tolist(), hp
            )
            inference_s = time.monotonic() - started
            command = dict(zip(names, action, strict=True))
            _positions(command, hp)
            if any(
                abs(target - measured) > limit
                for target, measured, limit in zip(
                    action, arm["positions"], hp["max_step"], strict=True
                )
            ):
                raise Rejected("ACT target exceeds calibrated step limit")
            if guard is not None:
                guard.check(
                    arm["positions"],
                    action,
                    advance=not isinstance(devices, LeRobotDevices),
                )
            if time.monotonic() >= deadline:
                # The last inference may finish inside its own limit but after the whole evaluation's.
                raise Rejected(
                    "evaluation duration reached before action; confirm outcome on site"
                )
            if (
                stop_event.is_set()
                or time.monotonic() - started > hp["timing"]["inference_timeout"]
                or wall_clock() - timestamp(arm["captured_at"])
                > hp["timing"]["observation_max_age"]
            ):
                raise Rejected("evaluation stopped or prediction expired before action")
            _positions(devices.send_action(command), hp)
            if trace is not None:
                # After every safety check and the motor write, so tracing cannot change their timing.
                trace.frame(frame, arm["positions"], action, inference_s, observation)
            stop_event.wait(
                max(0, 1 / profile["fps"] - (time.monotonic() - cycle_started))
            )
        raise Rejected("evaluation frame limit reached before verified completion")

    if plan is not None:
        with event.phase_guard:
            _check_before_start(event)
            event.alignment_phase = "ALIGNING"
        profile["_start_alignment"] = plan
    if guard is not None:
        profile["_demonstration_guard"] = guard
    try:
        return _run(request, profile, devices, "evaluate", body, prepare=prepare)
    finally:
        profile.pop("_start_alignment", None)
        profile.pop("_demonstration_guard", None)


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
        self.watchdog_ticks = None
        self.reset_run_state()

    def reset_run_state(self):
        """Per-run state: a reused object must not carry the previous run's stop evidence."""
        self.evaluation_armed = False
        self.startup_torque_off_verified = False
        self.applied_actions = 0
        self.applied_alignment_actions = 0
        self.alignment_fault = None
        self._alignment_velocity = None

    def _verify_watchdog(self, arm):
        bus = arm.bus
        # All pinned OMX motor models use the same one-byte register.
        values = bus.sync_read("Bus_Watchdog", normalize=False)
        if set(values) != set(bus.motors) or any(
            value != self.watchdog_ticks for value in values.values()
        ):
            raise Rejected("DYNAMIXEL Bus Watchdog tripped or readback mismatch")

    def start(self, profile, operation):
        if (
            profile.get("device_mode") != "hardware"
            or profile.get("authorization", {}).get("hardware") is not True
            or profile.get("authorization", {}).get("supervisor") is not True
            or operation not in ("record", "evaluate")
        ):
            raise Rejected("LeRobot adapters require explicit hardware authorization")
        hp = validated_host_profile(profile, operation)
        if (
            hp.get("synthetic_fixture") is True
            or tuple(hp["feature_names"]) != OMX_FEATURES
        ):
            raise Rejected("field OMX feature/calibration profile required")
        self.profile = profile
        if operation == "record" or "physical_stop" in profile:
            _callback(profile, "physical_stop")
        _callback(profile, "capture_metadata")
        verify = _callback(profile, "startup_verify")
        if operation == "evaluate":
            ticks = profile.get("bus_watchdog_ticks")
            if type(ticks) is not int or not 1 <= ticks <= 127:
                raise Rejected(
                    "hardware evaluation requires bus_watchdog_ticks (1–127, 20 ms per tick)"
                )
            self.watchdog_ticks = ticks
        else:
            self.watchdog_ticks = None
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
        self.reset_run_state()
        event = profile.get("stop_event")
        for arm in self.arms.values():
            if event is not None:
                _check_before_start(event)
            if operation == "record":
                arm.connect(calibrate=False)
                continue
            # Native follower connect configures the bus and enables torque.
            bus = arm.bus
            bus.connect()
            torque = bus.sync_read("Torque_Enable", normalize=False)
            if set(torque) != set(bus.motors) or any(torque.values()):
                raise Rejected(
                    "evaluation startup requires torque off on all followers"
                )
            if not bus.is_calibrated:
                raise Rejected("follower calibration differs from the verified file")
            # The pinned follower uses extended position (4) and gripper current position (5).
            modes = bus.sync_read("Operating_Mode", normalize=False)
            if modes != {motor: 5 if motor == "gripper" else 4 for motor in bus.motors}:
                raise Rejected(
                    "evaluation requires the commissioned follower operating modes"
                )
            for camera in arm.cameras.values():
                camera.connect()
            current = bus.sync_read("Bus_Watchdog", normalize=False)
            if set(current) != set(bus.motors) or any(
                not 0 <= value <= 127 for value in current.values()
            ):
                raise Rejected("DYNAMIXEL Bus Watchdog requires supervised recovery")
        if operation == "evaluate":
            self.startup_torque_off_verified = True
        if operation == "record":
            self.leader.connect(calibrate=False)
        if verify(self) is not True:
            raise Rejected("field startup verifier did not authorize execution")

    def capture(self):
        self.seq += 1
        arms, observations, windows = {}, {}, {}
        self.camera_times = []
        for name, arm in self.arms.items():
            started_ns = time.time_ns()
            if self.watchdog_ticks is None:
                observations[name] = arm.get_observation()
            else:
                observations[name] = {
                    f"{motor}.pos": value
                    for motor, value in arm.bus.sync_read("Present_Position").items()
                }
                for key, camera in arm.cameras.items():
                    with camera.frame_lock:
                        frame, captured = camera.latest_frame, camera.latest_timestamp
                    if frame is None or captured is None:
                        raise Rejected("current camera frame unavailable")
                    observations[name][key] = frame
                    self.camera_times.append(captured)
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

    def _check_evaluation_cameras(self):
        age = self.profile["host_profile"]["timing"]["observation_max_age"]
        if any(
            not 0 <= time.perf_counter() - stamp <= age
            for stamp in getattr(self, "camera_times", [])
        ):
            raise Rejected("camera frame expired before motor write")

    def begin_alignment(self):
        velocity = self.profile.get("start_alignment_motor_velocity")
        if type(velocity) is not int or not 1 <= velocity <= 32767:
            raise Rejected("explicit positive DYNAMIXEL alignment velocity required")
        event = self.profile["stop_event"]
        bus = self.arms[self.profile["arm_id"]].bus
        _check_before_start(event)
        modes = bus.sync_read("Drive_Mode", normalize=False)
        if modes != dict.fromkeys(bus.motors, 0):
            raise Rejected("alignment requires DYNAMIXEL velocity-based Drive Mode 0")
        previous = bus.sync_read("Profile_Velocity", normalize=False)
        if set(previous) != set(bus.motors) or any(
            type(value) is not int or not 0 <= value <= 32767
            for value in previous.values()
        ):
            raise Rejected("could not verify prior DYNAMIXEL profile velocities")
        if any(0 < value < velocity for value in previous.values()):
            raise Rejected(
                "alignment cannot increase the existing motor velocity limit"
            )
        limited = dict.fromkeys(bus.motors, velocity)
        _check_before_start(event)
        self._alignment_velocity = previous
        # The gripper's current-based position mode still relies on bounded joint steps.
        _write_profile_velocities(bus, limited)

    def capture_alignment_fault(self):
        """Bounded read-only evidence before the stop hold replaces the last goal."""
        self.alignment_fault = {"raw_registers": {}}
        bus = self.arms[self.profile["arm_id"]].bus
        try:
            for register in (
                "Goal_Position",
                "Present_Position",
                "Position_Trajectory",
                "Present_PWM",
            ):
                self.alignment_fault["raw_registers"][register] = bus.sync_read(
                    register, normalize=False
                )
        except Exception as exc:  # noqa: BLE001 -- Diagnostics must never prevent the stop path.
            self.alignment_fault["read_error"] = str(exc)

    def check_alignment_health(self):
        self._check_evaluation_cameras()
        for name, arm in self.arms.items():
            if self.evaluation_armed:
                self._verify_watchdog(arm)
            expected = int(self.evaluation_armed and name == self.profile["arm_id"])
            if arm.bus.sync_read("Torque_Enable", normalize=False) != dict.fromkeys(
                arm.bus.motors, expected
            ):
                raise Rejected("follower torque changed during alignment")

    def _restore_alignment_velocity(self, bus):
        if self._alignment_velocity is None:
            return
        _write_profile_velocities(bus, self._alignment_velocity)
        self._alignment_velocity = None

    def finish_alignment(self):
        plan = self.profile["_start_alignment"]
        event = self.profile["stop_event"]
        bus = self.arms[self.profile["arm_id"]].bus
        self.check_alignment_health()
        plan.check_hold(self._current_joint_positions(bus))
        hold = bus.sync_read("Present_Position", normalize=False)
        _check_before_start(event)
        bus.sync_write("Goal_Position", hold, normalize=False)
        if bus.sync_read("Goal_Position", normalize=False) != hold:
            raise Rejected("alignment hold position readback mismatch")
        _check_before_start(event)
        self._restore_alignment_velocity(bus)

    @staticmethod
    def _current_joint_positions(bus):
        current = bus.sync_read("Present_Position")
        return [current[key.removesuffix(".pos")] for key in OMX_FEATURES]

    def send_alignment(self, action):
        plan = self.profile["_start_alignment"]
        if self._alignment_velocity is None:
            raise Rejected("alignment velocity has not been applied and verified")
        try:
            return self._send_action(action, alignment=plan)
        except Rejected:
            # Read-only register evidence, same as a failed planning step; the stop path follows.
            self.capture_alignment_fault()
            raise

    def send_action(self, action):
        return self._send_action(action)

    def _send_action(self, action, *, alignment=None):
        arm = self.arms[self.profile["arm_id"]]
        if self.watchdog_ticks is not None and self.evaluation_armed:
            for follower in self.arms.values():
                self._verify_watchdog(follower)
        event = self.profile.get("stop_event")
        if event is not None and event.is_set():
            raise Rejected("supervisor stop before follower action")
        if self.watchdog_ticks is not None:
            hp = self.profile["host_profile"]
            values = _positions(action, hp)
            measured = self._current_joint_positions(arm.bus)
            if not inside(vector(measured), hp["joint_bounds"]):
                raise Rejected("measured arm outside joint bounds before motor write")
            if any(
                abs(a - b) > limit
                for a, b, limit in zip(values, measured, hp["max_step"], strict=True)
            ):
                raise Rejected("ACT target exceeds step limit at motor write")
            guard = self.profile.get("_demonstration_guard")
            if alignment is not None:
                alignment.check_write(measured, values)
            elif guard is not None:
                guard.check(measured, values)
            self._check_evaluation_cameras()
        first_write = False
        if self.watchdog_ticks is not None and not self.evaluation_armed:
            first_write = True
            for follower in self.arms.values():
                for motor in follower.bus.motors:
                    follower.bus.write(
                        "Bus_Watchdog", motor, self.watchdog_ticks, normalize=False
                    )
                self._verify_watchdog(follower)
            bus = arm.bus
            _seed_evaluation_hold(bus)
            if event is not None and event.is_set():
                raise Rejected("supervisor stop before enabling torque")
            self.evaluation_armed = True
            bus.enable_torque()
            self._verify_watchdog(arm)
            if event is not None and event.is_set():
                raise Rejected("supervisor stop before follower action")
        if self.watchdog_ticks is None:
            applied = arm.send_action(action)
        else:
            # Send the checked target unchanged; native send_action silently clamps it.
            if alignment is not None or first_write:
                # Torque was off until just now (first write) or the plan is time-bound
                # (alignment): the arm may have moved since the checks above.
                measured = self._current_joint_positions(arm.bus)
                if not inside(vector(measured), hp["joint_bounds"]):
                    raise Rejected(
                        "measured arm outside joint bounds before motor write"
                    )
                if alignment is not None:
                    alignment.check_write(measured, values)
                else:
                    if any(
                        abs(a - b) > limit
                        for a, b, limit in zip(
                            values, measured, hp["max_step"], strict=True
                        )
                    ):
                        raise Rejected("ACT target exceeds step limit at motor write")
                    if guard is not None:
                        guard.check(measured, values)
            if event is not None and event.is_set():
                raise Rejected("supervisor stop before motor write")
            self._check_evaluation_cameras()
            arm.bus.sync_write(
                "Goal_Position",
                {key.removesuffix(".pos"): value for key, value in action.items()},
            )
            applied = action
        if alignment is None:
            self.applied_actions += 1
        else:
            self.applied_alignment_actions += 1
        return applied

    def stop(self, reason):
        if self.watchdog_ticks is not None and "physical_stop" not in self.profile:
            return self._stop_evaluation()
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

    def _stop_evaluation(self):
        """Request a hold and release I/O. Field supervision confirms the actual stop."""
        hold_requested = False
        errors = []
        unarmed_off = not self.evaluation_armed and getattr(
            self, "startup_torque_off_verified", False
        )
        if unarmed_off:
            try:
                for arm in self.arms.values():
                    torque = arm.bus.sync_read("Torque_Enable", normalize=False)
                    if set(torque) != set(arm.bus.motors) or any(torque.values()):
                        unarmed_off = False
            except Exception:  # noqa: BLE001 -- Unknown torque keeps physical recovery pending.
                unarmed_off = False
        if unarmed_off and self._alignment_velocity is not None:
            try:
                self._restore_alignment_velocity(self.arms[self.profile["arm_id"]].bus)
            except Exception as exc:  # noqa: BLE001 -- Failed restore requires reconciliation.
                errors.append(f"alignment velocity restore failed: {exc}")
        if self.evaluation_armed:
            arm = self.arms[self.profile["arm_id"]]
            try:
                self._verify_watchdog(arm)
                bus = arm.bus
                hold = bus.sync_read("Present_Position", normalize=False)
                bus.sync_write("Goal_Position", hold, normalize=False)
                hold_requested = bus.sync_read("Goal_Position", normalize=False) == hold
                if not hold_requested:
                    errors.append("hold position readback mismatch")
            except Exception as exc:  # noqa: BLE001 -- Still release I/O if the bus fails.
                errors.append(f"hold request failed: {exc}")

        for arm in self.arms.values():
            try:
                if arm.bus.is_connected:
                    arm.bus.disconnect(disable_torque=False)
            except Exception as exc:  # noqa: BLE001 -- Close remaining I/O despite a bus failure.
                errors.append(f"bus release failed: {exc}")
            for camera in arm.cameras.values():
                try:
                    if camera.is_connected:
                        camera.disconnect()
                except Exception as exc:  # noqa: BLE001 -- Continue closing the other devices.
                    errors.append(f"camera release failed: {exc}")
        released = all(
            not arm.bus.is_connected
            and all(not camera.is_connected for camera in arm.cameras.values())
            for arm in self.arms.values()
        )
        clean_unarmed = unarmed_off and not errors and released
        return {
            "confirmed": clean_unarmed,
            "supported": clean_unarmed,
            "manual_recovery_required": not clean_unarmed,
            "ownership_released": released,
            "evidence": {
                "hold_requested": hold_requested,
                "never_armed_torque_off": unarmed_off,
                "alignment_velocity_restore_pending": self._alignment_velocity
                is not None,
                "alignment_original_profile_velocity": self._alignment_velocity,
                "errors": errors,
            },
        }


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
        if self.profile.get("camera_devices"):
            # Which device produced each image stream, so later training can be checked against it.
            manifest = dict(
                manifest, camera_devices=dict(self.profile["camera_devices"])
            )
        (root / "manifest.json").write_text(json.dumps(manifest, allow_nan=False))
        return manifest

    def abort(self):
        if self.dataset is not None and not self.closed:
            self.dataset.clear_episode_buffer(delete_images=False)
            self.dataset.finalize()
            self.closed = True
