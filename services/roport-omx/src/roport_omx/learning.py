"""Local dataset validation and ACT jobs. Heavy LeRobot imports stay lazy."""

import hashlib
import json
import math
import os
import re
import subprocess
from copy import deepcopy
from pathlib import Path
from uuid import uuid4

PHASES = ("GRASPING", "TRANSFERRING", "RELEASING", "RETURNING")
SPLITS = ("train", "validation", "evaluation")
PARAMETERS = {
    "dataset_id",
    "output_id",
    "phase",
    "steps",
    "batch_size",
    "seed",
    "chunk_size",
    "n_action_steps",
}
PROFILE_FIELDS = (
    "operation",
    "arm_id",
    "feature_names",
    "feature_units",
    "use_degrees",
    "feature_schema_version",
    "calibration_version",
    "fps",
    "cameras",
    "timestamp_tolerance_s",
)


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _json(path):
    try:
        result = json.loads(Path(path).read_text())
        if not isinstance(result, dict):
            raise TypeError("expected a JSON object")
        return result
    except (OSError, ValueError, TypeError) as exc:
        raise ValueError(f"invalid or missing JSON: {path}") from exc


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def _inside(root, relative):
    root = Path(root).resolve()
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("path must be relative to its configured root")
    path = (root / relative).resolve()
    if path == root or not path.is_relative_to(root) or ".." in Path(relative).parts:
        raise ValueError("path escapes its configured root")
    return path


def _file(root, relative, kind):
    path = _inside(root, relative)
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"missing or empty {kind}: {relative}")
    return path


def _profile(profile):
    if not isinstance(profile, dict) or any(
        key not in profile for key in PROFILE_FIELDS
    ):
        raise ValueError("incomplete dataset profile")
    if (profile["operation"], profile["arm_id"]) not in (
        ("INBOUND", "omx1"),
        ("OUTBOUND", "omx2"),
    ):
        raise ValueError("operation/arm must be INBOUND omx1 or OUTBOUND omx2")
    names = profile["feature_names"]
    if (
        not isinstance(names, list)
        or len(names) != 6
        or any(not isinstance(n, str) or not n for n in names)
        or len(set(names)) != 6
    ):
        raise ValueError("expected six ordered feature names")
    if type(profile["use_degrees"]) is not bool:
        raise ValueError("use_degrees must be explicit")
    units = ["degrees" if profile["use_degrees"] else "range_m100_100"] * 5 + [
        "range_0_100"
    ]
    if profile["feature_units"] != units:
        raise ValueError("feature units disagree with OMX normalization")
    for key in ("feature_schema_version", "calibration_version"):
        _text(profile[key], key)
    _integer(profile["fps"], "fps", 1)
    tolerance = profile["timestamp_tolerance_s"]
    if not _finite(tolerance) or not 0 <= tolerance < 0.5 / profile["fps"]:
        raise ValueError(
            "timestamp tolerance must be explicit and less than half a frame"
        )
    cameras = profile["cameras"]
    if not isinstance(cameras, dict) or not cameras:
        raise ValueError("ACT requires explicit RGB cameras")
    for key, camera in cameras.items():
        if (
            not isinstance(key, str)
            or not key.startswith("observation.images.")
            or not re.fullmatch(
                r"[A-Za-z0-9_-]{1,16}", key.removeprefix("observation.images.")
            )
        ):
            raise ValueError("camera must name a LeRobot RGB feature")
        if not isinstance(camera, dict):
            raise TypeError("camera specification must be an object")
        shape = camera.get("shape")
        if (
            not isinstance(shape, list)
            or len(shape) != 3
            or shape[-1] != 3
            or any(type(v) is not int or v <= 0 for v in shape)
        ):
            raise ValueError("camera shape must be RGB HWC")
        if camera.get("fps") != profile["fps"]:
            raise ValueError("camera FPS must match observation/action FPS")
    if len({tuple(c["shape"]) for c in cameras.values()}) != 1:
        raise ValueError("vendored ACT requires equal camera shapes")


def _timestamps(episode, profile):
    stamps = episode.get("frame_timestamps")
    if not isinstance(stamps, list) or len(stamps) != episode["num_frames"]:
        raise ValueError("one synchronized timestamp record is required per frame")
    previous = None
    tolerance_ns = profile["timestamp_tolerance_s"] * 1e9
    period_ns = 1e9 / profile["fps"]
    for record in stamps:
        if not isinstance(record, dict):
            raise TypeError("timestamp record must be an object")
        cameras = record.get("cameras_ns")
        if not isinstance(cameras, dict) or cameras.keys() != profile["cameras"].keys():
            raise ValueError("synchronized camera timestamps are missing")
        values = [
            record.get("observation_ns"),
            record.get("action_ns"),
            *(cameras[key] for key in profile["cameras"]),
        ]
        for value in values:
            _integer(value, "capture timestamp")
        if max(values) - min(values) > tolerance_ns:
            raise ValueError("camera/state/action synchronization exceeds tolerance")
        if previous is not None and any(
            now <= old or abs(now - old - period_ns) > tolerance_ns
            for now, old in zip(values, previous)
        ):
            raise ValueError("timestamp order or FPS synchronization mismatch")
        previous = values


def inspect_dataset(root: Path, expected_profile: dict) -> dict:
    """Validate a completed local manifest and referenced files, without decoding video."""
    root = Path(root).resolve()
    _profile(expected_profile)
    manifest = _json(root / "manifest.json")
    if (
        manifest.get("schema_version") != "omx.dataset/1"
        or manifest.get("complete") is not True
    ):
        raise ValueError("dataset manifest is incomplete or unsupported")
    for key in ("repo_id", "dataset_version"):
        _text(manifest.get(key), key)
    profile = manifest.get("profile")
    _profile(profile)
    if any(profile[key] != expected_profile[key] for key in PROFILE_FIELDS):
        raise ValueError(
            "dataset profile, feature order, units or calibration mismatch"
        )
    info = _json(root / "meta/info.json")
    if info.get("codebase_version") != "v3.0" or info.get("fps") != profile["fps"]:
        raise ValueError("LeRobot dataset version/FPS mismatch")
    _file(root, "meta/stats.json", "dataset statistics")
    for directory in ("data", "meta/episodes"):
        files = list((root / directory).rglob("*.parquet"))
        if not files:
            raise ValueError(f"incomplete dataset: no {directory} parquet files")
        for path in files:
            _file(root, str(path.relative_to(root)), "dataset parquet")
    features = info.get("features", {})
    for key in ("observation.state", "action"):
        feature = features.get(key, {})
        if (
            feature.get("shape") != [6]
            or feature.get("names") != profile["feature_names"]
            or feature.get("dtype") != "float32"
        ):
            raise ValueError("LeRobot six-feature schema/order mismatch")
    for key, camera in profile["cameras"].items():
        feature = features.get(key, {})
        if feature.get("shape") != camera["shape"] or feature.get("dtype") not in (
            "video",
            "image",
        ):
            raise ValueError("LeRobot camera schema mismatch")
    episodes = manifest.get("episodes")
    if not isinstance(episodes, list) or not episodes:
        raise ValueError("no episodes in dataset")
    groups = {
        key: {} for key in ("source_episode_id", "session_id", "condition_group_id")
    }
    indices = set()
    for episode in episodes:
        if not isinstance(episode, dict):
            raise TypeError("episode must be an object")
        index = _integer(episode.get("episode_index"), "episode index")
        if index in indices:
            raise ValueError("duplicate episode index")
        indices.add(index)
        length = _integer(episode.get("num_frames"), "num_frames", 1)
        if episode.get("complete") is not True:
            raise ValueError("incomplete recording episode")
        if (
            episode.get("operation") != profile["operation"]
            or episode.get("arm_id") != profile["arm_id"]
        ):
            raise ValueError("episode operation/arm mismatch")
        if episode.get("reversed_from_inbound", False) is not False:
            raise ValueError(
                "reversed inbound demonstrations cannot replace outbound data"
            )
        for key in ("slot_id", "package_id", "failure_label"):
            _text(episode.get(key), key)
        deviation = episode.get("stopping_deviation", {})
        if (
            not isinstance(deviation, dict)
            or set(deviation) != {"x_mm", "y_mm", "yaw_deg"}
            or not all(_finite(v) for v in deviation.values())
        ):
            raise ValueError("stopping deviation requires finite mm/degree values")
        split = episode.get("split")
        if split not in SPLITS:
            raise ValueError("split must be train, validation or evaluation")
        for key, values in groups.items():
            group = _text(episode.get(key), key)
            if group in values and values[group] != split:
                raise ValueError(f"{key} split leakage")
            values[group] = split
        phases = episode.get("phases")
        if (
            not isinstance(phases, list)
            or any(not isinstance(p, dict) for p in phases)
            or [p.get("phase") for p in phases] != list(PHASES)
        ):
            raise ValueError("four ordered phase annotations are required")
        end = 0
        for phase in phases:
            start = _integer(phase.get("start_frame"), "phase start")
            stop = _integer(phase.get("end_frame"), "phase end", 1)
            if start < end or stop <= start or stop > length:
                raise ValueError("overlapping phase or episode boundary crossing")
            _text(phase.get("evidence"), "phase evidence")
            end = stop
        files = episode.get("camera_files")
        if not isinstance(files, dict) or files.keys() != profile["cameras"].keys():
            raise ValueError("episode camera files are incomplete")
        for paths in files.values():
            if not isinstance(paths, list) or not paths:
                raise ValueError("episode camera file list is empty")
            for path in paths:
                _file(root, path, "camera file")
        _timestamps(episode, profile)
    if (
        indices != set(range(len(episodes)))
        or info.get("total_episodes") != len(episodes)
        or info.get("total_frames") != sum(e["num_frames"] for e in episodes)
    ):
        raise ValueError("episode metadata/frame counts disagree")
    result = deepcopy(manifest)
    result["root"] = str(root)
    result["manifest_sha256"] = hashlib.sha256(
        (root / "manifest.json").read_bytes()
    ).hexdigest()
    return result


def _cancel(stop_event):
    if stop_event is not None and stop_event.is_set():
        raise InterruptedError("learning job cancellation requested")


def prepare_phase_dataset(
    root, expected_profile, phase, output_dir, *, dataset_cls=None, stop_event=None
):
    """Re-record phase clips through LeRobot's writer; each split owns its statistics."""
    manifest = inspect_dataset(root, expected_profile)
    if phase not in PHASES:
        raise ValueError("unknown phase")
    _cancel(stop_event)
    if dataset_cls is None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        dataset_cls = LeRobotDataset
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    profile = manifest["profile"]
    split_roots = {}
    clips = []
    for split in SPLITS:
        episodes = [e for e in manifest["episodes"] if e["split"] == split]
        if not episodes:
            continue
        _cancel(stop_event)
        writer = None
        try:
            for episode in episodes:
                _cancel(stop_event)
                if split == "train" and episode["failure_label"] != "NONE":
                    raise ValueError(
                        "failed demonstrations require explicit curation before ACT training"
                    )
                source = dataset_cls(
                    repo_id=manifest["repo_id"],
                    root=Path(root),
                    episodes=[episode["episode_index"]],
                    return_uint8=True,
                    tolerance_s=profile["timestamp_tolerance_s"],
                )
                if len(source) != episode["num_frames"]:
                    raise ValueError("actual episode length disagrees with manifest")
                if writer is None:
                    keys = ("observation.state", "action", *profile["cameras"])
                    writer = dataset_cls.create(
                        repo_id=f"{manifest['repo_id']}_{phase.lower()}_{split}",
                        root=output_dir / split,
                        fps=profile["fps"],
                        features={k: deepcopy(source.features[k]) for k in keys},
                        robot_type="omx_follower",
                        use_videos=True,
                        tolerance_s=profile["timestamp_tolerance_s"],
                    )
                clip = next(p for p in episode["phases"] if p["phase"] == phase)
                for index in range(episode["num_frames"]):
                    _cancel(stop_event)
                    row = source[index]
                    if (
                        row["episode_index"].item() != episode["episode_index"]
                        or row["frame_index"].item() != index
                        or abs(row["timestamp"].item() - index / profile["fps"])
                        > profile["timestamp_tolerance_s"]
                    ):
                        raise ValueError(
                            "actual frame index/episode/timestamp mismatch"
                        )
                    frame = {}
                    for key in ("observation.state", "action"):
                        values = row[key].tolist()
                        if len(values) != 6 or not all(_finite(v) for v in values):
                            raise ValueError(
                                "actual feature vector must contain six finite values"
                            )
                        frame[key] = row[key].numpy()
                    for key, camera in profile["cameras"].items():
                        height, width, channels = camera["shape"]
                        if tuple(row[key].shape) != (channels, height, width):
                            raise ValueError("decoded camera shape mismatch")
                        # The reader returns uint8 CHW; the dataset writer accepts uint8 HWC.
                        frame[key] = row[key].permute(1, 2, 0).numpy()
                    if clip["start_frame"] <= index < clip["end_frame"]:
                        writer.add_frame(
                            {**frame, "task": f"{profile['operation']} {phase}"}
                        )
                writer.save_episode()
                clips.append(
                    {
                        **{
                            key: deepcopy(episode[key])
                            for key in (
                                "split",
                                "source_episode_id",
                                "session_id",
                                "condition_group_id",
                                "failure_label",
                                "slot_id",
                                "package_id",
                                "stopping_deviation",
                            )
                        },
                        "source_episode_index": episode["episode_index"],
                        "episode_index": len([c for c in clips if c["split"] == split]),
                        **deepcopy(clip),
                    }
                )
            split_roots[split] = str(output_dir / split)
        finally:
            if writer is not None:
                writer.finalize()
    if "train" not in split_roots:
        raise ValueError("a curated train split is required")
    result = {
        "schema_version": "omx.prepared/1",
        "phase": phase,
        "splits": split_roots,
        "statistics_split": "train",
        "source_manifest_sha256": manifest["manifest_sha256"],
        "source_dataset_version": manifest["dataset_version"],
        "episodes": clips,
        "evaluation": "held-out clips are retained; no physical success has been measured",
    }
    _write_json(output_dir / "prepared.json", result)
    return result


def _parameters(request):
    if (
        request.get("schema_version") != "omx.workflow/1"
        or request.get("operation") != "train"
    ):
        raise ValueError("expected an omx.workflow/1 train request")
    params = request.get("parameters")
    if not isinstance(params, dict) or set(params) != PARAMETERS:
        raise ValueError("training parameters must use the explicit ACT allowlist")
    for key in ("dataset_id", "output_id"):
        if not isinstance(params[key], str) or not re.fullmatch(
            r"[A-Za-z0-9][A-Za-z0-9_.-]*", params[key]
        ):
            raise ValueError(f"{key} must be a relative ID")
    if params["phase"] not in PHASES:
        raise ValueError("unknown phase")
    for key in ("steps", "batch_size", "chunk_size", "n_action_steps"):
        _integer(params[key], key, 1)
    _integer(params["seed"], "seed")
    if params["n_action_steps"] > params["chunk_size"]:
        raise ValueError("n_action_steps exceeds action chunk")
    return params


def _executable(profile):
    path = Path(profile["python_executable"])
    if not path.is_absolute() or not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError(
            "an absolute executable from the explicit learning environment is required"
        )
    return str(path)


def _output_path(profile, params):
    root = Path(profile["checkpoint_root"]).resolve()
    if (root / params["output_id"]).is_symlink():
        raise ValueError("training output cannot be a symlink")
    return _inside(root, params["output_id"])


def build_act_training(
    request: dict, dataset_manifest: dict, runtime_profile: dict
) -> list[str]:
    """Build an argv list from validated IDs and trusted runtime paths only."""
    params = _parameters(request)
    source_root = _inside(runtime_profile["dataset_root"], params["dataset_id"])
    manifest = inspect_dataset(source_root, runtime_profile["expected_profile"])
    if manifest != dataset_manifest:
        raise ValueError("dataset manifest changed; inspect it again")
    output = _output_path(runtime_profile, params)
    if output.exists():
        raise FileExistsError("training output already exists")
    prepared = Path(runtime_profile["prepared_dataset_root"])
    if not prepared.is_absolute():
        raise ValueError("prepared dataset root must be absolute")
    device = runtime_profile.get("device")
    if not isinstance(device, str) or not re.fullmatch(
        r"cpu|mps|cuda(?::[0-9]+)?", device
    ):
        raise ValueError("an explicit training device is required")
    return [
        _executable(runtime_profile),
        "-m",
        "lerobot.scripts.lerobot_train",
        "--policy.type=act",
        f"--policy.device={device}",
        "--policy.push_to_hub=false",
        "--policy.pretrained_backbone_weights=null",
        "--wandb.enable=false",
        "--save_checkpoint_to_hub=false",
        "--resume=false",
        "--save_checkpoint=true",
        "--dataset.use_imagenet_stats=false",
        "--dataset.eval_split=0",
        "--eval_steps=0",
        "--env_eval_freq=0",
        "--num_workers=0",
        f"--dataset.repo_id={manifest['repo_id']}_{params['phase'].lower()}_train",
        f"--dataset.root={prepared / 'train'}",
        f"--output_dir={output}",
        f"--steps={params['steps']}",
        f"--batch_size={params['batch_size']}",
        f"--seed={params['seed']}",
        f"--policy.chunk_size={params['chunk_size']}",
        f"--policy.n_action_steps={params['n_action_steps']}",
    ]


def _run_process(argv, *, cwd, env, log_path, stop_event, shell):
    with Path(log_path).open("wb") as log:
        process = subprocess.Popen(
            argv, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, shell=shell
        )
        try:
            while True:
                _cancel(stop_event)
                try:
                    result = process.wait(timeout=0.1)
                    _cancel(stop_event)
                    return result
                except subprocess.TimeoutExpired:
                    pass
        except BaseException:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            raise


def _checkpoint(output, params, profile):
    checkpoint = _inside(output, "checkpoints/last/pretrained_model")
    if not checkpoint.is_dir():
        raise ValueError("training exited without a completed checkpoint")
    _file(checkpoint, "model.safetensors", "checkpoint weights")
    config = _json(_file(checkpoint, "config.json", "checkpoint config"))
    if (
        config.get("type") != "act"
        or config.get("chunk_size") != params["chunk_size"]
        or config.get("n_action_steps") != params["n_action_steps"]
    ):
        raise ValueError("checkpoint ACT configuration mismatch")
    inputs = config.get("input_features", {})
    if inputs.get("observation.state", {}).get("shape") != [6] or config.get(
        "output_features", {}
    ).get("action", {}).get("shape") != [6]:
        raise ValueError("checkpoint feature dimension mismatch")
    if set(inputs) != {"observation.state", *profile["cameras"]}:
        raise ValueError("checkpoint camera set mismatch")
    for key, camera in profile["cameras"].items():
        h, w, c = camera["shape"]
        if inputs[key].get("shape") != [c, h, w]:
            raise ValueError("checkpoint camera dimension mismatch")
    for name in ("preprocessor", "postprocessor"):
        processor = _json(_file(checkpoint, f"policy_{name}.json", "processor config"))
        steps = processor.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError("checkpoint processor steps are missing")
        registry = (
            "normalizer_processor"
            if name == "preprocessor"
            else "unnormalizer_processor"
        )
        normalizers = [
            step
            for step in steps
            if isinstance(step, dict) and step.get("registry_name") == registry
        ]
        if len(normalizers) != 1 or not normalizers[0].get("state_file"):
            raise ValueError("checkpoint normalization statistics are missing")
        for step in steps:
            if not isinstance(step, dict):
                raise TypeError("invalid processor step")
            if step.get("state_file") is not None:
                _file(checkpoint, step["state_file"], "processor state")
    return checkpoint


def _run_training(
    request: dict, dataset_manifest: dict, runtime_profile: dict, job_dir: Path
) -> dict:
    """Prepare clips and train locally; success requires saved policy and processors."""
    params = _parameters(request)
    profile = dict(runtime_profile)
    stop_event = profile.get("stop_event")
    _cancel(stop_event)
    job_dir = Path(job_dir).resolve()
    prepared = job_dir / "phase_dataset"
    profile["prepared_dataset_root"] = prepared
    argv = build_act_training(request, dataset_manifest, profile)
    cli = Path(profile["learning_cli"])
    if not cli.is_absolute() or not cli.is_file():
        raise ValueError("explicit absolute learning CLI path is required")
    _text(profile.get("runtime_version"), "runtime version")
    if not isinstance(profile.get("environment"), dict) or not profile["environment"]:
        raise ValueError("record the explicit learning environment versions")
    model_metadata = profile.get("model_metadata")
    if not isinstance(model_metadata, dict):
        raise TypeError("trusted model registration metadata is required")
    _text(model_metadata.get("policy_bundle_version"), "policy bundle version")
    job_dir.mkdir(parents=True, exist_ok=True)
    expected_path = job_dir / "expected_profile.json"
    _write_json(expected_path, profile["expected_profile"])
    prepare_argv = [
        _executable(profile),
        str(cli),
        "prepare",
        "--root",
        dataset_manifest["root"],
        "--expected-profile",
        str(expected_path),
        "--phase",
        params["phase"],
        "--output-dir",
        str(prepared),
    ]
    env = dict(os.environ)
    env.update(
        HF_HUB_OFFLINE="1",
        HF_DATASETS_OFFLINE="1",
        WANDB_MODE="disabled",
        HF_HUB_DISABLE_TELEMETRY="1",
    )
    runner = profile.get("runner", _run_process)
    for label, command in (("prepare", prepare_argv), ("train", argv)):
        _cancel(stop_event)
        result = runner(
            command,
            cwd=job_dir,
            env=env,
            log_path=job_dir / f"{label}.log",
            stop_event=stop_event,
            shell=False,
        )
        if result != 0:
            raise RuntimeError(
                f"{label} process exited with code {result}; see job log"
            )
        _cancel(stop_event)
        if label == "prepare":
            receipt = _json(prepared / "prepared.json")
            if (
                receipt.get("source_manifest_sha256")
                != dataset_manifest["manifest_sha256"]
                or receipt.get("phase") != params["phase"]
                or receipt.get("statistics_split") != "train"
            ):
                raise ValueError("prepared dataset provenance mismatch")
            _file(prepared, "train/meta/stats.json", "train-only dataset statistics")
            # Validate again after preparation; a source modified during the job is not accepted.
            build_act_training(request, dataset_manifest, profile)
    output = _output_path(profile, params)
    checkpoint = _checkpoint(output, params, dataset_manifest["profile"])
    data_profile = dataset_manifest["profile"]
    metadata = deepcopy(model_metadata)
    metadata.update(
        {
            "model_version": params["output_id"],
            "operation": data_profile["operation"],
            "assigned_arm_id": data_profile["arm_id"],
            "phase": params["phase"],
            **{
                key: deepcopy(data_profile[key])
                for key in (
                    "feature_names",
                    "feature_units",
                    "use_degrees",
                    "feature_schema_version",
                    "calibration_version",
                )
            },
            "cameras": {
                key.removeprefix("observation.images."): {
                    "shape": [c["shape"][2], *c["shape"][:2]],
                    "fps": c["fps"],
                }
                for key, c in data_profile["cameras"].items()
            },
            "device": profile["device"],
            "dataset_version": dataset_manifest["dataset_version"],
            "action_chunk": {
                key: params[key] for key in ("chunk_size", "n_action_steps")
            },
            "training": {
                "parameters": deepcopy(params),
                "seed": params["seed"],
                "runtime_version": profile["runtime_version"],
                "environment": deepcopy(profile["environment"]),
                "manifest_sha256": dataset_manifest["manifest_sha256"],
                "argv": argv,
                "statistics_split": "train",
            },
            "evaluation": {
                "physical_success_rate": None,
                "failure_types": [],
                "conditions": [],
                "status": "NOT_RUN",
                "limits": "training completion only; held-out loss and physical rollout are not verified",
            },
        }
    )
    _write_json(checkpoint / "metadata.json", metadata)
    _write_json(
        job_dir / "training_result.json",
        {"checkpoint": str(checkpoint), "metadata": metadata},
    )
    return {
        "state": "SUCCEEDED",
        "checkpoint": str(checkpoint),
        "metadata": str(checkpoint / "metadata.json"),
        "physical_validation": "NOT_RUN",
    }


def run_training(
    request: dict, dataset_manifest: dict, runtime_profile: dict, job_dir: Path
) -> dict:
    """Reserve the output ID across processes; failed output remains for inspection."""
    params = _parameters(request)
    _cancel(runtime_profile.get("stop_event"))
    output = _output_path(runtime_profile, params)
    if output.exists():
        raise FileExistsError("training output already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    lock = output.parent / f".{params['output_id']}.lock"
    token = uuid4().hex
    # Exclusive creation also rejects existing and dangling lock symlinks.
    with lock.open("x") as reservation:
        reservation.write(token)
        reservation.flush()
        os.fsync(reservation.fileno())
    try:
        return _run_training(request, dataset_manifest, runtime_profile, job_dir)
    finally:
        if not lock.is_symlink() and lock.is_file() and lock.read_text() == token:
            lock.unlink()
