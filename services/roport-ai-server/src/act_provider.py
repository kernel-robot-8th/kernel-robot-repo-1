"""Local ACT action candidates with saved LeRobot v0.6.1 processors.

This module imports no torch/LeRobot until reset. It never applies robot actions.
"""

import json
import math
from copy import deepcopy
from pathlib import Path

from dedicated_policy import PHASES, REGISTRY_FIELDS
from udp_frame_protocol import MAX_FRAME_BYTES


def _read_json(path):
    with path.open(encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise TypeError(f"JSON object required: {path.name}")
    return value


def _vector(value):
    return (
        isinstance(value, list)
        and len(value) == 6
        and all(type(v) in (float, int) and math.isfinite(v) for v in value)
    )


def _load_runtime(checkpoint, metadata):
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    config = PreTrainedConfig.from_pretrained(checkpoint, local_files_only=True)
    config.device = metadata["device"]
    # The complete checkpoint supplies the backbone; never fetch ImageNet weights.
    config.pretrained_backbone_weights = None
    policy = get_policy_class("act").from_pretrained(
        checkpoint, config=config, local_files_only=True, strict=True
    )
    preprocess, postprocess = make_pre_post_processors(
        policy.config,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={
            "device_processor": {"device": metadata["device"]},
            "normalizer_processor": {"device": metadata["device"]},
        },
        postprocessor_overrides={
            "device_processor": {"device": "cpu"},
            "unnormalizer_processor": {"device": "cpu"},
        },
    )
    policy.eval()
    return torch, policy, preprocess, postprocess


def _decode_jpeg(data, torch):
    from torchvision.io import ImageReadMode, decode_jpeg

    return decode_jpeg(
        torch.frombuffer(bytearray(data), dtype=torch.uint8), mode=ImageReadMode.RGB
    )


class ActProvider:
    """Serial reset()/infer(observation) provider for PolicyRouter.

    metadata.json binds a local checkpoint to the trusted registry. cameras uses
    wire camera keys and CHW RGB shapes; feature_units follows feature_names.
    CPU/CUDA execution requires a separately installed LeRobot ACT environment.
    """

    def __init__(self, checkpoint, metadata, image_resolver):
        self.checkpoint = Path(checkpoint).resolve(strict=True)
        if not self.checkpoint.is_dir() or not callable(image_resolver):
            raise ValueError("local checkpoint directory and image resolver required")
        if not isinstance(metadata, dict):
            raise TypeError("registered model metadata required")
        self.metadata = deepcopy(metadata)
        self.image_resolver = image_resolver
        self._runtime = None
        self._ready = False
        self._validate_metadata()
        self._validate_checkpoint()

    def _validate_metadata(self):
        meta = self.metadata
        if any(
            not isinstance(meta.get(field), str) or not meta[field].strip()
            for field in REGISTRY_FIELDS
        ):
            raise ValueError("registered model/version metadata required")
        names = meta.get("feature_names")
        if (
            meta["operation"] not in {"INBOUND", "OUTBOUND"}
            or meta["assigned_arm_id"]
            != {"INBOUND": "omx1", "OUTBOUND": "omx2"}.get(meta["operation"])
            or meta["phase"] not in PHASES
            or not isinstance(names, list)
            or len(names) != 6
            or any(not isinstance(name, str) or not name.strip() for name in names)
            or len(set(names)) != 6
            or type(meta.get("use_degrees")) is not bool
        ):
            raise ValueError(
                "six ordered single-arm features and explicit units required"
            )
        arm_unit = "degrees" if meta["use_degrees"] else "range_m100_100"
        if meta.get("feature_units") != [arm_unit] * 5 + ["range_0_100"]:
            raise ValueError("feature_units differ from arm/gripper normalization")
        if meta.get("device") not in {"cpu", "cuda"}:
            raise ValueError("explicit cpu or cuda device required")
        cameras = meta.get("cameras")
        if not isinstance(cameras, dict) or not cameras:
            raise ValueError("explicit camera contract required")
        for camera, spec in cameras.items():
            if (
                not isinstance(camera, str)
                or not camera
                or not isinstance(spec, dict)
                or set(spec) != {"shape", "fps"}
            ):
                raise ValueError("camera key, CHW shape and FPS required")
            shape = spec["shape"]
            if (
                not isinstance(shape, list)
                or len(shape) != 3
                or shape[0] != 3
                or any(type(n) is not int or n <= 0 for n in shape)
                or type(spec["fps"]) not in (int, float)
                or not math.isfinite(spec["fps"])
                or spec["fps"] <= 0
            ):
                raise ValueError("positive RGB CHW shape and finite FPS required")

    def _validate_checkpoint(self):
        for name in (
            "config.json",
            "model.safetensors",
            "policy_preprocessor.json",
            "policy_postprocessor.json",
            "metadata.json",
        ):
            path = self.checkpoint / name
            if not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f"complete local checkpoint required: {name}")
        registered = _read_json(self.checkpoint / "metadata.json")
        # Device is a deployment selection; all model/data bindings must match.
        for key, value in self.metadata.items():
            if key != "device" and registered.get(key) != value:
                raise ValueError(f"checkpoint metadata mismatch: {key}")
        config = _read_json(self.checkpoint / "config.json")
        expected = {"observation.state": [6]} | {
            f"observation.images.{camera}": spec["shape"]
            for camera, spec in self.metadata["cameras"].items()
        }
        inputs = config.get("input_features", {})
        outputs = config.get("output_features", {})
        if (
            config.get("type") != "act"
            or not isinstance(inputs, dict)
            or set(inputs) != set(expected)
            or any(inputs[key].get("shape") != shape for key, shape in expected.items())
            or not isinstance(outputs, dict)
            or set(outputs) != {"action"}
            or outputs["action"].get("shape") != [6]
        ):
            raise ValueError("checkpoint ACT feature/camera schema mismatch")
        chunk = {key: config.get(key) for key in ("chunk_size", "n_action_steps")}
        if (
            any(type(n) is not int or n <= 0 for n in chunk.values())
            or chunk["n_action_steps"] > chunk["chunk_size"]
            or self.metadata.get("action_chunk") != chunk
            or config.get("temporal_ensemble_coeff") is not None
        ):
            raise ValueError(
                "explicit non-ensembled ACT action chunk contract required"
            )
        self.chunk_size = chunk["chunk_size"]
        self.n_action_steps = chunk["n_action_steps"]
        for name in ("policy_preprocessor.json", "policy_postprocessor.json"):
            processor = _read_json(self.checkpoint / name)
            self._validate_state_files(processor)

    def _validate_state_files(self, value):
        if isinstance(value, list):
            for item in value:
                self._validate_state_files(item)
        elif isinstance(value, dict):
            for key, item in value.items():
                if key == "state_file" and item is not None:
                    if not isinstance(item, str) or not item:
                        raise ValueError("invalid processor state file")
                    path = (self.checkpoint / item).resolve()
                    if not path.is_relative_to(self.checkpoint) or not path.is_file():
                        raise ValueError(
                            "local processor state file missing or escapes checkpoint"
                        )
                else:
                    self._validate_state_files(item)

    def reset(self):
        self._ready = False
        if self._runtime is None:
            self._runtime = _load_runtime(self.checkpoint, self.metadata)
        _, policy, preprocess, postprocess = self._runtime
        for component in (policy, preprocess, postprocess):
            if not callable(getattr(component, "reset", None)):
                raise TypeError("policy and both processors must support reset")
            component.reset()
        self._ready = True

    def infer(self, observation) -> list[list[float]]:
        if not self._ready:
            raise ValueError("ACT reset required")
        try:
            return self._infer(observation)
        except Exception:
            self._ready = False
            raise

    def _infer(self, observation):
        if (
            not isinstance(observation, dict)
            or not _vector(observation.get("positions"))
            or observation.get("feature_names") != self.metadata["feature_names"]
            or type(observation.get("use_degrees")) is not bool
            or observation["use_degrees"] != self.metadata["use_degrees"]
            or not isinstance(observation.get("images"), dict)
            or set(observation["images"]) != set(self.metadata["cameras"])
        ):
            raise ValueError("invalid ACT observation features/units/cameras")
        encoded = self.image_resolver(observation)
        if not isinstance(encoded, dict) or set(encoded) != set(
            self.metadata["cameras"]
        ):
            raise ValueError("complete current images required")
        torch, policy, preprocess, postprocess = self._runtime
        batch = {
            "observation.state": torch.tensor(
                observation["positions"], dtype=torch.float32
            )
        }
        for camera, data in encoded.items():
            if not isinstance(data, bytes) or not 0 < len(data) <= MAX_FRAME_BYTES:
                raise ValueError("bounded encoded JPEG bytes required")
            image = _decode_jpeg(data, torch)
            if list(image.shape) != self.metadata["cameras"][camera]["shape"]:
                raise ValueError("decoded RGB image shape mismatch")
            batch[f"observation.images.{camera}"] = image.to(dtype=torch.float32) / 255
        with torch.inference_mode():
            predicted = policy.predict_action_chunk(preprocess(batch))
            if list(predicted.shape) != [1, self.chunk_size, 6]:
                raise ValueError("invalid ACT chunk shape")
            actions = []
            for index in range(self.n_action_steps):
                # Saved postprocessors expect [batch, action], not [batch, time, action].
                action = postprocess(predicted[:, index, :]).detach().cpu().tolist()
                if (
                    not isinstance(action, list)
                    or len(action) != 1
                    or not _vector(action[0])
                ):
                    raise ValueError("nonfinite or invalid six-dimensional ACT action")
                actions.append([float(value) for value in action[0]])
        return actions
