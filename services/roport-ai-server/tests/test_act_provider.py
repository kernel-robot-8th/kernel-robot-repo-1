"""Fake ACT runtime/source contracts; no torch, weights download or hardware."""

import ast
import json
import math
import sys
from contextlib import nullcontext
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest

import act_provider
from act_provider import ActProvider

METADATA = {
    "operation": "INBOUND",
    "assigned_arm_id": "omx1",
    "phase": "GRASPING",
    "model_version": "synthetic-model-v1",
    "policy_bundle_version": "synthetic-bundle-v1",
    "feature_schema_version": "synthetic-features-v1",
    "calibration_version": "synthetic-calibration-v1",
    "feature_names": [f"synthetic_joint_{i}" for i in range(6)],
    "use_degrees": True,
    "feature_units": ["degrees"] * 5 + ["range_0_100"],
    "cameras": {"side": {"shape": [3, 4, 5], "fps": 30}},
    "device": "cpu",
    "action_chunk": {"chunk_size": 3, "n_action_steps": 2},
}
CONFIG = {
    "type": "act",
    "input_features": {
        "observation.state": {"type": "STATE", "shape": [6]},
        "observation.images.side": {"type": "VISUAL", "shape": [3, 4, 5]},
    },
    "output_features": {"action": {"type": "ACTION", "shape": [6]}},
    "chunk_size": 3,
    "n_action_steps": 2,
    "pretrained_backbone_weights": "never-download-this",
    "temporal_ensemble_coeff": None,
}


def write_json(path, data):
    path.write_text(json.dumps(data))


@pytest.fixture
def checkpoint(tmp_path):
    write_json(tmp_path / "config.json", CONFIG)
    write_json(tmp_path / "metadata.json", METADATA)
    (tmp_path / "model.safetensors").write_bytes(b"synthetic-not-real-weights")
    for name in ("policy_preprocessor", "policy_postprocessor"):
        write_json(
            tmp_path / f"{name}.json",
            {"steps": [{"state_file": f"{name}.safetensors"}]},
        )
        (tmp_path / f"{name}.safetensors").write_bytes(b"synthetic-stats")
    return tmp_path


class Tensor:
    def __init__(self, values=None, shape=None):
        self.values = values
        self.shape = shape

    def to(self, **kwargs):
        return self

    def __truediv__(self, value):
        assert value == 255
        return self

    def __getitem__(self, index):
        return Tensor([[float(index[1])] * 6], [1, 6])

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.values


class Processor:
    def __init__(self, *, post=False):
        self.resets = 0
        self.calls = []
        self.post = post
        self.nonfinite = False

    def reset(self):
        self.resets += 1
        self.calls.clear()

    def __call__(self, value):
        self.calls.append(value)
        if self.post:
            return Tensor(
                [[float("nan") if self.nonfinite else v + 10 for v in value.values[0]]],
                [1, 6],
            )
        return value


class Policy(Processor):
    def predict_action_chunk(self, value):
        self.calls.append(value)
        return Tensor(shape=[1, 3, 6])


@pytest.fixture
def runtime(monkeypatch):
    torch = SimpleNamespace(
        float32="float32",
        tensor=lambda value, **kwargs: Tensor(value),
        inference_mode=nullcontext,
    )
    policy, pre, post = Policy(), Processor(), Processor(post=True)
    calls = []

    def load(checkpoint, metadata):
        calls.append((checkpoint, metadata))
        return torch, policy, pre, post

    monkeypatch.setattr(act_provider, "_load_runtime", load)
    monkeypatch.setattr(
        act_provider, "_decode_jpeg", lambda data, torch: Tensor(shape=[3, 4, 5])
    )
    return policy, pre, post, calls


def observation():
    return {
        "positions": [0.0] * 6,
        "feature_names": METADATA["feature_names"].copy(),
        "use_degrees": True,
        "images": {"side": {"synthetic": "header"}},
    }


def test_lazy_load_saved_postprocessor_and_reset_every_component(checkpoint, runtime):
    policy, pre, post, calls = runtime
    provider = ActProvider(
        checkpoint, METADATA, lambda obs: {"side": b"synthetic-jpeg"}
    )
    assert calls == []
    with pytest.raises(ValueError, match="reset"):
        provider.infer(observation())
    provider.reset()
    assert provider.infer(observation()) == [[10.0] * 6, [11.0] * 6]
    assert set(pre.calls[0]) == {"observation.state", "observation.images.side"}
    assert len(post.calls) == 2
    provider.reset()
    assert (policy.resets, pre.resets, post.resets) == (2, 2, 2)
    assert policy.calls == pre.calls == post.calls == []
    assert len(calls) == 1


@pytest.mark.parametrize(
    "field,value",
    [
        ("feature_names", ["wrong"] * 6),
        ("feature_units", ["radian"] * 6),
        ("assigned_arm_id", "omx2"),
        ("model_version", "unregistered"),
        ("cameras", {"other": {"shape": [3, 4, 5], "fps": 30}}),
        ("device", "auto"),
        ("action_chunk", {"chunk_size": 5, "n_action_steps": 4}),
    ],
)
def test_registered_model_contract_rejects_mismatch(checkpoint, field, value):
    with pytest.raises(ValueError):
        ActProvider(checkpoint, METADATA | {field: value}, lambda obs: {})


@pytest.mark.parametrize(
    "name",
    [
        "model.safetensors",
        "policy_preprocessor.json",
        "policy_postprocessor.safetensors",
        "metadata.json",
    ],
)
def test_missing_local_artifact_rejected(checkpoint, name):
    (checkpoint / name).unlink()
    with pytest.raises(ValueError):
        ActProvider(checkpoint, METADATA, lambda obs: {})


def test_checkpoint_schema_ensemble_and_processor_path_rejected(checkpoint):
    for change in (
        {"type": "diffusion"},
        {"input_features": {}},
        {"output_features": {"action": {"shape": [12]}}},
        {"temporal_ensemble_coeff": 0.1},
    ):
        write_json(checkpoint / "config.json", CONFIG | change)
        with pytest.raises(ValueError):
            ActProvider(checkpoint, METADATA, lambda obs: {})
    write_json(checkpoint / "config.json", CONFIG)
    write_json(
        checkpoint / "policy_preprocessor.json",
        {"steps": [{"state_file": "../outside.safetensors"}]},
    )
    with pytest.raises(ValueError, match="escapes"):
        ActProvider(checkpoint, METADATA, lambda obs: {})


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, True])
def test_invalid_positions_fail_before_model_and_require_reset(
    checkpoint, runtime, value
):
    policy, _, _, _ = runtime
    provider = ActProvider(
        checkpoint, METADATA, lambda obs: {"side": b"synthetic-jpeg"}
    )
    provider.reset()
    bad = observation()
    bad["positions"][0] = value
    with pytest.raises(ValueError):
        provider.infer(bad)
    assert policy.calls == []
    with pytest.raises(ValueError, match="reset"):
        provider.infer(observation())


def test_missing_images_shape_nonfinite_and_missing_processor_reset(
    checkpoint, runtime, monkeypatch
):
    _, _, post, _ = runtime
    provider = ActProvider(checkpoint, METADATA, lambda obs: {})
    provider.reset()
    with pytest.raises(ValueError, match="images"):
        provider.infer(observation())
    provider.image_resolver = lambda obs: {"side": b"synthetic-jpeg"}
    provider.reset()
    monkeypatch.setattr(
        act_provider, "_decode_jpeg", lambda data, torch: Tensor(shape=[3, 99, 5])
    )
    with pytest.raises(ValueError, match="shape"):
        provider.infer(observation())
    monkeypatch.setattr(
        act_provider, "_decode_jpeg", lambda data, torch: Tensor(shape=[3, 4, 5])
    )
    post.nonfinite = True
    provider.reset()
    with pytest.raises(ValueError, match="nonfinite"):
        provider.infer(observation())
    post.reset = None
    with pytest.raises(TypeError, match="reset"):
        provider.reset()


def test_source_has_no_eager_optional_imports_and_uses_v061_api():
    module = ast.parse(Path(act_provider.__file__).read_text())
    imports = [
        node for node in module.body if isinstance(node, (ast.Import, ast.ImportFrom))
    ]
    assert not any(
        "torch" in ast.unparse(node) or "lerobot" in ast.unparse(node)
        for node in imports
    )
    factory = (
        Path(__file__).resolve().parents[2]
        / "roport-omx/lerobot/src/lerobot/policies/factory.py"
    )
    source = ast.parse(factory.read_text())
    names = {node.name for node in source.body if isinstance(node, ast.FunctionDef)}
    assert {"get_policy_class", "make_pre_post_processors"} <= names


def test_loader_local_only_overrides_backbone_and_preserves_saved_processors(
    checkpoint, monkeypatch
):
    calls = []
    config = SimpleNamespace(device="cuda", pretrained_backbone_weights="network")

    class FakeConfig:
        @staticmethod
        def from_pretrained(path, **kwargs):
            assert path == checkpoint and kwargs == {"local_files_only": True}
            return config

    class FakePolicy:
        @classmethod
        def from_pretrained(cls, path, **kwargs):
            assert kwargs == {
                "config": config,
                "local_files_only": True,
                "strict": True,
            }
            assert config.device == "cpu" and config.pretrained_backbone_weights is None
            return SimpleNamespace(config=config, eval=lambda: calls.append("eval"))

    def processors(cfg, **kwargs):
        assert cfg is config
        assert kwargs["pretrained_path"] == str(checkpoint)
        assert "dataset_stats" not in kwargs
        calls.append("saved-processors")
        return "pre", "post"

    for name, module in {
        "torch": SimpleNamespace(),
        "lerobot.configs.policies": SimpleNamespace(PreTrainedConfig=FakeConfig),
        "lerobot.policies.factory": SimpleNamespace(
            get_policy_class=lambda name: FakePolicy if name == "act" else None,
            make_pre_post_processors=processors,
        ),
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    _, _, pre, post = act_provider._load_runtime(checkpoint, deepcopy(METADATA))
    assert (pre, post) == ("pre", "post")
    assert calls == ["saved-processors", "eval"]
