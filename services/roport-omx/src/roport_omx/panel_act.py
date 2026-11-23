"""Local ACT inference for recorded and live OMX observations."""

import os
from pathlib import Path


class PanelAct:
    def __init__(self, checkpoint):
        if not (checkpoint / "model.safetensors").is_file():
            raise ValueError("local ACT checkpoint required")

        import torch
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors

        self.torch = torch
        self.model = (
            ACTPolicy.from_pretrained(checkpoint, local_files_only=True).cuda().eval()
        )
        self.pre, self.post = make_pre_post_processors(
            self.model.config,
            pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": {"device": "cuda"}},
        )
        self.warmup()

    def warmup(self):
        """Initialize CUDA inference before device ownership; discard every action."""
        observation = {
            key: self.torch.zeros(feature.shape, dtype=self.torch.float32)
            for key, feature in self.model.config.input_features.items()
            if key != "observation.environment_state"
        }
        try:
            for level in (1, 2):
                self.reset()
                self.select_action(observation, level)
                self.torch.cuda.synchronize()
        finally:
            self.reset()

    def reset(self):
        self.model.reset()
        self.pre.reset()
        self.post.reset()

    def _batch(self, observation, level):
        if type(level) is not int or level not in (1, 2):
            raise ValueError("select shelf level 1 or 2")
        torch = self.torch
        floor_key = "observation.environment_state"
        features = self.model.config.input_features
        if floor_key not in features:
            raise ValueError("ACT checkpoint lacks shelf conditioning")
        batch = {
            key: torch.as_tensor(observation[key])
            for key in features
            if key != floor_key
        }
        batch[floor_key] = torch.tensor([float(level == 1), float(level == 2)])
        for key, feature in features.items():
            value = batch[key]
            if key.startswith("observation.images.") and value.dtype == torch.uint8:
                if tuple(value.shape) == (*feature.shape[1:], feature.shape[0]):
                    value = value.permute(2, 0, 1)
                value = value.to(torch.float32) / 255
            if (
                tuple(value.shape) != tuple(feature.shape)
                or not torch.isfinite(value).all()
            ):
                raise ValueError(f"invalid ACT input: {key}")
            batch[key] = value.to(torch.float32)
        return batch

    def select_action(self, observation, level):
        """Use ACT's configured action queue/ensemble for a live control cycle."""
        with self.torch.inference_mode():
            action = self.post(
                self.model.select_action(self.pre(self._batch(observation, level)))
            )
        action = action[0].cpu()
        if tuple(action.shape) != (6,) or not self.torch.isfinite(action).all():
            raise ValueError("invalid ACT action")
        return action

    def predict(self, observation, level):
        torch = self.torch
        batch = self._batch(observation, level)
        with torch.inference_mode():
            actions = self.post(
                self.model.predict_action_chunk(self.pre(batch))[0]
            ).cpu()
        if (
            actions.ndim != 2
            or actions.shape[1] != 6
            or not torch.isfinite(actions).all()
        ):
            raise ValueError("invalid ACT prediction")
        return actions


def preview_act(operation, level):
    if (
        operation not in ("inbound", "outbound")
        or type(level) is not int
        or level not in (1, 2)
    ):
        raise ValueError("select inbound/outbound and shelf level 1 or 2")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(os.environ["ROPORT_ACT_DATA_ROOT"]) / operation / "validation"
    checkpoint = (
        Path(os.environ["ROPORT_ACT_RUN_ROOT"])
        / operation
        / "checkpoints/last/pretrained_model"
    )
    if not (root / "meta/info.json").is_file():
        raise ValueError("local validation data required")
    dataset = LeRobotDataset(f"omx1-{operation}", root=root, video_backend="pyav")
    actions = PanelAct(checkpoint).predict(dataset[0], level)
    return {
        "operation": operation,
        "shelf_level": level,
        "steps": len(actions),
        "first_action": actions[0].tolist(),
    }
