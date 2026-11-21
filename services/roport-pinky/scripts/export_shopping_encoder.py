"""Export the pad's local DINOv2 encoder on the PC for ONNX Runtime on Pinky."""
import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

from roport_smartcart.appearance_reid import DinoEncoder, normalized


class Embedding(torch.nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, image):
        return self.model.forward_features(image)['x_norm_clstoken']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('weights', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    model = DinoEncoder(args.source, args.weights).model
    batch = torch.randn(2, 3, 140, 140, generator=torch.Generator().manual_seed(14))
    with torch.inference_mode():
        expected = normalized(model(batch).numpy())
        # The pad always uses a 10 x 10 patch grid. Precompute its position embedding.
        positions = model.interpolate_pos_encoding(torch.zeros(1, 101, 384), 140, 140)
        model.pos_embed = torch.nn.Parameter(positions.clone(), requires_grad=False)
        torch.onnx.export(Embedding(model).eval(), batch, str(args.output), opset_version=18,
                          input_names=['crop'], output_names=['features'], dynamo=False,
                          dynamic_axes={'crop': {0: 'batch'}, 'features': {0: 'batch'}})
    session = ort.InferenceSession(str(args.output), providers=['CPUExecutionProvider'])
    for count in (1, 2):
        values, = session.run(None, {'crop': batch[:count].numpy()})
        np.testing.assert_allclose(normalized(values), expected[:count], atol=1e-4)


if __name__ == '__main__':
    main()
