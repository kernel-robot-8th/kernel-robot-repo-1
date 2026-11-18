"""PC appearance matching. Scores are similarities, not identity probabilities."""
import json
from pathlib import Path

import numpy as np


def normalized(features):
    values = np.asarray(features, dtype=np.float32)
    if values.ndim != 2 or not values.shape[1] or not np.isfinite(values).all():
        raise ValueError('features must be a finite matrix')
    lengths = np.linalg.norm(values, axis=1, keepdims=True)
    if not np.isfinite(lengths).all() or np.any(lengths <= 1.e-8):
        raise ValueError('empty appearance feature')
    return values / lengths


class AppearanceGallery:
    def __init__(self, labels, features, threshold=.7, margin=.05):
        self.features = normalized(features)
        if (not labels or len(labels) != len(self.features)
                or any(not isinstance(label, str) or not label.strip() for label in labels)
                or not np.isfinite([threshold, margin]).all()
                or not -1 <= threshold <= 1 or not 0 < margin <= 2):
            raise ValueError('invalid gallery or matching limits')
        self.labels = tuple(labels)
        self.identities = tuple(dict.fromkeys(labels))
        self.threshold, self.margin = threshold, margin

    def classify(self, features):
        values = normalized(features)
        if values.shape[1] != self.features.shape[1]:
            raise ValueError('encoder feature dimensions differ')
        similarities = values @ self.features.T
        result = []
        for row in similarities:
            # Compete identities, not different views of the same identity.
            scores = {label: float(max(row[i] for i, name in enumerate(self.labels) if name == label))
                      for label in self.identities}
            ranked = sorted(scores, key=scores.get, reverse=True)
            best = ranked[0]
            difference = scores[best] - scores[ranked[1]] if len(ranked) > 1 else 2.
            accepted = scores[best] >= self.threshold and difference >= self.margin
            result.append({'label': best if accepted else None,
                           'score': scores[best], 'margin': difference, 'scores': scores})
        return result

    def select(self, target, matches):
        if target not in self.identities:
            raise ValueError('target is not registered')
        # shortcut: identical copies cannot be distinguished; validate extra identity evidence before deployment.
        hits = [i for i, match in enumerate(matches) if match['label'] == target]
        return {'state': 'MATCH' if len(hits) == 1 else 'AMBIGUOUS' if hits else 'LOST',
                'index': hits[0] if len(hits) == 1 else None}


class DinoEncoder:
    """Explicit trusted local DINOv2 source/weights; no network or robot access."""
    def __init__(self, source, weights):
        import torch
        from PIL import Image, ImageOps
        source, weights = Path(source), Path(weights)
        if not (source/'hubconf.py').is_file() or not weights.is_file():
            raise ValueError('local DINOv2 source and weights required')
        self.torch, self.Image, self.ImageOps = torch, Image, ImageOps
        torch.set_num_threads(4)
        self.model = torch.hub.load(str(source), 'dinov2_vits14', source='local', pretrained=False)
        self.model.load_state_dict(torch.load(weights, map_location='cpu', weights_only=True))
        self.model.eval()
        self.mean = torch.tensor([.485, .456, .406]).reshape(3, 1, 1)
        self.std = torch.tensor([.229, .224, .225]).reshape(3, 1, 1)

    def encode(self, crops):
        if not crops:
            return np.empty((0, self.model.embed_dim), dtype=np.float32)
        tensors = []
        for crop in crops:
            if crop.ndim != 3 or crop.shape[2] != 3 or not crop.size or crop.dtype != np.uint8:
                raise ValueError('nonempty BGR uint8 crop required')
            image = self.Image.fromarray(crop[:, :, ::-1])
            image = self.ImageOps.pad(image, (224, 224), method=self.Image.Resampling.BICUBIC,
                                      color=(127, 127, 127))
            tensor = self.torch.from_numpy(np.array(image)).permute(2, 0, 1).float()/255.
            tensors.append((tensor-self.mean)/self.std)
        # Bound memory even when a reference file contains many views.
        with self.torch.inference_mode():
            batches = [self.model(self.torch.stack(tensors[i:i+16])).cpu().numpy()
                       for i in range(0, len(tensors), 16)]
        return normalized(np.concatenate(batches))


def load_gallery(manifest, encoder, threshold=.7, margin=.05):
    import cv2
    manifest = Path(manifest)
    rows = json.loads(manifest.read_text())
    if not isinstance(rows, list) or not rows:
        raise ValueError('reference manifest must be a nonempty list')
    labels, crops = [], []
    for row in rows:
        if (not isinstance(row, dict) or not isinstance(row.get('label'), str)
                or not row['label'].strip() or not isinstance(row.get('image'), str)):
            raise ValueError('reference needs label and image')
        image = cv2.imread(str(manifest.parent/row['image']))
        if image is None:
            raise ValueError('reference image is unavailable')
        labels.append(row['label'])
        crops.append(image)
    return AppearanceGallery(labels, encoder.encode(crops), threshold, margin)
