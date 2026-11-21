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
        if len(self.identities) < 2:
            raise ValueError('at least two distinct reference labels are required for margin checking')
        self.reference_columns = tuple(
            np.flatnonzero(np.asarray(self.labels) == label) for label in self.identities
        )
        self.threshold, self.margin = threshold, margin

    def classify(self, features):
        values = normalized(features)
        if values.shape[1] != self.features.shape[1]:
            raise ValueError('encoder feature dimensions differ')
        similarities = values @ self.features.T
        # Compare the best view per identity; multiple views of one identity never compete.
        scores = np.column_stack([similarities[:, cols].max(axis=1) for cols in self.reference_columns])
        scores = scores.astype(np.float64)
        best = scores.argmax(axis=1)
        best_scores = scores[np.arange(len(scores)), best]
        margins = best_scores - np.partition(scores, -2, axis=1)[:, -2]
        accepted = (best_scores >= self.threshold) & (margins >= self.margin)
        return [
            {'label': self.identities[best[i]] if accepted[i] else None,
             'score': float(best_scores[i]), 'margin': float(margins[i]),
             'scores': dict(zip(self.identities, row.tolist()))}
            for i, row in enumerate(scores)
        ]

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

    def tensor(self, crop, size):
        if crop.ndim != 3 or crop.shape[2] != 3 or not crop.size or crop.dtype != np.uint8:
            raise ValueError('nonempty BGR uint8 crop required')
        image = self.Image.fromarray(crop[:, :, ::-1])
        image = self.ImageOps.pad(image, (size, size), method=self.Image.Resampling.BICUBIC,
                                  color=(127, 127, 127))
        tensor = self.torch.from_numpy(np.array(image)).permute(2, 0, 1).float()/255.
        return (tensor-self.mean)/self.std

    def patch_features(self, frame, size):
        with self.torch.inference_mode():
            features = self.model.forward_features(self.tensor(frame, size)[None])
        return normalized(features['x_norm_patchtokens'][0].cpu().numpy())

    def encode(self, crops):
        if not crops:
            return np.empty((0, self.model.embed_dim), dtype=np.float32)
        tensors = [self.tensor(crop, 224) for crop in crops]
        # Bound memory even when a reference file contains many views.
        with self.torch.inference_mode():
            batches = [self.model(self.torch.stack(tensors[i:i+16])).cpu().numpy()
                       for i in range(0, len(tensors), 16)]
        return normalized(np.concatenate(batches))


def reference_images(manifest):
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
    return labels, crops


def patch_boxes(labels, height, width):
    """Map DINO's padded 32x32 patch regions back to image coordinates."""
    import cv2
    if labels.shape != (32, 32) or height <= 0 or width <= 0:
        raise ValueError('32x32 patch labels and positive image dimensions required')
    scale = 448 / max(height, width)
    content_w, content_h = round(width*scale), round(height*scale)
    left, top = (448-content_w)//2, (448-content_h)//2
    boxes = []
    centers = np.arange(32)*14 + 7
    valid = ((centers[:,None] >= top) & (centers[:,None] < top+content_h)
             & (centers[None,:] >= left) & (centers[None,:] < left+content_w))
    for label in sorted(set(labels.ravel()) - {None}):
        mask = (labels == label).astype(np.uint8)
        mask[~valid] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for x, y, w, h, area in stats[1:]:
            if area < 6:
                continue
            x1 = max(0, int(((x-1)*14-left)/scale))
            y1 = max(0, int(((y-1)*14-top)/scale))
            x2 = min(width, int(((x+w+1)*14-left)/scale))
            y2 = min(height, int(((y+h+1)*14-top)/scale))
            if x2-x1 >= 32 and y2-y1 >= 64:
                boxes.append([x1,y1,x2,y2])
    # shortcut: region boxes are coarse; validate their geometry before distance control.
    return sorted(boxes, key=lambda b: (b[2]-b[0])*(b[3]-b[1]), reverse=True)[:12]


class AppearanceLocalizer:
    """Locate registered appearance with DINO patches, without an object detector."""
    def __init__(self, manifest, encoder, threshold=.7, margin=.05):
        labels, crops = reference_images(manifest)
        self.encoder = encoder
        self.gallery = AppearanceGallery(labels, encoder.encode(crops), threshold, margin)
        pooled = [encoder.patch_features(crop, 224).mean(axis=0) for crop in crops]
        # Provisional patch limits, independent of the whole-crop comparison limits.
        self.patch_gallery = AppearanceGallery(labels, pooled, threshold=.55, margin=.05)

    def propose(self, frame):
        matches = self.patch_gallery.classify(self.encoder.patch_features(frame, 448))
        labels = np.array([row['label'] for row in matches], dtype=object).reshape(32,32)
        return patch_boxes(labels, *frame.shape[:2])
