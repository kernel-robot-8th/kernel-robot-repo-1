"""Local session ROI identity. Injects models; never loads weights or accesses devices.

A user-selected crop is the sole immutable reference. Similar-looking copies cannot
be distinguished reliably by appearance alone; ambiguous candidates stop selection.
"""
import math
import numpy as np


def _frame(frame):
    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
        raise ValueError('uint8 BGR frame required')
    if min(frame.shape[:2]) < 1:
        raise ValueError('empty frame')
    return frame.shape[:2]


def _box(box, shape, normalized=False):
    try:
        values = np.asarray(box, dtype=float)
    except (ValueError, TypeError):
        raise ValueError('invalid box') from None
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError('finite xyxy box required')
    h, w = shape
    if normalized:
        if np.any(values < 0) or np.any(values > 1):
            raise ValueError('normalized box outside source frame')
        values = values * [w, h, w, h]
    x1, y1, x2, y2 = values
    if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
        raise ValueError('box outside source frame')
    result = [math.floor(x1), math.floor(y1), math.ceil(x2), math.ceil(y2)]
    if result[2]-result[0] < 32 or result[3]-result[1] < 64:
        raise ValueError('selected crop must be at least 32x64 pixels')
    return result


def _features(encoder, crops):
    values = np.asarray(encoder.encode(crops), dtype=float)
    if values.ndim != 2 or values.shape[0] != len(crops) or values.shape[1] < 1 or not np.isfinite(values).all():
        raise ValueError('invalid appearance features')
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    if not np.isfinite(norms).all() or np.any(norms <= 1e-12):
        raise ValueError('empty appearance features')
    return values / norms


def _csrt():
    import cv2
    params = cv2.TrackerCSRT_Params()
    params.template_size = 150
    return cv2.TrackerCSRT_create(params)


class SessionTracker:
    """Use a new instance for explicit re-registration, never mutate the reference.

    detector.detect(frame) returns records with pixel xyxy box; propose/callable
    returning boxes are also supported.
    The OpenCV tracker is used for a manual selection without a detector.
    """
    def __init__(self, encoder, detector, reference_frame, normalized_box, *,
                 tracker_factory=None, threshold=None, margin=.08, preview=False):
        minimum = .72 if detector is not None and not preview else .85
        threshold = minimum if threshold is None else threshold
        if not (np.isfinite(threshold) and minimum <= threshold <= 1 and
                np.isfinite(margin) and .08 <= margin <= 1):
            raise ValueError(f'threshold >= {minimum} and margin >= .08 required')
        self.encoder, self.detector = encoder, detector
        self.shape = _frame(reference_frame)
        self.threshold, self.margin = float(threshold), float(margin)
        self.tracker_factory = tracker_factory or _csrt
        self.preview = preview
        self.tracker = None
        self.last_preview_match = None
        self.reacquisition = None
        box = _box(normalized_box, self.shape, normalized=True)
        self.last_verified_box = box
        x1, y1, x2, y2 = box
        feature = _features(encoder, [reference_frame[y1:y2, x1:x2].copy()])[0]
        # Bytes-backed storage cannot be made writable again through setflags.
        self._reference = np.frombuffer(feature.tobytes(), dtype=feature.dtype)
        if self.detector is None:
            self._begin(reference_frame, box)

    @property
    def reference_feature(self):
        return self._reference.copy()

    def _begin(self, frame, box):
        self.tracker = None
        try:
            tracker = self.tracker_factory()
            x1, y1, x2, y2 = box
            if tracker.init(frame, (x1, y1, x2-x1, y2-y1)) is not False:
                self.tracker = tracker
        except Exception:
            # Detection can reacquire later; no accepted track on init failure.
            pass

    def _match(self, frame, boxes):
        if not boxes:
            self.last_preview_match = None
            return [], {'state': 'LOST', 'index': None}
        crops = [frame[y1:y2, x1:x2].copy() for x1, y1, x2, y2 in boxes]
        features = _features(self.encoder, crops)
        if features.shape[1] != self._reference.shape[0]:
            raise ValueError('appearance dimension changed')
        scores = np.clip(features @ self._reference, -1., 1.)
        previous = self.last_preview_match
        self.last_preview_match = None
        if previous is not None:
            previous_box, previous_feature = previous
            # Preview only: keep the frozen reference as an identity gate as size changes.
            for i, box in enumerate(boxes):
                if scores[i] >= .75 and self._overlaps(box, previous_box):
                    scores[i] = max(scores[i], min(1., float(features[i] @ previous_feature)))
        matches = [{'label': 'session' if s >= self.threshold else None,
                    'score': float(s)} for s in scores]
        order = np.argsort(-scores)
        best = int(order[0])
        if scores[best] < self.threshold:
            return matches, {'state': 'LOST', 'index': None}
        if len(order) > 1 and scores[best]-scores[order[1]] < self.margin:
            return matches, {'state': 'AMBIGUOUS', 'index': None}
        if self.preview:
            self.last_preview_match = boxes[best], features[best]
        return matches, {'state': 'MATCH', 'index': best}

    def _detections(self, frame):
        if hasattr(self.detector, 'detect'):
            proposed = [item['box'] for item in self.detector.detect(frame)]
        else:
            proposed = self.detector.propose(frame) if hasattr(self.detector, 'propose') else self.detector(frame)
        if len(proposed) > 100:
            raise ValueError('too many detection candidates')
        return [_box(box, self.shape) for box in proposed]

    @staticmethod
    def _overlaps(a, b):
        intersection = (max(0, min(a[2], b[2])-max(a[0], b[0])) *
                        max(0, min(a[3], b[3])-max(a[1], b[1])))
        union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
        return intersection >= .5 * union

    def step(self, frame, targetignored=None, *, near_anchor=None, expected_box=None, recent_confirmation=False):
        lost = {'state': 'LOST', 'index': None}
        try:
            if _frame(frame) != self.shape:
                raise ValueError('source frame geometry changed')
            if self.detector is not None:
                boxes = self._detections(frame)
                matches, selection = self._match(frame, boxes)
                index = selection['index']
                if index is None:
                    self.reacquisition = None
                    if selection['state'] == 'LOST':
                        if (not self.preview and recent_confirmation and len(boxes) == 1
                                and matches[0]['score'] >= self.threshold - .03
                                and expected_box is not None and self._overlaps(boxes[0], expected_box)
                                and near_anchor is not None and near_anchor(boxes[0])):
                            # Bridge a brief dip only with a fresh box and agreeing geometry.
                            return boxes, matches, {'state': 'MATCH', 'index': 0, 'continuity': True}
                        selection['reason'] = 'appearance_mismatch' if boxes else 'no_detection'
                    return boxes, matches, selection
                box = boxes[index]
                continuous = (expected_box is not None and self._overlaps(box, expected_box)
                              and near_anchor is not None and near_anchor(box))
                if not self.preview and not continuous and not self._overlaps(box, self.last_verified_box):
                    candidate = self.reacquisition
                    # A distant box must be a stronger, consistent match before it can replace the last position.
                    self.reacquisition = None
                    anchored = near_anchor is not None and near_anchor(box)
                    # At a verified map position allow a partly occluded shopper only
                    # after repeated matches with no competing candidate.
                    anchored_only = anchored and len(boxes) == 1
                    threshold = .76 if anchored_only else .81 if anchored else .86
                    required = 3
                    if anchored_only and recent_confirmation:
                        threshold, required = self.threshold, 1
                    elif anchored and not anchored_only and recent_confirmation and all(
                            matches[index]['score'] - matches[i]['score'] >= max(.12, self.margin)
                            and (matches[i]['score'] < self.threshold or not near_anchor(other))
                            for i, other in enumerate(boxes) if i != index):
                        # A distinct match can ignore background proposals only
                        # while confirmation at the known location is still recent.
                        threshold, required = .76, 1
                    if not anchored and len(boxes) == 1 and matches[index]['score'] < .86:
                        threshold, required = .83, 5
                    if matches[index]['score'] >= threshold:
                        count = candidate[1]+1 if candidate and (self._overlaps(box, candidate[0]) or anchored and candidate[2]) else 1
                        self.reacquisition = box, count, anchored
                    if self.reacquisition is None or self.reacquisition[1] < required:
                        return boxes, matches, {'state': 'LOST', 'index': None, 'reason': 'position_mismatch',
                                                'anchor_agrees': anchored}
                self.reacquisition = None
                self.last_verified_box = box
                return boxes, matches, selection
            if self.tracker is not None:
                tracker, self.tracker = self.tracker, None
                ok, xywh = tracker.update(frame)
                if not ok:
                    return [], [], lost
                x, y, w, h = xywh
                box = _box([x, y, x+w, y+h], self.shape)
                matches, selection = self._match(frame, [box])
                if selection['state'] == 'MATCH':
                    self.tracker = tracker
                return [box], matches, selection
            return [], [], lost
        except Exception:
            self.tracker = None
            self.last_preview_match = None
            self.reacquisition = None
            if self.detector is not None:
                lost['reason'] = 'processing_error'
            return [], [], lost
