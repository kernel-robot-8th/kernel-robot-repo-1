"""Offline recognition of known printed dolls; no robot or network output."""

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np


class DollRecognizer:
    """Match masked reference artwork, rejecting unsupported geometry."""

    def __init__(self, manifest):
        manifest = Path(manifest)
        self.sift = cv2.SIFT_create(nfeatures=1800)
        self.matcher = cv2.BFMatcher()
        self.references = []
        rows = json.loads(manifest.read_text())
        if not isinstance(rows, list) or not rows:
            raise ValueError('reference manifest must be a nonempty list')
        for row in rows:
            if (not isinstance(row, dict) or not all(
                    isinstance(row.get(key), str) and row[key]
                    for key in ('label', 'view', 'image', 'mask'))):
                raise ValueError('reference needs label, view, image and mask')
            if row['label'] not in ('A', 'B', 'C'):
                raise ValueError('label must be A, B or C')
            image = cv2.imread(str(manifest.parent / row['image']), 0)
            mask = cv2.imread(str(manifest.parent / row['mask']), 0)
            if image is None or mask is None or image.shape != mask.shape:
                raise ValueError('reference image and mask must exist and have equal dimensions')
            points, descriptors = self.sift.detectAndCompute(image, mask)
            if descriptors is None or len(points) < 12:
                raise ValueError('reference needs at least 12 features')
            self.references.append((row['label'], row['view'], image.shape, points, descriptors))

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        points, descriptors = self.sift.detectAndCompute(gray, None)
        if descriptors is None or len(points) < 12:
            return []
        found = []
        for label, view, (height, width), ref_points, ref_desc in self.references:
            pairs = self.matcher.knnMatch(ref_desc, descriptors, k=2)
            matches = [a for pair in pairs if len(pair) == 2
                       for a, b in [pair] if a.distance < .7 * b.distance]
            # A scene feature must not count as multiple independent observations.
            unique = {m.trainIdx: m for m in sorted(matches, key=lambda m: -m.distance)}
            matches = list(unique.values())
            if len(matches) < 12:
                continue
            src = np.float32([ref_points[m.queryIdx].pt for m in matches])
            dst = np.float32([points[m.trainIdx].pt for m in matches])
            matrix, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
            if matrix is None or inliers is None:
                continue
            keep = inliers.ravel().astype(bool)
            count = int(keep.sum())
            if count < 10 or count / len(matches) < .5:
                continue
            support = cv2.contourArea(cv2.convexHull(src[keep])) / (width * height)
            if support < .08:
                continue
            corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
            quad = cv2.perspectiveTransform(corners[None], matrix)[0]
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
                continue
            area = cv2.contourArea(quad, oriented=True)
            edges = np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)
            if (area < 400 or area > gray.size * 1.5 or min(edges) < 12
                    or max(edges) / min(edges) > 6):
                continue
            fh, fw = gray.shape
            if (quad[:, 0].max() < 0 or quad[:, 1].max() < 0
                    or quad[:, 0].min() >= fw or quad[:, 1].min() >= fh):
                continue
            found.append({'label': label, 'view': view, 'inliers': count,
                          'quad': quad.tolist()})
        # shortcut: one physical doll per class; duplicate prints require instance tracking.
        best = {}
        for detection in sorted(found, key=lambda d: d['inliers'], reverse=True):
            best.setdefault(detection['label'], detection)
        return list(best.values())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--references', required=True, type=Path)
    parser.add_argument('--video', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='new output directory')
    args = parser.parse_args(argv)
    if not args.video.is_file():
        parser.error('video must be an existing local file')
    recognizer = DollRecognizer(args.references)
    args.output.mkdir(parents=True, exist_ok=False)
    capture = cv2.VideoCapture(str(args.video))
    writer = None
    frame_count, timings = 0, []
    detected_frames = dict.fromkeys('ABC', 0)
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not capture.isOpened() or not np.isfinite(fps) or fps <= 0:
            raise ValueError('invalid video or FPS')
        size = tuple(int(capture.get(p)) for p in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT))
        writer = cv2.VideoWriter(str(args.output / 'preview.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
        if not writer.isOpened():
            raise RuntimeError('video writer unavailable')
        with (args.output / 'detections.json').open('w') as result:
            result.write('[')
            try:
                while True:
                    ok, frame = capture.read()
                    if not ok:
                        break
                    start = time.perf_counter()
                    detections = recognizer.detect(frame)
                    timings.append(time.perf_counter() - start)
                    if frame_count:
                        result.write(',')
                    result.write(json.dumps({'frame': frame_count, 'seconds': frame_count / fps,
                                             'detections': detections}))
                    frame_count += 1
                    for label in detected_frames:
                        detected_frames[label] += any(d['label'] == label for d in detections)
                    for detection in detections:
                        quad = np.int32(detection['quad'])
                        color = {'A': (60, 220, 60), 'B': (0, 180, 255), 'C': (255, 180, 70)}[detection['label']]
                        cv2.polylines(frame, [quad], True, color, 2)
                        x, y = np.maximum(quad.min(axis=0), [0, 40])
                        cv2.putText(frame, f"{detection['label']} {detection['view']} ({detection['inliers']} matches)",
                                    (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                    cv2.putText(frame, 'OFFLINE | A idol  B firefighter  C football | no robot output',
                                (5, 20), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
                    writer.write(frame)
            finally:
                result.write(']')
        if not frame_count:
            raise ValueError('video has no readable frames')
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    summary = {'frames': frame_count, 'source_fps': fps,
               'median_processing_ms': round(float(np.median(timings)) * 1000, 2),
               'detected_frames': detected_frames,
               'note': 'Counts are not accuracy; no ground-truth labels. Boxes cover reference artwork, not full body.'}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
