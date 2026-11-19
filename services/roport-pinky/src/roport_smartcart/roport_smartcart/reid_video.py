"""Offline DINO appearance search + Re-ID. No YOLO, ROS or robot commands."""
import argparse
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import time

import numpy as np

from .appearance_reid import AppearanceLocalizer, DinoEncoder


def analyze(frame, localizer, encoder, gallery, target):
    height, width = frame.shape[:2]
    boxes, crops = [], []
    for box in localizer.propose(frame):
        x1, y1, x2, y2 = map(int, box)
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        # shortcut: this PC trial requires >=32x64 px; validate smaller toys before distant following.
        if x2-x1 < 32 or y2-y1 < 64:
            continue
        boxes.append([x1, y1, x2, y2])
        crops.append(frame[y1:y2, x1:x2])
    matches = gallery.classify(encoder.encode(crops)) if crops else []
    return boxes, matches, gallery.select(target, matches)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--references', required=True, type=Path)
    parser.add_argument('--encoder-source', required=True, type=Path)
    parser.add_argument('--encoder-weights', required=True, type=Path)
    parser.add_argument('--video', required=True, type=Path)
    parser.add_argument('--target', required=True)
    parser.add_argument('--output', required=True, type=Path, help='new private output folder')
    parser.add_argument('--stride', type=int, default=5)
    parser.add_argument('--threshold', type=float, default=.7)
    parser.add_argument('--margin', type=float, default=.05)
    args = parser.parse_args(argv)
    if args.stride < 1 or not args.video.is_file() or not args.references.is_file():
        parser.error('existing local video, references and positive stride required')
    if args.output.exists():
        parser.error('output must be a new directory')
    import cv2
    encoder = DinoEncoder(args.encoder_source, args.encoder_weights)
    localizer = AppearanceLocalizer(args.references, encoder, args.threshold, args.margin)
    gallery = localizer.gallery
    if args.target not in gallery.identities:
        parser.error('target is not registered')
    capture = cv2.VideoCapture(str(args.video))
    writer = None
    frames = processed = reacquired = 0
    counts = Counter()
    total_time = 0.
    ever_seen = previously_seen = False
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        reported_count = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        declared = int(reported_count) if np.isfinite(reported_count) and reported_count >= 1 else None
        if not capture.isOpened() or not np.isfinite(fps) or fps <= 0:
            raise ValueError('video has invalid metadata')
        args.output.mkdir(parents=True, exist_ok=False)
        with (args.output/'frames.jsonl').open('w') as log:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                index = frames
                frames += 1
                if index % args.stride:
                    continue
                start = time.perf_counter()
                boxes, matches, selection = analyze(frame, localizer, encoder, gallery, args.target)
                elapsed = time.perf_counter()-start
                total_time += elapsed
                state = selection['state']
                if state == 'MATCH' and ever_seen and not previously_seen:
                    state = 'REACQUIRED'
                    reacquired += 1
                previously_seen = selection['state'] == 'MATCH'
                ever_seen |= previously_seen
                counts[state] += 1
                processed += 1
                row = {'frame': index, 'seconds': index/fps, 'state': state,
                       'selected': selection['index'], 'boxes': boxes, 'matches': matches,
                       'processing_ms': round(elapsed*1000, 1)}
                log.write(json.dumps(row, allow_nan=False)+'\n')
                for i, (box, match) in enumerate(zip(boxes, matches)):
                    selected = i == selection['index']
                    color = (40, 220, 40) if selected else (0, 170, 255)
                    x1, y1, x2, y2 = box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = match['label'] or 'unknown'
                    cv2.putText(frame, f'{label} {match["score"]:.2f}', (x1,max(55,y1-6)),
                                cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                cv2.putText(frame, f'OFFLINE | {args.target} | {state} | {index/fps:.1f}s',
                            (8,25), cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2)
                if writer is None:
                    writer = cv2.VideoWriter(str(args.output/'preview.mp4'),
                        cv2.VideoWriter_fourcc(*'mp4v'), fps/args.stride,
                        (frame.shape[1], frame.shape[0]))
                    if not writer.isOpened():
                        raise RuntimeError('MP4 writer unavailable')
                writer.write(frame)
        if not processed:
            raise ValueError('video has no processed frames')
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    warnings = []
    if declared is None:
        warnings.append('Declared frame count unavailable; inspect decoded coverage.')
    elif frames != declared:
        warnings.append('Decoded frame count differs from metadata; verify the input for truncation or inaccurate metadata.')
    summary = {'target': args.target, 'frames': frames, 'processed': processed,
               'declared_frames': declared, 'warnings': warnings,
               'states': dict(counts), 'reacquisitions': reacquired,
               'mean_processing_ms': round(total_time/processed*1000, 1),
               'threshold': gallery.threshold, 'margin': gallery.margin,
               'reference_manifest_sha256': sha256(args.references.read_bytes()).hexdigest(),
               'encoder_weights_sha256': sha256(args.encoder_weights.read_bytes()).hexdigest(),
               'localizer': 'DINOv2 patch appearance search; no YOLO',
               'note': 'State counts are NOT accuracy. No frame ground truth. No robot output.'}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
