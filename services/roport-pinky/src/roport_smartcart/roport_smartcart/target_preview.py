"""Optional PC YOLO preview. Deliberately has no ROS/MQTT/motor output."""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import threading
import time

from .target_tracking import Detection, TargetSelection


@dataclass(frozen=True)
class FrameResult:
    stamp: float
    frame: object
    detections: tuple[Detection, ...]


class VideoWorker:
    """One capture/inference worker; the UI independently expires old results."""

    def __init__(self, cv2, model, source, classes, confidence):
        self.cv2, self.model, self.source = cv2, model, source
        self.classes, self.confidence = classes, confidence
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def snapshot(self):
        with self.lock:
            return self.latest, self.error

    def run(self):
        capture = None
        try:
            capture = self.cv2.VideoCapture(self.source)
            if not capture.isOpened():
                raise RuntimeError('Cannot open source')
            is_file = isinstance(self.source, str) and Path(self.source).is_file()
            fps = capture.get(self.cv2.CAP_PROP_FPS)
            period = 1 / fps if is_file and 0 < fps <= 240 else 0
            while not self.stop.is_set():
                stamp = time.monotonic()
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError('End of video / source disconnected')
                results = self.model.track(
                    frame, persist=True, tracker='bytetrack.yaml',
                    classes=self.classes, conf=self.confidence, verbose=False,
                    device='cpu',
                )
                boxes = results[0].boxes
                detections = ()
                if boxes is not None and boxes.id is not None:
                    detections = tuple(
                        Detection(int(track_id), tuple(map(float, box)), float(conf), int(cls))
                        for box, track_id, conf, cls in zip(
                            boxes.xyxy.cpu().tolist(), boxes.id.cpu().tolist(),
                            boxes.conf.cpu().tolist(), boxes.cls.cpu().tolist())
                    )
                with self.lock:
                    self.latest = FrameResult(stamp, frame, detections)
                self.stop.wait(max(0, period - (time.monotonic() - stamp)))
        except Exception as exc:
            # Source URLs may contain credentials; never display exception text.
            with self.lock:
                self.error = f'SOURCE / DETECTOR ERROR ({type(exc).__name__})'
        finally:
            if capture is not None:
                capture.release()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument('--source', help='camera index, video path or non-secret stream URL')
    sources.add_argument('--source-env', metavar='NAME',
                         help='environment variable containing the source; value is not logged')
    parser.add_argument('--model', required=True, type=Path, help='existing local YOLO .pt weights')
    parser.add_argument('--class-id', type=int, default=0, help='COCO person=0; custom model class ID')
    parser.add_argument('--confidence', type=float, default=0.35)
    parser.add_argument('--max-age', type=float, default=0.5, help='maximum capture age in seconds')
    args = parser.parse_args(argv)
    source = os.environ.get(args.source_env) if args.source_env is not None else args.source
    if source is None or not source.strip():
        parser.error('source must be nonempty; --source-env must name a populated environment variable')
    if (not args.model.is_file() or args.model.suffix != '.pt'
            or args.class_id < 0 or not 0 < args.confidence <= 1
            or not 0 < args.max_age <= 2):
        parser.error('existing .pt model, nonnegative class, confidence (0,1], max-age (0,2] required')
    args.source = int(source) if source.isdecimal() else source
    return parser, args


def main(argv=None):
    parser, args = parse_args(argv)
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
    except ImportError:
        parser.exit(2, 'Install the optional preview dependencies; see README.\n')
    model = YOLO(str(args.model), task='detect')
    if args.class_id not in model.names:
        parser.error('class ID is absent from this model')
    worker = VideoWorker(cv2, model, args.source, [args.class_id], args.confidence)
    selection = TargetSelection(args.max_age)
    window = 'Pinky target preview - NO ROBOT OUTPUT'
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    displayed = None
    scale = 1.0

    def click(event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN and displayed is not None:
            _, error = worker.snapshot()
            if error:
                selection.clear(error)
            else:
                selection.select(displayed.detections, x / scale, y / scale,
                                 displayed.stamp, time.monotonic())

    cv2.setMouseCallback(window, click)
    worker.thread.start()
    try:
        while True:
            latest, error = worker.snapshot()
            now = time.monotonic()
            if latest is not None:
                selection.update(latest.detections, latest.stamp, now)
                image = latest.frame.copy()
                scale = min(1.0, 1100 / image.shape[1], 750 / image.shape[0])
                image = cv2.resize(image, None, fx=scale, fy=scale)
                displayed = latest
            else:
                image = np.zeros((480, 800, 3), dtype=np.uint8)
            if error:
                selection.clear(error + ' - RESTART SOURCE')
            fresh = latest is not None and selection.fresh(latest.stamp, now) and not error
            if fresh:
                for detection in latest.detections:
                    x1, y1, x2, y2 = [round(v * scale) for v in detection.box]
                    selected = selection.target is not None and (
                        selection.target.track_id == detection.track_id)
                    color = (0, 220, 0) if selected else (0, 190, 255)
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(image, f'ID {detection.track_id} {detection.confidence:.2f}',
                                (x1, max(65, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
            else:
                displayed = None
            cv2.rectangle(image, (0, 0), (image.shape[1], 55), (0, 0, 0), -1)
            cv2.putText(image, selection.reason, (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
            age = '--' if latest is None else f'{now - latest.stamp:.2f}s'
            cv2.putText(image, f'PREVIEW ONLY | age {age} | click: select | R: reset | Q: quit',
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
            cv2.imshow(window, image)
            key = cv2.waitKey(20) & 0xff
            if key in (ord('q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord('r'):
                selection.clear()
    finally:
        worker.stop.set()
        worker.thread.join(timeout=1)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
