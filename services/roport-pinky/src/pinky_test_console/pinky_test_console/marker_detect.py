"""ArUco 마커 인식 확인 (ROS 의존성 없음, 단위 테스트 대상).

카메라 입력원(camera 파라미터):
  picamera2      로봇 카메라 (Picamera2, 이 입력원을 고를 때만 import)
  v4l2:<번호>    USB 웹캠 등 (/dev/video<번호>, 노트북 시험용)
  topic:<이름>   ROS sensor_msgs/Image 토픽 (노드가 frame 을 넣어 준다)

마커 표(markers.yaml, 저장소 docs/aruco)는 선택이다. 주면 ID 옆에 이름·크기를 보여 주고
아직 못 본 마커 목록을 만든다.
"""

import math
import re
import threading
import time
from collections import deque

import cv2
import numpy as np

DEFAULT_DICT = 'DICT_4X4_50'
STATS_WINDOW_SEC = 5.0
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
JPEG_QUALITY = 70
_SPEC_RE = re.compile(r'^(picamera2|v4l2:\d{1,2}|topic:/?[A-Za-z_][A-Za-z0-9_/]{0,100})$')


class MarkerError(ValueError):
    pass


def parse_camera_spec(spec):
    """'picamera2' | 'v4l2:0' | 'topic:/camera/image_raw' -> (kind, arg)."""
    s = str(spec).strip()
    if not _SPEC_RE.fullmatch(s):
        raise MarkerError(f"camera must be picamera2 | v4l2:<n> | topic:<name>, got '{spec}'")
    kind, _, arg = s.partition(':')
    return kind, (int(arg) if kind == 'v4l2' else arg or None)


def load_marker_table(path):
    """markers.yaml -> (사전 이름, {id: {name, size_mm, use}}). path 가 비면 (기본 사전, {})."""
    if not path:
        return DEFAULT_DICT, {}
    import yaml
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    table = {int(m['id']): {'name': str(m['name']), 'size_mm': float(m['size_mm']),
                            'use': str(m.get('use', ''))}
             for m in data.get('markers', [])}
    test = data.get('range_test')
    if test and int(test['id']) not in table:
        table[int(test['id'])] = {'name': 'TEST', 'size_mm': 0.0, 'use': 'range test'}
    return str(data.get('dictionary', DEFAULT_DICT)), table


def estimate_distance_m(side_px, image_width_px, hfov_deg, size_mm):
    """핀홀 근사: 거리 = f_px * 마커 크기 / 화면상 크기. 보정 전 참고값."""
    if side_px <= 0 or hfov_deg <= 0 or size_mm <= 0:
        return None
    f_px = (image_width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return f_px * (size_mm / 1000.0) / side_px


class MarkerDetector:
    def __init__(self, dict_name=DEFAULT_DICT):
        if not hasattr(cv2.aruco, dict_name):
            raise MarkerError(f'unknown ArUco dictionary: {dict_name}')
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, 'ArucoDetector'):     # OpenCV 4.7+
            self._detector = cv2.aruco.ArucoDetector(self.dictionary)
        else:                                         # Ubuntu 24.04 apt = 4.6
            self._detector = None

    def detect(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        out = []
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            q = c.reshape(4, 2)
            side = float(np.mean([np.linalg.norm(q[k] - q[(k + 1) % 4]) for k in range(4)]))
            center = [round(float(q[:, 0].mean()), 1), round(float(q[:, 1].mean()), 1)]
            out.append({'id': int(i), 'corners': q.tolist(), 'side_px': round(side, 1),
                        'center': center})
        return out


def draw_overlay(frame, detections, table):
    for d in detections:
        q = np.array(d['corners'], dtype=np.int32)
        known = d['id'] in table
        color = (0, 200, 0) if known else (0, 0, 255)
        cv2.polylines(frame, [q], True, color, 2)
        cv2.circle(frame, tuple(q[0]), 4, (255, 0, 255), -1)   # 마커 왼쪽 위 모서리
        name = table[d['id']]['name'] if known else '?'
        x, y = int(q[:, 0].min()), int(q[:, 1].min()) - 6
        cv2.putText(frame, f"{d['id']} {name}", (x, max(14, y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame


class PiCameraSource:
    def __init__(self, size=(FRAME_WIDTH, FRAME_HEIGHT)):
        from picamera2 import Picamera2   # 로봇에서만 필요
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_video_configuration(
            main={'size': size, 'format': 'RGB888'}))
        self.camera.start()

    def read(self):
        return cv2.cvtColor(self.camera.capture_array(), cv2.COLOR_RGB2BGR)

    def close(self):
        try:
            self.camera.stop()
            self.camera.close()
        except Exception:  # noqa: BLE001 - 종료 중
            pass


class V4l2Source:
    def __init__(self, index, size=(FRAME_WIDTH, FRAME_HEIGHT)):
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise MarkerError(f'cannot open /dev/video{index}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        self.cap.release()


class PushSource:
    """ROS 토픽용: 노드 콜백이 push() 하고 worker 가 read() 로 최신 영상을 가져간다."""

    def __init__(self, timeout=1.0):
        self._cv = threading.Condition()
        self._frame = None
        self._timeout = timeout

    def push(self, frame):
        with self._cv:
            self._frame = frame
            self._cv.notify()

    def read(self):
        with self._cv:
            if self._frame is None:
                self._cv.wait(self._timeout)
            frame, self._frame = self._frame, None
        return frame

    def close(self):
        self.push(None)


def ros_image_to_bgr(height, width, encoding, step, data):
    """sensor_msgs/Image -> BGR 배열 (cv_bridge 없이)."""
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, 'bgra8': 4, 'rgba8': 4}.get(encoding)
    if channels is None:
        raise MarkerError(f'unsupported image encoding: {encoding}')
    buf = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    img = buf[:, :width * channels].reshape(height, width, channels) if channels > 1 \
        else buf[:, :width]
    conv = {'rgb8': cv2.COLOR_RGB2BGR, 'mono8': cv2.COLOR_GRAY2BGR,
            'bgra8': cv2.COLOR_BGRA2BGR, 'rgba8': cv2.COLOR_RGBA2BGR}.get(encoding)
    return cv2.cvtColor(img, conv) if conv is not None else img.copy()


class MarkerWorker:
    """카메라를 읽어 인식하고 최신 JPEG·통계를 보관하는 스레드."""

    def __init__(self, source, detector, table=None, rotate_180=False, hfov_deg=0.0,
                 clock=time.monotonic):
        self.source, self.detector = source, detector
        self.table = table or {}
        self.rotate_180, self.hfov_deg = rotate_180, float(hfov_deg)
        self._clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._jpeg = None
        self._frame_times = deque()
        self._last = []                 # 최근 프레임 인식 결과
        self._frame_size = (0, 0)
        self._seen = {}                 # id -> {first, last, count}
        self._hits = deque()            # (t, id)
        self.error = None

    # ---------------- 처리 ----------------
    def process(self, frame):
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        now = self._clock()
        dets = self.detector.detect(frame)
        h, w = frame.shape[:2]
        for d in dets:
            info = self.table.get(d['id'])
            d['name'] = info['name'] if info else None
            d['size_mm'] = info['size_mm'] if info else None
            d['distance_m'] = None
            if info and info['size_mm'] > 0:
                dist = estimate_distance_m(d['side_px'], w, self.hfov_deg, info['size_mm'])
                d['distance_m'] = None if dist is None else round(dist, 2)
        jpeg = cv2.imencode('.jpg', draw_overlay(frame, dets, self.table),
                            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])[1].tobytes()
        with self._lock:
            self._jpeg = jpeg
            self._frame_size = (w, h)
            self._last = dets
            self._frame_times.append(now)
            for d in dets:
                s = self._seen.setdefault(d['id'], {'first': now, 'count': 0})
                s['last'] = now
                s['count'] += 1
                self._hits.append((now, d['id']))
            cutoff = now - STATS_WINDOW_SEC
            while self._frame_times and self._frame_times[0] < cutoff:
                self._frame_times.popleft()
            while self._hits and self._hits[0][0] < cutoff:
                self._hits.popleft()
        return dets

    def _run(self):
        while not self._stop.is_set():
            try:
                frame = self.source.read()
            except Exception as e:  # noqa: BLE001 - 카메라 오류는 화면에 표시
                self.error = f'camera read failed: {e}'
                time.sleep(0.5)
                continue
            if frame is None:
                continue
            self.error = None
            self.process(frame)

    def start(self):
        self._thread = threading.Thread(target=self._run, name='marker_worker', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self.source.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def reset_seen(self):
        with self._lock:
            self._seen.clear()
            self._hits.clear()

    # ---------------- 조회 ----------------
    def jpeg(self):
        with self._lock:
            return self._jpeg

    def snapshot(self):
        now = self._clock()
        with self._lock:
            frames = len(self._frame_times)
            span = (self._frame_times[-1] - self._frame_times[0]) if frames > 1 else 0.0
            hits = {}
            for _, i in self._hits:
                hits[i] = hits.get(i, 0) + 1
            seen = {i: dict(s) for i, s in self._seen.items()}
            last = [dict(d, corners=None) for d in self._last]
            w, h = self._frame_size
        fps = round((frames - 1) / span, 1) if span > 0 else None
        checklist = []
        for i in sorted(set(self.table) | set(seen)):
            info = self.table.get(i, {})
            s = seen.get(i)
            checklist.append({
                'id': i, 'name': info.get('name'), 'size_mm': info.get('size_mm'),
                'use': info.get('use'), 'in_table': i in self.table,
                'seen': s is not None,
                'ago_sec': round(now - s['last'], 1) if s else None,
                'rate': round(hits.get(i, 0) / frames, 2) if frames else 0.0,
            })
        return {'fps': fps, 'frame': [w, h], 'error': self.error, 'hfov_deg': self.hfov_deg,
                'window_sec': STATS_WINDOW_SEC, 'detections': last, 'checklist': checklist}
