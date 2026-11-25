"""Use the onboard shopping tracker under the Fleet driver's single drive lease."""
from datetime import UTC, datetime, timedelta
from functools import partial
from hmac import compare_digest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import re
import ssl
from threading import Thread
import time
from uuid import uuid4

FRAME_TTL = 15.
TARGET_TTL = 300.
MAX_IMAGE_BYTES = 2 * 1024 * 1024
REGISTRATION = 'follow_registration.requested'
SCOPE_KEYS = ('follow_session_id', 'binding_id', 'expected_boot_id', 'map_id', 'map_version')


class FrameHandler(BaseHTTPRequestHandler):
    def __init__(self, *args, lookup, token, **kwargs):
        self.lookup = lookup
        self.authorization = ('Bearer ' + token).encode()
        super().__init__(*args, **kwargs)

    def log_message(self, *_):
        pass

    def setup(self):
        self.request.settimeout(3)
        super().setup()

    def do_GET(self):
        if not compare_digest(self.headers.get('Authorization', '').encode(), self.authorization):
            return self.reply(401)
        match = re.fullmatch(r'/frames/([a-f0-9]{32})', self.path)
        image = self.lookup(match[1]) if match else None
        return self.reply(200, image) if image is not None else self.reply(404)

    def reply(self, status, image=b''):
        self.send_response(status)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        if image:
            self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(image)))
        self.end_headers()
        if image:
            self.wfile.write(image)


class FleetAppearance:
    """Registration methods run under TrialDriver.lock, before any Shopping lock."""

    def __init__(self, shop, lock, *, clock=time.monotonic):
        self.shop, self.lock, self.clock = shop, lock, clock
        self.perception_boot_id = uuid4().hex
        self.scope = self.key = self.frame = self.target = self.pending = None
        self.capture_started = False
        self.expires = self.deadline = 0.
        self.order = 0
        self.server = None

    @classmethod
    def from_env(cls, node, lock):
        config = os.environ.get('ROPORT_APPEARANCE_CONFIG')
        if not config:
            return None
        import cv2
        import yaml
        from sensor_msgs.msg import CameraInfo

        from .box_detector import ShoppingDetector
        from .shopping_follow import OnboardDinoEncoder, Shopping, ShoppingEncoder

        location = Path(config).expanduser().resolve()

        def local(name):
            return location.parent / Path(name).expanduser()

        data = yaml.safe_load(location.read_text())
        raw = yaml.safe_load(local(data['calibration']).read_text())
        if raw.get('optical_tf_verified') is not True or not data.get('optical_frame'):
            raise ValueError('appearance requires a verified optical transform')
        calibration = CameraInfo(height=raw['image_height'], width=raw['image_width'],
                                 k=raw['camera_matrix']['data'], d=raw['distortion_coefficients']['data'],
                                 distortion_model=raw['distortion_model'])
        frames = data['frames']
        token = os.environ.get('ROPORT_FOLLOW_FRAME_TOKEN', '')
        if len(token) < 24:
            raise ValueError('appearance frame token must be at least 24 characters')
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(local(frames['cert']), local(frames['key']))
        cv2.setNumThreads(1)
        detector = ShoppingDetector(local(data['detector_onnx']),
                                    {0: 'adult', 1: 'child', 2: 'human'}, confidence=.22)
        encoder = ShoppingEncoder(OnboardDinoEncoder(local(data['dinov2_onnx'])))
        shop = Shopping(node, detector, encoder, data['optical_frame'], calibration=calibration,
                        compressed=True, follow_distance_m=.26, external_lease=True)
        appearance = cls(shop, lock)
        try:
            handler = partial(FrameHandler, lookup=appearance.image, token=token)
            appearance.server = ThreadingHTTPServer((frames['host'], frames['port']), handler)
            appearance.server.socket = context.wrap_socket(
                appearance.server.socket, server_side=True, do_handshake_on_connect=False)
            appearance.server_thread = Thread(target=appearance.server.serve_forever, daemon=True)
            appearance.server_thread.start()
        except (OSError, ValueError):
            if appearance.server:
                appearance.server.server_close()
            shop.closed.set()
            shop.worker.join(timeout=2)
            raise
        return appearance

    @staticmethod
    def command_scope(payload):
        return tuple(payload[k] for k in SCOPE_KEYS)

    def live(self):
        return self.scope is not None and self.clock() < self.expires

    def quiescent(self):
        return self.shop.nav.quiescent()

    @property
    def motion_requested(self):
        nav = self.shop.nav
        return bool(self.shop.armed and self.live() and self.clock() < self.deadline
                    and nav.motion_requested and self.clock() - nav.motion_at < .2
                    and nav.handle is not None and nav.handle.accepted and not nav.cancelled)

    def expires_at(self):
        return (datetime.now(UTC) + timedelta(seconds=self.expires - self.clock())).isoformat()

    def image(self, ref):
        with self.lock:
            return self.frame.get('image') if self.live() and self.frame and self.frame['image_ref'] == ref else None

    def _send(self, op, **args):
        self.order += 1
        return self.shop.command(op, {'session': self.key, 'order': self.order, **args})

    def _clear(self):
        with self.shop.lock:
            self.shop.hold_motion(self.clock())
            self.shop.reset()
            self.shop.owner = None
        self.scope = self.key = self.frame = self.target = None
        self.expires = 0.

    def validate_registration(self, payload):
        requested = self.command_scope(payload)
        operation = payload['operation']
        if operation == 'capture':
            if self.live() and requested != self.scope:
                raise ValueError('follow_registration_busy')
            return
        if not self.live() or requested != self.scope:
            raise ValueError('follow_registration_expired')
        if operation == 'release':
            return
        if operation != 'select' or not self.frame or payload['frame_ref'] != self.frame['frame_ref']:
            raise ValueError('follow_frame_mismatch')
        if 'candidate_id' in payload and not any(
                c['candidate_id'] == payload['candidate_id'] for c in self.frame['candidates']):
            raise ValueError('follow_candidate_mismatch')

    def register(self, command):
        """Queue capture/selection; the sensor tick never waits for inference."""
        self.pending = command
        self.capture_started = False

    def _register_tick(self, stopped):
        command = self.pending
        payload = command['payload']
        self.validate_registration(payload)
        if not stopped:
            raise ValueError('fresh_stop_required')
        details = {'follow_session_id': payload['follow_session_id']}
        operation = payload['operation']
        if operation != 'release' and self.shop.reason:
            raise ValueError('follow_registration_unavailable')
        if operation == 'release':
            self._clear()
            details['cleared'] = True
        elif operation == 'capture':
            if not self.capture_started:
                self._clear()
                self.scope = self.command_scope(payload)
                self.key = uuid4().hex
                self.expires = self.clock() + FRAME_TTL
                self._send('capture')
                self.frame = {'image_ref': uuid4().hex}
                self.capture_started = True
                return None
            else:
                if not self.live():
                    raise ValueError('follow_registration_expired')
                status = self.shop.status(self.key)
                if status['detecting']:
                    return None
                frame_ref = status['frame']
                image = self.shop.image(frame_ref, self.key) if frame_ref else None
                if not image or len(image) > MAX_IMAGE_BYTES:
                    raise ValueError('follow_frame_unavailable')
                candidates = [
                    {'candidate_id': f'candidate-{i}', 'box': [float(x) for x in box]}
                    for i, box in enumerate(status['boxes'][:20])]
                self.frame.update(frame_ref=frame_ref, image=image, candidates=candidates)
                details.update(frame_ref=frame_ref, image_ref=self.frame['image_ref'], candidates=candidates,
                               expires_at=self.expires_at(), perception_boot_id=self.perception_boot_id)
        else:
            # Apply ROI only to the validated, still-current frozen frame.
            if 'box' in payload:
                selection = {'box': payload['box']}
            else:
                selection = {'index': next(i for i, candidate in enumerate(self.frame['candidates'])
                                           if candidate['candidate_id'] == payload['candidate_id'])}
            self._send('select', frame=payload['frame_ref'], **selection)
            self.frame = None
            self.target = uuid4().hex
            self.expires = self.clock() + TARGET_TTL
            details.update(target_ref=self.target, expires_at=self.expires_at(),
                           perception_boot_id=self.perception_boot_id)
        self.pending = None
        return command, 'completed', details

    def validate_start(self, payload):
        if (not self.live() or self.scope != self.command_scope(payload) or self.target is None
                or self.target != payload['target_ref'] or self.perception_boot_id != payload['perception_boot_id']):
            raise ValueError('follow_target_stale')
        with self.shop.lock:
            if not self.shop.can_start(self.clock()):
                raise ValueError('follow_not_tracking')

    def start(self, payload):
        with self.shop.lock:
            self.validate_start(payload)
            self.shop.arm()
        self.deadline = self.clock() + payload.get('max_duration_s', 240.)

    def stop(self, *, forget=False):
        self.pending = None
        if forget:
            self._clear()
            return
        with self.shop.lock:
            self.shop.hold_motion(self.clock())

    def tick(self, *, following, controller_ready, stopped):
        now = self.clock()
        error = None
        with self.shop.lock:
            if self.live() and controller_ready:
                self.shop.touch = now
            if following:
                packet = self.shop.packet
                if (not controller_ready or now - self.shop.odom_at > .5
                        or packet is None or now - packet[0] > .5):
                    error = 'follow_status_stale'
                elif not self.live() or now - self.shop.observed >= .6:
                    error = 'follow_target_stale'
                elif now >= self.deadline:
                    error = 'follow_max_duration'
                elif self.shop.track.state not in ('FOLLOWING', 'CLOSE'):
                    error = 'follow_lost'
                elif not self.shop.armed:
                    error = 'follow_status_stale' if not self.shop.nav.fault else 'follow_nav2_failed'
                if error:
                    self.shop.hold_motion(now)
            self.shop.tick()
            if following and self.shop.nav.response_failed:
                error = 'follow_action_response_failed'
            elif following and not self.shop.armed and not error:
                error = 'follow_nav2_failed' if self.shop.nav.fault else 'follow_status_stale'
        result = None
        if self.pending:
            try:
                if not controller_ready:
                    raise ValueError('follow_registration_unavailable')
                result = self._register_tick(stopped)
            except (ValueError, RuntimeError):
                result = self.pending, 'failed', {'error_code': 'follow_registration_failed'}
                self.pending = None
                self._clear()
        elif self.scope and not self.live():
            self._clear()
        return error, result

    def close(self):
        self.stop(forget=True)
        self.shop.closed.set()
        self.shop.worker.join(timeout=2)
        if self.server:
            self.server.shutdown()
            self.server.server_close()
