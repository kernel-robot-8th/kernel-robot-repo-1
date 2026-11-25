from functools import partial
from http.server import ThreadingHTTPServer
from threading import Condition, RLock, Thread
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from roport_smartcart.fleet_appearance import FleetAppearance, FRAME_TTL, FrameHandler


class Shop:
    def __init__(self):
        self.lock = Condition()
        self.nav = SimpleNamespace(pending=None, handle=None, result=None, motion_requested=False,
                                   motion_at=100., cancelled=False, fault=None)
        self.track = SimpleNamespace(state='FOLLOWING')
        self.observed = self.odom_at = 100.
        self.packet = (100.,)
        self.armed = False
        self.reason = ''
        self.reset()
        self.calls = []

    def command(self, operation, body):
        self.calls.append((operation, body))
        if operation == 'capture':
            self.frozen = 'frame'
        else:
            self.selected = True

    def reset(self):
        self.frozen, self.selected, self.detecting = None, False, True

    def hold_motion(self, now):
        self.armed = self.nav.motion_requested = False

    def status(self, key):
        return dict(frame=self.frozen, detecting=self.detecting, boxes=[[.1, .2, .5, .8]])

    def image(self, frame, key):
        return b'\xff\xd8jpeg'

    def can_start(self, now):
        return self.selected

    def arm(self):
        self.armed = True

    def tick(self):
        pass


P = dict(follow_session_id='test-session', expected_boot_id='boot', binding_id='binding',
         map_id='airport_map', map_version='v16')


def registration(app, op, **kwargs):
    message = {'type': 'follow_registration.requested', 'payload': {**P, 'operation': op, **kwargs}}
    app.validate_registration(message['payload'])
    app.register(message)
    return message


def tick(app, **kwargs):
    return app.tick(**{'following': False, 'controller_ready': True, 'stopped': True, **kwargs})


def capture(app):
    command = registration(app, 'capture')
    assert tick(app) == (None, None)
    app.shop.detecting = False
    error, (original, phase, details) = tick(app)
    assert error is None and original is command and phase == 'completed'
    return details


def select(app, details, **kwargs):
    command = registration(app, 'select', frame_ref=details['frame_ref'], **kwargs)
    error, (original, phase, result) = tick(app)
    assert error is None and original is command and phase == 'completed'
    return {**P, **{k: result[k] for k in ('target_ref', 'perception_boot_id')}}


def test_capture_select_and_release_require_current_session_and_do_not_move():
    shop = Shop()
    app = FleetAppearance(shop, RLock(), clock=lambda: 100.)
    details = capture(app)
    assert app.image(details['image_ref']) == b'\xff\xd8jpeg'
    with pytest.raises(ValueError, match='follow_registration_expired'):
        app.validate_registration({**P, 'follow_session_id': 'other', 'operation': 'release'})
    with pytest.raises(ValueError, match='follow_frame_mismatch'):
        app.validate_registration({**P, 'operation': 'select', 'frame_ref': 'old'})
    target = select(app, details, candidate_id='candidate-0')
    assert shop.calls[-1][1]['index'] == 0
    assert app.image(details['image_ref']) is None and not shop.armed
    app.validate_start(target)
    assert not shop.armed
    registration(app, 'release')
    assert tick(app)[1][2]['cleared'] is True
    with pytest.raises(ValueError, match='follow_target_stale'):
        app.validate_start(target)


def test_capture_without_fresh_stop_and_expired_frame_are_rejected():
    shop = Shop()
    now = [100.]
    app = FleetAppearance(shop, RLock(), clock=lambda: now[0])
    registration(app, 'capture')
    assert tick(app, stopped=False)[1][1] == 'failed'
    assert shop.calls == []
    details = capture(app)
    now[0] += FRAME_TTL + .1
    assert app.image(details['image_ref']) is None
    with pytest.raises(ValueError, match='follow_registration_expired'):
        registration(app, 'select', frame_ref=details['frame_ref'], box=[.1, .2, .5, .8])


@pytest.mark.parametrize(('cause', 'failure'), [
    ('lost', 'follow_lost'), ('inference', 'follow_target_stale'),
    ('odometry', 'follow_status_stale'), ('video', 'follow_status_stale'),
])
def test_observation_and_loss_revoke_motion_before_return(cause, failure):
    shop = Shop()
    now = [100.]
    app = FleetAppearance(shop, RLock(), clock=lambda: now[0])
    target = select(app, capture(app), box=[.1, .2, .5, .8])
    with pytest.raises(ValueError, match='follow_target_stale'):
        app.start({**target, 'perception_boot_id': 'old-process'})
    app.start(target)
    assert shop.armed and not app.motion_requested
    shop.nav.motion_requested = True
    assert not app.motion_requested
    shop.nav.handle = SimpleNamespace(accepted=True)
    assert app.motion_requested
    if cause == 'lost':
        shop.track.state = 'SEARCHING'
    elif cause == 'inference':
        shop.observed = 99.
    elif cause == 'odometry':
        shop.odom_at = 99.
    else:
        shop.packet = None
    assert tick(app, following=True) == (failure, None)
    assert not app.motion_requested and not app.quiescent()
    shop.nav.handle = None
    assert app.quiescent()


def test_frame_endpoint_requires_token_and_serves_only_current_jpeg():
    app = FleetAppearance(Shop(), RLock(), clock=lambda: 100.)
    details = capture(app)
    token = 'test-token-for-follow-frames'
    handler = partial(FrameHandler, lookup=app.image, token=token)
    with ThreadingHTTPServer(('127.0.0.1', 0), handler) as server:
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f'http://127.0.0.1:{server.server_port}/frames/{details["image_ref"]}'
        try:
            with pytest.raises(HTTPError) as denied:
                urlopen(url, timeout=2)
            assert denied.value.code == 401
            request = Request(url, headers={'Authorization': f'Bearer {token}'})
            with urlopen(request, timeout=2) as response:
                assert response.headers['Content-Type'] == 'image/jpeg'
                assert response.headers['Cache-Control'] == 'no-store'
            registration(app, 'release')
            tick(app)
            with pytest.raises(HTTPError) as missing:
                urlopen(request, timeout=2)
            assert missing.value.code == 404
        finally:
            server.shutdown()
            thread.join(timeout=2)
