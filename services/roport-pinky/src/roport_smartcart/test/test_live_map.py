"""Map identity and asynchronous proof expiry, independent of ROS discovery."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

from roport_smartcart.live_map import LiveMapCheck, matches_reference, reference_from_file


REFERENCE = ({'resolution': .05, 'origin': [0., 0., 0.]}, 2, 2, [0, 100, -1, 0])


def grid():
    return NS(header=NS(frame_id='map'), data=[0, 100, -1, 0],
        info=NS(width=2, height=2, resolution=.05,
                origin=NS(position=NS(x=0., y=0., z=0.),
                          orientation=NS(x=0., y=0., z=0., w=1.))))


@pytest.mark.parametrize('field', ['frame', 'size', 'resolution', 'origin', 'orientation',
                                  'nonplanar', 'nonfinite', 'cells'])
def test_runtime_grid_mismatch(field):
    message = grid()
    assert matches_reference(message, REFERENCE)
    if field == 'frame':
        message.header.frame_id = 'odom'
    elif field == 'size':
        message.info.width = 1
    elif field == 'resolution':
        message.info.resolution = .1
    elif field == 'origin':
        message.info.origin.position.x = .01
    elif field == 'orientation':
        message.info.origin.orientation.w = .9
    elif field == 'nonplanar':
        message.info.origin.orientation.x = .01
    elif field == 'nonfinite':
        message.info.origin.position.z = float('nan')
    else:
        message.data[0] = 100
    assert not matches_reference(message, REFERENCE)


def test_local_image_rows_and_static_map_timestamp_are_handled(tmp_path):
    (tmp_path / 'map.pgm').write_bytes(b'P5\n2 2\n255\n\x80\xff\xff\x00')
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: .05\norigin: [0, 0, 0]\n'
                    'negate: 0\noccupied_thresh: .65\nfree_thresh: .25\nmode: trinary\n')
    reference = reference_from_file(path)
    assert reference[3] == [0, 100, -1, 0]
    assert matches_reference(grid(), reference)  # No dynamic map header stamp needed.
    message = grid()
    message.info.origin.orientation.w = -1.
    assert matches_reference(message, reference)


class Client:
    ready = True

    def __init__(self):
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.requests.append(future)
        return future


def test_current_server_proof_requires_reply_and_revokes_mismatch_or_missing_service():
    wall = [0.]
    client = Client()
    check = LiveMapCheck(client, object, REFERENCE, clock=lambda: wall[0])
    assert check.failures()
    check.poll()
    assert check.failures()  # Sending a request is not proof.
    client.requests[-1].set_result(NS(map=grid()))
    check.poll()
    assert not check.failures()
    wall[0] = .9
    check.poll()
    assert len(client.requests) == 1
    wall[0] = 1.
    check.poll()
    bad = grid()
    bad.data[0] = 100
    client.requests[-1].set_result(NS(map=bad))
    check.poll()
    assert 'differs' in check.failures()[0]
    wall[0] = 2.
    check.poll()
    client.requests[-1].set_result(NS(map=grid()))
    check.poll()
    assert not check.failures()
    client.ready = False
    check.poll()
    assert 'unavailable' in check.failures()[0]


def test_new_goal_requires_new_reply_and_pending_timeout_never_reuses_old_proof():
    wall = [0.]
    client = Client()
    check = LiveMapCheck(client, object, REFERENCE, clock=lambda: wall[0])
    check.poll()
    client.requests[-1].set_result(NS(map=grid()))
    check.poll()
    assert not check.failures()
    check.recheck()
    assert check.failures()
    check.poll()
    pending = client.requests[-1]
    wall[0] = 2.
    check.poll()
    assert pending.cancelled() and check.failures()
    assert len(client.requests) == 3  # Timed out request replaced, never overlapped.
    client.requests[-1].set_exception(RuntimeError('transport'))
    check.poll()
    assert 'failed' in check.failures()[0]
    wall[0] = 3.
    check.poll()
    client.requests[-1].set_result(NS(map=grid()))
    check.poll()
    assert not check.failures()
    wall[0] = 5.1
    assert 'expired' in check.failures()[0]
