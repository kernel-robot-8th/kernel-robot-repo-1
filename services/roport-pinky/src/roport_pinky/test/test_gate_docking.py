"""Calibrated vision geometry and stop conditions; no robot or camera device."""
import copy
import math
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

from roport_pinky.gate_docking import DockPolicy, GateDocking


CONFIG = dict(version=1, field_verified=True, front_extent_m=.06, clearance_m=.05,
              image_topic='camera/image_raw/compressed', camera_info_topic='camera/camera_info',
              goals={'G2': {'marker_id': 9, 'marker_size_m': .1}})
COMMAND = dict(goal_id='G2', pose=dict(x=.32, y=0., yaw=0.),
               position_tolerance_m=.03, yaw_tolerance_rad=.1)


@pytest.mark.parametrize('key,value', [('field_verified', False), ('version', True),
    ('front_extent_m', math.nan), ('clearance_m', .001), ('goals', {}), ('image_topic', '')])
def test_unverified_or_unsafe_config_never_enables_motion(key, value):
    with pytest.raises(ValueError):
        DockPolicy({**CONFIG, key: value})


def test_front_clearance_includes_robot_extent_and_matches_control_target():
    policy = DockPolicy(CONFIG)
    marker = dict(x=.43, y=0., nx=-1., ny=0.)
    pose = dict(x=.15, y=0., yaw=0.)
    target = policy.target(marker, pose, COMMAND)
    assert target['x'] == pytest.approx(.32)
    assert .43-(target['x']+.06) == pytest.approx(.05)
    with pytest.raises(ValueError, match='control_target_mismatch'):
        policy.target(marker, pose, {**COMMAND, 'pose': dict(x=.2, y=0., yaw=0.)})
    with pytest.raises(ValueError, match='approach_outside_bounds'):
        policy.target(marker, dict(x=-1., y=0., yaw=0.), COMMAND)
    with pytest.raises(ValueError, match='invalid_plane'):
        policy.target({**marker, 'nx': .2}, pose, COMMAND)


def camera_rig():
    dock = object.__new__(GateDocking)
    dock.policy, dock.cv = DockPolicy(CONFIG), cv2
    dock.clock = 10.
    dock.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(dock.clock*1e9))))
    dock.info = NS(header=NS(frame_id='optical'), width=800, height=600,
                   distortion_model='plumb_bob', d=[0.]*5,
                   k=[600.,0.,400.,0.,600.,300.,0.,0.,1.])
    dock.requested_times = []
    def transform(parent, child, stamp):
        assert (parent, child) == ('map', 'optical')
        dock.requested_times.append(stamp.nanoseconds/1e9)
        return NS(transform=NS(translation=NS(x=.03,y=0.,z=.1), rotation=NS(x=-.5,y=.5,z=-.5,w=.5)))
    dock.tf = NS(lookup_transform=transform)
    dock.reset(copy.deepcopy(COMMAND))
    dock.begin(10.)
    return dock


def image(dock, marker_id=9, stamp=None):
    stamp = dock.clock if stamp is None else stamp
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.drawMarker(dictionary, marker_id, 150)
    canvas = np.full((600,800), 255, np.uint8)
    canvas[225:375,325:475] = marker
    _, encoded = cv2.imencode('.jpg', canvas)
    from builtin_interfaces.msg import Time
    sec = int(stamp)
    return NS(header=NS(frame_id='optical', stamp=Time(sec=sec,nanosec=round((stamp-sec)*1e9))), data=encoded.tobytes())


def test_real_aruco_detection_to_map_requires_new_stable_frames_and_exact_image_tf():
    dock = camera_rig()
    pose = dict(x=.15, y=0., yaw=0.)
    dock._process_image(image(dock))
    assert dock.target(pose, dock.clock) is None
    dock._process_image(image(dock))  # duplicate timestamp cannot satisfy acquisition
    assert len(dock.frames) == 1
    for stamp in [10.2,10.4]:
        dock.clock = stamp
        dock._process_image(image(dock))
    target = dock.target(pose, dock.clock)
    assert target['x'] == pytest.approx(.32, abs=.01)
    assert target['y'] == pytest.approx(0., abs=.01)
    assert abs(target['yaw']) < .03
    assert dock.requested_times == pytest.approx([10.,10.2,10.4])
    with pytest.raises(ValueError, match='marker_lost'):
        dock.target(pose, 10.91)


def test_wrong_marker_does_not_authorize_goal_and_missing_marker_times_out():
    dock = camera_rig()
    dock._process_image(image(dock, marker_id=8))
    assert dock.frames == []
    with pytest.raises(ValueError, match='marker_missing'):
        dock.target(dict(x=0.,y=0.,yaw=0.), 12.01)


def test_camera_size_intrinsics_stale_frame_and_tf_failure_are_closed():
    dock = camera_rig()
    dock._process_image(image(dock, stamp=9.))
    assert not dock.frames and dock.last_image is None
    dock.info.k[0] = 0.
    dock._process_image(image(dock))
    assert dock.error == 'gate_dock_camera_uncalibrated'
    dock = camera_rig()
    dock.tf.lookup_transform = lambda *a: (_ for _ in ()).throw(RuntimeError('no TF'))
    dock._process_image(image(dock))
    assert dock.error == 'gate_dock_transform_unavailable'


def test_image_waits_for_timestamp_tf_without_blocking_other_callbacks():
    from collections import deque
    dock = camera_rig()
    dock.pending = deque(maxlen=4)
    dock.tf.can_transform = lambda *a: False
    dock._image(image(dock))
    dock._consume()
    assert len(dock.pending) == 1 and dock.error is None
    dock.tf.can_transform = lambda *a: True
    dock._consume()
    assert not dock.pending and len(dock.frames) == 1
