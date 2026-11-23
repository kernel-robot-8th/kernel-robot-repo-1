from threading import Condition
from types import SimpleNamespace as NS
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from tf2_ros import TransformException
from sensor_msgs.msg import CompressedImage, Image

from roport_smartcart.shopping_follow import Shopping


@pytest.mark.parametrize('malformed', ['empty_jpeg', 'short_buffer', 'short_step'])
def test_bad_camera_frame_does_not_replace_last_good_packet(malformed):
    shop = Shopping.__new__(Shopping)
    shop.received = lambda msg: 10.
    shop.info = NS(width=2, height=2)
    shop.optical_frame, shop.lock, shop.tf = 'optical', Condition(), Mock()
    old = shop.packet = object()
    if malformed == 'empty_jpeg':
        msg = CompressedImage(format='jpeg')
    else:
        step = 4 if malformed == 'short_step' else 6
        msg = Image(width=2, height=2, encoding='bgr8', step=step,
                    data=bytes(step*2 if malformed == 'short_step' else 5))
    msg.header.frame_id = 'camera'
    assert not shop.camera(msg)
    assert shop.packet is old

    pixels = np.full((2, 2, 3), 170, np.uint8)
    if malformed == 'empty_jpeg':
        msg.data = cv2.imencode('.jpg', pixels)[1].tobytes()
    else:
        msg.step = 8
        msg.data = (bytes([170]*6) + bytes(2))*2
    assert shop.camera(msg)
    np.testing.assert_array_equal(shop.packet[1], pixels)


@pytest.mark.parametrize('skew_ns, accepted', [(80_000_000, True), (80_000_001, False)])
def test_sensor_transform_keeps_nearest_fallback_limit(skew_ns, accepted):
    shop = Shopping.__new__(Shopping)
    shop.tf = Mock()
    recent = NS(header=NS(stamp=Time(sec=10, nanosec=skew_ns)), transform=object())
    shop.tf.lookup_transform.side_effect = [TransformException('late'), recent]
    assert shop.sensor_transform('sensor', Time(sec=10)) is (recent.transform if accepted else None)
