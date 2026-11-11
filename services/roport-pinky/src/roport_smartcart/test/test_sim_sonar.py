import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from roport_smartcart.sim_sonar import nearest_range


@pytest.mark.parametrize('rays,expected', [
    ([.6, .2, .5], .2), ([math.inf] * 9, .95),
    ([-math.inf, .3], .02), ([.02, .95], .02),
    ([], None), ([math.nan], None), ([.01], None), ([1.], None),
])
def test_ray_distance_conversion(rays, expected):
    assert nearest_range(rays, .02, .95) == expected


def test_sim_model_preserves_lidar_and_adds_lower_range_sensor():
    import xacro
    package = Path(__file__).parents[1]
    robot = ET.fromstring(xacro.process_file(str(package / 'urdf/pinky_sonar_sim.urdf.xacro'),
                                           mappings={'is_sim': 'true'}).toxml())
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}

    def height(link):
        z = 0.
        while link in joints:
            j = joints[link]
            z += float(j.find('origin').get('xyz').split()[2])
            link = j.find('parent').get('link')
        return z

    assert height('rplidar_link') == pytest.approx(.125)
    assert height('ultrasonic_link') == pytest.approx(.037)
    sensors = {s.get('name'): s for s in robot.findall('.//sensor')}
    assert 'gpu_lidar' in sensors and 'sonar_proxy' in sensors
    assert sensors['sonar_proxy'].findtext('gz_frame_id') == 'ultrasonic_link'
    assert float(sensors['sonar_proxy'].findtext('lidar/range/max')) == .95
