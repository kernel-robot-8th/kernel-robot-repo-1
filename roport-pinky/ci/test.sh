#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
set -u
python3 -m unittest discover -s ci -p 'test_*.py' -v
# ARM-only drivers and the Gazebo stack require separate hardware/simulation jobs.
colcon build --packages-select roport_pinky pinky_interfaces pinky_description \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
set +u
source install/setup.bash
set -u
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import roport_pinky
import xacro
from ament_index_python.packages import get_package_share_directory
from pinky_interfaces.srv import Emotion, SetBrightness, SetLamp, SetLed

for service in (Emotion, SetBrightness, SetLamp, SetLed):
    service.Request()
    service.Response()
share = Path(get_package_share_directory('pinky_description'))
robot = ET.fromstring(xacro.process_file(str(share / 'urdf/robot.urdf.xacro')).toxml())
names = {joint.get('name') for joint in robot.findall('joint')}
assert {'l_wheel_joint', 'r_wheel_joint'} <= names, names
print('Built ROS services and expanded Pinky URDF successfully')
PY
