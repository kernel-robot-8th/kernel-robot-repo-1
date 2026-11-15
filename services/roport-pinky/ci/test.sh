#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
set -u
python3 -m unittest discover -s ci -p 'test_*.py' -v
# Build Python console/world resources too; smartcart launch depends on them.
# Exclude Gazebo discovery so this no-simulator job does not require its C++ stack.
colcon build --packages-ignore pinky_gz_sim \
  --packages-select roport_range_layer roport_pinky roport_pinky_line roport_line_nav roport_smartcart pinky_interfaces pinky_description pinky_navigation pinky_test_console roport_gz_world \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
set +u
source install/setup.bash
set -u
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import roport_pinky
import roport_smartcart
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
# Line follower logic and node tests (isolated ROS_DOMAIN_ID/namespace, no hardware).
python3 -m pytest src/roport_pinky_line/test -q -p no:cacheprovider
# Node-route navigation: route planning, 2D IR/odom simulator routes, node tests (isolated domain).
python3 -m pytest src/roport_line_nav/test -q -p no:cacheprovider
# 관제 trial-v1 판정부·ROS Nav2 모의 피어·독립 watchdog (실장비·MQTT 없음).
python3 -m pytest src/roport_pinky/test -q -p no:cacheprovider
python3 -m pytest -q src/roport_smartcart/test
