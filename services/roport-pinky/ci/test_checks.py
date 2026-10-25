"""Synthetic sensor checks; no ROS publishers, hardware or motor commands."""

import ast
import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "pinky_probe", ROOT / "scripts/check_hardware.py"
)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def message(kind, tick=1):
    header = NS(frame_id="test_frame", stamp=NS(sec=tick, nanosec=0))
    if kind == "joint_states":
        return NS(
            header=header,
            name=["left_wheel_joint", "right_wheel_joint"],
            position=[0.0, 0.0],
            velocity=[0.0, 0.0],
        )
    if kind == "scan":
        return NS(
            header=header,
            range_min=0.05,
            range_max=12.0,
            angle_increment=0.01,
            ranges=[1.0, math.inf],
        )
    return NS(
        header=header,
        child_frame_id="base_footprint",
        pose=NS(
            pose=NS(
                position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0, y=0, z=0, w=1)
            )
        ),
        twist=NS(twist=NS(linear=NS(x=0.0), angular=NS(z=0.0))),
    )


class SourceChecks(unittest.TestCase):
    def test_python_and_xml(self):
        source = ROOT / "src/pinky_pro"
        python_files = list(source.rglob("*.py"))
        manifests = list(source.glob("*/package.xml"))
        self.assertTrue(python_files)
        self.assertTrue(manifests)
        names = [ET.parse(path).getroot().findtext("name") for path in manifests]
        self.assertEqual(len(set(names)), len(names))
        for path in python_files:
            ast.parse(path.read_bytes(), filename=str(path))
        for path in list(source.rglob("*.xml")) + list(source.rglob("*.xacro")):
            ET.parse(path)


class SampleChecks(unittest.TestCase):
    def test_requires_samples_from_every_topic(self):
        state = probe.Samples(3)
        for tick in range(1, 4):
            for kind in state.topics:
                state.receive(kind, message(kind, tick))
        self.assertTrue(state.ok)
        missing = probe.Samples(3)
        for tick in range(1, 4):
            for kind in ("odom", "joint_states"):
                missing.receive(kind, message(kind, tick))
        self.assertFalse(missing.ok)

    def test_repeated_timestamp_fails(self):
        state = probe.Samples(2)
        state.receive("odom", message("odom"))
        state.receive("odom", message("odom"))
        self.assertEqual(state.topics["odom"]["samples"], 1)
        self.assertIn("timestamp", state.topics["odom"]["error"])

    def test_joint_frame_may_be_empty(self):
        msg = message("joint_states")
        msg.header.frame_id = ""
        probe.validate("joint_states", msg)

    def test_invalid_state_and_scan_fail(self):
        bad_messages = [
            ("odom", message("odom")),
            ("joint_states", message("joint_states")),
            ("scan", message("scan")),
        ]
        bad_messages[0][1].pose.pose.position.x = math.nan
        bad_messages[1][1].position = [0.0]
        bad_messages[2][1].ranges = [math.nan]
        for kind, msg in bad_messages:
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                probe.validate(kind, msg)

    def test_infinite_no_return_scan_is_valid(self):
        msg = message("scan")
        msg.ranges = [math.inf] * 3
        probe.validate("scan", msg)


if __name__ == "__main__":
    unittest.main()
