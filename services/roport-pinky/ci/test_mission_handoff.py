"""Dynamic handoff goals, including the actual Nav2 transport with a fake client."""
from concurrent.futures import Future
import ast
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import yaml


PINKY = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'mission_handoff_test', PINKY/'scripts/field_roundtrip/mission_handoff.py')
handoff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handoff)
sys.path.insert(0, str(PINKY/'src/roport_smartcart'))
from roport_smartcart.goals import map_hashes
from roport_smartcart.goals import load_destination
from roport_smartcart.nav_client import NavClient
from roport_smartcart.real_return import Acquisition


def state(x=.5, y=0., yaw=0.):
    return {'nav2_ready': True, 'localization_ready': True, 'stopped': True,
            'pose': {'x': x, 'y': y, 'yaw': yaw}}


class Client:
    def __init__(self, *args):
        self.goals = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return Future()


class MissionHandoffChecks(unittest.TestCase):
    def setUp(self):
        self.context = {'trip_id': 'a'*32, 'robot': 'test_robot', 'boot_id': 'test_boot',
                        'map_version': 'map_version', 'layout_sha256': 'layout_hash',
                        'places_sha256': 'places_hash', 'ir_sha256': 'ir_hash'}
        self.ir = {'node': True, 'lost': False, 'on_line': [True, True, True], 'error': .03}
        self.line = {'x': .53, 'y': 0.}
        self.entry = {'x': 1., 'y': 0., 'yaw': math.pi/2}

    def capture(self, observed=None, ir=None, age=.1):
        return handoff.capture(self.context, observed or state(), ir or self.ir,
                               age, .03, self.line, 0.)

    def test_sensor_point_is_preserved_after_reversing_body_heading(self):
        record = self.capture()
        pose = record['return_n12']
        self.assertAlmostEqual(pose['x'], .56)
        self.assertAlmostEqual(pose['yaw'], math.pi)
        self.assertAlmostEqual(pose['x']+.03*math.cos(pose['yaw']), .53)
        self.assertAlmostEqual(pose['y']+.03*math.sin(pose['yaw']), 0.)
        moved = self.capture(state(.51, .01, .02))
        self.assertNotEqual(moved['return_n12'], record['return_n12'])

    def test_invalid_physical_observations_are_not_recorded(self):
        for bad in ({**state(), 'stopped': False}, {**state(), 'localization_ready': False},
                    state(float('nan')), state(.7), state(yaw=.2)):
            with self.subTest(observation=bad), self.assertRaises(ValueError):
                self.capture(bad)
        for bad in ({**self.ir, 'node': False}, {**self.ir, 'lost': True},
                    {**self.ir, 'on_line': [False, True, True]}, {**self.ir, 'error': .5}):
            with self.subTest(ir=bad), self.assertRaises(ValueError):
                self.capture(ir=bad)
        with self.assertRaises(ValueError):
            self.capture(age=.31)

    def test_trip_boot_map_layout_robot_and_ir_geometry_are_bound(self):
        record = handoff.set_entry(self.capture(), state(1.01, .01, math.pi/2), self.entry)
        self.assertEqual(handoff.validate(record, self.context, .03, self.line, 0., self.entry), record)
        for key in self.context:
            changed = {**self.context, key: 'different'}
            with self.subTest(key=key), self.assertRaises(ValueError):
                handoff.validate(record, changed, .03, self.line, 0., self.entry)
        with self.assertRaises(ValueError):
            handoff.validate(record, self.context, .04, self.line, 0., self.entry)
        altered = copy.deepcopy(record)
        altered['return_n12']['x'] += .01
        with self.assertRaises(ValueError):
            handoff.validate(altered, self.context, .03, self.line, 0., self.entry)
        with self.assertRaises(ValueError):
            handoff.validate(self.capture(), self.context, .03, self.line, 0., self.entry)

    def test_dynamic_goal_reaches_nav2_and_original_field_file_is_unchanged(self):
        from builtin_interfaces.msg import Time
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'map.pgm').write_bytes(b'P5\n40 40\n255\n'+b'\xff'*1600)
            map_file = root/'map.yaml'
            map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, -1, 0]\n')
            base = root/'field.yaml'
            base.write_text(yaml.safe_dump({'environment': 'real',
                'map': {'frame': 'map', **map_hashes(map_file)},
                'places': {'return_n12': {'measured': True, 'pose': [9., 9., 0.]},
                           'return_entry': {'measured': True, 'pose': [8., 8., 0.]}}}))
            original = base.read_bytes()
            # Run the real field capture/assignment/entry callbacks, not a parallel implementation.
            tree = ast.parse((PINKY/'scripts/field_roundtrip/roundtrip.py').read_text())
            main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
            callbacks = [n for n in main.body if isinstance(n, ast.FunctionDef)
                         and n.name in ('activate_handoff', 'capture_n12', 'capture_entry')]
            wrapper = ast.parse('def bind():\n n12_record=None\n return_target=None\n return capture_n12,capture_entry\n').body[0]
            wrapper.body[2:2] = callbacks
            clock = [0.]
            options = NS(places=str(base), map=str(map_file))
            reading = NS(node=True, lost=False, error=.03, on_line=[True, True, True],
                         as_dict=lambda: copy.deepcopy(self.ir))
            runner = NS(owner='none', options=options, reading=reading, ir_at=0.,
                        require_guard=lambda: None, stop_confirmed=state,
                        line=NS(state='handoff', route=['H1', 'N09', 'N08', 'N12']))

            def now():
                clock[0] += .01
                return clock[0]

            def spin():
                clock[0] += .05
                runner.ir_at = clock[0]
                return state()

            namespace = {'ROOT': root, 'base_places': base, 'handoff_id': 'a'*32,
                         'mission_id': 'a'*32, 'handoff': handoff,
                         'handoff_context': lambda: self.context, 'ir_ahead': .03,
                         'line_hint': self.line, 'outbound_heading': 0.,
                         'expected_entry': self.entry, 'options': options, 'runner': runner,
                         'load_destination': load_destination, 'json': json, 'math': math,
                         'Acquisition': Acquisition, 'time': NS(monotonic=now), 'spin': spin,
                         'settle': lambda: state(1.01, .01, math.pi/2)}
            centered = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'centered')
            exec(compile(ast.Module(body=[centered, wrapper], type_ignores=[]), '<field callbacks>', 'exec'), namespace)
            capture_n12, capture_entry = namespace['bind']()
            capture_n12()
            runtime = Path(options.places)
            self.assertEqual(runtime, root/'runtime'/('a'*32+'-places.yaml'))
            self.assertAlmostEqual(runner.target.x, .56)
            self.assertNotIn('return_entry', yaml.safe_load(runtime.read_text())['places'])
            capture_entry()
            stamp = NS(to_msg=lambda: Time())
            node = NS(get_clock=lambda: NS(now=lambda: stamp))
            client = NavClient(node, action_name='/isolated_handoff/navigate_to_pose',
                               client_factory=Client, environment='real')
            client.start(runtime, map_file, 'return_n12')
            goal = client._client.goals[0].pose
            self.assertEqual(goal.header.frame_id, 'map')
            self.assertAlmostEqual(goal.pose.position.x, .56)
            self.assertAlmostEqual(goal.pose.position.y, 0.)
            self.assertAlmostEqual(goal.pose.orientation.z, 1.)
            self.assertEqual(base.read_bytes(), original)
            entry_client = NavClient(node, action_name='/isolated_handoff/navigate_to_pose',
                                     client_factory=Client, environment='real')
            entry_client.start(runtime, map_file, 'return_entry')
            self.assertAlmostEqual(entry_client._client.goals[0].pose.pose.position.x, 1.01)
            (root/'map.pgm').write_bytes(b'changed')
            blocked = NavClient(node, action_name='/isolated_handoff/navigate_to_pose',
                                client_factory=Client, environment='real')
            with self.assertRaises(ValueError):
                blocked.start(runtime, map_file, 'return_n12')
            self.assertEqual(blocked._client.goals, [])
            # An arbitrary bar while not in the N12 handoff state must not replace the record.
            runner.line.state = 'idle'
            unchanged = runtime.read_bytes()
            with self.assertRaisesRegex(RuntimeError, 'not stable'):
                capture_n12()
            self.assertEqual(runtime.read_bytes(), unchanged)


if __name__ == '__main__':
    unittest.main()
