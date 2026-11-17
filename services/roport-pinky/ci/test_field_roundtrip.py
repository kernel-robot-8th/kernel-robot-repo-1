"""Exercise the actual field helpers without ROS, a camera or motor publishers."""
import ast
import hashlib
import importlib.util
import json
import math
import os
import re
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np


SOURCE = Path(__file__).resolve().parents[1]/'scripts/field_roundtrip/roundtrip.py'
TREE = ast.parse(SOURCE.read_text())
MAIN = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == 'main')


def helper(name, namespace, nested=True):
    body = MAIN.body if nested else TREE.body
    function = next(n for n in body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace[name]


class Clock:
    def __init__(self):
        self.value = 0.

    def monotonic(self):
        self.value += .1
        return self.value


class Goal:
    def __init__(self):
        self.target = NS()
        self.time_allowance = NS()


class FieldChecks(unittest.TestCase):
    def test_handoff_identity_rejects_field_asset_edits_during_a_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            assets = {key: root/key for key in ('layout_sha256', 'places_sha256', 'ir_sha256')}
            for file in assets.values():
                file.write_bytes(b'original')
            expected = {k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in assets.items()}
            (root/'map.yaml').write_bytes(b'map')
            (root/'map.pgm').write_bytes(b'image')
            profile = {'map_version': hashlib.sha256(b'mapimage').hexdigest()[:16]}
            ns = {'handoff_id': 'a'*32, 're': re, 'hashlib': hashlib, 'Path': Path,
                  'options': NS(robot='test_robot'), 'asset_paths': assets,
                  'asset_hashes': expected, 'path': root/'map.yaml',
                  'map_data': {'image': 'map.pgm'}, 'profile': profile}
            context = helper('handoff_context', ns)
            self.assertEqual(context()['trip_id'], 'a'*32)
            assets['ir_sha256'].write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError, 'assets changed'):
                context()
            assets['ir_sha256'].write_bytes(b'original')
            (root/'map.pgm').write_bytes(b'new map')
            with self.assertRaisesRegex(RuntimeError, 'assets changed'):
                context()

    def test_ui_only_releases_live_stopped_current_delivery_wait(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
                'ROPORT_FIELD_CONFIG_DIR': directory, 'ROPORT_FIELD_REMOTE_DIR': '/field',
                'ROPORT_FIELD_REMOTE_SETUP': '/field/setup.sh',
                'ROPORT_PINKY_SSH_HOST': 'robot@example.com',
                'ROPORT_PINKY_CONSOLE_URL': 'http://example.com:8080'}):
            spec = importlib.util.spec_from_file_location('field_ui_test', SOURCE.with_name('ui_server.py'))
            ui = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(ui)
            ui.latest = {'phase': 'WAIT_DELIVERY', 'mission_id': 'a'*32}
            ui.observed = ui.odom_seen = ui.guard_seen = time.monotonic()
            ui.guard = {'owner': 'none'}
            ui.odom = NS(twist=NS(twist=NS(linear=NS(x=0.), angular=NS(z=0.))))
            sent = []
            ui.remote = sent.append
            self.assertTrue(ui.snapshot()['delivery_enabled'])
            with self.assertRaises(ValueError):
                ui.command('/delivery-complete', {'mission_id': 'b'*32})
            self.assertEqual(sent, [])
            ui.command('/delivery-complete', {'mission_id': 'a'*32})
            self.assertEqual(len(sent), 1)
            self.assertIn('delivery_complete', sent[0])
            ui.odom.twist.twist.linear.x = .02
            self.assertFalse(ui.snapshot()['delivery_enabled'])
            ui.odom.twist.twist.linear.x = 0.
            ui.observed -= 3
            self.assertFalse(ui.snapshot()['delivery_enabled'])
            ui.observed = time.monotonic()
            ui.latest['phase'] = 'STOPPED'
            self.assertFalse(ui.snapshot()['delivery_enabled'])
            self.assertTrue(ui.snapshot()['restart_enabled'])
            ui.latest['phase'] = 'LINE'
            self.assertFalse(ui.snapshot()['restart_enabled'])
            with self.assertRaises(ValueError):
                ui.command('/restart-stack', {})
            self.assertEqual(len(sent), 1)
            ui.latest['phase'] = 'STOPPED'
            captured = []
            ui.remote = lambda script, **kwargs: captured.append((script, kwargs))
            ui.command('/restart-stack', {})
            self.assertEqual(captured[0][1]['timeout'], 60)
            self.assertIn('restart_stack.py', captured[0][0])
            self.assertIn('--restart', captured[0][0])
            self.assertNotIn('roundtrip.py --drive', captured[0][0])
            self.assertFalse(ui.restarting)
            ast.parse(ui.trial_launch_script())
            recovery = ui.trial_launch_script(n12=True, checkpoint='a'*32)
            ast.parse(recovery)
            self.assertIn('--resume-n12', recovery)
            self.assertIn('a'*32, recovery)
            ui.latest.update(direction='RETURN', error='N12 realignment exceeds 8cm', phase='STOPPED',
                             n12_handoff_id='a'*32, n12_return_pose={'x': 0., 'y': 0., 'yaw': 0.})
            self.assertTrue(ui.snapshot()['return_resume_enabled'])
            self.assertFalse(ui.snapshot()['start_enabled'])
            ui.command('/delivery-complete', {'mission_id': 'a'*32})
            self.assertIn('--resume-n12', captured[-1][0])

    def test_restart_only_manages_explicit_field_sessions(self):
        spec = importlib.util.spec_from_file_location('field_restart_test', SOURCE.with_name('restart_stack.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        valid = {kind: 'field_'+kind for kind in module.KINDS}
        module.validate_sessions(valid)
        for sessions in ({}, None, {**valid, 'bringup': 'other_project'},
                         {**valid, 'bringup': []}, {**valid, 'bringup': 'field_nav2'},
                         {**valid, 'nav2': 'field_nav2;untrusted'}):
            with self.subTest(sessions=sessions), self.assertRaises(ValueError):
                module.validate_sessions(sessions)

    def test_completion_is_bound_to_current_mission(self):
        valid = helper('valid_delivery', {}, nested=False)
        self.assertTrue(valid({'mission_id': 'new', 'command': 'delivery_complete'}, 'new'))
        for packet in (None, {}, {'mission_id': 'old', 'command': 'delivery_complete'},
                       {'mission_id': 'new', 'command': 'start'}):
            self.assertFalse(valid(packet, 'new'))

    def test_n12_rollback_requires_a_recorded_short_straight_reverse(self):
        namespace = {'math': math}
        stopped = helper('stopped_checkpoint', namespace, nested=False)
        reverse = helper('reverse_to_search_start', namespace, nested=False)
        previous = {'mission_id': 'one', 'final_state': {'pose': None, 'stopped': False}}
        pose = {'x': .14, 'y': -.3, 'yaw': math.pi}
        rows = [dict(mission_id='one', phase='NAV2_RETURN_N12', direction='RETURN',
                     stopped=True, localization_ready=True,
                     pose={'x': .2, 'y': -.3, 'yaw': math.pi}),
                dict(mission_id='one', phase='DRIVE_ON_HEADING', direction='RETURN',
                     stopped=True, localization_ready=True, pose=pose)]
        self.assertEqual(stopped(previous, rows), pose)
        self.assertAlmostEqual(reverse(pose, previous, rows), .06)
        for bad_pose in ({**pose, 'y': -.33}, {**pose, 'x': .1}, {**pose, 'yaw': 0.}):
            with self.assertRaisesRegex(RuntimeError, 'outside bounds'):
                reverse(bad_pose, previous, rows)
        with self.assertRaises(RuntimeError):
            reverse(pose, {'mission_id': 'other'}, rows)
        with self.assertRaises(RuntimeError):
            stopped(previous, [])

    def test_ir_requires_real_center_and_fresh_sample(self):
        centered = helper('centered', {'math': math}, nested=False)
        self.assertTrue(centered(NS(lost=False, on_line=[0, 1, 0], error=0.), .1))
        self.assertFalse(centered(NS(lost=False, on_line=[0, 1, 1], error=.571), .1))
        self.assertFalse(centered(NS(lost=False, on_line=[0, 1, 0], error=0.), .31))
        self.assertFalse(centered(NS(lost=True, on_line=[0, 0, 0], error=0.), .1))

    def test_n12_lateral_offset_uses_native_actions_then_restores_heading(self):
        pose = {'x': .18, 'y': -.29, 'yaw': -3.1}
        target = NS(x=.213, y=-.258, yaw=3.08)
        motions = []
        state = lambda: {'nav2_ready': True, 'localization_ready': True, 'pose': dict(pose)}

        def behavior(kind, name, goal, seconds):
            motions.append(name)
            if name == 'spin':
                pose['yaw'] += goal.target_yaw
            else:
                self.assertLessEqual(goal.target.x, .08)
                self.assertEqual(goal.speed, .01)
                pose['x'] += goal.target.x*math.cos(pose['yaw'])
                pose['y'] += goal.target.x*math.sin(pose['yaw'])

        ns = {'load_destination': lambda *a, **k: target, 'runner': NS(
            stop_confirmed=state), 'options': NS(places='', map=''), 'math': math,
            'json': json, 'Spin': NS(Goal=Goal), 'DriveOnHeading': NS(Goal=Goal),
            'behavior': behavior}
        align = helper('realign_n12', ns)
        align()
        self.assertEqual(motions, ['spin', 'drive_on_heading', 'spin'])
        self.assertAlmostEqual(pose['x'], target.x)
        self.assertAlmostEqual(pose['y'], target.y)
        self.assertAlmostEqual(math.sin(pose['yaw']-target.yaw), 0.)
        motions.clear()
        pose['x'] = target.x+.09
        with self.assertRaisesRegex(RuntimeError, '8cm'):
            align()
        self.assertEqual(motions, [])

        ns['runner'].stop_confirmed = lambda: {
            'nav2_ready': False, 'localization_ready': False, 'pose': None}
        with self.assertRaisesRegex(RuntimeError, 'readiness'):
            align()
        self.assertEqual(motions, [])

    def test_normal_return_realigns_once_and_never_accepts_a_missing_ir_line(self):
        block = next(n for n in ast.walk(MAIN) if isinstance(n, ast.If)
                     and ast.unparse(n.test) == 'not centered(runner.reading, age)')
        runner = NS(reading='lost', ir_at=time.monotonic(), stop_confirmed=lambda: {})
        drives, alignments = [], []

        def behavior(kind, name, goal, seconds, **kwargs):
            self.assertTrue(kwargs['stop_on_line'])
            self.assertEqual(goal.target.x, .06)
            drives.append(name)
            return {}, False

        def align():
            alignments.append(True)
            runner.reading, runner.ir_at = 'center', time.monotonic()

        ns = {'runner': runner, 'age': .1, 'resume_n12': False,
              'centered': lambda reading, age: reading == 'center' and age <= .3,
              'realign_n12': align, 'behavior': behavior, 'time': time,
              'DriveOnHeading': NS(Goal=Goal)}
        code = compile(ast.Module(body=[block], type_ignores=[]), str(SOURCE), 'exec')
        exec(code, ns)
        self.assertEqual(len(drives), 0)
        self.assertEqual(len(alignments), 1)
        runner.reading = 'lost'
        drives.clear()
        alignments.clear()
        ns['realign_n12'] = lambda: alignments.append(True)
        with self.assertRaisesRegex(RuntimeError, 'Physical tape not found'):
            exec(code, ns)
        self.assertEqual(len(drives), 1)
        self.assertEqual(len(alignments), 1)

    def test_distance_steps_stop_at_original_range_and_camera_loss_does_not_move(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = b'measured camera test fixture'
            (root/'camera_calibration.yaml').write_bytes(calibration)
            reference = {'marker_id': 8, 'map_version': 'test',
                         'calibration_sha256': hashlib.sha256(calibration).hexdigest(),
                         'front_minus_lens_m': .001}
            (root/'gate-distance-reference.json').write_text(json.dumps(reference))
            gap, sequence, motions = [.197], [0], []

            def observation():
                sequence[0] += 1
                return {'stamp': sequence[0], 'safe': True, 'lens_plane_cm': gap[0]*100}

            def behavior(kind, name, goal, seconds, **kwargs):
                self.assertTrue(kwargs['marker_watch'])
                self.assertLessEqual(goal.target.x, .01)
                self.assertEqual(goal.speed, .01)
                motions.append(goal.target.x)
                gap[0] -= goal.target.x

            ns = {'TARGET': 'G1', 'MARKER_ID': 8, 'ROOT': root, 'json': json,
                  'math': math, 'hashlib': hashlib, 'np': np, 'profile': {'map_version': 'test'},
                  'time': Clock(), 'spin': lambda: {'stopped': True},
                  'runner': NS(stop_confirmed=lambda: None, require_guard=lambda: None),
                  'observe_marker': observation, 'DriveOnHeading': NS(Goal=Goal),
                  'behavior': behavior, 'marker_samples': [],
                  'DockMarkerLost': type('DockMarkerLost', (RuntimeError,), {})}
            confirm = helper('confirm_gate', ns)
            self.assertTrue(confirm())
            self.assertEqual(len(motions), 3)
            self.assertLessEqual(sum(motions), .06)
            self.assertTrue(.13 <= gap[0]+.001 <= .17)
            motions.clear()
            ns['time'], ns['observe_marker'] = Clock(), lambda: None
            self.assertFalse(confirm())
            self.assertEqual(motions, [])
            ns['observe_marker'], ns['time'] = observation, Clock()
            gap[0] = .197

            def lost(*args, **kwargs):
                raise ns['DockMarkerLost']('canceled after invalid frame')

            ns['behavior'] = lost
            self.assertFalse(confirm())
            ns['spin'] = lambda: {'stopped': False}
            with self.assertRaisesRegex(RuntimeError, 'standstill'):
                confirm()
            reference['map_version'] = 'other-map'
            (root/'gate-distance-reference.json').write_text(json.dumps(reference))
            with self.assertRaisesRegex(RuntimeError, 'reference mismatch'):
                confirm()
            (root/'gate-distance-reference.json').unlink()
            self.assertFalse(confirm())


if __name__ == '__main__':
    unittest.main()
