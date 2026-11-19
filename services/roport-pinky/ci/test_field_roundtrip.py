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
    def test_return_detour_checks_the_upper_path_before_following(self):
        def stamped():
            return NS(header=NS(frame_id='map'), pose=NS(position=NS(), orientation=NS()))

        class Frame:
            yaw_deg = 0.

            def to_measured(self, x, y):
                return x*100, y*100

            def to_world(self, x, y):
                return x/100, y/100

        frame = Frame()
        raw = {'objects': [{'name': 'box_03', 'min': [320, 140], 'max': [390, 160]}],
               'walls': [{'name': name, 'from': point} for name, point in
                         [('zone03_left', [140, 94]), ('zone03_bottom', [140, 94]),
                          ('outer_window', [440, 220]), ('outer_right', [440, 0])]]}
        start = dict(x=2.9, y=1.2, yaw=0.)
        entry = dict(x=4.1, y=.8, yaw=0.)
        start_state = dict(pose=start, nav2_ready=True, localization_ready=True, stopped=True)
        end_state = {**start_state, 'pose': entry}
        result = NS(status=4, result=NS(error_code=0, path=NS(header=NS(frame_id='map'))))
        handle = NS(accepted=True, get_result_async=lambda: result)
        events = []

        def make_path(*points):
            path = []
            for x, y in points:
                pose = stamped()
                pose.pose.position.x, pose.pose.position.y = x, y
                path.append(pose)
            result.result.path.poses = path

        def navigate(_):
            events.append('direct')
            if nav.outcome != 'succeeded':
                raise RuntimeError('Nav2 goal failed: '+nav.outcome)

        def plan(goal):
            events.append('plan')
            self.assertEqual(len(goal.goals), 3)
            self.assertEqual(runner.owner, 'none')
            return handle

        def follow(kind, name, goal, seconds, phase):
            events.append('follow')
            self.assertEqual((name, phase), ('follow_path', 'NAV2_RETURN_DETOUR'))
            self.assertIs(goal.path, result.result.path)
            return end_state, False

        nav = NS(outcome='succeeded', busy=False, fault=None)
        runner = NS(nav=NS(lifecycle=nav), lease=lambda _: events.append('release'),
                    stop_confirmed=lambda: start_state)
        ns = dict(math=math, runner=runner, navigate=navigate, settle=lambda: start_state,
                  layout=NS(frame=frame, raw=raw), PoseStamped=stamped,
                  ComputePathThroughPoses=NS(Goal=lambda: NS(goals=[])),
                  FollowPath=NS(Goal=NS), GoalStatus=NS(STATUS_SUCCEEDED=4),
                  detour_planner=NS(server_is_ready=lambda: True, send_goal_async=plan),
                  node=NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None))),
                  wait=lambda value, _: value, Destination=NS, behavior=follow,
                  handoff=NS(wrap=lambda angle: angle))
        for name in ('upper_return_waypoints', 'follows_upper_corridor', 'confirmed_gate_return'):
            helper(name, ns, nested=False)
        return_entry = helper('navigate_return_entry', ns)

        return_entry(entry)
        self.assertEqual(events, ['direct'])
        nav.outcome = 'aborted'
        make_path((2.9, 1.2), (2.95, 1.9), (3.5, 1.9), (4.15, 1.9), (4.1, .8))
        return_entry(entry)
        self.assertEqual(events[-4:], ['direct', 'release', 'plan', 'follow'])
        make_path((2.9, 1.2), (2.95, 1.9), (3.5, 1.2), (4.15, 1.9), (4.1, .8))
        with self.assertRaisesRegex(RuntimeError, 'No confirmed upper return path'):
            return_entry(entry)
        self.assertEqual(events[-3:], ['direct', 'release', 'plan'])
        nav.fault = 'transport_error'
        with self.assertRaisesRegex(RuntimeError, 'aborted'):
            return_entry(entry)
        self.assertEqual(events[-1], 'direct')

    def test_h1_branch_reduces_speed_without_changing_other_segments(self):
        normal, approach = NS(base_speed=.06), NS(base_speed=.03)
        line = NS(index=0, legs=[NS(frm='N08', to='N09'), NS(frm='N09', to='H1')],
                  state='follow', cfg=normal)
        step = helper('line_step', dict(runner=NS(line=line), line_config=normal,
                                      h1_approach=approach, original_step=lambda *_: line.cfg))
        self.assertIs(step(None, None, 1.), normal)
        line.index = 1
        self.assertIs(step(None, None, 2.), approach)
        line.state = 'rotate'
        self.assertIs(step(None, None, 3.), normal)

    def test_cached_detector_keeps_marker_geometry_and_acceptance_limits(self):
        import cv2
        import yaml
        ns = {'cv2': cv2, 'np': np, 'yaml': yaml, 'math': math, 'MARKER_ID': 8}
        names = {'marker_dictionary', 'marker_detector', 'marker_object'}
        setup = [n for n in MAIN.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id in names for t in n.targets)]
        exec(compile(ast.Module(body=setup, type_ignores=[]), str(SOURCE), 'exec'), ns)
        dictionary = ns['marker_dictionary']
        image = np.full((480, 640), 255, np.uint8)
        image[195:285, 275:365] = cv2.aruco.drawMarker(dictionary, 8, 90)
        ok, data = cv2.imencode('.png', image)
        self.assertTrue(ok)
        calibration = yaml.safe_dump({'camera_matrix': {'data': [300.,0.,320.,0.,300.,240.,0.,0.,1.]},
                                     'distortion_coefficients': {'data': [0.]*5}}).encode()
        detect = helper('detect_marker', ns)
        result = detect(NS(data=data.tobytes()), 1., calibration)
        self.assertEqual(result['id'], 8)
        self.assertTrue(result['safe'])
        self.assertAlmostEqual(result['lens_plane_cm'], 20., delta=.5)
        self.assertIsNone(detect(NS(data=b'not an image'), 2., calibration))

    def test_marker_cache_expires_and_rechecks_changed_frame_or_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = root/'camera_calibration.yaml'
            calibration.write_bytes(b'original')
            message = NS(header=NS(stamp=NS(sec=1, nanosec=0)))
            clock = NS(nanoseconds=1_100_000_000)
            calls = []
            def detect(msg, stamp, data):
                calls.append((msg, data))
                return {'stamp': stamp, 'safe': True} if data == b'original' else None
            ns = {'camera': {'message': message}, 'node': NS(get_clock=lambda: NS(now=lambda: clock)),
                  'ROOT': root, 'detect_marker': detect}
            observe = helper('observe_marker', ns)
            for _ in range(10):
                self.assertTrue(observe()['safe'])
            self.assertEqual(len(calls), 1)
            clock.nanoseconds = 1_400_000_000
            self.assertIsNone(observe())
            self.assertEqual(len(calls), 1)
            clock.nanoseconds = 1_100_000_000
            calibration.write_bytes(b'changed')
            self.assertIsNone(observe())
            self.assertIsNone(observe())
            self.assertEqual(len(calls), 2)
            ns['camera']['message'] = NS(header=message.header)
            self.assertIsNone(observe())
            self.assertEqual(len(calls), 3)

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
            ui.heartbeat({'control_epoch': ui.stop_epoch, 'browser_id': 'd'*32, 'seq': 1})
            ui.latest = {'phase': 'WAIT_DELIVERY', 'mission_id': 'a'*32}
            ui.observed = ui.odom_seen = ui.guard_seen = time.monotonic()
            ui.guard = {'owner': 'none', 'blocked': False}
            ui.odom = NS(twist=NS(twist=NS(linear=NS(x=0.), angular=NS(z=0.))))
            sent = []
            ui.remote = sent.append
            self.assertTrue(ui.snapshot()['delivery_enabled'])
            ui.guard['blocked'] = True
            self.assertFalse(ui.snapshot()['start_enabled'])
            ui.guard['blocked'] = False
            with self.assertRaises(ValueError):
                ui.command('/delivery-complete', {'control_epoch': ui.stop_epoch, 'browser_id': 'd'*32, 'mission_id': 'b'*32})
            self.assertEqual(sent, [])
            ui.command('/delivery-complete', {'control_epoch': ui.stop_epoch, 'browser_id': 'd'*32, 'mission_id': 'a'*32})
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
            ui.guard['blocked'] = True
            self.assertFalse(ui.snapshot()['start_enabled'])
            self.assertTrue(ui.snapshot()['restart_enabled'])
            ui.guard['blocked'] = False
            ui.latest['phase'] = 'LINE'
            self.assertFalse(ui.snapshot()['restart_enabled'])
            with self.assertRaises(ValueError):
                ui.command('/restart-stack', {'control_epoch': ui.stop_epoch, 'browser_id': 'd'*32})
            self.assertEqual(len(sent), 1)
            ui.latest['phase'] = 'STOPPED'
            captured = []
            ui.remote = lambda script, **kwargs: (captured.append((script, kwargs)) or 'c'*32)
            ui.command('/restart-stack', {'control_epoch': ui.stop_epoch, 'browser_id': 'd'*32})
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
            ui.command('/delivery-complete', {'control_epoch': ui.stop_epoch, 'browser_id': 'd'*32, 'mission_id': 'a'*32})
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
