"""Robot/SSH-free cancellation and UI lease interleavings."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts/field_roundtrip'
sys.path.insert(0, str(SCRIPTS))
from ui_control import Control, ControlLost, LeaseMonitor, TTL


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.now = 10.
        self.c = Control(self.tmp.name, 'a'*32, lambda: self.now, 'boot')

    def arm(self):
        with self.c.launch() as challenge:
            return challenge

    def test_stop_before_start_tombstones_even_delayed_launch(self):
        self.c.stop()
        with self.assertRaises(ControlLost):
            self.arm()

    def test_stop_during_launch_is_serialized_then_no_lease(self):
        started = threading.Event()
        with self.c.launch():
            t = threading.Thread(target=lambda: (started.set(), self.c.stop()))
            t.start()
            self.assertTrue(started.wait(1))
            self.assertTrue(self.c.valid())
        t.join(1)
        self.assertFalse(t.is_alive())
        with self.assertRaises(ControlLost):
            self.c.valid()

    def test_first_heartbeat_missing_and_late_packet_never_revive(self):
        challenge = self.arm()
        self.now += TTL
        for operation in (self.c.valid, lambda: self.c.heartbeat(challenge), self.arm):
            with self.assertRaises(ControlLost):
                operation()

    def test_heartbeat_crossing_deadline_during_validation_cannot_revive(self):
        challenge = self.arm()
        times = iter([12.99, 13.01])
        self.c.clock = lambda: next(times)
        with self.assertRaisesRegex(ControlLost, 'expired before renewal'):
            self.c.heartbeat(challenge)
        self.assertEqual(json.loads(self.c.path.read_text())['deadline'], 13.)

    def test_challenge_consumed_once_stop_cannot_be_renewed(self):
        challenge = self.arm()
        self.now += .5
        new = self.c.heartbeat(challenge)
        with self.assertRaises(ControlLost):
            self.c.heartbeat(challenge)
        self.c.stop()
        with self.assertRaises(ControlLost):
            self.c.heartbeat(new)

    def test_wrong_mission_boot_malformed_future_state_fail_closed(self):
        self.arm()
        original = json.loads(self.c.path.read_text())
        for change in ({'token': 'b'*32}, {'boot': 'other'}, {'deadline': float('nan')},
                       {'issued': self.now+1}, {'deadline': self.now+TTL+1}, {'stopped': 0}):
            self.c.write({**original, **change})
            with self.assertRaises(ControlLost):
                self.c.valid()
        self.c.path.write_text('[]')
        with self.assertRaises(ControlLost):
            self.c.valid()

    def test_every_motion_lease_checked_and_failure_latches(self):
        challenge = self.arm()
        calls = []
        runner = NS(owner='nav2', lease=calls.append)
        monitor = LeaseMonitor(self.c, runner)
        runner.lease('line')
        self.now += TTL
        with self.assertRaises(ControlLost):
            runner.lease('nav2')
        self.assertEqual(calls, ['line', 'none'])
        self.assertEqual(runner.owner, 'none')
        self.now -= TTL
        with self.assertRaises(ControlLost):
            runner.lease('line')
        runner.lease('none')  # Cleanup remains possible after loss.
        self.assertEqual(calls[-1], 'none')

    def test_watchdog_revokes_without_main_spin(self):
        self.arm()
        revoked = threading.Event()
        runner = NS(owner='nav2', lease=lambda owner: revoked.set() if owner == 'none' else None)
        monitor = LeaseMonitor(self.c, runner)
        monitor.start()
        self.c.stop()
        self.assertTrue(revoked.wait(1))
        monitor.close()
        self.assertIsNotNone(monitor.failed)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        env = {'ROPORT_FIELD_CONFIG_DIR': self.tmp.name, 'ROPORT_FIELD_REMOTE_DIR': self.tmp.name,
               'ROPORT_FIELD_REMOTE_SETUP': '/field/setup.sh', 'ROPORT_PINKY_SSH_HOST': 'robot@example.com',
               'ROPORT_PINKY_CONSOLE_URL': 'http://example.com:8080'}
        with patch.dict(os.environ, env):
            spec = importlib.util.spec_from_file_location('ui_test', SCRIPTS/'ui_server.py')
            self.ui = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.ui)
        self.browser = 'b'*32
        self.ui.heartbeat({'browser_id': self.browser, 'seq': 1})

    def test_stop_bypasses_locked_start_and_late_response(self):
        ui = self.ui
        entered, release = threading.Event(), threading.Event()
        ui.snapshot = lambda: {'start_enabled': True}
        def remote(script, **kwargs):
            if 'with control.launch()' in script:
                entered.set()
                self.assertTrue(release.wait(2))
                return 'c'*32
            exec(script, {})  # Stop writes real local tombstone, no process side effects.
            return ''
        ui.remote = remote
        errors = []
        def start():
            try: ui.command('/start', {'control_epoch': self.ui.stop_epoch, 'browser_id': self.browser})
            except ValueError as exc: errors.append(str(exc))
        t = threading.Thread(target=start)
        t.start()
        self.assertTrue(entered.wait(1))
        token = ui.active_control['token']
        reply = ui.command('/stop', {})
        self.assertTrue(reply['success'])
        saved = ui.message
        release.set(); t.join(2)
        self.assertEqual(ui.message, saved)
        self.assertIn('cancelled', errors[0])
        with self.assertRaises(ControlLost):
            with Control(self.tmp.name, token).launch():
                self.fail('Delayed launch must not run')

    def test_prior_server_command_epoch_is_not_reused(self):
        with self.assertRaisesRegex(ValueError, 'predates Stop'):
            self.ui.command('/start', {'control_epoch': self.ui.stop_epoch-1, 'browser_id': self.browser})
        self.assertLess(self.ui.stop_epoch, 2**53)

    def test_start_http_arriving_after_stop_has_old_browser_fence(self):
        ui = self.ui
        delayed_body = {'control_epoch': self.ui.stop_epoch, 'browser_id': self.browser}
        ui.command('/stop', {})
        with self.assertRaisesRegex(ValueError, 'predates Stop'):
            ui.command('/start', delayed_body)
        self.assertIsNone(ui.active_control)
        self.assertTrue(ui.stop_latched)

    def test_start_needs_browser_and_replayed_heartbeat_rejected(self):
        with self.assertRaises(ValueError): self.ui.command('/start', {})
        with self.assertRaises(ValueError): self.ui.heartbeat({'browser_id': self.browser, 'seq': 1})
        self.ui.browser_seen[self.browser] = (2, time.monotonic()-2)
        with self.assertRaises(ValueError): self.ui.command('/start', {'control_epoch': self.ui.stop_epoch, 'browser_id': self.browser})

    def test_get_host_rejected_before_camera_or_status(self):
        for path in ('/', '/status', '/camera.jpg', '/camera-raw.jpg'):
            replies = []
            handler = NS(path=path, headers={'Host': 'example.com'}, reply=lambda *args: replies.append(args))
            self.ui.Handler.do_GET(handler)
            self.assertEqual(replies[0][0], 403)

    def test_launch_refuses_foreign_guard_without_signal_or_tmux(self):
        calls = []
        def fake(args, **kwargs):
            calls.append(args)
            return NS(returncode=0 if args[:3] == ['pgrep', '-f', '[r]oport_pinky/trial_guard'] else 1, stdout='42' if args[0] == 'pgrep' else '')
        with patch('subprocess.run', fake):
            with self.assertRaisesRegex(RuntimeError, 'Incompatible'):
                exec(self.ui.trial_launch_script(token='a'*32), {})
        self.assertFalse(any(x[0] == 'kill' or x[:2] == ['tmux', 'new-session'] for x in calls))
        self.assertNotIn('os.kill', self.ui.trial_launch_script(token='a'*32))

    def test_foreign_stack_discovery_error_is_not_absence(self):
        from field_stack import compatible_guard
        with patch('subprocess.run', return_value=NS(returncode=2)):
            with self.assertRaisesRegex(RuntimeError, 'Cannot inspect'):
                compatible_guard(self.tmp.name)

    def test_owned_guard_requires_exact_root_domain_and_tmux_ancestor(self):
        from field_stack import owned_guard
        proc = Path(self.tmp.name)/'proc'
        for pid, parent in ((42, 41), (41, 1)):
            (proc/str(pid)).mkdir(parents=True)
            (proc/str(pid)/'status').write_text('PPid: '+str(parent)+'\n')
        environment = proc/'42/environ'
        environment.write_bytes(('ROS_DOMAIN_ID=13\0ROPORT_FIELD_CONFIG_DIR='+self.tmp.name+'\0').encode())
        self.assertTrue(owned_guard(42, self.tmp.name, [('field_guard_123', '41')], proc))
        self.assertFalse(owned_guard(42, self.tmp.name, [('other_stack', '41')], proc))
        self.assertFalse(owned_guard(42, '/other', [('field_guard_123', '41')], proc))
        environment.write_bytes(b'ROS_DOMAIN_ID=99\0')
        self.assertFalse(owned_guard(42, self.tmp.name, [('field_guard_123', '41')], proc))

    def test_normal_and_resume_launch_reuse_owned_guard_without_replacing_it(self):
        for resume in (False, True):
            if resume:
                (Path(self.tmp.name)/'result.json').write_text(json.dumps({'mission_id': 'e'*32, 'phase': 'STOPPED'}))
            calls = []
            def fake(args, **kwargs):
                calls.append(args)
                return NS(returncode=1 if args[0] == 'pgrep' or args[1] == 'has-session' else 0, stdout='')
            token = ('d' if resume else 'c')*32
            with patch('field_stack.compatible_guard', return_value=True), patch('subprocess.run', fake):
                exec(self.ui.trial_launch_script(resume, 'e'*32 if resume else None, token,
                                                 requested_target=None if resume else 'G3'), {})
            launches = [args for args in calls if args[:2] == ['tmux', 'new-session']]
            self.assertEqual(len(launches), 1)
            self.assertIn('h1g3_ui_mission', launches[0])
            self.assertFalse(any('trial_guard' in str(args) for args in calls))
            self.assertEqual(json.loads((Path(self.tmp.name)/'mission-target.json').read_text()), 'G3')

    def test_lost_ui_ssh_does_not_renew_and_stop_failure_is_prominent(self):
        self.ui.active_control = dict(token='a'*32, browser_id=self.browser, challenge='c'*32)
        self.ui.remote = lambda *a, **k: (_ for _ in ()).throw(OSError('transport down'))
        with self.assertRaises(OSError): self.ui.heartbeat({'browser_id': self.browser, 'seq': 2})
        self.assertEqual(self.ui.active_control['challenge'], 'c'*32)
        with self.assertRaisesRegex(ValueError, '정지 요청 실패'): self.ui.command('/stop', {})
        self.assertTrue(self.ui.stop_latched)


if __name__ == '__main__':
    unittest.main()
