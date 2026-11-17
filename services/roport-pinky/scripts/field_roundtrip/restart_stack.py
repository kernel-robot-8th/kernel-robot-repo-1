"""Operator-requested restart of this trial's stack; never sends driving goals."""
import argparse
import hashlib
import json
import math
import os
import shlex
import signal
import subprocess
import time
import urllib.request
from urllib.parse import urlsplit
from pathlib import Path


KINDS = ('bringup', 'adc', 'nav2', 'console')


def validate_sessions(sessions):
    if (not isinstance(sessions, dict) or set(sessions) != set(KINDS)
            or any(not isinstance(v, str) or not v.startswith('field_')
                   or not all(c.isalnum() or c in '_-' for c in v) for v in sessions.values())
            or len(set(sessions.values())) != len(KINDS)):
        raise ValueError('Invalid managed field sessions')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--restart', action='store_true')
    parser.add_argument('--setup', type=Path, required=True)
    args = parser.parse_args()
    if not args.restart or os.environ.get('ROS_DOMAIN_ID') != '13':
        parser.error('Explicit --restart and ROS_DOMAIN_ID=13 required')
    root = Path(os.environ['ROPORT_FIELD_CONFIG_DIR']).resolve()
    sessions_path = root/'stack-sessions.json'
    sessions = json.loads(sessions_path.read_text())
    validate_sessions(sessions)
    if not args.setup.is_absolute() or not args.setup.is_file():
        raise ValueError('Existing absolute ROS setup path required')
    if subprocess.run(['pgrep', '-f', '[r]oundtrip.py --drive'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Stop the active trial before restarting nodes')

    # Observe fresh physical stop locally as well as in the UI before any shutdown.
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('field_stack_restart_observer')
    seen = {}
    node.create_subscription(Odometry, 'odom',
        lambda m: seen.update(odom=(time.monotonic(), m)), qos_profile_sensor_data)
    node.create_subscription(String, 'trial/guard_state',
        lambda m: seen.update(guard=(time.monotonic(), json.loads(m.data))), 1)
    since, end = None, time.monotonic()+4
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic()
        at, odom = seen.get('odom', (0, None))
        gt, guard = seen.get('guard', (0, {}))
        stopped = (odom is not None and now-at < .3 and now-gt < .3
                   and guard.get('owner') == 'none'
                   and abs(odom.twist.twist.linear.x) <= .005
                   and abs(odom.twist.twist.angular.z) <= .02)
        since = (since or now) if stopped else None
        if since is not None and now-since >= .5:
            break
    else:
        raise RuntimeError('Fresh physical stop and released drive ownership required')

    url = os.environ['ROPORT_PINKY_CONSOLE_URL'].rstrip('/')
    parsed = urlsplit(url)
    if parsed.scheme != 'http' or not parsed.hostname or parsed.port != 8080:
        raise ValueError('Existing HTTP console on port8080 required')
    def get(path):
        return json.load(urllib.request.urlopen(url+path, timeout=2))

    before = get('/api/state')
    pose = before.get('pose')
    if before.get('navigating') or not pose or not all(math.isfinite(pose[k]) for k in ('x', 'y', 'yaw')):
        raise RuntimeError('Stopped localized pose and inactive navigation required')
    for name in sessions.values():
        if subprocess.run(['tmux', 'has-session', '-t', name],
                          stderr=subprocess.DEVNULL).returncode == 0:
            subprocess.run(['tmux', 'send-keys', '-t', name, 'C-c'], check=True)
    for value in subprocess.run(['pgrep', '-f', '[r]oport_pinky/trial_guard'],
                                capture_output=True, text=True).stdout.split():
        os.kill(int(value), signal.SIGINT)
    end = time.monotonic()+10
    while time.monotonic() < end:
        remaining = [name for name in sessions.values() if subprocess.run(
            ['tmux', 'has-session', '-t', name], stderr=subprocess.DEVNULL).returncode == 0]
        if not remaining:
            break
        time.sleep(.1)
    else:
        raise RuntimeError('Previous field nodes did not terminate; no duplicate startup attempted')
    if subprocess.run(['pgrep', '-f', '[r]oport_pinky/trial_guard'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Previous drive guard did not terminate')

    logs = root/'logs'/time.strftime('%Y%m%d-%H%M%S-ui-restart')
    logs.mkdir(parents=True)
    q = shlex.quote
    base = 'source '+q(str(args.setup))+'; export ROS_DOMAIN_ID=13; '
    commands = {
        'bringup': 'ros2 launch pinky_bringup bringup_robot.launch.xml',
        'adc': 'ros2 run pinky_sensor_adc main_node',
        'nav2': ('ros2 launch '+q(str(root/'field_trial.launch.py'))
                 +' robot:='+q(os.environ['ROBOT_NAME'])+' map:='+q(os.environ['ROPORT_PINKY_MAP'])
                 +' layout_file:='+q(str(root/'layout.yaml'))
                 +' params_file:='+q(str(root/'nav2-lidar-sonar-field.yaml'))+' max_linear:=0.12'),
        'console': ('ros2 run pinky_test_console web_server --ros-args -p mode:=amcl'
                    ' -p ip:='+q(parsed.hostname)+' -p port:=8080 -p teleop_enabled:=false'
                    ' -p layout_yaml:='+q(str(root/'layout.yaml'))),
        'guard': 'ros2 run roport_pinky trial_guard',
    }
    for kind, command in commands.items():
        name = 'field_'+kind+(('_'+str(time.time_ns())) if kind == 'guard' else '')
        subprocess.run(['tmux', 'new-session', '-d', '-s', name, 'bash', '-lc',
            base+'exec '+command+' > '+q(str(logs/(kind+'.log')))+' 2>&1'], check=True)
    sessions_path.write_text(json.dumps({k: 'field_'+k for k in KINDS}))

    # Keep the observed pose across restart; never assume the robot returned to H1.
    end = time.monotonic()+25
    while time.monotonic() < end:
        try:
            info = get('/api/amcl')
            frame = info['frame']
            angle = math.radians(frame['yaw_deg'])
            dx, dy = pose['x']-frame['x_m'], pose['y']-frame['y_m']
            body = {'x_cm': 100*(math.cos(angle)*dx+math.sin(angle)*dy),
                    'y_cm': 100*(-math.sin(angle)*dx+math.cos(angle)*dy),
                    'yaw_deg': math.degrees(pose['yaw'])-frame['yaw_deg']}
            request = urllib.request.Request(url+'/api/amcl/initialpose',
                data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'}, method='POST')
            response = json.load(urllib.request.urlopen(request, timeout=3))
            if response.get('success'):
                break
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(.5)
    else:
        raise RuntimeError('Stack restarted but console/pose restoration not confirmed; check logs')
    import threading
    import yaml
    from roport_pinky.trial_nav2 import Nav2Bridge
    path = Path(os.environ['ROPORT_PINKY_MAP'])
    data = yaml.safe_load(path.read_text())
    profile = {'map_id': path.stem, 'map_version': hashlib.sha256(
        path.read_bytes()+(path.parent/data['image']).read_bytes()).hexdigest()[:16]}
    bridge = Nav2Bridge(node, profile, str(path), threading.RLock())
    end, since = time.monotonic()+12, None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        state = bridge.snapshot()
        good = state['nav2_ready'] and state['localization_ready'] and state['stopped']
        since = (since or time.monotonic()) if good else None
        if since is not None and time.monotonic()-since >= .5:
            print(json.dumps({'success': True, 'logs': str(logs), 'no_goal_sent': True}))
            node.destroy_node()
            rclpy.shutdown()
            return
    raise RuntimeError('Nodes restarted but navigation readiness not confirmed; remain stopped')


if __name__ == '__main__':
    main()
