"""Explicit, local-UI lifecycle controls for the supervised Pinky demo."""

import json
import re
import subprocess
from threading import Lock, Thread


REMOTE_SCRIPT = r'''
import json, os, shlex, subprocess, sys, time
home = os.path.expanduser('~')
base = 'set -e; source /opt/ros/jazzy/setup.bash; '
domain = 'export ROS_DOMAIN_ID=13; '
pro = base + 'source ' + shlex.quote(home + '/pinky_pro/install/setup.bash') + '; ' + domain
team = base + 'source ' + shlex.quote(home + '/roport-baseline/repo/services/roport-pinky/install/setup.bash') + '; ' + domain
camera = home + '/roport-baseline/run/two/doll_camera_stream_20261114.py'
commands = {
 'adc': pro + 'exec ros2 run pinky_sensor_adc main_node',
 'camera': base + domain + 'exec python3 ' + shlex.quote(camera),
 'guard': team + 'exec ros2 run roport_pinky trial_guard',
 'bringup': pro + 'exec ros2 launch pinky_bringup bringup_robot.launch.xml',
}
patterns = {
 'adc': '[p]inky_sensor_adc/main_node',
 'camera': '[d]oll_camera_stream_20261114.py',
 'guard': '[r]oport_pinky/trial_guard',
 'bringup': '[r]os2 launch pinky_bringup|[/]pinky_bringup/bringup|[s]llidar_node',
}
def running(name):
 return subprocess.run(['pgrep', '-f', patterns[name]], stdout=subprocess.DEVNULL).returncode == 0
def managed(name):
 return subprocess.run(['tmux', 'has-session', '-t', 'doll_' + name], stderr=subprocess.DEVNULL).returncode == 0
def start(name):
 if running(name): return
 if managed(name): raise RuntimeError(name + ': 기존 세션 종료 후 다시 시도하세요')
 subprocess.run(['tmux', 'new-session', '-d', '-s', 'doll_' + name,
                 shlex.join(['bash', '-lc', commands[name]])], check=True)
 time.sleep(2 if name == 'bringup' else 1)
 if not running(name): raise RuntimeError(name + ': 실행 실패, 핑키 설치 경로와 장치를 확인하세요')
def stop(name):
 if not managed(name):
  if running(name): raise RuntimeError(name + ': 다른 터미널에서 실행 중입니다. 해당 터미널에서 종료하세요')
  return
 subprocess.run(['tmux', 'send-keys', '-t', 'doll_' + name, 'C-c'], check=True)
 for _ in range(40):
  if not running(name): return
  time.sleep(.1)
 raise RuntimeError(name + ': 종료 지연, 핑키에서 직접 확인하세요')
try:
 action = sys.argv[1]
 if action in ('sensors', 'bringup'):
  if not os.path.isfile(camera): raise RuntimeError('핑키 카메라 실행 파일이 없습니다')
  for name in ('guard', 'adc', 'camera'): start(name)
  if action == 'bringup': start('bringup')
 elif action == 'stop_all':
  # Keep the zero-command guard alive until motor bringup has exited.
  for name in ('bringup', 'camera', 'adc', 'guard'): stop(name)
 elif action != 'check': raise RuntimeError('허용되지 않은 작업')
 print(json.dumps({'ok': True, 'nodes': {name: running(name) for name in commands}}))
except Exception as exc:
 print(json.dumps({'ok': False, 'error': str(exc),
                   'nodes': {name: running(name) for name in commands}}))
'''


class RobotSetup:
    """SSH work stays off the ROS control thread; only fixed actions are accepted."""

    def __init__(self, host=None):
        if host and not re.fullmatch(r'[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*', host):
            raise ValueError('robot host must be user@hostname')
        self.host = host
        self.lock = Lock()
        self.pending_stop = False
        self.state = {'enabled': bool(host), 'busy': False, 'connected': False,
                      'message': '연결 확인을 눌러주세요' if host else '--robot-host 설정 필요',
                      'nodes': {}}

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def submit(self, action):
        if action not in ('check', 'sensors', 'bringup', 'stop_all') or not self.host:
            return False
        with self.lock:
            if self.state['busy']:
                if action == 'stop_all':
                    self.pending_stop = True
                    self.state['message'] = '현재 준비 단계 종료 후 전체 정지 중'
                    return True
                return False
            self.state.update(busy=True, message='핑키 작업 중…')
        Thread(target=self._work, args=(action,), daemon=True).start()
        return True

    def _work(self, action):
        while True:
            try:
                result = subprocess.run(
                    ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4',
                     '-o', 'StrictHostKeyChecking=yes', self.host, 'python3 - ' + action],
                    input=REMOTE_SCRIPT, capture_output=True, text=True, timeout=25, check=True)
                data = json.loads(result.stdout)
                message = ({'check': '핑키 연결 확인 완료', 'sensors': '센서·카메라·가드 실행 완료',
                            'bringup': '브링업 실행 완료 · 추종은 시작 버튼으로',
                            'stop_all': '모터 브링업·센서·카메라 종료 완료'}[action]
                           if data['ok'] else data['error'])
                update = dict(connected=True, message=message, nodes=data['nodes'])
            except (subprocess.SubprocessError, OSError, ValueError, KeyError, TypeError):
                update = dict(connected=False, message='SSH 연결/실행 실패 · 주소·SSH 키·핑키 전원을 확인하세요',
                              nodes={})
            with self.lock:
                self.state.update(update)
                if self.pending_stop:
                    self.pending_stop = False
                    action = 'stop_all'
                    self.state['message'] = '전체 정지 중…'
                else:
                    self.state['busy'] = False
                    return
