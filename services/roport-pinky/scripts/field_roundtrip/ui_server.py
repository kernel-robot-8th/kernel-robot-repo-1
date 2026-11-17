"""Local standalone-test UI; ROS observation only, fixed SSH lifecycle commands."""
import json
import os
import urllib.request
from pathlib import Path
import re
import shlex
import subprocess
import threading
import textwrap
import time
from uuid import uuid4
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UI_DIR = Path(__file__).resolve().parent
ROOT = Path(os.environ['ROPORT_FIELD_CONFIG_DIR']).expanduser().resolve()
REMOTE = os.environ['ROPORT_FIELD_REMOTE_DIR']
SETUP = os.environ['ROPORT_FIELD_REMOTE_SETUP']
HOST = os.environ['ROPORT_PINKY_SSH_HOST']
CONSOLE = os.environ['ROPORT_PINKY_CONSOLE_URL'].rstrip('/')
if not REMOTE.startswith('/') or not SETUP.startswith('/') or HOST.startswith('-') or not re.fullmatch(r'[A-Za-z0-9_.@:-]+', HOST):
    raise ValueError('Absolute remote paths and a valid SSH host are required')
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
       '-o', 'StrictHostKeyChecking=yes', HOST]
lock = threading.Lock()
command_lock = threading.Lock()
latest = json.loads((ROOT/'last-ui-state.json').read_text()) if (ROOT/'last-ui-state.json').exists() else {}
observed = 0.
guard = {}
guard_seen = 0.
odom = None
odom_seen = 0.
message = 'H1 위치와 방향을 확인한 뒤 출발하세요.'
camera_jpeg = b''
camera_raw = b''
camera_at = 0.
camera_info = {}
restarting = False
restart_failed = False

# No background renewal: only fresh browser requests can advance the robot lease.
browser_seen = {}
active_control = None
# Random safe-integer epoch also invalidates queued commands after a UI server restart.
stop_epoch = uuid4().int >> 80
stop_latched = False
heartbeat_lock = threading.Lock()


def browser_live(browser_id):
    entry = browser_seen.get(browser_id)
    return entry is not None and 0 <= time.monotonic()-entry[1] < 1.5


def control_script(token, operation, challenge=None):
    return ('import sys\nsys.path.insert(0, '+repr(REMOTE)+')\n'
            'from ui_control import Control\nc=Control('+repr(REMOTE)+','+repr(token)+')\n'
            + ('c.stop()\n' if operation == 'stop' else
               'print(c.heartbeat('+repr(challenge)+'))\n'))


def heartbeat(body):
    browser_id, seq = body.get('browser_id'), body.get('seq')
    if (not isinstance(browser_id, str) or not re.fullmatch('[0-9a-f-]{32,36}', browser_id)
            or type(seq) is not int or seq < 0):
        raise ValueError('Invalid browser heartbeat')
    if not heartbeat_lock.acquire(blocking=False):
        raise ValueError('Heartbeat already pending')
    try:
        with lock:
            previous = browser_seen.get(browser_id)
            if previous and seq <= previous[0]:
                raise ValueError('Stale browser heartbeat')
            browser_seen[browser_id] = (seq, time.monotonic())
            current = None if active_control is None else dict(active_control)
            epoch = stop_epoch
        if current and current['browser_id'] == browser_id and current.get('challenge') and not stop_latched:
            challenge = remote(control_script(current['token'], 'heartbeat', current['challenge']), timeout=2).strip()
            if not re.fullmatch('[0-9a-f]{32}', challenge):
                raise ValueError('Invalid robot heartbeat response')
            with lock:
                if stop_epoch == epoch and active_control is not None and active_control['token'] == current['token']:
                    active_control['challenge'] = challenge
        return {'success': True, 'control_epoch': stop_epoch}
    finally:
        heartbeat_lock.release()


def stop_command():
    global stop_epoch, stop_latched, message
    with lock:
        stop_epoch += 1
        stop_latched = True
        token = None if active_control is None else active_control['token']
        message = '정지 요청 중입니다. 아직 실제 정지가 확인되지 않았습니다.'
    try:
        if token:
            remote(control_script(token, 'stop'), timeout=5)
        with lock:
            message = '정지 요청을 보냈습니다. 실제 정지 확인 전에는 현장 정지 수단을 준비하세요.'
    except (ValueError, OSError, subprocess.SubprocessError):
        with lock:
            message = '정지 요청 실패: 현장 정지 수단을 즉시 사용하세요. 연결 유실 watchdog은 실제 정지 확인이 아닙니다.'
        raise ValueError(message)
    return {'success': True, 'message': message}



def snapshot():
    with lock:
        now = time.monotonic()
        fresh = now-observed < 2
        parked = (odom is not None and now-odom_seen < .5
                  and abs(odom.twist.twist.linear.x) <= .005
                  and abs(odom.twist.twist.angular.z) <= .02)
        active = fresh and latest.get('phase') not in ('DONE', 'STOPPED')
        guard_ready = (now-guard_seen < .3 and guard.get('owner') == 'none'
                       and guard.get('blocked') is False)
        coordinate_mode = (ROOT/'coordinate-recording.lock').exists()
        return_resume = (latest.get('phase') == 'STOPPED' and latest.get('direction') == 'RETURN'
                         and isinstance(latest.get('n12_handoff_id'), str)
                         and re.fullmatch('[0-9a-f]{32}', latest['n12_handoff_id']) is not None
                         and isinstance(latest.get('n12_return_pose'), dict)
                         and any(text in (latest.get('error') or '') for text in ('N12', 'Physical tape'))
                         and parked and guard_ready
                         and not restarting and not restart_failed)
        start_ok = parked and guard_ready and not active and not coordinate_mode and not restarting and not restart_failed and not return_resume
        return {**latest, 'fresh': fresh, 'physically_stopped': parked,
                'start_enabled': start_ok, 'coordinate_mode': coordinate_mode,
                'delivery_enabled': return_resume or (fresh and parked and not restarting and latest.get('phase') == 'WAIT_DELIVERY'),
                'return_resume_enabled': return_resume,
                'restarting': restarting,
                'restart_enabled': parked and now-guard_seen < .5 and guard.get('owner') == 'none' and not active and not restarting,
                'stop_confirmed': bool(stop_latched and fresh and parked and guard_ready
                    and latest.get('phase') in ('STOPPED', 'DONE') and active_control
                    and latest.get('ui_control_id') == active_control['token']),
                'message': message, 'camera': {**camera_info, 'live': now-camera_at < 1.5, 'age_s': round(now-camera_at, 1) if camera_at else None}}


def remote(script, timeout=7):
    result = subprocess.run(SSH+['python3 -'], input=script,
                            text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise ValueError((result.stderr or result.stdout or '로봇 연결 실패')[-1200:])
    return result.stdout



def refresh_start_localization():
    """Refresh real AMCL observations during the operator-requested startup only."""
    for _ in range(8):
        try:
            request = urllib.request.Request(
                CONSOLE+'/api/amcl/nomotion', data=b'{}',
                headers={'Content-Type': 'application/json'}, method='POST')
            with urllib.request.urlopen(request, timeout=2) as response:
                response.read()
        except OSError:
            pass  # The existing mission readiness deadline still rejects stale data.
        time.sleep(.5)


def trial_launch_script(n12=False, checkpoint=None, token=None):
    script = r'''
    import os, signal, subprocess, shlex, time, json
    from pathlib import Path
    root = FIELD_REMOTE_DIR
    checkpoint = FIELD_CHECKPOINT
    extra = FIELD_TRIAL_FLAGS
    if checkpoint is not None:
     result = json.loads((Path(root)/'result.json').read_text())
     if result.get('mission_id') != checkpoint or result.get('phase') != 'STOPPED':
      raise RuntimeError('Return checkpoint changed')
    target_file = Path(root)/'mission-target.json'
    target = json.loads(target_file.read_text()) if target_file.exists() else 'G1'
    if target != 'G1':
     raise RuntimeError('이 UI는 G1 전용입니다. 현장 목표 설정부터 확인하세요.')
    for pattern in ('[r]oundtrip.py --drive',):
     if subprocess.run(['pgrep','-f',pattern],stdout=subprocess.DEVNULL).returncode==0:
      raise RuntimeError('기존 시험 실행 중')
    base = FIELD_REMOTE_BASE
    import sys
    sys.path.insert(0, root)
    from ui_control import Control
    from field_stack import compatible_guard
    control = Control(root, FIELD_CONTROL_TOKEN)
    with control.launch() as challenge:
     guard_running = compatible_guard(root)
     if subprocess.run(['tmux','has-session','-t','h1g3_ui_mission'],stderr=subprocess.DEVNULL).returncode==0:
      raise RuntimeError('Existing UI mission session; refusing replacement')
     if not guard_running:
      subprocess.run(['tmux','new-session','-d','-s','h1g3_ui_guard_'+str(time.time_ns()),'bash','-lc',base+'exec ros2 run roport_pinky trial_guard'],check=True,timeout=2)
     cmd=base+'exec python3 '+shlex.quote(root+'/roundtrip.py')+' --drive'+extra+' > '+shlex.quote(root+'/mission.log')+' 2>&1'
     subprocess.run(['tmux','new-session','-d','-s','h1g3_ui_mission','bash','-lc',cmd],check=True,timeout=2)
    print(challenge)
    '''
    script = script.replace('FIELD_CONTROL_TOKEN', repr(token)).replace('FIELD_CHECKPOINT', repr(checkpoint)).replace('FIELD_TRIAL_FLAGS', repr(' --resume-n12' if n12 else '')).replace('FIELD_REMOTE_DIR', repr(REMOTE)).replace('FIELD_REMOTE_BASE', repr('source '+shlex.quote(SETUP)+'; export ROS_DOMAIN_ID=13; export ROPORT_FIELD_CONFIG_DIR='+shlex.quote(REMOTE)+'; export ROPORT_UI_CONTROL_ID='+shlex.quote(token or '')+'; '))
    return textwrap.dedent(script)


def command(name, body):
    global message, restarting, restart_failed, active_control, stop_latched
    if name == '/stop':
        return stop_command()
    if name == '/heartbeat':
        return heartbeat(body)
    with lock:
        epoch = stop_epoch
        if type(body.get('control_epoch')) is not int or body['control_epoch'] != epoch:
            raise ValueError('Command predates Stop; refresh browser heartbeat before a new action')
        if not browser_live(body.get('browser_id')):
            raise ValueError('브라우저 연결이 신선하지 않습니다.')
    if not command_lock.acquire(blocking=False):
        raise ValueError('이전 명령 처리 중입니다.')
    try:
        state = snapshot()
        def notify(text):
            global message
            with lock:
                if epoch == stop_epoch:
                    message = text
        def launch(n12=False, checkpoint=None):
            global active_control, stop_latched
            token = uuid4().hex
            with lock:
                if epoch != stop_epoch or not browser_live(body.get('browser_id')):
                    raise ValueError('Start cancelled or browser expired')
                active_control = dict(token=token, browser_id=body['browser_id'], challenge=None)
                stop_latched = False
            challenge = remote(trial_launch_script(n12, checkpoint, token)).strip()
            with lock:
                if epoch != stop_epoch:
                    raise ValueError('Start cancelled by Stop')
                if not re.fullmatch('[0-9a-f]{32}', challenge):
                    raise ValueError('Invalid robot launch response')
                active_control['challenge'] = challenge
        if name == '/restart-stack':
            if not state['restart_enabled']:
                raise ValueError('주행 중에는 재시작할 수 없습니다. 시험 중단 후 실제 정지를 확인하세요.')
            restarting = True
            notify('브링업·Nav2·센서를 다시 켜는 중입니다. 출발 명령은 보내지 않습니다.')
            base = ('source '+shlex.quote(SETUP)+'; export ROS_DOMAIN_ID=13; '
                    +'export ROPORT_FIELD_CONFIG_DIR='+shlex.quote(REMOTE)+'; '
                    +'export ROPORT_PINKY_CONSOLE_URL='+shlex.quote(CONSOLE)+'; ')
            command = base+'exec python3 '+shlex.quote(REMOTE+'/restart_stack.py')+' --restart --setup '+shlex.quote(SETUP)
            script = 'import subprocess\nsubprocess.run('+repr(['bash','-lc',command])+',check=True)\n'
            try:
                remote(script, timeout=60)
                restart_failed = False
                notify('주행 노드를 다시 켰습니다. 지도 위치·방향을 확인한 뒤 출발 버튼을 누르세요.')
            except (ValueError, OSError, subprocess.SubprocessError):
                restart_failed = True
                notify('노드 재시작 확인이 완료되지 않았습니다. 출발은 잠금 상태입니다.')
                raise
            finally:
                restarting = False
        elif name == '/start':
            if not state['start_enabled']:
                raise ValueError('로봇 정지·구동권 대기 상태를 먼저 확인하세요.')
            launch()
            threading.Thread(target=refresh_start_localization, daemon=True).start()
            notify('출발 요청을 보냈습니다. 현재 위치를 새로 관측하고 H1·방향·IR 확인 후 움직입니다.')
        elif name == '/delivery-complete':
            mission_id = body.get('mission_id')
            if (not state['delivery_enabled'] or mission_id != state.get('mission_id')
                    or not isinstance(mission_id, str) or not re.fullmatch('[0-9a-f]{32}', mission_id)):
                raise ValueError('현재 게이트에서 정지·전달 대기 상태일 때만 복귀할 수 있습니다.')
            if state.get('return_resume_enabled'):
                launch(n12=True, checkpoint=mission_id)
                notify('N12 복귀 재시도를 요청했습니다. 실제 정지 위치·기록을 확인한 뒤 선 정렬부터 재개합니다.')
                return {'success': True, 'message': message}
            packet = json.dumps({'mission_id': mission_id, 'command': 'delivery_complete'})
            remote('from pathlib import Path\nimport os,json\nroot=Path('+repr(REMOTE)+')\np=root/"delivery-command.tmp"\np.write_text('+repr(packet)+')\nos.replace(p,root/"delivery-command.json")\n')
            notify('전달 완료를 확인했습니다. 로봇이 복귀 조건을 확인한 뒤 H1으로 돌아옵니다.')
        else:
            raise ValueError('지원하지 않는 명령')
        return {'success': True, 'message': message}
    finally:
        command_lock.release()



class Handler(BaseHTTPRequestHandler):
    def reply(self, code, kind, data):
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.headers.get('Host') not in ('127.0.0.1:8086', 'localhost:8086'):
            self.reply(403, 'text/plain', b'Invalid Host')
            return
        if self.path == '/':
            self.reply(200, 'text/html; charset=utf-8', (UI_DIR/'index.html').read_bytes())
        elif self.path.startswith('/camera.jpg'):
            import cv2
            import numpy as np
            with lock:
                live = time.monotonic()-camera_at < 1.5
                data = camera_jpeg
            if not live:
                frame = np.zeros((480,640,3), np.uint8)
                cv2.putText(frame, 'CAMERA OFFLINE', (100,240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                data = cv2.imencode('.jpg', frame)[1].tobytes()
            self.reply(200, 'image/jpeg', data)
        elif self.path == '/camera-raw.jpg':
            with lock:
                data = camera_raw if time.monotonic()-camera_at < 1.5 else b''
            self.reply(200 if data else 204, 'image/jpeg', data)
        elif self.path == '/status':
            self.reply(200, 'application/json', json.dumps(snapshot()).encode())
        else:
            self.reply(404, 'text/plain', b'Not found')

    def do_POST(self):
        try:
            size = int(self.headers.get('Content-Length', '0'))
            if (self.headers.get('Host') not in ('127.0.0.1:8086', 'localhost:8086')
                    or self.headers.get('Origin', 'http://127.0.0.1:8086') not in
                        ('http://127.0.0.1:8086', 'http://localhost:8086')
                    or self.headers.get('Content-Type') != 'application/json' or not 0 < size <= 2048):
                raise ValueError('요청 형식 오류')
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('JSON 객체가 필요합니다.')
            result = command(self.path, body)
            code = 200
        except (ValueError, OSError, subprocess.SubprocessError) as exc:
            result, code = {'success': False, 'message': str(exc)}, 400
        self.reply(code, 'application/json', json.dumps(result).encode())

    def log_message(self, *_):
        pass


def main():
    from roport_smartcart.trial_contract import guard_body
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    import cv2, numpy as np, yaml
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('standalone_delivery_ui_readonly')
    def status(msg):
        global latest, observed
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            with lock:
                latest, observed = data, time.monotonic()
        except ValueError:
            pass
    def guard_update(msg):
        global guard, guard_seen
        data = guard_body(msg.data)
        age = time.time()-data.get('sent_at', 0)
        with lock:
            guard = data if data and -.1 <= age <= .3 else {}
            guard_seen = time.monotonic()-max(0., age)
    def odom_update(msg):
        global odom, odom_seen
        with lock:
            odom, odom_seen = msg, time.monotonic()
    calibration = yaml.safe_load((ROOT/'camera_calibration.yaml').read_text())
    k = np.array(calibration['camera_matrix']['data']).reshape(3,3)
    distortion = np.array(calibration['distortion_coefficients']['data'])
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary) if hasattr(cv2.aruco,'ArucoDetector') else None
    previous = [None, 0.]
    def camera_update(msg):
        global camera_jpeg, camera_raw, camera_at, camera_info
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        now = time.monotonic()
        if stamp == previous[0] or now-previous[1]<.2 or abs(node.get_clock().now().nanoseconds/1e9-stamp)>1.5:
            return
        previous[:] = [stamp, now]
        image = cv2.imdecode(np.frombuffer(bytes(msg.data),np.uint8),cv2.IMREAD_COLOR)
        if image is None:
            return
        corners,ids,_ = detector.detectMarkers(image) if detector else cv2.aruco.detectMarkers(image,dictionary)
        detections=[]
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image,corners,ids)
            for points,ident in zip(corners,ids.flatten()):
                ident=int(ident)
                entry={'id':ident}
                if ident in (8,9,10,27):
                    obj=np.array([[-.03,.03,0],[.03,.03,0],[.03,-.03,0],[-.03,-.03,0]],np.float32)
                    points=points.reshape(4,2)
                    ok,rv,tv=cv2.solvePnP(obj,points,k,distortion,flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok and tv[2,0]>0:
                        normal=cv2.Rodrigues(rv)[0][:,2]
                        projection,_=cv2.projectPoints(obj,rv,tv,k,distortion)
                        entry.update(lens_plane_cm=float(abs(normal@tv[:,0])*100),reprojection_px=float(np.sqrt(np.mean(np.sum((projection.reshape(4,2)-points)**2,axis=1)))))
                detections.append(entry)
        cv2.putText(image, time.strftime('%H:%M:%S')+' LIVE', (10,25), cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
        jpeg=cv2.imencode('.jpg',image)[1].tobytes()
        with lock:
            camera_jpeg,camera_raw,camera_at,camera_info=jpeg,bytes(msg.data),now,{'detections':detections}
    node.create_subscription(CompressedImage,'camera/image_raw/compressed',camera_update,qos_profile_sensor_data)
    node.create_subscription(String, 'smartcart/roundtrip_state', status, 1)
    node.create_subscription(String, 'trial/guard_state', guard_update, 1)
    node.create_subscription(Odometry, 'odom', odom_update, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    print('Standalone delivery UI http://127.0.0.1:8086', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8086), Handler).serve_forever()


if __name__ == '__main__':
    main()
