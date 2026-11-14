"""Supervised, low-speed paper-doll following demo through trial_guard."""

import argparse
import json
import math
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from queue import Empty, SimpleQueue
from threading import Thread
from uuid import uuid4


PAGE = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pinky doll follow</title>
<style>body{font:16px system-ui;background:#111827;color:#f9fafb;max-width:980px;margin:24px auto;padding:0 16px}
img{width:100%;background:#000;border-radius:12px} .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
.card{background:#1f2937;padding:14px;border-radius:10px}.value{font-size:23px;font-weight:700}
button{font:inherit;padding:12px 18px;margin:12px 8px 8px 0;border:0;border-radius:8px;cursor:pointer}
button.stop{background:#ef4444;color:white} button:disabled{opacity:.45;cursor:default}
button.emergency{position:fixed;right:16px;bottom:16px;z-index:10;box-shadow:0 2px 12px #000}
#state{font-size:22px;font-weight:700}</style>
<h1>Pinky 인형 추종 시험</h1><p id="state">연결 중…</p>
<div class="card"><strong>핑키 초기 준비</strong><p id="setup-message">상태 확인 중…</p>
<button data-setup onclick="act('robot/check')">① 연결 확인</button>
<button data-setup onclick="act('robot/sensors')">② 센서·카메라 켜기</button>
<button data-setup onclick="act('robot/bringup')">③ 브링업 켜기 · 모터 초기화</button>
<button id="shutdown" class="stop" onclick="act('robot/stop_all')">전체 정지 · 브링업 끄기</button>
<p id="setup-nodes">—</p><small>핑키를 빈 바닥에 놓고 옆에서 조작하세요. 브링업을 켜도 추종은 시작하지 않습니다.</small></div>
<img id="camera" alt="핑키 카메라 영상">
<p><button onclick="act('calibrate')">현재 거리 기준 저장</button>
<button id="follow-start" onclick="act('start')">따라가기 시작</button>
<button class="stop emergency" onclick="act('stop')">즉시 정지</button></p>
<p>인형을 현재 위치에서 기준 저장한 뒤 10~15cm 뒤로 옮기고 시작하세요. 한 번에 최대 30초, 전진 최고 2.5cm/s.</p>
<div class="grid"><div class="card">인형 인식<div class="value" id="target">—</div></div>
<div class="card">주행 준비<div class="value" id="readiness">—</div></div>
<div class="card">주행 가드<div class="value" id="guard">—</div></div>
<div class="card">초음파 · 정지 5cm 이하<div class="value" id="sonar">—</div></div>
<div class="card">저장한 목표 거리<div class="value" id="goal_range">—</div></div>
<div class="card">정면 라이다 · 정지 25cm 미만<div class="value" id="lidar">—</div></div>
<div class="card">전진 속도<div class="value" id="linear">—</div></div>
<div class="card">회전 속도<div class="value" id="angular">—</div></div></div>
<script>
const reasonText={
'PLACE DOLL AT DESIRED DISTANCE; PRESS C':'인형이 보이면 현재 거리 기준을 저장하세요',
'TARGET NOT VISIBLE':'인형이 보이지 않아 기준 저장 실패',
'MOVE DOLL CLOSER':'인형을 조금 가까이 두세요',
'MOVE DOLL BEYOND 5 CM':'인형을 5cm보다 멀리 두세요',
'CALIBRATED; PRESS SPACE TO FOLLOW':'기준 저장 완료 · 인형을 뒤로 옮긴 뒤 시작하세요',
'CALIBRATE VISIBLE TARGET FIRST':'인형이 보일 때 기준 저장부터 하세요',
'FOLLOWING':'인형 추종 중','AT CALIBRATED DISTANCE':'기준 거리 도착',
'TARGET TEMPORARILY LOST':'인형 잠시 미검출 · 정지',
'TARGET LOST; STOPPED':'인형을 놓쳐 정지',
'CAMERA STALE; STOPPED':'카메라 데이터 지연 · 정지',
'RANGE SENSOR STALE; STOPPED':'거리 센서 지연 · 정지',
'SONAR <=5 CM; STOPPED':'초음파 5cm 이하 · 정지',
'LIDAR <25 CM; STOPPED':'라이다 25cm 미만 · 정지',
'30 SECOND LIMIT; STOPPED':'30초 제한으로 자동 정지',
'DRIVE GUARD / SENSOR NOT READY':'주행 가드 또는 센서 이상 · 정지',
'OPERATOR STOPPED':'사용자 정지','STOPPED':'정지'};
async function act(name){await fetch('/'+name,{method:'POST'});refresh()}
async function refresh(){try{const s=await(await fetch('/status')).json();
document.getElementById('state').textContent=(reasonText[s.reason]||s.reason)+(s.armed?' · 주행 중':' · 정지 중');
document.getElementById('camera').style.opacity=s.camera_live?'1':'.3';
const p=s.setup||{};
document.getElementById('setup-message').textContent=p.message||'상태 확인 중…';
document.querySelectorAll('[data-setup]').forEach(b=>b.disabled=!p.enabled||p.busy||s.armed);
document.getElementById('shutdown').disabled=!p.enabled;
document.getElementById('follow-start').disabled=p.busy||s.readiness!=='준비 완료'||s.goal_range==='미설정';
document.getElementById('setup-nodes').textContent=Object.entries({bringup:'모터·라이다',adc:'초음파',camera:'카메라',guard:'주행 가드'}).map(([k,n])=>n+': '+(p.nodes&&k in p.nodes?(p.nodes[k]?'실행 중':'꺼짐'):'미확인')).join(' / ');
for(const k of ['target','readiness','guard','sonar','goal_range','lidar','linear','angular'])
document.getElementById(k).textContent=s[k]??'—';}catch(e){document.getElementById('state').textContent='연결 끊김 · 핑키 전원으로 정지하세요'}}
setInterval(refresh,300);setInterval(()=>document.getElementById('camera').src='/frame.jpg?t='+Date.now(),500);refresh();
</script></html>'''.encode('utf-8')


class FollowLogic:
    """Image-size following; stop on stale input, loss, or a near obstacle."""

    def __init__(self, label, reference_heights=None):
        self.label = label
        self.reference_heights = reference_heights or {}
        self.goal_height = None
        self.goal_range = None
        self.armed = False
        self.target = None
        self.width = 0
        self.frame_at = None
        self.seen_at = None
        self.started_at = None
        self.reason = 'PLACE DOLL AT DESIRED DISTANCE; PRESS C'

    def observe(self, detections, width, now):
        hits = [d for d in detections if d['label'] == self.label]
        self.target = hits[0] if len(hits) == 1 else None
        self.width, self.frame_at = width, now
        if self.target is not None:
            self.seen_at = now

    def calibrate(self, range_m):
        if self.target is None or self.frame_at is None or time.monotonic() - self.frame_at > .4:
            self.reason = 'TARGET NOT VISIBLE'
            return False
        if range_m is None or not math.isfinite(range_m) or range_m <= .05:
            self.reason = 'MOVE DOLL BEYOND 5 CM'
            return False
        if self.height(self.target) < 25:
            self.reason = 'MOVE DOLL CLOSER'
            return False
        self.goal_height = self.apparent_scale(self.target)
        self.goal_range = range_m
        self.reason = 'CALIBRATED; PRESS SPACE TO FOLLOW'
        return True

    @staticmethod
    def height(target):
        return max(p[1] for p in target['quad']) - min(p[1] for p in target['quad'])

    def apparent_scale(self, target):
        return self.height(target) / self.reference_heights.get(target['view'], 1)

    def arm(self, now):
        if self.goal_height is None or self.target is None:
            self.reason = 'CALIBRATE VISIBLE TARGET FIRST'
            return False
        self.armed, self.started_at = True, now
        self.reason = 'FOLLOWING'
        return True

    def disarm(self, reason='STOPPED'):
        self.armed = False
        self.reason = reason

    def command(self, now, sonar, scan):
        if not self.armed:
            return 0., 0.
        if self.started_at is None or now - self.started_at > 30:
            self.disarm('30 SECOND LIMIT; STOPPED')
        elif self.frame_at is None or now - self.frame_at > .4:
            self.disarm('CAMERA STALE; STOPPED')
        elif (sonar is None or scan is None or now - sonar[1] > .4
              or now - scan[1] > .4):
            self.disarm('RANGE SENSOR STALE; STOPPED')
        elif sonar[0] <= .05:
            self.reason = 'SONAR <=5 CM; STOPPED'
            return 0., 0.
        elif scan[0] < .25:
            self.reason = 'LIDAR <25 CM; STOPPED'
            return 0., 0.
        elif self.target is None:
            if self.seen_at is None or now - self.seen_at > 2.:
                self.disarm('TARGET LOST; STOPPED')
            else:
                self.reason = 'TARGET TEMPORARILY LOST'
            return 0., 0.
        if not self.armed:
            return 0., 0.
        quad = self.target['quad']
        center = sum(p[0] for p in quad) / 4
        error = (center - self.width / 2) / (self.width / 2)
        angular = max(-.25, min(.25, -.35 * error)) if abs(error) > .1 else 0.
        linear = (.025 if self.apparent_scale(self.target) < self.goal_height
                  and sonar[0] > self.goal_range and abs(error) < .3 else 0.)
        self.reason = 'FOLLOWING' if linear or angular else 'AT CALIBRATED DISTANCE'
        return linear, angular


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--references', required=True)
    parser.add_argument('--target', required=True, choices='ABC')
    parser.add_argument('--robot-host', help='SSH user@hostname for explicit setup buttons')
    args = parser.parse_args(argv)

    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, LaserScan, Range
    from std_msgs.msg import String
    from .doll_recognition import DollRecognizer
    from .doll_robot_setup import RobotSetup

    cv2.setNumThreads(1)
    detector = DollRecognizer(args.references)
    reference_heights = {view: shape[0] for label, view, shape, *_ in detector.references
                         if label == args.target}
    logic = FollowLogic(args.target, reference_heights)
    robot_setup = RobotSetup(args.robot_host)
    rclpy.init()
    node = rclpy.create_node('smartcart_doll_follow_demo')
    velocity = node.create_publisher(Twist, 'line_nav/cmd_vel', 1)
    lease = node.create_publisher(String, 'trial/lease', 1)
    session, seq = uuid4().hex, 0
    guard, guard_at, boot = {}, None, None
    sonar = scan = None
    frame = None
    owned = False
    controls = SimpleQueue()
    web_status = {'reason': 'STARTING', 'armed': False}
    web_frame = None

    def guard_update(msg):
        nonlocal guard, guard_at
        try:
            guard = json.loads(msg.data)
            guard_at = time.monotonic()
        except (ValueError, TypeError):
            guard = {}

    def camera_update(msg):
        nonlocal frame
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -.1 <= age <= .4:
            return
        captured_at = time.monotonic() - max(0., age)
        if logic.frame_at is not None and captured_at - logic.frame_at < .16:
            return
        decoded = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            return
        detections = detector.detect(decoded)
        logic.observe(detections, decoded.shape[1], captured_at)
        frame = decoded
        for detection in detections:
            quad = np.int32(detection['quad'])
            color = (0, 220, 0) if detection['label'] == args.target else (0, 180, 255)
            cv2.polylines(frame, [quad], True, color, 2)
            x, y = quad.min(axis=0)
            cv2.putText(frame, detection['label'], (int(x), max(55, int(y))),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)

    def sonar_update(msg):
        nonlocal sonar
        if (math.isfinite(msg.range) and math.isfinite(msg.min_range)
                and math.isfinite(msg.max_range) and msg.min_range <= msg.range <= msg.max_range):
            sonar = (msg.range, time.monotonic())

    def scan_update(msg):
        nonlocal scan
        if (not math.isfinite(msg.angle_increment) or msg.angle_increment == 0
                or not math.isfinite(msg.range_min) or not math.isfinite(msg.range_max)
                or msg.range_min >= msg.range_max):
            return
        rays = [distance for i, distance in enumerate(msg.ranges)
                if abs(msg.angle_min + i * msg.angle_increment) <= math.pi / 6]
        valid = [distance for distance in rays
                 if distance == math.inf or math.isfinite(distance) and distance >= msg.range_min]
        front = [distance for distance in rays
                 if math.isfinite(distance) and msg.range_min <= distance < msg.range_max]
        if valid:
            scan = (min(front, default=math.inf), time.monotonic())

    node.create_subscription(String, 'trial/guard_state', guard_update, 10)
    node.create_subscription(CompressedImage, 'camera/image_raw/compressed', camera_update,
                             qos_profile_sensor_data)
    node.create_subscription(Range, 'us_sensor/range', sonar_update, qos_profile_sensor_data)
    node.create_subscription(LaserScan, 'scan', scan_update, qos_profile_sensor_data)

    def single_publisher(topic, name):
        publishers = node.get_publishers_info_by_topic(topic)
        return len(publishers) == 1 and publishers[0].node_name == name

    def readiness_issue():
        now = time.monotonic()
        if robot_setup.snapshot()['busy']:
            return '핑키 준비 작업 중'
        if guard_at is None or now - guard_at >= .3:
            return '주행 가드 상태 지연'
        if not guard.get('boot') or guard.get('blocked') or guard.get('owner') not in ('none', 'line'):
            return '주행 가드 차단 또는 점유'
        if sonar is None or now - sonar[1] >= .4:
            return '초음파 데이터 지연'
        if scan is None or now - scan[1] >= .4:
            return '라이다 데이터 지연'
        if not single_publisher('cmd_vel', 'trial_guard'):
            return '모터 명령 발행자 충돌'
        if not single_publisher('trial/lease', node.get_name()):
            return '주행 lease 발행자 충돌'
        if not single_publisher('line_nav/cmd_vel', node.get_name()):
            return '주행 명령 발행자 충돌'
        return ''

    def ready():
        return not readiness_issue()

    def send(owner, linear=0., angular=0.):
        nonlocal seq, owned
        if boot is None:
            return
        seq += 1
        lease.publish(String(data=json.dumps(dict(
            boot=boot, owner=owner, seq=seq, session=session, sent_at=time.time()))))
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        velocity.publish(command)
        owned = owner == 'line'

    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, kind, body):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                self.respond(200, 'text/html; charset=utf-8', PAGE)
            elif path == '/status':
                self.respond(200, 'application/json', json.dumps(web_status).encode())
            elif path == '/frame.jpg' and web_frame is not None:
                self.respond(200, 'image/jpeg', web_frame)
            else:
                self.respond(404, 'text/plain', b'Not found')

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin and origin not in ('http://127.0.0.1:8082', 'http://localhost:8082'):
                self.respond(403, 'text/plain', b'Forbidden')
            elif self.path in ('/calibrate', '/start', '/stop', '/robot/check',
                               '/robot/sensors', '/robot/bringup', '/robot/stop_all'):
                controls.put(self.path[1:])
                self.respond(202, 'text/plain', b'Accepted')
            else:
                self.respond(404, 'text/plain', b'Not found')

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 8082), Handler)
    Thread(target=server.serve_forever, daemon=True).start()
    print('Pinky doll follow UI: http://127.0.0.1:8082', flush=True)
    next_tick = time.monotonic()
    next_preview = next_tick
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            try:
                while True:
                    action = controls.get_nowait()
                    if action.startswith('robot/'):
                        logic.disarm('OPERATOR STOPPED')
                        if boot is not None:
                            send('none')
                        if action == 'robot/stop_all':
                            boot = None
                            logic.goal_height = logic.goal_range = None
                        robot_setup.submit(action.split('/')[1])
                    elif action == 'calibrate':
                        logic.calibrate(sonar[0] if sonar and now - sonar[1] < .4 else None)
                    elif action == 'stop':
                        logic.disarm('OPERATOR STOPPED')
                        if owned:
                            send('none')
                    elif action == 'start':
                        issue = readiness_issue()
                        if (not issue and guard.get('owner') == 'none'
                                and logic.frame_at is not None and now - logic.frame_at < .4):
                            boot = guard['boot']
                            logic.arm(now)
                        else:
                            logic.reason = issue or '카메라 지연 또는 가드 점유'
            except Empty:
                pass
            if now >= next_tick:
                next_tick = now + .1
                linear = angular = 0.
                if logic.armed and (not ready() or guard.get('boot') != boot):
                    logic.disarm('DRIVE GUARD / SENSOR NOT READY')
                if logic.armed:
                    linear, angular = logic.command(now, sonar, scan)
                    if logic.armed:
                        send('line', linear, angular)
                if not logic.armed and boot is not None:
                    send('none')
                web_status = {
                    'reason': logic.reason, 'armed': logic.armed,
                    'target': ((logic.target['label'] + ' · ' + logic.target['view']
                                if logic.target else '미검출')
                               if logic.frame_at is not None and now - logic.frame_at < .4
                               else '카메라 꺼짐/지연'),
                    'camera_live': logic.frame_at is not None and now - logic.frame_at < .4,
                    'guard': (f"{guard.get('owner', '없음')} / {'차단' if guard.get('blocked') else '정상'}"
                              if guard_at is not None and now - guard_at < .3 else '꺼짐/지연'),
                    'sonar': f'{sonar[0] * 100:.1f} cm' if sonar and now - sonar[1] < .4 else '꺼짐/지연',
                    'goal_range': (f'{logic.goal_range * 100:.1f} cm'
                                   if logic.goal_range is not None else '미설정'),
                    'lidar': f'{scan[0] * 100:.1f} cm' if scan and now - scan[1] < .4 else '꺼짐/지연',
                    'linear': f'{linear * 100:.1f} cm/s',
                    'angular': f'{angular:.2f} rad/s',
                    'readiness': readiness_issue() or '준비 완료',
                    'setup': robot_setup.snapshot(),
                }
            if now >= next_preview:
                next_preview = now + .2
                image = frame.copy() if frame is not None else np.zeros((480, 640, 3), np.uint8)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    web_frame = encoded.tobytes()
    except KeyboardInterrupt:
        pass
    finally:
        logic.disarm()
        if owned:
            for _ in range(3):
                send('none')
                time.sleep(.05)
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
