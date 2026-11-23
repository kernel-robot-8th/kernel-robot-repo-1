"""line_nav 상태를 Pinky LCD 글자와 LED 색으로 보여 주는 노드. 구독만 하고 구동 명령은 보내지 않는다.

구독  line_nav/state (std_msgs/String JSON, line_nav 가 발행)
      battery/percent (std_msgs/Float32, pinky_bringup battery_publisher 5 s 주기) → LCD 아래 배터리
LED   led_backend:=led  → pinky_led led_server 의 set_led(fill/clear). 점멸은 이 노드가 켜고 끈다.
      led_backend:=lamp → pinky_lamp_control 의 set_lamp(모드 1 켜기 / 2 점멸, 점멸은 램프가 한다).
      led_backend:=none → LED 안 씀.
LCD   lcd:=true → pinky_emotion 의 LCD 드라이버(SPI)로 직접 그린다. 같은 LCD 를 쓰는
      pinky_emotion emotion_server 와 동시에 실행하지 않는다.

스마트카트(smartcart:=true): smartcart/shopping_indicator·roundtrip_state·trial/localization_state 에 더해
      smartcart/boarding_alert (std_msgs/Bool) True 면 탑승 시간 알림(주황 점멸 + 부저 BEEP_EVERY_S 마다, buzzer:=true).
      알림을 언제 켤지는 보내는 쪽이 정한다(이 노드는 받은 값만 보여 줌).

상태 표(문구·색)는 STYLES 하나에만 둔다. 2026-11-07 사용자 요청: 파란 바탕 흰 글자, MOVING 파랑 점멸,
DOCKING 주황 점멸. 나머지 색은 추천값(README).
"""

import json
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

BLUE_BG = (24, 78, 162)
RED_BG = (178, 34, 34)
GRAY_BG = (90, 90, 90)

# 이름: (LCD 문구, LCD 바탕, LED RGB, 점멸 Hz(0 = 고정, None = 끔))
STYLES = {
    'READY':     ('READY',     BLUE_BG, (255, 255, 255), 0.0),
    'MOVING':    ('MOVING',    BLUE_BG, (0, 0, 255), 1.0),
    'DOCKING':   ('DOCKING',   BLUE_BG, (255, 100, 0), 1.0),
    'DOCKED':    ('DOCKED',    BLUE_BG, (255, 100, 0), 0.0),
    'PARKING':   ('PARKING',   BLUE_BG, (0, 255, 0), 1.0),
    'PARKED':    ('PARKED',    BLUE_BG, (0, 255, 0), 0.0),
    'HANDOFF':   ('HANDOFF',   BLUE_BG, (160, 0, 255), 0.0),
    'STOPPED':   ('STOPPED',   RED_BG, (255, 0, 0), 0.0),
    'ERROR':     ('ERROR',     RED_BG, (255, 0, 0), 2.5),
    'CART_READY': ('대기 중', BLUE_BG, (45, 45, 45), 0.0),
    'SHOPPING': ('쇼핑 중', BLUE_BG, (0, 90, 255), 0.0),
    'PAUSED': ('쇼핑 대기', BLUE_BG, (45, 45, 45), 0.0),
    'GATE': ('이동 중', BLUE_BG, (0, 0, 255), 1.0),
    'DELIVERY': ('전달 대기', BLUE_BG, (0, 130, 70), 0.0),
    'COME_CLOSER': ('가까이 와 주세요', (113, 66, 0), (255, 140, 0), 0.0),
    'BOARDING': ('탑승 시간입니다', (113, 66, 0), (255, 140, 0), 2.0),
    'BLOCKED': ('길을 비켜 주세요', RED_BG, (255, 0, 0), 0.0),
    'UNAVAILABLE': ('상태 확인 중', GRAY_BG, (120, 80, 0), 0.0),
    'NO_SIGNAL': ('NO SIGNAL', GRAY_BG, (0, 0, 0), None),
}
READY_DIM = 0.25            # READY 흰색은 어둡게(밝기 배율)
BATTERY_STALE_SEC = 15.0    # battery_publisher 5 s 주기의 3배
BATTERY_LOW, BATTERY_MID = 20.0, 40.0   # % 미만이면 빨강·노랑 글자

MOVE_STATES = ('follow', 'pass', 'advance', 'rotate', 'reverse')

BUZZER_PIN = 4                      # BCM, Pinky Pro 부저. 기체의 배터리 경고(~/ap/battery_buzzer.py, BUZZER_PIN = 4)와 같은 핀·같은 방식
BEEP_PATTERN = ((880, 0.15, 0.08), (1175, 0.25, 0.0))   # (Hz, 켜짐 s, 꺼짐 s) 「삐-삐」
BEEP_EVERY_S = 5.0
FOLLOW_GOAL_ID = 'follow'           # roport_pinky 고객 카드 추종(follow.requested)의 Nav2 goal_id


@dataclass(frozen=True)
class Status:
    name: str
    detail: str = ''

    @property
    def label(self):
        return STYLES[self.name][0]

    @property
    def background(self):
        return STYLES[self.name][1]

    @property
    def led(self):
        rgb = STYLES[self.name][2]
        if self.name == 'READY':
            return tuple(int(c * READY_DIM) for c in rgb)
        return rgb

    @property
    def blink_hz(self):
        return STYLES[self.name][3]


def classify(state):
    """line_nav as_dict() → Status. state 가 None 이면(끊김) NO_SIGNAL."""
    if not state:
        return Status('NO_SIGNAL')
    st = state.get('state') or ''
    leg = state.get('leg') or {}
    action = leg.get('action') or ''
    where = f"{leg.get('from')} -> {leg.get('to')}" if leg.get('to') else ''
    if st == 'error':
        return Status('ERROR', str(state.get('reason') or '')[:40])
    if st == 'stopped':
        return Status('STOPPED', str(state.get('reason') or '')[:40])
    if st == 'loading':                     # 상차 칸 도착·정렬 끝, 상차 완료 대기(2026-11-07 사용자: DOCKED)
        return Status('DOCKED', f"{leg.get('to') or ''} waiting load".strip())
    if st == 'handoff':
        return Status('HANDOFF', str(leg.get('to') or ''))
    if st == 'arrived':
        if action == 'park':
            return Status('PARKED', str(leg.get('to') or ''))
        return Status('READY', str(leg.get('to') or ''))
    if st in MOVE_STATES:
        if action == 'load':
            return Status('DOCKING', where)
        if action == 'park' or st == 'reverse':
            return Status('PARKING', where)
        return Status('MOVING', where)
    return Status('READY')


SHOPPING_STATES = ('SHOPPING', 'COME_CLOSER', 'BLOCKED', 'PAUSED', 'UNAVAILABLE')


# 탑승 시간 알림보다 먼저 보여 줄 상태: 막힘과 명시적인 정지·이상(알림이 정지·오류를 가리지 않게)
BOARDING_YIELDS_TO = ('BLOCKED', 'PAUSED', 'UNAVAILABLE', 'STOPPED', 'ERROR')


def cart_status(shop, trip, navigation, line, boarding=False):
    """Passenger display. Stops and faults win over the boarding alert; the alert wins over the rest."""
    status = _cart_base(shop, trip, navigation, line)
    if boarding is True and status.name not in BOARDING_YIELDS_TO:
        return Status('BOARDING')
    return status


def _cart_base(shop, trip, navigation, line):
    """Prefer active missions over idle telemetry. A verified terminal blockage wins."""
    trip = trip or {}
    nav = navigation or {}
    recovery_exhausted = (nav.get('error_code') in ('nav2_recovery_timeout', 'nav2_recovery_limit')
                          and isinstance(nav.get('recoveries'), int) and nav['recoveries'] > 0)
    no_path = nav.get('nav2_error_code') in (104, 105, 106, 205, 206, 208)
    if shop == 'BLOCKED' or trip.get('path_blocked') is True or (
            nav.get('phase') == 'failed' and (recovery_exhausted or no_path)):
        return Status('BLOCKED', '카트가 멈춰 있어요')
    if shop in SHOPPING_STATES:
        return Status(shop, '카트 앞에 서 주세요' if shop == 'COME_CLOSER' else '')
    if nav.get('goal_id') == FOLLOW_GOAL_ID and nav.get('phase') in ('navigating', 'recovering'):
        return Status('SHOPPING')           # 고객 카드 추종도 고객을 따라가는 쇼핑 표시
    goal = trip.get('goal') or nav.get('goal_id')
    if isinstance(goal, str) and goal.startswith('demo_'):
        goal = goal[len('demo_'):]          # 시험 실행기 목표(demo_ARR → ARR)
    dest = 'H1' if trip.get('direction') == 'RETURN' else goal
    if trip.get('phase') == 'WAIT_DELIVERY':
        return Status('DELIVERY', str(goal or ''))
    if trip.get('phase') == 'STOPPED' or nav.get('phase') == 'failed':
        return Status('UNAVAILABLE')
    if trip.get('phase') not in (None, 'READY', 'DONE') or nav.get('phase') in ('navigating','recovering'):
        suffix = '으로 이동 중' if dest in ('G1','G3','H1') else '로 이동 중'
        return Status('GATE', dest + suffix if dest in ('G1','G2','G3','ARR','N12','H1','H2') else '')
    standard = classify(line)
    return Status('CART_READY') if standard.name == 'READY' or (
        standard.name == 'NO_SIGNAL' and (trip or shop == 'INACTIVE' or navigation is not None)) else standard


class BeepSchedule:
    """탑승 알림 부저: 알림이 켜질 때 한 번, 켜져 있는 동안 every_s 마다. due(now, on) 이 True 면 울린다."""

    def __init__(self, every_s=BEEP_EVERY_S):
        self.every_s, self.next_at = every_s, None

    def due(self, now, on):
        if not on:
            self.next_at = None
            return False
        if self.next_at is None or now >= self.next_at:
            self.next_at = now + self.every_s
            return True
        return False


def beep_once():
    """새 프로세스에서 한 번 울리고 GPIO 를 반납한다(battery_buzzer.py 와 같은 핀을 계속 잡지 않음).
    울리는 동안(약 0.5 s) 이 노드를 막지 않는다. 부저를 못 잡으면 조용히 끝난다."""
    code = ('import sys,time\n'
            'import RPi.GPIO as G\n'
            f'pin={BUZZER_PIN}; pat={BEEP_PATTERN!r}\n'
            'try:\n'
            '    G.setmode(G.BCM); G.setwarnings(False); G.setup(pin, G.OUT); p=G.PWM(pin, 1000); p.start(0)\n'
            'except Exception:\n'
            '    sys.exit(3)\n'
            'try:\n'
            '    for f,on,off in pat:\n'
            '        p.ChangeFrequency(f); p.ChangeDutyCycle(50); time.sleep(on); p.ChangeDutyCycle(0); time.sleep(off)\n'
            'finally:\n'
            '    p.stop(); G.cleanup(pin)\n')
    try:
        return subprocess.Popen([sys.executable, '-I', '-c', code], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except OSError:
        return None


class Blinker:
    """점멸 켜짐/꺼짐 판정. hz 0 = 계속 켜짐, None = 계속 꺼짐."""

    def __init__(self):
        self.t0 = 0.0
        self.hz = None

    def reset(self, hz, now):
        self.hz, self.t0 = hz, now

    def on(self, now):
        if self.hz is None:
            return False
        if self.hz <= 0:
            return True
        return int((now - self.t0) * self.hz * 2) % 2 == 0


def render(status, size=(320, 240), font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
           battery=False, face=None):
    """LCD 화면 이미지(PIL). 큰 상태 문구 + 작은 줄(구간·이유) + 맨 아래 배터리.

    battery: False = 배터리 줄 없음, None = 값 없음(--), 숫자 = %.
    """
    from PIL import Image, ImageDraw, ImageFont

    img = Image.new('RGB', size, status.background)
    draw = ImageDraw.Draw(img)

    def font(px):
        try:
            return ImageFont.truetype(font_path, px)
        except OSError:
            return ImageFont.load_default(size=px)

    w, h = size
    show_face = face is not None and status.name in ('CART_READY', 'SHOPPING', 'GATE', 'PAUSED', 'DELIVERY')
    if show_face:
        frame = face.copy()
        frame.thumbnail((w, round(h*.53)))
        img.paste(frame, ((w-frame.width)//2, 0))
    label = status.detail if status.name == 'GATE' and status.detail else status.label
    big = 44 if show_face else 64
    while big > 24:                           # 문구가 화면 폭 90% 안에 들어가는 가장 큰 글자
        f = font(big)
        if draw.textlength(label, font=f) <= w * 0.9:
            break
        big -= 4
    draw.text((w / 2, h * (.64 if show_face else .36)), label, font=font(big), fill=(255, 255, 255), anchor='mm')
    if status.detail and label != status.detail:
        small, limit = font(22), w * 0.92
        lines, cur = [], ''
        for word in status.detail.split():                # 길면 두 줄로(넘치는 끝은 자름)
            trial = f'{cur} {word}'.strip()
            if cur and draw.textlength(trial, font=small) > limit:
                lines.append(cur)
                cur = word
            else:
                cur = trial
        lines = (lines + [cur])[:2]
        for i, text in enumerate(lines):
            while text and draw.textlength(text, font=small) > limit:
                text = text[:-1]
            y = h * (0.66 if len(lines) == 1 else 0.62 + 0.11 * i)
            draw.text((w / 2, y), text, font=small, fill=(230, 236, 255), anchor='mm')
    if battery is not False:
        _draw_battery(draw, w, h, battery, font(24), status.background)
    return img


def battery_text(percent):
    return '--' if percent is None else f'{max(0, min(100, round(percent)))}%'


def _draw_battery(draw, w, h, percent, f, background=BLUE_BG):
    """맨 아래 가운데: 배터리 그림(채움 = 잔량) + 숫자. 낮으면 노랑·빨강."""
    if percent is None:
        color = (200, 200, 200)
    elif percent < BATTERY_LOW:
        color = (255, 235, 0) if background == RED_BG else (255, 90, 90)   # 빨간 바탕에서는 노랑
    elif percent < BATTERY_MID:
        color = (255, 210, 60)
    else:
        color = (255, 255, 255)
    text = battery_text(percent)
    tw = draw.textlength(text, font=f)
    bw, bh, gap = 40, 20, 10
    x0 = (w - (bw + 4 + gap + tw)) / 2
    y0 = h - 16 - bh
    draw.rectangle((x0, y0, x0 + bw, y0 + bh), outline=color, width=2)
    draw.rectangle((x0 + bw, y0 + 6, x0 + bw + 4, y0 + bh - 6), fill=color)
    if percent is not None:
        fill_w = (bw - 6) * max(0.0, min(100.0, percent)) / 100.0
        if fill_w > 0:
            draw.rectangle((x0 + 3, y0 + 3, x0 + 3 + fill_w, y0 + bh - 3), fill=color)
    draw.text((x0 + bw + 4 + gap, y0 + bh / 2), text, font=f, fill=color, anchor='lm')


def make_node(**kwargs):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class StatusIndicator(Node):
        def __init__(self):
            super().__init__('status_indicator', **kwargs)
            p = self.declare_parameter
            self.backend = str(p('led_backend', 'led').value)
            if self.backend not in ('led', 'lamp', 'none'):
                raise ValueError("led_backend must be led, lamp or none")
            self.use_lcd = bool(p('lcd', True).value)
            self.stale_sec = float(p('stale_sec', 2.0).value)
            self.cart = bool(p('smartcart', False).value)
            self.use_buzzer = bool(p('buzzer', True).value)
            self.beeps = BeepSchedule()
            self.beeper = beep_once
            self.font_path = str(p('font_path',
                                   '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc' if self.cart else
                                   '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf').value)
            self.inputs = {}
            self.face = None
            if self.cart:
                for topic, key in [('smartcart/shopping_indicator', 'shop'),
                                   ('smartcart/roundtrip_state', 'trip'), ('trial/localization_state', 'navigation')]:
                    self.create_subscription(String, topic, lambda msg, k=key: self.on_cart(k, msg), 1)
                from std_msgs.msg import Bool
                self.create_subscription(Bool, 'smartcart/boarding_alert', self.on_boarding, 1)
                if self.use_lcd:
                    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
                    from PIL import Image
                    try:
                        path = Path(get_package_share_directory('pinky_emotion')) / 'emotion' / 'basic.gif'
                        with Image.open(path) as face:
                            self.face = face.convert('RGB')
                    except (OSError, PackageNotFoundError) as exc:
                        self.get_logger().warn(f'Pinky face unavailable: {exc}')
            self.state = None
            self.t_state = None
            self.status = None
            self.blink = Blinker()
            self.led_on = None
            self.lamp_sent = None
            self.pending = None
            self.retry_led_at = 0.
            self.warned = set()
            self.lcd = None
            if self.use_lcd:
                try:
                    from pinky_emotion.pinky_lcd import LCD
                    self.lcd = LCD()
                except Exception as e:      # 노트북·LCD 없는 기체: LED 만 쓴다
                    self.get_logger().error(f'LCD unavailable, LED only: {e}')
            self.client = None
            if self.backend != 'none':
                from pinky_interfaces.srv import SetLamp, SetLed
                self.srv_type = SetLed if self.backend == 'led' else SetLamp
                name = 'set_led' if self.backend == 'led' else 'set_lamp'
                self.client = self.create_client(self.srv_type, name)
            self.create_subscription(String, 'line_nav/state', self.on_state, 10)
            from std_msgs.msg import Float32
            self.battery = None          # (percent, monotonic)
            self.shown_battery = False
            self.create_subscription(Float32, 'battery/percent', self.on_battery, 10)
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f'status_indicator: lcd={self.lcd is not None} '
                                   f'led_backend={self.backend}')

        def _now(self):
            return time.monotonic()

        def on_state(self, msg):
            try:
                self.state = json.loads(msg.data)
            except ValueError:
                return
            self.t_state = self._now()

        def on_cart(self, key, msg):
            try:
                value = msg.data if key == 'shop' else json.loads(msg.data)
                if key == 'shop' and value not in (*SHOPPING_STATES, 'INACTIVE'):
                    return
                if key != 'shop' and not isinstance(value, dict):
                    return
                if key == 'navigation':
                    value = value.get('navigation')
                    if value is not None and not isinstance(value, dict):
                        return
                # A finished field runner exits after its terminal report. Keep a
                # confirmed blocked trip visible until the next explicit mission.
                if (key == 'shop' and value == 'SHOPPING' or key == 'navigation' and value
                        and value.get('phase') == 'navigating'):
                    previous = self.inputs.get('trip')
                    if previous and previous[0] == float('inf'):
                        del self.inputs['trip']
                terminal_block = key == 'trip' and value.get('phase') == 'STOPPED' and value.get('path_blocked') is True
                self.inputs[key] = (float('inf') if terminal_block else self._now(), value)
            except (ValueError, TypeError):
                return

        def on_boarding(self, msg):
            self.inputs['boarding'] = (self._now(), bool(msg.data))

        def on_battery(self, msg):
            if msg.data == msg.data:           # NaN 거름
                self.battery = (float(msg.data), self._now())

        def battery_now(self, now):
            if self.battery is None or now - self.battery[1] > BATTERY_STALE_SEC:
                return None
            return self.battery[0]

        def tick(self):
            now = self._now()
            if self.cart and self.pending is not None and self.pending.done():
                try:
                    response = self.pending.result()
                    applied = response.success if self.backend == 'led' else response.result
                except (RuntimeError, AttributeError):
                    applied = False
                self.pending = None
                if not applied:
                    self.led_on = self.lamp_sent = None
                    self.retry_led_at = now + 1.
                    self.get_logger().warn('LED command failed; retrying', throttle_duration_sec=5.)
            fresh = self.t_state is not None and now - self.t_state <= self.stale_sec
            line = self.state if fresh else None
            if self.cart:
                current = {key: val for key, (stamp, val) in self.inputs.items() if now-stamp <= self.stale_sec}
                status = cart_status(current.get('shop'), current.get('trip'), current.get('navigation'), line,
                                     current.get('boarding') is True)
                if self.t_state is None and not (self.inputs.keys() - {'boarding'}):
                    status = Status('CART_READY') if status.name != 'BOARDING' else status
                if self.beeps.due(now, status.name == 'BOARDING') and self.use_buzzer:
                    self.beeper()
            else:
                status = classify(line)
            if status != self.status:
                self.status = status
                self.blink.reset(status.blink_hz, now)
                self.led_on = None
                self.get_logger().info(f'{status.label} {status.detail}')
                self.draw(status, now)
            elif battery_text(self.battery_now(now)) != self.shown_battery:
                self.draw(status, now)                  # 배터리 숫자만 바뀌어도 다시 그림
            if self.backend == 'lamp' and self.lamp_sent != status:
                self.lamp(status)                       # 서비스가 늦게 떠도 다음 tick 에 다시
            if self.backend == 'led':
                on = self.blink.on(now)
                if on != self.led_on:
                    self.led(status.led if on else None, on)

        def draw(self, status, now):
            batt = self.battery_now(now)
            self.shown_battery = battery_text(batt)
            if self.lcd is None:
                return
            try:
                self.lcd.img_show(render(status, font_path=self.font_path, battery=batt, face=self.face))
            except Exception as e:
                self.get_logger().error(f'LCD draw failed: {e}')

        def _ready(self):
            if self._now() < self.retry_led_at:
                return False
            if not self.client.service_is_ready():
                if 'srv' not in self.warned:
                    self.get_logger().warn(f'{self.client.srv_name} not available yet '
                                           '(pinky_led led_server / pinky_lamp_control)')
                    self.warned.add('srv')
                return False
            if self.pending is not None and not self.pending.done():
                return False
            return True

        def led(self, rgb, on):
            if not self._ready():
                return
            req = self.srv_type.Request()
            if rgb is None or rgb == (0, 0, 0):
                req.command = 'clear'
            else:
                req.command = 'fill'
                req.r, req.g, req.b = rgb
            self.pending = self.client.call_async(req)
            self.led_on = on

        def lamp(self, status):
            if not self._ready():
                return
            req = self.srv_type.Request()
            r, g, b = status.led
            req.color.r, req.color.g, req.color.b, req.color.a = r / 255, g / 255, b / 255, 1.0
            hz = status.blink_hz
            if hz is None:
                req.mode, req.time = 0, 0
            elif hz <= 0:
                req.mode, req.time = 1, 0
            else:
                req.mode, req.time = 2, int(500 / hz)       # 반주기 ms
            self.pending = self.client.call_async(req)
            self.lamp_sent = status

        def shutdown_outputs(self):
            """종료 때 LED 끄고 LCD 를 검게(LED 응답은 최대 1 s 기다림)."""
            try:
                if self.client is not None and self.client.service_is_ready():
                    req = self.srv_type.Request()
                    if self.backend == 'led':
                        req.command = 'clear'
                    else:
                        req.mode, req.time = 0, 0
                    rclpy.spin_until_future_complete(self, self.client.call_async(req),
                                                     timeout_sec=1.0)
                if self.lcd is not None:
                    self.lcd.clear()
                    self.lcd.close()
            except Exception as e:
                self.get_logger().warn(f'shutdown outputs: {e}')

    return StatusIndicator()


def main(args=None):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    # Ctrl+C 뒤에도 LED 끄기 서비스를 부를 수 있게 rclpy 기본 SIGINT 처리를 끈다
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def stop(*_):
        raise KeyboardInterrupt

    # 백그라운드(nohup ... &)로 띄우면 SIGINT 가 무시된 채 넘어온다. INT·TERM 모두 정상 종료로 받는다
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = make_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
