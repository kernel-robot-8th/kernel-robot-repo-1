"""line_nav 상태를 Pinky LCD 글자와 LED 색으로 보여 주는 노드. 구독만 하고 구동 명령은 보내지 않는다.

구독  line_nav/state (std_msgs/String JSON, line_nav 가 발행)
      battery/percent (std_msgs/Float32, pinky_bringup battery_publisher 5 s 주기) → LCD 아래 배터리
LED   led_backend:=led  → pinky_led led_server 의 set_led(fill/clear). 점멸은 이 노드가 켜고 끈다.
      led_backend:=lamp → pinky_lamp_control 의 set_lamp(모드 1 켜기 / 2 점멸, 점멸은 램프가 한다).
      led_backend:=none → LED 안 씀.
LCD   lcd:=true → pinky_emotion 의 LCD 드라이버(SPI)로 직접 그린다. 같은 LCD 를 쓰는
      pinky_emotion emotion_server 와 동시에 실행하지 않는다.

상태 표(문구·색)는 STYLES 하나에만 둔다. 2026-11-07 사용자 요청: 파란 바탕 흰 글자, MOVING 파랑 점멸,
DOCKING 주황 점멸. 나머지 색은 추천값(README).
"""

import json
import signal
import time
from dataclasses import dataclass

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
    'NO_SIGNAL': ('NO SIGNAL', GRAY_BG, (0, 0, 0), None),
}
READY_DIM = 0.25            # READY 흰색은 어둡게(밝기 배율)
BATTERY_STALE_SEC = 15.0    # battery_publisher 5 s 주기의 3배
BATTERY_LOW, BATTERY_MID = 20.0, 40.0   # % 미만이면 빨강·노랑 글자

MOVE_STATES = ('follow', 'pass', 'advance', 'rotate', 'reverse')


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
           battery=False):
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
    big = 64
    while big > 24:                           # 문구가 화면 폭 90% 안에 들어가는 가장 큰 글자
        f = font(big)
        if draw.textlength(status.label, font=f) <= w * 0.9:
            break
        big -= 4
    draw.text((w / 2, h * 0.36), status.label, font=font(big), fill=(255, 255, 255), anchor='mm')
    if status.detail:
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
            self.font_path = str(p('font_path',
                                   '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf').value)
            self.state = None
            self.t_state = None
            self.status = None
            self.blink = Blinker()
            self.led_on = None
            self.lamp_sent = None
            self.pending = None
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

        def on_battery(self, msg):
            if msg.data == msg.data:           # NaN 거름
                self.battery = (float(msg.data), self._now())

        def battery_now(self, now):
            if self.battery is None or now - self.battery[1] > BATTERY_STALE_SEC:
                return None
            return self.battery[0]

        def tick(self):
            now = self._now()
            fresh = self.t_state is not None and now - self.t_state <= self.stale_sec
            status = classify(self.state if fresh else None)
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
                self.lcd.img_show(render(status, font_path=self.font_path, battery=batt))
            except Exception as e:
                self.get_logger().error(f'LCD draw failed: {e}')

        def _ready(self):
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
