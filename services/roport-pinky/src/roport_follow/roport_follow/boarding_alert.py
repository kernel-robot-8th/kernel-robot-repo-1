"""노트북 인식 도구에서 로봇 표시(LCD·LED·부저)로 탑승 시간 알림을 켜고 끈다(tool --boarding-alert, 기본 끔).

패드를 띄운 노트북이 판단한 알림을 smartcart/boarding_alert(std_msgs/Bool)로 내보낸다. 받는 쪽은 로봇의
roport_line_nav status_indicator(smartcart:=true)다. 드라이버 hook(roport_pinky set_boarding_alert)과 같은 토픽·주기다.
- 켜져 있는 동안 PERIOD_S 마다 True(status_indicator 는 2 s 동안 안 오면 끈다), 끌 때 False 한 번.
- MAX_S 뒤에는 저절로 끈다(패드가 꺼져서 끄기 요청이 안 와도 계속 울리지 않게). 도구가 끝날 때도 끈다.
- 모터·주행 명령과는 무관하다. 로봇과 같은 ROS_DOMAIN_ID 여야 닿는다.
"""
from __future__ import annotations

import threading
import time

TOPIC = 'smartcart/boarding_alert'
PERIOD_S = 0.5
MAX_S = 60.0


class BoardingAlert:
    """publish(bool) 로 알림을 내보낸다. ROS 없이 시험할 수 있게 발행 함수만 받는다."""

    def __init__(self, publish, period_s=PERIOD_S, max_s=MAX_S, clock=time.monotonic):
        self.publish, self.period_s, self.max_s, self.clock = publish, period_s, max_s, clock
        self.lock = threading.Lock()
        self.on, self.since = False, None
        self.stop_event, self.thread = threading.Event(), None

    def set(self, on):
        """켜기·끄기. bool 이 아니면 ValueError('invalid_alert'). 지금 상태(bool)를 돌려준다."""
        if not isinstance(on, bool):
            raise ValueError('invalid_alert')
        with self.lock:
            if on:
                self.on, self.since = True, self.clock()     # 다시 켜면 MAX_S 를 새로 센다
                self.publish(True)
            else:
                self._off()
            return self.on

    def _off(self):
        was_on, self.on, self.since = self.on, False, None
        if was_on:
            self.publish(False)

    def tick(self):
        """주기마다 부른다: 켜져 있으면 True 를 다시 내보내고, MAX_S 가 지났으면 끈다."""
        with self.lock:
            if not self.on:
                return
            if self.clock() - self.since >= self.max_s:
                self._off()
            else:
                self.publish(True)

    def start(self):
        def loop():
            while not self.stop_event.wait(self.period_s):
                self.tick()
        self.thread = threading.Thread(target=loop, name='boarding_alert', daemon=True)
        self.thread.start()
        return self

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2 * self.period_s + 1)
        with self.lock:
            self._off()


def make_ros_boarding_alert(node, topic=TOPIC):
    """node 에 Bool 발행을 만들고 주기 스레드를 켠 BoardingAlert 를 돌려준다."""
    from std_msgs.msg import Bool
    pub = node.create_publisher(Bool, topic, 1)
    return BoardingAlert(lambda on: pub.publish(Bool(data=on))).start()
