"""Final drive gate, in a process independent of the MQTT runner and line/Nav2.

A lost owner lease latches closed until this process is restarted at rest.
This is a software watchdog, not a substitute for a motor-controller timeout.
"""
import json
import time
from uuid import uuid4

from roport_line_nav.cmd_vel_guard import GuardLogic

LEASE_TIMEOUT = 1.0


class LeaseGate:
    def __init__(self):
        self.boot = str(uuid4())
        self.owner = 'none'
        self.blocked = False
        self.last = None
        self.seq = -1
        self.session = None
        self.velocity = GuardLogic()

    def lease(self, body, now, wall):
        try:
            valid = (body['boot'] == self.boot and body['owner'] in ('none', 'line', 'nav2')
                     and type(body['seq']) is int and body['seq'] > self.seq
                     and isinstance(body['session'], str) and 0 < len(body['session']) <= 64
                     and -0.1 <= wall - body['sent_at'] <= 0.5)
        except (KeyError, TypeError):
            return
        if not valid or self.blocked:
            return
        if self.last is not None and now - self.last > LEASE_TIMEOUT:
            self.blocked = True
            self.owner = 'none'
            return
        if self.session is not None and self.session != body['session']:
            self.blocked = True
            self.owner = 'none'
            return
        if body['owner'] != self.owner:
            self.velocity = GuardLogic()  # old source commands never survive a handoff
        self.session, self.seq = body['session'], body['seq']
        self.last, self.owner = now, body['owner']

    def tick(self, now):
        if self.last is not None and now - self.last > LEASE_TIMEOUT:
            self.blocked, self.owner = True, 'none'
        return self.blocked or self.owner == 'none' or self.velocity.tick(now)

    def relay(self, source, now, linear, angular):
        self.tick(now)
        if self.blocked or self.owner != source:
            return None
        return self.velocity.relay(now, linear, angular)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = make_node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def make_node(**kwargs):
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    class TrialGuard(rclpy.node.Node):
        def __init__(self):
            super().__init__('trial_guard', **kwargs)
            self.gate = LeaseGate()
            self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.status = self.create_publisher(String, 'trial/guard_state', 10)
            self.create_subscription(String, 'trial/lease', self.on_lease, 1)
            for source in ('line', 'nav2'):
                topic = 'trial/nav2_cmd_vel' if source == 'nav2' else 'line_nav/cmd_vel'
                self.create_subscription(Twist, topic,
                                         lambda m, s=source: self.on_velocity(s, m), 1)
            self.create_timer(0.05, self.tick)

        def publish_zero(self):
            self.pub.publish(Twist())

        def on_lease(self, msg):
            try:
                body = json.loads(msg.data)
                old = self.gate.owner
                self.gate.lease(body, time.monotonic(), time.time())
                if old != self.gate.owner or self.gate.blocked:
                    self.publish_zero()
            except (ValueError, TypeError):
                return

        def on_velocity(self, source, msg):
            value = self.gate.relay(source, time.monotonic(), msg.linear.x, msg.angular.z)
            if value is not None:
                out = Twist()
                out.linear.x, out.angular.z = value
                self.pub.publish(out)

        def tick(self):
            if self.gate.tick(time.monotonic()):
                self.publish_zero()
            self.status.publish(String(data=json.dumps({
                'boot': self.gate.boot, 'owner': self.gate.owner,
                'blocked': self.gate.blocked, 'sent_at': time.time(),
            })))

    return TrialGuard()
