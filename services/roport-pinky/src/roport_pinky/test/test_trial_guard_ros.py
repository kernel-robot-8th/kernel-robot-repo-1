"""Owner process kill: keep sending line/Nav2 velocity, observe final output zero."""
import os
import subprocess
import sys
import time

import pytest


@pytest.mark.parametrize('source', ['line', 'nav2'])
def test_owner_killed_while_velocity_source_remains_alive(source):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from roport_pinky.trial_guard import make_node
    ctx = Context()
    rclpy.init(context=ctx, domain_id=193)
    guard = make_node(context=ctx, namespace='/trial_watchdog_test')
    probe = rclpy.create_node('probe', context=ctx, namespace='/trial_watchdog_test')
    topic = 'line_nav/cmd_vel' if source == 'line' else 'trial/nav2_cmd_vel'
    pub = probe.create_publisher(Twist, topic, 10)
    observed = []
    probe.create_subscription(Twist, 'cmd_vel', lambda m: observed.append((time.monotonic(), m.linear.x)), 10)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(guard)
    ex.add_node(probe)
    code = '''
import json, time, rclpy
from std_msgs.msg import String
rclpy.init()
n = rclpy.create_node('owner', namespace='/trial_watchdog_test')
p = n.create_publisher(String, 'trial/lease', 1)
state = {}
n.create_subscription(String, 'trial/guard_state', lambda m: state.update(json.loads(m.data)), 10)
seq = 0
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=.03)
    if state:
        seq += 1
        p.publish(String(data=json.dumps(dict(boot=state['boot'], session='owner', seq=seq,
                                              owner=SOURCE, sent_at=time.time()))))
'''.replace('SOURCE', repr(source))
    child = subprocess.Popen([sys.executable, '-c', code], env={**os.environ,
                              'ROS_DOMAIN_ID': '193', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    msg = Twist()
    msg.linear.x = .03

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pub.publish(msg)
            ex.spin_once(timeout_sec=.02)
    try:
        spin(2)
        assert any(v == .03 for _, v in observed), child.poll()
        child.kill()
        child.wait(timeout=3)
        killed = time.monotonic()
        spin(1.5)
        tail = [v for t, v in observed if t > killed + 1.15]
        assert tail and all(v == 0 for v in tail)
        assert guard.gate.blocked
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=3)
        ex.shutdown()
        guard.destroy_node()
        probe.destroy_node()
        rclpy.shutdown(context=ctx)
