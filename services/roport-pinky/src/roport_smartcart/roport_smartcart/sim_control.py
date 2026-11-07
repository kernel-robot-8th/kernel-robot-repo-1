"""Domain-198 control fixture. Not the production MQTT protocol or bay database."""

import argparse
import json
import math
import os
import time
from uuid import uuid4


class ReturnPermit:
    """A single stopped delivery session; stale/duplicate grants cannot renew it."""

    def __init__(self, session):
        self.session = session
        self.waiting = False
        self.grant = None
        self.last_seen = None
        self.stopped = False

    def observe(self, message, now):
        if not isinstance(message, dict) or message.get("session") != self.session:
            return
        if message.get("cmd") == "stop":
            self.stopped = True
            return
        if self.stopped or message.get("cmd") != "return" or not self.waiting:
            return
        expires = message.get("expires_at")
        if (
            message.get("receipt_confirmed") is not True
            or message.get("bay") not in ("H1", "H2")
            or not isinstance(message.get("reservation_id"), str)
            or not message["reservation_id"]
            or isinstance(expires, bool)
            or not isinstance(expires, (int, float))
            or not math.isfinite(expires)
            or not 0 < expires - now <= 300
        ):
            return
        if self.grant is None:
            self.grant = dict(message)
        if self.grant == message:
            self.last_seen = now

    def check(self, now):
        if self.stopped:
            raise RuntimeError("control stop requested")
        if self.grant is not None and (
            now >= self.grant["expires_at"]
            or self.last_seen is None
            or now - self.last_seen > 2
        ):
            raise RuntimeError("control reservation expired or heartbeat lost")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--occupied", nargs="*", choices=("H1", "H2"), default=[])
    options = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "198":
        parser.error("Gazebo domain 198 only")
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    for key in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER"):
        os.environ.pop(key, None)
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init(domain_id=198)
    node = rclpy.create_node("smartcart_sim_control")
    publisher = node.create_publisher(String, "smartcart/sim/control", 10)
    occupied, grants = set(options.occupied), {}
    current = [None, 0.0]

    def observe(message):
        try:
            state = json.loads(message.data)
        except ValueError:
            return
        if not isinstance(state, dict) or not isinstance(state.get("session"), str):
            return
        current[:] = [state, time.time()]
        grant = grants.get(state["session"])
        if (
            grant
            and state.get("state") == "ARRIVED"
            and state.get("bay") == grant["bay"]
        ):
            occupied.add(grant["bay"])

    def confirm(_request, response):
        state, seen = current
        if (
            state is None
            or time.time() - seen > 2
            or state.get("state") != "WAIT_UNLOAD"
        ):
            response.success, response.message = (
                False,
                "No fresh stopped delivery session",
            )
            return response
        session = state["session"]
        if session not in grants:
            reserved = {g["bay"] for g in grants.values()}
            bay = next((b for b in ("H1", "H2") if b not in occupied | reserved), None)
            if bay is None:
                response.success, response.message = False, "No free bay; keep waiting"
                return response
            grants[session] = {
                "cmd": "return",
                "session": session,
                "receipt_confirmed": True,
                "bay": bay,
                "reservation_id": uuid4().hex,
                "expires_at": time.time() + 290,
            }
        response.success, response.message = True, json.dumps(grants[session])
        publisher.publish(String(data=response.message))
        node.get_logger().info(f"RECEIPT CONFIRMED; reserved {grants[session]['bay']}")
        return response

    def heartbeat():
        state, seen = current
        if state and time.time() - seen <= 2:
            grant = grants.get(state["session"])
            if grant and state.get("state") in ("WAIT_UNLOAD", "RETURNING"):
                publisher.publish(String(data=json.dumps(grant)))

    def stop(_request, response):
        state, seen = current
        response.success = state is not None and time.time() - seen <= 2
        response.message = "Stop requested; reservation retained for recovery"
        if response.success:
            publisher.publish(
                String(data=json.dumps({"cmd": "stop", "session": state["session"]}))
            )
        return response

    node.create_subscription(String, "smartcart/sim/state", observe, 10)
    node.create_service(Trigger, "smartcart/sim/confirm_receipt", confirm)
    node.create_service(Trigger, "smartcart/sim/stop", stop)
    node.create_timer(0.5, heartbeat)
    node.get_logger().info(
        f"SIM ONLY; occupied={sorted(occupied)}; receipt service ready"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
