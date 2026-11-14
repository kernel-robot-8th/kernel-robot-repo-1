"""Run the r8 OMX endpoint owner. --mock is loopback-only and cannot move hardware."""

import argparse
import importlib
import json
import logging
import os
import time
from contextlib import closing
from pathlib import Path
from queue import Empty

from field_mock import virtual_alignment, virtual_executors

from roport_omx.field_host import FieldOmxHost
from roport_omx.mqtt_link.field_wire import decode, topic
from roport_omx.mqtt_link.journal import Journal
from roport_omx.mqtt_link.mqtt import MqttConfig, MqttTransport

log = logging.getLogger(__name__)


def serve(host, transport, *, stop=None):
    next_state = next_resend = 0.0
    channels = {topic(t, c): c for t in host.executors for c in ("cmd", "ack")}
    pinky_topics = {topic(p, "event") for p in ("pinky1", "pinky2", "pinky3")}

    def send(events):
        for event in events:
            transport.publish(topic(event["source"], "event"), event)

    try:
        while stop is None or not stop.is_set():
            try:
                delivery = transport.receive(timeout=0.05)
            except Empty:
                delivery = None
            if delivery:
                try:
                    message = decode(delivery.payload)
                    if delivery.topic in pinky_topics:
                        if (
                            delivery.qos != 1
                            or delivery.retained
                            or delivery.topic != topic(message["source"], "event")
                        ):
                            raise ValueError("untrusted Pinky observation")
                        if message["type"] == "robot_state":
                            host.observe_pinky(message)
                        events = []
                    elif delivery.topic in channels:
                        expected = "ack" if message["type"] == "event_stored" else "cmd"
                        if delivery.topic != topic(message["target"], expected):
                            raise ValueError("topic and command target differ")
                        events = host.handle(
                            message, qos=delivery.qos, retained=delivery.retained
                        )
                    else:
                        raise ValueError("unexpected topic")
                except ValueError as exc:
                    log.warning("Rejected field MQTT packet: %s", exc)
                    events = []
                transport.ack(delivery)
                send(events)
            send(host.poll())
            now = time.monotonic()
            if now >= next_state:
                send(host.state(target) for target in host.executors)
                next_state = now + 0.5
            if now >= next_resend:
                send(host.pending())
                next_resend = now + 1.0
    finally:
        # The device adapter owns bounded abort/hold. Never infer successful stopping here.
        command = host.data["commands"].get(host.data["active"])
        if command:
            host._fail(command["command"], "mqtt_owner_stopped")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--journal", type=Path, required=True)
    parser.add_argument(
        "--host", default=os.environ.get("ROPORT_MQTT_HOST", "127.0.0.1")
    )
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("ROPORT_MQTT_PORT", "1883"))
    )
    parser.add_argument("--control", default="control-01")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--mock", action="store_true")
    mode.add_argument(
        "--factory", help="module:function returning (executors, alignment_sensor)"
    )
    parser.add_argument("--hardware-authorized", action="store_true")
    parser.add_argument(
        "--package-size", choices=["small", "large", "unknown"], default="small"
    )
    parser.add_argument(
        "--package-sizes",
        type=Path,
        help="virtual package_id -> small/large/unknown JSON",
    )
    parser.add_argument("--run-s", type=float, default=0.15)
    args = parser.parse_args()
    if args.mock and (
        args.host not in {"localhost", "127.0.0.1", "::1"} or args.hardware_authorized
    ):
        parser.error(
            "mock endpoints require a loopback broker and no hardware authorization"
        )
    if args.factory and not args.hardware_authorized:
        parser.error(
            "hardware factories require --hardware-authorized after field checks"
        )
    sizes = json.loads(args.package_sizes.read_text()) if args.package_sizes else {}
    if not isinstance(sizes, dict) or any(
        v not in {"small", "large", "unknown"} for v in sizes.values()
    ):
        parser.error("package sizes must map IDs to small/large/unknown")
    logging.basicConfig(level=logging.INFO)
    with closing(Journal(args.journal)) as journal:
        if args.mock:
            host = FieldOmxHost(
                journal,
                virtual_executors(args.run_s, args.package_size, sizes),
                control=args.control,
            )
            host.alignment = lambda command: virtual_alignment(host, command)
        else:
            module, separator, name = args.factory.partition(":")
            if not separator:
                parser.error("factory must be module:function")
            executors, sensor = getattr(importlib.import_module(module), name)()
            host = FieldOmxHost(
                journal, executors, control=args.control, alignment=sensor
            )
        config = MqttConfig(
            host=args.host,
            port=args.port,
            client_id="roport-field-omx",
            tls=os.environ.get("ROPORT_MQTT_TLS") == "1",
            ca_file=os.environ.get("ROPORT_MQTT_CA_FILE"),
            username=os.environ.get("ROPORT_MQTT_USERNAME"),
            password=os.environ.get("ROPORT_MQTT_PASSWORD"),
        )
        subscriptions = [topic(t, c) for t in host.executors for c in ("cmd", "ack")]
        subscriptions += [topic(p, "event") for p in ("pinky1", "pinky2", "pinky3")]
        transport = MqttTransport(config, subscriptions)
        try:
            transport.start()
            print(
                json.dumps(
                    {
                        "ready": True,
                        "simulated": args.mock,
                        "boot_version": host.boot_version,
                    }
                ),
                flush=True,
            )
            serve(host, transport)
        except KeyboardInterrupt:
            pass
        finally:
            transport.stop()


if __name__ == "__main__":
    main()
