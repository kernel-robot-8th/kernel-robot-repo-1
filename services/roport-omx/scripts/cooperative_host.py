"""Run the shelf-01 COOPERATIVE Host: broker ↔ CooperativeHost ↔ executor.

Only the simulated executor exists so far; the panel adapter follows once its armed/result API is
fixed. Broker settings come from ROPORT_MQTT_* like the control server; secrets stay in the env.

  uv run python scripts/cooperative_host.py --journal ~/.local/state/roport/shelf-01.json
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path
from queue import Empty

from cooperative_mock import MockExecutor

from roport_omx.cooperative import WORKCELL, CooperativeHost, StorageError
from roport_omx.mqtt_link.journal import Journal
from roport_omx.mqtt_link.messages import decode, topic
from roport_omx.mqtt_link.mqtt import MqttConfig, MqttTransport

RESEND_S = 5.0
log = logging.getLogger("cooperative_host")


def mqtt_config(args) -> MqttConfig:
    return MqttConfig(
        host=args.host,
        client_id=args.client_id,
        port=args.port,
        tls=os.environ.get("ROPORT_MQTT_TLS") == "1",
        ca_file=os.environ.get("ROPORT_MQTT_CA_FILE"),
        username=os.environ.get("ROPORT_MQTT_USERNAME"),
        password=os.environ.get("ROPORT_MQTT_PASSWORD"),
    )


def publish(transport: MqttTransport, site: str, events: list[dict]) -> None:
    for event in events:
        try:
            transport.publish(topic(site, "workcell", WORKCELL, "event"), event)
            log.info(
                "event %s rev=%s", event["type"], event["payload"]["phase_revision"]
            )
        except TimeoutError as exc:
            log.warning(
                "event %s not confirmed (%s); kept for resend", event["type"], exc
            )


def serve(host: CooperativeHost, transport: MqttTransport, site: str) -> None:
    channels = {topic(site, "workcell", WORKCELL, c): c for c in ("cmd", "ack")}
    publish(transport, site, host.pending())
    next_resend = time.monotonic() + RESEND_S
    while True:
        try:
            delivery = transport.receive(timeout=0.2)
        except Empty:
            delivery = None
        if delivery is not None:
            events = []
            try:
                channel = channels.get(delivery.topic)
                if channel is None:
                    raise ValueError("unsubscribed topic")
                events = host.handle(
                    decode(delivery.payload),
                    channel=channel,
                    qos=delivery.qos,
                    retained=delivery.retained,
                )
            except ValueError as exc:
                log.warning("dropped message on %s: %s", delivery.topic, exc)
            # Reached only when the journal holds the result (StorageError propagates).
            transport.ack(delivery)
            publish(transport, site, events)
        publish(transport, site, host.poll())
        if time.monotonic() >= next_resend:
            publish(transport, site, host.pending())
            next_resend = time.monotonic() + RESEND_S


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--host", default=os.environ.get("ROPORT_MQTT_HOST", "127.0.0.1")
    )
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("ROPORT_MQTT_PORT", "1883"))
    )
    parser.add_argument("--client-id", default="roport-shelf-01-host")
    parser.add_argument("--site", default="test")
    parser.add_argument("--control", default="control-test")
    parser.add_argument("--binding", default="test-binding")
    parser.add_argument(
        "--journal", type=Path, required=True, help="persistent file, not under /tmp"
    )
    parser.add_argument(
        "--clear-hold",
        action="store_true",
        help="after checking arms, package and Pinky",
    )
    parser.add_argument("--mock-run-s", type=float, default=3.0)
    parser.add_argument("--mock-disarmed", action="store_true")
    parser.add_argument(
        "--mock-unreturned", action="append", default=[], choices=["omx1", "omx2"]
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    if str(args.journal.resolve()).startswith("/tmp/"):
        parser.error("journal must survive a restart; do not use /tmp")

    executor = MockExecutor(
        args.mock_run_s,
        armed=not args.mock_disarmed,
        unreturned=set(args.mock_unreturned),
    )
    journal = Journal(args.journal)
    try:
        host = CooperativeHost(
            journal,
            executor,
            site=args.site,
            control=args.control,
            binding=args.binding,
        )
    except (ValueError, StorageError) as exc:
        journal.close()
        parser.exit(2, f"{args.journal}: {exc}\n")
    if args.clear_hold:
        host.clear_hold()
    if host.hold:
        log.warning(
            "HOLD on command %s: new tasks are rejected until --clear-hold", host.hold
        )
    log.info("SIMULATED executor; boot_id=%s site=%s", host.boot_id, args.site)
    subscriptions = [topic(args.site, "workcell", WORKCELL, c) for c in ("cmd", "ack")]
    try:
        while True:
            transport = MqttTransport(mqtt_config(args), subscriptions)
            try:
                transport.start()
                log.info("subscribed %s:%s", args.host, args.port)
                serve(host, transport, args.site)
            except (ConnectionError, TimeoutError) as exc:
                log.warning("broker link lost (%s); reconnecting", exc)
                transport.stop()
                time.sleep(2)
    except KeyboardInterrupt:
        pass
    except StorageError as exc:
        # The delivery being handled is not ACKed; restart reconciles from the journal.
        log.critical("%s; stopping without ACK", exc)
        sys.exit(1)
    finally:
        journal.close()
    sys.exit(0)


if __name__ == "__main__":
    main()
