"""Run the shelf-01 COOPERATIVE Host: broker ↔ CooperativeHost ↔ executor.

Only the simulated executor exists so far; the panel adapter follows once its armed/result API is
fixed. Broker settings come from ROPORT_MQTT_* like the control server; secrets stay in the env.

  uv run python scripts/cooperative_host.py --journal ~/.local/state/roport/shelf-01.json
"""

import argparse
import json
import logging
import math
import os
import sys
import time
from pathlib import Path
from queue import Empty

from cooperative_mock import MockExecutor

from roport_omx.cooperative import WORKCELL, CooperativeHost, StorageError
from roport_omx.mqtt_link import field_wire
from roport_omx.mqtt_link.host_metrics import host_metrics_message, read_host_metrics
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


class HostMetrics:
    """Display-only shelf-01 host telemetry (QoS0). Never part of delivery ACK or stop paths."""

    def __init__(self, boot_version: str, target: str, period_s: float):
        self.boot_version, self.target, self.period_s = boot_version, target, period_s
        self.sequence, self.last = 0, -math.inf

    def tick(self, transport: MqttTransport, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if now - self.last < self.period_s:
            return
        self.last = now
        try:
            self.sequence += 1
            message = host_metrics_message(
                WORKCELL,
                self.target,
                self.boot_version,
                self.sequence,
                read_host_metrics(),
            )
            transport.publish_host_metrics(
                field_wire.topic(WORKCELL, "host"), message, self.period_s
            )
        except Exception as exc:  # noqa: BLE001 - telemetry must never stop the host
            log.warning("host metrics not sent: %s", type(exc).__name__)


class HoldChannel:
    """Local files next to the journal for the panel on this PC (same user, never the network).

    <journal stem>.host-state.json   written by the Host: boot_id, active task, HOLD and its cause
    <journal stem>.hold-request.json written by the panel's [HOLD 해제] button:
        {"request_id", "hold": <command_id shown>, "checks": {arms_safe, package_checked,
         pinky_checked}, "requested_at": <unix s>, "by": "panel"}
    <journal stem>.hold-result.json  written by the Host: the latest {"request_id", "cleared",
        "reason", ...} at the top, and "results": {request_id: result} for the last
        RESULT_KEEP_S, so a panel waiting for its own request_id finds it even after a newer one
    A request is read once and removed; one older than REQUEST_MAX_AGE_S is refused.

    These files are for the panel's screen only: an I/O error on them is logged and retried on
    the next tick, never raised into the MQTT loop (2026-11-21 review: a result file that could
    not be replaced stopped the Host after the HOLD was already cleared). A request already run
    is never run again; its result is kept and written once the file can be written, and until
    then a new request stays in its file unread (it must not replace the pending result: second
    2026-11-21 review). After a result is written the next request also waits RESULT_SHOWN_S, so
    a panel that reads only the top of the file (100 ms polls) still sees each answer (third
    2026-11-21 review). Journal errors (StorageError) still propagate and stop the Host as before.
    """

    REQUEST_MAX_AGE_S = 60.0
    RESULT_KEEP_S = (
        60.0  # results by request_id kept in the file (the panel waits up to 13 s)
    )
    RESULT_SHOWN_S = 1.0  # a written result stays on top at least this long

    def __init__(self, journal_path: Path):
        stem = journal_path.with_suffix("")
        self.state_path = stem.with_name(stem.name + ".host-state.json")
        self.request_path = stem.with_name(stem.name + ".hold-request.json")
        self.result_path = stem.with_name(stem.name + ".hold-result.json")
        self._last_state = None
        self._latest: dict | None = None  # the last request's result (top of the file)
        self._results: dict[str, dict] = {}  # request_id → result, last RESULT_KEEP_S
        self._unwritten = (
            False  # a result was run (journal updated) but not written yet
        )
        self._written_at: float | None = None  # when the file last got a new result
        self._handled = None  # (file mtime, request_id) of the request last run

    @staticmethod
    def _write(path: Path, data: dict) -> bool:
        tmp = path.with_name(path.name + ".tmp")
        try:
            tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1))
            tmp.replace(path)  # the panel never reads half a file
        except OSError as exc:
            log.warning("%s not written (retrying): %s", path.name, exc)
            return False
        return True

    def _take_request(self) -> dict | None:
        """The panel's request, removed once read; None when there is none or it was run."""
        try:
            seen = self.request_path.stat().st_mtime_ns
        except OSError:  # absent (or unreadable: nothing to run)
            return None
        try:
            request = json.loads(self.request_path.read_text())
        except (OSError, ValueError):
            request = {}
        request = request if isinstance(request, dict) else {}
        try:
            self.request_path.unlink(missing_ok=True)
        except OSError as exc:
            log.warning("%s not removed: %s", self.request_path.name, exc)
        key = (seen, request.get("request_id"))
        if key == self._handled:  # the same file again (it could not be removed)
            return None
        self._handled = key
        return request

    def _write_results(self, now: float) -> None:
        data = dict(self._latest or {}) | {"results": self._results}
        if self._write(self.result_path, data):
            self._unwritten, self._written_at = False, now

    def _ready_for_request(self, now: float) -> bool:
        if self._unwritten:
            return False
        return self._written_at is None or now - self._written_at >= self.RESULT_SHOWN_S

    def tick(self, host: CooperativeHost, now: float | None = None) -> None:
        now = time.time() if now is None else now
        # a result still waiting for its file goes first; a new request waits in its file
        if self._unwritten:
            self._write_results(now)
        request = self._take_request() if self._ready_for_request(now) else None
        if request is not None:
            asked = request.get("requested_at")
            if isinstance(asked, bool) or not isinstance(asked, (int, float)):
                asked = None
            if asked is None or not 0 <= now - asked < self.REQUEST_MAX_AGE_S:
                result = {
                    "request_id": request.get("request_id"),
                    "hold": host.hold,
                    "cleared": False,
                    "reason": "STALE_REQUEST",
                }
            else:
                result = host.release_hold(request)  # StorageError propagates
            entry = result | {"at": now}
            log.warning("HOLD release request %s: %s", result.get("request_id"), result)
            self._latest, self._unwritten = entry, True
            request_id = result.get("request_id")
            if isinstance(request_id, str):
                self._results[request_id] = entry
            self._results = {
                k: v
                for k, v in self._results.items()
                if now - v["at"] < self.RESULT_KEEP_S
            }
            self._write_results(now)
        state = {
            "boot_id": host.boot_id,
            "active": host.active,
            "hold": host.hold_info(),
        }
        if state != self._last_state and self._write(
            self.state_path, state | {"updated_at": now}
        ):
            self._last_state = state


def serve(
    host: CooperativeHost,
    transport: MqttTransport,
    site: str,
    metrics: HostMetrics | None = None,
    holds: HoldChannel | None = None,
) -> None:
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
        if metrics is not None:
            metrics.tick(transport)
        if holds is not None:
            holds.tick(host)


def positive_period(text: str) -> float:
    value = float(text)
    if not 0 < value < math.inf:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return value


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
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
    parser.add_argument(
        "--host-metrics-period-s",
        type=positive_period,
        default=None,
        help="publish display-only host telemetry every N s; off when omitted",
    )
    args = parser.parse_args(argv)
    if str(args.journal.resolve()).startswith("/tmp/"):
        parser.error("journal must survive a restart; do not use /tmp")
    return args


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )

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
        if args.clear_hold:
            host.clear_hold()
    except (ValueError, StorageError) as exc:
        journal.close()
        print(f"{args.journal}: {exc}", file=sys.stderr)
        sys.exit(2)
    if host.hold:
        log.warning(
            "HOLD on command %s: new tasks are rejected until --clear-hold", host.hold
        )
    log.info("SIMULATED executor; boot_id=%s site=%s", host.boot_id, args.site)
    metrics = None
    if args.host_metrics_period_s is not None:
        metrics = HostMetrics(host.boot_id, args.control, args.host_metrics_period_s)
    holds = HoldChannel(args.journal)  # the panel's [HOLD 해제] button
    holds.tick(host)
    subscriptions = [topic(args.site, "workcell", WORKCELL, c) for c in ("cmd", "ack")]
    try:
        while True:
            transport = MqttTransport(mqtt_config(args), subscriptions)
            try:
                transport.start()
                log.info("subscribed %s:%s", args.host, args.port)
                serve(host, transport, args.site, metrics, holds)
            except (ConnectionError, TimeoutError) as exc:
                log.warning("broker link lost (%s); reconnecting", exc)
                transport.stop()
                # The panel's [HOLD 해제] does not need the broker: keep answering it while
                # waiting to reconnect.
                for _ in range(10):
                    holds.tick(host)
                    time.sleep(0.2)
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
