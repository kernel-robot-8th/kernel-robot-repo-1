"""Run the shelf-01 COOPERATIVE Host: broker ↔ Host ↔ executor.

The field contract roport-control-v2 (Fleet control, topics roport/workcell/shelf-01/* and
roport/robot/omx1|omx2/*; src/roport_omx/cooperative_v2.py) is the default. --protocol v1 runs the
earlier roport-omx-task-v1 test link (OmxTrial) instead; each protocol needs its own journal (the
other's is refused).

--executor mock (default) simulates the arms. --executor panel runs each task through the local OMX
control panel (v3.14+, armed switch on the panel screen) with the model routed for its slot; the
arms move, so a person stays by them. Broker settings come from ROPORT_MQTT_* like the control
server; secrets stay in the env.

  uv run python scripts/cooperative_host.py --executor mock --no-require-docking \
      --mock-versions model-demo-v1 calibration-demo-v1 --journal ~/.local/state/roport/shelf-01-v2.json
  uv run python scripts/cooperative_host.py --executor panel --routes ~/.config/roport/shelf-01-routes.json \
      --journal ~/.local/state/roport/shelf-01-v2.json
  uv run python scripts/cooperative_host.py --protocol v1 --journal ~/.local/state/roport/shelf-01.json
"""

import argparse
import json
import logging
import math
import os
import sys
import threading
import time
from pathlib import Path
from queue import Empty

from cooperative_mock import MockExecutor

from roport_omx import panel_executor, shelf_panel
from roport_omx.cooperative import WORKCELL, CooperativeHost, StorageError
from roport_omx.cooperative_v2 import ENDPOINTS, PINKIES, FieldHost
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


class ReadinessSampler(threading.Thread):
    """v2: reads the executor's readiness every period_s so robot_state never waits on a slow
    panel read (Fleet drops a state observed more than 3 s ago). The latest sample and its time."""

    def __init__(self, executor, period_s: float = 1.0):
        super().__init__(name="readiness-sampler", daemon=True)
        self.executor, self.period_s = executor, period_s
        self._latest: tuple[dict, float] | None = None
        self._lock = threading.Lock()

    def run(self) -> None:
        while True:
            try:
                ready = self.executor.readiness()
            except Exception as exc:  # noqa: BLE001 - an unreachable executor is not ready
                ready = {
                    "armed": None,
                    "busy": False,
                    "error": str(exc)[:200],
                    "arms": {},
                }
            with self._lock:
                self._latest = (ready, time.time())
            time.sleep(self.period_s)

    def __call__(self) -> tuple[dict, float]:
        with self._lock:
            if self._latest is None:
                raise RuntimeError("no readiness sample yet")
            return self._latest


V2_RESEND_S, V2_STATE_S = 2.0, 1.0
# v2 PUBACK wait per message: short, so a slow broker never holds the keep-running watchdog
# (3 s) back for long; an unconfirmed event stays in the outbox and is resent.
V2_PUBLISH_TIMEOUT_S = 1.0


def publish_v2(transport: MqttTransport, events, host: FieldHost | None = None) -> None:
    queue = list(events)
    while queue:
        event = queue.pop(0)
        try:
            transport.publish(
                field_wire.topic(event["source"], "event"),
                event,
                timeout=V2_PUBLISH_TIMEOUT_S,
            )
            if event["type"] == "command_status":
                log.info(
                    "%s %s %s seq=%s",
                    event["source"],
                    event["command_id"],
                    event["payload"]["status"],
                    event["payload"]["sequence"],
                )
        except TimeoutError as exc:
            log.warning("%s not confirmed (%s); kept for resend", event["type"], exc)
        if host is not None:  # between slow publishes too
            queue += host.local_safety()


def wait_offline_v2(
    host: FieldHost, holds: HoldChannel | None, seconds: float, until=None
) -> None:
    """No broker: keep the local stop rules (watchdog, Pinky visit, stop timeout) and the panel's
    [HOLD 해제] running; their events wait in the outbox for the reconnect."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not (until and until()):
        host.poll()
        if holds is not None:
            holds.tick(host)
        time.sleep(0.1)


def start_v2(transport: MqttTransport, host: FieldHost, holds) -> None:
    """transport.start() waits up to 5 s for the broker; poll the Host meanwhile."""
    failure: list[BaseException] = []

    def connect() -> None:
        try:
            transport.start()
        except BaseException as exc:  # noqa: BLE001 - re-raised in the caller
            failure.append(exc)

    thread = threading.Thread(target=connect, name="mqtt-connect", daemon=True)
    thread.start()
    wait_offline_v2(host, holds, math.inf, until=lambda: not thread.is_alive())
    thread.join()
    if failure:
        raise failure[0]


def serve_v2(
    host: FieldHost,
    transport: MqttTransport,
    metrics: HostMetrics | None = None,
    holds: HoldChannel | None = None,
) -> None:
    channels = {
        field_wire.topic(e, c): (e, c) for e in ENDPOINTS for c in ("cmd", "ack")
    }
    pinkies = {field_wire.topic(p, "event"): p for p in PINKIES}
    publish_v2(transport, host.pending(), host)
    next_resend = time.monotonic() + V2_RESEND_S
    next_state = 0.0
    while True:
        try:
            delivery = transport.receive(timeout=0.1)
        except Empty:
            delivery = None
        if delivery is not None:
            events = []
            try:
                message = field_wire.decode(delivery.payload)
                if delivery.topic in pinkies:
                    if (
                        delivery.qos != 1
                        or delivery.retained
                        or message["source"] != pinkies[delivery.topic]
                    ):
                        raise ValueError("untrusted Pinky observation")
                    if message["type"] == "robot_state":
                        host.observe_pinky(message)
                elif delivery.topic in channels:
                    endpoint, channel = channels[delivery.topic]
                    events = host.handle(
                        message,
                        endpoint=endpoint,
                        channel=channel,
                        qos=delivery.qos,
                        retained=delivery.retained,
                    )
                else:
                    raise ValueError("unsubscribed topic")
            except ValueError as exc:
                log.warning("dropped message on %s: %s", delivery.topic, exc)
            # Reached only when the journal holds the result (StorageError propagates).
            transport.ack(delivery)
            publish_v2(transport, events, host)
        publish_v2(transport, host.poll(), host)
        now = time.monotonic()
        if now >= next_state:
            publish_v2(transport, [host.state(e) for e in ENDPOINTS], host)
            next_state = now + V2_STATE_S
        if now >= next_resend:
            publish_v2(transport, host.pending(), host)
            next_resend = now + V2_RESEND_S
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
    parser.add_argument(
        "--protocol",
        choices=["v1", "v2"],
        default="v2",
        help="v2 (default): roport-control-v2, the field contract (Fleet control); "
        "v1: the earlier roport-omx-task-v1 test link (OmxTrial), only when asked for",
    )
    parser.add_argument(
        "--control", default=None, help="control id (v1 control-test, v2 control-01)"
    )
    parser.add_argument(
        "--require-pinky",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="v2: accept a transfer only while the Pinky's own state shows it stopped at N11 "
        "with the named arrival; --no-require-pinky for bench tests without a Pinky "
        "(simulated executor only)",
    )
    parser.add_argument(
        "--require-docking",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="v2: a transfer also needs a docking sensor's fresh confirmation of the visit; "
        "none is connected yet, so the panel executor refuses every transfer "
        "(docking_alignment_unconfirmed). --no-require-docking: simulated executor only",
    )
    parser.add_argument(
        "--mock-versions",
        nargs=2,
        metavar=("MODEL_VERSION", "CALIBRATION_VERSION"),
        help="v2 simulated executor: the model/calibration pair it accepts (none without)",
    )
    parser.add_argument("--binding", default="test-binding")
    parser.add_argument(
        "--journal", type=Path, required=True, help="persistent file, not under /tmp"
    )
    parser.add_argument(
        "--clear-hold",
        action="store_true",
        help="after checking arms, package and Pinky",
    )
    parser.add_argument("--executor", choices=["mock", "panel"], default="mock")
    parser.add_argument("--panel-url", default="http://127.0.0.1:8865")
    parser.add_argument(
        "--routes", type=Path, help="panel: (operation, slot) → model JSON file"
    )
    parser.add_argument(
        "--box-marker",
        type=int,
        help="ignored (kept for old start commands): each command's package_id names the box",
    )
    parser.add_argument(
        "--report-placement",
        action=argparse.BooleanOptionalAction,
        default=False,  # off until the control tower handles two reports (#134 review)
        help="panel: loading in two reports (COMPLETE when the box is in its slot, then the "
        "alignment result); the control tower must accept evidence shelf_placed. Default off: "
        "one report after the alignment (control v1 holds on shelf_placed)",
    )
    parser.add_argument(
        "--end-on-place",
        action=argparse.BooleanOptionalAction,
        default=True,  # on by default (TK 2026-11-22); --no-end-on-place for the older flow
        help="panel 3.18+: end the loading evaluation once the box is placed and the hands are "
        "clear, and let the aligner take the arms there (hand-over). Default on; "
        "--no-end-on-place returns home first and aligns from there",
    )
    parser.add_argument(
        "--place-shadow",
        action="store_true",
        help="panel 3.18+: judge the placement during loading and only record it (no hand-over); "
        "only with --no-end-on-place",
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
    if args.control is None:
        args.control = "control-01" if args.protocol == "v2" else "control-test"
    if (
        args.protocol == "v2"
        and args.executor == "panel"
        and not (args.require_pinky and args.require_docking)
    ):
        # #135 review: the arms move only with a confirmed visit; bench runs use the mock
        parser.error(
            "--no-require-pinky/--no-require-docking need --executor mock "
            "(v2 runs the real arms only with the Pinky and docking checks)"
        )
    if args.report_placement or args.place_shadow:
        parser.error("the shelf host reports once, after alignment")
    if str(args.journal.resolve()).startswith("/tmp/"):
        parser.error("journal must survive a restart; do not use /tmp")
    if args.executor == "panel" and args.routes is None:
        parser.error("--executor panel needs --routes")
    return args


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )

    if args.executor == "panel":
        panel = shelf_panel if args.protocol == "v2" else panel_executor
        options = {"end_on_place": args.end_on_place} if args.protocol == "v2" else {}
        executor = panel.PanelExecutor(
            panel.load_routes(args.routes),
            box_marker=args.box_marker,
            base_url=args.panel_url,
            **options,
        )
    else:
        executor = MockExecutor(
            args.mock_run_s,
            armed=not args.mock_disarmed,
            unreturned=set(args.mock_unreturned),
            # v2 completes only with placement evidence; the mock claims it (SIMULATED)
            evidence={"INBOUND": "shelf_align", "OUTBOUND": "pinky_load"}
            if args.protocol == "v2"
            else None,
            versions=dict(
                zip(("model_version", "calibration_version"), args.mock_versions)
            )
            if args.mock_versions
            else None,
        )
    journal = Journal(args.journal)
    sampler = None
    try:
        if args.protocol == "v2":
            sampler = ReadinessSampler(executor)
            host = FieldHost(
                journal,
                executor,
                control=args.control,
                require_pinky=args.require_pinky,
                require_docking=args.require_docking,
                readiness=sampler,
            )
        else:
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
    if args.executor == "panel":
        routes = ", ".join(":".join(key) for key in executor.routes)
        log.info("PANEL executor %s routes=[%s]; arms move", args.panel_url, routes)
        episodes = ", ".join(
            f"{':'.join(key[:2])}={route.episode_s:g}s"
            for key, route in executor.routes.items()
        )
        log.info(
            "loading hand-over (end_on_place)=%s; early COMPLETE on placement "
            "(report_placement)=%s; episode limit [%s]",
            "on" if args.end_on_place else "off",
            "on" if args.report_placement else "off",
            episodes,
        )
    else:
        log.info("SIMULATED executor")
    if args.protocol == "v2":
        log.info(
            "protocol v2 (%s) control=%s boot_version=%s require_pinky=%s "
            "require_docking=%s",
            field_wire.PROFILE,
            args.control,
            host.boot_id,
            "on" if args.require_pinky else "off",
            "on" if args.require_docking else "off",
        )
        sampler.start()
    else:
        log.info("boot_id=%s site=%s", host.boot_id, args.site)
    metrics = None
    if args.host_metrics_period_s is not None:
        metrics = HostMetrics(host.boot_id, args.control, args.host_metrics_period_s)
    holds = HoldChannel(args.journal) if args.protocol == "v2" else None
    if holds:
        holds.tick(host)
    if args.protocol == "v2":
        subscriptions = [
            field_wire.topic(e, c) for e in ENDPOINTS for c in ("cmd", "ack")
        ] + [field_wire.topic(p, "event") for p in PINKIES]
    else:
        subscriptions = [
            topic(args.site, "workcell", WORKCELL, c) for c in ("cmd", "ack")
        ]
    try:
        while True:
            transport = MqttTransport(mqtt_config(args), subscriptions)
            try:
                if args.protocol == "v2":
                    start_v2(transport, host, holds)
                else:
                    transport.start()
                log.info("subscribed %s:%s", args.host, args.port)
                if args.protocol == "v2":
                    serve_v2(host, transport, metrics, holds)
                else:
                    serve(host, transport, args.site, metrics, holds)
            except (ConnectionError, TimeoutError) as exc:
                log.warning("broker link lost (%s); reconnecting", exc)
                transport.stop()
                # The panel's [HOLD 해제] does not need the broker: keep answering it while
                # waiting to reconnect (v2 also keeps its local stop rules running).
                if args.protocol == "v2":
                    wait_offline_v2(host, holds, 2.0)
                else:
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
