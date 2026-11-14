"""shelf-01 host telemetry in cooperative_host; fake transport, no Paho or broker."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from queue import Empty

ROOT = Path(__file__).resolve().parents[1]
CONTROL_MQTT = (
    ROOT.parent / "roport-control-server/src/roport_control_server/adapters/mqtt"
)
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

try:
    import roport_omx.mqtt_link.mqtt  # noqa: F401
except ImportError:  # system Python in CI has no Paho; serve() never needs it
    stub = types.ModuleType("roport_omx.mqtt_link.mqtt")
    stub.MqttConfig = stub.MqttTransport = object
    sys.modules["roport_omx.mqtt_link.mqtt"] = stub

from roport_omx.mqtt_link import field_wire

spec = importlib.util.spec_from_file_location(
    "cooperative_host", ROOT / "scripts/cooperative_host.py"
)
ch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ch)

TOPIC = "roport/workcell/shelf-01/host"


class Stop(Exception):
    pass


class FakeHost:
    def pending(self):
        return []

    def poll(self):
        return []

    def handle(self, message, **kw):
        return []


class FakeTransport:
    def __init__(self, deliveries=(), loops=3, fail=None):
        self.deliveries, self.loops, self.fail = list(deliveries), loops, fail
        self.samples, self.acked, self.published = [], [], []

    def receive(self, timeout):
        if self.deliveries:
            return self.deliveries.pop(0)
        if self.loops == 0:
            raise ConnectionError("end of test")
        self.loops -= 1
        raise Empty

    def ack(self, delivery):
        self.acked.append(delivery)

    def publish(self, destination, message):
        self.published.append(destination)

    def publish_host_metrics(self, destination, message, expiry_s):
        if self.fail:
            raise self.fail
        field_wire.validate(message)
        self.samples.append((destination, message, expiry_s))


def delivery():
    return types.SimpleNamespace(
        topic="roport/v1/test/workcell/shelf-01/cmd",
        payload=b"not json",
        qos=1,
        retained=False,
    )


class HostMetricsTest(unittest.TestCase):
    def run_serve(self, transport, metrics):
        with self.assertRaises(ConnectionError):
            ch.serve(FakeHost(), transport, "test", metrics)

    def test_off_without_period(self):
        transport = FakeTransport()
        self.run_serve(transport, None)
        self.assertEqual(transport.samples, [])
        args = ch.parse_args(["--journal", "/var/x.json"])
        self.assertIsNone(args.host_metrics_period_s)

    def test_cli_rejects_non_positive_period(self):
        for bad in ("0", "-1", "nan", "inf"):
            with self.assertRaises(SystemExit):
                ch.parse_args(
                    ["--journal", "/var/x.json", "--host-metrics-period-s", bad]
                )
        args = ch.parse_args(
            ["--journal", "/var/x.json", "--host-metrics-period-s", "5"]
        )
        self.assertEqual(args.host_metrics_period_s, 5.0)

    def test_periodic_samples_from_shelf01(self):
        metrics = ch.HostMetrics("boot-1", "control-test", 5.0)
        transport = FakeTransport()
        for now in (0.0, 1.0, 5.0, 9.9, 10.0):
            metrics.tick(transport, now)
        self.assertEqual(len(transport.samples), 3)
        seqs = [m["payload"]["sequence"] for _, m, _ in transport.samples]
        self.assertEqual(seqs, [1, 2, 3])
        destination, message, expiry = transport.samples[0]
        self.assertEqual(destination, TOPIC)
        self.assertEqual(message["source"], "shelf-01")
        self.assertEqual(message["target"], "control-test")
        self.assertEqual(message["payload"]["boot_version"], "boot-1")
        self.assertEqual(expiry, 5.0)

    def test_serve_publishes_with_boot_id(self):
        metrics = ch.HostMetrics("boot-2", "control-test", 1e-9)
        transport = FakeTransport(loops=2)
        self.run_serve(transport, metrics)
        self.assertGreaterEqual(len(transport.samples), 2)
        self.assertTrue(
            all(
                m["payload"]["boot_version"] == "boot-2"
                for _, m, _ in transport.samples
            )
        )

    def test_publish_failure_does_not_block_ack(self):
        for fail in (ConnectionError("down"), ValueError("bad"), RuntimeError("bug")):
            metrics = ch.HostMetrics("boot-3", "control-test", 1e-9)
            d = delivery()
            transport = FakeTransport([d], loops=2, fail=fail)
            with self.assertLogs("cooperative_host", "WARNING"):
                self.run_serve(transport, metrics)
            self.assertEqual(transport.acked, [d])
            self.assertGreaterEqual(metrics.sequence, 1)

    def test_copies_match_control_server(self):
        for name in ("field_wire.py", "host_metrics.py", "mqtt.py"):
            copy = (ROOT / "src/roport_omx/mqtt_link" / name).read_text()
            header, _, body = copy.partition("\n")
            self.assertTrue(header.startswith("# Copied unchanged"), name)
            self.assertEqual(body, (CONTROL_MQTT / name).read_text(), name)


if __name__ == "__main__":
    unittest.main()
