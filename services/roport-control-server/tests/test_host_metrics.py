"""Display-only host telemetry: fake /proc, fake transport and injected clock."""

import json
import logging
import subprocess
import sys
from queue import Empty

import pytest

from roport_control_server.adapters.mqtt import field_wire as wire
from roport_control_server.adapters.mqtt.host_metrics import (
    HostMetricsReceiver,
    host_metrics_message,
    read_host_metrics,
)
from roport_control_server.adapters.mqtt.mqtt import Delivery, MqttConfig, MqttTransport

MEMINFO = "MemTotal:        3884000 kB\nMemFree:  100 kB\nMemAvailable:    2710000 kB\n"
METRICS = (
    "cpu_count",
    "load_1m",
    "load_5m",
    "load_15m",
    "mem_total_kb",
    "mem_available_kb",
    "cpu_temp_c",
)


def proc(tmp_path, meminfo=MEMINFO, loadavg="0.42 0.31 0.20 1/123 4567\n", zones=()):
    (tmp_path / "meminfo").write_text(meminfo)
    (tmp_path / "loadavg").write_text(loadavg)
    thermal = tmp_path / "thermal"
    thermal.mkdir()
    for index, (kind, milli) in enumerate(zones):
        zone = thermal / f"thermal_zone{index}"
        zone.mkdir()
        (zone / "type").write_text(kind + "\n")
        (zone / "temp").write_text(milli + "\n")
    return {
        "meminfo": tmp_path / "meminfo",
        "loadavg": tmp_path / "loadavg",
        "thermal_root": thermal,
    }


def test_reads_proc_values_in_contract_units(tmp_path):
    paths = proc(tmp_path, zones=[("acpitz", "30000"), ("x86_pkg_temp", "52345")])
    m = read_host_metrics(**paths)
    assert set(m) == set(METRICS)
    assert (m["load_1m"], m["load_5m"], m["load_15m"]) == (0.42, 0.31, 0.2)
    assert (m["mem_total_kb"], m["mem_available_kb"]) == (3884000, 2710000)
    assert m["cpu_temp_c"] == 52.3  # the cpu zone, not the first zone
    assert m["cpu_count"] is None or m["cpu_count"] >= 1
    pi = tmp_path / "pi"
    pi.mkdir()
    assert (
        read_host_metrics(**proc(pi, zones=[("cpu-thermal", "61000")]))["cpu_temp_c"]
        == 61.0
    )


def test_temp_path_override(tmp_path):
    paths = proc(tmp_path, zones=[("x86_pkg_temp", "50000")])
    custom = tmp_path / "board_temp"
    custom.write_text("44400\n")
    assert read_host_metrics(**paths, temp_path=custom)["cpu_temp_c"] == 44.4


@pytest.mark.parametrize(
    ("meminfo", "loadavg", "zones", "nulls"),
    [
        ("", "", (), {"load_1m", "load_5m", "load_15m", "mem_total_kb", "cpu_temp_c"}),
        (
            "MemTotal: x kB\n",
            "a b c\n",
            [("acpitz", "1000")],
            set(METRICS) - {"cpu_count"},
        ),
        (
            "MemTotal: 10 kB\nMemAvailable: 11 kB\n",
            "nan inf -1\n",
            [("cpu-thermal", "999999")],
            set(METRICS) - {"cpu_count"},
        ),
    ],
)
def test_unknown_values_stay_null_never_zero(tmp_path, meminfo, loadavg, zones, nulls):
    m = read_host_metrics(**proc(tmp_path, meminfo, loadavg, zones))
    assert {k for k in METRICS if m[k] is None} >= nulls
    if "mem_total_kb" in nulls:
        assert m["mem_available_kb"] is None  # available>total nulls both


def test_missing_files_are_null(tmp_path):
    m = read_host_metrics(tmp_path / "none1", tmp_path / "none2", tmp_path / "none3")
    assert all(m[k] is None for k in METRICS if k != "cpu_count")


def sample(source="pinky1", boot="boot-a", sequence=1, **metrics):
    values = dict.fromkeys(METRICS)
    values.update(cpu_count=4, load_1m=0.5, **metrics)
    return host_metrics_message(source, "control-01", boot, sequence, values)


def delivery(message, *, topic=None, retained=False, qos=0):
    raw = message if isinstance(message, bytes) else wire.encode(message)
    return Delivery(
        topic or wire.topic(json.loads(raw)["source"], "host"), raw, retained, 1, qos, 0
    )


class QueueTransport:
    ready = True

    def __init__(self, items=()):
        self.items = list(items)

    def receive(self, timeout):
        if not self.items:
            raise Empty
        return self.items.pop(0)


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


def test_message_is_valid_display_sample():
    m = sample(cpu_temp_c=None)
    wire.validate(m)
    assert m["type"] == "host_metrics" and "command_id" not in m
    assert m["payload"]["cpu_temp_c"] is None and m["payload"]["sequence"] == 1


def test_receiver_states_age_and_null_preservation():
    clock = Clock()
    transport = QueueTransport([delivery(sample(cpu_temp_c=None))])
    r = HostMetricsReceiver(transport, ["pinky1", "shelf-01"], 15, clock=clock)
    rows = r.snapshot()
    assert rows["shelf-01"]["status"] == "unknown" and rows["pinky1"]["age_s"] is None
    r.poll()
    clock.now += 4
    row = r.snapshot()["pinky1"]
    assert row["status"] == "online" and row["age_s"] == 4
    assert row["cpu_temp_c"] is None and row["load_1m"] == 0.5
    assert (row["boot_version"], row["sequence"]) == ("boot-a", 1)
    assert row["received_at"] and row["observed_at"]
    clock.now += 12
    assert r.snapshot()["pinky1"]["status"] == "stale"
    assert r.snapshot()["shelf-01"]["status"] == "unknown"


def test_rejections_counted_by_reason_without_state_change():
    clock = Clock()
    r = HostMetricsReceiver(QueueTransport(), ["pinky1", "pinky2"], 15, clock=clock)
    r.consume(delivery(sample(sequence=5)))
    before = r.snapshot()
    cases = {
        "invalid_delivery": [
            delivery(sample(sequence=6), retained=True),
            delivery(sample(sequence=6), qos=1),
        ],
        "invalid_sample": [
            delivery(b"{not json", topic=wire.topic("pinky1", "host")),
            delivery(
                wire.encode(
                    wire.build_message(
                        "robot_state_request", "control-01", "pinky1", {}
                    )
                ),
                topic=wire.topic("pinky1", "host"),
            ),
        ],
        "topic_mismatch": [
            delivery(sample(sequence=6), topic=wire.topic("pinky2", "host"))
        ],
        "unknown_host": [delivery(sample("pinky3", sequence=6))],
        "out_of_order": [delivery(sample(sequence=5)), delivery(sample(sequence=4))],
    }
    r.transport.items = [d for items in cases.values() for d in items]
    r.poll()
    assert r.diagnostics()["rejected"] == {k: len(v) for k, v in cases.items()}
    assert r.snapshot() == before


def test_new_boot_accepted_and_previous_boot_retired():
    r = HostMetricsReceiver(QueueTransport(), ["pinky1"], 15, clock=Clock())
    r.consume(delivery(sample(boot="boot-a", sequence=900)))
    r.consume(delivery(sample(boot="boot-b", sequence=1)))
    assert r.snapshot()["pinky1"]["boot_version"] == "boot-b"
    with pytest.raises(ValueError, match="out_of_order"):
        r.consume(delivery(sample(boot="boot-a", sequence=901)))
    assert r.snapshot()["pinky1"]["sequence"] == 1


def test_poll_is_bounded_and_survives_disconnect():
    items = [delivery(sample(sequence=n)) for n in range(1, 41)]
    transport = QueueTransport(items)
    r = HostMetricsReceiver(transport, ["pinky1"], 15, clock=Clock())
    r.poll()
    assert len(transport.items) == 8 and r.snapshot()["pinky1"]["sequence"] == 32

    class Broken(QueueTransport):
        ready = False

        def receive(self, timeout):
            raise ConnectionError("down")

    r.transport = Broken()
    r.poll()
    assert r.diagnostics()["connected"] is False


@pytest.mark.parametrize(
    ("hosts", "stale"), [([], 15), (["omx1"], 15), (["pinky1"], 0), (["pinky1"], None)]
)
def test_receiver_requires_explicit_known_hosts_and_stale_limit(hosts, stale):
    with pytest.raises(ValueError):
        HostMetricsReceiver(QueueTransport(), hosts, stale)


class FakeClient:
    def __init__(self, rc=0):
        self.rc, self.calls = rc, []

    def publish(self, topic, raw, qos, retain, properties):
        self.calls.append((topic, json.loads(raw), qos, retain, properties))
        return type("Info", (), {"rc": self.rc})()


def test_publish_host_metrics_is_qos0_and_topic_checked():
    transport = MqttTransport(
        MqttConfig("127.0.0.1", "unit-host"), [wire.topic("pinky1", "host")]
    )
    message = sample()
    with pytest.raises(ConnectionError):
        transport.publish_host_metrics(wire.topic("pinky1", "host"), message, 5)
    transport._ready.set()
    transport._client = FakeClient()
    with pytest.raises(ValueError):
        transport.publish_host_metrics(wire.topic("pinky2", "host"), message, 5)
    with pytest.raises(ValueError):
        transport.publish_host_metrics(
            wire.topic("pinky1", "host"),
            wire.build_message("robot_state_request", "control-01", "pinky1", {}),
            5,
        )
    transport.publish_host_metrics(wire.topic("pinky1", "host"), message, 5)
    topic, body, qos, retain, properties = transport._client.calls[-1]
    assert (topic, body, qos, retain) == (
        wire.topic("pinky1", "host"),
        message,
        0,
        False,
    )
    assert properties.MessageExpiryInterval == 5
    transport._client = FakeClient(rc=4)
    with pytest.raises(ConnectionError):
        transport.publish_host_metrics(wire.topic("pinky1", "host"), message, 5)


def test_module_is_portable_stdlib_only():
    code = (
        "import sys, importlib.util as u;"
        "sys.modules['paho']=None;"
        "import roport_control_server.adapters.mqtt.host_metrics"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_rejected_payload_is_not_logged(caplog):
    r = HostMetricsReceiver(
        QueueTransport(
            [delivery(b'{"secret-ish":1}', topic=wire.topic("pinky1", "host"))]
        ),
        ["pinky1"],
        15,
        clock=Clock(),
    )
    with caplog.at_level(logging.DEBUG):
        r.poll()
    assert "secret-ish" not in caplog.text


def test_broker_roundtrip_is_qos0_non_retained(broker):
    from mqtt_helpers import wait_until

    port = broker[0]
    publisher = MqttTransport(
        MqttConfig("127.0.0.1", "pinky1-host-pub", port=port),
        [wire.topic("pinky1", "cmd")],
    )
    subscriber = MqttTransport(
        MqttConfig("127.0.0.1", "control-host", port=port),
        [wire.topic("pinky1", "host")],
    )
    receiver = HostMetricsReceiver(subscriber, ["pinky1"], 15)
    publisher.start()
    subscriber.start()
    try:
        publisher.publish_host_metrics(wire.topic("pinky1", "host"), sample(), 5)

        def received():
            receiver.poll()
            return receiver.snapshot()["pinky1"]["status"] == "online"

        wait_until(received)
        assert receiver.diagnostics() == {
            "connected": True,
            "rejected": dict.fromkeys(sorted(receiver.rejected), 0),
        }
    finally:
        publisher.stop()
        subscriber.stop()


def test_runner_passes_explicit_period_and_publishes_each_loop(monkeypatch):
    from roport_control_server.adapters.mqtt import trial_runner

    seen = {}

    class Client:
        transport = type("T", (), {"ready": True})()
        halted = None

        def __init__(self, *args, host_metrics_period_s=None):
            seen["period"] = host_metrics_period_s
            seen["published"] = 0
            self.polls = 0

        def start(self):
            pass

        def poll(self, timeout):
            self.polls += 1
            if self.polls > 1:
                raise KeyboardInterrupt

        def report(self, *args, **kwargs):
            pass

        def publish_motion(self):
            pass

        def publish_host_metrics(self):
            seen["published"] += 1

        def close(self):
            pass

    class Driver:
        def __getattr__(self, name):
            return lambda *a, **k: []

    monkeypatch.setattr(trial_runner, "FieldPinkyClient", Client)
    profile = {"robots": {"logistics": {"robot_id": "pinky1"}}}
    trial_runner.run(profile, "logistics", None, "j", Driver())
    assert seen == {"period": None, "published": 1}
    trial_runner.run(
        profile, "logistics", None, "j", Driver(), host_metrics_period_s=5.0
    )
    assert seen["period"] == 5.0


@pytest.mark.parametrize("value", ["0", "-5", "nan", "inf"])
def test_runner_cli_rejects_invalid_period_before_config(tmp_path, value):
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "roport_control_server.adapters.mqtt.trial_runner",
            "--config",
            str(tmp_path / "missing.json"),
            "--role",
            "logistics",
            "--journal",
            str(tmp_path / "j.json"),
            "--host-metrics-period-s",
            value,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert "--host-metrics-period-s must be a positive number" in result.stderr
    assert not (tmp_path / "j.json").exists()
