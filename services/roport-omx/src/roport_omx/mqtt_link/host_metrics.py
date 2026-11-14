# Copied unchanged from services/roport-control-server/.../adapters/mqtt (616ddf5); update both together.
"""Display-only host telemetry (stdlib + this folder). Never gates control.

Unknown values stay null at every layer; null is never turned into 0.
"""

import math
import os
from collections import deque
from datetime import UTC, datetime
from pathlib import Path
from queue import Empty
from time import monotonic

from . import field_wire as wire

METRICS = (
    "cpu_count",
    "load_1m",
    "load_5m",
    "load_15m",
    "mem_total_kb",
    "mem_available_kb",
    "cpu_temp_c",
)
CPU_ZONES = {"cpu-thermal", "x86_pkg_temp"}
REASONS = {
    "invalid_sample",
    "invalid_delivery",
    "topic_mismatch",
    "unknown_host",
    "out_of_order",
}
MAX_POLL = 32


def _text(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeError):
        return None


def _bounded(value, low, high):
    return value if math.isfinite(value) and low <= value <= high else None


def _loadavg(path):
    try:
        values = [float(v) for v in (_text(path) or "").split()[:3]]
    except ValueError:
        values = []
    if len(values) != 3:
        return None, None, None
    return tuple(_bounded(v, 0, 10000) for v in values)


def _meminfo(path):
    found = {}
    for line in (_text(path) or "").splitlines():
        key, _, rest = line.partition(":")
        parts = rest.split()
        if key in {"MemTotal", "MemAvailable"} and parts[1:] == ["kB"]:
            try:
                found[key] = int(parts[0])
            except ValueError:
                pass
    total, available = found.get("MemTotal"), found.get("MemAvailable")
    if total is not None and total < 0:
        total = None
    if available is not None and available < 0:
        available = None
    if None not in (total, available) and available > total:
        return None, None
    return total, available


def _temperature(thermal_root, temp_path):
    if temp_path is None:
        root = Path(thermal_root)
        zones = sorted(root.glob("thermal_zone*")) if root.is_dir() else []
        zone = next(
            (z for z in zones if (_text(z / "type") or "").strip() in CPU_ZONES), None
        )
        if zone is None:
            return None
        temp_path = zone / "temp"
    try:
        milli = int((_text(temp_path) or "").strip())
    except ValueError:
        return None
    return _bounded(round(milli / 1000, 1), -100, 200)


def read_host_metrics(
    meminfo="/proc/meminfo",
    loadavg="/proc/loadavg",
    thermal_root="/sys/class/thermal",
    temp_path=None,
):
    """temp_path overrides zone discovery; it must hold millidegrees Celsius."""
    load = _loadavg(loadavg)
    total, available = _meminfo(meminfo)
    return {
        "cpu_count": os.cpu_count(),
        "load_1m": load[0],
        "load_5m": load[1],
        "load_15m": load[2],
        "mem_total_kb": total,
        "mem_available_kb": available,
        "cpu_temp_c": _temperature(thermal_root, temp_path),
    }


def host_metrics_message(source, target, boot_version, sequence, metrics):
    return wire.build_message(
        "host_metrics",
        source,
        target,
        {
            "boot_version": boot_version,
            "sequence": sequence,
            "observed_at": datetime.now(UTC).isoformat(),
            **{key: metrics[key] for key in METRICS},
        },
    )


class HostMetricsReceiver:
    """Latest sample per host. Online means fresh telemetry, not robot readiness."""

    def __init__(self, transport, hosts, stale_after_s, clock=monotonic):
        hosts = list(hosts)
        if (
            not hosts
            or len(set(hosts)) != len(hosts)
            or not set(hosts) <= wire.HOSTS
            or type(stale_after_s) not in (int, float)
            or not 0 < stale_after_s < math.inf
        ):
            raise ValueError("host metrics require known hosts and stale_after_s > 0")
        self.transport, self.hosts = transport, hosts
        self.stale_after_s, self.clock = stale_after_s, clock
        self.latest = {}
        # ponytail: last 8 retired boots per host; older late samples look like new boots.
        self.retired = {host: deque(maxlen=8) for host in hosts}
        self.rejected = dict.fromkeys(sorted(REASONS), 0)

    def consume(self, delivery):
        if delivery.retained or delivery.qos != 0:
            raise ValueError("invalid_delivery")
        try:
            message = wire.decode(delivery.payload)
        except ValueError:
            raise ValueError("invalid_sample") from None
        if message["type"] != "host_metrics":
            raise ValueError("invalid_sample")
        host, p = message["source"], message["payload"]
        if delivery.topic != wire.topic(host, "host"):
            raise ValueError("topic_mismatch")
        if host not in self.hosts:
            raise ValueError("unknown_host")
        previous = self.latest.get(host)
        if p["boot_version"] in self.retired[host] or (
            previous
            and previous["payload"]["boot_version"] == p["boot_version"]
            and p["sequence"] <= previous["payload"]["sequence"]
        ):
            raise ValueError("out_of_order")
        if previous and previous["payload"]["boot_version"] != p["boot_version"]:
            self.retired[host].append(previous["payload"]["boot_version"])
        self.latest[host] = {
            "payload": p,
            "received": self.clock(),
            "received_at": datetime.now(UTC).isoformat(),
        }

    def poll(self):
        for _ in range(MAX_POLL):
            try:
                delivery = self.transport.receive(0)
            except (Empty, ConnectionError):
                break
            try:
                self.consume(delivery)
            except ValueError as exc:
                # Count the reason only; payloads are never logged.
                reason = str(exc) if str(exc) in REASONS else "invalid_sample"
                self.rejected[reason] += 1

    def diagnostics(self):
        return {
            "connected": bool(self.transport.ready),
            "rejected": dict(self.rejected),
        }

    def snapshot(self):
        now = self.clock()
        result = {}
        for host in self.hosts:
            sample = self.latest.get(host)
            if sample is None:
                row = dict.fromkeys(
                    (
                        "age_s",
                        "received_at",
                        "observed_at",
                        "boot_version",
                        "sequence",
                        *METRICS,
                    )
                )
                result[host] = {"host_id": host, "status": "unknown", **row}
                continue
            p, age = sample["payload"], max(0, now - sample["received"])
            result[host] = {
                "host_id": host,
                "status": "stale" if age > self.stale_after_s else "online",
                "age_s": age,
                "received_at": sample["received_at"],
                **{
                    key: p[key]
                    for key in ("observed_at", "boot_version", "sequence", *METRICS)
                },
            }
        return result
