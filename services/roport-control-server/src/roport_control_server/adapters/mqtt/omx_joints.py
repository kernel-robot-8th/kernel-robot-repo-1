"""Best-effort shelf-arm samples, isolated from the fleet command transport."""

from dataclasses import replace
from datetime import UTC, datetime
from threading import Lock
from time import monotonic

from . import field_wire as wire
from .mqtt import MqttTransport


class OmxJointReceiver(MqttTransport):
    def __init__(self, connection, target, observed, clock=monotonic):
        super().__init__(
            replace(connection, client_id=f"{connection.client_id}-joint-display"),
            [wire.topic(arm, "motion") for arm in wire.SHELF_ARMS],
        )
        self.target = target
        self.observed = observed
        self.clock = clock
        self.latest = {}
        self._sample_lock = Lock()

    def _on_message(self, client, userdata, message):
        if message.qos:
            client.ack(message.mid, message.qos)
            return
        if message.retain:
            return
        try:
            sample = wire.decode(message.payload)
            if sample["type"] != "omx_joint_state" or sample["target"] != self.target:
                return
            arm, p = sample["source"], sample["payload"]
            if message.topic != wire.topic(arm, "motion"):
                return
            measured = wire.timestamp(p["observed_at"])
            age = (datetime.now(UTC) - measured).total_seconds()
        except (ValueError, OverflowError):
            return
        if not -1 <= age <= self.stale_after(p):
            return
        with self._sample_lock:
            if self.observed.get(arm, {}).get("boot_version") != p["boot_version"]:
                return
            previous = self.latest.get(arm)
            if (
                previous
                and previous[0]["boot_version"] == p["boot_version"]
                and (
                    p["sequence"] <= previous[0]["sequence"] or measured <= previous[1]
                )
            ):
                return
            self.latest[arm] = (p, measured, self.clock() - max(0.0, age))

    @staticmethod
    def stale_after(payload):
        return 2 if payload["sample_kind"] == "mission" else 20

    def snapshot(self):
        with self._sample_lock:
            rows = {}
            for arm, (p, _, sampled_at) in self.latest.items():
                if self.observed.get(arm, {}).get("boot_version") != p["boot_version"]:
                    continue
                age = max(0, self.clock() - sampled_at)
                status = "idle" if p["sample_kind"] == "idle" else "live"
                rows[arm] = {
                    **p,
                    "age_s": age,
                    "status": "stale" if age > self.stale_after(p) else status,
                }
            return rows
