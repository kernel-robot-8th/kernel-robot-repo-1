"""Bounded MQTT 5 transport. PUBACK is not execution or durable application ACK."""

import logging
import math
import ssl
from dataclasses import dataclass, field
from datetime import UTC, datetime
from queue import Full, Queue
from threading import Event, Lock

import paho.mqtt.client as mqtt
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
from paho.mqtt.subscribeoptions import SubscribeOptions

from .messages import MAX_MESSAGE_BYTES, encode, timestamp, topic


@dataclass(frozen=True)
class MqttConfig:
    host: str
    client_id: str
    port: int = 1883
    tls: bool = False
    ca_file: str | None = None
    username: str | None = None
    password: str | None = field(default=None, repr=False)
    queue_size: int = 128

    def __post_init__(self):
        if not self.host or not self.client_id or not 1 <= self.port <= 65535:
            raise ValueError("host, unique client_id and valid port are required")
        if not 1 <= self.queue_size <= 65535:
            raise ValueError("queue_size must be 1..65535")
        if self.password is not None and (not self.username or not self.tls):
            raise ValueError("password authentication requires username and TLS")
        if self.ca_file and not self.tls:
            raise ValueError("ca_file requires TLS")


@dataclass(frozen=True)
class Delivery:
    topic: str
    payload: bytes
    retained: bool
    mid: int
    qos: int
    generation: int


class MqttTransport:
    def __init__(self, config: MqttConfig, subscriptions: list[str]):
        if not subscriptions or any(
            "+" in t or "#" in t or not t for t in subscriptions
        ):
            raise ValueError("use explicit subscriptions for assigned devices")
        self.config = config
        self._subscriptions = subscriptions.copy()
        self._incoming: Queue[Delivery] = Queue(maxsize=config.queue_size)
        self._ready = Event()
        self._lock = Lock()
        self._publish_lock = Lock()
        self._results_lock = Lock()
        self._publish_results: dict[int, bool] = {}
        self.rejected_messages = 0
        self._generation = 0
        self._sub_mid = None
        self._fault: str | None = None
        self._started = False
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=config.client_id,
            protocol=mqtt.MQTTv5,
            manual_ack=True,
        )
        self._client.max_queued_messages_set(config.queue_size)
        self._client.max_inflight_messages_set(min(config.queue_size, 20))
        self._client.reconnect_delay_set(1, 10)
        if config.tls:
            context = ssl.create_default_context(cafile=config.ca_file)
            context.minimum_version = ssl.TLSVersion.TLSv1_2
            self._client.tls_set_context(context)
        if config.username:
            self._client.username_pw_set(config.username, config.password)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_subscribe = self._on_subscribe
        self._client.on_message = self._on_message
        self._client.on_publish = self._on_publish

    @property
    def ready(self) -> bool:
        """Connection and subscriptions only; not device readiness."""
        return self._ready.is_set() and self._fault is None

    @property
    def fault(self) -> str | None:
        return self._fault

    def start(self, timeout: float = 5) -> None:
        if self._started:
            raise RuntimeError("create a new transport after stop or fault")
        self._started = True
        properties = Properties(PacketTypes.CONNECT)
        properties.SessionExpiryInterval = 0
        properties.ReceiveMaximum = self.config.queue_size
        properties.MaximumPacketSize = MAX_MESSAGE_BYTES + 1024
        self._client.connect_async(
            self.config.host,
            self.config.port,
            keepalive=15,
            clean_start=True,
            properties=properties,
        )
        self._client.loop_start()
        if not self._ready.wait(timeout) or self._fault:
            self.stop()
            raise ConnectionError(self._fault or "MQTT connect/subscribe timeout")

    def stop(self) -> None:
        self._ready.clear()
        self._client.disconnect()
        self._client.loop_stop()

    def _fail(self, reason: str) -> None:
        self._fault = reason
        self._ready.clear()
        self._client.disconnect()

    def _on_connect(self, client, userdata, flags, reason, properties):
        with self._lock:
            self._generation += 1
            if reason.is_failure:
                self._fail("MQTT connection rejected")
                return
            rc, self._sub_mid = client.subscribe(
                [
                    (t, SubscribeOptions(qos=1, retainAsPublished=True))
                    for t in self._subscriptions
                ]
            )
            if rc != mqtt.MQTT_ERR_SUCCESS:
                self._fail("MQTT subscribe failed")

    def _on_subscribe(self, client, userdata, mid, reasons, properties):
        if mid != self._sub_mid or any(r.is_failure for r in reasons):
            self._fail("MQTT subscription rejected")
        elif self._fault is None:
            self._ready.set()

    def _on_disconnect(self, client, userdata, flags, reason, properties):
        with self._lock:
            self._ready.clear()
            self._generation += 1

    def _on_message(self, client, userdata, message):
        if len(message.payload) > MAX_MESSAGE_BYTES:
            self.rejected_messages += 1
            logging.getLogger(__name__).warning("Rejected oversized MQTT message")
            if client.ack(message.mid, message.qos) != mqtt.MQTT_ERR_SUCCESS:
                self._fail("MQTT rejected-message ACK failed")
            return
        try:
            self._incoming.put_nowait(
                Delivery(
                    message.topic,
                    bytes(message.payload),
                    message.retain,
                    message.mid,
                    message.qos,
                    self._generation,
                )
            )
        except Full:
            # No silent loss of task events: stop and require caller reconciliation.
            self._fail("MQTT receive queue full; reconciliation required")

    def _on_publish(self, client, userdata, mid, reason, properties):
        with self._results_lock:
            self._publish_results.pop(mid, None)
            self._publish_results[mid] = reason.is_failure
            if len(self._publish_results) > self.config.queue_size:
                self._publish_results.pop(next(iter(self._publish_results)))

    def receive(self, timeout: float = 1) -> Delivery:
        if self._fault:
            raise ConnectionError(self._fault)
        delivery = self._incoming.get(timeout=timeout)
        if not self.ready or delivery.generation != self._generation:
            raise ConnectionError("stale delivery; reconcile after reconnect")
        return delivery

    def ack(self, delivery: Delivery) -> None:
        """Consumer calls after handling; caller owns any required durable commit."""
        with self._lock:
            if not self.ready or delivery.generation != self._generation:
                raise ConnectionError("stale delivery; reconcile after reconnect")
            if self._client.ack(delivery.mid, delivery.qos) != mqtt.MQTT_ERR_SUCCESS:
                raise ConnectionError("MQTT transport ACK failed")

    def publish(self, destination: str, message: dict, timeout: float = 5) -> None:
        """QoS1, non-retained task messages. Waits for broker PUBACK only."""
        with self._publish_lock:
            self._publish(destination, message, timeout)

    def _publish(self, destination: str, message: dict, timeout: float) -> None:
        if not self.ready:
            raise ConnectionError(self._fault or "MQTT subscriptions not ready")
        raw = encode(message)
        parts = destination.split("/")
        if len(parts) != 6 or destination != topic(
            message["site_id"], parts[3], parts[4], parts[5]
        ):
            raise ValueError("topic and envelope routing mismatch")
        device = message["target"] if parts[5] == "cmd" else message["source"]
        if device != parts[4]:
            raise ValueError("topic and envelope device mismatch")
        properties = None
        if destination.endswith("/cmd"):
            remaining = (
                timestamp(message.get("expires_at")) - datetime.now(UTC)
            ).total_seconds()
            if remaining <= 0:
                raise ValueError("expired_command")
            properties = Properties(PacketTypes.PUBLISH)
            properties.MessageExpiryInterval = min(math.ceil(remaining), 2**32 - 1)
        elif not destination.endswith("/event"):
            raise ValueError("only task cmd/event publishing is implemented")
        info = self._client.publish(
            destination, raw, qos=1, retain=False, properties=properties
        )
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            raise ConnectionError(f"MQTT publish failed: {info.rc}")
        info.wait_for_publish(timeout)
        if self._fault:
            raise ConnectionError(self._fault)
        if not info.is_published():
            raise TimeoutError(
                "PUBACK timeout; delivery is unknown, retry with the same command ID"
            )
        with self._results_lock:
            rejected = self._publish_results.pop(info.mid, None)
        if rejected is None:
            raise ConnectionError("PUBACK result unavailable; reconcile delivery")
        if rejected:
            raise ConnectionError("MQTT broker rejected publish")
