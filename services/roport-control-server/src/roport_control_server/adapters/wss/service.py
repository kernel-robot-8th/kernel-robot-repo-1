"""Service link with durable inbox/outbox and reconnect replay."""

import asyncio
import json
import logging
import re
from datetime import UTC, datetime
from time import monotonic
from urllib.parse import urlsplit
from uuid import uuid4

from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, WebSocketException

from ...contracts.service_wire import ServiceStorageError, validate_service_message

LOG = logging.getLogger(__name__)


class ServiceLink:
    def __init__(self, url, token, site_id, store):
        parsed = urlsplit(url)
        if (
            parsed.scheme not in {"ws", "wss"}
            or not parsed.hostname
            or parsed.username
            or parsed.password
            or parsed.query
            or parsed.fragment
        ):
            raise ValueError("service URL must be ws(s) without credentials or query")
        if parsed.scheme == "ws" and parsed.hostname not in {
            "127.0.0.1",
            "::1",
            "localhost",
        }:
            raise ValueError("remote service requires wss")
        if not token or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", site_id):
            raise ValueError("service token and site required")
        if parsed.path != f"/ws/v1/sites/{site_id}/control":
            raise ValueError("service URL site differs from configured site")
        self.url, self.token, self.site_id, self.store = url, token, site_id, store

    async def exchange(self, socket, stop):
        next_flush = 0.0
        while not stop.is_set():
            if monotonic() >= next_flush:
                for message in await asyncio.to_thread(self.store.outbox):
                    if message["site_id"] != self.site_id:
                        raise ValueError("outbox belongs to another site")
                    await socket.send(json.dumps(message))
                next_flush = monotonic() + 1
            try:
                raw = await asyncio.wait_for(socket.recv(), timeout=1)
            except TimeoutError:
                continue
            try:
                message = json.loads(raw)
                if (
                    not isinstance(message, dict)
                    or message.get("site_id") != self.site_id
                ):
                    raise ValueError("unexpected service site")
                validate_service_message(message, message.get("type"))
            except (ValueError, TypeError):
                LOG.warning("invalid_service_frame")
                continue  # No trusted identity to ACK; keep processing later frames.
            if message["type"] == "message.ack":
                await asyncio.to_thread(
                    self.store.acknowledge,
                    message["payload"]["message_id"],
                    self.site_id,
                )
                continue
            reason = await asyncio.to_thread(self.store.receive, message)
            if reason:
                LOG.warning("service_message_quarantined: %s", reason)
            # Acceptance or quarantine has committed; ACK means stored, never executed.
            await socket.send(
                json.dumps(
                    {
                        "schema_version": "1.0",
                        "message_id": str(uuid4()),
                        "type": "message.ack",
                        "site_id": self.site_id,
                        "source": "control-01",
                        "target": "service-01",
                        "sent_at": datetime.now(UTC).isoformat(),
                        "payload": {"message_id": message["message_id"]},
                    }
                )
            )

    async def run(self, stop):
        while not stop.is_set():
            try:
                async with connect(
                    self.url,
                    additional_headers={"Authorization": f"Bearer {self.token}"},
                    max_size=64 * 1024,
                ) as socket:
                    await self.exchange(socket, stop)
            except ConnectionClosed as exc:
                if exc.rcvd is not None and exc.rcvd.code == 4001:
                    return  # A replacement owner must not be evicted by reconnect.
            except ServiceStorageError:
                LOG.warning("service_storage_unavailable")
            except (
                OSError,
                WebSocketException,
                ValueError,
                TypeError,
                KeyError,
            ):
                # Do not log URLs, bearer tokens or customer payloads.
                pass
            if not stop.is_set():
                await asyncio.sleep(1)
