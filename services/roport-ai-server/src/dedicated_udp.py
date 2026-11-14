"""Explicit single-peer P0 UDP adapter; importing this module opens no socket."""

import logging
import socket
import time
from collections.abc import Callable

from dedicated_policy import MAX_DATAGRAM_BYTES, PolicyRouter


def serve_once(
    sock: socket.socket,
    router: PolicyRouter,
    *,
    allowed_peer: tuple[str, int],
    clock: Callable[[], float] = time.time,
) -> str | None:
    """Process one datagram on an already configured socket, serially.

    Returns ``None`` after replying, or a drop reason (``"unexpected_policy_peer"``)
    when the datagram came from another peer; dropped datagrams get no reply and
    are logged, so the caller keeps looping. ``allowed_peer`` is an IPv4
    ``(host, port)`` 2-tuple only; IPv6 4-tuple addresses never match.

    The caller owns binding, peer configuration, timeout and lifecycle. The Host
    still owns all expiry/phase checks and holds when a reply is lost or late.
    """
    data, peer = sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
    if peer != allowed_peer:
        logging.getLogger(__name__).warning(
            "Dropped datagram from unexpected peer %s", peer
        )
        return "unexpected_policy_peer"
    response = router.handle_datagram(data, now=clock())
    sock.sendto(response, peer)
    return None
