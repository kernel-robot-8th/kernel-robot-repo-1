"""Explicit single-peer P0 UDP adapter; importing this module opens no socket."""

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
) -> None:
    """Process one datagram on an already configured socket, serially.

    The caller owns binding, peer configuration, timeout and lifecycle. The Host
    still owns all expiry/phase checks and holds when a reply is lost or late.
    """
    data, peer = sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
    if peer != allowed_peer:
        raise ValueError("unexpected_policy_peer")
    response = router.handle_datagram(data, now=clock())
    sock.sendto(response, peer)
