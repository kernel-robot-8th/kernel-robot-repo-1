from dedicated_udp import serve_once


class SocketStub:
    def __init__(self, *peers):
        self.peers = list(peers)
        self.response = None

    def recvfrom(self, limit):
        assert limit == 60_001
        return b"{bad-json", self.peers.pop(0)

    def sendto(self, data, peer):
        self.response = data, peer


def test_unexpected_peer_is_dropped_then_assigned_peer_is_served():
    from dedicated_policy import PolicyRouter, decode_datagram

    peer = "127.0.0.1", 12345
    sock = SocketStub(("127.0.0.1", 23456), peer)
    router = PolicyRouter({}, {})
    assert (
        serve_once(sock, router, allowed_peer=peer, clock=lambda: 0)
        == "unexpected_policy_peer"
    )
    assert sock.response is None
    assert serve_once(sock, router, allowed_peer=peer, clock=lambda: 0) is None
    assert decode_datagram(sock.response[0])["type"] == "inference.error"
    assert sock.response[1] == peer


def test_malformed_datagram_is_explicit_error_on_assigned_peer():
    from dedicated_policy import PolicyRouter, decode_datagram

    peer = "127.0.0.1", 12345
    sock = SocketStub(peer)
    assert (
        serve_once(sock, PolicyRouter({}, {}), allowed_peer=peer, clock=lambda: 0)
        is None
    )
    assert decode_datagram(sock.response[0])["type"] == "inference.error"
    assert sock.response[1] == peer
