import pytest

from dedicated_udp import serve_once


class SocketStub:
    def __init__(self, peer):
        self.peer = peer
        self.response = None

    def recvfrom(self, limit):
        assert limit == 60_001
        return b"{bad-json", self.peer

    def sendto(self, data, peer):
        self.response = data, peer


def test_unexpected_peer_gets_no_response():
    from dedicated_policy import PolicyRouter

    sock = SocketStub(("127.0.0.1", 23456))
    with pytest.raises(ValueError, match="unexpected_policy_peer"):
        serve_once(sock, PolicyRouter({}, {}), allowed_peer=("127.0.0.1", 12345))
    assert sock.response is None


def test_malformed_datagram_is_explicit_error_on_assigned_peer():
    from dedicated_policy import PolicyRouter, decode_datagram

    peer = "127.0.0.1", 12345
    sock = SocketStub(peer)
    serve_once(sock, PolicyRouter({}, {}), allowed_peer=peer, clock=lambda: 0)
    assert decode_datagram(sock.response[0])["type"] == "inference.error"
    assert sock.response[1] == peer
