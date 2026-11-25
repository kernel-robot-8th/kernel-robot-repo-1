"""OMX inference UDP wire and server over loopback; no GPU, no model, no robot."""

import json
import runpy
import socket
import sys
import time
from pathlib import Path
from uuid import uuid4

import pytest

from omx_udp import errors, wire
from omx_udp import server as server_module
from omx_udp.panel_policy import PanelPolicy
from omx_udp.server import InferenceServer, PolicyError, bind_address, read_key
from omx_udp.wire import ACTIONS, ERROR, INFER, LOAD, LOADED, PING, PONG, Message

KEY = b"k" * 32
PEER = ("127.0.0.1", 40000)


def request(kind, request_id, header=None, data=b"", session=None):
    return Message(kind, session or SESSION, request_id, header or {}, data)


SESSION = uuid4()


def infer(request_id, step, data=b"", **change):
    header = {
        "schema_version": wire.SCHEMA,
        "inference_id": f"inf-{request_id}",
        "observation_seq": request_id,
        "task_id": "task-test-001",
        "command_id": "cmd-test-001",
        "execution_id": "exec-test-001",
        "model": "/models/m",
        "deadline": "2026-11-09T12:00:01.000Z",
        "obs": [],
        "step": step,
    } | change
    return request(INFER, request_id, header, data)


def test_large_message_round_trips_in_any_order():
    data = bytes(range(256)) * 400  # ~100 KB: about 90 datagrams
    datagrams = wire.pack(request(INFER, 1, {"obs": []}, data), KEY)
    assert len(datagrams) > 80
    assert all(len(d) <= wire.DATAGRAM_BYTES for d in datagrams)
    r = wire.Reassembler(KEY)
    results = [r.feed(d, PEER) for d in reversed(datagrams)]
    assert results[:-1] == [None] * (len(datagrams) - 1)
    assert results[-1] == request(INFER, 1, {"obs": []}, data)


def test_missing_chunk_never_yields_a_message_and_newer_request_wins():
    old = wire.pack(request(INFER, 1, {"obs": []}, b"x" * 5000), KEY)
    new = wire.pack(request(INFER, 2, {"obs": []}, b"y" * 5000), KEY)
    r = wire.Reassembler(KEY)
    assert [r.feed(d, PEER) for d in old[:-1]] == [None] * (len(old) - 1)
    *_, done = [r.feed(d, PEER) for d in new]
    assert done.request_id == 2
    assert r.feed(old[-1], PEER) is None  # the rest of #1 arrives late
    assert r.stats["superseded"] == 1 and r.stats["late"] == 1


def test_incomplete_message_expires():
    datagrams = wire.pack(request(INFER, 1, {}, b"x" * 5000), KEY)
    r = wire.Reassembler(KEY, timeout=0.01)
    r.feed(datagrams[0], PEER)
    time.sleep(0.02)
    r.expire()
    assert r.stats["timeout"] == 1


def test_lost_fragments_are_counted():
    first = wire.pack(request(INFER, 1, {}, b"x" * 5000), KEY)
    second = wire.pack(request(INFER, 2, {}, b"y" * 5000), KEY)
    r = wire.Reassembler(KEY, timeout=0.01)
    for d in first[:-2]:  # two fragments of #1 never come
        r.feed(d, PEER)
    for d in second[:-1]:  # a newer request replaces #1, then loses its last fragment
        r.feed(d, PEER)
    time.sleep(0.02)
    r.expire()
    n = len(first)
    assert r.stats["fragments_expected"] == 2 * n
    assert r.stats["fragments_received"] == 2 * n - 3
    assert r.stats["fragments_missing"] == 3
    assert (r.stats["superseded_infer"], r.stats["timeout_infer"]) == (1, 1)
    assert r.stats["completed"] == 0


@pytest.mark.parametrize(
    "case", ["wrong_key", "flipped_byte", "untrusted_peer", "replayed", "bad_magic"]
)
def test_rejected_datagrams(case):
    (datagram,) = wire.pack(request(PING, 1), KEY)
    r = wire.Reassembler(KEY, peers={"127.0.0.1"})
    if case == "wrong_key":
        (datagram,) = wire.pack(request(PING, 1), b"z" * 32)
    elif case == "flipped_byte":
        datagram = datagram[:-1] + bytes([datagram[-1] ^ 1])
    elif case == "untrusted_peer":
        assert r.feed(datagram, ("10.0.0.9", 1)) is None
        return
    elif case == "replayed":
        assert r.feed(datagram, PEER) is not None
    elif case == "bad_magic":
        datagram = b"XXXX" + datagram[4:]
    assert r.feed(datagram, PEER) is None


def test_malformed_datagrams_and_bodies_are_counted_apart():
    r = wire.Reassembler(KEY)
    (datagram,) = wire.pack(request(PING, 1), KEY)
    assert r.feed(b"XXXX" + datagram[4:], PEER) is None  # refused before reassembly
    assert r.feed(datagram[:10], PEER) is None
    # a verified message (CRC and HMAC pass) whose JSON header is not an object
    (unreadable,) = wire.pack(Message(PING, SESSION, 2, [1, 2]), KEY)
    assert r.feed(unreadable, PEER) is None
    assert (r.stats["malformed_datagram"], r.stats["malformed_body"]) == (2, 1)
    assert r.stats["malformed"] == 0


def test_unverified_datagram_never_blocks_a_stream():
    # a corrupted or forged request_id (wrong MAC) must not raise the stream's newest id
    (forged,) = wire.pack(request(PING, 2**63), b"z" * 32)
    r = wire.Reassembler(KEY)
    assert r.feed(forged, PEER) is None and r.stats["bad_mac"] == 1
    (ok,) = wire.pack(request(PING, 1), KEY)
    assert r.feed(ok, PEER).request_id == 1


def test_unverified_incomplete_message_only_holds_a_stream_until_it_expires():
    forged = wire.pack(request(INFER, 2**63, {}, b"x" * 5000), b"z" * 32)
    r = wire.Reassembler(KEY, timeout=0.01)
    r.feed(forged[0], PEER)
    (ok,) = wire.pack(request(PING, 1), KEY)
    assert r.feed(ok, PEER) is None and r.stats["late"] == 1
    time.sleep(0.02)
    assert r.feed(ok, PEER).request_id == 1


def test_verified_streams_are_capped():
    r = wire.Reassembler(KEY, max_streams=2)
    sessions = [uuid4() for _ in range(3)]
    for session in sessions:
        (datagram,) = wire.pack(request(PING, 1, session=session), KEY)
        assert r.feed(datagram, PEER) is not None
    assert [s for _, s in r._latest] == sessions[1:]


def test_non_finite_numbers_are_refused_both_ways():
    with pytest.raises(ValueError):
        wire.encode_body({"actions": [[float("nan")]]})
    # what an older peer could still send
    raw = json.dumps({"actions": [[float("inf")]]}).encode()
    with pytest.raises(ValueError):
        wire.decode_body(len(raw).to_bytes(4, "big") + raw)


def test_short_key_is_refused():
    with pytest.raises(ValueError):
        wire.Reassembler(b"short")


class FakePolicy:
    def __init__(self, delay=0.0):
        self.delay, self.seen = delay, []

    def load(self, model, overrides):
        if model == "/bad":
            raise PolicyError("MODEL_NOT_ALLOWED")
        return {"ok": True, "image_size": None, "n_obs": 1}

    def infer(self, header, data):
        self.seen.append(header["step"])
        time.sleep(self.delay)
        if header["step"] == "boom":
            raise RuntimeError("cuda")
        if header["step"] == "nan":
            return {"actions": [[0.1, float("nan")]], "original": [[0.0, 0.0]]}
        if header["step"] == "huge":  # a reply over MAX_MESSAGE_BYTES
            return {"actions": [[0]], "original": [[0]], "note": "x" * 5_000_000}
        return {"actions": [[len(data)]], "original": [[0]]}

    def status(self):
        return {"loaded": []}


@pytest.fixture
def link():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_sock.bind(("127.0.0.1", 0))
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.bind(("127.0.0.1", 0))
    client.settimeout(2)
    policy = FakePolicy()
    server = InferenceServer(server_sock, policy, KEY, peers={"127.0.0.1"})
    replies = wire.Reassembler(KEY)

    def call(message, *, serve=True):
        for d in wire.pack(message, KEY):
            client.sendto(d, server_sock.getsockname())
        if not serve:
            return None
        server.serve_once(2)
        while True:
            packet, peer = client.recvfrom(65535)
            reply = replies.feed(packet, peer)
            if reply is not None:
                return reply

    yield call, server, policy
    server_sock.close()
    client.close()


def test_server_answers_ping_load_and_infer(link):
    call, _, _ = link
    assert call(request(PING, 1)).kind == PONG
    loaded = call(request(LOAD, 2, {"model": "/ok"}))
    assert (loaded.kind, loaded.header["n_obs"]) == (LOADED, 1)
    reply = call(infer(3, "a", b"j" * 3000))
    assert (reply.kind, reply.request_id) == (ACTIONS, 3)
    assert reply.header["actions"] == [[3000]] and "infer_ms" in reply.header
    echoed = {k: reply.header[k] for k in wire.ECHO_FIELDS}
    assert echoed == {k: infer(3, "a").header[k] for k in wire.ECHO_FIELDS}
    assert reply.header["model_version"] == "/models/m"
    assert reply.header["valid_until"] == "2026-11-09T12:00:01.000Z"


def test_server_reports_refusals_and_failures(link):
    call, _, _ = link
    refused = call(request(LOAD, 1, {"model": "/bad"}))
    assert (refused.kind, refused.header["code"]) == (ERROR, "MODEL_NOT_ALLOWED")
    for request_id, change in enumerate(
        ({"schema_version": "old"}, {"observation_seq": 99}, {"obs": None}), start=2
    ):
        bad = call(infer(request_id, "x", **change))
        assert bad.header["code"] == "BAD_REQUEST", change
    failed = call(infer(5, "boom"))
    assert failed.header["code"] == "INFER_FAILED"


def test_reply_that_cannot_be_built_or_sent_never_stops_the_server(link):
    call, server, _ = link
    nan = call(infer(1, "nan"))
    assert (nan.kind, nan.header["code"]) == (ERROR, "INFER_FAILED")
    huge = call(infer(2, "huge"))
    assert (huge.kind, huge.header["code"]) == (ERROR, "REPLY_FAILED")
    assert server.stats["reply_failed"] == 1
    # both went out as ERROR: the NaN refusal and the reply that could not be packed
    assert (server.stats["replied_error"], server.stats["replied_actions"]) == (2, 0)
    real = server.sock

    class Refusing:  # a socket whose sends fail (ENOBUFS, firewall EPERM, ...)
        def __getattr__(self, name):
            return getattr(real, name)

        def sendto(self, *_):
            raise PermissionError("firewall")

    server.sock = Refusing()
    call(infer(3, "a"), serve=False)
    assert server.serve_once(2) == 0 and server.stats["send_failed"] == 1
    server.sock = real
    assert call(infer(4, "b")).kind == ACTIONS


def test_backlog_is_skipped_for_the_newest_observation(link):
    call, server, policy = link
    call(infer(1, "old"), serve=False)
    reply = call(infer(2, "new"))
    assert reply.request_id == 2
    assert policy.seen == ["new"]
    assert server.stats["skipped_backlog"] == 1


def test_key_file_and_bind_checks(tmp_path):
    key_file = tmp_path / "omx-udp.key"
    key_file.write_text(KEY.hex())
    key_file.chmod(0o644)
    with pytest.raises(ValueError):
        read_key(key_file)
    key_file.chmod(0o600)
    assert read_key(key_file) == KEY
    with pytest.raises(ValueError):
        bind_address("0.0.0.0")
    assert bind_address("192.0.2.10") == "192.0.2.10"


@pytest.mark.filterwarnings("ignore:'omx_udp.server' found in sys.modules")
def test_policy_errors_are_one_class_when_run_as_main():
    # `python -m omx_udp.server` runs server.py a second time beside the imported omx_udp.server
    as_main = runpy.run_module("omx_udp.server", run_name="omx_udp_as_main")
    assert as_main["PolicyError"] is errors.PolicyError


FAKE_INFER = """
from pathlib import Path

ALLOWED = Path(__file__).parent / "checkpoints"


def allowed_model(model):
    return Path(model).is_relative_to(ALLOWED)


class ModelCache:
    items = []

    def get(self, model, overrides):
        raise AssertionError("info_only must not load a model on the GPU")
"""

FAKE_HOME = """
def model_home(model):
    if model.endswith("no_data"):
        raise FileNotFoundError("training data is gone")
    return {"left": [0.0] * 6}
"""


@pytest.fixture
def panel(tmp_path, monkeypatch):
    (tmp_path / "omx_infer.py").write_text(FAKE_INFER)
    (tmp_path / "omx_home.py").write_text(FAKE_HOME)
    for name in ("ok", "no_data"):
        model = tmp_path / "checkpoints" / name
        model.mkdir(parents=True)
        (model / "config.json").write_text('{"type": "act"}')
    monkeypatch.setattr(sys, "path", list(sys.path))
    yield PanelPolicy(tmp_path), tmp_path / "checkpoints"
    for name in ("omx_infer", "omx_home"):
        sys.modules.pop(name, None)


def test_load_info_only_sends_files_and_home_without_loading(panel):
    policy, checkpoints = panel
    info = policy.load(str(checkpoints / "ok"), {"_info_only": True})
    assert info["info_only"] and info["ok"]
    assert info["files"] == {"config.json": '{"type": "act"}'}  # no train_config.json
    assert info["home"] == {"left": [0.0] * 6}
    gone = policy.load(str(checkpoints / "no_data"), {"_info_only": True})
    assert gone["home"] is None and "FileNotFoundError" in gone["home_error"]
    with pytest.raises(errors.PolicyError) as refused:
        policy.load("/elsewhere/model", {"_info_only": True})
    assert refused.value.code == "MODEL_NOT_ALLOWED"


def test_host_copy_of_wire_matches():
    # roport-omx keeps a hand copy for the host PC; the two must change together
    ours = Path(wire.__file__).read_text()
    host = (
        Path(__file__).resolve().parents[2]
        / "roport-omx/src/roport_omx/infer_link/wire.py"
    )
    if not host.is_file():
        pytest.skip("roport-omx is not next to this service")
    assert host.read_text().split("\n", 1)[1] == ours


def test_stats_line_counts_requests_replies_and_infer_time(link, capsys):
    call, server, _ = link
    server.stats_every = 3600  # on; this test calls stats_line itself
    assert call(request(PING, 1)).kind == PONG
    for request_id in (2, 3, 4):
        assert call(infer(request_id, "a", b"j" * 3000)).kind == ACTIONS
    line = server.stats_line()
    assert (line["completed_ping"], line["completed_infer"]) == (1, 3)
    assert (line["replied_pong"], line["replied_actions"]) == (1, 3)
    assert line["fragments_expected"] == line["fragments_received"]
    assert line["infer_by_peer"] == {"127.0.0.1": 3}
    assert line["infer_ms_n"] == 3
    assert line["infer_ms_p50"] <= line["infer_ms_p95"] <= line["infer_ms_max"]
    assert server.stats_line() is None  # nothing new since the last line

    server.stats_every = 0.01
    call(infer(5, "boom"))
    time.sleep(0.02)
    server._log_stats()
    out = capsys.readouterr().out.strip().splitlines()[-1]
    assert out.startswith("[omx-udp] stats {")
    logged = json.loads(out.removeprefix("[omx-udp] stats "))
    assert logged["failed"] == logged["replied_error"] == 1
    assert logged["infer_ms_n"] == 0  # a failed inference has no time


def test_no_infer_time_samples_while_stats_are_off(link):
    call, server, _ = link
    assert server.stats_every == 0  # --stats-every 0
    for request_id in range(1, 21):
        assert call(infer(request_id, "a")).kind == ACTIONS
    assert len(server._infer_ms) == 0
    assert server.stats["replied_actions"] == 20  # counters still count


def test_infer_time_samples_are_bounded(link):
    _, server, _ = link
    server._infer_ms.extend([1.0] * (server_module.INFER_MS_KEPT + 100))
    assert len(server._infer_ms) == server_module.INFER_MS_KEPT
