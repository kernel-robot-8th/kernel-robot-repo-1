"""UDP inference client against a fake GPU server on loopback; no GPU, no panel, no robot."""

import socket
import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.infer_link import wire
from roport_omx.infer_link.client import (
    InferenceClient,
    InferenceError,
    InferenceTimeout,
)
from roport_omx.infer_link.wire import ACTIONS, ERROR, INFER, LOADED, PONG, Message

KEY = b"k" * 32


class FakeServer:
    """Answers each complete request according to self.mode."""

    def __init__(self, key=KEY):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.address = self.sock.getsockname()
        self.key, self.mode, self.seen, self.requests = key, "answer", [], []
        self.reassembler = wire.Reassembler(KEY)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join()
        self.sock.close()

    def _send(self, kind, request, header):
        reply = Message(kind, request.session, request.request_id, header)
        for datagram in wire.pack(reply, self.key):
            self.sock.sendto(datagram, self.peer)

    def _run(self):
        while not self.stop.is_set():
            try:
                packet, self.peer = self.sock.recvfrom(65535)
            except TimeoutError:
                continue
            request = self.reassembler.feed(packet, self.peer)
            if request is None:
                continue
            self.seen.append((request.kind, request.request_id, len(request.data)))
            if self.mode == "silent":
                continue
            if self.mode == "error":
                self._send(ERROR, request, {"code": "MODEL_NOT_ALLOWED"})
                continue
            if self.mode == "stale_first":
                old = Message(INFER, request.session, request.request_id - 1, {})
                self._send(ACTIONS, old, {"actions": [["old"]]})
            kind = {wire.PING: PONG, wire.LOAD: LOADED, INFER: ACTIONS}[request.kind]
            reply = {"actions": [[len(request.data)]], "ok": True}
            if kind == ACTIONS:
                self.requests.append(request.header)
                reply |= {k: request.header.get(k) for k in wire.ECHO_FIELDS}
                reply["model_version"] = request.header.get("model")
                if self.mode == "wrong_model":
                    reply["model_version"] = "/models/other"
                if self.mode == "wrong_attempt":
                    reply["execution_id"] = "exec-previous"
            self._send(kind, request, reply)


class InferenceClientTest(unittest.TestCase):
    def setUp(self):
        self.server = FakeServer()
        self.client = InferenceClient(self.server.address, KEY, bind_host="127.0.0.1")

    def tearDown(self):
        self.client.close()
        self.server.close()

    def test_ping_load_and_large_infer(self):
        self.assertTrue(self.client.ping()["ok"])
        self.assertTrue(self.client.load("/models/m")["ok"])
        data = b"j" * 100_000  # four camera JPEGs, roughly
        reply = self.client.infer({"model": "/models/m", "obs": []}, data)
        self.assertEqual(reply["actions"], [[100_000]])
        self.assertGreater(self.client.last["bytes"], 100_000)
        ids = [request_id for _, request_id, _ in self.server.seen]
        self.assertEqual(ids, [1, 2, 3])

    def test_late_reply_to_a_timed_out_request_is_ignored(self):
        self.server.mode = "silent"
        with self.assertRaises(InferenceTimeout):
            self.client.infer({"obs": []}, b"old", timeout=0.1)
        self.server.mode = "stale_first"  # #1's reply arrives just before #2's
        reply = self.client.infer({"obs": []}, b"x")
        self.assertEqual(reply["actions"], [[1]])
        self.assertEqual(self.client.stats["stale_reply"], 1)

    def test_timeout_then_newest_observation_uses_a_new_id(self):
        self.server.mode = "silent"
        with self.assertRaises(InferenceTimeout):
            self.client.infer({"obs": []}, b"old", timeout=0.1)
        self.server.mode = "answer"
        self.client.infer({"obs": []}, b"newest")
        self.assertEqual([r[1] for r in self.server.seen], [1, 2])

    def test_infer_carries_team_identifiers(self):
        context = {"task_id": "t1", "command_id": "c1", "execution_id": "e1"}
        self.client.infer({"model": "/models/m", "obs": []}, b"x", context=context)
        (sent,) = self.server.requests
        self.assertEqual(sent["schema_version"], wire.SCHEMA)
        self.assertEqual(sent["observation_seq"], 1)
        self.assertEqual(sent["boot_id"], str(self.client.session))
        self.assertEqual({k: sent[k] for k in context}, context)
        for field in ("inference_id", "captured_at", "deadline"):
            self.assertIsInstance(sent[field], str)

    def test_reply_for_another_model_or_attempt_is_refused(self):
        context = {"execution_id": "e1"}
        for mode, code in (
            ("wrong_model", "MODEL_MISMATCH"),
            ("wrong_attempt", "RESULT_MISMATCH"),
        ):
            with self.subTest(mode):
                self.server.mode = mode
                with self.assertRaises(InferenceError) as caught:
                    self.client.infer(
                        {"model": "/models/m", "obs": []}, b"x", context=context
                    )
                self.assertEqual(caught.exception.code, code)

    def test_server_error_is_raised_with_its_code(self):
        self.server.mode = "error"
        with self.assertRaises(InferenceError) as caught:
            self.client.load("/models/x", attempts=1)
        self.assertEqual(caught.exception.code, "MODEL_NOT_ALLOWED")

    def test_load_retries_with_new_ids(self):
        self.server.mode = "silent"
        with self.assertRaises(InferenceTimeout):
            self.client.load("/models/m", timeout=0.05, attempts=3)
        self.assertEqual([r[1] for r in self.server.seen], [1, 2, 3])

    def test_reply_signed_with_another_key_is_rejected(self):
        self.server.key = b"z" * 32
        with self.assertRaises(InferenceTimeout):
            self.client.ping(timeout=0.2)
        self.assertEqual(self.client.stats["bad_mac"], 1)


if __name__ == "__main__":
    unittest.main()
