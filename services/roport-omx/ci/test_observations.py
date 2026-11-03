"""Source/fake and explicitly local UDP checks; no cameras or robot adapters."""

import socket
import sys
import unittest
from pathlib import Path
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx.observations import ObservationImageProducer, validate_images
from roport_omx.udp_frame_protocol import FrameInfo, parse_datagram
from roport_omx.udp_frame_sender import FrameSender, fragment_frame


class FakeSender:
    def __init__(self):
        self.sent = []

    def send(self, frame, info):
        self.sent.append((frame, info))


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.sender = FakeSender()
        self.producer = ObservationImageProducer(
            {"side": self.sender}, source="omx1", stream="policy", session=uuid4()
        )

    def send(self, frames=None):
        return self.producer.send(
            frames if frames is not None else {"side": b"encoded-jpeg"},
            frame_id=3,
            captured_ns=1000,
            observation_group="group3",
        )

    def test_exact_header_and_deepcopy(self):
        manifest = self.send()
        self.assertEqual(manifest["side"]["camera"], "side")
        copied = validate_images(manifest)
        copied["side"]["frame_id"] = 4
        self.assertEqual(manifest["side"]["frame_id"], 3)

    def test_missing_extra_mismatched_headers_rejected(self):
        manifest = self.send()
        for header in (
            {**manifest["side"], "raw": b"bad"},
            {**manifest["side"], "camera": "other"},
            {**manifest["side"], "frame_id": True},
            {**manifest["side"], "session": "not-a-uuid"},
        ):
            with self.subTest(header=header), self.assertRaises(ValueError):
                validate_images({"side": header})

    def test_current_complete_set_required_before_any_send(self):
        for frames in ({}, {"side": b"", "extra": b"x"}, {"side": b""}):
            with self.assertRaises(ValueError):
                self.send(frames)
        self.assertFalse(self.sender.sent)

    def test_multi_camera_group_cannot_cross_session_or_group(self):
        manifest = self.send()
        other = {**manifest["side"], "camera": "top", "observation_group": "old"}
        with self.assertRaises(ValueError):
            validate_images({**manifest, "top": other})

    def test_protocol_preserves_headers_and_boundaries(self):
        info = FrameInfo("omx1", "side", "policy", uuid4(), 3, 1000, "g3")
        packets = list(fragment_frame(b"x" * 3000, info))
        self.assertGreater(len(packets), 1)
        decoded = [parse_datagram(packet) for packet in packets]
        self.assertTrue(all(item[0] == info for item in decoded))
        self.assertEqual(b"".join(item[-1] for item in decoded), b"x" * 3000)
        self.assertTrue(all(len(packet) <= 1200 for packet in packets))
        with self.assertRaises(ValueError):
            next(fragment_frame(b"x" * (1024 * 1024 + 1), info))

    def test_loopback_datagrams(self):
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        ):
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            info = FrameInfo("omx1", "side", "policy", uuid4(), 3, 1000, "g3")
            transport = FrameSender(
                sender, receiver.getsockname(), fps=1000, bitrate=100_000_000
            )
            frame = b"image" * 300
            transport.send(frame, info)
            packets = []
            for _ in fragment_frame(frame, info):
                packet, _ = receiver.recvfrom(1200)
                packets.append(parse_datagram(packet))
            self.assertEqual(b"".join(packet[-1] for packet in packets), frame)
            self.assertTrue(all(packet[0] == info for packet in packets))


if __name__ == "__main__":
    unittest.main()
