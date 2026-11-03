"""Host image binding checks using only synthetic measured observations."""

import sys
import tempfile
import unittest
from pathlib import Path
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
from dedicated_demo import MockClock, mock_envelopes, mock_observe, synthetic_profile
from roport_omx.dedicated import DedicatedHost, Profile, Rejected


class DedicatedImageTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.clock = MockClock()
        self.host = DedicatedHost(
            Profile(synthetic_profile()),
            Path(temporary.name) / "host.sqlite",
            host_boot_id="mock-host-boot",
            wall_clock=self.clock.wall,
            monotonic_clock=self.clock.monotonic,
        )
        self.addCleanup(self.host.close)
        mock_observe(self.host, self.clock, 1)
        command, permit = mock_envelopes(self.clock)
        self.host.update_permit(permit)
        self.host.accept_command(command)
        self.images = {
            "side": {
                "source": "omx1",
                "camera": "side",
                "stream": "policy",
                "session": str(uuid4()),
                "frame_id": 2,
                "captured_ns": 100,
                "observation_group": "group2",
            }
        }

    def request(self):
        reset = self.host.reset_request()
        self.host.acknowledge_reset({**reset, "type": "policy.reset.ack"})
        return self.host.infer_request()

    def test_legacy_joint_only_observation(self):
        self.assertNotIn("images", self.request()["observation"])

    def test_manifest_is_bound_and_copied(self):
        self.clock.advance()
        self.host.observe(
            "omx1",
            [0, 0, 0, 0, 0, 50],
            observation_seq=2,
            captured_at=self.clock.stamp(),
            images=self.images,
        )
        self.images["side"]["frame_id"] = 999
        request = self.request()
        self.assertEqual(request["observation"]["images"]["side"]["frame_id"], 2)
        self.assertEqual(request["captured_at"], request["observation"]["captured_at"])
        request["observation"]["images"]["side"]["frame_id"] = 888
        self.assertEqual(
            self.host.pending["request"]["observation"]["images"]["side"]["frame_id"], 2
        )

    def test_new_joint_only_frame_cannot_reuse_prior_images(self):
        self.clock.advance()
        self.host.observe(
            "omx1",
            [0, 0, 0, 0, 0, 50],
            observation_seq=2,
            captured_at=self.clock.stamp(),
            images=self.images,
        )
        self.clock.advance()
        self.host.observe(
            "omx1",
            [0, 0, 0, 0, 0, 50],
            observation_seq=3,
            captured_at=self.clock.stamp(),
        )
        self.assertNotIn("images", self.request()["observation"])

    def test_invalid_manifest_holds_and_clears_pending(self):
        self.images["side"]["camera"] = "wrong"
        with self.assertRaises(Rejected):
            self.host.observe(
                "omx1",
                [0, 0, 0, 0, 0, 50],
                observation_seq=2,
                captured_at=self.clock.stamp(),
                images=self.images,
            )
        self.assertEqual(self.host.state, "HOLD")

    def test_large_manifest_cannot_exceed_inference_json_datagram(self):
        template = self.images["side"]
        images = {
            f"cam{index}": {**template, "camera": f"cam{index}"} for index in range(400)
        }
        self.clock.advance()
        self.host.observe(
            "omx1",
            [0, 0, 0, 0, 0, 50],
            observation_seq=2,
            captured_at=self.clock.stamp(),
            images=images,
        )
        with self.assertRaisesRegex(Rejected, "datagram limit"):
            self.request()
        self.assertEqual(self.host.state, "HOLD")


if __name__ == "__main__":
    unittest.main()
