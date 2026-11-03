"""Cross-module source/mock contract check; requires the integration branch."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from dedicated_demo import MockClock, synthetic_profile
from dedicated_integration import exchange, run_cycle


class IntegrationTests(unittest.TestCase):
    def test_inbound_cart_leaves_before_outbound_begins(self):
        result = run_cycle()
        self.assertTrue(result["source_mock_only"])
        self.assertFalse(result["physical_motion"])
        self.assertEqual(result["state"], "DELIVERY_READY")
        self.assertLess(
            result["trace"].index("INBOUND:P7-clear"),
            result["trace"].index("OUTBOUND:accepted"),
        )

    def test_unconfigured_ai_never_returns_a_candidate(self):
        import tempfile

        from dedicated_demo import mock_envelopes, mock_observe
        from dedicated_policy import PolicyRouter

        from roport_omx.dedicated import DedicatedHost, Profile

        clock = MockClock()
        with tempfile.TemporaryDirectory() as directory:
            host = DedicatedHost(
                Profile.from_dict(synthetic_profile()),
                Path(directory) / "journal.sqlite",
                host_boot_id="mock-host-boot",
                wall_clock=clock.wall,
                monotonic_clock=clock.monotonic,
            )
            try:
                cmd, permit = mock_envelopes(clock)
                mock_observe(host, clock, 1)
                host.update_permit(permit)
                host.accept_command(cmd)
                response = exchange(PolicyRouter({}, {}), host.reset_request(), clock)
                self.assertEqual(response["type"], "inference.error")
                with self.assertRaises(ValueError):
                    host.acknowledge_reset(response)
                self.assertEqual(host.state, "HOLD")
            finally:
                host.close()


if __name__ == "__main__":
    unittest.main()
