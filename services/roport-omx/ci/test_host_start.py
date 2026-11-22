"""cooperative_host main() starts with each executor (2026-11-19: a route-key change stopped it).

No broker, no panel: the MQTT transport is replaced by one that stops the loop at once.
"""

import importlib.util
import json
import shutil
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

try:
    import roport_omx.mqtt_link.mqtt  # noqa: F401
except ImportError:  # system Python in CI has no Paho; main() is stopped before it
    stub = types.ModuleType("roport_omx.mqtt_link.mqtt")
    stub.MqttConfig = stub.MqttTransport = object
    sys.modules["roport_omx.mqtt_link.mqtt"] = stub

spec = importlib.util.spec_from_file_location(
    "cooperative_host_start", ROOT / "scripts/cooperative_host.py"
)
ch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ch)


class StopAtStart:
    """Stands in for MqttTransport: the Host has finished starting when it gets here."""

    def __init__(self, *_args, **_kwargs):
        pass

    def start(self):
        raise KeyboardInterrupt  # the same way Ctrl+C ends the Host

    def stop(self):
        pass


class HostStartTest(unittest.TestCase):
    def setUp(self):
        # the Host refuses a journal under /tmp (it must survive a restart)
        self.dir = Path(tempfile.mkdtemp(prefix="host-start-", dir=ROOT))
        self.addCleanup(shutil.rmtree, self.dir, ignore_errors=True)

    def run_main(self, *args):
        argv = ["cooperative_host.py", "--journal", str(self.dir / "j.json"), *args]
        with (
            patch.object(sys, "argv", argv),
            patch.object(ch, "MqttTransport", StopAtStart),
            patch.object(ch, "mqtt_config", lambda _args: None),
            self.assertLogs(level="INFO") as logs,
            self.assertRaises(SystemExit) as stop,
        ):
            ch.main()
        self.assertEqual(stop.exception.code, 0)
        return "\n".join(logs.output)

    def test_panel_executor_starts_with_its_routes(self):
        routes = self.dir / "routes.json"
        route = {
            "slot_id": "2b",
            "model_path": "/models/m/pretrained_model",
            "task": "t {box}",
            "episode_s": 40,
            "reset_s": 15,
        }
        routes.write_text(
            json.dumps(
                {
                    "routes": [
                        route | {"operation": "INBOUND", "markers": [16, 17, 20]},
                        route | {"operation": "OUTBOUND"},
                    ]
                }
            )
        )
        flags = ("--report-placement", "--place-shadow", "--box-marker", "16")
        output = self.run_main("--executor", "panel", "--routes", str(routes), *flags)
        self.assertIn(
            "routes=[INBOUND:2b:COOPERATIVE, OUTBOUND:2b:COOPERATIVE]", output
        )
        self.assertIn("boot_id=", output)

    def test_hand_over_on_and_no_time_limit_by_default(self):
        # TK 2026-11-22: loading hand-over (end_on_place) on by default, and a route without
        # episode_s gets the panel's longest episode (the run ends on its own)
        routes = self.dir / "routes.json"
        route = {
            "slot_id": "2b",
            "model_path": "/models/m/pretrained_model",
            "task": "t {box}",
            "reset_s": 15,
        }
        routes.write_text(
            json.dumps(
                {
                    "routes": [
                        route | {"operation": "INBOUND"},
                        route | {"operation": "OUTBOUND", "episode_s": 70},
                    ]
                }
            )
        )
        output = self.run_main("--executor", "panel", "--routes", str(routes))
        self.assertIn("hand-over (end_on_place)=on", output)
        self.assertIn("early COMPLETE on placement (report_placement)=on", output)
        self.assertIn("episode limit [INBOUND:2b=600s, OUTBOUND:2b=70s]", output)
        output = self.run_main(
            "--executor",
            "panel",
            "--routes",
            str(routes),
            "--no-end-on-place",
            "--no-report-placement",
        )
        self.assertIn("hand-over (end_on_place)=off", output)
        self.assertIn("(report_placement)=off", output)

    def test_mock_executor_starts(self):
        self.assertIn("SIMULATED executor", self.run_main("--executor", "mock"))


if __name__ == "__main__":
    unittest.main()
