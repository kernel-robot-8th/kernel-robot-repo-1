"""Lightweight source checks and hardware-probe failure paths, without a device."""

import ast
import importlib.util
import runpy
import tomllib
import unittest
from pathlib import Path
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "omx_probe", ROOT / "scripts/check_hardware.py"
)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class SourceChecks(unittest.TestCase):
    def test_python_syntax_and_cli_targets(self):
        trees = {}
        for path in (ROOT / "lerobot/src").rglob("*.py"):
            trees[path] = ast.parse(path.read_bytes(), filename=str(path))
        self.assertTrue(trees)
        config = tomllib.loads((ROOT / "lerobot/pyproject.toml").read_text())
        for target in config["project"]["scripts"].values():
            module, name = target.split(":")
            tree = trees[ROOT / "lerobot/src" / (module.replace(".", "/") + ".py")]
            self.assertIn(
                name, [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]
            )

    def test_signed_encoder_boundaries(self):
        # This standalone upstream module has no heavyweight imports.
        encoding = runpy.run_path(
            str(ROOT / "lerobot/src/lerobot/motors/encoding_utils.py")
        )
        for value in (-(2**31), -1, 0, 2**31 - 1):
            raw = encoding["encode_twos_complement"](value, 4)
            self.assertEqual(encoding["decode_twos_complement"](raw, 4), value)
        with self.assertRaises(ValueError):
            encoding["encode_twos_complement"](2**31, 4)

    def test_probe_motor_models_match_upstream(self):
        tables = ast.parse(
            (ROOT / "lerobot/src/lerobot/motors/dynamixel/tables.py").read_text()
        )
        models = next(
            ast.literal_eval(node.value)
            for node in tables.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(t, ast.Name) and t.id == "MODEL_NUMBER_TABLE"
                for t in node.targets
            )
        )
        follower = ast.parse(
            (
                ROOT / "lerobot/src/lerobot/robots/omx_follower/omx_follower.py"
            ).read_text()
        )
        motors = [
            node
            for node in ast.walk(follower)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "Motor"
        ]
        self.assertEqual(
            probe.MOTORS,
            {
                ast.literal_eval(m.args[0]): models[ast.literal_eval(m.args[1])]
                for m in motors
            },
        )


class ProbeChecks(unittest.TestCase):
    def sdk(self):
        sdk = Mock(COMM_SUCCESS=0)
        sdk.PortHandler.return_value.is_open = True
        sdk.PortHandler.return_value.openPort.return_value = True
        sdk.PortHandler.return_value.setBaudRate.return_value = True
        packet = sdk.PacketHandler.return_value
        packet.ping.side_effect = lambda port, mid: (probe.MOTORS[mid], 0, 0)
        packet.read4ByteTxRx.return_value = (2**32 - 1, 0, 0)
        packet.read1ByteTxRx.return_value = (0, 0, 0)
        return sdk

    def test_reads_all_motors_and_never_writes(self):
        sdk = self.sdk()
        rows = probe.probe(sdk, "dummy", 1_000_000)
        self.assertEqual(len(rows), 6)
        self.assertTrue(all(row["ok"] and row["position_counts"] == -1 for row in rows))
        calls = {call[0] for call in sdk.PacketHandler.return_value.method_calls}
        self.assertEqual(calls, {"ping", "read4ByteTxRx", "read1ByteTxRx"})
        sdk.PortHandler.return_value.closePort.assert_called_once()

    def test_wrong_model_is_not_read(self):
        sdk = self.sdk()
        sdk.PacketHandler.return_value.ping.side_effect = None
        sdk.PacketHandler.return_value.ping.return_value = (9999, 0, 0)
        self.assertFalse(any(row["ok"] for row in probe.probe(sdk, "dummy", 1_000_000)))
        sdk.PacketHandler.return_value.read4ByteTxRx.assert_not_called()

    def test_timeout_and_hardware_error_fail(self):
        for failure in ("timeout", "hardware_error"):
            with self.subTest(failure=failure):
                sdk = self.sdk()
                if failure == "timeout":
                    sdk.PacketHandler.return_value.ping.side_effect = None
                    sdk.PacketHandler.return_value.ping.return_value = (0, -1, 0)
                else:
                    sdk.PacketHandler.return_value.read1ByteTxRx.return_value = (
                        1,
                        0,
                        0,
                    )
                self.assertFalse(
                    any(row["ok"] for row in probe.probe(sdk, "dummy", 1_000_000))
                )
                sdk.PortHandler.return_value.closePort.assert_called_once()

    def test_preserves_open_failure(self):
        sdk = self.sdk()
        sdk.PortHandler.return_value.openPort.return_value = False
        sdk.PortHandler.return_value.is_open = False
        with self.assertRaisesRegex(RuntimeError, "Cannot open port"):
            probe.probe(sdk, "dummy", 1_000_000)
        sdk.PortHandler.return_value.closePort.assert_not_called()


if __name__ == "__main__":
    unittest.main()
