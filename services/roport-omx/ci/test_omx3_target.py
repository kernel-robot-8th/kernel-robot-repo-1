"""OMX 3 대상 ID 해석 검사 (표준 라이브러리만)."""

import ast
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.omx3.settings import Omx3Settings  # noqa: E402
from roport_omx.omx3.target import InvalidTarget, resolve_marker_id  # noqa: E402

S = Omx3Settings()


class ResolveMarkerId(unittest.TestCase):
    def test_number_string_and_name(self):
        self.assertEqual(resolve_marker_id({"marker_id": 17}), 17)
        self.assertEqual(resolve_marker_id({"package_id": " 21 "}), 21)
        self.assertEqual(
            resolve_marker_id({"package_id": "PKG3"}, S.id_fields, S.id_table), 17
        )

    def test_field_order_and_missing_value(self):
        self.assertEqual(resolve_marker_id({"marker_id": 16, "package_id": 22}), 16)
        self.assertEqual(resolve_marker_id({"marker_id": None, "package_id": 15}), 15)
        self.assertEqual(resolve_marker_id({"box": 18}, fields=("box",)), 18)

    def test_rejects_bad_values(self):
        for payload in (
            {},
            {"marker_id": True},
            {"marker_id": -1},
            {"marker_id": 50},
            {"marker_id": 1.5},
            {"package_id": "box"},
            {"package_id": "PKG9"},
        ):
            with self.subTest(payload=payload), self.assertRaises(InvalidTarget):
                resolve_marker_id(payload, S.id_fields, S.id_table)

    def test_default_table_matches_aruco_v14(self):
        text = (ROOT.parents[1] / "docs/aruco/markers.yaml").read_text()
        for name, marker in S.id_table.items():
            self.assertIn(f"{{id: {marker}, name: {name},", text)


class CoreImports(unittest.TestCase):
    def test_core_imports_only_stdlib(self):
        allowed = set(sys.stdlib_module_names)
        for path in (ROOT / "src/roport_omx/omx3").glob("*.py"):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom) and node.level == 0:
                    names = [node.module]
                elif isinstance(node, ast.Import):
                    names = [a.name for a in node.names]
                else:
                    continue
                for name in names:
                    self.assertIn(name.split(".")[0], allowed, f"{path.name}: {name}")


if __name__ == "__main__":
    unittest.main()
