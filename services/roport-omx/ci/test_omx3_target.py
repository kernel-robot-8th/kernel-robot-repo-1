"""OMX 3 대상 ID 해석 검사 (표준 라이브러리만)."""

import ast
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from roport_omx.omx3.settings import Omx3Settings  # noqa: E402
from roport_omx.omx3.target import InvalidTarget, resolve_marker_id  # noqa: E402

S = Omx3Settings()
MARKERS = (ROOT.parents[1] / "docs/aruco/markers.yaml").read_text()


class ResolveMarkerId(unittest.TestCase):
    def test_package_markers_as_strings(self):
        for value in ("15", "17", "22"):
            with self.subTest(value=value):
                self.assertEqual(resolve_marker_id({"package_id": value}), int(value))

    def test_rejects_non_package_markers(self):
        # 경계 14·23, P4 바닥 4, 예비 0·11~14·34~49, 다른 그룹 1~10·23~33
        for marker in (0, 1, 4, 10, 11, 12, 14, 23, 26, 27, 28, 33, 34, 40, 49, 50):
            with self.subTest(marker=marker), self.assertRaises(InvalidTarget):
                resolve_marker_id({"package_id": str(marker)})

    def test_rejects_bad_format(self):
        for value in (17, True, None, "", " 17", "17 ", "017", "１７", "+17", "17.0",
                      "PKG3", "package-test"):
            with self.subTest(value=value), self.assertRaises(InvalidTarget):
                resolve_marker_id({"package_id": value})

    def test_rejects_bad_payload(self):
        for payload in (None, [17], "17", {}, {"marker_id": "17"}):
            with self.subTest(payload=payload), self.assertRaises(InvalidTarget):
                resolve_marker_id(payload)

    def test_field_is_configurable(self):
        self.assertEqual(resolve_marker_id({"box": "21"}, field="box"), 21)

    def test_package_ids_match_aruco_table_both_ways(self):
        rows = re.findall(r"\{id: (\d+), name: (\w+), group: (\w+)", MARKERS)
        self.assertTrue(rows)
        package = {int(i) for i, _, group in rows if group == "package"}
        self.assertEqual(set(S.package_ids), package)

    def test_settings_are_immutable(self):
        self.assertIsInstance(S.package_ids, frozenset)


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
