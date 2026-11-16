"""OMX 3 마커 원 표시 검사: 학습 때 그림 값이 그대로인지 AST로 확인(cv2 없이)."""

import ast
import unittest
from pathlib import Path

ADAPTERS = Path(__file__).resolve().parents[1] / "src/roport_omx/omx3/adapters"


def tree(name):
    return ast.parse((ADAPTERS / name).read_text())


class TargetMarkValues(unittest.TestCase):
    def test_constants_match_training(self):
        consts = {
            t.id: ast.literal_eval(n.value)
            for n in tree("target_mark.py").body
            if isinstance(n, ast.Assign)
            for t in n.targets
            if isinstance(t, ast.Name)
        }
        self.assertEqual(consts["COLOR"], (255, 0, 255))
        self.assertEqual(consts["RADIUS"], 48)
        self.assertEqual(consts["THICKNESS"], 6)

    def test_dictionary_and_line_type(self):
        attrs = {
            n.attr for n in ast.walk(tree("target_mark.py")) if isinstance(n, ast.Attribute)
        }
        self.assertIn("DICT_4X4_50", attrs)
        self.assertIn("LINE_AA", attrs)
        self.assertIn("COLOR_RGB2GRAY", attrs)  # 카메라 영상은 RGB

    def test_core_does_not_import_adapters_libs(self):
        # 코어 omx3/는 표준 라이브러리만: cv2·numpy는 adapters/에서만
        for path in ADAPTERS.parent.glob("*.py"):
            names = {
                a.name.split(".")[0]
                for n in ast.walk(ast.parse(path.read_text()))
                if isinstance(n, ast.Import)
                for a in n.names
            } | {
                (n.module or "").split(".")[0]
                for n in ast.walk(ast.parse(path.read_text()))
                if isinstance(n, ast.ImportFrom) and n.level == 0
            }
            with self.subTest(path.name):
                self.assertFalse(names & {"cv2", "numpy"})


if __name__ == "__main__":
    unittest.main()
