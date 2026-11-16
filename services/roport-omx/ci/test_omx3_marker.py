"""OMX 3 마커 원 표시 검사: 학습 그림 값(AST)과 보임·가림·reset 동작(가짜 cv2). cv2 없이 돈다."""

import ast
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ADAPTERS = ROOT / "src/roport_omx/omx3/adapters"
sys.path.insert(0, str(ROOT / "src"))


class FakeFrame:
    """영상 대신 '이 프레임에 보이는 마커 {번호: 중심}'과 그려진 원 목록만 가진다."""

    ndim = 3

    def __init__(self, markers, circles=()):
        self.markers, self.circles = dict(markers), list(circles)

    def copy(self):
        return FakeFrame(self.markers, self.circles)


DETECTS = []  # detectMarkers 호출 횟수


class FakeDetector:
    def __init__(self, dictionary):
        pass

    def detectMarkers(self, gray):  # noqa: N802 - cv2 API 이름
        DETECTS.append(1)
        if not gray.markers:
            return [], None, []
        ids = list(gray.markers)
        # corners[k][0].mean(0) → 중심
        corners = [
            [types.SimpleNamespace(mean=lambda axis, p=gray.markers[i]: p)] for i in ids
        ]
        return corners, types.SimpleNamespace(flatten=lambda: ids), []


def _circle(out, center, radius, color, thickness, line_type):
    out.circles.append((center, radius, color, thickness))


FAKE_CV2 = types.SimpleNamespace(
    COLOR_RGB2GRAY=7,
    LINE_AA=16,
    cvtColor=lambda img, code: img,
    circle=_circle,
    aruco=types.SimpleNamespace(
        DICT_4X4_50=0, getPredefinedDictionary=lambda d: d, ArucoDetector=FakeDetector
    ),
)
FAKE_NP = types.SimpleNamespace(ascontiguousarray=lambda img: img, ndarray=object)

_saved = {k: sys.modules.get(k) for k in ("cv2", "numpy")}
sys.modules.update(cv2=FAKE_CV2, numpy=FAKE_NP)
from roport_omx.omx3.adapters.marker import ArucoTargetMarker  # noqa: E402

for _k, _v in _saved.items():
    if _v is None:
        sys.modules.pop(_k)
    else:
        sys.modules[_k] = _v


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
            n.attr
            for n in ast.walk(tree("target_mark.py"))
            if isinstance(n, ast.Attribute)
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


class MarkerBehaviour(unittest.TestCase):
    """보임 → 가림 → reset 순서를 고정한다(#89 리뷰). runner 완료 판정은 이번 프레임 검출점을 쓴다."""

    CIRCLE = ((300, 201), 48, (255, 0, 255), 6)

    def setUp(self):
        DETECTS.clear()
        self.m = ArucoTargetMarker(17)

    def test_visible_then_hidden_keeps_circle_but_reports_none(self):
        visible = FakeFrame({17: (300.4, 200.6), 18: (10.0, 10.0)})
        out, point = self.m.mark(visible)
        self.assertEqual(point, (300.4, 200.6))  # 다른 번호(18)는 무시
        self.assertEqual(out.circles, [self.CIRCLE])
        self.assertEqual(visible.circles, [])  # 입력 프레임은 그대로, 사본에 그린다
        self.assertEqual(len(DETECTS), 1)  # mark 한 번에 검출 한 번

        hidden = FakeFrame({18: (10.0, 10.0)})  # 그리퍼가 17을 가림
        out, point = self.m.mark(hidden)
        self.assertIsNone(point)
        self.assertEqual(out.circles, [self.CIRCLE])  # 원은 마지막 위치에 유지

    def test_find_does_not_move_last_position(self):
        self.m.mark(FakeFrame({17: (300.4, 200.6)}))
        self.assertEqual(self.m.find(FakeFrame({17: (50.0, 60.0)})), (50.0, 60.0))
        out, _ = self.m.mark(FakeFrame({}))
        self.assertEqual(out.circles, [self.CIRCLE])

    def test_reset_clears_circle(self):
        self.m.mark(FakeFrame({17: (300.4, 200.6)}))
        self.m.reset()
        out, point = self.m.mark(FakeFrame({}))
        self.assertEqual((out.circles, point), ([], None))


if __name__ == "__main__":
    unittest.main()
