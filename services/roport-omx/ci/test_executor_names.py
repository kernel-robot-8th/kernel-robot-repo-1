"""The two-arm shelf executor is CooperativeShelfExecutor; the old name still imports."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from roport_omx import field_executors


class ExecutorNameTests(unittest.TestCase):
    def test_new_name_is_the_class(self):
        self.assertEqual(
            field_executors.CooperativeShelfExecutor.__name__,
            "CooperativeShelfExecutor",
        )

    def test_old_name_is_a_deprecated_alias_of_the_same_class(self):
        self.assertIs(
            field_executors.PanelFieldExecutor,
            field_executors.CooperativeShelfExecutor,
        )


if __name__ == "__main__":
    unittest.main()
