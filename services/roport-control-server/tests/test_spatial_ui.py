import os
import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.parametrize("mode", [[], ["--offline"]], ids=["demo", "offline"])
def test_standalone_webengine_controls_and_state(tmp_path, mode):
    if os.environ.get("REQUIRE_UI_TESTS") != "1":
        pytest.importorskip("PyQt6.QtWebEngineWidgets")
    if not os.environ.get("DISPLAY"):
        if os.environ.get("REQUIRE_UI_TESTS") == "1":
            pytest.fail("run UI integration tests under xvfb-run or a desktop DISPLAY")
        pytest.skip("desktop DISPLAY or xvfb-run required for WebGL integration")
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env["QT_QPA_PLATFORM"] = "xcb"
    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).parents[1] / "tools/check_spatial_ui.py"),
            "--output",
            str(tmp_path),
            *mode,
        ],
        env=env,
        check=False,
        capture_output=True,
        text=True,
        timeout=35,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "3D view:" not in result.stderr, result.stderr
