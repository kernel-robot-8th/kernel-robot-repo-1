"""Exercise the standalone WebEngine view without robot or network inputs."""

import argparse
import json
import math
import time
from pathlib import Path

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QLabel

from roport_control_server.spatial_ui import ASSETS, SpatialWindow
from roport_control_server.ui import demo_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    app = QApplication(["roport-display-check"])
    window = SpatialWindow(demo=not args.offline, map_path=ASSETS / "maps/demo.yaml")
    window.set_detail_widget("Pinky", QLabel("team widget"))
    window.show_tab(1)
    assert window.tabs.widget(0).text() == "team widget"
    assert window.stack.currentIndex() == 1 and window.tabs.currentIndex() == 0
    window.show_tab(2)
    assert window.tabs.currentIndex() == 1
    window.show_tab(0)
    if not args.offline:
        assert not window.apply_snapshot(demo_snapshot(1))
    window.show()
    result = {"ok": False, "mode": "offline" if args.offline else "demo"}
    phase, previous_pose = 0, None
    started = time.monotonic()
    poll = QTimer()

    def finish(error=None):
        poll.stop()
        result["ok"] = error is None
        if error:
            result["error"] = str(error)
        if args.output:
            args.output.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(args.output / "spatial-ui.png"))
            (args.output / "result.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n"
            )
        print(json.dumps(result, ensure_ascii=False), flush=True)
        window.close()
        assert not window.timer.isActive()
        app.quit()

    def inspect(raw):
        nonlocal phase, previous_pose
        try:
            data = json.loads(raw)
            result.update(data)
            if data.get("ready") != "true":
                return
            assert data["robots"] == 6 and data["canvases"] == 2
            if args.offline:
                if data.get("fps") is None:
                    return
                assert data["visible_robots"] == 0
                assert data["demo_hidden"] and data["mode"].startswith("OFFLINE")
                assert all(row["stale"] for row in window.cache.snapshot())
                assert data["tasks"] == 0
                window.bridge.seek(6)
                window.bridge.drop("pinky1", True)
                assert window.demo_time == 0 and not window.dropped
                finish()
            elif phase == 0:
                assert data["visible_robots"] == 6 and not data["demo_hidden"]
                previous_pose = window.cache.samples["pinky1"].pose
                phase = 1
            elif phase == 1:
                assert window.cache.samples["pinky1"].pose != previous_pose
                window.page.runJavaScript(
                    "document.querySelectorAll('.robot')[3].click();"
                    "document.getElementById('focus').click();"
                    "document.querySelector('[data-view=top]').click();"
                    "document.getElementById('play').click();"
                    "document.getElementById('time').value=6;"
                    "document.getElementById('time').dispatchEvent(new Event('input'))"
                )
                phase = 2
            elif phase == 2:
                assert data["selected"] == "OMX 01" and data["top"] == "true"
                assert not window.playing and window.demo_time == 6
                assert math.isclose(
                    dict(window.cache.samples["omx1"].joints)["joint1"],
                    0.35 * math.sin(4),
                )
                window.page.runJavaScript("document.getElementById('drop').click()")
                phase = 3
            elif phase == 3 and window.cache.snapshot()[3]["stale"]:
                # Both cache and rendered status must expose stale data before recovery.
                if "갱신 지연" not in data["state"]:
                    return
                result["stale_display_verified"] = True
                window.page.runJavaScript(
                    "document.getElementById('drop').click();"
                    "document.getElementById('left').click();"
                    "document.getElementById('right').click();"
                    "document.getElementById('home').click();"
                    "document.querySelectorAll('.robot')[0].click()"
                )
                phase = 4
            elif phase == 4 and not window.cache.snapshot()[3]["stale"]:
                assert data["selected"] == "Pinky 01" and data["top"] == "false"
                assert data["fps"] is not None and float(data["fps"]) > 0
                result["recovery_verified"] = True
                finish()
        except (AssertionError, ValueError, TypeError, KeyError) as exc:
            finish(f"phase {phase}: {type(exc).__name__}: {exc}")

    def check():
        if time.monotonic() - started > 25:
            finish(f"UI probe timed out in phase {phase}")
            return
        window.page.runJavaScript(
            "document.getElementById('mode') ? JSON.stringify({ready:document.body.dataset.ready,"
            "robots:document.querySelectorAll('.robot').length,"
            "canvases:document.querySelectorAll('canvas').length,"
            "visible_robots:document.querySelectorAll('.label:not([hidden])').length,"
            "demo_hidden:document.getElementById('demo-controls').hidden,"
            "mode:document.getElementById('mode').textContent,"
            "tasks:document.querySelectorAll('.task').length,"
            "selected:document.getElementById('name').textContent,"
            "state:document.getElementById('state').textContent,"
            "top:document.querySelector('[data-view=top]').getAttribute('aria-pressed'),"
            "fps:document.getElementById('fps').dataset.value}) : '{}'",
            inspect,
        )

    poll.timeout.connect(check)
    poll.start(250)
    app.exec()
    assert result["ok"], result


if __name__ == "__main__":
    main()
