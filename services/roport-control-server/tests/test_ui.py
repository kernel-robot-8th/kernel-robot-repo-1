import os
import subprocess
import sys
from dataclasses import replace
from threading import Thread

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
if os.environ.get("REQUIRE_UI_TESTS") == "1":
    import PyQt6  # noqa: F401
else:
    pytest.importorskip(
        "PyQt6", reason="install the ui extra to test the desktop shell"
    )

from PyQt6.QtWidgets import QApplication, QLabel

from roport_control_server import ui


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app):
    widget = ui.ControlWindow()
    yield widget
    widget.close()
    widget.deleteLater()
    app.processEvents()


def test_default_is_offline_with_two_tabs_and_no_live_controls(window):
    assert window.connection.text() == "미연결"
    assert window.tasks.rowCount() == 0
    assert [window.tabs.tabText(i) for i in range(2)] == ["Pinky", "OMX"]
    assert all(not button.isEnabled() for button in window.action_buttons)
    assert "OFFLINE" in window.banner.text()


def test_snapshot_replaces_full_view_and_keeps_error_text_plain(window):
    window.set_source("demo")
    snapshot = ui.demo_snapshot(1)
    assert window.apply_snapshot(snapshot)
    assert window.tasks.rowCount() == 3
    assert window.tasks.item(0, 0).text() == "DEMO-001"
    assert window.tasks.cellWidget(0, 3).value() == 10
    assert "모의 경고" in window.errors.toPlainText()
    assert window.apply_snapshot(
        ui.Snapshot("demo", 2, errors=("<b>literal error</b>",))
    )
    assert window.tasks.rowCount() == 0
    assert window.errors.toPlainText() == "<b>literal error</b>"
    assert all(not b.isEnabled() for b in window.action_buttons)


def test_duplicate_or_old_revision_does_not_refresh_stale_data(window, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(ui.time, "monotonic", lambda: clock[0])
    window.set_source("demo")
    window.apply_snapshot(ui.demo_snapshot(2))
    clock[0] += 6
    assert not window.apply_snapshot(ui.demo_snapshot(2))
    assert not window.apply_snapshot(ui.demo_snapshot(1))
    window.refresh_age()
    assert "갱신 지연" in window.connection.text()
    assert "6초 전" in window.updated.text()
    assert window.apply_snapshot(ui.demo_snapshot(3))
    assert window.connection.text() == "상태 수신"


def test_source_change_clears_previous_rows_and_rejects_late_old_session(window):
    window.set_source("demo")
    window.apply_snapshot(ui.demo_snapshot(3))
    window.set_source("new-session")
    assert window.tasks.rowCount() == 0
    assert not window.apply_snapshot(ui.demo_snapshot(100))
    assert window.apply_snapshot(ui.Snapshot("new-session", 0))
    window.set_source(None)
    assert window.connection.text() == "미연결"
    assert not window.apply_snapshot(ui.Snapshot("new-session", 1))


def test_teammate_widget_replaces_exact_tab_and_preserves_selection(window):
    window.tabs.setCurrentIndex(1)
    detail = QLabel("Pinky team widget")
    window.set_detail_widget("Pinky", detail)
    assert window.tabs.count() == 2
    assert window.tabs.widget(0) is detail
    assert window.tabs.currentIndex() == 1
    window.set_detail_widget("Pinky", detail)
    with pytest.raises(ValueError):
        window.set_detail_widget("OMX", detail)


def test_worker_thread_cannot_update_widgets(window):
    failures = []

    def update():
        try:
            window.set_source("bad-thread")
        except RuntimeError as exc:
            failures.append(str(exc))

    worker = Thread(target=update)
    worker.start()
    worker.join(timeout=2)
    assert failures == ["call UI methods on the Qt GUI thread"]
    assert window.connection.text() == "미연결"


@pytest.mark.parametrize("value", [-1, 101, 1.5, True])
def test_invalid_progress_is_rejected(value):
    with pytest.raises(ValueError):
        ui.TaskRow("id", "Pinky 1", "stage", value, "진행")


@pytest.mark.parametrize(
    "changes",
    [
        {"revision": -1},
        {"revision": True},
        {"session_id": ""},
        {"errors": ("x" * 501,)},
        {"errors": tuple("x" for _ in range(21))},
        {"tasks": (ui.TaskRow("same", "Pinky 1", "stage", 0, "대기"),) * 2},
    ],
)
def test_invalid_snapshot_is_rejected(changes):
    with pytest.raises(ValueError):
        replace(ui.demo_snapshot(1), **changes)


def test_demo_is_visibly_marked_and_does_not_claim_live_connection(app):
    window = ui.ControlWindow(demo=True)
    try:
        window.set_source("demo")
        window.apply_snapshot(ui.demo_snapshot(1))
        assert "DEMO" in window.windowTitle() and "모의 데이터" in window.banner.text()
        assert window.connection.text() == "모의 상태 수신"
    finally:
        window.close()
        window.deleteLater()


@pytest.mark.parametrize("args", [[], ["--demo"]])
def test_cli_event_loop_opens_and_closes(args):
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    script = """
import sys
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication
from roport_control_server.ui import main
original = QApplication.exec
def briefly(*args):
    QTimer.singleShot(100, QApplication.instance().quit)
    return original()
QApplication.exec = briefly
main()
"""
    subprocess.run(
        [sys.executable, "-c", script, *args],
        env=environment,
        capture_output=True,
        text=True,
        check=True,
        timeout=5,
    )
