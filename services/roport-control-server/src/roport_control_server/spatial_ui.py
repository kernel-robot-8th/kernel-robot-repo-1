"""Read-only 3D desktop view: python -m roport_control_server.spatial_ui --demo."""

import argparse
import json
import math
import sys
import time
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtSignal, pyqtSlot
from PyQt6.QtWebChannel import QWebChannel
from PyQt6.QtWebEngineCore import (
    QWebEnginePage,
    QWebEngineProfile,
    QWebEngineSettings,
    QWebEngineUrlRequestInterceptor,
)
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QStackedWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .spatial_map import load_map
from .spatial_state import ROBOT_IDS, RobotSample, StateCache
from .ui import ControlWindow, Snapshot, demo_snapshot

ASSETS = Path(__file__).with_name("spatial_assets")


class LocalAssets(QWebEngineUrlRequestInterceptor):
    def interceptRequest(self, info):
        url = info.requestUrl()
        allowed = (
            url.scheme() in ("data", "blob")
            or url.toString() == "qrc:///qtwebchannel/qwebchannel.js"
        )
        if url.isLocalFile():
            allowed = Path(url.toLocalFile()).resolve().is_relative_to(ASSETS.resolve())
        info.block(not allowed)


class LocalPage(QWebEnginePage):
    def acceptNavigationRequest(self, url, kind, is_main_frame):
        return (
            url.isLocalFile()
            and Path(url.toLocalFile()).resolve() == ASSETS / "index.html"
        )

    def javaScriptConsoleMessage(self, level, message, line, source):
        if level == self.JavaScriptConsoleMessageLevel.ErrorMessageLevel:
            print(f"3D view: {message} (line {line})", file=sys.stderr)


class DisplayBridge(QObject):
    state = pyqtSignal(str)

    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.available = False

    @pyqtSlot(result=str)
    def bootstrap(self):
        return json.dumps({"map": self.window.map, "demo": self.window.demo})

    @pyqtSlot()
    def rendered(self):
        self.available = True

    @pyqtSlot(str)
    def failed(self, message):
        self.window.statusBar().showMessage("3D 초기화 실패: " + message[:160])

    @pyqtSlot(bool)
    def play(self, value):
        if self.window.demo:
            self.window.playing = value

    @pyqtSlot(float)
    def seek(self, value):
        if self.window.demo and math.isfinite(value) and 0 <= value <= 24:
            self.window.demo_time = value

    @pyqtSlot(str, bool)
    def drop(self, robot, enabled):
        if self.window.demo and robot in ROBOT_IDS:
            if enabled:
                self.window.dropped.add(robot)
            else:
                self.window.dropped.discard(robot)


class SpatialWindow(QMainWindow):
    """GUI-thread methods accept display state only, never issue robot commands."""

    def __init__(self, *, demo=False, map_path=None):
        super().__init__()
        if not demo and map_path is None:
            raise ValueError("provide a ROS map YAML, or explicitly choose --demo")
        self.demo = demo
        self.map = load_map(map_path or ASSETS / "maps/demo.yaml")
        self.cache = StateCache(self.map["id"])
        self._session = None
        self._revision = -1
        self._received_at = None
        self._snapshot = None
        self.playing = True
        self.dropped = set()
        self.demo_time = 0.0
        self._sequence = 0
        self._last_tick = time.monotonic()
        self.setWindowTitle("RO:PORT · 통합관제 3D" + (" · DEMO" if demo else ""))
        self.resize(1380, 900)
        self.setMinimumSize(940, 700)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        header = QHBoxLayout()
        brand = QLabel("  RO:PORT  ")
        brand.setStyleSheet(
            "background:#292f33;color:#f4f6f7;font-size:18px;padding:14px;font-weight:600"
        )
        header.addWidget(brand)
        self.stack = QStackedWidget()
        for index, label in enumerate(("통합관제", "Pinky", "OMX")):
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, i=index: self.show_tab(i))
            header.addWidget(button)
        header.addStretch()
        layout.addLayout(header)
        layout.addWidget(self.stack)
        self.view = QWebEngineView()
        self.profile = QWebEngineProfile(self)
        self.interceptor = LocalAssets(self.profile)
        self.profile.setUrlRequestInterceptor(self.interceptor)
        self.profile.downloadRequested.connect(lambda download: download.cancel())
        self.page = LocalPage(self.profile, self.view)
        self.page.settings().setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, False
        )
        self.page.settings().setAttribute(
            QWebEngineSettings.WebAttribute.JavascriptCanOpenWindows, False
        )
        self.page.settings().setAttribute(
            QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True
        )
        self.view.setPage(self.page)
        self.bridge = DisplayBridge(self)
        self.channel = QWebChannel(self.page)
        self.channel.registerObject("display", self.bridge)
        self.page.setWebChannel(self.channel)
        self.stack.addWidget(self.view)
        self.tabs = QTabWidget()
        for name in ("Pinky", "OMX"):
            self.tabs.addTab(
                QLabel(f"  {name} 담당자 세부 관제 QWidget 연결 영역"), name
            )
        self.stack.addWidget(self.tabs)
        self.view.load(QUrl.fromLocalFile(str(ASSETS / "index.html")))
        self.setStyleSheet(
            "QMainWindow{background:#f5f6f7} QPushButton{border:0;padding:12px 18px;color:#343d43;background:transparent} QPushButton:focus{border:1px solid #547e87}"
        )
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        if demo:
            self.cache.begin_connection()
            self.set_source("demo")
            self.apply_snapshot(demo_snapshot(1))

    _on_ui_thread = ControlWindow._on_ui_thread
    set_detail_widget = ControlWindow.set_detail_widget

    def show_tab(self, index):
        self.stack.setCurrentIndex(0 if index == 0 else 1)
        if index:
            self.tabs.setCurrentIndex(index - 1)

    def set_source(self, session):
        self._on_ui_thread()
        if session is not None and (
            not isinstance(session, str) or not 0 < len(session) <= 160
        ):
            raise ValueError("nonempty source session required")
        self._session, self._revision, self._received_at, self._snapshot = (
            session,
            -1,
            None,
            None,
        )

    def apply_snapshot(self, snapshot: Snapshot):
        self._on_ui_thread()
        if not isinstance(snapshot, Snapshot):
            raise TypeError("Snapshot required")
        if snapshot.session_id != self._session or snapshot.revision <= self._revision:
            return False
        self._snapshot, self._revision, self._received_at = (
            snapshot,
            snapshot.revision,
            time.monotonic(),
        )
        return True

    def apply_robot_sample(self, sample: RobotSample):
        self._on_ui_thread()
        return self.cache.apply(sample)

    def _demo(self, elapsed):
        if self.playing:
            self.demo_time = (self.demo_time + elapsed) % 24
        self._sequence += 1
        t = self.demo_time
        for index, robot in enumerate(ROBOT_IDS):
            if robot in self.dropped:
                continue
            if robot.startswith("pinky"):
                phase = (t / 24 + index / 3) * math.tau
                pose = (
                    2.3 + 1.3 * math.cos(phase),
                    1.3 + 0.65 * math.sin(phase),
                    0,
                    math.atan2(0.65 * math.cos(phase), -1.3 * math.sin(phase)),
                )
                joints = (("l_wheel_joint", t * 3), ("r_wheel_joint", t * 3))
                mode, velocity = "주행 · 모의", 0.2
            else:
                pose = (0.7 + (index - 3) * 1.45, 2.15, 0, -math.pi / 2)
                joints = tuple(
                    (f"joint{i}", 0.35 * math.sin(t * 0.5 + i)) for i in range(1, 6)
                ) + (("gripper_joint_1", 0.3 + 0.2 * math.sin(t)),)
                mode, velocity = "작업 · 모의", None
            self.cache.apply(
                RobotSample(
                    robot,
                    "demo",
                    self._sequence,
                    datetime.now(UTC).isoformat(),
                    self.map["id"],
                    pose=pose,
                    joints=joints,
                    mode=mode,
                    velocity=velocity,
                )
            )

    def tick(self):
        now = time.monotonic()
        elapsed, self._last_tick = min(now - self._last_tick, 0.2), now
        if self.demo:
            self._demo(elapsed)
        if not self.bridge.available or self.stack.currentIndex() != 0:
            return
        snapshot = self._snapshot
        age = now - self._received_at if self._received_at is not None else None
        task_source = (
            "실행부 미연결"
            if age is None
            else ("갱신 지연 · 마지막 작업 상태" if age >= 5 else "작업 상태 수신")
        )
        if self.demo:
            task_source = "모의 작업 · 로봇 동작과 독립된 표시 예시"
        payload = {
            "robots": self.cache.snapshot(),
            "tasks": [asdict(t) for t in snapshot.tasks] if snapshot else [],
            "errors": list(snapshot.errors) if snapshot else [],
            "task_source": task_source,
            "source": "모의 상태 수신"
            if self.demo
            else ("표시 상태 입력" if self.cache.connected else "표시 입력 미연결"),
            "demo_time": self.demo_time,
            "playing": self.playing,
            "dropped": sorted(self.dropped),
            "message": "URDF 기준 외형 · 실기 캘리브레이션 미검증",
        }
        self.bridge.available = False
        self.bridge.state.emit(json.dumps(payload, ensure_ascii=False))

    def closeEvent(self, event):
        self.timer.stop()
        self.view.close()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--map", type=Path, help="ROS trinary PNG/PGM map YAML")
    args = parser.parse_args()
    if not args.demo and args.map is None:
        parser.error("provide --map PATH, or explicitly choose --demo")
    app = QApplication(sys.argv[:1])
    window = SpatialWindow(demo=args.demo, map_path=args.map)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
