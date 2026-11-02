"""Read-only PyQt shell. Snapshot rendering only; no runtime or robot connections."""

import argparse
import sys
import time
from dataclasses import dataclass

from PyQt6.QtCore import Qt, QThread, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


def _text(value, limit=160):
    if not isinstance(value, str) or not value or len(value) > limit:
        raise ValueError(f"nonempty text up to {limit} characters required")


@dataclass(frozen=True)
class TaskRow:
    task_id: str
    robot: str
    stage: str
    progress: int
    status: str

    def __post_init__(self):
        for value in (self.task_id, self.robot, self.stage):
            _text(value)
        if type(self.progress) is not int or not 0 <= self.progress <= 100:
            raise ValueError("progress requires an integer from 0 to 100")
        if self.status not in ("대기", "진행", "완료", "오류"):
            raise ValueError("unknown display status")


@dataclass(frozen=True)
class Snapshot:
    session_id: str
    revision: int
    tasks: tuple[TaskRow, ...] = ()
    errors: tuple[str, ...] = ()

    def __post_init__(self):
        _text(self.session_id)
        if type(self.revision) is not int or self.revision < 0:
            raise ValueError("revision requires a nonnegative integer")
        if (
            not isinstance(self.tasks, tuple)
            or len(self.tasks) > 100
            or not all(isinstance(t, TaskRow) for t in self.tasks)
        ):
            raise ValueError("snapshot requires at most 100 TaskRow entries")
        if len({t.task_id for t in self.tasks}) != len(self.tasks):
            raise ValueError("duplicate task IDs")
        if not isinstance(self.errors, tuple) or len(self.errors) > 20:
            raise ValueError("snapshot requires at most 20 errors")
        for error in self.errors:
            _text(error, 500)


def _label(text, name=""):
    widget = QLabel(text)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setObjectName(name)
    widget.setWordWrap(True)
    return widget


class ControlWindow(QMainWindow):
    def __init__(self, *, demo=False):
        super().__init__()
        self.demo = demo
        self._session = None
        self._revision = -1
        self._received_at = None
        self.setWindowTitle("RO:PORT · 통합관제" + (" · DEMO" if demo else ""))
        self.resize(1120, 820)
        self.setMinimumSize(820, 720)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(28, 22, 28, 22)
        layout.setSpacing(16)
        heading = QHBoxLayout()
        heading.addWidget(_label("RO:PORT  /  통합관제", "title"))
        heading.addStretch()
        heading.addWidget(_label("Pinky 3대  ·  OMX 3대", "muted"))
        layout.addLayout(heading)
        self.banner = _label(
            "DEMO · 모의 데이터입니다. 실제 장비와 연결하지 않습니다."
            if demo
            else "OFFLINE · 실행부 미연결. 표시할 상태를 기다리고 있습니다.",
            "banner",
        )
        layout.addWidget(self.banner)
        summary = QHBoxLayout()
        self.connection = _label("미연결", "value")
        self.task_count = _label("—", "value")
        self.updated = _label("수신 이력 없음", "value")
        for title, value in [
            ("상태 수신", self.connection),
            ("표시 중인 작업", self.task_count),
            ("마지막 갱신", self.updated),
        ]:
            card = QGroupBox(title)
            QVBoxLayout(card).addWidget(value)
            summary.addWidget(card)
        layout.addLayout(summary)
        task_header = QHBoxLayout()
        task_header.addWidget(_label("작업 현황", "section"))
        task_header.addStretch()
        self.action_buttons = []
        for title in ("작업 승인", "출발 승인", "작업 취소"):
            button = QPushButton(title)
            button.setEnabled(False)
            button.setToolTip("실행부 연동 전에는 사용할 수 없습니다.")
            self.action_buttons.append(button)
            task_header.addWidget(button)
        layout.addLayout(task_header)
        self.tasks = QTableWidget(0, 5)
        self.tasks.setHorizontalHeaderLabels(
            ["작업 ID", "대상", "현재 단계", "진행률", "상태"]
        )
        self.tasks.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.tasks.verticalHeader().hide()
        self.tasks.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.tasks.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.tasks.setAlternatingRowColors(True)
        self.tasks.setMinimumHeight(160)
        layout.addWidget(self.tasks, 2)
        self.errors = QPlainTextEdit(
            "수신된 오류 정보가 없습니다. 장비 정상 판정이 아닙니다."
        )
        self.errors.setReadOnly(True)
        self.errors.setMaximumHeight(85)
        self.errors.setAccessibleName("오류 및 주의 내역")
        error_box = QGroupBox("오류 · 주의")
        QVBoxLayout(error_box).addWidget(self.errors)
        layout.addWidget(error_box)
        self.tabs = QTabWidget()
        for name in ("Pinky", "OMX"):
            page = QFrame()
            box = QVBoxLayout(page)
            box.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title = _label(f"{name} 세부 관제", "section")
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            detail = _label(
                "팀원 개발 위젯 연결 영역\n통합관제는 탭 틀만 제공하며 세부 제어는 포함하지 않습니다.",
                "muted",
            )
            detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
            box.addWidget(title)
            box.addWidget(detail)
            self.tabs.addTab(page, name)
        self.tabs.setMinimumHeight(180)
        layout.addWidget(self.tabs, 2)
        layout.addWidget(
            _label("화면 뼈대 · 실행부 / DB / MQTT / UDP 연결 없음", "muted")
        )
        self.setStyleSheet("""
            QMainWindow { background: #f4f6fa; }
            QWidget { font-size: 14px; color: #233248; }
            QLabel#title { font-size: 25px; font-weight: 700; }
            QLabel#section { font-size: 17px; font-weight: 600; }
            QLabel#muted { color: #526279; }
            QLabel#banner { background: #fff1cd; color: #674c09; padding: 12px; border-radius: 6px; }
            QLabel#value { font-size: 19px; font-weight: 600; padding: 6px; }
            QGroupBox { background: white; border: 1px solid #d8dee8; border-radius: 6px; margin-top: 12px; padding-top: 12px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QTableWidget { background: white; alternate-background-color: #f7f9fc; border: 1px solid #d8dee8; gridline-color: #e8edf4; }
            QHeaderView::section { background: #e9eef5; padding: 8px; border: none; }
            QTabWidget::pane { background: white; border: 1px solid #d8dee8; }
            QTabBar::tab { padding: 10px 28px; background: #e4eaf2; }
            QTabBar::tab:selected { background: #233e60; color: white; }
            QPushButton { padding: 7px 12px; }
            QPushButton:disabled { color: #747e8d; background: #e5e9f0; border: 1px solid #cbd3df; border-radius: 4px; }
            QProgressBar { text-align: center; background: #e6ebf3; border: none; }
            QProgressBar::chunk { background: #79b6bd; }
        """)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh_age)
        self._timer.start(1000)

    def _on_ui_thread(self):
        if QThread.currentThread() != self.thread():
            raise RuntimeError("call UI methods on the Qt GUI thread")

    def set_source(self, session_id: str | None):
        """Explicitly change the display source; old session snapshots remain invalid."""
        self._on_ui_thread()
        if session_id is not None:
            _text(session_id)
        self._session = session_id
        self._revision = -1
        self._received_at = None
        self.tasks.setRowCount(0)
        self.task_count.setText("—")
        self.errors.setPlainText(
            "수신된 오류 정보가 없습니다. 장비 정상 판정이 아닙니다."
        )
        self.refresh_age()

    def apply_snapshot(self, snapshot: Snapshot) -> bool:
        """Full display snapshot. Wire parsing/IPC and business decisions are external."""
        self._on_ui_thread()
        if snapshot.session_id != self._session or snapshot.revision <= self._revision:
            return False
        self._revision = snapshot.revision
        self._received_at = time.monotonic()
        self.tasks.setRowCount(len(snapshot.tasks))
        for row, task in enumerate(snapshot.tasks):
            for column, text in enumerate(
                (task.task_id, task.robot, task.stage, "", task.status)
            ):
                self.tasks.setItem(row, column, QTableWidgetItem(text))
            progress = QProgressBar()
            progress.setValue(task.progress)
            progress.setAccessibleName(f"{task.task_id} 진행률")
            self.tasks.setCellWidget(row, 3, progress)
        self.task_count.setText(f"{len(snapshot.tasks)}건")
        self.errors.setPlainText(
            "\n".join(snapshot.errors) or "수신된 오류 없음 · 장비 정상 판정과 별개"
        )
        self.refresh_age()
        return True

    def refresh_age(self):
        self._on_ui_thread()
        if self._received_at is None:
            self.connection.setText(
                "미연결" if self._session is None else "첫 상태 대기"
            )
            self.updated.setText("수신 이력 없음")
        else:
            age = max(0, time.monotonic() - self._received_at)
            stale = age >= 5
            self.connection.setText(
                "갱신 지연 · 상태 확인 필요"
                if stale
                else ("모의 상태 수신" if self.demo else "상태 수신")
            )
            self.updated.setText(f"{int(age)}초 전 · revision {self._revision}")
        if not self.demo:
            self.banner.setText(
                "OFFLINE · 실행부 미연결"
                if self._session is None
                else "표시용 상태 입력 · 실제 통신·장비 준비는 별도 확인 필요"
            )

    def set_detail_widget(self, name: str, widget: QWidget):
        """Install a teammate's QWidget in the matching tab on the UI thread."""
        self._on_ui_thread()
        if name not in ("Pinky", "OMX") or not isinstance(widget, QWidget):
            raise ValueError("Pinky/OMX QWidget required")
        index = 0 if name == "Pinky" else 1
        previous = self.tabs.widget(index)
        if widget is previous:
            return
        if widget.parent() is not None:
            raise ValueError("provide an unparented detail widget")
        selected = self.tabs.currentIndex()
        self.tabs.removeTab(index)
        self.tabs.insertTab(index, widget, name)
        self.tabs.setCurrentIndex(selected)
        previous.deleteLater()


def demo_snapshot(revision: int) -> Snapshot:
    return Snapshot(
        "demo",
        revision,
        (
            TaskRow(
                "DEMO-001",
                "Pinky 1",
                "고정 경로 운송 (모의)",
                min(revision * 10, 100),
                "진행" if revision < 10 else "완료",
            ),
            TaskRow("DEMO-002", "OMX 1·2", "공동 작업대 대기 (모의)", 0, "대기"),
            TaskRow("DEMO-003", "OMX 3", "카트 상차 완료 (모의)", 100, "완료"),
        ),
        ("[모의 경고] 카메라 상태 확인 예시 · 실제 장비 오류가 아닙니다.",),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo", action="store_true", help="display explicitly simulated data"
    )
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    window = ControlWindow(demo=args.demo)
    if args.demo:
        window.set_source("demo")
        revision = 1

        def update():
            nonlocal revision
            window.apply_snapshot(demo_snapshot(revision))
            revision += 1

        timer = QTimer(window)
        timer.timeout.connect(update)
        timer.start(1000)
        update()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
