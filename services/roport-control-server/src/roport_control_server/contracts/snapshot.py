"""UI display snapshot contract and validation (stdlib only)."""

from dataclasses import dataclass


def check_text(value, limit=160):
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
            check_text(value)
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
        check_text(self.session_id)
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
            check_text(error, 500)
