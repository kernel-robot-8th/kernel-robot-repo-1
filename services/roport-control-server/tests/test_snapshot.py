from dataclasses import replace

import pytest

from roport_control_server.contracts.snapshot import Snapshot, TaskRow


@pytest.mark.parametrize("value", [-1, 101, 1.5, True])
def test_invalid_progress_is_rejected(value):
    with pytest.raises(ValueError):
        TaskRow("id", "Pinky 1", "stage", value, "진행")


@pytest.mark.parametrize(
    "changes",
    [
        {"revision": -1},
        {"revision": True},
        {"session_id": ""},
        {"errors": ("x" * 501,)},
        {"errors": tuple("x" for _ in range(21))},
        {"tasks": (TaskRow("same", "Pinky 1", "stage", 0, "대기"),) * 2},
    ],
)
def test_invalid_snapshot_is_rejected(changes):
    with pytest.raises(ValueError):
        replace(Snapshot("demo", 1), **changes)
