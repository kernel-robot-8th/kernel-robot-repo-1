"""Simulated COOPERATIVE executor for link tests. No device, no panel; never use for hardware.

Imports only the standard library so ci/ tests can load it with the system Python.
"""

from time import monotonic

ARMS = ("omx1", "omx2")


class MockExecutor:
    """Finishes `run_s` seconds after start; `unreturned` arms stay away from home."""

    def __init__(
        self,
        run_s: float = 3.0,
        *,
        armed: bool = True,
        unreturned: set[str] = frozenset(),
    ):
        self.run_s, self.armed, self.unreturned = run_s, armed, set(unreturned)
        self.started_at: float | None = None
        self.tasks: list[dict] = []

    def _running(self) -> bool:
        return (
            self.started_at is not None and monotonic() - self.started_at < self.run_s
        )

    def _arms(self, *, moving: bool) -> dict:
        return {
            arm: {
                "connected": True,
                "grasping": moving,
                "returned": not moving and arm not in self.unreturned,
                "safe": True,
            }
            for arm in ARMS
        }

    def readiness(self) -> dict:
        return {
            "armed": self.armed,
            "busy": self._running(),
            "error": None,
            "arms": self._arms(moving=self._running()),
        }

    def start(self, task: dict) -> None:
        if self._running():
            raise RuntimeError("mock already running")
        self.tasks.append(task)
        self.started_at = monotonic()

    def observe(self) -> dict:
        if self.started_at is None:
            return {"state": "idle"}
        elapsed = monotonic() - self.started_at
        if elapsed < self.run_s:
            step = "pick" if elapsed < self.run_s / 2 else "place"
            return {"state": "running", "step": step, "arms": self._arms(moving=True)}
        return {"state": "finished", "step": "home", "arms": self._arms(moving=False)}
