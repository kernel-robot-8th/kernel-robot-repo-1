"""Virtual sensors/executors for contract tests. Never loaded by a hardware-mode runner."""

from copy import deepcopy
from time import monotonic


class VirtualOmxExecutor:
    def __init__(
        self,
        arms,
        *,
        run_s=0.15,
        fail=False,
        placement=True,
        unreturned=(),
        package_size="small",
        package_sizes=None,
        model_version="sim-model",
        calibration_version="sim-cal",
    ):
        self.versions = {
            "model_version": model_version,
            "calibration_version": calibration_version,
        }
        self.arms = tuple(arms)
        self.run_s, self.fail, self.placement = run_s, fail, placement
        self.unreturned, self.package_size = set(unreturned), package_size
        self.package_sizes = dict(package_sizes or {})
        self.tasks, self.started, self.aborted = [], None, False

    def _arms(self, running=False):
        return {
            a: {
                "connected": True,
                "safe": True,
                "returned": not running and a not in self.unreturned,
                "grasping": running,
                "stopped": not running,
            }
            for a in self.arms
        }

    def readiness(self):
        running = (
            self.started is not None
            and not self.aborted
            and monotonic() - self.started < self.run_s
        )
        return {
            "armed": True,
            "busy": running,
            "error": None,
            "arms": self._arms(running),
        }

    def start(self, task):
        self.tasks.append(deepcopy(task))
        self.started, self.aborted = monotonic(), False

    def observe(self):
        if self.aborted:
            return {"state": "stopped", "stopped": True, "arms": self._arms()}
        if self.started is None:
            return {"state": "idle"}
        if monotonic() - self.started < self.run_s:
            return {"state": "running", "step": "moving", "arms": self._arms(True)}
        if self.fail:
            return {"state": "failed", "error": "virtual_arm_failure"}
        return {
            "state": "finished",
            "arms": self._arms(),
            "placement_confirmed": self.placement,
            "release_confirmed": True,
            "package_size": self.package_sizes.get(
                self.tasks[-1].get("package_id"), self.package_size
            ),
        }

    def abort(self):
        self.aborted = True


def virtual_executors(run_s=0.15, package_size="small", package_sizes=None):
    return {
        target: VirtualOmxExecutor(
            arms, run_s=run_s, package_size=package_size, package_sizes=package_sizes
        )
        for target, arms in {
            "shelf-01": ("omx1", "omx2"),
            "omx1": ("omx1",),
            "omx2": ("omx2",),
            "omx3": ("omx3",),
        }.items()
    }


def virtual_alignment(host, command):
    """Explicitly simulated docking sensor: observes a simulated Pinky's reported stationary visit."""
    pinky_id = host._pinky(command)
    p = host.pinkies.get(pinky_id)
    if not p or p["state"].get("stopped") is not True:
        return None
    return {
        "pinky_id": pinky_id,
        "pinky_boot_version": p["boot_version"],
        "arrival_command_id": p["state"].get("arrival_command_id"),
        "observed_at": p["observed_at"],
        "alignment": "confirmed",
    }
