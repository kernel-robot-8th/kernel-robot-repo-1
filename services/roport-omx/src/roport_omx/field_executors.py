"""Adapters preserve the team's device owners; no simulated evidence in a hardware executor."""

from copy import deepcopy
from datetime import UTC, datetime
from threading import RLock

from .omx3 import judge
from .omx3.host import Omx3Host
from .omx3.target import resolve_marker_id


class WarehouseExecutor:
    """r8 command owner -> PR71 Host -> existing OMX3 Runner.

    package_sizes is a provisional operator-maintained package catalogue, not a vision claim.
    marker_ids maps business package IDs to the existing ArUco marker identities.
    """

    def __init__(
        self,
        runner,
        *,
        marker_ids,
        model_version,
        calibration_version,
        package_sizes=None,
    ):
        self.versions = {
            "model_version": model_version,
            "calibration_version": calibration_version,
        }
        self.runner, self.marker_ids = runner, dict(marker_ids)
        self.package_sizes = dict(package_sizes or {})
        self.lock = RLock()
        self.observation = {"state": "idle"}
        self.task = None
        self.host = Omx3Host(runner, self._event)

    def _arms(self):
        try:
            ready = self.runner.robot.ready() is None
            pose = self.runner.robot.observe().state
            home = judge.near_init(pose, self.runner.settings)
            released = judge.gripper_open(pose, self.runner.settings)
        except Exception:  # noqa: BLE001 - a failed device read is unknown, never ready
            ready, home, released = False, None, None
        return {
            "omx3": {
                "connected": ready,
                "safe": ready,
                "returned": home,
                "grasping": None if released is None else not released,
            }
        }

    def readiness(self):
        status = self.host.status()
        return {
            "armed": status["robot"] == "ok",
            "busy": status["active"] is not None,
            "error": "omx3_reconciling" if status["reconciling"] else None,
            "arms": self._arms(),
        }

    def check(self, task):
        marker = self.marker_ids.get(task["package_id"])
        resolve_marker_id(
            {"package_id": str(marker)}, "package_id", self.runner.settings.package_ids
        )
        code = self.runner.precheck(marker)
        if code is not None:
            raise ValueError(str(code))

    def start(self, task):
        self.check(task)
        with self.lock:
            self.task = deepcopy(task)
            self.observation = {"state": "running", "step": "accepted"}
        # This ID stays inside the legacy Host; only command_id is transmitted in r8.
        request = {
            "command_id": task["command_id"],
            "execution_id": task["command_id"],
            "payload": {"package_id": str(self.marker_ids[task["package_id"]])},
        }
        self.host.handle_load(request)

    def _event(self, kind, request, payload):
        with self.lock:
            if kind == "load.completed":
                self.observation = {
                    "state": "finished",
                    "arms": {
                        "omx3": {
                            "connected": True,
                            "safe": payload.get("departure_ready") is True,
                            "returned": payload.get("arm_returned"),
                            "grasping": not payload["gripper_open"]
                            if isinstance(payload.get("gripper_open"), bool)
                            else None,
                        }
                    },
                    "placement_confirmed": payload.get("placement_confirmed"),
                    "release_confirmed": payload.get("gripper_open"),
                    "package_size": self.package_sizes.get(
                        self.task["package_id"], "unknown"
                    ),
                }
            elif kind in {"load.failed", "load.rejected"}:
                self.observation = {
                    "state": "failed",
                    "error": payload.get("error_code", "omx3_failed"),
                }
            else:
                self.observation = {
                    "state": "running",
                    "step": payload.get("phase", "accepted"),
                }

    def observe(self):
        with self.lock:
            return deepcopy(self.observation)

    def abort(self):
        self.host.abort()


class PanelFieldExecutor:
    """PR66 panel executor supports cooperative actions only. Missing sensors remain unknown."""

    def __init__(
        self, panel, *, model_version, calibration_version, completion=None, stop=None
    ):
        self.versions = {
            "model_version": model_version,
            "calibration_version": calibration_version,
        }
        self.panel, self.completion, self.stop = panel, completion, stop
        self.task = None

    def readiness(self):
        return self.panel.readiness()

    def check(self, task):
        if task["robot_ids"] != ["omx1", "omx2"]:
            raise ValueError("independent_panel_driver_unavailable")
        if self.stop is None:
            raise ValueError("physical_stop_adapter_unavailable")
        self.panel.check(task)

    def start(self, task):
        self.check(task)
        self.task = deepcopy(task)
        self.panel.start(task)

    def observe(self):
        result = self.panel.observe()
        # The panel's return evidence alone is not an independent placement sensor.
        result.pop("placement_confirmed", None)
        result.pop("release_confirmed", None)
        if result.get("state") == "finished" and self.completion:
            proof = self.completion(self.task)
            # A cached result from another package/visit must never release the Pinky.
            try:
                age = (
                    datetime.now(UTC) - datetime.fromisoformat(proof["observed_at"])
                ).total_seconds()
            except (TypeError, KeyError, ValueError):
                age = float("inf")
            if (
                isinstance(proof, dict)
                and -1 <= age <= 3
                and all(
                    proof.get(key) == self.task[key]
                    for key in (
                        "command_id",
                        "package_id",
                        "slot_id",
                        "arrival_command_id",
                    )
                )
            ):
                result = {
                    **result,
                    "placement_confirmed": proof.get("placement_confirmed"),
                    "release_confirmed": proof.get("release_confirmed"),
                }
        return result

    def abort(self):
        if self.stop is None:
            raise RuntimeError("physical_stop_adapter_unavailable")
        self.stop()


class IndependentArmExecutor:
    """Execute a single calibrated arm using the team's real Runner, selected per slot/direction.

    Each routes[(INBOUND|OUTBOUND, slot)] Runner has its own arm, policy, home pose and
    destination camera region. Sharing the OMX3 calibration is deliberately unsupported.
    """

    def __init__(
        self, arm_id, routes, *, marker_ids, model_version, calibration_version
    ):
        if arm_id not in {"omx1", "omx2"} or not routes:
            raise ValueError("independent arm and calibrated routes are required")
        self.versions = {
            "model_version": model_version,
            "calibration_version": calibration_version,
        }
        self.arm_id = arm_id
        self.routes = {
            key: WarehouseExecutor(runner, marker_ids=marker_ids, **self.versions)
            for key, runner in routes.items()
        }
        self.current = None

    def readiness(self):
        statuses = [adapter.readiness() for adapter in self.routes.values()]
        flags = [s["arms"]["omx3"] for s in statuses]

        def agreed(key):
            values = [f.get(key) for f in flags]
            return values[0] if all(v is values[0] for v in values) else None

        return {
            "armed": all(s["armed"] is True for s in statuses),
            "busy": any(s["busy"] for s in statuses),
            "error": next((s["error"] for s in statuses if s["error"]), None),
            "arms": {
                self.arm_id: {
                    k: agreed(k) for k in ("connected", "safe", "returned", "grasping")
                }
            },
        }

    def check(self, task):
        if task["robot_ids"] != [self.arm_id]:
            raise ValueError("independent_arm_mismatch")
        adapter = self.routes.get((task["operation"], task.get("slot_id")))
        if adapter is None:
            raise ValueError("calibrated_independent_route_unavailable")
        adapter.check(task)

    def start(self, task):
        self.check(task)
        self.current = self.routes[(task["operation"], task["slot_id"])]
        self.current.start(task)

    def observe(self):
        result = self.current.observe() if self.current else {"state": "idle"}
        if "arms" in result:
            result["arms"] = {self.arm_id: result["arms"]["omx3"]}
        result.pop("package_size", None)
        return result

    def abort(self):
        if self.current:
            self.current.abort()
