"""Adapters preserve the team's device owners; no simulated evidence in a hardware executor."""

from concurrent.futures import Future
from copy import deepcopy
from datetime import UTC, datetime
from logging import getLogger
from threading import Lock, RLock, Thread
from time import monotonic

from .omx3 import judge
from .omx3.contract import Phase
from .omx3.host import Omx3Host
from .omx3.target import resolve_marker_id


def _expired(task):
    return datetime.fromisoformat(task["expires_at"]) <= datetime.now(UTC)


def _background(call):
    future = Future()

    def run():
        try:
            future.set_result(call())
        except Exception as exc:  # noqa: BLE001 - hand the device failure back to the owner
            future.set_exception(exc)

    Thread(target=run, name="omx-panel-io", daemon=True).start()
    return future


def _stopped(sensor, since=None):
    """A home pose or an abort request is not a measured stop."""
    if sensor is None:
        return False
    try:
        sample = sensor()
        observed = datetime.fromisoformat(sample["observed_at"])
        age = (datetime.now(UTC) - observed).total_seconds()
        return (
            sample.get("stopped") is True
            and 0 <= age <= 3
            and (since is None or observed >= since)
        )
    except Exception:  # noqa: BLE001 - missing or unreadable stop evidence is unknown
        return False


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
        stop_status=None,
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
        self.stop_status, self.stop_requested_at = stop_status, None
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
                "stopped": _stopped(self.stop_status),
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
        if _expired(task):
            raise ValueError("expired_command")
        with self.lock:
            self.stop_requested_at = None
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
            # Runner repeats camera/readiness checks on its worker before any motor target.
            if (
                kind == "load.progress"
                and payload.get("phase") == Phase.HOMING
                and _expired(self.task)
            ):
                self.host.abort()
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
            result = deepcopy(self.observation)
        stopped = _stopped(self.stop_status, self.stop_requested_at)
        if self.stop_requested_at is not None and stopped:
            return {"state": "stopped", "stopped": True}
        for arm in result.get("arms", {}).values():
            arm["stopped"] = stopped
        return result

    def abort(self):
        self.stop_requested_at = datetime.now(UTC)
        self.host.abort()
        if self.host.status()["active"] is None:
            self.runner.robot.hold()


class PanelFieldExecutor:
    """PR66 panel executor supports cooperative actions only. Missing sensors remain unknown."""

    def __init__(
        self,
        panel,
        *,
        model_version,
        calibration_version,
        completion=None,
        stop=None,
        stop_status=None,
    ):
        self.versions = {
            "model_version": model_version,
            "calibration_version": calibration_version,
        }
        self.panel, self.completion, self.stop = panel, completion, stop
        self.task = None
        self.stop_status, self.stop_requested_at = stop_status, None
        self.lock, self.io_lock = RLock(), Lock()
        self.closed = False
        self.start_future = self.stop_future = None
        self.reads = {}
        self.start_unknown = self.stop_again = False
        self.stop_running = False

    def _sample(self, name, read):
        """One bounded outstanding read per kind; stale replies are never re-dated."""
        with self.lock:
            entry = self.reads.setdefault(
                name, {"future": None, "sample": None, "since": monotonic()}
            )
            future = entry["future"]
            if future and future.done():
                entry["future"] = None
                entry["sample"] = None
                entry["sample"] = future.result()
            sample = entry["sample"]
            if entry["future"] is None and not self.closed:

                def fetch():
                    with self.io_lock:
                        if self.closed:
                            return monotonic(), {}
                        observed = monotonic()
                        return observed, read()

                entry["future"] = _background(fetch)
            if sample and monotonic() - sample[0] <= 3:
                return deepcopy(sample[1])
            if monotonic() - (sample[0] if sample else entry["since"]) > 3:
                raise TimeoutError("panel observation stale")
            return None

    def readiness(self):
        if self.start_future is not None and not self.start_future.done():
            return {"armed": False, "busy": True, "error": None, "arms": {}}
        error = None
        try:
            result = self._sample("ready", self.panel.prepare)
        except Exception:  # noqa: BLE001 - failed reads cannot assert readiness
            result = None
            error = "panel_observation_failed"
        if result is None or self.closed or self.stop_requested_at is not None:
            result = {
                "armed": False,
                "busy": self.task is not None
                and not self.closed
                and self.stop_requested_at is None,
                "error": error,
                "arms": {},
            }
        stopped = _stopped(self.stop_status)
        for arm in result.get("arms", {}).values():
            arm["stopped"] = stopped
        return result

    def check(self, task):
        if task["robot_ids"] != ["omx1", "omx2"]:
            raise ValueError("independent_panel_driver_unavailable")
        if self.stop is None:
            raise ValueError("physical_stop_adapter_unavailable")
        self.panel.check(task, cached=True)

    def start(self, task):
        self.check(task)
        if _expired(task):
            raise ValueError("expired_command")
        with self.lock:
            if (
                self.closed
                or self.start_unknown
                or self.stop_requested_at is not None
                or (self.start_future and not self.start_future.done())
            ):
                raise ValueError("panel_held_or_starting")
            self.task = deepcopy(task)
            self.reads.pop("ready", None)
            self.reads.pop("observe", None)
            self.start_future = _background(self._start)

    def _start(self):
        sent = False
        try:
            with self.io_lock:
                self.panel.check(self.task)
                ready = self.panel.readiness()
                if (
                    ready.get("armed") is not True
                    or ready.get("busy")
                    or ready.get("error")
                ):
                    raise ValueError("executor_not_ready")
                if not _stopped(self.stop_status) or any(
                    any(
                        ready.get("arms", {}).get(arm, {}).get(key) is not expected
                        for key, expected in (
                            ("connected", True),
                            ("safe", True),
                            ("returned", True),
                            ("grasping", False),
                        )
                    )
                    for arm in self.task["robot_ids"]
                ):
                    raise ValueError("arm_not_safely_home")
                with self.lock:
                    if self.closed or self.stop_requested_at is not None:
                        return
                    if _expired(self.task):
                        raise ValueError("expired_command")
                    sent = True
                self.panel.start(self.task)
        except Exception:
            # A timeout does not cancel a POST already received by the panel.
            self.start_unknown = sent
            raise
        finally:
            with self.lock:
                if sent and self.stop_requested_at is not None:
                    self._request_stop()

    def observe(self):
        with self.lock:
            stopped = _stopped(self.stop_status, self.stop_requested_at)
            if self.stop_requested_at is not None:
                cancel_settled = getattr(self.panel, "cancellation_settled", None)
                if cancel_settled and self.panel.command_id is not None:
                    try:
                        self.start_unknown = (
                            self._sample("cancel", cancel_settled) is not True
                        )
                    except Exception:  # noqa: BLE001 - a query failure cannot prove cancellation
                        self.start_unknown = True
                settled = self.start_future is None or self.start_future.done()
                stop_done = (
                    self.stop_future
                    and self.stop_future.done()
                    and self.stop_future.exception() is None
                )
                return (
                    {"state": "stopped", "stopped": True}
                    if stopped and settled and stop_done and not self.start_unknown
                    else {"state": "stopping"}
                )
        if self.start_future is None:
            return {"state": "idle"}
        if not self.start_future.done():
            return {"state": "running", "step": "starting"}
        if self.start_future.exception() is not None:
            return {"state": "failed", "error": "panel_start_unconfirmed"}
        try:
            result = self._sample("observe", self.panel.observe)
        except Exception:  # noqa: BLE001 - preserve uncertain device outcomes
            return {"state": "failed", "error": "panel_observation_failed"}
        if result is None:
            return {"state": "running", "step": "observing"}
        for arm in result.get("arms", {}).values():
            arm["stopped"] = stopped
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
                and 0 <= age <= 3
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
        with self.lock:
            self._request_stop()

    def _request_stop(self):
        self.stop_requested_at = datetime.now(UTC)
        if self.stop_running:
            self.stop_again = True
        else:
            self.stop_running = True
            self.stop_future = _background(self._stop)

    def _stop(self):
        # This path never waits for the HTTP start/read lock.
        while True:
            with self.lock:
                self.stop_again = False
                self.stop_requested_at = datetime.now(UTC)
            try:
                cancel = getattr(self.panel, "cancel", None)
                try:
                    self.stop()
                finally:
                    if cancel:
                        cancel()
            except Exception:
                with self.lock:
                    self.stop_running = False
                raise
            with self.lock:
                if not self.stop_again:
                    self.stop_running = False
                    return

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            if self.start_future is not None:
                self._request_stop()
            stop = self.stop_future
        if stop:
            try:
                stop.result(timeout=3)
            except Exception:  # noqa: BLE001 - shutdown cannot certify a failed/timed-out stop
                getLogger(__name__).warning("Panel shutdown stop remains unconfirmed")


class IndependentArmExecutor:
    """Execute a single calibrated arm using the team's real Runner, selected per slot/direction.

    Each routes[(INBOUND|OUTBOUND, slot)] Runner has its own arm, policy, home pose and
    destination camera region. Sharing the OMX3 calibration is deliberately unsupported.
    """

    def __init__(
        self,
        arm_id,
        routes,
        *,
        marker_ids,
        model_version,
        calibration_version,
        stop_status=None,
    ):
        if arm_id not in {"omx1", "omx2"} or not routes:
            raise ValueError("independent arm and calibrated routes are required")
        self.versions = {
            "model_version": model_version,
            "calibration_version": calibration_version,
        }
        self.arm_id = arm_id
        self.routes = {
            key: WarehouseExecutor(
                runner, marker_ids=marker_ids, stop_status=stop_status, **self.versions
            )
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
                    k: agreed(k)
                    for k in ("connected", "safe", "returned", "grasping", "stopped")
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
        if self.current is None:
            stopped = all(
                adapter.observe().get("stopped") is True
                for adapter in self.routes.values()
            )
            return (
                {"state": "stopped", "stopped": True} if stopped else {"state": "idle"}
            )
        result = self.current.observe()
        if "arms" in result:
            result["arms"] = {self.arm_id: result["arms"]["omx3"]}
        result.pop("package_size", None)
        return result

    def abort(self):
        if self.current:
            self.current.abort()
        else:
            for adapter in self.routes.values():
                adapter.abort()
