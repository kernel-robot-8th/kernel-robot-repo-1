"""Optional registration metadata owned by the same executor as Fleet roles."""

from uuid import uuid4

from ..contracts.robots import (
    PINKY_SLOTS,
    RegistryError,
    RegistryRequest,
    ReleaseEvidence,
)


class FleetRegistry:
    def __init__(self, control):
        self.control = control
        if not hasattr(control.store, "registry_update"):
            raise ValueError("registry requires the operational PostgreSQL store")
        control.store.registry_update(self._adopt)
        control.registry = self

    @staticmethod
    def roles(snapshot):
        return {
            row["robot_id"]: row["desired_config"]["role"]
            if row["registered"] and not row["excluded"]
            else None
            for row in snapshot["robots"]
            if row["robot_id"] in PINKY_SLOTS
        }

    def _adopt(self, data, snapshot, _result, _changed):
        roles = self.roles(snapshot)
        if data.get("registry_id") == snapshot["registry_id"]:
            if data["roles"] != roles:
                raise RegistryError("registry_roles_mismatch", 409)
            return
        if data.get("registry_id"):
            raise RegistryError("registry_identity_changed", 409)
        if (
            data["enabled"]
            or data["commands"]
            or data["resources"]
            or any(
                task["status"] not in {"COMPLETE", "CANCELED"}
                for task in data["tasks"].values()
            )
            or data.get("pending_arrivals")
        ):
            raise RegistryError("finish_work_before_enabling_registry", 409)
        data.update(roles=roles, registry_id=snapshot["registry_id"])

    def _evidence(self, robot, role, confirmed):
        control = self.control
        control.ready(robot)
        control.validate_assignment(robot, role)
        owned_tasks = {
            key
            for key, task in control.data["tasks"].items()
            if task.get("robot") == robot
        }
        if "robot:" + robot in control.data["resources"] or any(
            owner in owned_tasks for owner in control.data["resources"].values()
        ):
            raise RegistryError("occupied_resource", 409)
        return ReleaseEvidence(
            online=True,
            stopped=True,
            active_task=False,
            pending_command=False,
            occupied_resource=False,
            loaded=False if confirmed else None,
            maintenance=False,
        )

    def change(self, robot, operation, data, *, actor, confirmed=None):
        if robot not in PINKY_SLOTS:
            raise RegistryError("unknown_pinky_slot", 404)
        request = RegistryRequest.parse(operation, data)
        if operation == "unregister":
            if (
                not isinstance(confirmed, dict)
                or set(confirmed) != {"empty_load", "removed_from_lane"}
                or any(value is not True for value in confirmed.values())
            ):
                raise RegistryError("confirm_empty_load_and_removed_from_lane", 409)
        elif confirmed is not None:
            raise RegistryError("unexpected_confirmation")

        def check(row):
            role = (
                None
                if operation == "unregister"
                or (operation == "exclude" and request.excluded)
                else request.config.role
                if request.config
                else row["role"]
            )
            return self._evidence(robot, role, confirmed)

        def update(state, snapshot, _result, changed):
            if not changed:
                return  # Replayed unregister remains valid after its observation was cleared.
            state["roles"] = self.roles(snapshot)
            if operation == "unregister":
                boot = self.control.observed[robot]["boot_version"]
                retired = state.setdefault("registry_retired_boots", {}).setdefault(
                    robot, []
                )
                if boot not in retired:
                    retired.append(boot)
                state["last_nodes"].pop(robot, None)
                state.get("pending_arrivals", {}).pop(robot, None)

        result, changed = self.control.store.registry_update(
            update,
            robot=robot,
            action=operation,
            request=request,
            check=check,
            actor=actor,
            confirmed=confirmed,
        )
        if changed and operation == "unregister":
            for cache in (
                self.control.observed,
                self.control.poses,
                self.control.heard,
                self.control.observed_since,
            ):
                cache.pop(robot, None)
            self.control.awaiting_state.discard(robot)
        return result

    def assign(self, robot, role, *, actor):
        row = next(
            (row for row in self.snapshot()["robots"] if row["robot_id"] == robot), None
        )
        if not row or not row["registered"]:
            raise RegistryError("register_robot_first", 409)
        request = {
            "request_id": str(uuid4()),
            "expected_revision": row["revision"],
            "binding_id": row["binding_id"],
        }
        if role is None:
            return self.change(
                robot, "exclude", {**request, "excluded": True}, actor=actor
            )
        if row["excluded"]:
            raise RegistryError("reinclude_robot_before_assigning_role", 409)
        return self.change(
            robot,
            "config",
            {**request, "config": {**row["desired_config"], "role": role}},
            actor=actor,
        )

    def snapshot(self):
        snapshot = self.control.store.registry_snapshot()
        for row in snapshot["robots"]:
            robot = row["robot_id"]
            row["fleet_role"] = self.control.data["roles"].get(robot)
            observed = self.control.observed.get(robot)
            if observed:
                row["observation"].update(
                    boot_id=observed["boot_version"],
                    sequence=observed["sequence"],
                    observed_at=observed.get("observed_at"),
                    online=self.control.state_age(robot) <= 3,
                    stopped=observed["state"].get("stopped"),
                )
            # Registration is operator metadata, not proof of SSH-applied settings or serial identity.
            try:
                self.control.ready(robot)
            except ValueError as exc:
                reason = str(exc)
            else:
                reason = None
            row["display"].update(
                freshness="fresh"
                if observed and self.control.state_age(robot) <= 3
                else "unknown",
                ready=bool(
                    row["registered"] and not row["excluded"] and reason is None
                ),
                reasons=(["unregistered"] if not row["registered"] else [])
                + (["excluded"] if row["excluded"] else [])
                + ([reason] if reason else [])
                + ["config_unverified"],
            )
        return snapshot
