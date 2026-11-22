"""Use the existing local socket server; only the owner mutates fleet state."""

import socket
import struct
import time
from copy import deepcopy

from .fleet_recovery import reconcile, recover_stop, release, request_host_release
from .trial_socket import TrialSocket


class FleetSocket(TrialSocket):
    def snapshot(self):
        return {
            **self.control.snapshot(),
            "session_id": self.session_id,
            "mode": self.control.config["mode"],
            "map_version": self.control.config["map_version"],
            "graph_version": self.control.config["graph_version"],
        }

    def _release_status(self, request_id):
        if not isinstance(request_id, str) or not request_id:
            raise ValueError("release request ID required")
        record = self.control.data.get("host_releases", {}).get(request_id)
        if record is None:
            raise ValueError("unknown release request")
        return {"status": record["status"]}

    def _dispatch_client(self, request, client):
        if isinstance(request, dict) and request.get("action") == "release_workcell":
            return self.dispatch(request, client=client)
        return super()._dispatch_client(request, client)

    def dispatch(self, request, *, client=None):
        if request == {"action": "snapshot"}:
            return self.snapshot()
        if (
            isinstance(request, dict)
            and request.get("action") == "host_release_status"
            and set(request) == {"action", "request_id"}
        ):
            return self._release_status(request["request_id"])
        if (
            not isinstance(request, dict)
            or request.get("session_id") != self.session_id
        ):
            raise ValueError("executor changed; refresh before acting")
        action = request.get("action")
        expiry = request.get("expires_at")
        # A delayed STOP may only hold this owner session; other actions still expire.
        if action != "stop" and (
            type(expiry) not in (int, float)
            or not time.time() < expiry <= time.time() + 2
        ):
            raise ValueError("expired operator request")
        fields = {
            "enable": set(),
            "stop": set(),
            "assign": {"robot", "role"},
            "submit": {"request"},
            "receipt": {"request_id"},
            "reconcile": {"command_id", "outcome", "note"},
            "release_workcell": {"request_id", "hold_command_id", "confirmed"},
            "release_hold": {"note"},
            "recover_stop": {"command_id", "stop_id", "boot_version", "note"},
            "confirm_arrival": {"robot", "command_id", "boot_version", "note"},
        }
        if (
            action not in fields
            or set(request) != {"action", "session_id", "expires_at"} | fields[action]
        ):
            raise ValueError("invalid fleet request")
        if action == "enable":
            self.control.enable()
        elif action == "stop":
            self.control.hold("operator_stop")
        elif action == "assign":
            self.control.assign(request["robot"], request["role"])
        elif action == "reconcile":
            reconcile(
                self.control, request["command_id"], request["outcome"], request["note"]
            )
        elif action == "confirm_arrival":
            self.control.confirm_arrival_after_restart(
                request["robot"],
                command_id=request["command_id"],
                boot_version=request["boot_version"],
                note=request["note"],
            )
        elif action == "recover_stop":
            recover_stop(
                self.control,
                request["command_id"],
                request["stop_id"],
                request["boot_version"],
                request["note"],
            )
        elif action == "release_hold":
            release(self.control, request["note"])
        elif action == "release_workcell":
            if client is None:
                raise ValueError("operator peer credentials required")
            try:
                credentials = client.getsockopt(
                    socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("iII")
                )
                _, uid, _ = struct.unpack("iII", credentials)
            except (OSError, struct.error) as exc:
                raise ValueError("could not verify operator peer credentials") from exc
            request_host_release(
                self.control,
                request["request_id"],
                request["hold_command_id"],
                f"uid:{uid}",
                request["confirmed"],
            )
            return self._release_status(request["request_id"])
        elif action == "submit":
            self.control.submit(request["request"])
        elif action == "receipt":
            if self.control.config["mode"] != "simulation":
                raise ValueError(
                    "hardware receipt requires authenticated service evidence"
                )
            self.control.receipt(request["request_id"])
        return deepcopy(self.snapshot())
