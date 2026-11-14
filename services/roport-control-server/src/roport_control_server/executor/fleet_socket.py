"""Use the existing local socket server; only the owner mutates fleet state."""

import time
from copy import deepcopy

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

    def dispatch(self, request):
        if request == {"action": "snapshot"}:
            return self.snapshot()
        if (
            not isinstance(request, dict)
            or request.get("session_id") != self.session_id
        ):
            raise ValueError("executor changed; refresh before acting")
        expiry = request.get("expires_at")
        if (
            type(expiry) not in (int, float)
            or not time.time() < expiry <= time.time() + 2
        ):
            raise ValueError("expired operator request")
        action = request.get("action")
        fields = {
            "enable": set(),
            "stop": set(),
            "assign": {"robot", "role"},
            "submit": {"request"},
            "receipt": {"request_id"},
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
        elif action == "submit":
            self.control.submit(request["request"])
        elif action == "receipt":
            if self.control.config["mode"] != "simulation":
                raise ValueError(
                    "hardware receipt requires authenticated service evidence"
                )
            self.control.receipt(request["request_id"])
        return deepcopy(self.snapshot())
