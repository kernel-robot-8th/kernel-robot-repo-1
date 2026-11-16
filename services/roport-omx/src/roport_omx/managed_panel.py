"""Opt-in client for the provisional command-aware panel API; no legacy fallback."""

from urllib.parse import quote

from .mqtt_link.messages import check_expiry
from .panel_executor import PanelExecutor


class ManagedPanelExecutor(PanelExecutor):
    command_id = None

    def prepare(self):
        capability = self._call("GET", "/api/control/capabilities", timeout=3)
        if capability.get("command_cancel") != "durable":
            raise ValueError("panel command cancellation unavailable")
        return super().prepare()

    def start(self, task):
        check_expiry(task)
        self.command_id = task["command_id"]
        self._call(
            "POST",
            "/api/control/commands",
            {
                "command_id": self.command_id,
                "expires_at": task["expires_at"],
                "payload": self.start_body(task),
            },
            timeout=3,
        )

    def command_state(self):
        if self.command_id is None:
            raise ValueError("no panel command selected")
        result = self._call(
            "GET", "/api/control/commands/" + quote(self.command_id, safe=""), timeout=3
        )
        if result.get("command_id") != self.command_id:
            raise ValueError("panel command mismatch")
        return result

    def cancel(self):
        if self.command_id is not None:
            self._call(
                "POST",
                "/api/control/commands/" + quote(self.command_id, safe="") + "/cancel",
                {},
                timeout=3,
            )

    def cancellation_settled(self):
        return self.command_state()["state"] in {
            "cancelled",
            "expired",
            "completed",
            "failed",
        }

    def observe(self):
        reply = self.command_state()
        state = reply["state"]
        if state in {"pending", "running"}:
            return {"state": "running", "step": state}
        if state in {"completed", "failed"} and isinstance(
            reply.get("observation"), dict
        ):
            return reply["observation"]
        return {"state": "failed", "error": "panel_command_" + str(state)}
