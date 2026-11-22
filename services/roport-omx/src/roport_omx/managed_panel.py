"""Opt-in client for the provisional command-aware panel API; no legacy fallback."""

from urllib.parse import quote

from .mqtt_link.messages import check_expiry
from .panel_executor import PanelExecutor, PanelHTTPError


class PanelStartRejected(ValueError):
    """The panel definitively refused this start; no ambiguous POST outcome."""


class ManagedPanelExecutor(PanelExecutor):
    command_id = None
    capability_checked = False

    def prepare(self):
        if not self.capability_checked:
            capability = self._call("GET", "/api/control/capabilities", timeout=3)
            if capability.get("command_cancel") != "durable":
                raise ValueError("panel command cancellation unavailable")
            self.capability_checked = True
        return super().prepare()

    def start(self, task):
        check_expiry(task)
        self.command_id = task["command_id"]
        try:
            result = self._call(
                "POST",
                "/api/control/commands",
                {
                    "command_id": self.command_id,
                    "expires_at": task["expires_at"],
                    "payload": self.start_body(task),
                },
                timeout=3,
            )
        except PanelHTTPError as exc:
            if 400 <= exc.status < 500:
                raise PanelStartRejected(f"panel_start_rejected_{exc.status}") from exc
            raise
        if not isinstance(result, dict) or result.get("command_id") != self.command_id:
            raise ValueError("panel command mismatch")
        state = result.get("state")
        if state in {"cancelled", "expired", "completed", "failed"}:
            raise PanelStartRejected("panel_command_" + state)
        if state not in {"pending", "running"}:
            raise ValueError("panel start outcome unknown")

    def command_state(self, *, timeout=3):
        if self.command_id is None:
            raise ValueError("no panel command selected")
        result = self._call(
            "GET",
            "/api/control/commands/" + quote(self.command_id, safe=""),
            timeout=timeout,
        )
        if result.get("command_id") != self.command_id:
            raise ValueError("panel command mismatch")
        return result

    def cancel(self, *, timeout=1):
        if self.command_id is not None:
            self._call(
                "POST",
                "/api/control/commands/" + quote(self.command_id, safe="") + "/cancel",
                {},
                timeout=timeout,
            )

    def cancellation_settled(self):
        return self.cancellation_status()["settled"]

    def cancellation_status(self):
        reply = self.command_state(timeout=1)
        state = reply["state"]
        observation = reply.get("observation")
        return {
            "settled": state in {"cancelled", "expired", "completed", "failed"},
            "error": observation.get("error")
            if isinstance(observation, dict)
            else None,
            # A bare cancellation can confirm stop but may omit an emergency stop's cause.
            "local_release_only": state == "cancelled"
            and not isinstance(observation, dict),
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
