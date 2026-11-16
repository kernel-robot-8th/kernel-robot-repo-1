"""Mount inside the actual panel device owner; transport/auth stay with its HTTP server."""

import re
from urllib.parse import unquote, urlsplit

from .mqtt_link.field_wire import ID


class PanelCommandAPI:
    def __init__(self, ledger, *, validate_payload, begin, stop):
        self.ledger = ledger
        self.validate_payload, self.begin, self.stop = validate_payload, begin, stop

    def handle(self, method, path, body=None):
        """Return (HTTP status, JSON). begin must spawn locally and return promptly.

        Device completion and measured stop go to the ledger from trusted local
        callbacks, never from an HTTP caller's claimed result.
        """
        path = urlsplit(path).path
        if method == "GET" and path == "/api/control/capabilities":
            return 200, {"command_cancel": "durable"}
        if method == "POST" and path == "/api/control/commands":
            try:
                if (
                    not isinstance(body, dict)
                    or not isinstance(body.get("command_id"), str)
                    or not re.fullmatch(ID["pattern"], body["command_id"])
                    or not isinstance(body.get("payload"), dict)
                ):
                    raise ValueError("invalid command")
                if self.ledger.get(body["command_id"])["state"] == "not_found":
                    self.validate_payload(body["payload"])
            except (ValueError, TypeError, KeyError):
                return 400, {"error": "invalid_command_payload"}
            try:
                self.ledger.submit(body)
            except (ValueError, TypeError, KeyError):
                return 409, {"error": "command_conflict_or_invalid_expiry"}
            try:
                result = self.ledger.start(body["command_id"], self.begin)
            except Exception:  # noqa: BLE001 - preserve unknown device outcome
                return 503, {"error": "local_start_outcome_unknown"}
            return 202 if result["state"] == "running" else 200, result
        prefix = "/api/control/commands/"
        if path.startswith(prefix):
            tail = path[len(prefix) :]
            cancel = tail.endswith("/cancel")
            command_id = unquote(tail[:-7] if cancel else tail)
            if not re.fullmatch(ID["pattern"], command_id):
                return 400, {"error": "invalid_command_id"}
            if method == "GET" and not cancel:
                return 200, self.ledger.get(command_id)
            if method == "POST" and cancel and body == {}:
                try:
                    return 200, self.ledger.cancel(command_id, self.stop)
                except Exception:  # noqa: BLE001 - preserve unresolved physical stop
                    return 503, {"error": "local_stop_outcome_unknown"}
        return 404, {"error": "unknown_command_endpoint"}
