"""Authenticated current-frame relay; robot origins come only from deployment config."""

import json
import secrets
import ssl
from datetime import UTC, datetime
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import HTTPRedirectHandler, HTTPSHandler, Request, build_opener

from fastapi import HTTPException
from fastapi.responses import Response

MAX_IMAGE_BYTES = 2 * 1024 * 1024


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class FollowFrames:
    def __init__(self, token, sources, frame_token, *, ca_file=None):
        self.token, self.frame_token = token, frame_token
        self.ca_file = ca_file
        self.sources = json.loads(sources or "{}")
        if not isinstance(self.sources, dict):
            raise TypeError("follow frame sources must map robot IDs to origins")
        for robot, origin in self.sources.items():
            parsed = urlsplit(origin)
            if (
                robot not in {"pinky1", "pinky2", "pinky3"}
                or parsed.scheme not in {"http", "https"}
                or not parsed.hostname
                or parsed.username
                or parsed.password
                or parsed.query
                or parsed.fragment
                or parsed.path not in {"", "/"}
                or (
                    parsed.scheme == "http"
                    and parsed.hostname not in {"localhost", "127.0.0.1", "::1"}
                )
            ):
                raise ValueError(
                    "follow frame source requires HTTPS or loopback HTTP origin"
                )

    def get(self, fleet, authorization, session_id, frame_ref):
        if not self.token or not fleet:
            raise HTTPException(503, "follow preview not configured")
        if not secrets.compare_digest(authorization, "Bearer " + self.token):
            raise HTTPException(401, "follow preview authentication required")
        try:
            data = fleet.rpc({"action": "snapshot"})
            fleet._check_map(data)
        except (OSError, ValueError):
            raise HTTPException(503, "follow owner unavailable") from None
        matches = [
            t
            for t in data["tasks"].values()
            if t.get("follow", {}).get("session_id") == session_id
        ]
        if len(matches) != 1:
            raise HTTPException(404, "unknown follow session")
        task = matches[0]
        registration = task["follow"].get("registration", {})
        if registration.get("frame_ref") != frame_ref:
            raise HTTPException(404, "frame is no longer current")
        try:
            expires = datetime.fromisoformat(registration["expires_at"])
            valid = expires.tzinfo and expires > datetime.now(UTC)
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            raise HTTPException(410, "frame expired")
        robot = task["robot"]
        observed = data.get("robots", {}).get(robot, {})
        if data.get("ages", {}).get(robot, 4) > 3 or observed.get(
            "boot_version"
        ) != task["follow"].get("robot_boot_version"):
            raise HTTPException(409, "frame source stale")
        origin = self.sources.get(robot)
        image_ref = registration.get("image_ref")
        if (
            not origin
            or not self.frame_token
            or not image_ref
            or image_ref in {".", ".."}
        ):
            raise HTTPException(503, "robot frame provider unavailable")
        url = origin.rstrip("/") + "/frames/" + quote(image_ref, safe="")
        request = Request(url, headers={"Authorization": "Bearer " + self.frame_token})
        try:
            handlers = [NoRedirect()]
            if self.ca_file and urlsplit(origin).scheme == "https":
                context = ssl.create_default_context()
                context.load_verify_locations(cafile=self.ca_file)
                handlers.append(HTTPSHandler(context=context))
            with build_opener(*handlers).open(request, timeout=2) as response:
                content_type = response.headers.get_content_type()
                if content_type not in {"image/jpeg", "image/png"}:
                    raise ValueError("unsupported frame content")
                image = response.read(MAX_IMAGE_BYTES + 1)
                if not image or len(image) > MAX_IMAGE_BYTES:
                    raise ValueError("invalid frame size")
        except (HTTPError, URLError, OSError, ValueError):
            raise HTTPException(502, "robot frame unavailable") from None
        # Recheck ownership after I/O; a late capture must not replace a newer selection.
        try:
            latest = fleet.rpc({"action": "snapshot"})
        except (OSError, ValueError):
            raise HTTPException(503, "follow owner unavailable") from None
        current = latest.get("tasks", {}).get(task["request"]["request_id"], {})
        if (
            current.get("follow", {}).get("registration") != registration
            or latest.get("robots", {}).get(robot, {}).get("boot_version")
            != task["follow"].get("robot_boot_version")
            or latest.get("ages", {}).get(robot, 4) > 3
            or expires <= datetime.now(UTC)
        ):
            raise HTTPException(409, "frame superseded")
        return Response(
            image, media_type=content_type, headers={"Cache-Control": "no-store"}
        )
