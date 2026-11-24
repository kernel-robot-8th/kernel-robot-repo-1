from copy import deepcopy
from datetime import UTC, datetime, timedelta
from email.message import Message
from io import BytesIO
from unittest.mock import Mock

import pytest
from fastapi import HTTPException

from roport_control_server.adapters.api import follow_frames
from roport_control_server.adapters.api.follow_frames import FollowFrames


@pytest.fixture
def frames():
    data = {
        "tasks": {
            "task": {
                "request": {"request_id": "task"},
                "robot": "pinky3",
                "follow": {
                    "session_id": "receipt",
                    "robot_boot_version": "boot",
                    "registration": {
                        "frame_ref": "frame",
                        "image_ref": "image",
                        "expires_at": (
                            datetime.now(UTC) + timedelta(seconds=60)
                        ).isoformat(),
                    },
                },
            }
        },
        "robots": {"pinky3": {"boot_version": "boot"}},
        "ages": {"pinky3": 0},
    }
    fleet = Mock()
    fleet.rpc.return_value = data
    relay = FollowFrames(
        "preview-token", '{"pinky3":"http://127.0.0.1:9999"}', "frame-token"
    )
    return relay, fleet, data


@pytest.mark.parametrize(
    "origin",
    [
        "http://robot.local",
        "https://robot.local/path",
        "https://user:password@robot.local",
        "https://robot.local?token=a",
    ],
)
def test_only_configured_secure_origins(origin):
    import json

    with pytest.raises(ValueError):
        FollowFrames("token", json.dumps({"pinky3": origin}), "token")


@pytest.mark.parametrize(
    "problem,code",
    [
        ("auth", 401),
        ("session", 404),
        ("frame", 404),
        ("expired", 410),
        ("boot", 409),
        ("stale", 409),
    ],
)
def test_frame_ownership_and_freshness_before_io(frames, monkeypatch, problem, code):
    relay, fleet, data = frames
    opened = Mock()
    monkeypatch.setattr(follow_frames, "build_opener", opened)
    token, session, frame = "Bearer preview-token", "receipt", "frame"
    if problem == "auth":
        token = "wrong"
    if problem == "session":
        session = "another"
    if problem == "frame":
        frame = "old"
    if problem == "expired":
        data["tasks"]["task"]["follow"]["registration"]["expires_at"] = (
            "2000-01-01T00:00:00+00:00"
        )
    if problem == "boot":
        data["robots"]["pinky3"]["boot_version"] = "reboot"
    if problem == "stale":
        data["ages"]["pinky3"] = 4
    with pytest.raises(HTTPException) as exc:
        relay.get(fleet, token, session, frame)
    assert exc.value.status_code == code
    opened.assert_not_called()


@pytest.mark.parametrize(
    "problem", [None, "oversized", "html", "superseded", "rebooted", "stale"]
)
def test_relay_limits_and_post_io_session_check(frames, monkeypatch, problem):
    relay, fleet, data = frames
    stream = BytesIO(
        b"x" * (follow_frames.MAX_IMAGE_BYTES + 1) if problem == "oversized" else b"png"
    )
    stream.headers = Message()
    stream.headers["Content-Type"] = "text/html" if problem == "html" else "image/png"
    opener = Mock()
    opener.open.return_value = stream
    monkeypatch.setattr(follow_frames, "build_opener", lambda *args: opener)
    if problem in {"superseded", "rebooted", "stale"}:
        latest = deepcopy(data)
        if problem == "superseded":
            latest["tasks"]["task"]["follow"].pop("registration")
        elif problem == "rebooted":
            latest["robots"]["pinky3"]["boot_version"] = "new-boot"
        else:
            latest["ages"]["pinky3"] = 4
        fleet.rpc.side_effect = [data, latest]
    if problem:
        with pytest.raises(HTTPException) as exc:
            relay.get(fleet, "Bearer preview-token", "receipt", "frame")
        assert exc.value.status_code == (
            502 if problem in {"oversized", "html"} else 409
        )
    else:
        response = relay.get(fleet, "Bearer preview-token", "receipt", "frame")
        assert (
            response.body == b"png" and response.headers["cache-control"] == "no-store"
        )
        request = opener.open.call_args.args[0]
        assert request.full_url == "http://127.0.0.1:9999/frames/image"
        assert request.headers["Authorization"] == "Bearer frame-token"
