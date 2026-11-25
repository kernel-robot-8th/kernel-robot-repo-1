import json
import shutil
import ssl
import subprocess
from copy import deepcopy
from datetime import UTC, datetime, timedelta
from email.message import Message
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from threading import Thread
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


def test_frame_ca_accepts_only_valid_server_identity(frames, tmp_path, monkeypatch):
    openssl = shutil.which("openssl")
    if not openssl:
        pytest.skip("OpenSSL CLI is required to generate a temporary test certificate")
    cert, key = tmp_path / "localhost.pem", tmp_path / "localhost.key"
    subprocess.run(
        [
            openssl,
            "req",
            "-new",
            "-x509",
            "-newkey",
            "ec",
            "-pkeyopt",
            "ec_paramgen_curve:prime256v1",
            "-nodes",
            "-days",
            "1",
            "-subj",
            "/CN=localhost",
            "-addext",
            "subjectAltName=DNS:localhost",
            "-keyout",
            str(key),
            "-out",
            str(cert),
        ],
        check=True,
        capture_output=True,
    )
    monkeypatch.setenv("no_proxy", "localhost,127.0.0.1")

    class FrameHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if (
                self.path != "/frames/image"
                or self.headers.get("Authorization") != "Bearer frame-token"
            ):
                self.send_error(401)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.end_headers()
            self.wfile.write(b"test jpeg")

        def log_message(self, *_):
            pass

    _, fleet, _ = frames
    tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    tls.load_cert_chain(cert, key)
    with ThreadingHTTPServer(("127.0.0.1", 0), FrameHandler) as server:
        server.socket = tls.wrap_socket(server.socket, server_side=True)
        thread = Thread(target=server.serve_forever)
        thread.start()
        try:

            def relay(host, ca_file=None):
                sources = json.dumps({"pinky3": f"https://{host}:{server.server_port}"})
                return FollowFrames(
                    "preview-token", sources, "frame-token", ca_file=ca_file
                ).get(fleet, "Bearer preview-token", "receipt", "frame")

            assert relay("localhost", cert).body == b"test jpeg"
            for host, ca_file in (
                ("localhost", None),
                ("127.0.0.1", cert),
                ("localhost", tmp_path / "missing.pem"),
            ):
                with pytest.raises(HTTPException) as exc:
                    relay(host, ca_file)
                assert exc.value.status_code == 502
        finally:
            server.shutdown()
            thread.join()
