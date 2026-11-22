from contextlib import ExitStack
from unittest.mock import AsyncMock

import pytest
from fastapi import WebSocket
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect
from uvicorn.middleware.proxy_headers import ProxyHeadersMiddleware

from roport_control_server.adapters.api.app import create_app
from roport_control_server.adapters.api.display import Display


@pytest.fixture
def client():
    with TestClient(create_app()) as client:
        yield client


def test_host_ids_and_unknown_host_never_invent_values():
    from roport_control_server.contracts.robots import HOST_IDS, unknown_host

    assert HOST_IDS == ("pinky1", "pinky2", "pinky3", "omx3", "shelf-01")
    row = unknown_host("omx3")
    assert row.pop("host_id") == "omx3" and row.pop("status") == "unknown"
    assert row and all(value is None for value in row.values())


def test_host_metrics_endpoint_without_trial_is_not_configured(client):
    response = client.get("/api/hosts/metrics")
    assert response.status_code == 200
    data = response.json()
    assert data["receiver"] == "not_configured" and data["rejected"] == {}
    assert [h["host_id"] for h in data["hosts"]] == [
        "pinky1",
        "pinky2",
        "pinky3",
        "omx3",
        "shelf-01",
    ]
    assert all(
        h["status"] == "unknown" and h["cpu_temp_c"] is None for h in data["hosts"]
    )
    assert client.post("/api/hosts/metrics").status_code == 405


def test_offline_bootstrap_assets_and_no_hardware_controls(client):
    response = client.get("/")
    assert response.status_code == 200
    assert "RO:PORT" in response.text
    assert "qtwebchannel" not in response.text
    assert client.get("/ui/models/pinky/robot.glb").content[:4] == b"glTF"
    assert client.get("/ui/vendor/three/esm/three.module.js").status_code == 200
    config = client.get("/api/bootstrap").json()
    assert config["demo"] is False and config["example_map"] is True
    assert "source" not in config["map"]  # No server-local path disclosure.
    assert len(config["robots"]) == 6
    data = client.get("/api/snapshot").json()
    assert data["tasks"] == [] and data["demo"] is False
    assert all(r["stale"] and r["pose"] is None for r in data["robots"])
    assert client.post("/api/commands", json={"action": "start"}).status_code == 404
    assert client.get("/api/bootstrap?mode=live").status_code == 422
    assert client.get("/ui/%2e%2e/%2e%2e/pyproject.toml").status_code == 404


def test_stream_revisions_reconnect_and_viewer_isolation(client):
    headers = {"origin": "http://testserver"}
    with client.websocket_connect("/api/state?mode=demo", headers=headers) as first:
        a = first.receive_json()
        b = first.receive_json()
        assert b["revision"] > a["revision"]
        assert b["session_id"] == a["session_id"]
        assert any(r["pose"] for r in b["robots"])
        first.send_json({"action": "play", "value": False})
        for _ in range(10):
            paused = first.receive_json()
            if not paused["playing"]:
                break
        assert not paused["playing"]
        with client.websocket_connect(
            "/api/state?mode=demo", headers=headers
        ) as second:
            other = second.receive_json()
            assert other["playing"] and other["session_id"] != a["session_id"]
    with client.websocket_connect("/api/state", headers=headers) as reconnected:
        data = reconnected.receive_json()
        assert data["session_id"] != a["session_id"]
        assert data["tasks"] == [] and not data["demo"]
    assert client.get("/healthz").json() == {"status": "ok"}


@pytest.mark.parametrize("origin", ["https://other.example", "null", ""])
def test_cross_origin_stream_rejected(client, origin):
    with (
        pytest.raises(WebSocketDisconnect) as error,
        client.websocket_connect("/api/state", headers={"origin": origin}),
    ):
        pass
    assert error.value.code == 1008


@pytest.mark.parametrize("accept_fails", [False, True])
def test_stream_limit_rejects_excess_and_releases_slots(
    client, monkeypatch, accept_fails
):
    headers = {"origin": "http://testserver"}
    if accept_fails:
        with monkeypatch.context() as patch:
            patch.setattr(WebSocket, "accept", AsyncMock(side_effect=OSError))
            with (
                pytest.raises(OSError),
                client.websocket_connect("/api/state", headers=headers),
            ):
                pass
    # Fill twice: all slots must be returned after disconnect, including errors.
    for _ in range(2):
        with ExitStack() as connections:
            streams = [
                connections.enter_context(
                    client.websocket_connect(f"/api/state?mode={mode}", headers=headers)
                )
                for mode in ("offline", "demo") * 4
            ]
            for ws in streams:
                ws.receive_json()
            with (
                pytest.raises(WebSocketDisconnect) as error,
                client.websocket_connect("/api/state", headers=headers),
            ):
                pass
            assert error.value.code == 1013
            assert client.get("/healthz").status_code == 200
            streams[-1].send_text("invalid JSON")
            with pytest.raises(WebSocketDisconnect) as error:
                while True:
                    streams[-1].receive_json()
            assert error.value.code == 1008


@pytest.mark.parametrize(
    "peer, host, forwarded_proto, origin, accepted",
    [
        ("192.0.2.10", "control.example", None, "http://control.example", True),
        ("127.0.0.1", "control.example", "https", "https://control.example", True),
        ("192.0.2.10", "control.example", "https", "https://control.example", False),
        ("127.0.0.1", "127.0.0.1:8000", "https", "https://control.example", False),
    ],
)
def test_stream_origin_behind_proxy(peer, host, forwarded_proto, origin, accepted):
    app = ProxyHeadersMiddleware(create_app(), trusted_hosts=["127.0.0.1"])
    headers = {"host": host, "origin": origin}
    if forwarded_proto:
        headers["x-forwarded-proto"] = forwarded_proto
    with TestClient(app, client=(peer, 12345)) as client:
        if accepted:
            with client.websocket_connect("/api/state", headers=headers) as ws:
                assert ws.receive_json()["demo"] is False
        else:
            with (
                pytest.raises(WebSocketDisconnect) as error,
                client.websocket_connect("/api/state", headers=headers),
            ):
                pass
            assert error.value.code == 1008


@pytest.mark.parametrize(
    "payload, code", [('"x"', 1008), ('{"action":"start"}', 1008), ("x" * 1025, 1009)]
)
def test_invalid_stream_controls_close_connection(client, payload, code):
    with client.websocket_connect(
        "/api/state?mode=demo", headers={"origin": "http://testserver"}
    ) as ws:
        ws.receive_json()
        ws.send_text(payload)
        with pytest.raises(WebSocketDisconnect) as error:
            while True:
                ws.receive_json()
        assert error.value.code == code


def test_offline_controls_and_binary_frames_are_rejected(client):
    for binary, code in ((False, 1008), (True, 1003)):
        with client.websocket_connect(
            "/api/state", headers={"origin": "http://testserver"}
        ) as ws:
            ws.receive_json()
            if binary:
                ws.send_bytes(b"wrong protocol")
            else:
                ws.send_json({"action": "play", "value": True})
            with pytest.raises(WebSocketDisconnect) as error:
                while True:
                    ws.receive_json()
            assert error.value.code == code


def test_demo_drop_preserves_last_pose_but_marks_stale(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr(
        "roport_control_server.adapters.api.display.time.monotonic", lambda: clock[0]
    )
    display = Display(demo=True)
    first = display.snapshot()["robots"][0]
    display.command({"action": "drop", "robot_id": "pinky1", "value": True})
    clock[0] += 3
    stale = display.snapshot()["robots"][0]
    assert stale["stale"] and stale["pose"] == first["pose"]
    display.command({"action": "drop", "robot_id": "pinky1", "value": False})
    assert not display.snapshot()["robots"][0]["stale"]


@pytest.mark.parametrize(
    "data",
    [
        {"action": "play", "value": 1},
        {"action": "seek", "value": float("nan")},
        {"action": "seek", "value": 25},
        {"action": "seek", "value": True},
        {"action": "drop", "value": True, "robot_id": "unknown"},
        {"action": "play", "value": True, "extra": 1},
    ],
)
def test_demo_command_validation(data):
    with pytest.raises(ValueError):
        Display(demo=True).command(data)
