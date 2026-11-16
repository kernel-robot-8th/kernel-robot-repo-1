"""Fleet UI/API boundaries against the real single owner and Unix socket."""

import time
from copy import deepcopy
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient
from fleet_web_fixture import TOKEN, owner

HEADERS = {"origin": "http://127.0.0.1", "x-roport-trial-token": TOKEN}


@pytest.fixture
def local_owner(tmp_path):
    with (
        owner(tmp_path) as fixture,
        TestClient(
            fixture.app, base_url="http://127.0.0.1", client=("127.0.0.1", 50000)
        ) as client,
    ):
        yield client, fixture


def command(client, action, **fields):
    state = client.get("/api/snapshot?mode=fleet").json()["fleet"]
    return {"action": action, "session_id": state["session_id"], **fields}


def post(client, action, **fields):
    return client.post(
        "/api/fleet/commands", headers=HEADERS, json=command(client, action, **fields)
    )


def test_fleet_command_checks_one_snapshot_before_dispatch(local_owner, monkeypatch):
    client, fixture = local_owner
    payload = command(client, "enable")
    view = fixture.app.state.fleet
    rpc = Mock(wraps=view.rpc)
    monkeypatch.setattr(view, "rpc", rpc)
    response = client.post("/api/fleet/commands", headers=HEADERS, json=payload)
    assert response.status_code == 200 and fixture.control.data["enabled"]
    assert [call.args[0]["action"] for call in rpc.call_args_list] == [
        "snapshot",
        "enable",
    ]


@pytest.mark.parametrize("field", ["map_version", "graph_version"])
def test_fleet_map_mismatch_blocks_bootstrap_and_command(
    local_owner, monkeypatch, field
):
    client, fixture = local_owner
    payload = command(client, "enable")
    view = fixture.app.state.fleet
    fixture.control.config[field] = "different-map"
    with pytest.raises(ValueError, match="fleet/map mismatch"):
        view.bootstrap()
    rpc = Mock(wraps=view.rpc)
    monkeypatch.setattr(view, "rpc", rpc)
    response = client.post("/api/fleet/commands", headers=HEADERS, json=payload)
    assert response.status_code == 409
    assert "fleet/map mismatch" in response.json()["detail"]
    assert [call.args[0]["action"] for call in rpc.call_args_list] == ["snapshot"]
    assert not fixture.control.data["enabled"]


def test_fleet_bootstrap_auth_role_assignment_and_request_dedup(local_owner):
    client, fixture = local_owner
    data = client.get("/api/bootstrap").json()
    assert data["mode"] == "fleet" and data["fleet"] and not data["trial"]
    assert len(data["robots"]) == 6
    payload = command(client, "enable")
    for headers in (
        {},
        {**HEADERS, "origin": "https://elsewhere"},
        {**HEADERS, "x-roport-trial-token": "wrong"},
    ):
        assert (
            client.post(
                "/api/fleet/commands", headers=headers, json=payload
            ).status_code
            == 403
        )
    assert not fixture.control.data["enabled"]
    assert post(client, "enable").status_code == 200
    assert post(client, "assign", robot="pinky2", role=None).status_code == 200
    assert fixture.control.data["roles"]["pinky2"] is None
    assert post(client, "assign", robot="pinky2", role="smartcart").status_code == 200
    request = {
        "request_id": "request-1",
        "operation": "inbound",
        "package_id": "box-1",
        "goal": None,
    }
    assert post(client, "submit", request=request).status_code == 200
    assert post(client, "submit", request=request).status_code == 200
    assert len(fixture.control.data["tasks"]) == 1
    assert (
        post(
            client, "submit", request={**request, "package_id": "different"}
        ).status_code
        == 409
    )
    assert fixture.control.data["commands"] == {}


@pytest.mark.parametrize("shelf_state", [None, "reserved"])
def test_outbound_requires_confirmed_shelf_stock_without_mutating_state(
    local_owner, shelf_state
):
    client, fixture = local_owner
    if shelf_state:
        state = deepcopy(fixture.control.data)
        state["slots"]["1a"] = {"state": shelf_state, "package_id": "box"}
        fixture.control.save(state)
    before = deepcopy(fixture.control.data)
    request = {
        "request_id": "missing-stock",
        "operation": "outbound",
        "package_id": "box",
        "goal": "gate",
    }
    for _ in range(2):
        response = post(client, "submit", request=request)
        assert response.status_code == 409
        assert response.json() == {
            "detail": "출고할 물품이 선반에 등록되어 있지 않습니다."
        }
        assert fixture.control.data == before
    snapshot = client.get("/api/snapshot?mode=fleet").json()["fleet"]
    assert "missing-stock" not in snapshot["tasks"]


def test_outbound_confirmed_stock_is_accepted_once(local_owner):
    client, fixture = local_owner
    state = deepcopy(fixture.control.data)
    state["slots"]["1a"] = {"state": "occupied", "package_id": "box"}
    fixture.control.save(state)
    request = {
        "request_id": "confirmed-stock",
        "operation": "outbound",
        "package_id": "box",
        "goal": "gate",
    }
    for _ in range(2):
        response = post(client, "submit", request=request)
        assert response.status_code == 200 and response.json()["accepted"]
    assert len(fixture.control.data["tasks"]) == 1
    task = fixture.control.data["tasks"]["confirmed-stock"]
    assert task["request"] == request and task["status"] == "QUEUED"
    assert fixture.control.data["slots"] == state["slots"]
    assert fixture.control.data["commands"] == {}


def test_stale_robot_new_owner_and_expired_requests_are_rejected(local_owner):
    client, fixture = local_owner
    with fixture.freshness_lock:
        fixture.fresh.clear()
        fixture.control.heard["pinky2"] = time.monotonic() - 5
    data = client.get("/api/snapshot?mode=fleet").json()
    assert next(r for r in data["robots"] if r["robot_id"] == "pinky2")["stale"]
    assert post(client, "assign", robot="pinky2", role=None).status_code == 409
    fixture.fresh.set()
    old = command(client, "enable")
    fixture.channel.session_id = "restarted-owner"
    assert (
        client.post("/api/fleet/commands", headers=HEADERS, json=old).status_code == 409
    )
    assert not fixture.control.data["enabled"]
    current = command(client, "enable")
    assert (
        client.post(
            "/api/fleet/commands",
            headers=HEADERS,
            json={**current, "expires_at": time.time() + 1},
        ).status_code
        == 409
    )
    with pytest.raises(ValueError, match="expired"):
        fixture.channel.dispatch({**current, "expires_at": time.time() - 1})
    assert post(client, "enable").status_code == 200


def test_websocket_reconnect_and_owner_loss_keep_saved_tasks(local_owner):
    client, fixture = local_owner
    request = {
        "request_id": "saved",
        "operation": "inbound",
        "package_id": "box",
        "goal": None,
    }
    assert post(client, "submit", request=request).status_code == 200
    with client.websocket_connect(
        "ws://127.0.0.1/api/state?mode=fleet", headers=HEADERS
    ) as ws:
        first = ws.receive_json()
        assert first["fleet"]["available"]
    with client.websocket_connect(
        "ws://127.0.0.1/api/state?mode=fleet", headers=HEADERS
    ) as ws:
        second = ws.receive_json()
        assert second["fleet"]["tasks"] == first["fleet"]["tasks"]
        assert second["revision"] > first["revision"]
    fixture.app.state.fleet.socket_path += "-missing"
    snapshot = client.get("/api/snapshot?mode=fleet").json()
    assert not snapshot["fleet"]["available"]
    assert all(r["stale"] for r in snapshot["robots"])
    assert snapshot["fleet"]["tasks"] == first["fleet"]["tasks"]
    assert (
        client.post(
            "/api/fleet/commands",
            headers=HEADERS,
            json={"action": "stop", "session_id": first["fleet"]["session_id"]},
        ).status_code
        == 503
    )


@pytest.mark.parametrize("mode,status", [("simulation", 200), ("hardware", 409)])
def test_customer_receipt_requires_service_evidence_on_hardware(tmp_path, mode, status):
    with owner(tmp_path, mode) as fixture:
        state = deepcopy(fixture.control.data)
        state["slots"]["1a"] = {"state": "occupied", "package_id": "box"}
        fixture.control.save(state)
        fixture.control.submit(
            {
                "request_id": "receipt",
                "operation": "outbound",
                "package_id": "box",
                "goal": "gate",
            }
        )
        state = deepcopy(fixture.control.data)
        state["tasks"]["receipt"].update(
            stage="await_receipt", status="AWAITING_RECEIPT"
        )
        state["slots"]["1a"] = {"state": "empty", "package_id": None}
        fixture.control.save(state)
        with TestClient(
            fixture.app, base_url="http://127.0.0.1", client=("127.0.0.1", 50000)
        ) as client:
            response = post(client, "receipt", request_id="receipt")
            assert response.status_code == status
            assert fixture.control.data["tasks"]["receipt"]["stage"] == (
                "return_nav" if mode == "simulation" else "await_receipt"
            )
            if mode == "hardware":
                assert "service evidence" in response.json()["detail"]


def test_hardware_remote_http_blocks_commands_without_losing_state(tmp_path):
    with (
        owner(tmp_path, "hardware") as fixture,
        TestClient(
            fixture.app,
            base_url="http://control.example.test",
            client=("192.0.2.4", 50000),
        ) as client,
    ):
        response = client.post(
            "/api/fleet/commands",
            headers={**HEADERS, "origin": "http://control.example.test"},
            json=command(client, "enable"),
        )
        assert response.status_code == 403
        assert "HTTPS" in response.json()["detail"]
        assert not fixture.control.data["enabled"]
