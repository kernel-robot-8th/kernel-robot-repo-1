"""The registration UI uses the existing authenticated Fleet operator boundary."""

import os
from uuid import uuid4

import pytest
from fastapi.testclient import TestClient
from fleet_web_fixture import TOKEN
from registry_web_fixture import owner

HEADERS = {"origin": "http://127.0.0.1", "x-roport-trial-token": TOKEN}


@pytest.fixture
def registered_owner(tmp_path):
    if not os.environ.get("ROPORT_TEST_DATABASE_URL"):
        if os.environ.get("REQUIRE_REGISTRY_TESTS") == "1":
            pytest.fail("ROPORT_TEST_DATABASE_URL required")
        pytest.skip("isolated PostgreSQL required")
    with (
        owner(tmp_path) as fixture,
        TestClient(
            fixture.app, base_url="http://127.0.0.1", client=("127.0.0.1", 50000)
        ) as client,
    ):
        yield client, fixture


def registration(client, **changes):
    state = client.get("/api/snapshot?mode=fleet").json()["fleet"]
    return {
        "action": "registry",
        "session_id": state["session_id"],
        "robot": "pinky1",
        "operation": "registration",
        "request": {
            "request_id": str(uuid4()),
            "expected_revision": 0,
            "device_id": "sim-browser-device",
            "config": {
                "ip": "192.0.2.11",
                "role": "smartcart",
                "ros_domain_id": 14,
                "calibration": "pinky_d660",
            },
        },
        **changes,
    }


def test_registry_auth_and_three_slots_share_fleet_state(registered_owner):
    client, fixture = registered_owner
    payload = registration(client)
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
    for headers in (HEADERS, HEADERS):
        response = client.post("/api/fleet/commands", headers=headers, json=payload)
        assert response.status_code == 200, response.json()
    data = response.json()["result"]
    rows = [r for r in data["registry"]["robots"] if r["kind"] == "pinky"]
    assert [r["robot_id"] for r in rows] == ["pinky1", "pinky2", "pinky3"]
    assert sum(r["registered"] for r in rows) == 1
    assert data["roles"]["pinky1"] == "smartcart"
    assert fixture.control.data["roles"] == data["roles"]
    assert rows[0]["revision"] == 1


@pytest.mark.parametrize(
    "change",
    [
        {"robot": "pinky4"},
        {"operation": "deploy"},
        {"session_id": "old-session"},
        {"confirmed": {"empty_load": True, "removed_from_lane": True}},
    ],
)
def test_registry_invalid_requests_do_not_mutate_roles(registered_owner, change):
    client, fixture = registered_owner
    response = client.post(
        "/api/fleet/commands", headers=HEADERS, json=registration(client, **change)
    )
    assert response.status_code == 409
    assert all(role is None for role in fixture.control.data["roles"].values())


def test_unregister_requires_exclusion_operator_confirmation_and_fresh_stop(
    registered_owner,
):
    client, fixture = registered_owner
    message = registration(client)
    accepted = client.post("/api/fleet/commands", headers=HEADERS, json=message).json()[
        "result"
    ]
    row = next(r for r in accepted["registry"]["robots"] if r["robot_id"] == "pinky1")
    message.update(
        operation="exclude",
        request={
            "request_id": str(uuid4()),
            "expected_revision": row["revision"],
            "binding_id": row["binding_id"],
            "excluded": True,
        },
    )
    assert (
        client.post("/api/fleet/commands", headers=HEADERS, json=message).status_code
        == 200
    )
    message.update(
        operation="unregister",
        request={
            "request_id": str(uuid4()),
            "expected_revision": row["revision"] + 1,
            "binding_id": row["binding_id"],
        },
    )
    assert (
        client.post("/api/fleet/commands", headers=HEADERS, json=message).status_code
        == 409
    )
    message["confirmed"] = {"empty_load": True, "removed_from_lane": True}
    fixture.control.observed["pinky1"]["state"]["stopped"] = False
    assert (
        client.post("/api/fleet/commands", headers=HEADERS, json=message).status_code
        == 409
    )
    fixture.control.observed["pinky1"]["state"]["stopped"] = True
    response = client.post("/api/fleet/commands", headers=HEADERS, json=message)
    assert response.status_code == 200, response.json()
    assert not next(
        r
        for r in response.json()["result"]["registry"]["robots"]
        if r["robot_id"] == "pinky1"
    )["registered"]
