import pytest
from fastapi.testclient import TestClient

from roport_control_server.adapters.api.app import create_app
from roport_control_server.contracts.pinky_settings import (
    validate_job,
    validate_registration,
)

GOOD = {
    "name": "pinky1",
    "ip": "192.0.2.11",
    "role": "logistics",
    "ros_domain_id": 14,
    "calibration": "pinky_d660",
}


@pytest.fixture
def client():
    with TestClient(create_app()) as client:
        yield client


def test_registration_normalized():
    assert validate_registration(dict(GOOD, ip=" 192.0.2.11 ")) == GOOD


@pytest.mark.parametrize(
    "field, value",
    [
        ("name", "Pinky 1"),
        ("name", "a" * 17),
        ("ip", "pinky.local"),
        ("ip", "192.0.2.11; reboot"),
        ("role", "omx"),
        ("ros_domain_id", 102),
        ("ros_domain_id", "14"),
        ("ros_domain_id", True),
        ("calibration", "d660"),
    ],
)
def test_registration_rejects(field, value):
    with pytest.raises(ValueError):
        validate_registration(dict(GOOD, **{field: value}))


def test_jobs():
    assert validate_job({"name": "pinky1", "action": "deploy", "release": True}) == {
        "name": "pinky1",
        "action": "deploy",
        "ref": "origin/main",
    }
    job = {"name": "pinky1", "action": "set_domain", "ros_domain_id": 16}
    assert validate_job(job)["ros_domain_id"] == 16
    for bad in (
        {"name": "pinky1", "action": "reboot"},
        {"name": "pinky1", "action": "deploy", "ref": "main"},
        {"name": "pinky1", "action": "deploy", "ref": "origin/../x"},
        {"name": "pinky1", "action": "deploy", "ref": "origin/a;ls"},
        {"name": "pinky1", "action": "set_domain", "ros_domain_id": -1},
    ):
        with pytest.raises(ValueError):
            validate_job(bad)


def test_offline_view_is_empty_and_not_connected(client):
    view = client.get("/api/pinky/settings").json()
    assert view["robots"] == [] and view["executor_connected"] is False
    assert view["release_ref"] == "origin/main" and view["domain_range"] == [0, 101]
    demo = client.get("/api/pinky/settings?mode=demo").json()
    assert [r["ros_domain_id"] for r in demo["robots"]] == [14, 15, 16]
    assert client.get("/api/pinky/settings?mode=live").status_code == 422


def test_writes_validate_then_report_executor_missing(client):
    r = client.post("/api/pinky/settings/robots", json=GOOD)
    assert r.status_code == 503 and r.json()["detail"] == "executor_not_connected"
    assert (
        client.post("/api/pinky/settings/robots", json={"name": "X"}).status_code == 400
    )
    job = {"name": "pinky1", "action": "build"}
    assert client.post("/api/pinky/settings/jobs", json=job).status_code == 503
    bad_job = {"name": "pinky1", "action": "rm"}
    assert client.post("/api/pinky/settings/jobs", json=bad_job).status_code == 400


def test_validate_endpoint_and_input_limits(client):
    r = client.post(
        "/api/pinky/settings/validate", json={"kind": "robot", "value": GOOD}
    )
    assert r.status_code == 200 and r.json()["value"] == GOOD
    assert (
        client.post(
            "/api/pinky/settings/validate", json={"kind": "x", "value": {}}
        ).status_code
        == 400
    )
    form = client.post(
        "/api/pinky/settings/robots",
        content=b"name=pinky1",
        headers={"content-type": "application/x-www-form-urlencoded"},
    )
    assert form.status_code == 415  # CSRF-style form posts are refused
    big = client.post(
        "/api/pinky/settings/robots",
        content=b'{"name": "' + b"a" * 5000 + b'"}',
        headers={"content-type": "application/json"},
    )
    assert big.status_code == 413
    arr = client.post("/api/pinky/settings/robots", json=[GOOD])
    assert arr.status_code == 400
