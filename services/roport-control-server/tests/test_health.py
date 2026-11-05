from fastapi.testclient import TestClient

from roport_control_server.adapters.api.app import app

client = TestClient(app)


def test_liveness():
    response = client.get("/healthz")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}
