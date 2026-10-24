from fastapi.testclient import TestClient

from roport_ai_server.app import app

client = TestClient(app)


def test_liveness():
    response = client.get("/healthz")
    assert response.status_code == 200
    assert response.json() == {"status": "ok"}


def test_model_not_ready():
    response = client.get("/readyz")
    assert response.status_code == 503
    assert response.json()["reason"] == "model_not_configured"
