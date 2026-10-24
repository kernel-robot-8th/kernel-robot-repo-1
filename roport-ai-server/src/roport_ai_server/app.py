from fastapi import FastAPI

app = FastAPI(title="roport-ai-server")


@app.get("/healthz")
def healthz() -> dict[str, str]:
    """Process liveness only; does not indicate robot/model readiness."""
    return {"status": "ok"}


@app.get("/readyz", status_code=503)
def readyz() -> dict[str, str]:
    return {"status": "not_ready", "reason": "model_not_configured"}
