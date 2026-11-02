from fastapi import FastAPI

app = FastAPI(title="roport-control-server")


@app.get("/healthz")
def healthz() -> dict[str, str]:
    """Process liveness only; does not indicate robot/model readiness."""
    return {"status": "ok"}
