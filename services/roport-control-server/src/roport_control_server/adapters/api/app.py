"""Web display and opt-in trial IPC; never owns MQTT or robot execution."""

import asyncio
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from ...contracts.pinky_settings import validate_job, validate_registration
from .display import ROBOTS, Display
from .maps import load_map
from .pinky_settings import NOT_CONNECTED, settings_view
from .trial import TrialView

MAX_JSON_BYTES = 4096

ASSETS = Path(__file__).resolve().parents[2] / "spatial_assets"


def create_app(
    map_path=None,
    *,
    layout_path=None,
    trial_socket=None,
    trial_layout=None,
    trial_token=None,
):
    # shortcut: per-worker demo limit; measure with AI load before scaling out.
    stream_slots = asyncio.BoundedSemaphore(8)

    @asynccontextmanager
    async def lifespan(app):
        # Only the server operator supplies a local path; no file-upload/path API.
        app.state.map = await asyncio.to_thread(
            load_map, map_path or ASSETS / "maps/demo.yaml", layout_path or trial_layout
        )
        app.state.example_map = map_path is None
        app.state.trial = (
            TrialView(trial_socket, trial_token, app.state.map)
            if trial_socket
            else None
        )
        yield

    app = FastAPI(title="RO:PORT control web display", lifespan=lifespan)

    @app.middleware("http")
    async def headers(request, call_next):
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["Referrer-Policy"] = "no-referrer"
        if request.url.path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/healthz")
    def healthz() -> dict[str, str]:
        """Process liveness only; does not indicate robot/model readiness."""
        return {"status": "ok"}

    @app.get("/", include_in_schema=False)
    def index():
        return RedirectResponse("/ui/")

    @app.get("/api/bootstrap")
    def bootstrap(
        mode: Literal["auto", "offline", "demo", "trial"] = "auto",
    ):
        if mode == "auto":
            mode = "trial" if app.state.trial else "offline"
        result = {
            "mode": mode,
            "map": app.state.map,
            "example_map": app.state.example_map,
            "demo": mode == "demo",
            "robots": ROBOTS,
        }
        if mode == "trial":
            if not app.state.trial:
                return JSONResponse({"detail": "trial UI is not configured"}, 503)
            try:
                result.update(app.state.trial.bootstrap())
            except (OSError, ValueError):
                return JSONResponse(
                    {"detail": "시험 실행부 연결·지도 설정을 확인하세요."}, 503
                )
        return result

    @app.get("/api/snapshot")
    def snapshot(mode: Literal["offline", "trial"] = "offline"):
        if mode == "trial":
            if not app.state.trial:
                return JSONResponse({"detail": "trial UI is not configured"}, 503)
            return app.state.trial.snapshot()
        return Display().snapshot()

    async def json_object(request):
        if request.headers.get("content-type", "").split(";")[0] != "application/json":
            return None, JSONResponse({"detail": "application/json required"}, 415)
        raw = bytearray()
        async for chunk in request.stream():
            if len(raw) + len(chunk) > MAX_JSON_BYTES:
                return None, JSONResponse({"detail": "body too large"}, 413)
            raw.extend(chunk)
        try:
            data = json.loads(raw)
        except (ValueError, RecursionError):
            return None, JSONResponse({"detail": "invalid JSON"}, 400)
        if not isinstance(data, dict):
            return None, JSONResponse({"detail": "JSON object required"}, 400)
        return data, None

    @app.get("/api/pinky/settings")
    def pinky_settings(mode: Literal["offline", "demo"] = "offline"):
        return settings_view(mode == "demo")

    @app.post("/api/pinky/settings/validate")
    async def pinky_settings_validate(request: Request):
        # DEMO edits stay in the browser; this only checks the shared contract.
        data, error = await json_object(request)
        if error:
            return error
        kind = data.get("kind")
        check = {"robot": validate_registration, "job": validate_job}.get(kind)
        if check is None:
            return JSONResponse({"detail": "kind must be robot or job"}, 400)
        try:
            return {"kind": kind, "value": check(data.get("value"))}
        except (ValueError, TypeError) as e:
            return JSONResponse({"detail": str(e)}, 400)

    @app.post("/api/pinky/settings/robots")
    @app.post("/api/pinky/settings/jobs")
    async def pinky_settings_write(request: Request):
        # Validate first so the contract is testable; storage/execution needs the executor.
        data, error = await json_object(request)
        if error:
            return error
        check = (
            validate_job
            if request.url.path.endswith("/jobs")
            else validate_registration
        )
        try:
            value = check(data)
        except (ValueError, TypeError) as e:
            return JSONResponse({"detail": str(e)}, 400)
        return JSONResponse({"detail": NOT_CONNECTED, "value": value}, 503)

    @app.post("/api/trial/commands")
    async def trial_command(request: Request):
        trial = app.state.trial
        if trial is None:
            return JSONResponse({"detail": "trial UI is not configured"}, 503)
        if (
            request.headers.get("origin")
            != f"{request.url.scheme}://{request.headers.get('host')}"
            or not trial.authorized(request.headers.get("x-roport-trial-token", ""))
        ):
            return JSONResponse(
                {"detail": "같은 화면에서 올바른 조작 키를 입력하세요."}, 403
            )
        data, error = await json_object(request)
        if error:
            return error
        try:
            await asyncio.to_thread(trial.command, data)
            return {"accepted": True}
        except ValueError as exc:
            return JSONResponse({"detail": str(exc)}, 409)
        except OSError:
            return JSONResponse(
                {"detail": "실행부 응답 없음 · 결과 미확인. 상태 확인 후 조작하세요."},
                503,
            )

    @app.websocket("/api/state")
    async def state(
        websocket: WebSocket, mode: Literal["offline", "demo", "trial"] = "offline"
    ):
        scheme = "https" if websocket.url.scheme == "wss" else "http"
        if (
            websocket.headers.get("origin")
            != f"{scheme}://{websocket.headers.get('host')}"
        ):
            await websocket.close(code=1008)
            return
        if stream_slots.locked():
            await websocket.close(code=1013)
            return
        if mode == "trial" and app.state.trial is None:
            await websocket.close(code=1008)
            return
        display = Display(demo=mode == "demo", map_geometry=app.state.map)

        async def send():
            while True:
                data = (
                    await asyncio.to_thread(app.state.trial.snapshot)
                    if mode == "trial"
                    else display.snapshot()
                )
                # No unbounded broadcast queue: slow clients time out independently.
                await asyncio.wait_for(websocket.send_json(data), 3)
                await asyncio.sleep(0.25 if mode == "trial" else 0.1)

        async def receive():
            while True:
                message = await websocket.receive()
                if message["type"] == "websocket.disconnect":
                    return
                if mode == "trial":
                    await websocket.close(code=1008)
                    return
                raw = message.get("text")
                if raw is None:
                    await websocket.close(code=1003)
                    return
                if len(raw.encode("utf-8")) > 1024:
                    await websocket.close(code=1009)
                    return
                try:
                    display.command(json.loads(raw))
                except (ValueError, TypeError, RecursionError):
                    await websocket.close(code=1008)
                    return
                await asyncio.sleep(0.05)

        # No await between the capacity check and acquisition on this event loop.
        async with stream_slots:
            await websocket.accept()
            try:
                async with asyncio.TaskGroup() as group:
                    tasks = [group.create_task(send()), group.create_task(receive())]
                    try:
                        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                    finally:
                        for task in tasks:
                            task.cancel()
            except* (WebSocketDisconnect, TimeoutError, OSError):
                pass

    app.mount("/ui", StaticFiles(directory=ASSETS, html=True), name="ui")
    return app


def create_app_from_env(environment):
    return create_app(
        environment.get("ROPORT_MAP"),
        layout_path=environment.get("ROPORT_MAP_LAYOUT"),
        trial_socket=environment.get("ROPORT_TRIAL_SOCKET"),
        trial_layout=environment.get("ROPORT_TRIAL_LAYOUT"),
        trial_token=environment.get("ROPORT_TRIAL_TOKEN"),
    )


app = create_app_from_env(os.environ)
