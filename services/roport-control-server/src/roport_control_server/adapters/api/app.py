"""Headless web display API. No DB, executor, MQTT or UDP connections at startup."""

import asyncio
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles

from .display import ROBOTS, Display
from .maps import load_map

ASSETS = Path(__file__).resolve().parents[2] / "spatial_assets"


def create_app(map_path=None):
    @asynccontextmanager
    async def lifespan(app):
        # Only the server operator supplies a local path; no file-upload/path API.
        app.state.map = await asyncio.to_thread(
            load_map, map_path or ASSETS / "maps/demo.yaml"
        )
        app.state.example_map = map_path is None
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
    def bootstrap(mode: Literal["offline", "demo"] = "offline"):
        return {
            "map": app.state.map,
            "example_map": app.state.example_map,
            "demo": mode == "demo",
            "robots": ROBOTS,
        }

    @app.get("/api/snapshot")
    def snapshot():
        return Display().snapshot()

    @app.websocket("/api/state")
    async def state(websocket: WebSocket, mode: Literal["offline", "demo"] = "offline"):
        scheme = "https" if websocket.url.scheme == "wss" else "http"
        if (
            websocket.headers.get("origin")
            != f"{scheme}://{websocket.headers.get('host')}"
        ):
            await websocket.close(code=1008)
            return
        await websocket.accept()
        display = Display(demo=mode == "demo")

        async def send():
            while True:
                # No unbounded broadcast queue: slow clients time out independently.
                await asyncio.wait_for(websocket.send_json(display.snapshot()), 3)
                await asyncio.sleep(0.1)

        async def receive():
            while True:
                message = await websocket.receive()
                if message["type"] == "websocket.disconnect":
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

        tasks = [asyncio.create_task(send()), asyncio.create_task(receive())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, TimeoutError, OSError):
            pass
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    app.mount("/ui", StaticFiles(directory=ASSETS, html=True), name="ui")
    return app


app = create_app(os.environ.get("ROPORT_MAP"))
