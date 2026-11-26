"""Web display and opt-in trial IPC; never owns MQTT or robot execution."""

import asyncio
import json
import os
from contextlib import asynccontextmanager
from ipaddress import ip_address
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from ...contracts.pinky_settings import validate_job, validate_registration
from ...contracts.robots import HOST_IDS, unknown_host
from .display import ROBOTS, Display
from .fleet import FleetView
from .follow_frames import FollowFrames
from .maps import load_map
from .pinky_settings import NOT_CONNECTED, settings_view
from .trial import TrialView

MAX_JSON_BYTES = 4096

ASSETS = Path(__file__).resolve().parents[2] / "spatial_assets"


def loopback(host):
    try:
        return ip_address(host).is_loopback
    except ValueError:
        return False


def create_app(
    map_path=None,
    *,
    layout_path=None,
    trial_socket=None,
    trial_layout=None,
    trial_token=None,
    virtual_reset_path=None,
    fleet_socket=None,
):
    # shortcut: per-worker demo limit; measure with AI load before scaling out.
    stream_slots = asyncio.BoundedSemaphore(8)
    joint_slots = asyncio.BoundedSemaphore(8)
    joint_lock = asyncio.Lock()
    joint_next_read = 0.0
    joint_frame = {"available": False, "arms": {}}

    async def joint_snapshot():
        # Share a compact owner read across viewers. A failed display read has
        # its own retry interval and never affects the full fleet stream.
        nonlocal joint_next_read, joint_frame
        loop = asyncio.get_running_loop()
        async with joint_lock:
            started = loop.time()
            if started >= joint_next_read:
                try:
                    arms = await asyncio.to_thread(
                        app.state.fleet.rpc, {"action": "omx_joints"}
                    )
                    joint_frame = {"available": True, "arms": arms}
                    joint_next_read = started + 0.1
                except (OSError, ValueError):
                    joint_frame = {"available": False, "arms": {}}
                    joint_next_read = loop.time() + 1
            return joint_frame

    @asynccontextmanager
    async def lifespan(app):
        # Only the server operator supplies a local path; no file-upload/path API.
        app.state.map = await asyncio.to_thread(
            load_map, map_path or ASSETS / "maps/demo.yaml", layout_path or trial_layout
        )
        app.state.example_map = map_path is None
        app.state.trial = (
            TrialView(
                trial_socket,
                trial_token,
                app.state.map,
                virtual_reset_path=virtual_reset_path,
            )
            if trial_socket
            else None
        )
        app.state.fleet = (
            FleetView(fleet_socket, trial_token, app.state.map)
            if fleet_socket
            else None
        )
        yield

    app = FastAPI(title="RO:PORT control web display", lifespan=lifespan)
    frames = FollowFrames(
        os.environ.get("ROPORT_FOLLOW_PREVIEW_TOKEN", ""),
        os.environ.get("ROPORT_FOLLOW_FRAME_SOURCES", "{}"),
        os.environ.get("ROPORT_FOLLOW_FRAME_TOKEN", ""),
        ca_file=os.environ.get("ROPORT_FOLLOW_FRAME_CA_FILE"),
    )

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
        mode: Literal["auto", "offline", "demo", "trial", "fleet"] = "auto",
    ):
        if mode == "auto":
            mode = (
                "fleet"
                if app.state.fleet
                else "trial"
                if app.state.trial
                else "offline"
            )
        result = {
            "mode": mode,
            "map": app.state.map,
            "example_map": app.state.example_map,
            "demo": mode == "demo",
            "robots": ROBOTS,
        }
        if mode == "fleet":
            if not app.state.fleet:
                return JSONResponse({"detail": "fleet UI is not configured"}, 503)
            try:
                result.update(app.state.fleet.bootstrap())
                result["omx_joint_stream"] = True
            except (OSError, ValueError):
                return JSONResponse({"detail": "fleet owner/map unavailable"}, 503)
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
    def snapshot(mode: Literal["offline", "trial", "fleet"] = "offline"):
        if mode == "fleet":
            if not app.state.fleet:
                return JSONResponse({"detail": "fleet UI is not configured"}, 503)
            return app.state.fleet.snapshot()
        if mode == "trial":
            if not app.state.trial:
                return JSONResponse({"detail": "trial UI is not configured"}, 503)
            return app.state.trial.snapshot()
        return Display().snapshot()

    @app.get("/api/hosts/metrics")
    async def host_metrics():
        """Display-only host telemetry; always 200, never device readiness."""
        if app.state.trial is None:
            return {
                "receiver": "not_configured",
                "rejected": {},
                "hosts": [unknown_host(host) for host in HOST_IDS],
            }
        return await asyncio.to_thread(app.state.trial.host_metrics)

    async def json_object(request, max_bytes=MAX_JSON_BYTES):
        if request.headers.get("content-type", "").split(";")[0] != "application/json":
            return None, JSONResponse({"detail": "application/json required"}, 415)
        raw = bytearray()
        async for chunk in request.stream():
            if len(raw) + len(chunk) > max_bytes:
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

    @app.post("/api/fleet/commands")
    async def fleet_command(request: Request):
        return await operator_command(request, app.state.fleet)

    @app.get("/api/fleet/follow/{session_id}/frames/{frame_ref}")
    def follow_frame(request: Request, session_id: str, frame_ref: str):
        return frames.get(
            app.state.fleet,
            request.headers.get("authorization", ""),
            session_id,
            frame_ref,
        )

    @app.post("/api/trial/commands")
    async def trial_command(request: Request):
        return await operator_command(request, app.state.trial)

    async def operator_command(request, trial):
        if trial is None:
            return JSONResponse(
                {"detail": "requested control UI is not configured"}, 503
            )
        if (
            request.headers.get("origin")
            != f"{request.url.scheme}://{request.headers.get('host')}"
            or not trial.authorized(request.headers.get("x-roport-trial-token", ""))
        ):
            return JSONResponse(
                {"detail": "같은 화면에서 올바른 조작 키를 입력하세요."}, 403
            )
        data, error = await json_object(request, 32768)
        if error:
            return error
        try:
            result = await asyncio.to_thread(
                trial.command,
                data,
                secure_transport=request.url.scheme == "https"
                or (
                    request.client is not None
                    and loopback(request.client.host)
                    and (
                        request.url.hostname == "localhost"
                        or loopback(request.url.hostname)
                    )
                ),
            )
            return {"accepted": True, "result": result}
        except PermissionError as exc:
            return JSONResponse({"detail": str(exc)}, 403)
        except ValueError as exc:
            return JSONResponse({"detail": str(exc)}, 409)
        except OSError:
            return JSONResponse(
                {
                    "detail": "정지 요청 응답 없음 · 정지 여부 미확인. 현장 정지 수단으로 기체 정지를 확인하세요."
                    if data.get("action") == "stop"
                    else "실행부 응답 없음 · 결과 미확인. 상태 확인 후 조작하세요."
                },
                503,
            )

    @app.websocket("/api/state")
    async def state(
        websocket: WebSocket,
        mode: Literal["offline", "demo", "trial", "fleet", "omx_joints"] = "offline",
    ):
        scheme = "https" if websocket.url.scheme == "wss" else "http"
        if (
            websocket.headers.get("origin")
            != f"{scheme}://{websocket.headers.get('host')}"
        ):
            await websocket.close(code=1008)
            return
        slots = joint_slots if mode == "omx_joints" else stream_slots
        if slots.locked():
            await websocket.close(code=1013)
            return
        if mode == "trial" and app.state.trial is None:
            await websocket.close(code=1008)
            return
        if mode in {"fleet", "omx_joints"} and app.state.fleet is None:
            await websocket.close(code=1008)
            return
        display = Display(demo=mode == "demo", map_geometry=app.state.map)

        async def send():
            while True:
                if mode == "omx_joints":
                    data = await joint_snapshot()
                elif mode in {"trial", "fleet"}:
                    data = await asyncio.to_thread(
                        (
                            app.state.fleet if mode == "fleet" else app.state.trial
                        ).snapshot
                    )
                else:
                    data = display.snapshot()
                # No unbounded broadcast queue: slow clients time out independently.
                await asyncio.wait_for(websocket.send_json(data), 3)
                if mode == "omx_joints":
                    # Readers share a deadline so IPC and send time count toward 100 ms.
                    await asyncio.sleep(
                        max(0, joint_next_read - asyncio.get_running_loop().time())
                    )
                else:
                    await asyncio.sleep(0.25 if mode in {"trial", "fleet"} else 0.1)

        async def receive():
            while True:
                message = await websocket.receive()
                if message["type"] == "websocket.disconnect":
                    return
                if mode in {"trial", "fleet", "omx_joints"}:
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
        async with slots:
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
        virtual_reset_path=environment.get("ROPORT_VIRTUAL_RESET_FILE"),
        fleet_socket=environment.get("ROPORT_FLEET_SOCKET"),
    )


app = create_app_from_env(os.environ)
