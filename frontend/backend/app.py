"""FastAPI application for the missile-guidance web control terminal.

Serves the single-page UI (frontend/web) and a small JSON/WebSocket API that
bridges the browser to the simulation stack:

    GET  /api/dems                     list DEM tiles + bounds
    GET  /api/dems/{name}/grid         downsampled elevation grid
    GET  /api/dems/{name}/elevation    point elevation lookup
    GET  /api/profiles                 missile profiles
    GET  /api/missions                 recorded flights (summaries)
    GET  /api/missions/{id}            full telemetry time series + verdict
    GET  /api/results                  saved mission-result verdicts
    POST /api/plan                     run A* + spline, return trajectory
    WS   /ws/live                      drive the live simulation, stream frames
"""
from __future__ import annotations

import asyncio
import queue
import threading
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import catalog, dem_service
from .bootstrap import PROJECT_ROOT

WEB_DIR = PROJECT_ROOT / "frontend" / "web"

app = FastAPI(title="Missile Guidance — Web Control Terminal", version="1.0.0")


# --------------------------------------------------------------------------- #
# DEM / terrain
# --------------------------------------------------------------------------- #
@app.get("/api/dems")
def get_dems() -> list[dict]:
    return dem_service.list_dems()


@app.get("/api/dems/{name}/grid")
def get_dem_grid(name: str, max_size: int = 180):
    try:
        return dem_service.elevation_grid(name, max_size=max(32, min(max_size, 512)))
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc)) from exc


@app.get("/api/dems/{name}/elevation")
def get_dem_elevation(name: str, lat: float, lon: float):
    try:
        return {"lat": lat, "lon": lon, "elevation_m": dem_service.elevation_at(name, lat, lon)}
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc)) from exc


# --------------------------------------------------------------------------- #
# Catalog: profiles, recorded flights, results
# --------------------------------------------------------------------------- #
@app.get("/api/profiles")
def get_profiles() -> list[dict]:
    return catalog.list_profiles()


class ProfileRequest(BaseModel):
    name: str
    basic: dict
    detailed: dict | None = None
    warhead: dict | None = None


@app.post("/api/profiles")
def post_profile(req: ProfileRequest):
    """Create / overwrite a missile profile (data/missiles/<slug>.json)."""
    from . import bootstrap  # noqa: F401 - ensure src on path

    try:
        from missile.config_store import save_configuration

        config = {"name": req.name, "basic": req.basic, "detailed": req.detailed or {}}
        path = save_configuration(config)
        # config_store validates basic+detailed; preserve a warhead block if supplied.
        if req.warhead:
            import json
            saved = json.loads(path.read_text())
            saved["warhead"] = req.warhead
            path.write_text(json.dumps(saved, indent=2))
        return catalog.list_profiles()
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(400, str(exc)) from exc


@app.get("/api/missions")
def get_missions() -> list[dict]:
    return catalog.list_missions()


@app.get("/api/missions/{mission_id}")
def get_mission(mission_id: str):
    data = catalog.load_flightlog(mission_id)
    if data is None:
        raise HTTPException(404, f"No recorded flight: {mission_id}")
    return data


@app.get("/api/results")
def get_results() -> list[dict]:
    return catalog.list_results()


# --------------------------------------------------------------------------- #
# Planning
# --------------------------------------------------------------------------- #
class PlanRequest(BaseModel):
    dem_name: str
    start_gps: list[float]
    target_gps: list[float]
    heuristic_weight: float = 2.0


@app.post("/api/plan")
async def post_plan(req: PlanRequest):
    from . import planning

    try:
        # Heavy C++/numpy work: keep it off the event loop.
        return await asyncio.to_thread(
            planning.run_plan, req.dem_name, req.start_gps, req.target_gps, req.heuristic_weight
        )
    except Exception as exc:  # noqa: BLE001 - report cleanly to the UI
        raise HTTPException(400, str(exc)) from exc


# --------------------------------------------------------------------------- #
# Live simulation stream
# --------------------------------------------------------------------------- #
@app.websocket("/ws/live")
async def ws_live(ws: WebSocket):
    await ws.accept()
    try:
        spec = await ws.receive_json()
    except (WebSocketDisconnect, ValueError):
        await _safe_close(ws)
        return

    profile_name = spec.get("profile")
    config = spec.get("config") or {}
    loop = asyncio.get_running_loop()
    q: "queue.Queue[dict | None]" = queue.Queue(maxsize=512)
    stop = threading.Event()

    def worker() -> None:
        from . import live_runner

        def emit_log(msg: str) -> None:
            loop.call_soon_threadsafe(q.put_nowait, {"type": "log", "line": msg})

        try:
            for item in live_runner.iter_frames(
                profile_name, config, on_log=emit_log, should_stop=stop.is_set
            ):
                loop.call_soon_threadsafe(q.put_nowait, item)
        except Exception as exc:  # noqa: BLE001
            loop.call_soon_threadsafe(q.put_nowait, {"type": "error", "message": str(exc)})
        finally:
            loop.call_soon_threadsafe(q.put_nowait, None)

    thread = threading.Thread(target=worker, name="sim-worker", daemon=True)
    thread.start()

    async def watch_client() -> None:
        """A client message (or disconnect) requests a stop."""
        try:
            while True:
                await ws.receive_text()
        except Exception:  # noqa: BLE001
            stop.set()

    watcher = asyncio.create_task(watch_client())
    try:
        while True:
            item = await asyncio.to_thread(q.get)
            if item is None:
                break
            try:
                await ws.send_json(item)
            except Exception:  # noqa: BLE001 - client vanished mid-stream
                stop.set()
                break
    finally:
        stop.set()
        watcher.cancel()
        await _safe_close(ws)


async def _safe_close(ws: WebSocket) -> None:
    """Close a WebSocket without raising if it's already gone.

    Guards against double-close / post-close-send ASGI errors and a websockets
    version quirk that throws AttributeError from inside close().
    """
    try:
        await ws.close()
    except Exception:  # noqa: BLE001
        pass


# --------------------------------------------------------------------------- #
# Static UI (mounted last so /api and /ws win)
# --------------------------------------------------------------------------- #
@app.get("/healthz")
def healthz() -> JSONResponse:
    return JSONResponse({"ok": True})


class _NoCacheStaticFiles(StaticFiles):
    """Static files with revalidation forced.

    Without Cache-Control, browsers heuristically cache ES modules and keep
    serving stale JS after an edit; no-cache makes them revalidate (ETag) on
    every load while still allowing 304s.
    """

    def file_response(self, *args, **kwargs):
        resp = super().file_response(*args, **kwargs)
        resp.headers["Cache-Control"] = "no-cache"
        return resp


if WEB_DIR.exists():
    app.mount("/", _NoCacheStaticFiles(directory=str(WEB_DIR), html=True), name="web")
