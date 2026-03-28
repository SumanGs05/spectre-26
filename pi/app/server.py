"""
FastAPI web server for UAV-AudioLoc monitoring dashboard.

Serves the polar plot dashboard and provides a WebSocket endpoint
for real-time telemetry streaming. Runs on the Pi's WiFi hotspot
so any phone/tablet on the same network can monitor DOA results.
"""

import asyncio
import json
import logging
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

logger = logging.getLogger(__name__)

app = FastAPI(title="UAV-AudioLoc Dashboard")

STATIC_DIR = Path(__file__).parent / "static"

app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# Global reference to the telemetry bus (set by main.py at startup)
_telemetry_bus = None


def set_telemetry_bus(bus):
    """Called by the pipeline orchestrator to inject the telemetry bus."""
    global _telemetry_bus
    _telemetry_bus = bus


@app.get("/")
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.get("/api/status")
async def status():
    """Pipeline health check endpoint."""
    if _telemetry_bus is None:
        return {"status": "no_pipeline", "latest": None}

    latest = _telemetry_bus.latest
    return {
        "status": "running",
        "latest": latest.to_dict() if latest else None,
    }


@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    """
    WebSocket endpoint for real-time telemetry streaming.

    Clients receive JSON telemetry packets as they are produced
    by the DSP pipeline (~30 packets/sec).
    """
    await websocket.accept()

    if _telemetry_bus is None:
        await websocket.send_json({"error": "Pipeline not running"})
        await websocket.close()
        return

    queue = _telemetry_bus.register_ws_client()
    logger.info("Dashboard client connected")

    try:
        while True:
            try:
                msg = await asyncio.wait_for(
                    queue.get(), timeout=5.0
                )
                await websocket.send_text(msg)
            except asyncio.TimeoutError:
                # Send heartbeat to keep connection alive
                await websocket.send_json({"heartbeat": True})
    except WebSocketDisconnect:
        logger.info("Dashboard client disconnected")
    except Exception as e:
        logger.error("WebSocket error: %s", e)
    finally:
        _telemetry_bus.unregister_ws_client(queue)


def run_server(host: str = "0.0.0.0", port: int = 8080):
    """Start the dashboard server (blocking)."""
    import uvicorn
    uvicorn.run(app, host=host, port=port, log_level="info")
