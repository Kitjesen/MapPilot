"""Foreground video acceptance Gateway; owns no Product or device processes."""

from __future__ import annotations

import argparse
import asyncio
import os
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from gateway.gateway_module import GatewayModule
from gateway.media_access import MediaAccessMiddleware
from gateway.routes.video import register_video_routes
from gateway.services.app_bootstrap import build_app_bootstrap, build_app_capabilities
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.state_snapshot import build_state_snapshot


def create_app(probe_dir: Path | None = None) -> FastAPI:
    """Expose actual, unstarted Gateway state and the configured media backend."""
    for name in ("LINGTU_PRODUCT", "LINGTU_PRODUCT_SESSION_ID", "LINGTU_RUN_PLAN"):
        if os.environ.get(name):
            raise ValueError(f"video test requires an unmanaged environment: unset {name}")
    for name in ("LINGTU_ROBOT", "LINGTU_WEBRTC_URL", "LINGTU_VIDEO_STATUS_FILE"):
        if not os.environ.get(name):
            raise ValueError(f"video test requires {name}")

    # Construction supplies status projection only: never call setup() or start().
    gateway = GatewayModule(product="", product_session_id="")
    app = FastAPI(title="LingTu video acceptance Gateway", docs_url=None, redoc_url=None)
    gateway._app = app
    app.state.gateway = gateway
    app.add_middleware(
        MediaAccessMiddleware,
        required=os.environ.get("LINGTU_MEDIA_REQUIRE_TOKEN") == "1",
        viewer_token=os.environ.get("LINGTU_MEDIA_VIEWER_TOKEN"),
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=os.environ.get(
            "LINGTU_CORS_ORIGINS", "http://localhost:5050,http://127.0.0.1:5050"
        ).split(","),
        allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
        allow_headers=["*"],
        expose_headers=["Location", "ETag"],
    )

    def snapshot(payload: dict) -> dict:
        paths = {route.path for route in app.routes}
        payload["links"] = {
            name: path for name, path in payload["links"].items() if path in paths
        }
        flag_key = "features" if "features" in payload else "capabilities"
        if flag_key in payload:
            payload[flag_key] = {
                name: name in {"state", "whep"} for name in payload[flag_key]
            }
        if "endpoints" in payload:
            payload["endpoints"] = {
                group: {name: spec for name, spec in specs.items() if spec["path"] in paths}
                for group, specs in payload["endpoints"].items()
            }
            payload["probes"] = {}
            payload["realtime"] = {}
            payload["validation_gates"] = {}
        # These standard snapshots contain links to streams absent from this app.
        if "media" in payload:
            payload["media"] = {
                name: value for name, value in payload["media"].items()
                if not isinstance(value, str) or value in paths
            }
        if "traffic" in payload:
            payload.pop("traffic")
        if "robot" in payload:
            payload["robot"]["online"] = None
        return payload

    @app.get("/api/v1/app/bootstrap")
    async def bootstrap():
        return snapshot(await asyncio.to_thread(build_app_bootstrap, gateway))

    @app.get("/api/v1/app/capabilities")
    async def capabilities():
        return snapshot(await asyncio.to_thread(build_app_capabilities, gateway))

    @app.get("/api/v1/state")
    async def state():
        return snapshot(await asyncio.to_thread(build_state_snapshot, gateway))

    @app.get("/api/v1/readiness")
    async def readiness():
        payload, _ = build_readiness_snapshot(gateway)
        return payload

    register_video_routes(app, gateway)
    if probe_dir is not None:
        assets = {name: probe_dir / name for name in (
            "index.html", "probe.mjs", "probe-core.mjs", "whepReceiver.mjs",
        )}
        if not all(path.is_file() for path in assets.values()):
            raise ValueError("video probe directory must contain the four declared assets")

        @app.get("/video-probe/")
        async def probe_index():
            return FileResponse(assets["index.html"], media_type="text/html")

        @app.get("/video-probe/{asset}")
        async def probe_asset(asset: str):
            if asset not in assets:
                raise HTTPException(status_code=404)
            return FileResponse(
                assets[asset], media_type="text/html" if asset == "index.html" else "text/javascript"
            )

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5051, type=int)
    parser.add_argument("--probe-dir", type=Path)
    args = parser.parse_args()
    import uvicorn

    uvicorn.run(create_app(args.probe_dir), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
