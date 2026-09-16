"""FastAPI app assembly for GatewayModule.

GatewayModule owns runtime ports and state.  This module owns HTTP app
construction, middleware, and route registration.
"""

from __future__ import annotations

import asyncio
import logging
import os
import time
from typing import Any

logger = logging.getLogger(__name__)


def build_gateway_app(gw: Any):
    try:
        from fastapi import FastAPI
        from fastapi.encoders import jsonable_encoder
        from fastapi.exceptions import RequestValidationError
        from fastapi.middleware.cors import CORSMiddleware
        from fastapi.responses import JSONResponse
    except ImportError:
        logger.error("FastAPI not installed -run: pip install fastapi uvicorn")
        return None

    cors_origins = os.environ.get(
        "LINGTU_CORS_ORIGINS",
        "http://localhost:5050,http://127.0.0.1:5050",
    ).split(",")
    app = FastAPI(
        title="LingTu Gateway",
        version="2.0",
        docs_url="/docs",
        redoc_url="/redoc",
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_origins,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    from gateway.maps.locations import register_location_routes
    from gateway.maps.places import register_place_routes
    from gateway.maps.routes import register_map_routes
    from gateway.navigation.diagnostics import register_navigation_diagnostic_routes
    from gateway.navigation.routes import register_navigation_routes
    from gateway.routes import (
        mount_dashboard_assets,
        register_app_routes,
        register_asset_routes,
        register_camera_routes,
        register_command_routes,
        register_diagnostic_routes,
        register_health_routes,
        register_inspection_routes,
        register_operation_routes,
        register_realtime_routes,
        register_recording_routes,
        register_safety_routes,
        register_session_routes,
        register_status_routes,
    )
    from gateway.services.rate_limit import RateLimitMiddleware

    app.add_middleware(RateLimitMiddleware)

    # ------------------------------------------------------------------
    # Global exception handlers - structured JSON errors for all routes
    # ------------------------------------------------------------------

    @app.exception_handler(RequestValidationError)
    async def _validation_error_handler(request, exc):
        return JSONResponse(
            status_code=422,
            content=jsonable_encoder({"error": "validation_error", "detail": exc.errors()}),
        )

    @app.exception_handler(Exception)
    async def _unhandled_exception_handler(request, exc):
        """Catch-all handler: log the traceback and return a safe JSON body."""
        logger.error(
            "Unhandled exception on %s %s: %s",
            request.method,
            request.url.path,
            exc,
            exc_info=True,
        )
        return JSONResponse(
            status_code=500,
            content={
                "error": "internal_server_error",
                "message": "An unexpected error occurred. Check gateway logs.",
                "path": str(request.url.path),
            },
        )

    @app.exception_handler(asyncio.TimeoutError)
    async def _timeout_exception_handler(request, exc):
        """Return 504 when a request exceeds its time budget."""
        logger.warning(
            "Request timeout on %s %s",
            request.method,
            request.url.path,
        )
        return JSONResponse(
            status_code=504,
            content={
                "error": "gateway_timeout",
                "message": "The request exceeded its time budget.",
                "path": str(request.url.path),
            },
        )

    # ------------------------------------------------------------------
    # Request timing middleware - adds X-Response-Time header
    # ------------------------------------------------------------------

    @app.middleware("http")
    async def _timing_middleware(request, call_next):
        start = time.perf_counter()
        response = await call_next(request)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        response.headers["X-Response-Time-Ms"] = f"{elapsed_ms:.1f}"
        return response

    register_operation_routes(app, gw)
    register_recording_routes(app, gw)
    register_diagnostic_routes(app, gw)
    register_map_routes(app, gw)
    register_location_routes(app, gw)
    register_place_routes(app, gw)
    register_status_routes(app, gw)
    register_health_routes(app, gw)
    register_session_routes(app, gw)
    register_command_routes(app, gw)
    register_navigation_routes(app, gw)
    register_navigation_diagnostic_routes(app, gw)
    register_inspection_routes(app, gw)
    register_safety_routes(app, gw)

    register_app_routes(app, gw)
    register_asset_routes(app)
    register_camera_routes(app, gw)
    register_realtime_routes(app, gw)
    mount_dashboard_assets(app)

    return app
