"""FastAPI app assembly for GatewayModule.

GatewayModule owns runtime ports and state.  This module owns HTTP app
construction, middleware, and route registration.
"""

from __future__ import annotations

import logging
import os
from typing import Any

from gateway.schemas import (
    DriverSwapRequest,
    DriverSwapResponse,
    GatewayErrorResponse,
    MapLifecycleResponse,
    MapRequest,
)

logger = logging.getLogger(__name__)


def build_gateway_app(gw: Any):
    try:
        from fastapi import FastAPI
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

    from gateway.auth import APIKeyMiddleware, gateway_api_key_required
    from gateway.routes import (
        map_lifecycle_payload,
        mount_dashboard_assets,
        register_app_routes,
        register_auth_routes,
        register_camera_routes,
        register_command_routes,
        register_diagnostic_routes,
        register_inspection_routes,
        register_map_routes,
        register_operation_routes,
        register_place_routes,
        register_realtime_routes,
        register_session_routes,
        register_status_routes,
        register_voice_routes,
    )

    app.add_middleware(
        APIKeyMiddleware,
        require_key=gateway_api_key_required(),
    )

    @app.post("/api/v1/runtime/backend")
    async def post_runtime_backend(body: dict[str, Any]):
        category = str(body.get("category", ""))
        backend = str(body.get("backend", ""))
        config = body.get("config") or {}
        if not category or not backend:
            return {
                "ok": False,
                "reason": "missing_category_or_backend",
                "category": category,
                "requested_backend": backend,
            }
        if not isinstance(config, dict):
            return {
                "ok": False,
                "category": category,
                "requested_backend": backend,
                "reason": "invalid_config",
            }
        return gw.reconfigure_backend(category, backend, **config)

    @app.post(
        "/api/v1/driver/swap",
        summary="Swap the active driver backend at runtime",
        response_model=DriverSwapResponse,
        responses={
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_driver_swap(body: DriverSwapRequest):
        result = gw._on_driver_swap(body.model_dump())
        if not result.get("success"):
            return JSONResponse(
                status_code=503,
                content=GatewayErrorResponse(
                    error=result.get("message", "driver swap failed"),
                    message=result.get("message", "driver swap failed"),
                    detail=result,
                ).model_dump(),
            )
        return DriverSwapResponse(
            success=True,
            message=result.get("message", f"Swapped to {body.driver}"),
            swap_time_ms=result.get("swap_time_ms", 0.0),
            driver=body.driver,
            detail=result.get("detail"),
        )

    register_operation_routes(app, gw)
    register_diagnostic_routes(app, gw)
    register_map_routes(app, gw)
    register_place_routes(app, gw)
    register_status_routes(app, gw)
    register_session_routes(app, gw)
    register_command_routes(app, gw)
    register_inspection_routes(app, gw)
    register_voice_routes(app, gw)

    @app.post(
        "/api/v1/maps",
        summary="Map lifecycle management",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_maps(body: MapRequest):
        action = {
            "use": "set_active",
            "build": "build_octomap",
        }.get(body.action, body.action)
        cmd = {"action": action}
        if body.name:
            cmd["name"] = body.name
        if body.new_name:
            cmd["new_name"] = body.new_name
        try:
            from gateway.services.map_service import map_service_command

            resp = map_service_command(gw, cmd)
        except RuntimeError as exc:
            message = str(exc)
            return JSONResponse(
                status_code=503,
                content=GatewayErrorResponse(error=message, message=message).model_dump(),
            )
        if not resp.get("success"):
            message = str(resp.get("message") or "failed")
            return JSONResponse(
                status_code=400,
                content=GatewayErrorResponse(
                    error=message,
                    message=message,
                    detail=resp,
                ).model_dump(),
            )
        legacy = dict(resp)
        legacy.pop("success", None)
        return map_lifecycle_payload(True, **legacy)

    register_auth_routes(app)
    register_app_routes(app, gw)
    register_camera_routes(app, gw)
    register_realtime_routes(app, gw)
    mount_dashboard_assets(app)
    try:
        from gateway.services.app_bootstrap import prewarm_app_capability_contracts

        prewarm_app_capability_contracts(gw)
    except Exception:
        logger.debug(
            "GatewayModule: App/Web capability contract prewarm failed",
            exc_info=True,
        )

    return app
