"""Authentication and validation routes for GatewayModule."""

from __future__ import annotations

from typing import Any

from gateway.schemas import AuthCheckResponse, AuthLoginRequest, AuthLoginResponse


try:
    from fastapi import Request as FastAPIRequest
except ImportError:  # FastAPI remains optional until routes are registered.
    FastAPIRequest = Any


_AUTH_PATHS = frozenset({"/api/v1/auth/login", "/api/v1/auth/check"})


def register_auth_routes(app) -> None:
    from fastapi.exceptions import RequestValidationError
    from fastapi.responses import JSONResponse

    app.router.routes[:] = [
        route
        for route in app.router.routes
        if getattr(route, "path", "") not in _AUTH_PATHS
    ]

    @app.post(
        "/api/v1/auth/login",
        summary="Login with API key",
        response_model=AuthLoginResponse,
        responses={403: {"model": AuthLoginResponse}},
    )
    async def auth_login(payload: AuthLoginRequest):
        key = payload.key
        from gateway.auth import _get_configured_key

        configured = _get_configured_key()
        if not configured:
            return JSONResponse({"ok": True, "message": "\u8ba4\u8bc1\u672a\u542f\u7528"})

        import hashlib
        import hmac as _hmac

        if _hmac.compare_digest(
            hashlib.sha256(key.encode()).hexdigest(),
            hashlib.sha256(configured.encode()).hexdigest(),
        ):
            resp = JSONResponse({"ok": True, "message": "\u767b\u5f55\u6210\u529f"})
            resp.set_cookie("lingtu_api_key", key, httponly=True, samesite="lax", max_age=86400 * 30)
            return resp
        return JSONResponse({"ok": False, "message": "Key \u65e0\u6548"}, status_code=403)

    @app.get(
        "/api/v1/auth/check",
        summary="Check if auth is required",
        response_model=AuthCheckResponse,
    )
    async def auth_check():
        from gateway.auth import _get_configured_key

        configured = _get_configured_key()
        return {"auth_required": configured is not None}

    @app.exception_handler(RequestValidationError)
    async def validation_error(req: FastAPIRequest, exc: RequestValidationError):
        return JSONResponse(
            status_code=422,
            content={"error": "validation_error", "detail": exc.errors()},
        )
