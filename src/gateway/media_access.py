"""Opt-in live-media viewing access; independent from motion and MCP credentials."""

from __future__ import annotations

import hmac

from starlette.responses import JSONResponse


def media_access_error(*, required: bool, viewer_token: bytes | None, authorization: list[bytes]):
    if not required:
        return None
    if not viewer_token:
        return 503, "media_access_unconfigured"
    value = authorization[0] if len(authorization) == 1 else b""
    scheme, _, token = value.partition(b" ")
    if scheme.lower() != b"bearer" or not token:
        return 401, "media_authorization_required"
    if not hmac.compare_digest(token, viewer_token):
        return 403, "media_authorization_denied"
    return None


class MediaAccessMiddleware:
    """A shared viewing credential admits new live-media requests only.

    It does not identify individual Apps, revoke established peers, or protect
    recordings. Browser camera WebSockets need a client capable of a Bearer
    header when this policy is enabled; query strings and cookies are ignored.
    """

    def __init__(self, app, *, required: bool = False, viewer_token: str | None = None):
        self.app = app
        self.required = required
        self.token = viewer_token.encode("utf-8") if viewer_token else None

    async def __call__(self, scope, receive, send):
        if not self.required or scope["type"] not in {"http", "websocket"}:
            return await self.app(scope, receive, send)

        path = scope.get("path", "")
        protected = path.startswith(("/api/v1/camera/", "/api/v1/webrtc/")) or path == "/ws/camera"
        if not protected or scope.get("method") == "OPTIONS":
            return await self.app(scope, receive, send)

        failure = media_access_error(
            required=self.required,
            viewer_token=self.token,
            authorization=[value for name, value in scope.get("headers", []) if name.lower() == b"authorization"],
        )
        if failure is None:
            return await self.app(scope, receive, send)
        status, error = failure

        if scope["type"] == "websocket":
            return await send({"type": "websocket.close", "code": 4401 if status == 401 else 4403})
        response = JSONResponse(
            {"error": error},
            status_code=status,
            headers={"Cache-Control": "no-store", "WWW-Authenticate": 'Bearer realm="lingtu-media"'},
        )
        await response(scope, receive, send)
