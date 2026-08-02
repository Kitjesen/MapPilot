"""API Key authentication middleware for GatewayModule.

Implemented as a **pure ASGI middleware** (not ``BaseHTTPMiddleware``) so
that it works cleanly with both HTTP and WebSocket scopes. Starlette's
``BaseHTTPMiddleware`` has known issues with WebSocket upgrades in some
versions (returns HTTP 403 before the WebSocket handler is reached, see
https://github.com/encode/starlette/issues/1012 and related).

Security model:
  - API key set via ``LINGTU_API_KEY`` env var or ``robot_config.yaml``
  - Optional route-scoped Open-RMF key via ``LINGTU_RMF_API_KEY``
  - If no key configured, auth is disabled by default (dev/testing mode)
  - If ``require_key=True`` and no key is configured, protected paths fail closed
  - Protected: ``/api/*``, ``/ws/*``, ``/mcp``
  - Public: ``/``, ``/docs``, ``/redoc``, ``/openapi.json``, static assets,
    ``/api/v1/auth/login``, ``/api/v1/auth/check``

Clients send the key as:
  - Header: ``X-API-Key: <key>``
  - Header: ``Authorization: Bearer <key>`` (service-to-service clients)
  - Query param: ``?api_key=<key>`` (for WebSocket / SSE where setting
    headers is inconvenient)
  - Cookie: ``lingtu_api_key=<key>`` (set by the login page, HTTP only)

⚠️ Danger: Query parameter and Cookie auth both expose the API key to
``/?api_key=...`` referrer leakage, browser history, and server access logs.
Cookie auth additionally leaks via CORS preflight if the gateway allows
``credentials: include`` from untrusted origins — the browser sends the
cookie even on cross-origin requests when CORS is misconfigured.
**Prefer X-API-Key header on LAN/WAN. Disable query-param auth in
production** by not documenting /login to external users and by
redacting ``api_key`` from access logs at the reverse proxy level.

On auth failure:
  - HTTP → JSON response with 401/403
  - WebSocket → ``websocket.close`` with code 4401/4403
"""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import logging
import os
import re
from urllib.parse import parse_qs

logger = logging.getLogger(__name__)


class _RedactAPIKeyFilter(logging.Filter):
    """Logging filter that redacts ``api_key=<value>`` from all log records.

    This prevents accidental leakage of the API key via log messages that
    contain the full URL or query string.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        if hasattr(record, "msg") and isinstance(record.msg, str):
            record.msg = _redact_api_key(record.msg)
        if hasattr(record, "args") and record.args:
            redacted = tuple(_redact_api_key(str(a)) if isinstance(a, str) else a for a in record.args)
            record.args = redacted
        return True


logger.addFilter(_RedactAPIKeyFilter())


def _redact_api_key(value: str) -> str:
    """Redact the ``api_key`` query parameter from a URL or query string.

    Returns the input with any ``api_key=<secret>`` replaced by
    ``api_key=REDACTED``.  Safe to call on strings that do not contain a
    query parameter — they pass through unchanged.
    """
    import re

    return re.sub(
        r"(?i)(api_key=)[^&\s]+",
        r"\1REDACTED",
        value,
    )


# Paths that never require auth
_PUBLIC_PREFIXES = ("/docs", "/redoc", "/openapi.json", "/favicon")
_PUBLIC_EXACT = {"/", "/api/v1/auth/login", "/api/v1/auth/check"}
_LOOPBACK_PUBLIC_EXACT = {"/health", "/ready"}
_PRODUCT_SESSION_ALLOWED_ROUTES = frozenset(
    {
        ("POST", "/api/v1/explore/start"),
        ("GET", "/api/v1/explore/status"),
    }
)
_MAP_ALLOWED_STATIC_ROUTES = frozenset(
    {
        ("GET", "/api/v1/slam/maps"),
        ("POST", "/api/v1/map/save"),
        ("GET", "/api/v1/maps/operations"),
    }
)
_MAP_OPERATION_PATH_RE = re.compile(
    r"^/api/v1/maps/operations/(?P<operation_id>[A-Za-z0-9_][A-Za-z0-9_-]{0,127})(?P<action>/(?:cancel|retry))?$"
)
_MAP_PCD_PATH_RE = re.compile(r"^/api/v1/maps/(?P<map_name>[A-Za-z0-9][A-Za-z0-9_.-]{0,99})/pcd$")
_RMF_ALLOWED_ROUTES = frozenset(
    {
        ("GET", "/api/v1/session"),
        ("GET", "/api/v1/navigation/status"),
        ("GET", "/api/v1/navigation/dds_snapshot"),
        ("POST", "/api/v1/lease"),
        ("POST", "/api/v1/goal"),
        ("POST", "/api/v1/navigation/cancel"),
    }
)


def _map_key_allows(method: str, path: str) -> bool:
    if (method, path) in _MAP_ALLOWED_STATIC_ROUTES:
        return True
    match = _MAP_OPERATION_PATH_RE.fullmatch(path)
    if match is not None:
        action = match.group("action")
        return (method == "GET" and action is None) or (method == "POST" and action in {"/cancel", "/retry"})
    match = _MAP_PCD_PATH_RE.fullmatch(path)
    return bool(method == "GET" and match is not None and ".." not in match.group("map_name"))


def _get_configured_key() -> str | None:
    """Read API key from env or config. Returns None if auth is disabled."""
    key = os.environ.get("LINGTU_API_KEY")
    if key:
        return key
    try:
        from runtime.config import get_config

        cfg = get_config()
        key = cfg.raw.get("gateway", {}).get("api_key")
        if key:
            return str(key)
    except Exception as e:
        logger.debug("_get_configured_key: failed to read api_key from config: %s", e)
    return None


def gateway_api_key_required() -> bool:
    """Return whether the main Gateway must fail closed without an API key."""
    env = os.environ.get("LINGTU_ENV", "").strip().lower()
    if env == "real":
        return True
    value = os.environ.get("LINGTU_GATEWAY_REQUIRE_API_KEY", "").strip().lower()
    return value in {"1", "true", "yes", "on"}


def _is_loopback_client(scope) -> bool:
    client = scope.get("client")
    if not client:
        return False
    host = str(client[0]).strip()
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


class APIKeyMiddleware:
    """Pure ASGI middleware for API key authentication.

    Works with both HTTP and WebSocket scopes. When auth is disabled (no
    configured key), every request passes through unchanged unless
    ``require_key`` is set.
    """

    def __init__(
        self,
        app,
        api_key: str | None = None,
        rmf_api_key: str | None = None,
        map_api_key: str | None = None,
        product_session_id: str | None = None,
        require_key: bool = False,
    ):
        self.app = app
        self._key = api_key or _get_configured_key()
        self._rmf_key = rmf_api_key or os.environ.get("LINGTU_RMF_API_KEY")
        self._map_key = map_api_key or os.environ.get("LINGTU_MAP_API_KEY")
        self._product_session_id = (
            product_session_id or os.environ.get("LINGTU_PRODUCT_SESSION_ID") or ""
        ).strip()
        if self._key and self._rmf_key and hmac.compare_digest(self._key, self._rmf_key):
            raise ValueError("LINGTU_RMF_API_KEY must differ from the operator API key")
        if self._map_key and self._key and hmac.compare_digest(self._map_key, self._key):
            raise ValueError("LINGTU_MAP_API_KEY must differ from the operator API key")
        if self._map_key and self._rmf_key and hmac.compare_digest(self._map_key, self._rmf_key):
            raise ValueError("LINGTU_MAP_API_KEY must differ from LINGTU_RMF_API_KEY")
        self._require_key = bool(require_key)
        if self._key:
            self._key_hash: str | None = hashlib.sha256(self._key.encode()).hexdigest()
            logger.info(
                "API key auth enabled (key hash: %s...)",
                self._key_hash[:8],
            )
        else:
            self._key_hash = None
            logger.info("API key auth disabled (no LINGTU_API_KEY set)")
        if self._rmf_key:
            self._rmf_key_hash: str | None = hashlib.sha256(self._rmf_key.encode()).hexdigest()
            logger.info(
                "Scoped Open-RMF API key enabled (key hash: %s...)",
                self._rmf_key_hash[:8],
            )
        else:
            self._rmf_key_hash = None
        if self._map_key:
            self._map_key_hash: str | None = hashlib.sha256(self._map_key.encode()).hexdigest()
            logger.info(
                "Scoped map API key enabled (key hash: %s...)",
                self._map_key_hash[:8],
            )
        else:
            self._map_key_hash = None
        self._product_session_hash = (
            hashlib.sha256(self._product_session_id.encode()).hexdigest()
            if self._product_session_id
            else None
        )

    async def __call__(self, scope, receive, send):
        # Lifespan and other scope types: pass through.
        """Authenticate one HTTP or WebSocket request."""
        if scope["type"] not in ("http", "websocket"):
            return await self.app(scope, receive, send)

        path = scope.get("path", "")

        # Public paths (exact match).
        if path in _PUBLIC_EXACT:
            return await self.app(scope, receive, send)
        if path in _LOOPBACK_PUBLIC_EXACT and _is_loopback_client(scope):
            return await self.app(scope, receive, send)

        # Public path prefixes (docs, static).
        for prefix in _PUBLIC_PREFIXES:
            if path.startswith(prefix):
                return await self.app(scope, receive, send)

        # Static dashboard assets: files with an extension, not under /api.
        last_segment = path.rsplit("/", 1)[-1] if path else ""
        if "." in last_segment and not path.startswith("/api"):
            return await self.app(scope, receive, send)

        # The current boot-scoped Product session may operate only the local
        # Explore task boundary. This lets the robot-side operator CLI work
        # without exposing the administrator API key to the ``sunrise`` user.
        product_session_headers = [
            value.decode("latin-1")
            for key, value in scope.get("headers", [])
            if key.decode("latin-1").lower() == "x-lingtu-product-session"
        ]
        if product_session_headers:
            method = str(scope.get("method") or "").upper()
            provided_hash = (
                hashlib.sha256(product_session_headers[0].encode()).hexdigest()
                if len(product_session_headers) == 1
                else None
            )
            allowed = bool(
                scope["type"] == "http"
                and _is_loopback_client(scope)
                and (method, path) in _PRODUCT_SESSION_ALLOWED_ROUTES
                and provided_hash
                and self._product_session_hash
                and hmac.compare_digest(
                    provided_hash,
                    self._product_session_hash,
                )
            )
            if allowed:
                return await self.app(scope, receive, send)
            return await self._reject(
                scope,
                send,
                403,
                "Product session credential is invalid or not permitted for this route",
            )

        # Auth disabled by default; fail closed for protected paths only when
        # the caller explicitly requires an API key.
        if self._key_hash is None and self._rmf_key_hash is None and self._map_key_hash is None:
            if self._require_key:
                return await self._reject(scope, send, 401, "API key required")
            return await self.app(scope, receive, send)

        # Extract API key from headers / query / cookies.
        header_keys = [
            value.decode("latin-1")
            for key, value in scope.get("headers", [])
            if key.decode("latin-1").lower() == "x-api-key"
        ]
        header_key = header_keys[0] if len(header_keys) == 1 else None
        client_key = self._extract_key(scope)

        client_hash = hashlib.sha256(client_key.encode()).hexdigest() if client_key else None

        # 1. Check main operator API key.
        if client_hash and self._key_hash and hmac.compare_digest(client_hash, self._key_hash):
            return await self.app(scope, receive, send)

        # 2. Check scoped Open-RMF key (restricted to allowed routes only).
        header_hash = hashlib.sha256(header_key.encode()).hexdigest() if header_key else None
        if self._rmf_key_hash and header_hash and hmac.compare_digest(header_hash, self._rmf_key_hash):
            method = str(scope.get("method") or "").upper()
            if scope["type"] == "http" and (method, path) in _RMF_ALLOWED_ROUTES:
                return await self.app(scope, receive, send)
            return await self._reject(
                scope,
                send,
                403,
                "Open-RMF key is not permitted for this route",
            )

        # 3. Check scoped map key (header-only and route-scoped).
        if (
            self._map_key_hash
            and header_hash
            and hmac.compare_digest(
                header_hash,
                self._map_key_hash,
            )
        ):
            method = str(scope.get("method") or "").upper()
            if scope["type"] == "http" and _map_key_allows(method, path):
                return await self.app(scope, receive, send)
            return await self._reject(
                scope,
                send,
                403,
                "Map key is not permitted for this route",
            )

        # 4. No valid key matched.
        if not client_key:
            return await self._reject(scope, send, 401, "需要 API Key 认证")
        return await self._reject(scope, send, 403, "API Key 无效")

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _extract_key(scope) -> str | None:
        """Return the API key from headers, query params, or cookies (or None)."""
        # ASGI headers are a list of (bytes, bytes) tuples.
        raw_headers = scope.get("headers", [])
        headers = {k.decode("latin-1").lower(): v.decode("latin-1") for k, v in raw_headers}

        # 1. X-API-Key header
        key = headers.get("x-api-key")
        if key:
            return key

        # 2. Standard Bearer authentication for service-to-service clients.
        authorization = headers.get("authorization", "").strip()
        if authorization:
            scheme, separator, credentials = authorization.partition(" ")
            if separator and scheme.lower() == "bearer" and credentials.strip():
                return credentials.strip()

        # 3. Query param ?api_key=
        query = scope.get("query_string", b"").decode("latin-1")
        if query:
            parsed = parse_qs(query)
            if parsed.get("api_key"):
                return parsed["api_key"][0]

        # 4. Cookie lingtu_api_key=
        cookie_header = headers.get("cookie", "")
        if cookie_header:
            for cookie in cookie_header.split(";"):
                cookie = cookie.strip()
                if cookie.startswith("lingtu_api_key="):
                    return cookie[len("lingtu_api_key=") :]

        return None

    @staticmethod
    async def _reject(scope, send, status_code: int, message: str) -> None:
        """Reject a request. HTTP → JSON body; WebSocket → close with 4xxx."""
        if scope["type"] == "http":
            body = json.dumps(
                {
                    "error": "unauthorized" if status_code == 401 else "forbidden",
                    "message": message,
                }
            ).encode()
            await send(
                {
                    "type": "http.response.start",
                    "status": status_code,
                    "headers": [
                        (b"content-type", b"application/json; charset=utf-8"),
                        (b"content-length", str(len(body)).encode()),
                    ],
                }
            )
            await send(
                {
                    "type": "http.response.body",
                    "body": body,
                }
            )
        else:
            # WebSocket: custom close codes in the 4xxx range.
            close_code = 4401 if status_code == 401 else 4403
            reason = message.encode("utf-8")[:123].decode("utf-8", errors="ignore")
            await send(
                {
                    "type": "websocket.close",
                    "code": close_code,
                    "reason": reason,
                }
            )
