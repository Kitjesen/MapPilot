"""Same-origin AMap JS API proxy; the security code stays on the host."""

import http.client
import json
import os
from pathlib import Path
import re
from urllib.parse import urlencode

from fastapi import Request
from fastapi.responses import JSONResponse, Response


def amap_credentials() -> tuple[str, str, str]:
    key = os.environ.get("LINGTU_AMAP_KEY", "")
    code = os.environ.get("LINGTU_AMAP_SECURITY_CODE", "")
    if not (key or code):
        default = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "LingTu" / "amap.json" if os.name == "nt" else Path("/etc/lingtu/amap.json")
        path = Path(os.environ.get("LINGTU_AMAP_CONFIG", str(default)))
        try:
            config = json.loads(path.read_text(encoding="utf-8-sig"))
        except FileNotFoundError:
            return "", "", "missing"
        except (OSError, ValueError):
            return "", "", "invalid"
        if not isinstance(config, dict):
            return "", "", "invalid"
        key, code = config.get("key", ""), config.get("security_code", "")
    if not isinstance(key, str) or not isinstance(code, str):
        return "", "", "invalid"
    key, code = key.strip(), code.strip()
    return key, code, "configured" if key and code else "incomplete"


def register_amap_routes(app) -> None:
    @app.get("/api/v1/geography/map-config")
    def map_config():
        key, _, state = amap_credentials()
        return JSONResponse({"state": state, "key": key if state == "configured" else None,
                             "service_host": "/_AMapService"}, headers={"Cache-Control": "no-store"})

    @app.get("/_AMapService/{path:path}")
    def amap_proxy(path: str, request: Request):
        key, code, state = amap_credentials()
        if state != "configured":
            return JSONResponse({"error": "amap_not_configured"}, status_code=503)
        if not re.fullmatch(r"v[34]/[A-Za-z0-9_/-]+", path) or "//" in path:
            return JSONResponse({"error": "amap_path_not_supported"}, status_code=400)
        # Match the official JS API proxy routing, without forwarding cookies or credentials.
        host = "webapi.amap.com" if path == "v4/map/styles" else "restapi.amap.com"
        params = [(name, value) for name, value in request.query_params.multi_items() if name.lower() not in {"key", "jscode"}]
        params.extend((("key", key), ("jscode", code)))
        connection = http.client.HTTPSConnection(host, timeout=10)
        try:
            connection.request("GET", f"/{path}?{urlencode(params)}")
            upstream = connection.getresponse()
            if upstream.status != 200:
                return JSONResponse({"error": "amap_upstream_failed"}, status_code=502)
            content = upstream.read(8 * 1024 * 1024 + 1)
            if len(content) > 8 * 1024 * 1024:
                return JSONResponse({"error": "amap_response_too_large"}, status_code=502)
            return Response(content, headers={"Content-Type": upstream.getheader("Content-Type", "application/octet-stream"),
                                              "Cache-Control": "no-store"})
        except (OSError, http.client.HTTPException):
            return JSONResponse({"error": "amap_upstream_unavailable"}, status_code=502)
        finally:
            connection.close()
