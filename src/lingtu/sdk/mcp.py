"""Minimal JSON-RPC client for the LingTu MCP server."""

from __future__ import annotations

import json
import urllib.request
from typing import Any, cast
from urllib.parse import urlsplit

from lingtu.sdk.client import LingTuClient

_MCP_PORT = 8090


class LingTuMCP:
    """Call MCP tools without mirroring the server's dynamic tool catalog."""

    def __init__(
        self,
        client: LingTuClient | None = None,
        *,
        host: str | None = None,
        port: int = _MCP_PORT,
        timeout: float | None = None,
        api_key: str | None = None,
    ) -> None:
        if host:
            resolved_host = host
        elif client is not None:
            resolved_host = cast(str, urlsplit(client.base_url).hostname)
        else:
            resolved_host = "127.0.0.1"
        if ":" in resolved_host and not resolved_host.startswith("["):
            resolved_host = f"[{resolved_host}]"
        self._base = f"http://{resolved_host}:{port}"
        self._timeout = client._timeout if client is not None else 10.0
        self._api_key = client._api_key if client is not None else None
        if timeout is not None:
            self._timeout = timeout
        if api_key is not None:
            self._api_key = api_key
        self._request_id = 0

    def call(self, tool: str, params: dict[str, Any] | None = None) -> Any:
        """Call one MCP tool by its server-owned name."""
        self._request_id += 1
        body = self._post(
            "/mcp",
            {
                "jsonrpc": "2.0",
                "id": self._request_id,
                "method": "tools/call",
                "params": {"name": tool, "arguments": params or {}},
            },
        )
        result = body.get("result", {})
        if result.get("isError"):
            text = next(
                (
                    str(item.get("text") or "").strip()
                    for item in result.get("content") or []
                    if str(item.get("text") or "").strip()
                ),
                "MCP tool failed",
            )
            raise MCPError(-32000, text)
        for item in result.get("content") or []:
            text = item.get("text", "")
            if not text:
                continue
            try:
                return json.loads(text)
            except (json.JSONDecodeError, TypeError):
                return text
        return result

    def list_tools(self) -> list[dict[str, Any]]:
        """Return the server-owned MCP tool catalog."""
        self._request_id += 1
        body = self._post(
            "/mcp",
            {
                "jsonrpc": "2.0",
                "id": self._request_id,
                "method": "tools/list",
            },
        )
        return list(body.get("result", {}).get("tools", []))

    def health(self) -> dict[str, Any]:
        """Return MCP server health."""
        request = urllib.request.Request(f"{self._base}/health", method="GET")
        return self._read(request)

    def _post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        request = urllib.request.Request(
            f"{self._base}{path}",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        body = self._read(request)
        if "error" in body:
            error = body["error"]
            raise MCPError(
                int(error.get("code", -1)),
                str(error.get("message", "Unknown error")),
            )
        return body

    def _read(self, request: urllib.request.Request) -> dict[str, Any]:
        if self._api_key:
            request.add_header("X-API-Key", self._api_key)
        with urllib.request.urlopen(request, timeout=self._timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("MCP response must be a JSON object")
        return payload


class MCPError(Exception):
    """Raised when an MCP tool call returns a JSON-RPC error."""

    def __init__(self, code: int, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(f"[MCP {code}] {message}")
