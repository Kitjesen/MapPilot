"""Minimal JSON-RPC client for the LingTu MCP server."""

from __future__ import annotations

import json
import urllib.request
from typing import Any

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
        timeout: float = 10.0,
    ) -> None:
        if host:
            resolved_host = host
        elif client is not None:
            resolved_host = client.base_url.split("://", 1)[1].rsplit(":", 1)[0]
        else:
            resolved_host = "127.0.0.1"
        self._base = f"http://{resolved_host}:{port}"
        self._timeout = timeout
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
        if "error" in body:
            error = body["error"]
            raise MCPError(
                int(error.get("code", -1)),
                str(error.get("message", "Unknown error")),
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
        return self._read(request)

    def _read(self, request: urllib.request.Request) -> dict[str, Any]:
        with urllib.request.urlopen(request, timeout=self._timeout) as response:  # noqa: S310
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
