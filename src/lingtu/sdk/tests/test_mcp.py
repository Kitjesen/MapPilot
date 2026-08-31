"""Tests for the minimal stdlib MCP client."""

from __future__ import annotations

import json
from unittest.mock import patch

import pytest

from lingtu.sdk.mcp import LingTuMCP, MCPError


class _Response:
    def __init__(self, payload: object) -> None:
        self._raw = json.dumps(payload).encode("utf-8")

    def __enter__(self) -> _Response:
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def read(self) -> bytes:
        return self._raw


@patch("urllib.request.urlopen")
def test_call_posts_tool_and_decodes_text_json(urlopen) -> None:
    urlopen.return_value = _Response(
        {"result": {"content": [{"type": "text", "text": '{"ok":true}'}]}}
    )
    client = LingTuMCP(host="robot")

    result = client.call("navigate_to", {"x": 1.0})

    assert result == {"ok": True}
    request = urlopen.call_args.args[0]
    assert request.full_url == "http://robot:8090/mcp"
    assert json.loads(request.data) == {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "tools/call",
        "params": {"name": "navigate_to", "arguments": {"x": 1.0}},
    }


@patch("urllib.request.urlopen")
def test_call_raises_json_rpc_error(urlopen) -> None:
    urlopen.return_value = _Response({"error": {"code": 42, "message": "denied"}})

    with pytest.raises(MCPError, match="denied"):
        LingTuMCP().call("stop")


@patch("urllib.request.urlopen")
def test_call_raises_tool_error(urlopen) -> None:
    urlopen.return_value = _Response(
        {"result": {"isError": True, "content": [{"type": "text", "text": "denied"}]}}
    )

    with pytest.raises(MCPError, match="denied"):
        LingTuMCP().call("stop")


@patch("urllib.request.urlopen")
def test_list_tools_returns_server_catalog(urlopen) -> None:
    tools = [{"name": "stop", "inputSchema": {}}]
    urlopen.return_value = _Response({"result": {"tools": tools}})

    assert LingTuMCP().list_tools() == tools


@patch("urllib.request.urlopen")
def test_health_uses_health_endpoint(urlopen) -> None:
    urlopen.return_value = _Response({"ok": True})

    assert LingTuMCP(host="robot").health() == {"ok": True}
    request = urlopen.call_args.args[0]
    assert request.full_url == "http://robot:8090/health"
    assert request.get_method() == "GET"
