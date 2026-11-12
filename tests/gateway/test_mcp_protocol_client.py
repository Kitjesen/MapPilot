"""Official MCP client against the actual LingTu HTTP handler, without hardware."""

import asyncio
import json
import socket
import time

import pytest
from agents.mcp import MCPServerStreamableHttp

from gateway.mcp_server import MCPServerModule


@pytest.mark.asyncio
async def test_lingtu_server_initializes_and_serves_tool_to_official_client(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    module = MCPServerModule(host="127.0.0.1", port=port, require_api_key=False)
    module._tool_list = [{"name": "fixture_read", "description": "Read fixture evidence",
                         "inputSchema": {"type": "object", "properties": {}}}]
    module._tool_registry = {"fixture_read": lambda: {"observed": "fixture", "motion": False}}
    module.setup()
    module.start()
    try:
        deadline = time.monotonic() + 20
        while True:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                    break
            except OSError:
                if time.monotonic() > deadline:
                    pytest.fail("LingTu MCP did not start")
                await asyncio.sleep(0.1)
        async with MCPServerStreamableHttp(params={"url": f"http://127.0.0.1:{port}/mcp"}, name="lingtu",
                                          max_retry_attempts=0, client_session_timeout_seconds=10) as client:
            tools = await client.list_tools()
            assert [tool.name for tool in tools] == ["fixture_read"]
            result = await client.call_tool("fixture_read", {})
            assert '"motion": false' in json.dumps(result.model_dump(mode="json", by_alias=True)).replace('\\"', '"')
    finally:
        module.stop()
