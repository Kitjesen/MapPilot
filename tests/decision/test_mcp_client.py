import asyncio
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from decision.mcp.client import Connections, content_for_conversation
from decision.mcp.config import load_servers

FIXTURE = Path(__file__).parent / "fixtures/mcp_server.py"


def configuration(path, **overrides):
    path.write_text(json.dumps({"mcpServers": {"manual": {"transport": "stdio", "command": sys.executable,
        "args": [str(FIXTURE)], "read_only_tools": ["lookup_manual", "slow_lookup"], **overrides}}}), encoding="utf-8")
    return path


@pytest.mark.asyncio
async def test_real_stdio_tools_resources_prompts_and_permissions(tmp_path):
    async with Connections(configuration(tmp_path / "mcp.json")) as connections:
        assert connections.status[0]["status"] == "ready", connections.status
        lookup = next(alias for alias, (_, name) in connections.bindings.items() if name == "lookup_manual")
        assert lookup == "mcp_manual_lookup_manual"
        write = next(alias for alias, (_, name) in connections.bindings.items() if name == "save_note")
        schemas = {item["function"]["name"]: item for item in connections.tools}
        assert not schemas[lookup]["writes"] and schemas[write]["writes"]
        result = await connections.invoke(lookup, {"query": "map"})
        assert "inspect the map" in json.dumps(result)
        failed = next(alias for alias, (_, name) in connections.bindings.items() if name == "unavailable_manual")
        assert (await connections.invoke(failed, {}))["error"] == "mcp_tool_error"
        resources = await connections.invoke("list_mcp_resources", {"server": "manual"})
        assert "manual://start" in json.dumps(resources)
        resource = await connections.invoke("read_mcp_resource", {"server": "manual", "uri": "manual://start"})
        assert "No robot actions" in json.dumps(resource)
        prompts = await connections.invoke("list_mcp_prompts", {"server": "manual"})
        assert "explain" in json.dumps(prompts)
        prompt = await connections.invoke("get_mcp_prompt", {"server": "manual", "name": "explain", "arguments": {"topic": "maps"}})
        assert "Explain maps" in json.dumps(prompt)


@pytest.mark.asyncio
async def test_cancelled_call_closes_session(tmp_path):
    entered = asyncio.Event()
    async def work():
        async with Connections(configuration(tmp_path / "mcp.json")) as connections:
            slow = next(alias for alias, (_, name) in connections.bindings.items() if name == "slow_lookup")
            entered.set()
            await connections.invoke(slow, {})
    task = asyncio.create_task(work())
    await asyncio.wait_for(entered.wait(), 20)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(task, 10)


@pytest.mark.asyncio
async def test_missing_environment_does_not_leak_or_start_server(tmp_path, monkeypatch):
    monkeypatch.delenv("MCP_TEST_KEY", raising=False)
    async with Connections(configuration(tmp_path / "mcp.json", env_names=["MCP_TEST_KEY"])) as connections:
        assert not connections.servers
        assert connections.status[0]["status"] == "missing_environment"


def test_configuration_rejects_mixed_transport_fields_and_preserves_absence(tmp_path):
    assert load_servers(tmp_path / "absent.json") == {}
    with pytest.raises(ValueError):
        load_servers(configuration(tmp_path / "mixed.json", url="https://example.com/mcp"))


@pytest.mark.asyncio
@pytest.mark.parametrize("transport,path", [("streamable_http", "/mcp"), ("sse", "/sse")])
async def test_real_network_transports(tmp_path, transport, path):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    process = subprocess.Popen([sys.executable, str(FIXTURE), transport.replace("_", "-"), str(port)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0)
    try:
        deadline = time.monotonic() + 45
        while True:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                    break
            except OSError:
                assert process.poll() is None, "Fixture MCP server exited"
                if time.monotonic() > deadline:
                    pytest.fail("Fixture MCP server did not listen")
                await asyncio.sleep(0.1)
        config = tmp_path / "mcp.json"
        config.write_text(json.dumps({"mcpServers": {"http": {"transport": transport,
            "url": f"http://127.0.0.1:{port}{path}", "read_only_tools": ["lookup_manual"]}}}), encoding="utf-8")
        async with Connections(config) as connections:
            assert connections.status[0]["status"] == "ready", connections.status
            lookup = next(alias for alias, (_, name) in connections.bindings.items() if name == "lookup_manual")
            assert "inspect the map" in json.dumps(await connections.invoke(lookup, {"query": "map"}))
    finally:
        process.terminate()
        process.wait(timeout=10)


def test_nested_images_are_separate_and_binary_limit_is_explicit():
    images = []
    result = content_for_conversation({"messages": [{"content": {"type": "image", "mimeType": "image/png", "data": "aGVsbG8="}}],
                                       "contents": [{"uri": "file://sample", "blob": "aGVsbG8="}]}, images)
    assert images == ["data:image/png;base64,aGVsbG8="]
    assert "aGVsbG8=" not in json.dumps(result)
    assert "unsupported_content" in result["contents"][0]
