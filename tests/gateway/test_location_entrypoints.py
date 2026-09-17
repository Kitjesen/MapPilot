"""Public location clients must use the same server-owned save contract."""

from __future__ import annotations

import asyncio
import json
import time
from io import BytesIO
from urllib.parse import quote, urlsplit

import pytest

pytest.importorskip("fastapi")
from fastapi import FastAPI
from fastapi.testclient import TestClient

from decision.modules.agent_planner import AgentPlannerModule
from decision.tasks.agent import AgentLoop
from gateway.gateway_module import GatewayModule
from gateway.maps.locations import register_location_routes
from gateway.mcp_server import MCPServerModule
from lingtu.sdk import LingTuClient
from memory.modules.tagged_locations_module import TaggedLocationsModule


@pytest.fixture
def location_runtime(monkeypatch):
    gateway = GatewayModule()
    tagged = TaggedLocationsModule()
    gateway._tagged_loc_module = tagged
    binding = {"map_id": "yard", "map_content_epoch": 7, "frame_id": "map", "binding_status": "bound"}
    tagged.store.tag("dock", x=1, y=2, z=0.4, yaw=0.8, metadata=binding)
    gateway._runtime_cache.record_odometry(
        {"x": 3.0, "y": 4.0, "z": 1.2, "yaw": 0.5, "frame_id": "map"}, ts=time.time(),
    )
    monkeypatch.setattr("gateway.maps.locations._location_map_binding", lambda gw: dict(binding))
    mcp = MCPServerModule()
    mcp.on_system_modules({"GatewayModule": gateway, "TaggedLocationsModule": tagged, "MCPServerModule": mcp})
    mcp._odom = {"x": 90.0, "y": 91.0, "z": 0.0}
    return gateway, tagged, mcp, binding


def test_mcp_save_uses_gateway_pose_and_preserves_map_binding(location_runtime):
    _gateway, tagged, mcp, binding = location_runtime
    result = json.loads(mcp.tag_location("dock"))
    assert result.get("error") is None
    entry = tagged.store.query("dock")
    assert entry["position"] == [3.0, 4.0, 1.2]
    assert entry["yaw"] == 0.5
    assert entry["metadata"] == binding


@pytest.mark.parametrize("failure", ["stale_pose", "missing_gateway", "disk_failure"])
def test_mcp_save_failure_does_not_replace_a_bound_location(location_runtime, monkeypatch, failure):
    gateway, tagged, mcp, _ = location_runtime
    before = tagged.store.query("dock")
    if failure == "stale_pose":
        gateway._odom_timestamps[:] = [time.time() - 30]
    elif failure == "missing_gateway":
        mcp._all_modules.pop("GatewayModule")
    else:
        def fail_write(data):
            raise OSError("disk full")
        monkeypatch.setattr(tagged.store, "_persist", fail_write)

    result = json.loads(mcp.tag_location("dock"))
    assert result.get("error")
    assert tagged.store.query("dock") == before


def test_sdk_delete_reaches_the_actual_location_route(location_runtime, monkeypatch):
    gateway, tagged, _, _ = location_runtime
    name = "北侧停靠点 1"
    tagged.store.tag(name, x=1, y=2)
    app = FastAPI()
    register_location_routes(app, gateway)
    requests = []
    with TestClient(app) as http:
        def urlopen(request, **kwargs):
            requests.append(request)
            response = http.request(request.get_method(), urlsplit(request.full_url).path,
                                    content=request.data, headers=dict(request.header_items()))
            return BytesIO(response.content)
        monkeypatch.setattr("urllib.request.urlopen", urlopen)
        result = LingTuClient(api_key="test-key").delete_location(name)

    assert result.ok
    assert tagged.store.query(name) is None
    assert requests[0].get_method() == "DELETE"
    assert requests[0].full_url.endswith("/locations/" + quote(name, safe=""))
    assert dict(requests[0].header_items())["X-api-key"] == "test-key"


def test_agent_does_not_advertise_a_fake_tagging_fallback(monkeypatch):
    planner = AgentPlannerModule(llm_backend="mock")
    planner._llm_client = object()
    planner.on_system_modules({"TaggedLocationsModule": TaggedLocationsModule()})

    async def inspect_run(loop, instruction):
        return {tool["function"]["name"] for tool in loop._tools}

    monkeypatch.setattr(AgentLoop, "run", inspect_run)
    tools = asyncio.run(planner._run_agent_loop("save current location as dock"))
    assert "tag_location" not in tools


def test_agent_uses_the_discovered_tagging_service(location_runtime, monkeypatch):
    gateway, tagged, mcp, binding = location_runtime
    planner = AgentPlannerModule(llm_backend="mock")
    planner._llm_client = object()
    planner.on_system_modules({"GatewayModule": gateway, "TaggedLocationsModule": tagged, "MCPServerModule": mcp})

    async def save_during_run(loop, instruction):
        return json.loads(loop._handlers["tag_location"](name="dock"))

    monkeypatch.setattr(AgentLoop, "run", save_during_run)
    result = asyncio.run(planner._run_agent_loop("save current location as dock"))
    assert result.get("error") is None
    assert tagged.store.query("dock")["metadata"] == binding
    assert tagged.store.query("dock")["position"] == [3.0, 4.0, 1.2]


@pytest.mark.parametrize("binding_status", ["unavailable", "unbound", "content_epoch_unavailable"])
def test_bound_location_is_preserved_when_map_binding_cannot_be_read(location_runtime, binding_status):
    gateway, tagged, _, binding = location_runtime
    before = tagged.store.query("dock")
    binding.clear()
    binding.update({"binding_status": binding_status, "frame_id": "map"})
    app = FastAPI()
    register_location_routes(app, gateway)
    with TestClient(app) as http:
        response = http.put("/api/v1/locations/dock", json={"name": "dock", "use_current_pose": True})

    assert response.json()["ok"] is False
    assert response.json()["error"] == "location_binding_unavailable"
    assert tagged.store.query("dock") == before
