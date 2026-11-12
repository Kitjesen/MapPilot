from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from gateway.schemas import ConversationMessageRequest
from gateway.services.conversation_tools import workbench_tools


@pytest.mark.asyncio
async def test_live_camera_card_only_requests_a_view_not_capture_or_motion():
    tools, invoke = workbench_tools(SimpleNamespace())
    tool = next(item for item in tools if item["function"]["name"] == "view_live_camera")
    assert tool["writes"] is False
    result = await invoke("view_live_camera", {})
    assert result == {"success": True, "camera_view": {"source": "robot"}, "read_only": True}
    assert "connected" not in result and "image_data_url" not in result


@pytest.mark.asyncio
async def test_view_map_returns_versioned_card_without_geometry_or_activation(monkeypatch):
    calls = []
    def request(gw, command):
        calls.append(command)
        return {"success": True, "record": {"content_epoch": 12, "state": "READY", "private_path": "/private/map.pcd"}}
    monkeypatch.setattr("gateway.services.conversation_tools.mapd_request", request)
    tools, invoke = workbench_tools(SimpleNamespace())
    definition = next(item for item in tools if item["function"]["name"] == "view_saved_map")
    assert definition["writes"] is False
    result = await invoke("view_saved_map", {"map_id": "hall"})
    assert result == {"success": True, "map_view": {"map_id": "hall", "content_epoch": 12}, "state": "READY", "read_only": True}
    assert calls == [{"action": "get_record", "map_id": "hall"}]
    invalid = await invoke("view_saved_map", {"map_id": "../other"})
    assert invalid["success"] is False and len(calls) == 1


@pytest.mark.asyncio
@pytest.mark.parametrize("response", [{"success": False, "reason_code": "map_not_found"},
    {"success": True, "record": {"content_epoch": 0}}, {"success": True, "record": {"content_epoch": True}},
    {"success": True, "record": "invalid"}])
async def test_view_map_rejects_missing_map_and_invalid_version(monkeypatch, response):
    monkeypatch.setattr("gateway.services.conversation_tools.mapd_request", lambda *_: response)
    _, invoke = workbench_tools(SimpleNamespace())
    result = await invoke("view_saved_map", {"map_id": "hall"})
    assert result["success"] is False and "map_view" not in result


@pytest.mark.asyncio
async def test_preview_uses_native_owner_and_keeps_geometry_out_of_model(monkeypatch):
    calls = []

    def request(gw, command):
        calls.append(command)
        return {"success": True, "source_content_epoch": 7, "frame_id": "map", "point_count": 1,
                "preview_points": [1, 2, 3], "preview_removed": [], "preview_sample": False,
                "added_points": 1, "removed_points": 0}

    monkeypatch.setattr("gateway.maps.routes._mapd_http_request", request)
    tools, invoke = workbench_tools(SimpleNamespace())
    operation = {"operation": "add_plane", "axis": "z", "offset": 0, "spacing": .1,
                 "bounds": {"min_x": 0, "min_y": 0, "min_z": 0, "max_x": 1, "max_y": 1, "max_z": 0}}
    args = {"map_id": "garage", "expected_source_content_epoch": 7, "operations": [operation]}
    result = await invoke("preview_map_edits", args)
    assert result.get("success") is True or result.get("ok") is True
    assert "preview_points" not in result and "preview_removed" not in result
    assert result["preview_request"] == args and result["added_points"] == 1
    assert calls == [{"action": "preview_source_edits", **args}]
    region = next(tool for tool in tools if tool["function"]["name"] == "set_navigation_region")
    assert region["writes"] and "NoGoRegion" in region["function"]["parameters"]["$defs"]


@pytest.mark.asyncio
async def test_invalid_map_and_point_do_not_reach_native_owner(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Invalid input reached the map owner")

    monkeypatch.setattr("gateway.maps.routes._mapd_http_request", forbidden)
    _, invoke = workbench_tools(SimpleNamespace())
    invalid = await invoke("preview_map_edits", {"map_id": "../other", "operations": [],
                                                "expected_source_content_epoch": 1})
    assert invalid["success"] is False
    invalid = await invoke("save_inspection_point", {"request": {"name": "entrance", "x": float("nan"), "y": 1}})
    assert invalid["success"] is False


@pytest.mark.asyncio
async def test_named_point_search_is_scoped_to_map_and_region():
    entries = [
        {"name": "east-1", "x": 0, "y": 1, "z": .5, "tags": [], "metadata": {"map_id": "garage", "region": "东区"}},
        {"name": "east-2", "x": 0, "y": 1, "z": .5, "tags": [], "metadata": {"map_id": "other", "region": "东区"}},
    ]
    gw = SimpleNamespace(_tagged_loc_module=SimpleNamespace(store=SimpleNamespace(list_all=lambda: entries)))
    _, invoke = workbench_tools(gw)
    result = await invoke("find_inspection_points", {"map_id": "garage", "query": "东区"})
    assert [point["name"] for point in result["locations"]] == ["east-1"]


def test_context_rejects_nonfinite_or_reversed_bounds():
    for bounds in [{"min": [0, 0, float("nan")], "max": [1, 1, 1]},
                   {"min": [2, 0, 0], "max": [1, 1, 1]}]:
        with pytest.raises(ValidationError):
            ConversationMessageRequest(message_id="x", text="fill", map_context={
                "map_id": "garage", "content_epoch": 1, "selection": bounds})

@pytest.mark.asyncio
async def test_observation_cards_do_not_read_or_mutate_robot_until_opened():
    tools, invoke = workbench_tools(SimpleNamespace())
    definition = next(item for item in tools if item["function"]["name"] == "view_robot_card")
    assert definition["writes"] is False
    for view in ("robot", "joints", "imu", "navigation"):
        assert await invoke("view_robot_card", {"view": view}) == {"success": True, "observation_view": view, "read_only": True}
    assert (await invoke("view_robot_card", {"view": "unknown"}))["success"] is False
