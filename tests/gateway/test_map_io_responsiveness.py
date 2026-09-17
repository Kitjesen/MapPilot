"""Slow map and location I/O must leave the HTTP event loop available."""

from __future__ import annotations

import asyncio
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")
from fastapi import FastAPI

from gateway.maps.locations import register_location_routes
from gateway.maps.routes import register_map_routes
from gateway.schemas import LocationUpsertRequest, MapRenameRequest
from memory.spatial.tagged_locations import TaggedLocationStore


def _endpoint(app, name):
    return next(route.endpoint for route in app.routes if route.name == name)


def _wait_for_loop(loop, progress):
    release = threading.Event()
    loop.call_soon_threadsafe(release.set)
    progress.append(release.wait(timeout=1.0))


@pytest.mark.parametrize("route_name", [
    "import_pcd_map", "crop_saved_map", "mark_saved_map_zone",
    "edit_saved_map_voxels", "get_saved_map_voxel_edits", "rename_map",
    "get_saved_map_points", "build_saved_map_octomap",
])
def test_map_io_does_not_block_other_requests(monkeypatch, tmp_path, route_name):
    import gateway.maps.routes as routes

    identity = {"map_id": "yard", "protocol_version": 1, "epoch": 1, "sequence": 1, "stamp_s": 1.0}
    gw = SimpleNamespace(
        clear_map_cloud_cache=lambda **kwargs: None,
        _cloud_viewer=SimpleNamespace(scene_identity=lambda: dict(identity)),
    )
    app = FastAPI()
    register_map_routes(app, gw)
    source = tmp_path / "map.pcd"
    source.write_text("DATA ascii\n", encoding="utf-8")
    monkeypatch.setattr(routes, "map_import_root", lambda: tmp_path)
    args = {
        "import_pcd_map": ({"name": "yard", "source_path": str(source)},),
        "crop_saved_map": ("yard", {"bounds": {}}),
        "mark_saved_map_zone": ("yard", {}),
        "edit_saved_map_voxels": ("yard", {}),
        "get_saved_map_voxel_edits": ("yard",),
        "rename_map": (MapRenameRequest(old_name="yard", new_name="yard2"),),
        "get_saved_map_points": ("yard",),
        "build_saved_map_octomap": ("yard",),
    }
    progress = []

    async def scenario():
        loop = asyncio.get_running_loop()

        def request(*args):
            _wait_for_loop(loop, progress)
            return {"success": True, "map_id": "yard", "content_epoch": 1,
                    "frame_id": "map", "points": [], "edits": []}

        def active(*args):
            _wait_for_loop(loop, progress)
            return "yard"

        monkeypatch.setattr(routes, "_mapd_http_request", request)
        monkeypatch.setattr(routes, "active_map", active)
        await _endpoint(app, route_name)(*args[route_name])

    asyncio.run(scenario())
    assert progress and all(progress), "map I/O blocked the event loop"


@pytest.mark.parametrize("route_name", ["post_location", "put_location", "delete_location", "get_locations"])
def test_location_io_does_not_block_other_requests(monkeypatch, route_name):
    import gateway.maps.locations as locations

    store = TaggedLocationStore()
    store.tag("dock", x=1.0, y=2.0)
    app = FastAPI()
    register_location_routes(app, SimpleNamespace(_tagged_loc_module=SimpleNamespace(store=store)))
    progress = []
    body = LocationUpsertRequest(name="dock", x=2.0, y=3.0)
    args = {"post_location": (body,), "put_location": ("dock", body),
            "delete_location": ("dock",), "get_locations": ()}

    async def scenario():
        loop = asyncio.get_running_loop()
        remove = store.remove
        list_all = store.list_all

        def map_request(*args):
            _wait_for_loop(loop, progress)
            return {"success": True, "active": "yard", "record": {"content_epoch": 1}}

        def slow_remove(name):
            _wait_for_loop(loop, progress)
            return remove(name)

        def slow_list():
            _wait_for_loop(loop, progress)
            return list_all()

        monkeypatch.setattr(locations, "mapd_request", map_request)
        monkeypatch.setattr(store, "remove", slow_remove)
        monkeypatch.setattr(store, "list_all", slow_list)
        payload = await _endpoint(app, route_name)(*args[route_name])
        assert payload.get("ok", True)

    asyncio.run(scenario())
    assert progress and all(progress), "location I/O blocked the event loop"


@pytest.mark.parametrize("route_name", ["post_location", "delete_location"])
def test_location_api_reports_failed_persistence(monkeypatch, tmp_path, route_name):
    import os

    import gateway.maps.locations as locations

    store = TaggedLocationStore(str(tmp_path / "tags.json"))
    store.tag("dock", x=1.0, y=2.0)
    events = []
    gw = SimpleNamespace(
        _tagged_loc_module=SimpleNamespace(store=store, tag_status=SimpleNamespace(publish=events.append)),
    )
    app = FastAPI()
    register_location_routes(app, gw)
    monkeypatch.setattr(locations, "_location_map_binding", lambda gw: {})

    def fail_replace(*args):
        raise OSError("disk write failed")

    monkeypatch.setattr(os, "replace", fail_replace)
    argument = LocationUpsertRequest(name="dock", x=8.0, y=9.0) if route_name == "post_location" else "dock"
    result = asyncio.run(_endpoint(app, route_name)(argument))

    assert result["ok"] is False
    assert result["status"] == "error"
    assert "disk write failed" in result["error"]
    assert store.query("dock")["position"] == [1.0, 2.0, 0.0]
    assert events == []
