from __future__ import annotations

import pytest

from nav.inspection import service as inspection_service
from runtime import Module


class _Store:
    calls = []

    def __init__(self, root) -> None:
        self.root = root

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def list(self, map_id):
        self.calls.append(("list", self.root, map_id))
        return {"routes": []}

    def put(self, route):
        self.calls.append(("put", self.root, route))
        return route

    def get(self, map_id, route_id):
        self.calls.append(("get", self.root, map_id, route_id))
        return {"id": route_id, "map_id": map_id}

    def delete(self, map_id, route_id):
        self.calls.append(("delete", self.root, map_id, route_id))

    def status(self):
        self.calls.append(("status", self.root))
        return {"state": "idle"}


class _Commands:
    def __init__(self) -> None:
        self.calls = []

    def start_inspection(self, **kwargs):
        self.calls.append(("start", kwargs))
        return True


def test_inspection_service_uses_native_store_and_explicit_command_capability(
    monkeypatch,
    tmp_path,
) -> None:
    _Store.calls = []
    monkeypatch.setattr(inspection_service, "NativeInspectionStore", _Store)
    commands = _Commands()
    service = inspection_service.Inspection(map_root=str(tmp_path))
    service.on_system_modules({"nav.commands": commands})

    assert inspection_service.Inspection.start is Module.start
    assert service.list_routes("map-a") == {"routes": []}
    assert service.put_route({"id": "route-a"}) == {"id": "route-a"}
    assert service.get_route("map-a", "route-a") == {
        "id": "route-a",
        "map_id": "map-a",
    }
    assert service.delete_route("map-a", "route-a") is True
    assert service.status() == {"state": "idle"}
    assert service.start_route("route-a", 7, "request-1") is True

    assert [call[0] for call in _Store.calls] == ["list", "put", "get", "delete", "status"]
    assert commands.calls == [
        (
            "start",
            {"route_id": "route-a", "revision": 7, "request_id": "request-1"},
        )
    ]


def test_inspection_execution_fails_closed_without_command_capability(tmp_path) -> None:
    service = inspection_service.Inspection(map_root=str(tmp_path))

    with pytest.raises(RuntimeError, match="command capability is unavailable"):
        service.start_route("route-a")
