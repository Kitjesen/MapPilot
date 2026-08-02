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

    def start_inspection_task(self, **kwargs):
        self.calls.append(("task_start", kwargs))
        return True

    def pause_inspection_task(self, **kwargs):
        self.calls.append(("task_pause", kwargs))
        return True

    def resume_inspection_task(self, **kwargs):
        self.calls.append(("task_resume", kwargs))
        return True

    def cancel_inspection_task(self, **kwargs):
        self.calls.append(("task_cancel", kwargs))
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

    assert [call[0] for call in _Store.calls] == ["list", "put", "get", "delete", "status"]
    assert commands.calls == []


def test_inspection_service_task_operations_keep_the_caller_task_id(tmp_path) -> None:
    commands = _Commands()
    service = inspection_service.Inspection(map_root=str(tmp_path))
    service.on_system_modules({"nav.commands": commands})

    assert service.start_task("task-42", "route-a", 7, "request-start") is True
    assert service.pause_task("task-42", "operator_hold", "request-pause") is True
    assert service.resume_task("task-42", "operator_resume", "request-resume") is True
    assert service.cancel_task("task-42", "operator_cancel", "request-cancel") is True

    assert commands.calls == [
        (
            "task_start",
            {
                "task_id": "task-42",
                "route_id": "route-a",
                "revision": 7,
                "request_id": "request-start",
            },
        ),
        (
            "task_pause",
            {
                "task_id": "task-42",
                "reason": "operator_hold",
                "request_id": "request-pause",
            },
        ),
        (
            "task_resume",
            {
                "task_id": "task-42",
                "reason": "operator_resume",
                "request_id": "request-resume",
            },
        ),
        (
            "task_cancel",
            {
                "task_id": "task-42",
                "reason": "operator_cancel",
                "request_id": "request-cancel",
            },
        ),
    ]


def test_inspection_execution_fails_closed_without_command_capability(tmp_path) -> None:
    service = inspection_service.Inspection(map_root=str(tmp_path))

    with pytest.raises(RuntimeError, match="command capability is unavailable"):
        service.start_task("task-42", "route-a")


@pytest.mark.parametrize("ack", [None, "true", 1, {"accepted": True}])
def test_inspection_execution_rejects_non_boolean_ack(tmp_path, ack) -> None:
    class MalformedCommands:
        @staticmethod
        def start_inspection_task(**_kwargs):
            return ack

    service = inspection_service.Inspection(map_root=str(tmp_path))
    service.on_system_modules({"nav.commands": MalformedCommands()})

    with pytest.raises(RuntimeError, match="invalid acknowledgement"):
        service.start_task("task-42", "route-a")


def test_taskless_inspection_service_methods_are_retired(tmp_path) -> None:
    service = inspection_service.Inspection(map_root=str(tmp_path))

    for method in ("start_route", "pause", "resume", "cancel"):
        assert not hasattr(service, method)


def test_inspection_execution_returns_native_boolean_rejection_to_its_boundary(tmp_path) -> None:
    class RejectingCommands:
        @staticmethod
        def start_inspection_task(**_kwargs):
            return False

    service = inspection_service.Inspection(map_root=str(tmp_path))
    service.on_system_modules({"nav.commands": RejectingCommands()})

    assert service.start_task("task-42", "route-a", 7, "request-42") is False
