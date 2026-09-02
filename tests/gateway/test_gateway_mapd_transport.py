from __future__ import annotations

from types import SimpleNamespace

import pytest


class RecordingMapClient:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, object]]] = []

    def service(self, action: str, **arguments: object) -> dict[str, object]:
        self.calls.append((action, arguments))
        return {"success": True, "action": action}


def test_mapd_command_uses_mapd_canonical_action_without_maps_module() -> None:
    from gateway.services.mapd_transport import mapd_command

    client = RecordingMapClient()
    gateway = SimpleNamespace(_map_client=client)

    response = mapd_command(
        gateway,
        {"action": "rename_map", "map_id": "old", "new_map_id": "new"},
    )

    assert response == {"success": True, "action": "rename_map"}
    assert client.calls == [("rename_map", {"map_id": "old", "new_map_id": "new"})]


def test_mapd_command_flattens_structured_gateway_arguments() -> None:
    from gateway.services.mapd_transport import mapd_command

    client = RecordingMapClient()
    gateway = SimpleNamespace(_map_client=client)

    mapd_command(
        gateway,
        {
            "action": "crop_pcd",
            "map_id": "yard",
            "bounds": {
                "min": [1.0, 2.0, 3.0],
                "max": [4.0, 5.0, 6.0],
            },
            "invert": True,
            "voxel_size": 0.1,
        },
    )

    assert client.calls == [
        (
            "crop_pcd",
            {
                "map_id": "yard",
                "has_bounds": True,
                "min_x": 1.0,
                "min_y": 2.0,
                "min_z": 3.0,
                "max_x": 4.0,
                "max_y": 5.0,
                "max_z": 6.0,
                "invert": True,
                "voxel_size": 0.1,
            },
        )
    ]


def test_mapd_command_uses_native_save_map_entrypoint() -> None:
    from gateway.services.mapd_transport import mapd_command

    client = RecordingMapClient()
    gateway = SimpleNamespace(_map_client=client)

    mapd_command(
        gateway,
        {
            "action": "save_map",
            "map_id": "yard",
            "request_id": "request-1",
            "slam_profile": "fastlio2",
        },
    )

    assert client.calls == [
        (
            "save_map",
            {
                "map_id": "yard",
                "request_id": "request-1",
            },
        )
    ]


@pytest.mark.parametrize("action", ["set_active_map", "clear_active_map"])
def test_gateway_transport_rejects_active_map_mutation(action: str) -> None:
    from gateway.services.mapd_transport import mapd_command

    client = RecordingMapClient()

    with pytest.raises(RuntimeError, match="ProductControl"):
        mapd_command(
            SimpleNamespace(_map_client=client),
            {"action": action, "name": "yard"},
        )

    assert client.calls == []


def test_module_discovery_does_not_attach_maps_service_manager(monkeypatch) -> None:
    from gateway.services import module_refs

    monkeypatch.setattr(module_refs, "bind_navigation_commands", lambda *_args: None)
    monkeypatch.setattr(module_refs, "_native_relocalization_service", lambda: None)
    gateway = SimpleNamespace(
        localization=SimpleNamespace(bind=lambda _backend: None),
    )

    module_refs.attach_module_refs(gateway, {"maps.service": object()})

    assert not hasattr(gateway, "_map_mgr")


def test_mapd_transport_has_no_local_artifact_path_resolver() -> None:
    from gateway.services import mapd_transport

    assert not hasattr(mapd_transport, "artifact_path")
