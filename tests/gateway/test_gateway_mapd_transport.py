from __future__ import annotations

import json
from types import SimpleNamespace

import pytest


class RecordingMapClient:
    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, object]]] = []

    def service(self, action: str, **arguments: object) -> dict[str, object]:
        self.calls.append((action, arguments))
        return {"success": True, "action": action}


def test_mapd_request_uses_mapd_canonical_action_without_maps_module() -> None:
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    gateway = SimpleNamespace(_map_client=client)

    response = mapd_request(
        gateway,
        {"action": "rename_map", "map_id": "old", "new_map_id": "new"},
    )

    assert response == {"success": True, "action": "rename_map"}
    assert client.calls == [("rename_map", {"map_id": "old", "new_map_id": "new"})]


def test_mapd_request_flattens_structured_gateway_arguments() -> None:
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    gateway = SimpleNamespace(_map_client=client)

    mapd_request(
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


def test_mapd_request_uses_native_save_map_entrypoint() -> None:
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    gateway = SimpleNamespace(_map_client=client)

    mapd_request(
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
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()

    with pytest.raises(RuntimeError, match="ProductControl"):
        mapd_request(
            SimpleNamespace(_map_client=client),
            {"action": action, "name": "yard"},
        )

    assert client.calls == []


@pytest.mark.parametrize("native_active", ["yard", "other", None, RuntimeError("mapd unavailable")])
def test_goal_map_gate_reads_injected_mapd_without_gateway_forwarders(native_active, monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import GoalRequest
    from gateway.services.control_commands import ControlCommandService

    gateway = GatewayModule()
    calls = []
    submitted = []

    def read_map(action, **arguments):
        calls.append((action, arguments))
        if isinstance(native_active, Exception):
            raise native_active
        return {"success": True, "active": native_active}

    gateway._map_client = SimpleNamespace(service=read_map)
    monkeypatch.setattr(
        "gateway.navigation.status.evaluate_navigation_gate",
        lambda gw: {"can_accept_goal": True, "blockers": []},
    )
    body = GoalRequest(x=1.0, y=2.0, metadata={"map_name": "yard"})

    def submit():
        submitted.append(body)
        return {"accepted": True}

    response = ControlCommandService(gateway).run_planned_goal_command("goal", body, submit)

    assert calls == [("get_active_map", {})]
    if native_active == "yard":
        assert submitted == [body]
        assert response["command"]["accepted"] is True
    else:
        assert submitted == []
        assert response.status_code == 409
        assert json.loads(response.body)["error"] == "active_map_mismatch"


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
    from gateway.maps import transport as mapd_transport

    assert not hasattr(mapd_transport, "artifact_path")
