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


def test_no_go_transport_keeps_policy_separate_from_occupancy():
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    gw = SimpleNamespace(_map_client=client)
    mapd_request(gw, {"action": "get_no_go_regions", "map_id": "hall"})
    mapd_request(gw, {"action": "set_no_go_region", "map_id": "hall",
                     "expected_content_epoch": 11,
                     "region": {"id": "door", "min": [1, 2, 0], "max": [2, 3, 2]}})
    mapd_request(gw, {"action": "delete_no_go_region", "map_id": "hall",
                     "expected_content_epoch": 12, "region_id": "door"})
    assert client.calls == [
        ("get_no_go_regions", {"map_id": "hall"}),
        ("set_no_go_region", {"map_id": "hall", "expected_content_epoch": 11,
                              "region_id": "door", "min_x": 1, "min_y": 2, "min_z": 0,
                              "max_x": 2, "max_y": 3, "max_z": 2}),
        ("delete_no_go_region", {"map_id": "hall", "expected_content_epoch": 12,
                                 "region_id": "door"}),
    ]


@pytest.mark.parametrize("auto_process", [True, False])
def test_save_forwards_processing_choice(auto_process):
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    mapd_request(SimpleNamespace(_map_client=client), {
        "action": "save_map", "map_id": "yard", "request_id": "save_1", "auto_process": auto_process,
    })
    assert client.calls == [("save_map", {
        "map_id": "yard", "request_id": "save_1", "auto_process": auto_process,
    })]


def test_process_uses_existing_job_without_a_new_snapshot():
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    mapd_request(SimpleNamespace(_map_client=client), {"action": "process_save_map", "job_id": "save_1"})
    assert client.calls == [("process_save_map", {"job_id": "save_1"})]


def test_reoptimize_forwards_source_and_request():
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    mapd_request(SimpleNamespace(_map_client=client), {
        "action": "reoptimize_save_map", "job_id": "save_1", "request_id": "optimize_1",
    })
    assert client.calls == [("reoptimize_save_map", {
        "job_id": "save_1", "request_id": "optimize_1",
    })]


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


@pytest.mark.parametrize(
    "body",
    [
        {"center": {"x": 1.5, "y": -2.0, "z": 0.3}},
        {"center": [1.5, -2.0, 0.3]},
        {"x": 1.5, "y": -2.0, "z": 0.3},
        {"x_m": 1.5, "y_m": -2.0, "z_m": 0.3},
    ],
)
def test_voxel_edit_forwards_the_requested_center(body: dict[str, object]) -> None:
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    mapd_request(
        SimpleNamespace(_map_client=client),
        {"action": "edit_octomap_voxels", "map_id": "demo", "state": "preblocked", **body},
    )

    arguments = client.calls[0][1]
    assert (arguments["x_m"], arguments["y_m"], arguments["z_m"]) == (1.5, -2.0, 0.3)


def test_voxel_edit_without_center_is_rejected() -> None:
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    with pytest.raises(ValueError, match="center is required"):
        mapd_request(
            SimpleNamespace(_map_client=client),
            {"action": "edit_octomap_voxels", "map_id": "demo", "state": "clear"},
        )
    assert client.calls == []


def test_active_map_holds_the_last_answer_through_a_failed_query(monkeypatch) -> None:
    from gateway.maps import transport

    answers: list[object] = [{"success": True, "active": "floor_2"}, TimeoutError("late")]

    class Client:
        def service(self, action: str, **_arguments: object) -> dict[str, object]:
            answer = answers.pop(0)
            if isinstance(answer, Exception):
                raise answer
            return answer

    clock = [100.0]
    monkeypatch.setattr(transport.time, "monotonic", lambda: clock[0])
    gateway = SimpleNamespace(_map_client=Client())

    assert transport.active_map(gateway) == "floor_2"
    clock[0] += 1.0
    assert transport.active_map(gateway) == "floor_2"

    answers.append(TimeoutError("still late"))
    clock[0] += transport._ACTIVE_MAP_HOLD_S + 1.0
    assert transport.active_map(gateway) is None

    answers.append({"success": True, "active": ""})
    assert transport.active_map(gateway) is None


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
            "slam_profile": "ltlio",
        },
    )

    assert client.calls == [
        (
            "save_map",
            {
                "map_id": "yard",
                "request_id": "request-1",
                "auto_process": True,
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


def test_edited_source_and_octomap_build_keep_structured_contract():
    from gateway.maps.transport import mapd_request
    client = RecordingMapClient()
    gw = SimpleNamespace(_map_client=client)
    operations = [{"operation": "keep_inside", "bounds": {"min_x": 0, "max_x": 2}}]
    mapd_request(gw, {"action": "save_edited_source", "map_id": "source", "new_map_id": "new",
                      "expected_source_content_epoch": 21, "operations": operations})
    mapd_request(gw, {"action": "build_octomap_artifact", "map_id": "new",
                      "frame_id": "floor_903", "resolution": 0.1, "source_profile": "ltlio"})
    assert client.calls == [
        ("save_edited_source", {"map_id": "source", "new_map_id": "new",
          "expected_source_content_epoch": 21, "operations": operations}),
        ("build_octomap_artifact", {"map_id": "new", "octomap_frame_id": "floor_903",
          "octomap_resolution": 0.1, "octomap_source_profile": "ltlio"})]


def test_edit_preview_forwards_the_same_filter_history_without_a_target():
    from gateway.maps.transport import mapd_request

    client = RecordingMapClient()
    operations = [{"operation": "statistical", "neighbors": 20, "std_ratio": 1.5},
                  {"operation": "plane_crop", "distance_threshold": 0.05, "margin": 0.5}]
    mapd_request(SimpleNamespace(_map_client=client), {
        "action": "preview_source_edits", "map_id": "source",
        "expected_source_content_epoch": 21, "operations": operations})

    assert client.calls == [("preview_source_edits", {
        "map_id": "source", "expected_source_content_epoch": 21, "operations": operations})]


def test_surface_preview_transport_keeps_physical_options():
    from gateway.maps.transport import mapd_request
    client = RecordingMapClient()
    mapd_request(SimpleNamespace(_map_client=client), {"action": "get_surface_map", "map_id": "hall",
        "radius_m": .3, "max_hole_area_m2": .1, "headroom_m": .5, "max_slope": .4})
    assert client.calls == [("get_surface_map", {"map_id": "hall", "radius_m": .3,
        "max_hole_area_m2": .1, "headroom_m": .5, "max_slope": .4})]
