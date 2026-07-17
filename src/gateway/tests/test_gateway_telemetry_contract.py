from __future__ import annotations

import asyncio
import json
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str, method: str | None = None):
    for route in gateway._app.routes:
        if route.path != path:
            continue
        if method is None or method.upper() in getattr(route, "methods", set()):
            return route.endpoint
    raise AssertionError(f"route not found: {method or '*'} {path}")


def test_scene_graph_path_and_locations_routes_validate_response_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationsResponse, PathResponse, SceneGraphResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._sg_json = (
            '{"frame_id":"map","ts":123.0,'
            '"objects":[{"id":"obj-1","label":"dock","x":1.0,"y":2.0,'
            '"confidence":0.9}],'
            '"relations":[{"source":"obj-1","target":"region-1",'
            '"relation":"inside"}],'
            '"regions":[{"id":"region-1","name":"yard"}]}'
        )
        gateway._last_path = [
            {"x": 0.0, "y": 0.0, "frame_id": "map"},
            (1.0, 1.0, 0.1),
        ]
        gateway._odom = {"x": 0.5, "y": 0.25, "yaw": 0.2, "vx": 0.1}
    gateway._tagged_loc_module = SimpleNamespace(
        store=SimpleNamespace(
            _store={
                "dock": {
                    "name": "dock",
                    "x": 1.23456,
                    "y": 2.34567,
                    "z": 0.0,
                    "yaw": 0.4,
                    "tags": ["charger", "home"],
                    "source": "test",
                    "ts": 456.0,
                },
                "bad": {"x": 9.0, "y": 9.0},
            }
        )
    )

    scene_payload = asyncio.run(_endpoint(gateway, "/api/v1/scene_graph")())
    path_payload = asyncio.run(_endpoint(gateway, "/api/v1/path")())
    locations_payload = asyncio.run(_endpoint(gateway, "/api/v1/locations")())

    scene = SceneGraphResponse.model_validate(scene_payload)
    path = PathResponse.model_validate(path_payload)
    locations = LocationsResponse.model_validate(locations_payload)

    assert scene.schema_version == 1
    assert scene.frame_id == "map"
    assert scene.ts == 123.0
    assert scene.count == 1
    assert scene.objects[0].id == "obj-1"
    assert scene.objects[0].label == "dock"
    assert scene.objects[0].confidence == 0.9
    assert scene.relations[0].relation == "inside"
    assert scene.regions[0].name == "yard"
    assert scene.scene_graph == gateway._sg_json
    assert path.schema_version == 1
    assert len(path.path) == 2
    assert path.count == 2
    assert path.path[0].frame_id == "map"
    assert path.path[1].z == 0.1
    assert path.robot is not None
    assert path.robot.x == 0.5
    assert path.robot.y == 0.25
    assert path.robot.yaw == 0.2
    assert path.source == "gateway_cache"
    assert locations.schema_version == 1
    assert locations.count == 1
    assert len(locations.locations) == 1
    assert locations.locations[0].name == "dock"
    assert locations.locations[0].x == 1.235
    assert locations.locations[0].y == 2.346
    assert locations.locations[0].yaw == 0.4
    assert locations.locations[0].tags == ["charger", "home"]
    assert locations.locations[0].source == "test"


def test_map_event_is_forwarded_to_sse_payload():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    events: list[dict] = []
    gateway.push_event = events.append

    gateway._on_map_event(
        {
            "schema_version": "map.event.v1",
            "event": "map.active_changed",
            "map_id": "field_01",
        }
    )

    assert "map_event" in gateway.port_summary()["ports_in"]
    assert events == [
        {
            "type": "map_event",
            "data": {
                "schema_version": "map.event.v1",
                "event": "map.active_changed",
                "map_id": "field_01",
            },
        }
    ]


def test_scene_graph_route_degrades_invalid_json_to_empty_structured_payload():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SceneGraphResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._sg_json = "{not-json"

    scene_payload = asyncio.run(_endpoint(gateway, "/api/v1/scene_graph")())
    scene = SceneGraphResponse.model_validate(scene_payload)

    assert scene.scene_graph == "{not-json"
    assert scene.objects == []
    assert scene.relations == []
    assert scene.regions == []
    assert scene.count == 0


def test_path_route_accepts_core_path_messages_without_dropping_points():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PathResponse
    from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
    from runtime.msgs.nav import Path

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._last_path = Path(
            poses=[
                PoseStamped(
                    pose=Pose(
                        position=Vector3(1.0, 2.0, 0.0),
                        orientation=Quaternion.from_yaw(0.25),
                    ),
                    frame_id="map",
                    ts=100.0,
                ),
                PoseStamped(
                    pose=Pose(position=Vector3(2.0, 3.0, 0.1)),
                    frame_id="map",
                    ts=101.0,
                ),
            ],
            frame_id="map",
        )
        gateway._odom = {"x": 0.5, "y": 0.25, "frame_id": "map"}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/path")())
    path = PathResponse.model_validate(payload)

    assert path.schema_version == 1
    assert path.frame_id == "map"
    assert path.count == 2
    assert path.path[0].x == 1.0
    assert path.path[0].y == 2.0
    assert path.path[0].yaw == pytest.approx(0.25)
    assert path.path[0].frame_id == "map"
    assert path.path[1].z == 0.1


def test_navigation_dds_snapshot_exposes_paths_and_muxed_cmd_vel():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationDdsSnapshotResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._last_path = [{"x": 1.0, "y": 2.0, "z": 0.3, "frame_id": "map"}]
        gateway._last_local_path = [{"x": 0.2, "y": 0.1, "z": 0.0, "frame_id": "map"}]
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "map"}
        gateway._mode = "auto"
        gateway._mission = {"state": "EXECUTING", "ts": 123.0}

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {},
                "last_driver_cmd_vel": {
                    "linear": {"x": 0.4, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.2},
                    "ts": 456.0,
                    "active_source": "path_follower",
                },
            }

    gateway._cmd_vel_mux = FakeMux()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/dds_snapshot")())
    snapshot = NavigationDdsSnapshotResponse.model_validate(payload)

    assert snapshot.schema_version == "lingtu.navigation.dds_snapshot.v1"
    assert snapshot.global_path.count == 1
    assert snapshot.global_path.path[0].z == 0.3
    assert snapshot.local_path.count == 1
    assert snapshot.cmd_vel is not None
    assert snapshot.cmd_vel.linear["x"] == 0.4
    assert snapshot.cmd_vel.angular["z"] == 0.2
    assert snapshot.cmd_vel.active_source == "path_follower"


def test_navigation_dds_snapshot_reads_native_endpoint_status(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationDdsSnapshotResponse

    nav_status = tmp_path / "nav_endpoint_status.json"
    trav_status = tmp_path / "traversability_status.json"
    nav_status.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.nav.endpoint.status.v1",
                "stamp_s": 789.0,
                "global_path": [[1.0, 2.0, 0.3], [3.0, 4.0, 0.1]],
                "local_path": [[0.2, 0.1, 0.0]],
                "last_local": {"cmd_vel": {"vx": 0.3, "vy": 0.0, "wz": -0.2}},
                "counters": {"odom": 3, "goals": 1, "outputs": 2},
                "has_odom": True,
                "has_traversability": True,
            }
        ),
        encoding="utf-8",
    )
    trav_status.write_text(
        '{"schema_version":"lingtu.traversability.status.v1",'
        '"counters":{"odom":3,"registered_clouds":4,"published":2}}',
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(nav_status))
    monkeypatch.setenv("LINGTU_TRAVERSABILITY_STATUS_FILE", str(trav_status))

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/dds_snapshot")())
    snapshot = NavigationDdsSnapshotResponse.model_validate(payload)

    assert snapshot.nav_endpoint is not None
    assert snapshot.nav_endpoint["counters"]["goals"] == 1
    assert snapshot.global_path.count == 2
    assert snapshot.global_path.path[0].frame_id == "map"
    assert snapshot.local_path.count == 1
    assert snapshot.cmd_vel is not None
    assert snapshot.cmd_vel.linear["x"] == 0.3
    assert snapshot.cmd_vel.angular["z"] == -0.2
    assert snapshot.cmd_vel.active_source == "native_nav_endpoint_preview"
    assert snapshot.traversability_endpoint is not None
    assert snapshot.traversability_endpoint["counters"]["published"] == 2
    assert snapshot.source == "gateway_navigation_cache+native_status"


def test_navigation_dds_snapshot_prefers_native_final_teleop_command(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationDdsSnapshotResponse

    nav_status = tmp_path / "nav_endpoint_status.json"
    nav_status.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.nav.endpoint.status.v1",
                "stamp_s": time.time(),
                "control_mode": "teleop",
                "publish_cmd_vel": True,
                "final_cmd_vel": {"vx": 0.25, "vy": -0.05, "wz": 0.3},
                "teleop": {"fresh": True, "published": True},
                "last_local": {"cmd_vel": {"vx": 0.0, "vy": 0.0, "wz": 0.0}},
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(nav_status))
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    gateway = GatewayModule()
    gateway.setup()
    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/dds_snapshot")())
    snapshot = NavigationDdsSnapshotResponse.model_validate(payload)

    assert snapshot.cmd_vel is not None
    assert snapshot.cmd_vel.linear["x"] == 0.25
    assert snapshot.cmd_vel.linear["y"] == -0.05
    assert snapshot.cmd_vel.angular["z"] == 0.3
    assert snapshot.cmd_vel.active_source == "native_teleop"
    assert gateway._teleop_active is True


def test_native_autonomy_takeover_is_reported_as_active_teleop(
    monkeypatch,
    tmp_path,
):
    from gateway.gateway_module import GatewayModule

    nav_status = tmp_path / "nav_endpoint_status.json"
    nav_status.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.nav.endpoint.status.v1",
                "stamp_s": time.time(),
                "control_mode": "autonomy",
                "active_cmd_source": "teleop",
                "publish_cmd_vel": True,
                "control_authority": {
                    "operator_takeover_latched": True,
                    "resume_required": True,
                },
                "final_cmd_vel": {"vx": 0.1, "vy": 0.0, "wz": 0.0},
                "teleop": {"fresh": True, "published": True},
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(nav_status))
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "30")

    gateway = GatewayModule()
    gateway.setup()

    assert gateway._teleop_active is True


def test_velocity_mux_health_includes_latest_driver_command():
    from nav.services.safety.velocity_mux import VelocityMux
    from runtime.msgs.geometry import Twist, Vector3

    mux = VelocityMux()
    mux.setup()
    mux._on_source(
        "path_follower",
        Twist(
            linear=Vector3(0.3, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.1),
        ),
    )

    health = mux.health()

    assert health["active_source"] == "path_follower"
    assert health["last_driver_cmd_vel"]["linear"]["x"] == 0.3
    assert health["last_driver_cmd_vel"]["angular"]["z"] == 0.1
    assert health["last_driver_cmd_vel"]["active_source"] == "path_follower"


def test_locations_route_is_empty_without_tagged_location_module():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationsResponse

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/locations")())
    locations = LocationsResponse.model_validate(payload)

    assert locations.locations == []
    assert locations.count == 0


def test_locations_route_reads_real_tagged_location_store_shape():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationsResponse
    from memory.spatial.tagged_locations import TaggedLocationStore

    store = TaggedLocationStore()
    store.tag(
        "dock",
        x=1.23456,
        y=2.34567,
        z=0.1,
        yaw=0.4,
        tags=["charger"],
        source="test",
        metadata={"floor": "1f"},
    )
    gateway = GatewayModule()
    gateway.setup()
    gateway._tagged_loc_module = SimpleNamespace(store=store)

    payload = asyncio.run(_endpoint(gateway, "/api/v1/locations", "GET")())
    locations = LocationsResponse.model_validate(payload)

    assert locations.count == 1
    assert locations.locations[0].name == "dock"
    assert locations.locations[0].x == 1.235
    assert locations.locations[0].y == 2.346
    assert locations.locations[0].tags == ["charger"]
    assert locations.locations[0].source == "test"


def test_tagged_location_store_persists_tag_and_remove(tmp_path):
    from memory.spatial.tagged_locations import TaggedLocationStore

    path = tmp_path / "locations.json"
    store = TaggedLocationStore(str(path))

    store.tag(
        "dock",
        x=1.0,
        y=2.0,
        z=0.1,
        yaw=0.25,
        tags=["charger"],
        source="web",
        ts=123.0,
        metadata={"floor": "1f"},
    )

    reloaded = TaggedLocationStore(str(path))
    entry = reloaded.query("dock")
    assert entry is not None
    assert entry["position"] == [1.0, 2.0, 0.1]
    assert entry["yaw"] == 0.25
    assert entry["tags"] == ["charger"]
    assert entry["source"] == "web"
    assert entry["ts"] == 123.0
    assert entry["metadata"] == {"floor": "1f"}

    assert reloaded.remove("dock") is True
    removed = TaggedLocationStore(str(path))
    assert removed.query("dock") is None


def test_location_write_routes_save_delete_and_emit_events():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationOperationResponse, LocationUpsertRequest
    from memory.spatial.tagged_locations import TaggedLocationStore

    statuses: list[str] = []
    events: list[dict] = []
    gateway = GatewayModule()
    gateway.setup()
    gateway._tagged_loc_module = SimpleNamespace(
        store=TaggedLocationStore(),
        tag_status=SimpleNamespace(publish=statuses.append),
    )
    gateway.push_event = events.append

    body = LocationUpsertRequest(
        name="dock",
        x=1.0,
        y=2.0,
        yaw=0.25,
        tags=["charger", "home"],
        source="test",
        request_id="req-1",
        client_id="client-1",
    )
    saved_payload = asyncio.run(_endpoint(gateway, "/api/v1/locations", "POST")(body))
    saved = LocationOperationResponse.model_validate(saved_payload)

    assert saved.ok is True
    assert saved.status == "saved"
    assert saved.action == "create"
    assert saved.location is not None
    assert saved.location.name == "dock"
    assert saved.locations.count == 1
    assert statuses[-1] == "saved:dock"
    assert [event["type"] for event in events[-2:]] == ["location", "locations"]

    deleted_payload = asyncio.run(_endpoint(gateway, "/api/v1/locations/{name}", "DELETE")("dock"))
    deleted = LocationOperationResponse.model_validate(deleted_payload)

    assert deleted.ok is True
    assert deleted.status == "deleted"
    assert deleted.locations.count == 0
    assert statuses[-1] == "removed:dock"


def test_location_update_route_uses_current_pose_and_preserves_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationOperationResponse, LocationUpsertRequest
    from memory.spatial.tagged_locations import TaggedLocationStore

    gateway = GatewayModule()
    gateway.setup()
    gateway._tagged_loc_module = SimpleNamespace(
        store=TaggedLocationStore(),
        tag_status=SimpleNamespace(publish=lambda _msg: None),
    )
    with gateway._state_lock:
        gateway._odom = {"x": 3.25, "y": -1.5, "z": 0.2, "yaw": 1.1}

    gateway._tagged_loc_module.store.tag("dock", x=0.0, y=0.0, tags=["old"])
    body = LocationUpsertRequest(
        name="dock",
        use_current_pose=True,
        tags=["charger"],
        source="web",
        request_id="req-update",
        client_id="web-client",
    )

    updated_payload = asyncio.run(_endpoint(gateway, "/api/v1/locations/{name}", "PUT")("dock", body))
    updated = LocationOperationResponse.model_validate(updated_payload)

    assert updated.ok is True
    assert updated.status == "saved"
    assert updated.action == "update"
    assert updated.request_id == "req-update"
    assert updated.client_id == "web-client"
    assert updated.location is not None
    assert updated.location.x == 3.25
    assert updated.location.y == -1.5
    assert updated.location.z == 0.2
    assert updated.location.yaw == 1.1
    assert updated.location.tags == ["charger"]
    assert updated.locations.count == 1


def test_location_update_rejects_name_mismatch_without_mutating_store():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationOperationResponse, LocationUpsertRequest
    from memory.spatial.tagged_locations import TaggedLocationStore

    gateway = GatewayModule()
    gateway.setup()
    store = TaggedLocationStore()
    store.tag("dock", x=1.0, y=2.0, tags=["original"])
    gateway._tagged_loc_module = SimpleNamespace(store=store)

    body = LocationUpsertRequest(
        name="other",
        x=8.0,
        y=9.0,
        request_id="req-mismatch",
        client_id="web-client",
    )
    response = asyncio.run(_endpoint(gateway, "/api/v1/locations/{name}", "PUT")("dock", body))
    payload = json.loads(response.body)
    rejected = LocationOperationResponse.model_validate(payload)

    assert response.status_code == 400
    assert rejected.ok is False
    assert rejected.status == "invalid"
    assert rejected.error == "location_name_mismatch"
    assert rejected.request_id == "req-mismatch"
    assert rejected.locations.count == 1
    assert store.query("dock")["position"] == [1.0, 2.0, 0.0]
    assert store.query("other") is None
