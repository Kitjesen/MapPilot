from __future__ import annotations

import pytest

from runtime.msgs.geometry import Pose, Quaternion, Transform, Vector3
from runtime.msgs.map import (
    MAP_CLOUD_FRAME_SCHEMA,
    MAP_CONTROL_REQUEST_SCHEMA,
    MAP_OBSERVATION_FRAME_SCHEMA,
    MAP_SCENE_FRAME_SCHEMA,
    MapCloudFrame,
    MapControlRequest,
    MapObservationFrame,
    MapSceneFrame,
    SemanticLabelsFrame,
    SemanticSaveRequest,
    SemanticSaveResult,
)
from runtime.msgs.numpy_compat import np
from runtime.msgs.protocol import resolve_msg_type
from runtime.msgs.sensor import PointCloud2


def test_map_cloud_frame_encode_round_trip():
    frame = MapCloudFrame(
        points=np.array([[1.0, 2.0, 0.1, 7.0]], dtype=np.float32),
        mode="keyframe",
        frame_id="map",
        map_id="field_01",
        source="slam",
        sequence=3,
        metadata={"quality": "ok"},
    )

    decoded = MapCloudFrame.decode(frame.encode())

    assert decoded.schema_version == MAP_CLOUD_FRAME_SCHEMA
    assert decoded.mode == "KEYFRAME"
    assert decoded.frame_id == "map"
    assert decoded.map_id == "field_01"
    assert decoded.source == "slam"
    assert decoded.sequence == 3
    assert decoded.metadata == {"quality": "ok"}
    assert decoded.points.shape == (1, 4)
    assert decoded.points.dtype == np.float32


def test_map_control_request_is_typed_and_transport_round_trips():
    request = MapControlRequest.from_mapping(
        {
            "action": "save",
            "name": "warehouse",
            "request_id": "save-17",
            "activate_on_success": True,
        }
    )

    decoded = MapControlRequest.decode(request.encode())

    assert decoded.schema_version == MAP_CONTROL_REQUEST_SCHEMA
    assert decoded.action == "save"
    assert decoded.request_id == "save-17"
    assert decoded.params == {
        "name": "warehouse",
        "activate_on_success": True,
    }
    assert decoded.to_mapping()["request_id"] == "save-17"
    assert resolve_msg_type("map.MapControlRequest") is MapControlRequest


def test_map_control_request_rejects_untyped_payloads():
    with pytest.raises(ValueError, match="action is required"):
        MapControlRequest.from_mapping({"name": "warehouse"})
    with pytest.raises(TypeError, match="must be a mapping"):
        MapControlRequest.from_mapping("save")  # type: ignore[arg-type]


def test_map_cloud_frame_rejects_unknown_mode():
    with pytest.raises(ValueError, match="mode must be one of"):
        MapCloudFrame(points=[[0.0, 0.0, 0.0]], mode="delta")


def test_map_cloud_frame_from_pointcloud2_and_resolve_type():
    cloud = PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        frame_id="map",
    )

    frame = MapCloudFrame.from_pointcloud2(cloud, mode="incremental")

    assert frame.mode == "INCREMENTAL"
    assert frame.frame_id == "map"
    assert frame.finite_xyz().shape == (1, 3)
    assert resolve_msg_type("map.MapCloudFrame") is MapCloudFrame


def test_map_observation_frame_accepts_geometry_only_incremental_scan():
    transform = Transform(
        translation=Vector3(1.0, 2.0, 0.3),
        frame_id="map",
        child_frame_id="body",
    )
    frame = MapObservationFrame(
        points=np.array([[1.0, 0.0, 0.1]], dtype=np.float32),
        sequence=42,
        frame_id="map",
        sensor_frame_id="body",
        sensor_origin=transform.translation,
        map_sensor_pose=Pose(transform.translation, transform.rotation),
        map_sensor_transform=transform,
        pose_quality={"confidence": 0.91},
        source="native_slam:fastlio2:registered_cloud_body",
    )

    decoded = MapObservationFrame.decode(frame.encode())

    assert decoded.schema_version == MAP_OBSERVATION_FRAME_SCHEMA
    assert decoded.observation_kind == "INCREMENTAL"
    assert decoded.sequence == 42
    assert decoded.points.shape == (1, 3)
    assert decoded.sensor_origin.x == pytest.approx(1.0)
    assert decoded.map_sensor_transform.frame_id == "map"
    assert decoded.map_sensor_transform.child_frame_id == "body"
    assert decoded.pose_quality == {"confidence": 0.91}
    assert resolve_msg_type("map.MapObservationFrame") is MapObservationFrame


def test_map_observation_frame_transforms_sensor_points_exactly_once():
    half_sqrt = 2.0**-0.5
    transform = Transform(
        translation=Vector3(10.0, -2.0, 0.5),
        rotation=Quaternion(0.0, 0.0, half_sqrt, half_sqrt),
        frame_id="map",
        child_frame_id="lidar",
    )
    frame = MapObservationFrame(
        points=np.array([[1.0, 0.0, 0.0, 8.0]], dtype=np.float32),
        sequence=3,
        frame_id="map",
        sensor_frame_id="lidar",
        sensor_origin=transform.translation,
        map_sensor_pose=Pose(transform.translation, transform.rotation),
        map_sensor_transform=transform,
    )

    mapped = frame.to_map_pointcloud2()

    assert mapped.frame_id == "map"
    assert mapped.points[0, :3].tolist() == pytest.approx([10.0, -1.0, 0.5])
    assert mapped.points[0, 3] == pytest.approx(8.0)


def test_map_observation_frame_rejects_malformed_or_non_incremental_data():
    transform = Transform(frame_id="map", child_frame_id="body")
    kwargs = {
        "sequence": 1,
        "frame_id": "map",
        "sensor_frame_id": "body",
        "sensor_origin": transform.translation,
        "map_sensor_pose": Pose(transform.translation, transform.rotation),
        "map_sensor_transform": transform,
    }
    zero_sequence_kwargs = dict(kwargs)
    zero_sequence_kwargs["sequence"] = 0

    with pytest.raises(ValueError, match="only accepts INCREMENTAL"):
        MapObservationFrame(
            points=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            observation_kind="FULL",
            **kwargs,
        )
    with pytest.raises(ValueError, match="sequence must be a positive"):
        MapObservationFrame(
            points=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            **zero_sequence_kwargs,
        )
    with pytest.raises(ValueError, match="points must be finite"):
        MapObservationFrame(
            points=np.array([[float("nan"), 0.0, 0.0]], dtype=np.float32),
            **kwargs,
        )


def test_semantic_labels_frame_is_uint16_and_transport_round_trips():
    frame = SemanticLabelsFrame(
        labels=[0, 3, 65535],
        confidence=[0.0, 0.75, 1.0],
        sequence=42,
        ts=10.25,
        frame_id="body",
        taxonomy="lingtu.semantic",
        taxonomy_version=2,
        source="semantic_projection",
    )

    decoded = SemanticLabelsFrame.decode(frame.encode())

    assert decoded.labels.dtype == np.uint16
    assert decoded.labels.tolist() == [0, 3, 65535]
    assert decoded.confidence.tolist() == pytest.approx([0.0, 0.75, 1.0])
    assert decoded.sequence == 42
    assert decoded.frame_id == "body"
    assert decoded.taxonomy_version == 2
    assert resolve_msg_type("map.SemanticLabelsFrame") is SemanticLabelsFrame


def test_semantic_labels_frame_rejects_invalid_contract() -> None:
    with pytest.raises(ValueError, match="uint16"):
        SemanticLabelsFrame(
            labels=[-1],
            sequence=1,
            frame_id="body",
            taxonomy="lingtu.semantic",
            taxonomy_version=1,
        )
    with pytest.raises(ValueError, match="confidence"):
        SemanticLabelsFrame(
            labels=[1, 2],
            confidence=[0.5],
            sequence=1,
            frame_id="body",
            taxonomy="lingtu.semantic",
            taxonomy_version=1,
        )


def test_semantic_save_messages_round_trip_and_resolve() -> None:
    request = SemanticSaveRequest(
        request_id="req-7",
        map_id="warehouse",
        path="/maps/warehouse/semantic_map.bin",
    )
    result = SemanticSaveResult(
        request_id="req-7",
        map_id="warehouse",
        path=request.path,
        success=True,
        generation=9,
        voxel_count=123,
        message="committed",
    )

    assert SemanticSaveRequest.decode(request.encode()) == request
    assert SemanticSaveResult.decode(result.encode()) == result
    assert resolve_msg_type("map.SemanticSaveRequest") is SemanticSaveRequest
    assert resolve_msg_type("map.SemanticSaveResult") is SemanticSaveResult


def test_map_scene_frame_strips_heavy_payload_from_dict():
    cloud = PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        frame_id="map",
    )
    frame = MapSceneFrame(
        frame_id="map",
        source="maps.voxel",
        sequence=7,
        layers=[
            {
                "id": "maps.voxel_cloud",
                "type": "pointcloud",
                "payload": cloud,
            }
        ],
    )

    compact = frame.to_dict()
    full = frame.to_dict(include_payload=True)

    assert frame.schema_version == MAP_SCENE_FRAME_SCHEMA
    assert compact["layers"][0]["id"] == "maps.voxel_cloud"
    assert "payload" not in compact["layers"][0]
    assert full["layers"][0]["payload"] is cloud
    decoded = MapSceneFrame.decode(frame.encode())
    assert decoded.sequence == 7
    assert decoded.layers[0]["id"] == "maps.voxel_cloud"
    assert "payload" not in decoded.layers[0]
    assert resolve_msg_type("map.MapSceneFrame") is MapSceneFrame
