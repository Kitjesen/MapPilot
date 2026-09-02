import pytest

from runtime.contracts import (
    CAMERA_BACKEND_DDS,
    CAMERA_BACKEND_ORBBEC,
    CAMERA_BACKEND_REPLAY,
    CAMERA_BACKEND_SIM,
    CAMERA_CONFIG_FORCE,
    CAMERA_CONTRACT,
    CAMERA_PORTS,
    CAMERA_ROLE,
    LIDAR_BACKEND_DDS,
    LIDAR_BACKEND_MID360,
    LIDAR_BACKEND_MUJOCO,
    LIDAR_BACKEND_REPLAY,
    LIDAR_CONTRACT,
    LIDAR_PORTS,
    LIDAR_ROLE,
    ContractError,
    MessageEnvelope,
    assert_valid_message,
    validate_message,
)
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    REAL_RUNTIME_CONTRACT,
    TOPICS,
    adapter_aliases,
    runtime_contract_manifest,
    runtime_topic_allowed_frame_ids,
    topic_formats,
)


def test_camera_contract_defines_generic_stream_boundary():
    assert CAMERA_ROLE == "camera"
    assert CAMERA_CONTRACT.alias == "camera"
    assert CAMERA_CONTRACT.config_keys == (CAMERA_CONFIG_FORCE,)
    assert "compat_aliases" not in CAMERA_CONTRACT.to_dict()
    assert "compat_config_keys" not in CAMERA_CONTRACT.to_dict()
    assert CAMERA_CONTRACT.backends == (
        CAMERA_BACKEND_ORBBEC,
        CAMERA_BACKEND_REPLAY,
        CAMERA_BACKEND_SIM,
        CAMERA_BACKEND_DDS,
    )
    assert CAMERA_PORTS == ("color_image", "depth_image", "camera_info", "alive")
    assert CAMERA_CONTRACT.to_dict()["stream_ports"] == [
        "color_image",
        "depth_image",
        "camera_info",
    ]


def test_lidar_contract_defines_generic_raw_sensor_boundary():
    assert LIDAR_ROLE == "lidar"
    assert LIDAR_CONTRACT.alias == "lidar"
    assert "compat_aliases" not in LIDAR_CONTRACT.to_dict()
    assert "compat_config_keys" not in LIDAR_CONTRACT.to_dict()
    assert LIDAR_CONTRACT.backends == (
        LIDAR_BACKEND_MID360,
        LIDAR_BACKEND_MUJOCO,
        LIDAR_BACKEND_DDS,
        LIDAR_BACKEND_REPLAY,
    )
    assert LIDAR_PORTS == ("scan", "raw_scan", "imu", "alive")
    assert LIDAR_CONTRACT.to_dict()["stream_ports"] == [
        "scan",
        "raw_scan",
        "imu",
    ]


def test_mission_status_contract_accepts_navigation_payload():
    assert_valid_message(
        "mission_status",
        {
            "state": "EXECUTING",
            "replan_count": 1,
            "wp_index": 2,
            "wp_total": 5,
            "speed_scale": 0.7,
            "degeneracy": "MILD",
            "ts": 10.0,
        },
    )


def test_contract_reports_missing_required_fields():
    issues = validate_message("localization_status", {"state": "TRACKING"})

    assert {issue.path for issue in issues} >= {"confidence", "degeneracy", "ts"}
    assert any(issue.code == "missing" for issue in issues)


def test_contract_reports_schema_version_mismatch():
    issues = validate_message(
        "fused_cost",
        {
            "schema_version": 999,
            "grid": [[0.0]],
            "resolution": 0.2,
            "origin": [0.0, 0.0],
            "ts": 1.0,
        },
    )

    assert any(issue.code == "version_mismatch" for issue in issues)


def test_contract_reports_illegal_values():
    issues = validate_message(
        "mission_status",
        {
            "state": "DRIVING",
            "replan_count": -1,
            "wp_index": 0,
            "wp_total": 1,
            "speed_scale": 1.5,
            "degeneracy": "NONE",
            "ts": 1.0,
        },
    )

    assert any(issue.path == "state" and issue.code == "invalid_value" for issue in issues)
    assert any(issue.path == "speed_scale" and issue.code == "out_of_range" for issue in issues)
    assert any(issue.path == "replan_count" and issue.code == "out_of_range" for issue in issues)


def test_scene_graph_contract_accepts_minimal_graph_dict():
    assert_valid_message(
        "scene_graph",
        {
            "objects": [],
            "relations": [],
            "regions": [],
            "frame_id": "map",
            "ts": 1.0,
        },
    )


def test_height_rays_contract_accepts_fixed_ray_payload():
    payload = {
        "heights": [0.55, 0.56],
        "points_body": [[0.0, 0.0, -0.55], [0.1, 0.0, -0.56]],
        "points_world": [[1.0, 2.0, 0.0], [1.1, 2.0, 0.0]],
        "valid_mask": [True, True],
        "frame_id": "body",
        "ts": 1.0,
    }

    assert_valid_message("height_rays", payload)

    bad = {**payload, "points_world": [[1.0, 2.0, 0.0]]}
    issues = validate_message("height_rays", bad)
    assert any(issue.path == "points_world" and issue.code == "shape_mismatch" for issue in issues)


def test_traversability_contract_accepts_risk_grid_payload():
    assert_valid_message(
        "traversability",
        {
            "grid": [[0.0, 70.0], [20.0, 0.0]],
            "resolution": 1.0,
            "origin": [0.0, 0.0],
            "ts": 1.0,
        },
    )


def test_traversability_contract_keeps_status_payload_valid():
    assert_valid_message("traversability", {"status": "passthrough"})


def test_assert_valid_message_raises_with_issue_detail():
    with pytest.raises(ContractError, match="confidence"):
        assert_valid_message("localization_status", {"state": "LOST"})


def test_message_envelope_wraps_existing_dict_payload_without_changing_contract():
    envelope = MessageEnvelope.from_payload(
        "mission_status",
        {
            "state": "EXECUTING",
            "replan_count": 1,
            "wp_index": 2,
            "wp_total": 5,
            "speed_scale": 0.7,
            "degeneracy": "MILD",
            "ts": 10.0,
        },
        frame_id="map",
    )

    assert envelope.validate() == []
    assert envelope.to_dict()["type"] == "mission_status"
    assert envelope.to_dict()["payload"]["state"] == "EXECUTING"
    assert envelope.to_dict()["frame_id"] == "map"


def test_runtime_interface_resolves_topics_and_data_sources_directly():
    manifest = runtime_contract_manifest()
    assert manifest["schema_version"] == "lingtu.runtime_interface.v1"
    assert topic_formats(TOPICS.cmd_vel) == ("cmd_vel",)
    assert runtime_topic_allowed_frame_ids(None)[TOPICS.cmd_vel] == ("body",)
    assert topic_formats(TOPICS.raw_imu) == ("lingtu.dds.Imu",)
    assert topic_formats(TOPICS.camera_color) == ("lingtu.dds.Image",)
    assert topic_formats(TOPICS.camera_depth) == ("lingtu.dds.Image",)
    assert topic_formats(TOPICS.camera_info) == ("lingtu.dds.CameraInfo",)
    from runtime.runtime_interface import MESSAGE_FORMATS

    assert "depth_scale" in MESSAGE_FORMATS["lingtu.dds.CameraInfo"].required_fields
    assert "topic_ros_types" not in manifest
    assert topic_formats(TOPICS.maps_scene) == ("maps_scene",)
    assert runtime_topic_allowed_frame_ids(None)[TOPICS.maps_scene] == ("map", "odom")
    assert runtime_topic_allowed_frame_ids(REAL_RUNTIME_CONTRACT)[TOPICS.maps_scene] == ("map",)
    assert topic_formats(TOPICS.height_rays) == ("height_rays",)
    assert runtime_topic_allowed_frame_ids(None)[TOPICS.height_rays] == ("body",)

    mujoco = DATA_SOURCE_CONTRACTS["mujoco_module_graph"]
    assert TOPICS.height_rays in mujoco.normalized_outputs
    assert mujoco.algorithm_context_outputs == (TOPICS.height_rays,)
    assert DATA_SOURCE_CONTRACTS["field"].name == "field"

def test_clearing_topics_keep_explicit_legacy_adapter_aliases():
    assert topic_formats(TOPICS.map_clearing) == ("lingtu.dds.Bool",)
    assert topic_formats(TOPICS.cloud_clearing) == ("lingtu.dds.Bool",)

    assert topic_formats("/map_clearing") == ("std_msgs/msg/Bool",)
    assert topic_formats("/cloud_clearing") == ("std_msgs/msg/Bool",)

    terrain_aliases = adapter_aliases("terrain_analysis")
    map_alias = next(alias for alias in terrain_aliases if alias.target == TOPICS.map_clearing)
    assert map_alias.source == "/map_clearing"
    assert map_alias.msg_format == "std_msgs/msg/Bool"

    terrain_ext_aliases = adapter_aliases("terrain_analysis_ext")
    cloud_alias = next(alias for alias in terrain_ext_aliases if alias.target == TOPICS.cloud_clearing)
    assert cloud_alias.source == "/cloud_clearing"
    assert cloud_alias.msg_format == "std_msgs/msg/Bool"
