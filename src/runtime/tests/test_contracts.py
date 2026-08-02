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
    GNSS_BACKEND_DDS,
    GNSS_BACKEND_HW,
    GNSS_BACKEND_REPLAY,
    GNSS_BACKEND_WTRTK980,
    GNSS_CONTRACT,
    GNSS_PORTS,
    GNSS_ROLE,
    HW_CONFIG_BRIDGE,
    HW_CONFIG_ENABLE,
    HW_CONTRACT,
    HW_PORTS,
    HW_ROLE,
    IMU_BACKEND_DDS,
    IMU_BACKEND_LIVOX,
    IMU_BACKEND_MUJOCO,
    IMU_BACKEND_REPLAY,
    IMU_CONTRACT,
    IMU_PORTS,
    IMU_ROLE,
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
    default_runtime_contract_registry,
    validate_message,
)
from runtime.runtime_interface import (
    REAL_RUNTIME_CONTRACT,
    THUNDER_LITE_RUNTIME_CONTRACT,
    TOPICS,
    adapter_aliases,
    runtime_contract_manifest,
    topic_formats,
    topic_ros_types,
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


def test_hw_contract_defines_generic_inventory_boundary():
    assert HW_ROLE == "hw"
    assert HW_CONTRACT.alias == "hw"
    assert HW_CONTRACT.compat_aliases == ()
    assert HW_CONTRACT.config_keys == (HW_CONFIG_ENABLE, HW_CONFIG_BRIDGE)
    assert HW_CONTRACT.compat_config_keys == ()
    assert HW_PORTS == ("device_status", "device_event", "alive")


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


def test_imu_contract_defines_generic_motion_sensor_boundary():
    assert IMU_ROLE == "imu"
    assert IMU_CONTRACT.alias == "imu"
    assert IMU_CONTRACT.compat_aliases == ()
    assert IMU_CONTRACT.backends == (
        IMU_BACKEND_LIVOX,
        IMU_BACKEND_MUJOCO,
        IMU_BACKEND_DDS,
        IMU_BACKEND_REPLAY,
    )
    assert IMU_PORTS == ("imu", "alive")
    assert IMU_CONTRACT.to_dict()["stream_ports"] == ["imu"]


def test_gnss_contract_defines_generic_position_sensor_boundary():
    assert GNSS_ROLE == "gnss"
    assert GNSS_CONTRACT.alias == "gnss"
    assert "compat_aliases" not in GNSS_CONTRACT.to_dict()
    assert "compat_config_keys" not in GNSS_CONTRACT.to_dict()
    assert GNSS_CONTRACT.backends == (
        GNSS_BACKEND_WTRTK980,
        GNSS_BACKEND_HW,
        GNSS_BACKEND_DDS,
        GNSS_BACKEND_REPLAY,
    )
    assert GNSS_PORTS == (
        "rtcm_bytes",
        "gnss_fix",
        "gnss_status",
        "gnss_odom",
        "alive",
    )
    assert GNSS_CONTRACT.to_dict()["stream_ports"] == [
        "gnss_fix",
        "gnss_status",
        "gnss_odom",
    ]
    health_fields = set(GNSS_CONTRACT.to_dict()["health_fields"])
    assert {
        "role",
        "dataflow_owner",
        "product_owner",
        "python_compat",
        "serial_port",
        "uses_hw_inventory",
        "requires_hw_bridge",
        "dds_compat_reader",
        "replay_source",
        "direct_serial",
    } <= health_fields


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
    registry = default_runtime_contract_registry()
    assert (
        registry.validate_envelope(
            {
                "topic": TOPICS.height_rays,
                "message_contract": "height_rays",
                "frame_id": "body",
                "payload": payload,
            }
        )
        == []
    )

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


def test_runtime_contract_registry_exports_json_ready_manifest():
    registry = default_runtime_contract_registry()
    manifest = registry.manifest()

    assert manifest["schema_version"] == "lingtu.runtime_interface.v1"
    assert manifest["contract_registry_schema_version"] == registry.schema_version
    assert isinstance(manifest["core_required_topics"], list)
    assert manifest["topic_ros_types"][TOPICS.cmd_vel] == [
        "lingtu.dds.FinalVelocityCommand",
    ]
    assert manifest["message_contracts"]["mission_status"]["envelope"]["type"] == ("mission_status")
    assert registry.validate_manifest(manifest) == []


def test_runtime_contract_registry_resolves_topic_and_data_source_contracts():
    registry = default_runtime_contract_registry()

    cmd_vel = registry.topic_contract(TOPICS.cmd_vel)
    assert cmd_vel.formats == ("cmd_vel",)
    assert cmd_vel.default_frame_id == "body"
    assert registry.topic_ros_types(TOPICS.cmd_vel) == (
        "lingtu.dds.FinalVelocityCommand",
    )
    assert registry.topic_ros_types(TOPICS.map_cloud) == ("sensor_msgs/msg/PointCloud2",)
    assert topic_ros_types(TOPICS.odometry) == ("nav_msgs/msg/Odometry",)
    assert topic_ros_types(TOPICS.lidar_scan) == (
        "livox_ros_driver2/msg/CustomMsg",
        "sensor_msgs/msg/PointCloud2",
    )
    assert topic_formats(TOPICS.raw_imu) == ("lingtu.dds.Imu",)
    assert topic_ros_types(TOPICS.raw_imu) == ("sensor_msgs/msg/Imu",)
    assert topic_formats(TOPICS.camera_color) == ("lingtu.dds.Image",)
    assert topic_formats(TOPICS.camera_depth) == ("lingtu.dds.Image",)
    assert topic_formats(TOPICS.camera_info) == ("lingtu.dds.CameraInfo",)
    from runtime.runtime_interface import MESSAGE_FORMATS

    assert "depth_scale" in MESSAGE_FORMATS["lingtu.dds.CameraInfo"].required_fields
    assert topic_ros_types(TOPICS.camera_color) == ("sensor_msgs/msg/Image",)
    assert topic_ros_types(TOPICS.camera_depth) == ("sensor_msgs/msg/Image",)
    assert topic_ros_types(TOPICS.camera_info) == ("sensor_msgs/msg/CameraInfo",)
    assert topic_ros_types(TOPICS.height_rays) == ("application/json",)
    assert runtime_contract_manifest()["topic_ros_types"][TOPICS.map_cloud] == ("sensor_msgs/msg/PointCloud2",)
    assert topic_formats(TOPICS.maps_scene) == ("maps_scene",)
    assert topic_ros_types(TOPICS.maps_scene) == ("lingtu.dds.MapScene",)
    maps_scene = registry.topic_contract(TOPICS.maps_scene)
    assert maps_scene.allowed_frame_ids == ("map", "odom")
    assert maps_scene.default_frame_id == "map"
    assert registry.topic_contract(
        TOPICS.maps_scene,
        runtime_contract=REAL_RUNTIME_CONTRACT,
    ).allowed_frame_ids == ("map",)

    height_rays = registry.topic_contract(TOPICS.height_rays)
    assert height_rays.formats == ("height_rays",)
    assert height_rays.allowed_frame_ids == ("body",)
    assert height_rays.default_frame_id == "body"

    mujoco = registry.data_source_contract("mujoco_module_graph")
    assert TOPICS.height_rays in mujoco.normalized_outputs
    assert mujoco.algorithm_context_outputs == (TOPICS.height_rays,)

    current = registry.data_source_contract(REAL_RUNTIME_CONTRACT)
    assert current.name == REAL_RUNTIME_CONTRACT

    lite_cmd_vel = registry.topic_contract(
        TOPICS.cmd_vel,
        runtime_contract=THUNDER_LITE_RUNTIME_CONTRACT,
    )
    assert lite_cmd_vel.allowed_frame_ids == ("body",)
    assert lite_cmd_vel.default_frame_id == "body"


def test_clearing_topics_are_native_bool_with_ros_compat_aliases():
    assert topic_formats(TOPICS.map_clearing) == ("lingtu.dds.Bool",)
    assert topic_formats(TOPICS.cloud_clearing) == ("lingtu.dds.Bool",)
    assert topic_ros_types(TOPICS.map_clearing) == ("std_msgs/msg/Bool",)
    assert topic_ros_types(TOPICS.cloud_clearing) == ("std_msgs/msg/Bool",)

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


def test_runtime_contract_registry_validates_lcm_style_envelope():
    registry = default_runtime_contract_registry()
    envelope = {
        "format": "lingtu.transport.json.v1",
        "topic": TOPICS.cmd_vel,
        "schema": "lingtu.nav.mission_status.v1",
        "message_contract": "mission_status",
        "payload": {
            "state": "EXECUTING",
            "replan_count": 1,
            "wp_index": 2,
            "wp_total": 5,
            "speed_scale": 0.7,
            "degeneracy": "MILD",
            "ts": 10.0,
        },
    }

    assert registry.validate_envelope(envelope) == []


def test_runtime_contract_registry_reuses_payload_validator():
    registry = default_runtime_contract_registry()
    payload = {"state": "LOST"}

    direct = validate_message("localization_status", payload)
    wrapped = registry.validate_payload("localization_status", payload)
    assert wrapped == direct

    envelope_issues = registry.validate_envelope(
        {
            "type": "localization_status",
            "payload": payload,
        }
    )
    assert envelope_issues == direct


def test_runtime_contract_registry_reports_manifest_issues():
    registry = default_runtime_contract_registry()
    manifest = registry.manifest()
    manifest["schema_version"] = "bad"
    manifest["topic_formats"][TOPICS.cmd_vel] = []
    manifest["data_sources"][REAL_RUNTIME_CONTRACT]["name"] = "wrong"

    issues = registry.validate_manifest(manifest)

    assert "schema_version must be lingtu.runtime_interface.v1" in issues
    assert any("topic_formats" in issue for issue in issues)
    assert any("data_sources" in issue for issue in issues)


def test_runtime_contract_registry_reports_unknown_envelope_contract():
    registry = default_runtime_contract_registry()

    issues = registry.validate_envelope(
        {
            "type": "unknown_status",
            "payload": {},
        }
    )

    assert issues[0].code == "unknown_contract"
