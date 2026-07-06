import pytest

from runtime.contracts import (
    ContractError,
    MessageEnvelope,
    assert_valid_message,
    default_runtime_contract_registry,
    validate_message,
)
from runtime.runtime_interface import LEGACY_REAL_RUNTIME_CONTRACT, REAL_RUNTIME_CONTRACT, TOPICS
from runtime.runtime_interface import THUNDER_LITE_RUNTIME_CONTRACT
from runtime.runtime_interface import (
    adapter_aliases,
    runtime_contract_manifest,
    topic_formats,
    topic_ros_types,
)


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
    assert registry.validate_envelope(
        {
            "topic": TOPICS.height_rays,
            "message_contract": "height_rays",
            "frame_id": "body",
            "payload": payload,
        }
    ) == []

    bad = {**payload, "points_world": [[1.0, 2.0, 0.0]]}
    issues = validate_message("height_rays", bad)
    assert any(
        issue.path == "points_world" and issue.code == "shape_mismatch"
        for issue in issues
    )


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
        "geometry_msgs/msg/TwistStamped",
    ]
    assert manifest["message_contracts"]["mission_status"]["envelope"]["type"] == (
        "mission_status"
    )
    assert registry.validate_manifest(manifest) == []


def test_runtime_contract_registry_resolves_topic_and_data_source_contracts():
    registry = default_runtime_contract_registry()

    cmd_vel = registry.topic_contract(TOPICS.cmd_vel)
    assert cmd_vel.formats == ("cmd_vel",)
    assert cmd_vel.default_frame_id == "body"
    assert registry.topic_ros_types(TOPICS.cmd_vel) == (
        "geometry_msgs/msg/TwistStamped",
    )
    assert registry.topic_ros_types(TOPICS.map_cloud) == (
        "sensor_msgs/msg/PointCloud2",
    )
    assert topic_ros_types(TOPICS.odometry) == ("nav_msgs/msg/Odometry",)
    assert topic_ros_types(TOPICS.lidar_scan) == (
        "livox_ros_driver2/msg/CustomMsg",
        "sensor_msgs/msg/PointCloud2",
    )
    assert topic_ros_types(TOPICS.height_rays) == ("application/json",)
    assert runtime_contract_manifest()["topic_ros_types"][TOPICS.map_cloud] == (
        "sensor_msgs/msg/PointCloud2",
    )

    height_rays = registry.topic_contract(TOPICS.height_rays)
    assert height_rays.formats == ("height_rays",)
    assert height_rays.allowed_frame_ids == ("body",)
    assert height_rays.default_frame_id == "body"

    mujoco = registry.data_source_contract("mujoco_module_graph")
    assert TOPICS.height_rays in mujoco.normalized_outputs
    assert mujoco.algorithm_context_outputs == (TOPICS.height_rays,)

    legacy = registry.data_source_contract(LEGACY_REAL_RUNTIME_CONTRACT)
    current = registry.data_source_contract(REAL_RUNTIME_CONTRACT)
    assert legacy is current
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
    map_alias = next(
        alias for alias in terrain_aliases if alias.target == TOPICS.map_clearing
    )
    assert map_alias.source == "/map_clearing"
    assert map_alias.msg_format == "std_msgs/msg/Bool"

    terrain_ext_aliases = adapter_aliases("terrain_analysis_ext")
    cloud_alias = next(
        alias for alias in terrain_ext_aliases if alias.target == TOPICS.cloud_clearing
    )
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
