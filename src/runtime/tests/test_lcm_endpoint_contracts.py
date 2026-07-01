from __future__ import annotations

from runtime.adapters.lcm.contracts import (
    LCM_PAYLOAD_FORMAT,
    THUNDER_FIELD_LCM_CONTRACT,
    binding_for_topic,
    lcm_channel_for_topic,
)
from runtime.profiles.catalog.endpoints import RUNTIME_ENDPOINTS
from runtime.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    runtime_topic_allowed_frame_ids,
)


def test_thunder_field_lcm_contract_available_for_optional_lcm_endpoint() -> None:
    """LCM contract supports smoke/replay/lcm-endpoint mode; field default is typed DDS."""

    endpoint = RUNTIME_ENDPOINTS["thunder_field"]

    assert endpoint.endpoint_transport == "dds"
    assert endpoint.endpoint_contract != THUNDER_FIELD_LCM_CONTRACT.name
    assert THUNDER_FIELD_LCM_CONTRACT.runtime_contract == THUNDER_FIELD_RUNTIME_CONTRACT
    assert THUNDER_FIELD_LCM_CONTRACT.transport == "lcm"


def test_thunder_field_lcm_contract_covers_runtime_boundary_topics() -> None:
    expected_required = {
        TOPICS.lidar_scan,
        TOPICS.imu,
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_cloud,
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
    }

    assert expected_required <= set(THUNDER_FIELD_LCM_CONTRACT.required_topics)
    assert TOPICS.goal_pose in THUNDER_FIELD_LCM_CONTRACT.topics
    assert TOPICS.cancel in THUNDER_FIELD_LCM_CONTRACT.topics
    assert TOPICS.semantic_instruction in THUNDER_FIELD_LCM_CONTRACT.topics
    assert TOPICS.goal_pose not in THUNDER_FIELD_LCM_CONTRACT.required_topics
    assert TOPICS.cancel not in THUNDER_FIELD_LCM_CONTRACT.required_topics
    assert TOPICS.semantic_instruction not in THUNDER_FIELD_LCM_CONTRACT.required_topics
    assert TOPICS.nav_way_point in THUNDER_FIELD_LCM_CONTRACT.topics
    assert TOPICS.nav_way_point not in THUNDER_FIELD_LCM_CONTRACT.required_topics
    assert TOPICS.localization_health in THUNDER_FIELD_LCM_CONTRACT.topics
    assert TOPICS.localization_quality in THUNDER_FIELD_LCM_CONTRACT.topics


def test_lcm_contract_uses_product_neutral_schemas_not_ros_types() -> None:
    for binding in THUNDER_FIELD_LCM_CONTRACT.bindings:
        assert binding.payload_format == LCM_PAYLOAD_FORMAT
        assert binding.schema.startswith("lingtu.")
        assert "ros" not in binding.schema.lower()
        assert binding.channel == lcm_channel_for_topic(binding.topic)
        assert binding.channel == binding.channel.upper()
        assert "/" not in binding.channel


def test_lcm_contract_preserves_runtime_frame_expectations() -> None:
    allowed = runtime_topic_allowed_frame_ids(THUNDER_FIELD_RUNTIME_CONTRACT)

    for binding in THUNDER_FIELD_LCM_CONTRACT.bindings:
        assert tuple(allowed.get(binding.topic, ())) == binding.frame_ids

    cmd = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT.name, TOPICS.cmd_vel)
    assert cmd.direction == "lingtu_to_endpoint"
    assert cmd.frame_ids == ("body",)

    waypoint = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT.name, TOPICS.nav_way_point)
    assert waypoint.direction == "lingtu_to_endpoint"
    assert waypoint.frame_ids == ("map", "odom")
    assert waypoint.schema == "lingtu.geometry.pose_stamped.v1"
    assert waypoint.required is False

    goal = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT.name, TOPICS.goal_pose)
    assert goal.direction == "endpoint_to_lingtu"
    assert goal.frame_ids == ("map", "odom")
    assert goal.schema == "lingtu.geometry.pose_stamped.v1"
    assert goal.required is False

    cancel = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT.name, TOPICS.cancel)
    assert cancel.direction == "endpoint_to_lingtu"
    assert cancel.schema == "lingtu.control.cancel.v1"
    assert cancel.required is False

    instruction = binding_for_topic(
        THUNDER_FIELD_LCM_CONTRACT.name,
        TOPICS.semantic_instruction,
    )
    assert instruction.direction == "endpoint_to_lingtu"
    assert instruction.schema == "lingtu.control.instruction.v1"
    assert instruction.required is False

    odom = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT.name, TOPICS.odometry)
    assert odom.direction == "endpoint_to_lingtu"
    assert odom.frame_ids == ("odom", "map")
