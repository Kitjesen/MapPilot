from __future__ import annotations

from runtime.endpoints.dds.contracts import (
    DDS_PAYLOAD_FORMAT,
    THUNDER_FIELD_DDS_CONTRACT,
    THUNDER_FIELD_DDS_CONTRACT_NAME,
    binding_for_topic,
)
from runtime.profiles.catalog.endpoints import RUNTIME_ENDPOINTS
from runtime.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    runtime_topic_allowed_frame_ids,
)


def test_thunder_field_endpoint_references_dds_contract() -> None:
    endpoint = RUNTIME_ENDPOINTS["thunder_field"]

    assert endpoint.endpoint_transport == "dds"
    assert endpoint.endpoint_contract == THUNDER_FIELD_DDS_CONTRACT_NAME
    assert THUNDER_FIELD_DDS_CONTRACT.runtime_contract == THUNDER_FIELD_RUNTIME_CONTRACT
    assert THUNDER_FIELD_DDS_CONTRACT.transport == "dds"


def test_thunder_field_dds_contract_covers_runtime_boundary_topics() -> None:
    expected_required = {
        TOPICS.lidar_scan,
        TOPICS.imu,
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_cloud,
        TOPICS.nav_command_request,
        TOPICS.nav_command_ack,
        TOPICS.inspection_evidence_request,
        TOPICS.inspection_evidence_result,
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
    }

    assert expected_required <= set(THUNDER_FIELD_DDS_CONTRACT.required_topics)
    assert TOPICS.goal_pose in THUNDER_FIELD_DDS_CONTRACT.topics
    assert TOPICS.cancel in THUNDER_FIELD_DDS_CONTRACT.topics
    assert TOPICS.semantic_instruction not in THUNDER_FIELD_DDS_CONTRACT.topics
    assert TOPICS.goal_pose not in THUNDER_FIELD_DDS_CONTRACT.required_topics
    assert TOPICS.cancel not in THUNDER_FIELD_DDS_CONTRACT.required_topics
    assert TOPICS.teleop_cmd_vel in THUNDER_FIELD_DDS_CONTRACT.topics
    assert TOPICS.teleop_cmd_vel not in THUNDER_FIELD_DDS_CONTRACT.required_topics
    assert TOPICS.nav_way_point in THUNDER_FIELD_DDS_CONTRACT.topics
    assert TOPICS.nav_way_point not in THUNDER_FIELD_DDS_CONTRACT.required_topics
    assert TOPICS.localization_health in THUNDER_FIELD_DDS_CONTRACT.topics
    assert TOPICS.localization_quality in THUNDER_FIELD_DDS_CONTRACT.topics


def test_dds_contract_uses_typed_idl_schemas() -> None:
    for binding in THUNDER_FIELD_DDS_CONTRACT.bindings:
        assert binding.payload_format == DDS_PAYLOAD_FORMAT
        assert binding.type_name
        assert binding.idl_type
        assert binding.channel


def test_dds_contract_preserves_runtime_frame_expectations() -> None:
    allowed = runtime_topic_allowed_frame_ids(THUNDER_FIELD_RUNTIME_CONTRACT)

    for binding in THUNDER_FIELD_DDS_CONTRACT.bindings:
        assert tuple(allowed.get(binding.topic, ())) == binding.frame_ids

    cmd = binding_for_topic(THUNDER_FIELD_DDS_CONTRACT.name, TOPICS.cmd_vel)
    assert cmd.direction == "lingtu_to_endpoint"
    assert cmd.frame_ids == ("body",)
    assert cmd.idl_type == "lingtu.dds.FinalVelocityCommand"

    waypoint = binding_for_topic(THUNDER_FIELD_DDS_CONTRACT.name, TOPICS.nav_way_point)
    assert waypoint.direction == "lingtu_to_endpoint"
    assert waypoint.frame_ids == ("map", "odom")
    assert waypoint.required is False

    request = binding_for_topic(
        THUNDER_FIELD_DDS_CONTRACT.name,
        TOPICS.nav_command_request,
    )
    assert request.direction == "lingtu_to_endpoint"
    assert request.frame_ids == ("map", "body")

    ack = binding_for_topic(
        THUNDER_FIELD_DDS_CONTRACT.name,
        TOPICS.nav_command_ack,
    )
    assert ack.direction == "endpoint_to_lingtu"
    assert ack.frame_ids == ("map",)

    evidence_request = binding_for_topic(
        THUNDER_FIELD_DDS_CONTRACT.name,
        TOPICS.inspection_evidence_request,
    )
    assert evidence_request.direction == "endpoint_to_lingtu"
    assert evidence_request.idl_type == "lingtu.dds.InspectionEvidenceRequest"
    assert evidence_request.frame_ids == ("map",)

    evidence_result = binding_for_topic(
        THUNDER_FIELD_DDS_CONTRACT.name,
        TOPICS.inspection_evidence_result,
    )
    assert evidence_result.direction == "lingtu_to_endpoint"
    assert evidence_result.idl_type == "lingtu.dds.InspectionEvidenceResult"
    assert evidence_result.frame_ids == ("map",)

    goal = binding_for_topic(THUNDER_FIELD_DDS_CONTRACT.name, TOPICS.goal_pose)
    assert goal.direction == "lingtu_to_endpoint"
    assert goal.frame_ids == ("map", "odom")
    assert goal.required is False

    cancel = binding_for_topic(THUNDER_FIELD_DDS_CONTRACT.name, TOPICS.cancel)
    assert cancel.direction == "lingtu_to_endpoint"
    assert cancel.required is False

    odom = binding_for_topic(THUNDER_FIELD_DDS_CONTRACT.name, TOPICS.odometry)
    assert odom.direction == "endpoint_to_lingtu"
    assert odom.frame_ids == ("odom", "map")
