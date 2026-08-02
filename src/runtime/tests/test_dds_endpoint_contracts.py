from __future__ import annotations

from dataclasses import fields
from pathlib import Path

from message.dds import dds_topic_name, topic_spec
from message.dds_types import (
    DDS_ExplorationCommandAck,
    DDS_ExplorationCommandRequest,
    DDS_NavigationGoalStatus,
    DDS_OperatorMotionAck,
    DDS_OperatorMotionControl,
    DDS_OperatorMotionSample,
    DDS_OperatorMotionStatus,
    ExplorationCommandAck,
    ExplorationCommandRequest,
    NavigationGoalStatus,
    OperatorMotionAck,
    OperatorMotionControl,
    OperatorMotionSample,
    OperatorMotionStatus,
)
from message.dds_types_generated import (
    ExplorationCommandAck as GeneratedExplorationCommandAck,
)
from message.dds_types_generated import (
    ExplorationCommandRequest as GeneratedExplorationCommandRequest,
)
from message.dds_types_generated import (
    NavigationGoalStatus as GeneratedNavigationGoalStatus,
)
from message.dds_types_generated import (
    OperatorMotionAck as GeneratedOperatorMotionAck,
)
from message.dds_types_generated import (
    OperatorMotionControl as GeneratedOperatorMotionControl,
)
from message.dds_types_generated import (
    OperatorMotionSample as GeneratedOperatorMotionSample,
)
from message.dds_types_generated import (
    OperatorMotionStatus as GeneratedOperatorMotionStatus,
)
from message.dds_types_generated.types import NavigationCommandRequest
from runtime.endpoints.dds.contracts import (
    DDS_PAYLOAD_FORMAT,
    THUNDER_DDS_CONTRACT,
    THUNDER_DDS_CONTRACT_NAME,
    binding_for_topic,
)
from runtime.graph import load_runtime_graph
from runtime.routes import robot
from runtime.runtime_interface import (
    MESSAGE_FORMATS,
    REAL_RUNTIME_CONTRACT,
    TOPICS,
    runtime_topic_allowed_frame_ids,
)

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_real_env_references_the_field_dds_contract() -> None:
    real = load_runtime_graph().envs["real"]

    assert real["host_config"]["_endpoint_transport"] == "dds"
    assert real["host_config"]["_endpoint_contract"] == THUNDER_DDS_CONTRACT_NAME
    assert THUNDER_DDS_CONTRACT.runtime_contract == REAL_RUNTIME_CONTRACT
    assert THUNDER_DDS_CONTRACT.transport == "dds"


def test_thunder_dds_contract_covers_runtime_boundary_topics() -> None:
    expected_required = {
        TOPICS.lidar_scan,
        TOPICS.imu,
        TOPICS.odometry,
        TOPICS.registered_cloud,
        TOPICS.map_observation,
        TOPICS.map_cloud,
        TOPICS.nav_command_request,
        TOPICS.nav_command_ack,
        TOPICS.operator_motion_control,
        TOPICS.operator_motion_sample,
        TOPICS.operator_motion_ack,
        TOPICS.operator_motion_status,
        TOPICS.nav_goal_status,
        TOPICS.nav_state,
        TOPICS.exploration_grid,
        TOPICS.exploration_snapshot,
        TOPICS.exploration_execution_snapshot,
        TOPICS.exploration_segment_request,
        TOPICS.exploration_segment_ack,
        TOPICS.exploration_segment_status,
        TOPICS.inspection_evidence_request,
        TOPICS.inspection_evidence_result,
        TOPICS.global_path,
        TOPICS.local_path,
        TOPICS.cmd_vel,
    }

    assert expected_required <= set(THUNDER_DDS_CONTRACT.required_topics)
    assert TOPICS.goal_pose in THUNDER_DDS_CONTRACT.topics
    assert TOPICS.cancel in THUNDER_DDS_CONTRACT.topics
    assert TOPICS.semantic_instruction not in THUNDER_DDS_CONTRACT.topics
    assert TOPICS.goal_pose not in THUNDER_DDS_CONTRACT.required_topics
    assert TOPICS.cancel not in THUNDER_DDS_CONTRACT.required_topics
    assert TOPICS.teleop_cmd_vel in THUNDER_DDS_CONTRACT.topics
    assert TOPICS.teleop_cmd_vel not in THUNDER_DDS_CONTRACT.required_topics
    assert TOPICS.nav_way_point in THUNDER_DDS_CONTRACT.topics
    assert TOPICS.nav_way_point not in THUNDER_DDS_CONTRACT.required_topics
    assert TOPICS.localization_health in THUNDER_DDS_CONTRACT.topics
    assert TOPICS.localization_quality in THUNDER_DDS_CONTRACT.topics

    observation = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.map_observation,
    )
    assert observation.direction == "endpoint_to_lingtu"
    assert observation.idl_type == "lingtu.dds.MapObservation"
    assert observation.frame_ids == ("map",)

    navigation_state = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.nav_state,
    )
    assert navigation_state.direction == "endpoint_to_lingtu"
    assert navigation_state.idl_type == "lingtu.dds.NavigationState"
    assert navigation_state.frame_ids == ("map",)


def test_runtime_graph_native_and_field_product_typed_topics_are_routed_by_dds() -> None:
    graph = load_runtime_graph()
    contract_topics = set(THUNDER_DDS_CONTRACT.topics)
    robot_topics = set(robot().routes)
    native_topics = set(graph.native_contract_topics)
    real = graph.envs["real"]
    supported_products = set(real["supported_products"])
    field_products = {
        product_name: product
        for product_name, product in graph.products.items()
        if product_name in supported_products
    }

    assert real["process_control"] == "systemd"
    assert field_products
    assert native_topics <= contract_topics
    assert native_topics <= robot_topics
    for product_name, product in sorted(field_products.items()):
        required_topics = set(product.get("required_topics", ()))
        assert required_topics <= contract_topics, product_name
        assert required_topics <= robot_topics, product_name


def test_map_state_and_scene_are_latched_endpoint_to_lingtu_dds_routes() -> None:
    route = robot()
    expected_types = {
        TOPICS.maps_state: ("MapRuntimeState", "lingtu.dds.MapRuntimeState"),
        TOPICS.maps_scene: ("MapScene", "lingtu.dds.MapScene"),
    }

    for topic, (type_name, idl_type) in expected_types.items():
        spec = topic_spec(topic)
        binding = binding_for_topic(THUNDER_DDS_CONTRACT.name, topic)

        assert spec is not None
        assert spec.type_name == type_name
        assert binding.direction == "endpoint_to_lingtu"
        assert binding.idl_type == idl_type
        assert binding.frame_ids == ("map",)
        assert binding.required is True
        assert route.backend_for(topic) == "dds"
        assert route.binding_for("dds", topic) == {
            "qos": "state",
            "transient_local": True,
        }


def test_dds_contract_uses_typed_idl_schemas() -> None:
    for binding in THUNDER_DDS_CONTRACT.bindings:
        assert binding.payload_format == DDS_PAYLOAD_FORMAT
        assert binding.type_name
        assert binding.idl_type
        assert binding.channel


def test_dds_contract_preserves_runtime_frame_expectations() -> None:
    allowed = runtime_topic_allowed_frame_ids(REAL_RUNTIME_CONTRACT)

    for binding in THUNDER_DDS_CONTRACT.bindings:
        assert tuple(allowed.get(binding.topic, ())) == binding.frame_ids

    cmd = binding_for_topic(THUNDER_DDS_CONTRACT.name, TOPICS.cmd_vel)
    assert cmd.direction == "lingtu_to_endpoint"
    assert cmd.frame_ids == ("body",)
    assert cmd.idl_type == "lingtu.dds.FinalVelocityCommand"

    waypoint = binding_for_topic(THUNDER_DDS_CONTRACT.name, TOPICS.nav_way_point)
    assert waypoint.direction == "lingtu_to_endpoint"
    assert waypoint.frame_ids == ("map", "odom")
    assert waypoint.required is False

    request = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.nav_command_request,
    )
    assert request.direction == "lingtu_to_endpoint"
    assert request.frame_ids == ("map", "body")

    ack = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.nav_command_ack,
    )
    assert ack.direction == "endpoint_to_lingtu"
    assert ack.frame_ids == ("map",)

    operator_control = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.operator_motion_control,
    )
    assert operator_control.direction == "lingtu_to_endpoint"
    assert operator_control.frame_ids == ()
    assert operator_control.idl_type == "lingtu.dds.OperatorMotionControl"

    operator_sample = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.operator_motion_sample,
    )
    assert operator_sample.direction == "lingtu_to_endpoint"
    assert operator_sample.frame_ids == ("body",)
    assert operator_sample.idl_type == "lingtu.dds.OperatorMotionSample"

    operator_ack = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.operator_motion_ack,
    )
    assert operator_ack.direction == "endpoint_to_lingtu"
    assert operator_ack.frame_ids == ()
    assert operator_ack.idl_type == "lingtu.dds.OperatorMotionAck"

    operator_status = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.operator_motion_status,
    )
    assert operator_status.direction == "endpoint_to_lingtu"
    assert operator_status.frame_ids == ("map",)
    assert operator_status.idl_type == "lingtu.dds.OperatorMotionStatus"

    goal_status = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.nav_goal_status,
    )
    assert goal_status.direction == "endpoint_to_lingtu"
    assert goal_status.idl_type == "lingtu.dds.NavigationGoalStatus"
    assert goal_status.frame_ids == ("map",)

    exploration_bindings = {
        TOPICS.exploration_grid: (
            "endpoint_to_lingtu",
            "lingtu.dds.OccupancyGrid",
            ("map", "odom"),
        ),
        TOPICS.exploration_snapshot: (
            "endpoint_to_lingtu",
            "lingtu.dds.ExplorationGrid",
            ("map",),
        ),
        TOPICS.exploration_execution_snapshot: (
            "endpoint_to_lingtu",
            "lingtu.dds.ExplorationExecutionGrid",
            ("map",),
        ),
        TOPICS.exploration_segment_request: (
            "endpoint_to_lingtu",
            "lingtu.dds.ExplorationSegmentRequest",
            ("map",),
        ),
        TOPICS.exploration_segment_ack: (
            "endpoint_to_lingtu",
            "lingtu.dds.ExplorationSegmentAck",
            ("map",),
        ),
        TOPICS.exploration_segment_status: (
            "endpoint_to_lingtu",
            "lingtu.dds.ExplorationSegmentStatus",
            ("map",),
        ),
    }
    for topic, (direction, idl_type, frame_ids) in exploration_bindings.items():
        binding = binding_for_topic(THUNDER_DDS_CONTRACT.name, topic)
        assert binding.direction == direction
        assert binding.idl_type == idl_type
        assert binding.frame_ids == frame_ids

    evidence_request = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.inspection_evidence_request,
    )
    assert evidence_request.direction == "endpoint_to_lingtu"
    assert evidence_request.idl_type == "lingtu.dds.InspectionEvidenceRequest"
    assert evidence_request.frame_ids == ("map",)

    evidence_result = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.inspection_evidence_result,
    )
    assert evidence_result.direction == "lingtu_to_endpoint"
    assert evidence_result.idl_type == "lingtu.dds.InspectionEvidenceResult"
    assert evidence_result.frame_ids == ("map",)

    goal = binding_for_topic(THUNDER_DDS_CONTRACT.name, TOPICS.goal_pose)
    assert goal.direction == "lingtu_to_endpoint"
    assert goal.frame_ids == ("map", "odom")
    assert goal.required is False

    cancel = binding_for_topic(THUNDER_DDS_CONTRACT.name, TOPICS.cancel)
    assert cancel.direction == "lingtu_to_endpoint"
    assert cancel.required is False

    odom = binding_for_topic(THUNDER_DDS_CONTRACT.name, TOPICS.odometry)
    assert odom.direction == "endpoint_to_lingtu"
    assert odom.frame_ids == ("odom", "map")


def test_navigation_command_idempotency_contract_has_client_identity() -> None:
    idl = (REPO_ROOT / "src/message/idl/lingtu_slam.idl").read_text(
        encoding="utf-8"
    )
    struct = idl.split("struct NavigationCommandRequest {", 1)[1].split("};", 1)[0]
    assert "string client_id;" in struct
    assert struct.index("string client_id;") < struct.index("string task_id;")
    assert struct.index("string task_id;") < struct.index("string request_id;")

    generated_fields = [field.name for field in fields(NavigationCommandRequest)]
    assert generated_fields[:4] == ["header", "client_id", "task_id", "request_id"]
    assert "velocity" not in generated_fields
    assert "Twist velocity;" not in struct

    runtime_type = (REPO_ROOT / "src/message/dds_types/nav.py").read_text(
        encoding="utf-8"
    )
    assert "    client_id: str\n    task_id: str\n    request_id: str" in runtime_type

    native_client = (REPO_ROOT / "src/nav/cpp/client/client.cpp").read_text(
        encoding="utf-8"
    )
    native_driver = (
        REPO_ROOT / "src/drivers/real/thunder/native/dds.cpp"
    ).read_text(encoding="utf-8")
    assert "message.client_id =" in native_client
    assert "sync.client_id =" in native_client
    assert "msg.client_id =" in native_driver


def test_driver_control_state_correlates_brainstem_ack_to_nav_output() -> None:
    idl = (REPO_ROOT / "src/message/idl/lingtu_slam.idl").read_text(
        encoding="utf-8"
    )
    struct = idl.split("struct DriverControlState {", 1)[1].split("};", 1)[0]
    assert "string accepted_producer_boot_id;" in struct
    assert "unsigned long long accepted_output_sequence;" in struct

    generated_fields = [
        field.name
        for field in fields(
            __import__(
                "message.dds_types_generated.types",
                fromlist=["DriverControlState"],
            ).DriverControlState
        )
    ]
    assert "accepted_producer_boot_id" in generated_fields
    assert "accepted_output_sequence" in generated_fields

    driver_main = (
        REPO_ROOT / "src/drivers/real/thunder/native/main.cpp"
    ).read_text(encoding="utf-8")
    compact_driver_main = " ".join(driver_main.split())
    assert "stats.accepted_producer_boot_id = read.latest->producer_boot_id" in compact_driver_main
    assert "stats.accepted_output_sequence = read.latest->output_seq" in compact_driver_main

    input_projector = (
        REPO_ROOT / "src/nav/cpp/endpoint/input/nav_input_state_projector.cpp"
    ).read_text(encoding="utf-8")
    assert "message.accepted_producer_boot_id" in input_projector
    assert "message.accepted_output_sequence" in input_projector


def test_navigation_goal_status_is_exposed_across_python_dds_boundary() -> None:
    expected_fields = [
        "header",
        "boot_id",
        "event_sequence",
        "task_id",
        "request_id",
        "state",
        "goal_epoch",
        "reason",
    ]

    assert NavigationGoalStatus is DDS_NavigationGoalStatus
    assert [field.name for field in fields(NavigationGoalStatus)] == expected_fields
    assert [field.name for field in fields(GeneratedNavigationGoalStatus)] == expected_fields

    spec = topic_spec(TOPICS.nav_goal_status)
    assert spec is not None
    assert spec.type_name == "NavigationGoalStatus"
    assert spec.import_path == "message.dds_types.nav.NavigationGoalStatus"
    assert spec.idl_type == "lingtu.dds.NavigationGoalStatus"
    assert spec.cpp_type == "lingtu::dds::NavigationGoalStatus"
    assert dds_topic_name(TOPICS.nav_goal_status, typed=True) == "rt/nav/goal/status"

    format_spec = MESSAGE_FORMATS["nav_goal_status"]
    assert format_spec.ros_type == "lingtu.dds.NavigationGoalStatus"
    assert format_spec.required_fields == tuple(expected_fields)


def test_operator_motion_interface_is_native_typed_and_body_sampled() -> None:
    control_fields = [
        "header",
        "source_id",
        "source_epoch",
        "source_sequence",
        "request_id",
        "action",
        "lease_ttl_ms",
        "reason",
    ]
    sample_fields = [
        "header",
        "source_id",
        "source_epoch",
        "source_sequence",
        "request_id",
        "deadman",
        "velocity",
        "freshness_budget_ms",
        "source_stamp_ns",
    ]
    ack_fields = [
        "header",
        "source_id",
        "source_epoch",
        "source_sequence",
        "request_id",
        "action",
        "accepted",
        "reason",
        "accepted_sequence",
        "final_output_sequence",
    ]
    status_fields = [
        "header",
        "active_source_id",
        "active_source_epoch",
        "has_active_authority",
        "holding",
        "has_active_sample",
        "last_sample_sequence",
        "admitted_sequence",
        "final_output_sequence",
        "authority_reason",
        "input_gate_reason",
        "teleop_output",
        "final_cmd_vel",
    ]

    assert OperatorMotionControl is DDS_OperatorMotionControl
    assert OperatorMotionSample is DDS_OperatorMotionSample
    assert OperatorMotionAck is DDS_OperatorMotionAck
    assert OperatorMotionStatus is DDS_OperatorMotionStatus
    assert [field.name for field in fields(OperatorMotionControl)] == control_fields
    assert [field.name for field in fields(OperatorMotionSample)] == sample_fields
    assert [field.name for field in fields(OperatorMotionAck)] == ack_fields
    assert [field.name for field in fields(OperatorMotionStatus)] == status_fields
    assert [field.name for field in fields(GeneratedOperatorMotionControl)] == control_fields
    assert [field.name for field in fields(GeneratedOperatorMotionSample)] == sample_fields
    assert [field.name for field in fields(GeneratedOperatorMotionAck)] == ack_fields
    assert [field.name for field in fields(GeneratedOperatorMotionStatus)] == status_fields

    expected_topics = {
        TOPICS.operator_motion_control: (
            "OperatorMotionControl",
            "rt/nav/operator_motion/control",
            "operator_motion_control",
            (),
        ),
        TOPICS.operator_motion_sample: (
            "OperatorMotionSample",
            "rt/nav/operator_motion/sample",
            "operator_motion_sample",
            ("body",),
        ),
        TOPICS.operator_motion_ack: (
            "OperatorMotionAck",
            "rt/nav/operator_motion/ack",
            "operator_motion_ack",
            (),
        ),
        TOPICS.operator_motion_status: (
            "OperatorMotionStatus",
            "rt/nav/operator_motion/status",
            "operator_motion_status",
            ("map",),
        ),
    }
    allowed = runtime_topic_allowed_frame_ids(REAL_RUNTIME_CONTRACT)
    for topic, (type_name, wire_topic, format_name, frames) in expected_topics.items():
        spec = topic_spec(topic)
        assert spec is not None
        assert spec.type_name == type_name
        assert spec.import_path == f"message.dds_types.nav.{type_name}"
        assert spec.idl_type == f"lingtu.dds.{type_name}"
        assert spec.cpp_type == f"lingtu::dds::{type_name}"
        assert dds_topic_name(topic, typed=True) == wire_topic
        assert MESSAGE_FORMATS[format_name].ros_type == f"lingtu.dds.{type_name}"
        assert allowed[topic] == frames

    assert MESSAGE_FORMATS["operator_motion_control"].required_fields == tuple(control_fields[1:])
    assert MESSAGE_FORMATS["operator_motion_sample"].required_fields == tuple(sample_fields[1:])
    assert MESSAGE_FORMATS["operator_motion_ack"].required_fields == tuple(ack_fields[1:])
    assert MESSAGE_FORMATS["operator_motion_status"].required_fields == tuple(status_fields[1:])
    assert MESSAGE_FORMATS["operator_motion_control"].frame_role == "metadata"
    assert MESSAGE_FORMATS["operator_motion_sample"].frame_role == "body"
    assert MESSAGE_FORMATS["operator_motion_ack"].frame_role == "metadata"

    idl = (REPO_ROOT / "src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    assert "struct OperatorMotionControl" in idl
    assert "struct OperatorMotionSample" in idl
    assert "struct OperatorMotionAck" in idl
    assert "struct OperatorMotionStatus" in idl

    header = (REPO_ROOT / "src/message/cpp/operator_motion.hpp").read_text(encoding="utf-8")
    assert "Claim = 1" in header
    assert "Release = 2" in header
    assert "Hold = 3" in header
    assert "Sample = 4" not in header

    client = (REPO_ROOT / "src/nav/cpp/client/client.cpp").read_text(encoding="utf-8")
    assert "OperatorMotionAction::Sample" not in client
    assert "operator-motion-sample-" in client
    sample_segment = client.split("void writeOperatorMotionSample(", 1)[1].split(
        "static bool requiresEndpointClock", 1
    )[0]
    assert "registerOperatorMotionAck" not in sample_segment


def test_directed_exploration_command_is_exposed_across_python_dds_boundary() -> None:
    request_fields = [
        "header",
        "request_id",
        "exploration_run_id",
        "kind",
        "session_id",
        "has_directed_target",
        "directed_target_x",
        "directed_target_y",
        "directed_target_ttl_s",
        "reason",
    ]
    ack_fields = [
        "header",
        "request_id",
        "exploration_run_id",
        "kind",
        "accepted",
        "duplicate",
        "reason",
        "session_id",
        "intent_revision",
    ]

    assert ExplorationCommandRequest is DDS_ExplorationCommandRequest
    assert ExplorationCommandAck is DDS_ExplorationCommandAck
    assert [field.name for field in fields(ExplorationCommandRequest)] == request_fields
    assert [field.name for field in fields(ExplorationCommandAck)] == ack_fields
    assert [field.name for field in fields(GeneratedExplorationCommandRequest)] == request_fields
    assert [field.name for field in fields(GeneratedExplorationCommandAck)] == ack_fields

    idl = (REPO_ROOT / "src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    request_struct = idl.split("struct ExplorationCommandRequest {", 1)[1].split("};", 1)[0]
    ack_struct = idl.split("struct ExplorationCommandAck {", 1)[1].split("};", 1)[0]
    request_declarations = [
        "string exploration_run_id;",
        "string session_id;",
        "boolean has_directed_target;",
        "double directed_target_x;",
        "double directed_target_y;",
        "double directed_target_ttl_s;",
        "string reason;",
    ]
    ack_declarations = [
        "string exploration_run_id;",
        "boolean duplicate;",
        "string session_id;",
        "unsigned long long intent_revision;",
    ]
    assert all(declaration in request_struct for declaration in request_declarations)
    assert all(declaration in ack_struct for declaration in ack_declarations)
    assert [request_struct.index(item) for item in request_declarations] == sorted(
        request_struct.index(item) for item in request_declarations
    )
    assert [ack_struct.index(item) for item in ack_declarations] == sorted(
        ack_struct.index(item) for item in ack_declarations
    )

    request_spec = topic_spec(TOPICS.exploration_command)
    ack_spec = topic_spec(TOPICS.exploration_ack)
    assert request_spec is not None
    assert ack_spec is not None
    assert request_spec.dds_type() is ExplorationCommandRequest
    assert ack_spec.dds_type() is ExplorationCommandAck
    assert dds_topic_name(TOPICS.exploration_command, typed=True) == "rt/nav/exploration/command"
    assert dds_topic_name(TOPICS.exploration_ack, typed=True) == "rt/nav/exploration/ack"

    request_binding = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.exploration_command,
    )
    ack_binding = binding_for_topic(
        THUNDER_DDS_CONTRACT.name,
        TOPICS.exploration_ack,
    )
    assert "directed map-frame target" in request_binding.note
    assert "intent_revision" in ack_binding.note

    assert MESSAGE_FORMATS["exploration_command"].required_fields == tuple(request_fields[1:])
    assert MESSAGE_FORMATS["exploration_ack"].required_fields == tuple(ack_fields[1:])
