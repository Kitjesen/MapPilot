from dataclasses import fields
from pathlib import Path

from runtime.adapters.native import inspection as native_inspection
from runtime.profiles.resolver import resolve_profile_config
from runtime.runtime_interface import TOPIC_ALLOWED_FRAME_IDS, TOPIC_ROS_TYPES, TOPICS


def test_inspection_profile_does_not_enable_python_patrol_runtime() -> None:
    config = resolve_profile_config("inspection")

    assert config["enable_patrol_routes"] is False
    assert config["enable_scheduler"] is False


def test_typed_inspection_topics_are_canonical() -> None:
    assert TOPICS.inspection_command == "/nav/inspection/command"
    assert TOPICS.inspection_ack == "/nav/inspection/ack"
    assert TOPICS.inspection_status == "/nav/inspection/status"
    assert TOPICS.inspection_evidence_request == "/nav/inspection/evidence/request"
    assert TOPICS.inspection_evidence_result == "/nav/inspection/evidence/result"
    assert TOPIC_ROS_TYPES[TOPICS.inspection_command] == ("lingtu.dds.InspectionCommandRequest",)
    assert TOPIC_ROS_TYPES[TOPICS.inspection_ack] == ("lingtu.dds.InspectionCommandAck",)
    assert TOPIC_ROS_TYPES[TOPICS.inspection_status] == ("lingtu.dds.InspectionStatus",)
    assert TOPIC_ROS_TYPES[TOPICS.inspection_evidence_request] == ("lingtu.dds.InspectionEvidenceRequest",)
    assert TOPIC_ROS_TYPES[TOPICS.inspection_evidence_result] == ("lingtu.dds.InspectionEvidenceResult",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_command] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_ack] == ()
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_status] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_evidence_request] == ("map",)
    assert TOPIC_ALLOWED_FRAME_IDS[TOPICS.inspection_evidence_result] == ("map",)


def test_inspection_idl_and_native_endpoint_are_wired() -> None:
    idl = Path("src/message/idl/lingtu_slam.idl").read_text(encoding="utf-8")
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")
    cmake = Path("src/nav/services/endpoint/cpp/CMakeLists.txt").read_text(encoding="utf-8")

    assert "struct InspectionCommandRequest" in idl
    assert "struct InspectionCommandAck" in idl
    assert "struct InspectionStatus" in idl
    assert "drainInspectionCommands" in endpoint
    assert "inspection_executor.PendingGoal" in endpoint
    assert "inspection_executor.OnGoalReached" in endpoint
    assert "lingtu_inspection_core" in cmake


def test_inspection_action_deadline_is_started_before_evidence_publish() -> None:
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")
    action_block = endpoint.split(
        "if (const auto action = inspection_executor.PendingAction()) {",
        1,
    )[1].split("const std::string driver_blocker", 1)[0]

    assert action_block.index("inspection_executor.OnActionStarted") < action_block.index(
        "dds.writeInspectionEvidenceRequest"
    )
    assert "evidence_request_publish_failed" in action_block
    assert "inspection_executor.OnActionResult" in action_block


def test_inspection_evidence_dds_contract_is_complete() -> None:
    from message.dds import dds_topic_name, dds_type_for_topic, topic_spec
    from message.dds_types.nav import (
        InspectionEvidenceRequest,
        InspectionEvidenceResult,
        InspectionStatus,
    )

    request_topic = TOPICS.inspection_evidence_request
    result_topic = TOPICS.inspection_evidence_result
    expected = {
        request_topic: (
            InspectionEvidenceRequest,
            "rt/nav/inspection/evidence/request",
            "InspectionEvidenceRequest",
        ),
        result_topic: (
            InspectionEvidenceResult,
            "rt/nav/inspection/evidence/result",
            "InspectionEvidenceResult",
        ),
    }

    for topic, (dds_type, wire_topic, type_name) in expected.items():
        spec = topic_spec(topic)
        assert spec is not None
        assert spec.type_name == type_name
        assert spec.dds_topic == wire_topic
        assert spec.idl_type == f"lingtu.dds.{type_name}"
        assert spec.cpp_type == f"lingtu::dds::{type_name}"
        assert dds_topic_name(topic, typed=True) == wire_topic
        assert dds_type_for_topic(topic) is dds_type

    assert [field.name for field in fields(InspectionEvidenceRequest)] == [
        "header",
        "request_id",
        "run_id",
        "route_id",
        "revision",
        "map_id",
        "map_version",
        "point_index",
        "point_id",
        "action",
        "deadline_s",
    ]
    assert [field.name for field in fields(InspectionEvidenceResult)] == [
        "header",
        "request_id",
        "evidence_id",
        "persisted",
        "reason",
        "analysis_verdict",
    ]
    assert [field.name for field in fields(InspectionStatus)][-7:] == [
        "action",
        "action_request_id",
        "evidence_id",
        "phase_started_at",
        "stable_since",
        "deadline",
        "reason",
    ]


def test_native_inspection_library_candidates_cover_repo_and_deployed_builds(
    monkeypatch,
) -> None:
    monkeypatch.delenv("LINGTU_INSPECTION_LIBRARY", raising=False)

    candidates = native_inspection._library_candidates()
    repo_root = Path(native_inspection.__file__).resolve().parents[4]
    library_name = native_inspection._library_names()[0]

    assert repo_root / "src" / "nav" / "inspection" / "build" / library_name in candidates
    assert repo_root / "build" / "nav_endpoint" / "inspection" / library_name in candidates
    assert Path("/opt/lingtu/current/build/nav_endpoint/inspection") / library_name in candidates


def test_inspection_resume_requires_autonomy_control_before_executor_resume() -> None:
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")
    resume_block = endpoint.split("} else if (kind == InspectionCommand::kResume) {", 1)[1].split(
        "remember_inspection_ack", 1
    )[0]

    assert "control_authority.operatorTakeoverLatched()" in resume_block
    assert "inspection_resume_requires_autonomy" in resume_block
    assert resume_block.index("operatorTakeoverLatched") < resume_block.index("inspection_executor.Resume")


def test_active_map_change_clears_inspection_motion_immediately() -> None:
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")

    assert "inspection_state_before_map_check" in endpoint
    assert 'inspection_executor.status().reason == "active_map_changed"' in endpoint
    assert 'clear_endpoint_motion("inspection_active_map_changed")' in endpoint


def test_local_recovery_exhaustion_routes_into_inspection_failure_policy() -> None:
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")

    assert "out.recovery_exhausted" in endpoint
    assert "inspection_local_recovery_exhausted" in endpoint
    clear_index = endpoint.index('clear_endpoint_motion("inspection_local_recovery_exhausted")')
    failure_index = endpoint.index("inspection_executor.OnNavigationFailed")
    assert clear_index < failure_index


def test_endpoint_feeds_final_inspection_point_progress_to_watchdog() -> None:
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")

    assert "inspection_executor.OnNavigationProgress" in endpoint
    assert endpoint.count("inspection_executor.OnNavigationProgress") == 1
    assert "active_inspection_point->x_m" in endpoint
    assert "active_inspection_point->y_m" in endpoint


def test_post_arrival_inspection_actions_fail_closed_on_localization_gate() -> None:
    endpoint = Path("src/nav/services/endpoint/cpp/nav_native_endpoint.cpp").read_text(encoding="utf-8")

    gate_index = endpoint.index("input_gate_state = input_gate.evaluate(input_snapshot);")
    action_index = endpoint.index("if (const auto action = inspection_executor.PendingAction()) {")
    assert gate_index < action_index
    assert "inspectionPostArrivalState" in endpoint
    assert "localizationGateBlocked" in endpoint
    assert "localizationGateBlocked(input_gate_state, gate_cfg)" in endpoint
    assert "state.localization_health_age_s" in endpoint
    assert "inspection_post_arrival_localization_pause" in endpoint
    pause_index = endpoint.index("inspection_post_arrival_localization_pause")
    tick_index = endpoint.index("inspection_executor.Tick")
    assert pause_index < tick_index
