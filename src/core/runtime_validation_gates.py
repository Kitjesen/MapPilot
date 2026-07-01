"""Canonical validation gate descriptors for runtime evidence workflows."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Mapping

from core.runtime_evidence import (
    REAL_RUNTIME_CONTRACT,
    REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED,
)


RUNTIME_AUDIT_COMMAND = (
    "python lingtu.py runtime-audit "
    "--json-out artifacts/runtime_contract_audit.json"
)
REAL_RUNTIME_EVIDENCE_ARTIFACT = "artifacts/thunder_field_runtime/report.json"
REAL_RUNTIME_EVIDENCE_GATE_ARTIFACT = (
    "artifacts/thunder_field_runtime/runtime_evidence.json"
)
REAL_RUNTIME_EVIDENCE_COMMAND = (
    "python lingtu.py real-runtime-evidence "
    "--duration-sec 20 "
    f"--json-out {REAL_RUNTIME_EVIDENCE_ARTIFACT}"
)
REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND = (
    "python scripts/gates/real_runtime_evidence_collect.py "
    "--duration-sec 20 "
    f"--expected-contract {REAL_RUNTIME_CONTRACT} "
    f"--json-out {REAL_RUNTIME_EVIDENCE_ARTIFACT}"
)
REAL_RUNTIME_EVIDENCE_GATE_COMMAND = (
    "python scripts/gates/real_runtime_evidence_gate.py "
    f"{REAL_RUNTIME_EVIDENCE_ARTIFACT} "
    f"--expected-contract {REAL_RUNTIME_CONTRACT} "
    f"--json-out {REAL_RUNTIME_EVIDENCE_GATE_ARTIFACT}"
)
SAVED_MAP_ARTIFACT_GATE_COMMAND = (
    "python lingtu.py saved-map-artifact-gate "
    "<map-dir> "
    "--require-tomogram "
    "--require-occupancy "
    "--json-out artifacts/saved_map_artifacts/report.json"
)

RUNTIME_AUDIT_VALIDATES = (
    "runtime_manifest_vs_topic_contract_yaml",
    "profile_runtime_switch_specs",
    "thunder_field_collector_topic_coverage",
    "canonical_frame_links",
    "topic_default_frame_integrity",
    "ros_frame_contract_doc_mirror",
    "runtime_validation_gates_self_description",
    "runtime_algorithm_interfaces_cover_data_flow",
    "runtime_stage_algorithm_interface_binding",
    "data_source_resolved_flow_inputs_outputs",
    "runtime_stage_owner_frame_map_dependency_integrity",
    "source_code_uses_canonical_frame_contract",
    "source_code_uses_canonical_runtime_topics",
    "collector_is_read_only",
)
RUNTIME_AUDIT_CHECKS = (
    "yaml_manifest",
    "profile_runtime_specs",
    "runtime_contract_integrity",
    "real_runtime_collector",
    "runtime_validation_gates",
    "ros_frame_contract_doc",
    "source_frame_contracts",
    "source_topic_contracts",
)
RUNTIME_AUDIT_VALIDATE_CHECK_COVERAGE = {
    "runtime_manifest_vs_topic_contract_yaml": ("yaml_manifest",),
    "profile_runtime_switch_specs": ("profile_runtime_specs",),
    "thunder_field_collector_topic_coverage": ("real_runtime_collector",),
    "canonical_frame_links": ("runtime_contract_integrity",),
    "topic_default_frame_integrity": ("runtime_contract_integrity",),
    "ros_frame_contract_doc_mirror": ("ros_frame_contract_doc",),
    "runtime_validation_gates_self_description": ("runtime_validation_gates",),
    "runtime_algorithm_interfaces_cover_data_flow": ("runtime_contract_integrity",),
    "runtime_stage_algorithm_interface_binding": ("runtime_contract_integrity",),
    "data_source_resolved_flow_inputs_outputs": ("runtime_contract_integrity",),
    "runtime_stage_owner_frame_map_dependency_integrity": (
        "runtime_contract_integrity",
    ),
    "source_code_uses_canonical_frame_contract": ("source_frame_contracts",),
    "source_code_uses_canonical_runtime_topics": ("source_topic_contracts",),
    "collector_is_read_only": ("real_runtime_collector", "runtime_validation_gates"),
}
REAL_RUNTIME_EVIDENCE_VALIDATES = (
    "simulation_only_false",
    "collector_read_only_and_no_control_topics_payload",
    "real_robot_motion_true",
    "real_motion_delta_and_sample_payload",
    "localization_health_locked_or_recovered",
    "localization_quality_sampled",
    "localization_quality_healthy_range",
    "raw_lidar_imu_endpoint_input_samples",
    "live_topic_freshness_payload",
    "cmd_vel_reaches_hardware_boundary",
    "hardware_boundary_evidence_payload",
    "nonempty_global_and_local_path",
    "local_planner_input_output_chain_observed",
    "complete_runtime_topic_evidence_coverage",
    "canonical_frames_payload",
    "topic_default_frame_contract_payload",
    "topic_allowed_frame_contract_payload",
    "required_topic_frame_contract_declared",
    "required_topic_default_allowed_and_observed_frame_payload",
    "required_topic_sample_window_payload",
    "data_flow_temporal_order_payload",
    "map_cloud_and_global_path_in_map_frame",
    "canonical_frame_links_observed",
    "frame_link_evidence_payload",
    "resolved_runtime_data_flow_observed",
    "data_flow_observed_missing_token_payload",
    "algorithm_interface_payload",
    "runtime_stage_algorithm_interface_payload",
    "runtime_data_flow_stage_evidence_payload",
)
SAVED_MAP_ARTIFACT_GATE_VALIDATES = (
    "metadata_json_exists",
    "map_pcd_sha256_matches_file",
    "tomogram_source_map_sha256_matches_map_pcd",
    "occupancy_source_map_sha256_matches_map_pcd",
    "artifact_frame_and_source_metadata_consistent",
    "artifact_gate_payload_declares_checked_artifacts_frames_sources",
)
RUNTIME_AUDIT_PROVES = (
    "canonical_runtime_manifest_matches_yaml",
    "profile_endpoint_runtime_specs_resolve",
    "source_code_uses_canonical_topics_and_frames",
    "real_runtime_collector_is_read_only",
)
RUNTIME_AUDIT_OPERATOR_SUMMARY_SECTIONS = (
    "Blockers",
    "Checks",
    "Validation gate sequence",
    "Validation commands",
)
REAL_RUNTIME_EVIDENCE_PROVES = (
    "observed_thunder_field_runtime_contract",
    "observed_required_topic_frame_ids",
    "observed_map_odom_body_lidar_tf_links",
    "observed_resolved_runtime_data_flow",
    "observed_hardware_command_boundary_after_cmd_vel_mux",
)
SAVED_MAP_ARTIFACT_GATE_PROVES = (
    "saved_map_metadata_exists",
    "map_pcd_checksum_matches_metadata",
    "tomogram_and_occupancy_derive_from_same_map_pcd",
    "artifact_frame_and_source_metadata_are_consistent",
)
REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS = (
    "Blockers",
    "Checked runtime topics",
    "Checked frame links",
    "Topic frame evidence",
    "Frame link evidence",
    "Stage evidence matrix",
    "Data-flow evidence",
)
SAVED_MAP_ARTIFACT_GATE_OPERATOR_SUMMARY_SECTIONS = (
    "Expected",
    "Required artifacts",
    "Metadata",
    "Artifacts",
    "Blockers",
)

_RUNTIME_VALIDATION_GATES: dict[str, dict[str, Any]] = {
    "runtime_audit": {
        "schema_version": "lingtu.runtime_contract_audit.v1",
        "scope": "offline_runtime_contract",
        "acceptance_step": 1,
        "required_when": "before_any_runtime_contract_or_field_readiness_claim",
        "requires_prior_gates": [],
        "conditional_prior_gates": [],
        "proves": list(RUNTIME_AUDIT_PROVES),
        "operator_summary_sections": list(RUNTIME_AUDIT_OPERATOR_SUMMARY_SECTIONS),
        "command": RUNTIME_AUDIT_COMMAND,
        "artifact": "artifacts/runtime_contract_audit.json",
        "requires_ros": False,
        "requires_real_robot_runtime": False,
        "requires_active_robot_run": False,
        "collector_publishes_control_topics": False,
        "control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        "validates": list(RUNTIME_AUDIT_VALIDATES),
        "checks": list(RUNTIME_AUDIT_CHECKS),
        "coverage": {
            name: list(checks)
            for name, checks in RUNTIME_AUDIT_VALIDATE_CHECK_COVERAGE.items()
        },
    },
    "real_runtime_evidence": {
        "schema_version": "lingtu.real_runtime_evidence.v1",
        "scope": "observed_thunder_field_runtime",
        "acceptance_step": 3,
        "required_when": "before_claiming_thunder_field_runtime_or_field_navigation",
        "requires_prior_gates": ["runtime_audit"],
        "conditional_prior_gates": [
            "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
        ],
        "proves": list(REAL_RUNTIME_EVIDENCE_PROVES),
        "operator_summary_sections": list(
            REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS
        ),
        "command": REAL_RUNTIME_EVIDENCE_COMMAND,
        "collector_command": REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND,
        "gate_command": REAL_RUNTIME_EVIDENCE_GATE_COMMAND,
        "artifact": REAL_RUNTIME_EVIDENCE_ARTIFACT,
        "expected_runtime_contract": REAL_RUNTIME_CONTRACT,
        "requires_ros": True,
        "requires_real_robot_runtime": True,
        "requires_active_robot_run": True,
        "collector_publishes_control_topics": False,
        "control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        "validates": list(REAL_RUNTIME_EVIDENCE_VALIDATES),
    },
    "saved_map_artifact_gate": {
        "schema_version": "lingtu.saved_map_artifacts.gate.v1",
        "scope": "saved_map_artifact_provenance",
        "acceptance_step": 2,
        "required_when": "saved_map_tomogram_occupancy_or_pct_artifact_is_used",
        "requires_prior_gates": ["runtime_audit"],
        "conditional_prior_gates": [],
        "proves": list(SAVED_MAP_ARTIFACT_GATE_PROVES),
        "operator_summary_sections": list(
            SAVED_MAP_ARTIFACT_GATE_OPERATOR_SUMMARY_SECTIONS
        ),
        "command": SAVED_MAP_ARTIFACT_GATE_COMMAND,
        "script": "scripts/gates/saved_map_artifact_gate.py",
        "artifact": "artifacts/saved_map_artifacts/report.json",
        "requires_ros": False,
        "requires_real_robot_runtime": False,
        "requires_active_robot_run": False,
        "collector_publishes_control_topics": False,
        "control_topics_published": list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED),
        "validates": list(SAVED_MAP_ARTIFACT_GATE_VALIDATES),
    },
}

_GATE_ACCEPTANCE_EXPECTATIONS: dict[str, dict[str, Any]] = {
    "runtime_audit": {
        "acceptance_step": 1,
        "required_when": "before_any_runtime_contract_or_field_readiness_claim",
        "requires_prior_gates": [],
        "conditional_prior_gates": [],
        "proves": list(RUNTIME_AUDIT_PROVES),
        "operator_summary_sections": list(RUNTIME_AUDIT_OPERATOR_SUMMARY_SECTIONS),
    },
    "saved_map_artifact_gate": {
        "acceptance_step": 2,
        "required_when": "saved_map_tomogram_occupancy_or_pct_artifact_is_used",
        "requires_prior_gates": ["runtime_audit"],
        "conditional_prior_gates": [],
        "proves": list(SAVED_MAP_ARTIFACT_GATE_PROVES),
        "operator_summary_sections": list(
            SAVED_MAP_ARTIFACT_GATE_OPERATOR_SUMMARY_SECTIONS
        ),
    },
    "real_runtime_evidence": {
        "acceptance_step": 3,
        "required_when": "before_claiming_thunder_field_runtime_or_field_navigation",
        "requires_prior_gates": ["runtime_audit"],
        "conditional_prior_gates": [
            "saved_map_artifact_gate when saved map, tomogram, occupancy, or PCT artifact is used"
        ],
        "proves": list(REAL_RUNTIME_EVIDENCE_PROVES),
        "operator_summary_sections": list(
            REAL_RUNTIME_EVIDENCE_OPERATOR_SUMMARY_SECTIONS
        ),
    },
}


@dataclass(frozen=True)
class RuntimeValidationGateCheck:
    """Result of a system validation gate check with blockers and remedial commands."""
    ok: bool
    blockers: tuple[str, ...]
    commands: dict[str, str | None]
    validates: dict[str, list[str]]
    checks: dict[str, list[str]]
    coverage: dict[str, dict[str, list[str]]]
    acceptance: dict[str, dict[str, Any]]


def runtime_validation_gates() -> dict[str, dict[str, Any]]:
    """Return the canonical runtime validation gates for API and CLI surfaces."""

    return deepcopy(_RUNTIME_VALIDATION_GATES)


def _gate_acceptance_payload(gate: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "acceptance_step": gate.get("acceptance_step"),
        "required_when": gate.get("required_when"),
        "requires_prior_gates": list(gate.get("requires_prior_gates") or ()),
        "conditional_prior_gates": list(gate.get("conditional_prior_gates") or ()),
        "proves": list(gate.get("proves") or ()),
        "operator_summary_sections": list(
            gate.get("operator_summary_sections") or ()
        ),
    }


def validate_runtime_validation_gates(
    gates: Mapping[str, Mapping[str, Any]] | None = None,
) -> RuntimeValidationGateCheck:
    """Validate runtime validation gate descriptors for API/CLI drift."""

    blockers: list[str] = []
    source = gates if gates is not None else runtime_validation_gates()
    runtime_audit = source.get("runtime_audit")
    real_evidence = source.get("real_runtime_evidence")
    saved_map_artifacts = source.get("saved_map_artifact_gate")
    if not isinstance(runtime_audit, Mapping):
        blockers.append("runtime_audit validation gate missing")
        runtime_audit = {}
    if not isinstance(real_evidence, Mapping):
        blockers.append("real_runtime_evidence validation gate missing")
        real_evidence = {}
    if not isinstance(saved_map_artifacts, Mapping):
        blockers.append("saved_map_artifact_gate validation gate missing")
        saved_map_artifacts = {}

    acceptance = {
        "runtime_audit": _gate_acceptance_payload(runtime_audit),
        "saved_map_artifact_gate": _gate_acceptance_payload(saved_map_artifacts),
        "real_runtime_evidence": _gate_acceptance_payload(real_evidence),
    }
    for gate_name, expected in _GATE_ACCEPTANCE_EXPECTATIONS.items():
        if acceptance.get(gate_name) != expected:
            blockers.append(f"{gate_name} validation gate acceptance metadata drifted")

    if runtime_audit.get("command") != RUNTIME_AUDIT_COMMAND:
        blockers.append("runtime_audit validation gate command drifted")
    if runtime_audit.get("requires_ros") is not False:
        blockers.append("runtime_audit validation gate must not require ROS")
    if runtime_audit.get("collector_publishes_control_topics") is not False:
        blockers.append("runtime_audit validation gate must not publish control topics")
    expected_control_topics = list(REAL_RUNTIME_CONTROL_TOPICS_PUBLISHED)
    if runtime_audit.get("control_topics_published") != expected_control_topics:
        blockers.append("runtime_audit validation gate control topic list must be empty")
    if tuple(runtime_audit.get("validates") or ()) != RUNTIME_AUDIT_VALIDATES:
        blockers.append("runtime_audit validation gate validates list drifted")
    if tuple(runtime_audit.get("checks") or ()) != RUNTIME_AUDIT_CHECKS:
        blockers.append("runtime_audit validation gate checks list drifted")
    expected_coverage = {
        name: list(checks)
        for name, checks in RUNTIME_AUDIT_VALIDATE_CHECK_COVERAGE.items()
    }
    runtime_audit_coverage_source = runtime_audit.get("coverage")
    if runtime_audit_coverage_source is None:
        runtime_audit_coverage: Mapping[str, Any] = {}
    elif isinstance(runtime_audit_coverage_source, Mapping):
        runtime_audit_coverage = runtime_audit_coverage_source
    else:
        blockers.append("runtime_audit validation gate coverage map must be a mapping")
        runtime_audit_coverage = {}
    if runtime_audit_coverage != expected_coverage:
        blockers.append("runtime_audit validation gate coverage map drifted")
    if tuple(runtime_audit_coverage) != RUNTIME_AUDIT_VALIDATES:
        blockers.append("runtime_audit coverage keys do not match validates list")
    covered_checks = {
        str(check)
        for checks in runtime_audit_coverage.values()
        if isinstance(checks, list)
        for check in checks
    }
    missing_covered_checks = sorted(set(RUNTIME_AUDIT_CHECKS) - covered_checks)
    if missing_covered_checks:
        blockers.append(
            "runtime_audit checks have no validation coverage: "
            + ", ".join(missing_covered_checks)
        )
    unknown_covered_checks = sorted(covered_checks - set(RUNTIME_AUDIT_CHECKS))
    if unknown_covered_checks:
        blockers.append(
            "runtime_audit coverage references unknown checks: "
            + ", ".join(unknown_covered_checks)
        )

    if real_evidence.get("command") != REAL_RUNTIME_EVIDENCE_COMMAND:
        blockers.append("real_runtime_evidence validation gate command drifted")
    if real_evidence.get("collector_command") != REAL_RUNTIME_EVIDENCE_COLLECTOR_COMMAND:
        blockers.append(
            "real_runtime_evidence validation gate collector command drifted"
        )
    if real_evidence.get("gate_command") != REAL_RUNTIME_EVIDENCE_GATE_COMMAND:
        blockers.append("real_runtime_evidence validation gate gate command drifted")
    if real_evidence.get("expected_runtime_contract") != REAL_RUNTIME_CONTRACT:
        blockers.append(
            f"real_runtime_evidence expected contract is not {REAL_RUNTIME_CONTRACT}"
        )
    if real_evidence.get("requires_ros") is not True:
        blockers.append("real_runtime_evidence validation gate must require ROS")
    if real_evidence.get("requires_real_robot_runtime") is not True:
        blockers.append("real_runtime_evidence validation gate must require real runtime")
    if real_evidence.get("collector_publishes_control_topics") is not False:
        blockers.append("real_runtime_evidence collector must not publish control topics")
    if real_evidence.get("control_topics_published") != expected_control_topics:
        blockers.append("real_runtime_evidence control topic list must be empty")
    if tuple(real_evidence.get("validates") or ()) != REAL_RUNTIME_EVIDENCE_VALIDATES:
        blockers.append("real_runtime_evidence validation gate validates list drifted")

    if saved_map_artifacts.get("command") != SAVED_MAP_ARTIFACT_GATE_COMMAND:
        blockers.append("saved_map_artifact_gate validation gate command drifted")
    if saved_map_artifacts.get("requires_ros") is not False:
        blockers.append("saved_map_artifact_gate validation gate must not require ROS")
    if saved_map_artifacts.get("requires_real_robot_runtime") is not False:
        blockers.append("saved_map_artifact_gate validation gate must not require real runtime")
    if saved_map_artifacts.get("collector_publishes_control_topics") is not False:
        blockers.append("saved_map_artifact_gate must not publish control topics")
    if saved_map_artifacts.get("control_topics_published") != expected_control_topics:
        blockers.append("saved_map_artifact_gate control topic list must be empty")
    if tuple(saved_map_artifacts.get("validates") or ()) != SAVED_MAP_ARTIFACT_GATE_VALIDATES:
        blockers.append("saved_map_artifact_gate validation gate validates list drifted")

    return RuntimeValidationGateCheck(
        ok=not blockers,
        blockers=tuple(blockers),
        commands={
            "runtime_audit": runtime_audit.get("command"),
            "real_runtime_evidence": real_evidence.get("command"),
            "real_runtime_evidence_collector": real_evidence.get("collector_command"),
            "real_runtime_evidence_gate": real_evidence.get("gate_command"),
            "saved_map_artifact_gate": saved_map_artifacts.get("command"),
        },
        validates={
            "runtime_audit": list(runtime_audit.get("validates") or ()),
            "real_runtime_evidence": list(real_evidence.get("validates") or ()),
            "saved_map_artifact_gate": list(saved_map_artifacts.get("validates") or ()),
        },
        checks={
            "runtime_audit": list(runtime_audit.get("checks") or ()),
        },
        coverage={
            "runtime_audit": {
                str(name): list(checks) if isinstance(checks, list) else []
                for name, checks in runtime_audit_coverage.items()
            },
        },
        acceptance=acceptance,
    )
