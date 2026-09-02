# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path

from sim.runtime.qualification import production as production_qualification

ROOT = Path(__file__).resolve().parents[2] / "sim"
PLUGIN_SOURCE = (
    ROOT
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
    / "Source"
)
SESSION = PLUGIN_SOURCE / "LingTuSimSession"
VISUAL = PLUGIN_SOURCE / "LingTuSimVisual"


def test_session_ingress_routes_scenario_snapshot_on_existing_udp_port() -> None:
    service_header = (SESSION / "Public" / "LingTuSimSessionService.h").read_text(
        encoding="utf-8"
    )
    service_source = (SESSION / "Private" / "LingTuSimSessionService.cpp").read_text(
        encoding="utf-8"
    )
    module_source = (SESSION / "Private" / "LingTuSimSessionModule.cpp").read_text(
        encoding="utf-8"
    )

    assert "PublishScenarioSnapshotJson" in service_header
    assert "TryTakeLatestScenarioSnapshotJson" in service_header
    assert 'TEXT("lingtu.sim.truth-snapshot.v1")' in module_source
    assert 'TEXT("lingtu.sim.scenario-snapshot.v1")' in module_source
    assert "LingTuScenarioSnapshotPort" not in module_source
    assert "PendingScenarioSnapshotJson = SnapshotJson" in service_source
    assert "LastScenarioResetGeneration" in service_source
    assert "LastScenarioSequence" in service_source


def test_session_ingress_rejects_oversized_datagram_before_json_dispatch() -> None:
    module_source = (SESSION / "Private" / "LingTuSimSessionModule.cpp").read_text(
        encoding="utf-8"
    )

    assert "PendingBytes > MaxSnapshotDatagramBytes" in module_source
    assert "Rejected oversized snapshot datagram" in module_source


def test_visual_sink_projects_the_canonical_snapshot_without_recomputing_motion() -> None:
    subsystem_header = (
        VISUAL / "Public" / "LingTuSimVisualWorldSubsystem.h"
    ).read_text(encoding="utf-8")
    subsystem_source = (
        VISUAL / "Private" / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    registry_header = (
        VISUAL / "Public" / "LingTuSimScenarioVisualRegistry.h"
    ).read_text(encoding="utf-8")
    registry_source = (
        VISUAL / "Private" / "LingTuSimScenarioVisualRegistry.cpp"
    ).read_text(encoding="utf-8")

    assert "SubmitScenarioSnapshotJson" in subsystem_header
    assert "TryTakeLatestScenarioSnapshotJson" in subsystem_source
    assert "ApplySnapshotJson" in registry_header
    assert "const FString& SnapshotJson" in registry_header
    assert "FScenarioSnapshot" not in registry_header
    assert 'TEXT("lingtu.sim.scenario-snapshot.v1")' in registry_source
    assert 'Authority != TEXT("scenario")' in registry_source
    assert 'Authority != TEXT("ue_animation")' in registry_source
    assert "FCoordinateConverter::TryMakeWorldTransform" in registry_source
    assert "INVALID_OBJECTNAME_CHARACTERS" in registry_source
    assert "ReplaceCharInline" in registry_source
    assert "linear_crossing" not in registry_source.lower()


def test_visual_sink_has_actor_transform_and_evidence_automation_coverage() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")
    registry_source = (
        VISUAL / "Private" / "LingTuSimScenarioVisualRegistry.cpp"
    ).read_text(encoding="utf-8")

    assert "CreatesActorsFromCanonicalSnapshot" in automation
    assert "snapshot x converts from metres to centimetres" in automation
    assert "snapshot y changes handedness" in automation
    assert "maximum_position_error_m" in automation
    assert "position_tolerance_m" in registry_source
    assert "PositionToleranceMeters = 0.02" in registry_source
    assert 'TEXT("ue_registry_applied")' in registry_source
    assert "canonical_scenario_snapshot" in registry_source
    assert 'TEXT("run_id")' in registry_source
    assert 'TEXT("entity_id")' in registry_source
    assert "expected_actor_count" in registry_source
    assert "complete_actor_set" in registry_source
    assert "all_actors_visible" in registry_source


def test_visual_sink_has_identity_ordering_and_reset_rebuild_coverage() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")

    assert "RejectsWrongIdentityAndOutOfOrder" in automation
    assert "ResetGenerationRebuildsActorsFromFirstObservedSequence" in automation
    assert "first observed sequence may be non-zero" in automation
    assert "duplicate sequence is rejected" in automation
    assert "wrong session id is rejected" in automation
    assert "old reset generation is rejected" in automation
    assert "reset rebuild replaces actor identity" in automation


def test_visual_sink_has_strict_identity_set_validation_coverage() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")
    registry_source = (
        VISUAL / "Private" / "LingTuSimScenarioVisualRegistry.cpp"
    ).read_text(encoding="utf-8")

    assert "RejectsDuplicateStableIdAndMalformedEntity" in automation
    assert "EntitySetChangesOnlyOnReset" in automation
    assert "duplicate stable_id is rejected" in automation
    assert "unknown entity field is rejected" in automation
    assert "unsupported authority is rejected" in automation
    assert "empty run_id is rejected" in automation
    assert "run_id cannot change after binding" in automation
    assert "entity set change outside reset is rejected" in automation
    assert "DuplicateStableId" in registry_source
    assert "EntitySetMismatch" in registry_source


def test_visual_sink_rejects_same_reset_frame_without_partial_actor_update() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")
    actor_source = (
        VISUAL / "Private" / "LingTuSimScenarioActor.cpp"
    ).read_text(encoding="utf-8")
    registry_source = (
        VISUAL / "Private" / "LingTuSimScenarioVisualRegistry.cpp"
    ).read_text(encoding="utf-8")

    assert "SameResetFailureKeepsCompletePreviousFrame" in automation
    assert "failed same-reset frame leaves earlier actor at previous pose" in automation
    assert "CanConfigureSnapshotIdentity" in actor_source
    assert "SetActorHiddenInGame(true)" in actor_source
    assert "SetActorHiddenInGame(false)" in registry_source
    assert "PreviousActorStates" in registry_source


def test_session_raw_scenario_mailbox_has_ordering_automation_coverage() -> None:
    automation = (
        SESSION / "Private" / "Tests" / "LingTuSimSnapshotMailboxTest.cpp"
    ).read_text(encoding="utf-8")

    assert "ScenarioRawMailboxLatestWins" in automation
    assert "first observed reset sequence can be non-zero" in automation
    assert "wrong scenario model generation is rejected" in automation
    assert "stale scenario sequence is rejected" in automation
    assert "raw scenario JSON is copied and consumed" in automation


def test_visual_subsystem_has_mailbox_to_evidence_integration_coverage() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")
    subsystem_source = (
        VISUAL / "Private" / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")

    assert "MailboxToGameThreadRegistryWritesEvidence" in automation
    assert "scenario-visual-evidence.json" in automation
    assert "evidence session_id matches source snapshot" in automation
    assert "evidence sim_time_ns matches source snapshot" in automation
    assert "written evidence run_id matches authoritative allocation context" in automation
    assert "written evidence actor keeps canonical entity_id" in automation
    assert "ApplyLatestScenarioSnapshot();" in subsystem_source
    assert 'TEXT("LingTuRunId=")' in subsystem_source
    assert "CommandLineRunId != AllocationRunId" in subsystem_source


def test_allocation_backed_visual_evidence_requires_explicit_generations() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")
    subsystem_source = (
        VISUAL / "Private" / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")

    assert "ReadRequiredCommandLineGeneration" in subsystem_source
    init_body = _function_body(
        subsystem_source,
        "void ULingTuSimVisualWorldSubsystem::InitializeReadinessEvidenceFromCommandLine()",
        "void ULingTuSimVisualWorldSubsystem::TryWritePreparedReadinessEvidence()",
    )
    assert (
        'ReadRequiredCommandLineGeneration(TEXT("LingTuModelGeneration="), ModelGeneration)'
        in init_body
    )
    assert (
        'ReadRequiredCommandLineGeneration(TEXT("LingTuResetGeneration="), ResetGeneration)'
        in init_body
    )
    assert (
        'ReadCommandLineGeneration(TEXT("LingTuModelGeneration="), 0, ModelGeneration)'
        not in init_body
    )
    assert (
        'ReadCommandLineGeneration(TEXT("LingTuResetGeneration="), 0, ResetGeneration)'
        not in init_body
    )
    assert "RejectsMissingAllocationGenerationArguments" in automation
    assert "missing model generation is rejected" in automation
    assert "missing reset generation is rejected" in automation


def test_allocation_backed_visual_evidence_rejects_unsafe_log_directory() -> None:
    automation = (
        VISUAL / "Private" / "Tests" / "LingTuSimScenarioVisualTest.cpp"
    ).read_text(encoding="utf-8")
    subsystem_source = (
        VISUAL / "Private" / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")

    assert "ValidateRunAllocationEvidencePaths" in subsystem_source
    init_body = _function_body(
        subsystem_source,
        "void ULingTuSimVisualWorldSubsystem::InitializeReadinessEvidenceFromCommandLine()",
        "void ULingTuSimVisualWorldSubsystem::TryWritePreparedReadinessEvidence()",
    )
    assert "ValidateRunAllocationFilePath" in init_body
    assert "ValidateRunAllocationEvidencePaths" in init_body
    assert "CanonicalLogDirectory" in init_body
    assert "ConfigureReadinessEvidence(" in init_body
    assert "IFileManager::Get().IsSymlink" in subsystem_source
    assert "RejectsUnsafeRunAllocationEvidencePaths" in automation
    assert "relative log_dir is rejected" in automation
    assert "traversal log_dir is rejected" in automation
    assert "foreign log_dir is rejected" in automation


def _function_body(source: str, start_marker: str, end_marker: str) -> str:
    start = source.index(start_marker)
    end = source.index(end_marker, start)
    return source[start:end]


def test_ue_shaped_scenario_evidence_satisfies_production_visual_contract(
    tmp_path: Path,
) -> None:
    run_id = "ue-evidence-contract"
    session_id = "ue-session"
    evidence = {
        "schema": "lingtu.sim.scenario-visual-evidence.v1",
        "source": "ue_registry_applied",
        "input_source": "canonical_scenario_snapshot",
        "run_id": run_id,
        "session_id": session_id,
        "model_generation": 2,
        "reset_generation": 1,
        "sequence": 9,
        "sim_time_ns": 900_000_000,
        "basis": "snapshot_pose_applied_to_unreal_actor",
        "position_tolerance_m": 0.02,
        "maximum_position_error_m": 0.0,
        "within_tolerance": True,
        "expected_actor_count": 1,
        "actor_count": 1,
        "complete_actor_set": True,
        "all_actors_visible": True,
        "actors": [
            {
                "entity_id": "pedestrian_01",
                "stable_id": "pedestrian_01",
                "authority": "scenario",
                "semantic_class": "person",
                "source_epoch": 0,
                "visible": True,
                "expected_position_m": [1.0, 2.0, 0.9],
                "observed_position_m": [1.0, 2.0, 0.9],
                "position_error_m": 0.0,
            }
        ],
    }
    (tmp_path / "scenario-visual-evidence.json").write_text(
        json.dumps(evidence), encoding="utf-8"
    )

    result = production_qualification._scenario_visual_check(
        tmp_path,
        {"scenario_visual_evidence": "scenario-visual-evidence.json"},
        {
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 2,
            "reset_generation": 1,
        },
        expected_entity_ids=frozenset({"pedestrian_01"}),
    )

    assert result["qualified"] is True
    assert result["entity_ids"] == ["pedestrian_01"]
