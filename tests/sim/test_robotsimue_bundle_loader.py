# ruff: noqa: S101

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "sim"
RUNTIME = (
    ROOT
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
    / "Source"
    / "LingTuSimRuntime"
)


def _read(relative_path: str) -> str:
    return (RUNTIME / relative_path).read_text(encoding="utf-8")


def test_exposes_one_deep_loader_interface_with_structured_errors() -> None:
    header = _read("Public/LingTuSimBundleLoader.h")
    build_rules = _read("LingTuSimRuntime.Build.cs")

    assert "class LINGTUSIMRUNTIME_API FSessionBundleLoader final" in header
    for entry_point in (
        "LoadSessionBundle",
        "LoadSnapshotFile",
        "ParseSnapshotJson",
    ):
        assert re.search(rf"static\s+bool\s+{entry_point}\s*\(", header)

    assert "enum class ERuntimeLoadErrorCode" in header
    assert "struct LINGTUSIMRUNTIME_API FRuntimeLoadError" in header
    assert "FRuntimeLoadError& OutError" in header

    dependencies = set(re.findall(r'"([A-Za-z0-9_]+)"', build_rules))
    assert {"Core", "Json", "JsonUtilities"} <= dependencies


def test_loads_the_compiled_bundle_boundary_with_optional_scenario() -> None:
    implementation = _read("Private/LingTuSimBundleLoader.cpp")
    runtime_types = _read("Public/LingTuSimRuntimeTypes.h")

    artifact_contracts = {
        "physics.plan.json": "lingtu.sim.physics-plan.v1",
        "visual.plan.json": "lingtu.sim.visual-plan.v1",
        "sensor.plan.json": "lingtu.sim.sensor-plan.v1",
        "control.plan.json": "lingtu.sim.control-plan.v1",
        "scenario.plan.json": "lingtu.sim.scenario-plan.v1",
        "transport.intent.json": "lingtu.sim.transport-intent.v1",
    }
    for filename, schema in artifact_contracts.items():
        assert filename in implementation
        assert schema in implementation

    assert "IFileManager::Get().FileExists" in implementation
    assert "&& !ScenarioPlanPath.IsEmpty()" not in runtime_types


def test_loader_uses_the_session_slug_contract() -> None:
    implementation = _read("Private/LingTuSimBundleLoader.cpp")
    tests = _read("Private/Tests/LingTuSimBundleLoaderTest.cpp")

    assert "Value.Len() > 63" in implementation
    assert "Character >= TEXT('A')" in implementation
    assert "Character >= TEXT('a')" in implementation
    assert "Character >= TEXT('0')" in implementation
    for punctuation in ("_", ".", "-"):
        assert f"Character != TEXT('{punctuation}')" in implementation
    assert "64-character session_id is rejected" in tests
    assert "session_id with a space is rejected" in tests


def test_parses_immutable_snapshots_with_dynamic_entity_cardinality() -> None:
    implementation = _read("Private/LingTuSimBundleLoader.cpp")

    for field in (
        "lingtu.sim.truth-snapshot.v1",
        "session_id",
        "model_generation",
        "reset_generation",
        "sequence",
        "sim_time_ns",
        "entities",
        "bodies",
        "stable_id",
        "instance_id",
        "frame_id",
        "position_m",
        "quaternion_wxyz",
        "linear_velocity_mps",
        "angular_velocity_rps",
    ):
        assert field in implementation

    assert "ExpectedSessionId" in implementation
    assert "EntityValues->Num()" in implementation
    assert "Candidate.Entities.Reserve(EntityValues->Num())" in implementation
    assert "Candidate.Entities.Add" in implementation
    assert "OutSnapshot = MoveTemp(Candidate)" in implementation
    assert "FMath::IsFinite" in implementation
    assert "ReadNonNegativeUint64Field" in implementation
    assert "ReadNonNegativeInt64Field" in implementation
    assert "Value->Type != EJson::Number" in implementation
    assert not re.search(r"\b(?:Thunder|thunder|body_count)\b", implementation)
    assert not re.search(r"(?:Reserve|Num|SetNum)\s*\(\s*21\s*\)", implementation)


def test_loader_returns_errors_without_crossing_the_compiled_json_boundary() -> None:
    implementation = _read("Private/LingTuSimBundleLoader.cpp")
    header = _read("Public/LingTuSimBundleLoader.h")
    build_rules = _read("LingTuSimRuntime.Build.cs")
    loader_surface = "\n".join((implementation, header, build_rules))

    for error_code in (
        "InvalidArgument",
        "MissingArtifact",
        "ReadFailed",
        "InvalidJson",
        "SchemaMismatch",
        "InvalidField",
    ):
        assert error_code in loader_surface

    assert "FJsonSerializer::Deserialize" in implementation
    assert "FFileHelper::LoadFileToString" in implementation
    assert not re.search(
        r"\b(?:check|checkf|verify|ensure|ensureMsgf)\s*\(", implementation
    )
    assert "UE_LOG(Fatal" not in implementation

    for forbidden in (
        "RobotConfig",
        "MJCF",
        "mj_loadXML",
        "mjs_loadXML",
        "FXmlFile",
        "yaml",
        "YAML",
        "FPlatformProcess",
        "CreateProc",
        "systemd",
    ):
        assert forbidden not in loader_surface
