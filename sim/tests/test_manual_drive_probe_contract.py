"""Behavioral contracts for the manual RobotSimUE drive qualification probe."""

# ruff: noqa: S101, S603

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
PROBE_PATH = REPO_ROOT / "build/manual-playable/probe_drive_input.ps1"
POWERSHELL = shutil.which("powershell")

RUN_ID = "manual-contract-run"
SESSION_ID = "manual-drive-session"
BOOT_ID = f"{RUN_ID}-boot"
MODEL_GENERATION = 7
RESET_GENERATION = 3


def test_live_probe_keeps_capture_evidence_inside_the_exact_run_directory() -> None:
    script = PROBE_PATH.read_text(encoding="utf-8")

    assert 'Join-Path $RunDirectory "drive-probe"' in script
    assert "baseline_a = $baselineA" in script
    assert "baseline_b = $baselineB" in script
    assert "driven = $driven" in script
    assert "LINGTU_DRIVE_PROBE_FAILED stage={0} error={1}" in script
    assert '$script:LingTuDriveProbeStage = "input_delivery"' in script
    assert "function Acquire-ProbeForegroundWindow" in script
    assert "foreground window is unavailable after bounded retries" in script
    assert '$process = Get-Process -Id $ProcessId -ErrorAction Stop' in script

_FUNCTION_DRIVER = r"""
$ErrorActionPreference = "Stop"
$tokens = $null
$errors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile(
    $env:LINGTU_PROBE_PATH,
    [ref]$tokens,
    [ref]$errors
)
if ($errors.Count -gt 0) {
    throw ($errors | ForEach-Object { $_.Message } | Out-String)
}
$definitions = $ast.FindAll({
    param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst]
}, $true)
foreach ($definition in $definitions) {
    Invoke-Expression $definition.Extent.Text
}

$fixture = Get-Content -LiteralPath $env:LINGTU_PROBE_FIXTURE -Raw | ConvertFrom-Json
if ($fixture.operation -eq "contract") {
    Assert-LocomotionProbeContract `
        -RequireLocomotion ([bool]$fixture.require_locomotion) `
        -InputMethod ([string]$fixture.input_method) `
        -AllowTargetedLocomotionDiagnostic ([bool]$fixture.allow_targeted) `
        -RunDirectory ([string]$fixture.run_directory) `
        -ExpectedRunId ([string]$fixture.expected_run_id) `
        -ProductSessionId ([string]$fixture.expected_session_id) | Out-Null
    [ordered]@{ passed = $true } | ConvertTo-Json -Compress
    exit 0
}
if ($fixture.operation -eq "identity") {
    Get-RunIdentityQualification `
        -RunDirectory ([string]$fixture.run_directory) `
        -ExpectedRunId ([string]$fixture.expected_run_id) `
        -ProductSessionId ([string]$fixture.expected_session_id) `
        -Allocation $fixture.allocation `
        -RuntimeBefore $fixture.runtime_before `
        -RuntimeAfter $fixture.runtime_after `
        -HealthBefore $fixture.health_before `
        -HealthAfter $fixture.health_after `
        -ControlEvidenceRecords @($fixture.control_records) `
        -SnapshotDocuments @($fixture.documents) |
        ConvertTo-Json -Depth 8 -Compress
    exit 0
}
if ($fixture.operation -eq "movement") {
    Get-SnapshotQualification `
        -Path "fixture://truth-snapshots" `
        -Documents @($fixture.documents) `
        -StableId "thunder_01/base_link" `
        -HoldStartTimeNs ([int64]$fixture.hold_start_time_ns) `
        -HoldEndTimeNs ([int64]$fixture.hold_end_time_ns) `
        -MinimumPositionDisplacementMeters 0.02 `
        -MinimumYawDeltaRadians 0.05 `
        -RequirePositionDisplacement $true |
        ConvertTo-Json -Depth 8 -Compress
    exit 0
}
if ($fixture.operation -eq "upright") {
    Get-UprightQualification `
        -Path "fixture://truth-snapshots" `
        -Documents @($fixture.documents) `
        -StableId "thunder_01/base_link" `
        -HoldStartTimeNs ([int64]$fixture.hold_start_time_ns) `
        -HoldEndTimeNs ([int64]$fixture.hold_end_time_ns) `
        -ReleaseStartTimeNs ([int64]$fixture.release_start_time_ns) `
        -ProductSessionId ([string]$fixture.expected_session_id) `
        -ExpectedModelGeneration ([int64]$fixture.expected_model_generation) `
        -ExpectedResetGeneration ([int64]$fixture.expected_reset_generation) `
        -MinimumHoldSamples 3 `
        -MinimumReleaseSamples 3 `
        -MinimumBodyUpDot 0.75 `
        -MaximumAbsoluteRollRadians 0.60 `
        -MaximumAbsolutePitchRadians 0.60 `
        -MinimumBaseHeightMeters 0.25 `
        -MaximumBaseHeightDropMeters 0.20 `
        -QuaternionNormTolerance 0.01 |
        ConvertTo-Json -Depth 8 -Compress
    exit 0
}
if ($fixture.operation -eq "release_stop") {
    Get-PostReleaseQualification `
        -Path "fixture://truth-snapshots" `
        -Documents @($fixture.documents) `
        -StableId "thunder_01/base_link" `
        -ReleaseStartTimeNs ([int64]$fixture.release_start_time_ns) `
        -MaximumDriftMeters 0.05 `
        -MaximumSpeedMps 0.08 |
        ConvertTo-Json -Depth 8 -Compress
    exit 0
}
if ($fixture.operation -eq "window_selection") {
    $candidates = @($fixture.candidates)
    $provider = { $candidates }.GetNewClosure()
    Get-RobotSimWindowCandidate `
        -ProcessId ([uint32]$fixture.expected_process_id) `
        -CandidateProvider $provider |
        ConvertTo-Json -Depth 8 -Compress
    exit 0
}
if ($fixture.operation -eq "foreground_handle") {
    $fixtureHandle = [int64]$fixture.handle
    $provider = { [IntPtr]$fixtureHandle }.GetNewClosure()
    $handle = Get-ProbeForegroundWindowHandle `
        -Required `
        -ForegroundWindowProvider $provider
    [ordered]@{ handle = $handle } | ConvertTo-Json -Compress
    exit 0
}
if ($fixture.operation -eq "uintptr") {
    $pointer = ConvertTo-ProbeUIntPtr -Value ([uint64]$fixture.value)
    [ordered]@{ value = $pointer.ToUInt64() } | ConvertTo-Json -Compress
    exit 0
}
if ($fixture.operation -eq "origin_correlation") {
    [ordered]@{
        press = Test-PressClaimCorrelation `
            -PressRecord $fixture.press `
            -ClaimRecord $fixture.claim
        hold = Test-HoldClaimCorrelation `
            -HoldRecord $fixture.hold `
            -ClaimRecord $fixture.claim
    } | ConvertTo-Json -Compress
    exit 0
}
if ($fixture.operation -eq "claim_selection") {
    $claims = @(Select-AcceptedControlClaims `
        -Records @($fixture.records) `
        -ExpectedRunId ([string]$fixture.expected_run_id) `
        -ProductSessionId ([string]$fixture.expected_session_id) `
        -ExpectedBootId ([string]$fixture.expected_boot_id) `
        -ExpectedModelGeneration ([int64]$fixture.expected_model_generation) `
        -ExpectedResetGeneration ([int64]$fixture.expected_reset_generation))
    [ordered]@{
        source_sequences = @($claims | ForEach-Object { [int64]$_.source_sequence })
    } | ConvertTo-Json -Depth 4 -Compress
    exit 0
}
throw "Unknown fixture operation: $($fixture.operation)"
"""


def _invoke_probe_functions(
    tmp_path: Path,
    fixture: dict[str, Any],
) -> subprocess.CompletedProcess[str]:
    if POWERSHELL is None:
        pytest.skip("Windows PowerShell is required for the probe contract")
    fixture_path = tmp_path / "probe-fixture.json"
    fixture_path.write_text(json.dumps(fixture), encoding="utf-8")
    environment = os.environ.copy()
    environment["LINGTU_PROBE_PATH"] = str(PROBE_PATH)
    environment["LINGTU_PROBE_FIXTURE"] = str(fixture_path)
    return subprocess.run(
        [
            POWERSHELL,
            "-NoLogo",
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            _FUNCTION_DRIVER,
        ],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        env=environment,
        timeout=20,
    )


def _result(tmp_path: Path, fixture: dict[str, Any]) -> dict[str, Any]:
    completed = _invoke_probe_functions(tmp_path, fixture)
    assert completed.returncode == 0, completed.stderr
    return json.loads(completed.stdout)


def _snapshot(
    time_ns: int,
    *,
    x: float,
    z: float = 0.55,
    quaternion_wxyz: list[float] | None = None,
    session_id: str = SESSION_ID,
    model_generation: int = MODEL_GENERATION,
    reset_generation: int = RESET_GENERATION,
) -> dict[str, Any]:
    return {
        "event": "snapshot",
        "sim_time_ns": time_ns,
        "sequence": time_ns // 100_000_000,
        "session_id": session_id,
        "model_generation": model_generation,
        "reset_generation": reset_generation,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "position_m": [x, 0.0, z],
                "quaternion_wxyz": quaternion_wxyz or [1.0, 0.0, 0.0, 0.0],
            }
        ],
    }


def _upright_documents() -> list[dict[str, Any]]:
    return [
        _snapshot(800_000_000, x=0.00, z=0.55),
        _snapshot(1_000_000_000, x=0.00, z=0.55),
        _snapshot(1_500_000_000, x=0.05, z=0.54),
        _snapshot(2_000_000_000, x=0.10, z=0.53),
        _snapshot(2_200_000_000, x=0.10, z=0.53),
        _snapshot(2_600_000_000, x=0.105, z=0.53),
        _snapshot(3_000_000_000, x=0.106, z=0.53),
    ]


def _upright_fixture(documents: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "operation": "upright",
        "documents": documents,
        "hold_start_time_ns": 1_000_000_000,
        "hold_end_time_ns": 2_000_000_000,
        "release_start_time_ns": 2_200_000_000,
        "expected_session_id": SESSION_ID,
        "expected_model_generation": MODEL_GENERATION,
        "expected_reset_generation": RESET_GENERATION,
    }


def _runtime(run_directory: Path) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "state": "RUNNING",
        "model_generation": MODEL_GENERATION,
        "reset_generation": RESET_GENERATION,
        "allocation": {"run_dir": str(run_directory)},
    }


def _health(*, monitor_sequence: int, event_sequence: int, truth_sequence: int) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.manual-runtime-health.v1",
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "session_state": "RUNNING",
        "owner_thread_alive": True,
        "monitor_sequence": monitor_sequence,
        "event_sequence": event_sequence,
        "non_running_transition_count": 0,
        "owner_thread_stop_count": 0,
        "model_generation": MODEL_GENERATION,
        "reset_generation": RESET_GENERATION,
        "last_truth_sequence": truth_sequence,
    }


def _window_candidate(
    handle: int,
    *,
    title: str,
    process_id: int = 4242,
    visible: bool = True,
    owner_handle: int = 0,
    window_width: int = 1280,
    window_height: int = 720,
    client_width: int = 1260,
    client_height: int = 680,
) -> dict[str, Any]:
    return {
        "Handle": handle,
        "ProcessId": process_id,
        "Visible": visible,
        "OwnerHandle": owner_handle,
        "Title": title,
        "WindowWidth": window_width,
        "WindowHeight": window_height,
        "ClientWidth": client_width,
        "ClientHeight": client_height,
    }


def test_window_selection_prefers_true_robotsimue_title_prefix_over_path_contains(
    tmp_path: Path,
) -> None:
    result = _result(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [
                _window_candidate(
                    11,
                    title=r"D:\Program Files\RobotSimUE\UnrealEditor.exe",
                    client_width=1920,
                    client_height=1080,
                ),
                _window_candidate(
                    22,
                    title="RobotSimUE （64-位 Development PCD3D_SM5）",
                    client_width=1280,
                    client_height=720,
                ),
            ],
        },
    )

    assert result["Handle"] == 22
    assert result["Title"].startswith("RobotSimUE")


def test_window_selection_rejects_path_only_title_without_project_prefix(
    tmp_path: Path,
) -> None:
    completed = _invoke_probe_functions(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [
                _window_candidate(
                    11,
                    title=r"D:\Program Files\RobotSimUE\UnrealEditor.exe",
                    client_width=1920,
                    client_height=1080,
                )
            ],
        },
    )

    assert completed.returncode != 0
    assert (
        "Process 4242 has no eligible window whose title starts with RobotSimUE"
        in completed.stderr
    )


def test_window_selection_empty_enumeration_uses_stable_eligibility_error(
    tmp_path: Path,
) -> None:
    completed = _invoke_probe_functions(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [],
        },
    )

    assert completed.returncode != 0
    assert (
        "Process 4242 has no eligible visible unowned window with non-zero window and client area"
        in completed.stderr
    )
    assert "Cannot bind argument" not in completed.stderr


@pytest.mark.parametrize(
    "malformed_title",
    [
        "RobotSimUEvil",
        "RobotSimUE.exe",
        "RobotSimUE.EXE",
        r"RobotSimUE\UnrealEditor.exe",
        "RobotSimUE / UnrealEditor",
        r"RobotSimUE \ UnrealEditor",
    ],
)
def test_window_selection_rejects_non_project_prefix_and_path_shaped_titles(
    tmp_path: Path,
    malformed_title: str,
) -> None:
    completed = _invoke_probe_functions(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [
                _window_candidate(
                    11,
                    title=malformed_title,
                    client_width=1920,
                    client_height=1080,
                )
            ],
        },
    )

    assert completed.returncode != 0
    assert (
        "Process 4242 has no eligible window whose title starts with RobotSimUE"
        in completed.stderr
    )


@pytest.mark.parametrize(
    "project_title",
    [
        "RobotSimUE",
        "RobotSimUE Development",
        "RobotSimUE(Development)",
        "RobotSimUE（Development）",
    ],
)
def test_window_selection_accepts_only_frozen_project_title_boundaries(
    tmp_path: Path,
    project_title: str,
) -> None:
    result = _result(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [_window_candidate(22, title=project_title)],
        },
    )

    assert result["Handle"] == 22


def test_production_window_path_uses_discovery_seam_without_main_window_fallback() -> None:
    script = PROBE_PATH.read_text(encoding="utf-8")

    assert "$windowCandidate = Get-RobotSimWindowCandidate" in script
    assert "EnumerateTopLevelWindows(uint requestedProcessId)" in script
    assert ".MainWindowHandle" not in script
    assert "FindLargestTopLevelWindow" not in script


def test_window_selection_filters_ownership_visibility_and_zero_areas_then_uses_largest(
    tmp_path: Path,
) -> None:
    result = _result(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [
                _window_candidate(
                    10,
                    title="RobotSimUE foreign",
                    process_id=9999,
                    client_width=3000,
                    client_height=2000,
                ),
                _window_candidate(
                    11,
                    title="RobotSimUE invisible",
                    visible=False,
                    client_width=2800,
                    client_height=1800,
                ),
                _window_candidate(
                    12,
                    title="RobotSimUE owned popup",
                    owner_handle=77,
                    client_width=2600,
                    client_height=1600,
                ),
                _window_candidate(
                    13,
                    title="RobotSimUE zero window",
                    window_width=0,
                    client_width=2400,
                    client_height=1400,
                ),
                _window_candidate(
                    14,
                    title="RobotSimUE zero client",
                    client_height=0,
                ),
                _window_candidate(
                    21,
                    title="RobotSimUE valid small",
                    client_width=800,
                    client_height=600,
                ),
                _window_candidate(
                    22,
                    title="RobotSimUE valid largest",
                    client_width=1280,
                    client_height=720,
                ),
            ],
        },
    )

    assert result["Handle"] == 22


def test_window_selection_fails_closed_when_largest_client_area_is_tied(
    tmp_path: Path,
) -> None:
    completed = _invoke_probe_functions(
        tmp_path,
        {
            "operation": "window_selection",
            "expected_process_id": 4242,
            "candidates": [
                _window_candidate(
                    31,
                    title="RobotSimUE primary A",
                    client_width=1280,
                    client_height=720,
                ),
                _window_candidate(
                    32,
                    title="RobotSimUE primary B",
                    client_width=960,
                    client_height=960,
                ),
            ],
        },
    )

    assert completed.returncode != 0
    assert (
        "Process 4242 has multiple equally sized eligible RobotSimUE windows"
        in completed.stderr
    )


def test_required_foreground_zero_uses_stable_fail_closed_error(
    tmp_path: Path,
) -> None:
    completed = _invoke_probe_functions(
        tmp_path,
        {"operation": "foreground_handle", "handle": 0},
    )

    assert completed.returncode != 0
    assert "RobotSimUE foreground window is unavailable" in completed.stderr
    assert "Cannot convert" not in completed.stderr
    assert "TypeError" not in completed.stderr


def test_targeted_key_values_are_marshaled_through_uint64_to_uintptr(
    tmp_path: Path,
) -> None:
    assert _result(tmp_path, {"operation": "uintptr", "value": 0xA0}) == {
        "value": 0xA0
    }


def test_control_claim_correlation_uses_source_identity_sequence_and_time(
    tmp_path: Path,
) -> None:
    fixture = {
        "operation": "origin_correlation",
        "press": {
            "source_id": "robotsimue.local_player.0",
            "source_epoch": 1,
            "source_sequence": 3,
            "source_monotonic_ns": 8_745_816_998,
            "datagram_sha256": "a" * 64,
        },
        "claim": {
            "event": "runtime_request_accepted",
            "request": "control_claim",
            "status": "accepted",
            "source_id": "robotsimue.local_player.0",
            "source_epoch": 1,
            "source_sequence": 4,
            "source_monotonic_ns": 8_745_816_998,
            "datagram_sha256": "b" * 64,
        },
        "hold": {
            "source_id": "robotsimue.local_player.0",
            "source_epoch": 1,
            "source_sequence": 7,
            "source_monotonic_ns": 8_888_000_100,
            "datagram_sha256": "c" * 64,
        },
    }

    assert _result(tmp_path, fixture) == {"press": True, "hold": True}
    assert _result(
        tmp_path,
        {
            **fixture,
            "press": {
                **fixture["press"],
                "source_sequence": 6,
                "source_monotonic_ns": fixture["claim"]["source_monotonic_ns"]
                + 56_300_000,
            },
        },
    ) == {"press": True, "hold": True}
    assert _result(
        tmp_path,
        {
            **fixture,
            "claim": {**fixture["claim"], "source_id": "other.source"},
        },
    ) == {"press": False, "hold": False}
    assert _result(
        tmp_path,
        {
            **fixture,
            "claim": {
                **fixture["claim"],
                "source_monotonic_ns": fixture["claim"]["source_monotonic_ns"]
                + 250_000_001,
            },
        },
    ) == {"press": False, "hold": True}


def test_control_claim_selection_uses_runtime_trace_and_exact_run_identity(
    tmp_path: Path,
) -> None:
    valid = {
        "event": "runtime_request_accepted",
        "request": "control_claim",
        "status": "accepted",
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "boot_id": BOOT_ID,
        "model_generation": MODEL_GENERATION,
        "reset_generation": RESET_GENERATION,
        "source_id": "robotsimue.local_player.0",
        "source_epoch": 1,
        "source_sequence": 4,
        "source_monotonic_ns": 17_774_298_101,
    }
    fixture = {
        "operation": "claim_selection",
        "expected_run_id": RUN_ID,
        "expected_session_id": SESSION_ID,
        "expected_boot_id": BOOT_ID,
        "expected_model_generation": MODEL_GENERATION,
        "expected_reset_generation": RESET_GENERATION,
        "records": [
            {**valid, "event": "runtime_request_received"},
            valid,
            {**valid, "run_id": "stale-run", "source_sequence": 40},
            {**valid, "model_generation": MODEL_GENERATION + 1, "source_sequence": 41},
            {**valid, "request": "control_release", "source_sequence": 42},
        ],
    }

    assert _result(tmp_path, fixture) == {"source_sequences": [4]}
    script = PROBE_PATH.read_text(encoding="utf-8")
    assert 'Join-Path $RunDirectory "runtime-request-trace.jsonl"' in script
    assert "Read-JsonLines -Path $runtimeRequestTracePath -Skip 0" in script


def test_require_locomotion_requires_physical_foreground_and_explicit_identity(
    tmp_path: Path,
) -> None:
    base = {
        "operation": "contract",
        "require_locomotion": True,
        "input_method": "ForegroundInput",
        "run_directory": str(tmp_path.resolve()),
        "expected_run_id": RUN_ID,
        "expected_session_id": SESSION_ID,
    }
    assert _result(tmp_path, base)["passed"] is True

    targeted = {**base, "input_method": "TargetedMessages"}
    completed = _invoke_probe_functions(tmp_path, targeted)
    assert completed.returncode != 0
    assert "AllowTargetedLocomotionDiagnostic" in completed.stderr

    targeted_opt_in = {**targeted, "allow_targeted": True}
    assert _result(tmp_path, targeted_opt_in)["passed"] is True

    unpinned = {**base, "expected_session_id": ""}
    completed = _invoke_probe_functions(tmp_path, unpinned)
    assert completed.returncode != 0
    assert "ProductSessionId" in completed.stderr


def test_identity_qualification_rejects_cross_run_or_cross_generation_truth(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path.resolve()
    runtime = _runtime(run_directory)
    allocation = {
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "boot_id": BOOT_ID,
        "log_dir": str(run_directory / "logs"),
    }
    control_record = {
        "run_id": RUN_ID,
        "session_id": SESSION_ID,
        "boot_id": BOOT_ID,
        "model_generation": MODEL_GENERATION,
        "reset_generation": RESET_GENERATION,
    }
    fixture = {
        "operation": "identity",
        "run_directory": str(run_directory),
        "expected_run_id": RUN_ID,
        "expected_session_id": SESSION_ID,
        "allocation": allocation,
        "runtime_before": runtime,
        "runtime_after": runtime,
        "health_before": _health(
            monitor_sequence=10,
            event_sequence=100,
            truth_sequence=1_000,
        ),
        "health_after": _health(
            monitor_sequence=20,
            event_sequence=200,
            truth_sequence=2_000,
        ),
        "control_records": [control_record],
        "documents": _upright_documents(),
    }
    assert _result(tmp_path, fixture)["passed"] is True

    wrong_run = json.loads(json.dumps(fixture))
    wrong_run["control_records"][0]["run_id"] = "manual-other-run"
    result = _result(tmp_path, wrong_run)
    assert result["passed"] is False
    assert "control_run_id_mismatch" in result["failure_codes"]

    wrong_generation = json.loads(json.dumps(fixture))
    wrong_generation["documents"][3]["model_generation"] += 1
    result = _result(tmp_path, wrong_generation)
    assert result["passed"] is False
    assert "snapshot_model_generation_mismatch" in result["failure_codes"]

    missing_control = json.loads(json.dumps(fixture))
    missing_control["control_records"] = []
    result = _result(tmp_path, missing_control)
    assert result["passed"] is False
    assert "control_evidence_missing" in result["failure_codes"]

    transient_non_running = json.loads(json.dumps(fixture))
    transient_non_running["health_after"]["non_running_transition_count"] = 1
    result = _result(tmp_path, transient_non_running)
    assert result["passed"] is False
    assert "health_non_running_transition_observed" in result["failure_codes"]


def test_shift_w_locomotion_cannot_pass_with_orientation_only_change(
    tmp_path: Path,
) -> None:
    yaw = 0.2
    documents = [
        _snapshot(1_000_000_000, x=0.0),
        _snapshot(
            2_000_000_000,
            x=0.0,
            quaternion_wxyz=[math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
        ),
    ]
    result = _result(
        tmp_path,
        {
            "operation": "movement",
            "documents": documents,
            "hold_start_time_ns": 1_000_000_000,
            "hold_end_time_ns": 2_000_000_000,
        },
    )

    assert result["orientation_delta_rad"] == pytest.approx(yaw)
    assert result["position_displacement_m"] == pytest.approx(0.0)
    assert result["position_displacement_required"] is True
    assert result["state"] == "failed"
    assert result["passed"] is False


def test_release_stop_accepts_bounded_drift_and_settled_final_speed(
    tmp_path: Path,
) -> None:
    result = _result(
        tmp_path,
        {
            "operation": "release_stop",
            "release_start_time_ns": 2_200_000_000,
            "documents": [
                _snapshot(2_200_000_000, x=0.100),
                _snapshot(2_600_000_000, x=0.105),
                _snapshot(3_000_000_000, x=0.106),
            ],
        },
    )

    assert result["state"] == "passed"
    assert result["passed"] is True
    assert result["duration_seconds"] == pytest.approx(0.8)
    assert result["maximum_planar_drift_m"] == pytest.approx(0.006)
    assert result["final_planar_speed_mps"] == pytest.approx(0.0025)


@pytest.mark.parametrize(
    ("documents", "reason"),
    [
        (
            [
                _snapshot(2_200_000_000, x=0.0),
                _snapshot(2_300_000_000, x=0.0),
                _snapshot(2_400_000_000, x=0.0),
            ],
            "release snapshots covered less than 0.5 seconds",
        ),
        (
            [
                _snapshot(2_200_000_000, x=0.0),
                _snapshot(2_600_000_000, x=0.060),
                _snapshot(3_000_000_000, x=0.061),
            ],
            "robot continued drifting beyond the configured release threshold",
        ),
        (
            [
                _snapshot(2_200_000_000, x=0.0),
                _snapshot(2_600_000_000, x=0.0),
                _snapshot(3_000_000_000, x=0.040),
            ],
            "robot retained excessive speed after release",
        ),
    ],
)
def test_release_stop_fails_closed_on_short_drift_or_excess_final_speed(
    tmp_path: Path,
    documents: list[dict[str, Any]],
    reason: str,
) -> None:
    result = _result(
        tmp_path,
        {
            "operation": "release_stop",
            "release_start_time_ns": 2_200_000_000,
            "documents": documents,
        },
    )

    assert result["state"] == "failed"
    assert result["passed"] is False
    assert result["reason"] == reason


def test_upright_qualification_accepts_same_generation_hold_and_release_truth(
    tmp_path: Path,
) -> None:
    result = _result(tmp_path, _upright_fixture(_upright_documents()))

    assert result["state"] == "passed"
    assert result["passed"] is True
    assert result["hold_samples"] == 3
    assert result["release_samples"] == 3
    assert result["minimum_body_up_dot"] == pytest.approx(1.0)
    assert result["maximum_absolute_roll_rad"] == pytest.approx(0.0)
    assert result["maximum_absolute_pitch_rad"] == pytest.approx(0.0)
    assert result["minimum_base_height_m"] == pytest.approx(0.53)
    assert result["maximum_base_height_drop_m"] == pytest.approx(0.02)
    assert result["failure_codes"] == []


@pytest.mark.parametrize(
    ("mutation", "failure_code"),
    [
        ("tipped", "minimum_body_up_dot"),
        ("too_low", "minimum_base_height"),
        ("height_drop", "maximum_base_height_drop"),
        ("invalid_quaternion", "invalid_quaternion"),
        ("cross_generation", "snapshot_reset_generation_mismatch"),
    ],
)
def test_upright_qualification_fails_closed_on_uncontrolled_truth(
    tmp_path: Path,
    mutation: str,
    failure_code: str,
) -> None:
    documents = _upright_documents()
    if mutation == "tipped":
        angle = math.radians(70.0)
        documents[3]["bodies"][0]["quaternion_wxyz"] = [
            math.cos(angle / 2.0),
            math.sin(angle / 2.0),
            0.0,
            0.0,
        ]
    elif mutation == "too_low":
        for document in documents:
            document["bodies"][0]["position_m"][2] = 0.10
    elif mutation == "height_drop":
        documents[3]["bodies"][0]["position_m"][2] = 0.30
    elif mutation == "invalid_quaternion":
        documents[4]["bodies"][0]["quaternion_wxyz"] = [0.0, 0.0, 0.0, 0.0]
    elif mutation == "cross_generation":
        documents[5]["reset_generation"] += 1

    result = _result(tmp_path, _upright_fixture(documents))

    assert result["state"] == "failed"
    assert result["passed"] is False
    assert failure_code in result["failure_codes"]
