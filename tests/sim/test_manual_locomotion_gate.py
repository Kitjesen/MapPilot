"""Offline contracts for the manual UE-to-MuJoCo locomotion diagnostic gate."""


from __future__ import annotations

import json
import math
import os
import subprocess
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from sim.runtime.coordinator.manual_locomotion_gate import (
    ManualLocomotionGateConfig,
    build_locomotion_probe_command,
    supervise_manual_locomotion_gate,
)

RUN_ID = "manual-locomotion-contract"
SESSION_ID = "manual-locomotion-session"


def _runtime(run_dir: Path) -> None:
    (run_dir / "session.runtime.json").write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.session-runtime.v1",
                "run_id": RUN_ID,
                "session_id": SESSION_ID,
                "state": "RUNNING",
            }
        ),
        encoding="utf-8",
    )


def _passing_probe(
    run_dir: Path,
    *,
    process_id: int,
    input_method: str = "ForegroundInput",
) -> dict[str, Any]:
    foreground = input_method == "ForegroundInput"
    return {
        "schema": "lingtu.sim.debug.drive-input-probe.v3",
        "process_id": process_id,
        "input_method": input_method,
        "direction_key": "W",
        "run_directory": str(run_dir),
        "expected_run_id": RUN_ID,
        "expected_session_id": SESSION_ID,
        "locomotion_qualification_required": True,
        "physical_foreground_input": foreground,
        "window_handle": 9001,
        "foreground_before_input": 9001 if foreground else 0,
        "foreground_handle": 9001 if foreground else 0,
        "process_still_running": True,
        "new_received_intents": 10,
        "new_nonzero_accepted_commands": 8,
        "unexpected_nonzero_commands": 0,
        "command_accepted": True,
        "physics_motion_observed": True,
        "motion_observed": True,
        "max_observed_linear_speed_mps": 0.18,
        "press": {
            "passed": True,
            "accepted_nonzero_commands": 1,
            "accepted_control_claims": 1,
            "origin_matches": 1,
        },
        "hold": {
            "passed": True,
            "samples": 8,
            "minimum_samples": 2,
            "window_milliseconds": 2800.0,
            "minimum_window_milliseconds": 250.0,
            "origin_matches": 8,
        },
        "release": {
            "passed": True,
            "control_release_requests": 1,
            "released_authority_statuses": 1,
            "zero_audit_records": 1,
            "origin_matches": 1,
        },
        "transport_probe": {
            "passed": True,
            "input_received": True,
            "press": True,
            "hold": True,
            "release": True,
            "origin_correlation": True,
        },
        "run_identity_qualification": {
            "state": "passed",
            "passed": True,
            "control_records": 24,
            "snapshot_documents": 40,
            "failure_codes": [],
        },
        "locomotion_qualification": {
            "state": "passed",
            "passed": True,
            "position_displacement_required": True,
            "position_displacement_m": 0.21,
            "minimum_position_displacement_m": 0.02,
            "hold_samples": 20,
            "hold_start_time_ns": 1_000_000_000,
            "hold_end_time_ns": 4_000_000_000,
        },
        "release_stop_qualification": {
            "state": "passed",
            "passed": True,
            "duration_seconds": 1.2,
            "maximum_planar_drift_m": 0.01,
            "maximum_planar_drift_m_allowed": 0.05,
            "final_planar_speed_mps": 0.02,
            "maximum_final_planar_speed_mps": 0.08,
            "release_start_time_ns": 4_100_000_000,
        },
        "upright_qualification": {
            "state": "passed",
            "passed": True,
            "hold_samples": 20,
            "minimum_hold_samples": 3,
            "release_samples": 10,
            "minimum_release_samples": 3,
            "minimum_base_height_m": 0.48,
            "minimum_base_height_m_allowed": 0.25,
            "maximum_base_height_drop_m": 0.03,
            "maximum_base_height_drop_m_allowed": 0.20,
            "minimum_body_up_dot": 0.98,
            "minimum_body_up_dot_allowed": 0.75,
            "maximum_absolute_roll_rad": 0.08,
            "maximum_absolute_roll_rad_allowed": 0.60,
            "maximum_absolute_pitch_rad": 0.09,
            "maximum_absolute_pitch_rad_allowed": 0.60,
            "failure_codes": [],
        },
        "passed": True,
    }


def _config(
    tmp_path: Path,
    *,
    process_id: int = 4242,
    input_method: str = "ForegroundInput",
) -> ManualLocomotionGateConfig:
    run_dir = (tmp_path / "run").resolve()
    run_dir.mkdir()
    probe = (tmp_path / "probe_drive_input.ps1").resolve()
    probe.write_text("# fixture", encoding="utf-8")
    return ManualLocomotionGateConfig(
        run_dir=run_dir,
        run_id=RUN_ID,
        session_id=SESSION_ID,
        unreal_pid=process_id,
        profile_deadline_monotonic=100.0,
        drive_probe_script=probe,
        input_method=input_method,
    )


def test_locomotion_command_is_run_bound_foreground_and_physical(
    tmp_path: Path,
) -> None:
    config = _config(tmp_path)

    command = build_locomotion_probe_command(config)

    assert command.count("-File") == 1
    assert command[command.index("-File") + 1] == str(config.drive_probe_script)
    assert command[command.index("-ProcessId") + 1] == str(config.unreal_pid)
    assert command[command.index("-InputMethod") + 1] == "ForegroundInput"
    assert command[command.index("-DirectionKey") + 1] == "W"
    assert command[command.index("-RunDirectory") + 1] == str(config.run_dir)
    assert command[command.index("-ExpectedRunId") + 1] == RUN_ID
    assert command[command.index("-ProductSessionId") + 1] == SESSION_ID
    assert command[command.index("-SnapshotPath") + 1] == str(
        config.run_dir / "truth-snapshots.jsonl"
    )
    assert command.count("-RequireLocomotion") == 1


def test_direction_key_is_parameterized(
    tmp_path: Path,
) -> None:
    config = _config(tmp_path)
    config = replace(config, direction_key="A")

    command = build_locomotion_probe_command(config)
    assert command[command.index("-DirectionKey") + 1] == "A"


def test_turn_gate_uses_angular_motion_not_planar_displacement(tmp_path: Path) -> None:
    config = replace(_config(tmp_path), direction_key="Q")
    _runtime(config.run_dir)
    probe = _passing_probe(config.run_dir, process_id=config.unreal_pid)
    probe["direction_key"] = "Q"
    probe["max_observed_linear_speed_mps"] = 0.001
    probe["max_observed_angular_speed_radps"] = .55
    probe["locomotion_qualification"].update({
        "position_displacement_required": False,
        "position_displacement_m": .001,
        "orientation_delta_rad": 1.2,
        "yaw_delta_rad": 1.2,
        "minimum_orientation_delta_rad": .8,
    })
    result = supervise_manual_locomotion_gate(
        config, prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0, stdout=json.dumps(probe), stderr=""),
        monotonic=lambda: 10.0, sleep=lambda _seconds: None, release_input=lambda: None,
    )
    assert result.passed is True


def test_ten_second_hold_requires_budget_for_capture_and_release(tmp_path: Path) -> None:
    config = replace(
        _config(tmp_path),
        hold_milliseconds=10_000,
        release_settle_milliseconds=800,
        profile_deadline_monotonic=25.5,
    )
    _runtime(config.run_dir)
    calls: list[tuple[str, ...]] = []

    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda command, **_kwargs: calls.append(command),
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert result.passed is False
    assert calls == []
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["failure_reason"] == (
        "profile deadline cannot safely cover the locomotion probe"
    )


def test_targeted_diagnostic_is_explicit_and_does_not_claim_physical_foreground(
    tmp_path: Path,
) -> None:
    config = _config(tmp_path, input_method="TargetedMessages")
    _runtime(config.run_dir)
    command = build_locomotion_probe_command(config)

    assert command[command.index("-InputMethod") + 1] == "TargetedMessages"
    assert command.count("-AllowTargetedLocomotionDiagnostic") == 1
    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                _passing_probe(
                    config.run_dir,
                    process_id=config.unreal_pid,
                    input_method="TargetedMessages",
                )
            ),
            stderr="",
        ),
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert result.passed is True
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["input_method"] == "TargetedMessages"
    assert summary["physical_foreground_input"] is False


def test_gate_waits_for_current_runtime_and_publishes_strict_pass(
    tmp_path: Path,
) -> None:
    config = _config(tmp_path)
    _runtime(config.run_dir)
    calls: list[tuple[str, ...]] = []

    def command_runner(command: tuple[str, ...], **kwargs: Any) -> SimpleNamespace:
        calls.append(command)
        assert kwargs["timeout"] > 0.0
        assert kwargs["encoding"] == "utf-8"
        assert kwargs["errors"] == "strict"
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps(_passing_probe(config.run_dir, process_id=config.unreal_pid)),
            stderr="",
        )

    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=command_runner,
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert len(calls) == 1
    assert result.passed is True
    assert result.probe_invocation_count == 1
    assert result.probe_path == config.run_dir / "runtime-locomotion-probe.json"
    probe = json.loads(result.probe_path.read_text(encoding="utf-8"))
    assert probe["passed"] is True
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["schema"] == "lingtu.sim.manual-locomotion-gate.v1"
    assert summary["passed"] is True
    assert summary["probe_invocation_count"] == 1


@pytest.mark.skipif(os.name != "nt", reason="PowerShell console encoding is Windows-specific")
def test_drive_probe_output_is_explicit_utf8_for_chinese_titles() -> None:
    script = Path(__file__).resolve().parents[2] / "sim/tools/diagnostics/probe_drive_input.ps1"
    setup = script.read_text(encoding="utf-8").split("Add-Type -AssemblyName", 1)[0]
    result = subprocess.run(
        ["powershell", "-NoLogo", "-NoProfile", "-NonInteractive", "-Command",
         setup + '\n[Console]::WriteLine(\'{"title":"机器狗 仿真"}\')'],
        capture_output=True, check=True,
    )
    assert json.loads(result.stdout.decode("utf-8"))["title"] == "机器狗 仿真"


@pytest.mark.skipif(os.name != "nt", reason="Windows PowerShell scalar counting regression")
def test_single_successful_send_counts_as_one_origin(tmp_path: Path) -> None:
    script = Path(__file__).resolve().parents[2] / "sim/tools/diagnostics/probe_drive_input.ps1"
    source = script.read_text(encoding="utf-8")
    functions = source.split('$script:LingTuDriveProbeStage = "contract"', 1)[0]
    correlate = source.split("$acceptedEvidence = @()", 1)[1].split("$nonzeroAccepted =", 1)[0]
    setup = r'''
    $newAccepted = @([pscustomobject]@{datagram_sha256="abc";time_ns=1;
        admitted_twist=[pscustomobject]@{linear_x=.6;linear_y=0;angular_z=0}})
    $authorityByHash = @{}
    $originByHash = @{abc=@([pscustomobject]@{successful_send=$true;datagram_sha256="abc"})}
    $acceptedEvidence = @()
    '''
    harness = tmp_path / "origin-count.ps1"
    harness.write_text(functions + setup + correlate + '\n$acceptedEvidence | ConvertTo-Json -Depth 4', encoding="utf-8-sig")
    completed = subprocess.run(
        ["powershell", "-NoLogo", "-NoProfile", "-NonInteractive", "-File", str(harness)],
        capture_output=True, text=True, encoding="utf-8", check=True,
    )
    assert json.loads(completed.stdout)["origin_count"] == 1


@pytest.mark.skipif(os.name != "nt", reason="Real Windows PowerShell turn-probe contract")
@pytest.mark.parametrize(("key", "yaw", "expected"),
                         [("Q", .1, False), ("Q", 1.2, True), ("E", -1.2, True),
                          ("Q", -1.2, False), ("E", 1.2, False)])
def test_turn_probe_requires_signed_yaw_even_with_translation(tmp_path: Path, key, yaw, expected):
    source = (Path(__file__).resolve().parents[2] / "sim/tools/diagnostics/probe_drive_input.ps1").read_text(encoding="utf-8")
    functions = source.split('$script:LingTuDriveProbeStage = "contract"', 1)[0]
    snapshots = [
        {"sim_time_ns": t, "bodies": [{"stable_id": "thunder_01/base_link",
          "position_m": [x, 0, .5], "quaternion_wxyz": [math.cos(a / 2), 0, 0, math.sin(a / 2)]}]}
        for t, x, a in [(100, 0, 0), (2_000_000_100, 1.0, yaw)]
    ]
    body = f"\n$snapshots = '{json.dumps(snapshots)}' | ConvertFrom-Json\n" + f'''
    Get-SnapshotQualification -Documents $snapshots -StableId "thunder_01/base_link" `
      -HoldStartTimeNs 100 -HoldEndTimeNs 2000000100 -MinimumPositionDisplacementMeters .02 `
      -MinimumYawDeltaRadians .8 -RequirePositionDisplacement $false -DirectionKey {key} | ConvertTo-Json
    '''
    harness = tmp_path / "turn-probe.ps1"
    harness.write_text(functions + body, encoding="utf-8-sig")
    result = subprocess.run(["powershell", "-NoLogo", "-NoProfile", "-NonInteractive", "-File", str(harness)],
                            capture_output=True, text=True, encoding="utf-8", check=True)
    result = json.loads(result.stdout)
    assert result["yaw_delta_rad"] == pytest.approx(yaw)
    assert result["passed"] is expected


def test_gate_preserves_failed_probe_without_claiming_locomotion(
    tmp_path: Path,
) -> None:
    config = _config(tmp_path)
    _runtime(config.run_dir)
    probe = _passing_probe(config.run_dir, process_id=config.unreal_pid)
    probe["locomotion_qualification"] = {"state": "failed", "passed": False}
    probe["passed"] = False

    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=1,
            stdout=json.dumps(probe),
            stderr="locomotion gate failed",
        ),
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert result.passed is False
    assert result.probe_invocation_count == 1
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["passed"] is False
    assert summary["failure_reason"] == "locomotion probe did not pass all physical gates"
    assert summary["probe_stderr"] == "locomotion gate failed"
    assert summary["probe"]["locomotion_qualification"]["passed"] is False


def test_atomic_invocation_marker_prevents_second_input_attempt(tmp_path: Path) -> None:
    config = _config(tmp_path)
    _runtime(config.run_dir)
    calls = 0

    def command_runner(*_args: Any, **_kwargs: Any) -> SimpleNamespace:
        nonlocal calls
        calls += 1
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps(_passing_probe(config.run_dir, process_id=config.unreal_pid)),
            stderr="",
        )

    first = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=command_runner,
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert first.passed is True
    with pytest.raises(RuntimeError, match="already reserved"):
        supervise_manual_locomotion_gate(
            config,
            prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
            command_runner=command_runner,
            monotonic=lambda: 10.0,
            sleep=lambda _seconds: None,
            release_input=lambda: None,
        )
    assert calls == 1


def test_timeout_still_runs_parent_key_release_cleanup(tmp_path: Path) -> None:
    config = _config(tmp_path)
    _runtime(config.run_dir)
    releases: list[str] = []

    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            subprocess.TimeoutExpired("probe", 10.0)
        ),
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: releases.append("released"),
    )

    assert result.passed is False
    assert releases == ["released"]


def test_bare_pass_flags_cannot_forge_physical_locomotion(tmp_path: Path) -> None:
    config = _config(tmp_path)
    _runtime(config.run_dir)
    forged = _passing_probe(config.run_dir, process_id=config.unreal_pid)
    forged["locomotion_qualification"] = {"state": "passed", "passed": True}

    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=json.dumps(forged),
            stderr="",
        ),
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert result.passed is False


@pytest.mark.parametrize(
    ("field", "value"),
    (("direction_key", "Q"), ("unexpected_nonzero_commands", 1)),
)
def test_gate_rejects_wrong_or_mixed_direction_input(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    config = _config(tmp_path)
    _runtime(config.run_dir)
    probe = _passing_probe(config.run_dir, process_id=config.unreal_pid)
    probe[field] = value

    result = supervise_manual_locomotion_gate(
        config,
        prove_presence=lambda pid: SimpleNamespace(owned_unreal_pid=pid),
        command_runner=lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0,
            stdout=json.dumps(probe),
            stderr="",
        ),
        monotonic=lambda: 10.0,
        sleep=lambda _seconds: None,
        release_input=lambda: None,
    )

    assert result.passed is False
