# ruff: noqa: S101
from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION_SOURCE = (
    REPO_ROOT
    / "sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Source/LingTuSimSession"
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_session_service_exposes_only_narrow_ui_control_transport_surface() -> None:
    header = _read(SESSION_SOURCE / "Public/LingTuSimSessionService.h")

    assert "static bool PublishOperatorIntent(" in header
    assert "static bool PublishRuntimeRequest(" in header
    assert "static bool GetLatestControlAck(" in header
    assert "static bool PublishControlResponseJson(" in header
    assert "static bool GetLatestControlStatus(" in header
    assert "static bool GetControlTransportBinding(" in header


def test_control_transport_is_strictly_run_allocation_and_loopback_bound() -> None:
    source = _read(SESSION_SOURCE / "Private/LingTuSimControlTransportRuntime.cpp")

    assert 'TEXT("LingTuControlIntentPort")' in source
    assert 'TEXT("LingTuControlStatusPort")' in source
    assert "intent/status ports must appear exactly once together" in source
    assert 'TEXT("LingTuRunAllocation")' in source
    assert "ParseRunAllocationJson(" in source
    assert "FIPv4Address::InternalLoopback" in source
    assert "Rejected non-loopback control ACK datagram" in source
    assert "control intent and status ports must be distinct" in source
    assert "symlinks or reparse points" in source


def test_origin_evidence_follows_complete_send_and_never_overwrites() -> None:
    protocol = _read(SESSION_SOURCE / "Private/LingTuSimControlTransport.cpp")
    runtime = _read(SESSION_SOURCE / "Private/LingTuSimControlTransportRuntime.cpp")
    send_and_record = protocol[protocol.index("SendAndRecord(") :]

    send_index = send_and_record.index("SendDatagram(Datagram, OutError)")
    hash_index = send_and_record.index("Sha256Hex(")
    remember_index = send_and_record.index("RememberSuccessfulSend(")
    append_index = send_and_record.index("AppendOriginEvidence(EvidenceJson, OutError)")
    assert send_index < hash_index < append_index < remember_index
    assert 'TEXT("datagram_sha256")' in protocol
    assert 'TEXT("successful_send")' in protocol
    automation = _read(
        SESSION_SOURCE / "Private" / "Tests" / "LingTuSimControlTransportTest.cpp"
    )
    assert "durable origin evidence failure rejects publication" in automation
    assert "ACK is rejected when durable origin evidence append failed" in automation
    assert "status is rejected when durable origin evidence append failed" in automation
    assert "CalcSHA256" in protocol
    assert "FILEWRITE_Append" in runtime
    assert "SaveStringToFile" in runtime


def test_ack_is_correlated_to_a_bounded_ledger_of_successful_sends() -> None:
    header = _read(SESSION_SOURCE / "Public/LingTuSimControlTransport.h")
    protocol = _read(SESSION_SOURCE / "Private/LingTuSimControlTransport.cpp")
    service = _read(SESSION_SOURCE / "Private/LingTuSimSessionService.cpp")

    assert "ValidateAckAgainstSuccessfulSend(" in header
    assert "ValidateStatusAgainstSuccessfulSend(" in header
    assert "MaxTrackedSuccessfulSends" in protocol
    assert "RememberSuccessfulSend(" in protocol
    assert "successful control send" in protocol
    assert "ValidateAckAgainstSuccessfulSend(Ack" in service
    assert "ValidateStatusAgainstSuccessfulSend(Status" in service


def test_every_control_only_argument_prevents_silent_viewer_fallback() -> None:
    header = _read(SESSION_SOURCE / "Public/LingTuSimControlTransport.h")
    runtime = _read(SESSION_SOURCE / "Private/LingTuSimControlTransportRuntime.cpp")

    assert "ValidateControlCommandLineShape(" in header
    assert 'TEXT("LingTuControlSourceId")' in runtime
    assert 'TEXT("LingTuHudDriveScreenshot")' in runtime
    assert 'TEXT("LingTuHudTacticalScreenshot")' in runtime
    assert 'TEXT("LingTuHudMenuRecordingScreenshot")' in runtime
    assert "legacy singular HUD screenshot argument is unsupported" in runtime
    assert "ControlOnlyAssignments" in runtime


def test_exact_field_scanner_tracks_strings_at_every_depth() -> None:
    protocol = _read(SESSION_SOURCE / "Private/LingTuSimControlTransport.cpp")

    assert "bInString" in protocol
    assert "bEscaped" in protocol
    assert "StringDepth" in protocol


def test_minimal_ack_and_full_status_keep_independent_exact_schemas() -> None:
    protocol = _read(SESSION_SOURCE / "Private/LingTuSimControlTransport.cpp")
    full_status = _read(SESSION_SOURCE / "Private/LingTuSimControlStatus.cpp")

    assert 'TEXT("lingtu.sim.ue-control-ack.v1")' in protocol
    assert 'TEXT("intent_datagram_sha256")' in protocol
    assert 'TEXT("lingtu.sim.ue-control-status.v1")' not in protocol
    assert "must contain exactly the canonical top-level fields" in protocol
    assert 'TEXT("lingtu.sim.ue-control-status.v1")' in full_status
    assert "HasNoDuplicateObjectFields" in full_status
    assert "requested_axes" in full_status
    assert "admitted_twist_mps_radps" in full_status
    assert "observed_base_velocity_mps_radps" in full_status
    assert "ExpectedStreams" in full_status


def test_session_control_transport_never_writes_actor_or_mujoco_state() -> None:
    sources = "\n".join(
        _read(path)
        for path in SESSION_SOURCE.rglob("*Control*.cpp")
        if path.is_file()
    )

    assert "SetActorTransform" not in sources
    assert "SetActorLocation" not in sources
    assert "apply_actuator" not in sources
    assert "MuJoCo" not in sources


def test_control_runtime_pimpl_lifecycle_is_defined_out_of_line() -> None:
    header = _read(SESSION_SOURCE / "Private/LingTuSimControlTransportRuntime.h")
    source = _read(SESSION_SOURCE / "Private/LingTuSimControlTransportRuntime.cpp")

    assert "FLingTuSimControlTransportRuntime();" in header
    assert "FLingTuSimControlTransportRuntime() = default" not in header
    assert (
        "FLingTuSimControlTransportRuntime::FLingTuSimControlTransportRuntime() = default;"
        in source
    )
    assert "FLingTuSimControlTransportRuntime::~FLingTuSimControlTransportRuntime()" in source


def test_motion_wire_is_drive_only_even_when_ui_mode_changes() -> None:
    protocol = _read(SESSION_SOURCE / "Private/LingTuSimControlTransport.cpp")
    ui_adapter = _read(
        SESSION_SOURCE.parent / "LingTuSimUI/Private/LingTuSimRuntimeUIWorldSubsystem.cpp"
    )

    assert 'Sample.InputMode != TEXT("drive")' in protocol
    assert 'Sample.InputMode = TEXT("drive")' in ui_adapter
    assert "RuntimeUIModeName(CurrentMode)" not in ui_adapter


def test_intent_serialization_distinguishes_camera_mode_echo_from_camera_axes() -> None:
    automation = _read(
        SESSION_SOURCE / "Private" / "Tests" / "LingTuSimControlTransportTest.cpp"
    )

    assert '\\"camera_mode\\"' in automation
    assert 'HasField(TEXT("camera_yaw"))' in automation
    assert 'HasField(TEXT("camera_pitch"))' in automation
    assert 'Json.Contains(TEXT("camera"))' not in automation
