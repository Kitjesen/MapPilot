# ruff: noqa: S101

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "sim"
PLUGIN = (
    ROOT
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
)
UI = PLUGIN / "Source" / "LingTuSimUI"


def _extract_cpp_method_body(source: str, qualified_name: str) -> str:
    definition = re.search(
        rf"{re.escape(qualified_name)}\s*\([^;{{}}]*\)\s*(?:const\s*)?\{{",
        source,
        flags=re.DOTALL,
    )
    assert definition is not None, f"missing C++ method definition: {qualified_name}"

    opening_brace = definition.end() - 1
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1 : index]
    raise AssertionError(f"unterminated C++ method definition: {qualified_name}")


def test_plugin_declares_asset_free_runtime_ui_module() -> None:
    descriptor = json.loads((PLUGIN / "LingTuSim.uplugin").read_text(encoding="utf-8"))

    modules = {module["Name"]: module for module in descriptor["Modules"]}
    assert modules["LingTuSimUI"] == {
        "Name": "LingTuSimUI",
        "Type": "Runtime",
        "LoadingPhase": "Default",
    }
    assert descriptor["CanContainContent"] is False
    assert (UI / "LingTuSimUI.Build.cs").is_file()


def test_runtime_ui_enable_policy_is_explicit_and_automation_covered() -> None:
    policy_header = (UI / "Public" / "LingTuSimRuntimeUIPolicy.h").read_text(
        encoding="utf-8"
    )
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")

    assert "ShouldEnable" in policy_header
    assert "LingTuRuntimeUI" in policy_header
    assert "LingTuDisableRuntimeUI" in policy_header
    for behavior in (
        "InteractiveGameEnablesByDefault",
        "UnattendedDisablesByDefault",
        "ForceFlagOverridesUnattended",
        "DisableFlagWins",
    ):
        assert behavior in automation


def test_runtime_ui_modes_cover_drive_build_tactical_and_pause() -> None:
    model_header = (UI / "Public" / "LingTuSimRuntimeUIModel.h").read_text(
        encoding="utf-8"
    )
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")

    for mode in ("Drive", "Build", "Tactical", "Pause"):
        assert mode in model_header
    for operation in ("ToggleBuild", "ToggleTactical", "TogglePause"):
        assert operation in model_header
    for behavior in (
        "BuildKeyTogglesBuildMode",
        "TabKeyTogglesTacticalMode",
        "EscapeRestoresPreviousMode",
    ):
        assert behavior in automation


def test_runtime_ui_reads_session_and_visual_status_without_taking_mailboxes() -> None:
    status_header = (UI / "Public" / "LingTuSimRuntimeUIStatus.h").read_text(
        encoding="utf-8"
    )
    status_source = (
        UI / "Private" / "LingTuSimRuntimeUIStatus.cpp"
    ).read_text(encoding="utf-8")
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")
    production = "\n".join(
        path.read_text(encoding="utf-8")
        for path in UI.rglob("*")
        if path.is_file() and "Tests" not in path.parts
    )

    assert "Unavailable" in status_header
    assert "GetBoundSession" in status_source
    assert "GetRegisteredBindingCount" in status_source
    assert "GetScenarioActorCount" in status_source
    assert "IsWaitingForRebind" in status_source
    assert "TryTakeLatest" not in production
    assert "PublishSnapshot" not in production
    assert "SubmitSnapshot" not in production
    for forbidden_mutation in (
        "SpawnActor<",
        "DestroyActor(",
        "SetActorTransform",
        "SetWorldTransform",
        "SetSimulatePhysics",
        "SetGamePaused",
    ):
        assert forbidden_mutation not in production
    assert "NoSessionDisplaysUnavailable" in automation
    assert "BoundSessionReadLeavesIngressOwnedByVisual" in automation


def test_runtime_ui_mounts_asset_free_slate_hud_and_routes_game_keys() -> None:
    build_rules = (UI / "LingTuSimUI.Build.cs").read_text(encoding="utf-8")
    subsystem = (
        UI / "Private" / "LingTuSimRuntimeUIWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    hud = (UI / "Private" / "SLingTuSimRuntimeHUD.cpp").read_text(encoding="utf-8")
    input_header = (
        UI / "Private" / "LingTuSimRuntimeUIInputProcessor.h"
    ).read_text(encoding="utf-8")
    model_source = (
        UI / "Private" / "LingTuSimRuntimeUIModel.cpp"
    ).read_text(encoding="utf-8")
    module_source = (UI / "Private" / "LingTuSimUIModule.cpp").read_text(
        encoding="utf-8"
    )

    for dependency in (
        "CoreUObject",
        "Engine",
        "InputCore",
        "Slate",
        "SlateCore",
        "LingTuSimRuntime",
        "LingTuSimSession",
        "LingTuSimVisual",
    ):
        assert f'"{dependency}"' in build_rules
    assert "IMPLEMENT_MODULE" in module_source
    assert "LingTuSimUI" in module_source
    assert "AddViewportWidgetContent" in subsystem
    assert "RemoveViewportWidgetContent" in subsystem
    assert "RegisterInputPreProcessor" in subsystem
    assert "UnregisterInputPreProcessor" in subsystem
    assert "FRuntimeUIStatusReader::Read" in hud
    assert '#include "Widgets/SOverlay.h"' in hud
    assert 'Widgets/Layout/SOverlay.h' not in hud
    assert "virtual void Tick" in input_header
    assert 'TEXT("LINGTU")' in hud
    assert "READ-ONLY PREVIEW" in hud
    assert "EKeys::B" in model_source
    assert "EKeys::Tab" in model_source
    assert "EKeys::Escape" in model_source


def test_runtime_ui_drive_input_is_pure_safe_and_transport_agnostic() -> None:
    drive_header = UI / "Public" / "LingTuSimRobotDriveInput.h"
    drive_source = UI / "Private" / "LingTuSimRobotDriveInput.cpp"
    input_header = UI / "Private" / "LingTuSimRuntimeUIInputProcessor.h"
    input_source = UI / "Private" / "LingTuSimRuntimeUIInputProcessor.cpp"
    subsystem = UI / "Private" / "LingTuSimRuntimeUIWorldSubsystem.cpp"

    assert drive_header.is_file()
    assert drive_source.is_file()

    drive_contract = drive_header.read_text(encoding="utf-8")
    drive_implementation = drive_source.read_text(encoding="utf-8")
    processor_contract = input_header.read_text(encoding="utf-8")
    processor_implementation = input_source.read_text(encoding="utf-8")
    subsystem_implementation = subsystem.read_text(encoding="utf-8")
    input_production = "\n".join(
        (
            drive_contract,
            drive_implementation,
            processor_contract,
            processor_implementation,
        )
    )

    for symbol in (
        "FRobotDriveInputState",
        "FRobotDriveInputSnapshot",
        "SetDriveMode",
        "SetViewportFocused",
        "HandleKeyDown",
        "HandleKeyUp",
        "HandleAnalog",
        "ReleaseAll",
        "CameraYaw",
        "CameraPitch",
    ):
        assert symbol in drive_contract

    for key in (
        "EKeys::W",
        "EKeys::S",
        "EKeys::A",
        "EKeys::D",
        "EKeys::Q",
        "EKeys::E",
        "EKeys::LeftShift",
        "EKeys::Gamepad_LeftShoulder",
        "EKeys::Gamepad_LeftX",
        "EKeys::Gamepad_LeftY",
        "EKeys::Gamepad_LeftTriggerAxis",
        "EKeys::Gamepad_RightTriggerAxis",
        "EKeys::Gamepad_RightX",
        "EKeys::Gamepad_RightY",
    ):
        assert key in drive_implementation

    assert "HandleKeyUpEvent" in processor_contract
    assert "HandleAnalogInputEvent" in processor_contract
    assert "IsRepeat" in processor_implementation
    assert "IsActive" in processor_implementation
    assert "HasUserFocusedDescendants" in processor_implementation
    assert "!Snapshot.bDeadman" in processor_implementation
    assert "PublishOperatorIntent" in subsystem_implementation
    assert "GetNumLocalPlayers" in subsystem_implementation
    assert "multiple_eligible_game_worlds" in subsystem_implementation

    for forbidden_transport_detail in (
        "FSocket",
        "FUdpSocket",
        "CreateInternetAddr",
        "Serialize",
        "FJson",
        "Sha256",
        "OriginEvidence",
        "FLingTuSimOperatorIntentSender",
        "BindControlTransport",
    ):
        assert forbidden_transport_detail not in input_production


def test_runtime_ui_drive_input_automation_covers_safety_lifecycle() -> None:
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")

    for behavior in (
        "KeyboardMapsAndCancelsOpposites",
        "KeyboardRepeatIsIgnored",
        "GamepadDeadZoneClampAndCameraAxesStaySeparate",
        "MotionRequiresDriveDeadmanAndForeground",
        "ReleasePathsPublishZero",
        "MultipleEligibleViewportContextsFailClosed",
    ):
        assert behavior in automation


def test_runtime_hud_uses_light_gameplay_tokens_and_truthful_labels() -> None:
    status_header = (UI / "Public" / "LingTuSimRuntimeUIStatus.h").read_text(
        encoding="utf-8"
    )
    status_source = (UI / "Private" / "LingTuSimRuntimeUIStatus.cpp").read_text(
        encoding="utf-8"
    )
    hud = (UI / "Private" / "SLingTuSimRuntimeHUD.cpp").read_text(
        encoding="utf-8"
    )

    for light_token in (
        "WarmWhiteGlass",
        "WarmWhitePanel",
        "WarmWhiteSoft",
        "TealAccent",
    ):
        assert light_token in hud
    assert "TopBar" not in hud
    assert "0.018F, 0.035F, 0.047F" not in hud

    for status_field in (
        "RunId",
        "ResetGeneration",
        "ControlAckState",
        "RequestedInput",
        "ObservedBaseLinearVelocityMps",
        "ObservedBaseAngularVelocityRadps",
    ):
        assert status_field in status_header
    for read_only_source in (
        "GetBoundSession",
        "GetControlTransportBinding",
        "GetLatestControlAck",
        "GetLatestAppliedSnapshot",
    ):
        assert read_only_source in status_source

    for authoritative_label in (
        "ADMITTED",
        "OBSERVED",
        "PHYSICS %s",
        "SENSORS %s",
        "CONTROL STATUS",
        "RECORDING",
        "CAMERA  UE",
        "RUNTIME ECHO",
    ):
        assert authoritative_label in hud
    assert "ACK v1" not in hud
    assert "Authoritative pause state is unavailable" not in hud
    assert "X  END SESSION" in hud
    assert "SIMULATION CONTINUES" not in hud
    assert "simulation continues" not in hud.lower()
    assert "R STOP + COMMIT" in hud


def test_runtime_hud_screenshot_is_exact_three_mode_gated_and_includes_ui() -> None:
    subsystem = (
        UI / "Private" / "LingTuSimRuntimeUIWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    contract = (
        UI / "Private" / "LingTuSimHudScreenshotContract.cpp"
    ).read_text(encoding="utf-8")
    automation_test = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")
    coordinator = (
        ROOT / "runtime" / "coordinator" / "unreal_process.py"
    ).read_text(encoding="utf-8")

    assert "LingTuHudScreenshot=" not in subsystem
    assert "FHudScreenshotContract::ParseCommandLine" in subsystem
    assert "TrimQuotes" not in contract
    assert "DecodeAssignmentValue" in contract
    assert "bHasLeadingQuote != bHasTrailingQuote" in contract
    assert 'OutValue.Contains(TEXT("\\\""))' in contract
    screenshot_test = automation_test[
        automation_test.index(
            "FLingTuRuntimeUIHudScreenshotArgumentsAreExactTripleTest::RunTest"
        ) : automation_test.index(
            "FLingTuRuntimeUIHudScreenshotPathValidationRejectsForeignAndNoncanonicalTargetsTest"
        )
    ]
    assert screenshot_test.index(
        "if (!bParsed || !bConfigured || Targets.Num() != 3)"
    ) < screenshot_test.index("Targets[0]")
    for strict_quote_case in (
        "an unmatched leading quote is rejected",
        "an unmatched trailing quote is rejected",
        "quoted and unquoted duplicate assignments are rejected",
        "the valid quoted screenshot triple parses",
    ):
        assert strict_quote_case in screenshot_test
    assert subsystem.count("FScreenshotRequest::RequestScreenshot(") == 1
    for argument, filename in (
        ("LingTuHudDriveScreenshot", "hud-drive.png"),
        ("LingTuHudTacticalScreenshot", "hud-tactical.png"),
        ("LingTuHudMenuRecordingScreenshot", "hud-menu-recording.png"),
    ):
        assert argument in contract
        assert argument in coordinator
        assert filename in contract
        assert filename in coordinator
    assert "Status.IsDriveCaptureReady()" in contract
    assert "Status.IsTacticalCaptureReady()" in contract
    assert "Status.IsMenuRecordingCaptureReady()" in contract
    assert "FHudScreenshotContract::IsNextCaptureMode" in subsystem
    assert "NextHudScreenshotIndex" in subsystem
    assert "++NextHudScreenshotIndex" in subsystem
    assert "ActiveHudScreenshotIndex != NextHudScreenshotIndex" in subsystem
    assert "Drive -> Tactical -> Menu-recording" in (
        UI / "Public" / "LingTuSimHudScreenshotContract.h"
    ).read_text(encoding="utf-8")
    assert "GetSizeXY() != FIntPoint(1920, 1080)" in subsystem
    assert "ReadPngDimensions" in subsystem
    assert "ScreenshotWidth != 1920" in subsystem
    assert "ScreenshotHeight != 1080" in subsystem
    assert "ScreenshotWritten" in subsystem
    assert "EvidenceCommitted" in subsystem
    assert "WriteHudScreenshotEvidence" in subsystem
    assert subsystem.index("ScreenshotBytes <= 0") < subsystem.index(
        "WriteHudScreenshotEvidence(Capture"
    )
    assert 'SetBoolField(TEXT("show_ui"), true)' in contract
    assert 'SetBoolField(TEXT("qualification_ready"), true)' in contract
    assert "FJsonObject" in contract
    assert "FJsonSerializer::Serialize" in contract
    assert (
        "CapturedMonotonicNs - FullStatus.ReceivedMonotonicNs" in contract
    )
    assert 'SetExactUint64(Root, TEXT("status_age_ns"), ExactStatusAgeNs)' in contract
    assert 'FString::Printf(\n      TEXT("{\\n"' not in subsystem
    for actual_capture_field in (
        'TEXT("control_status")',
        'TEXT("event_id")',
        'TEXT("source_sequence")',
        'TEXT("requested_axes")',
        'TEXT("admitted_twist_mps_radps")',
        'TEXT("observed_base_velocity_mps_radps")',
        'TEXT("readiness")',
        'TEXT("sensors")',
        'TEXT("recording")',
        'TEXT("camera_mode")',
    ):
        assert actual_capture_field in contract

    status_header = (UI / "Public" / "LingTuSimRuntimeUIStatus.h").read_text(
        encoding="utf-8"
    )
    for required_gate in (
        "bInputObserved",
        "FullStatus.Motion.RequestedAxes.bAvailable",
        "IsFullStatusCaptureReady",
    ):
        assert required_gate in status_header.split("bool IsDriveCaptureReady() const", 1)[1]
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")
    assert "HudCaptureRequiresInputAckAndObservedTruth" in automation
    assert "menu cannot capture before drive" in automation
    assert "menu cannot skip tactical after drive" in automation
    assert "status age is measured at the capture instant" in automation
    assert "captured_time_precedes_status_receive" in automation


def test_runtime_hud_capture_only_qualifies_accepted_or_confirmed_ack() -> None:
    status_header = (UI / "Public" / "LingTuSimRuntimeUIStatus.h").read_text(
        encoding="utf-8"
    )
    status_source = (UI / "Private" / "LingTuSimRuntimeUIStatus.cpp").read_text(
        encoding="utf-8"
    )
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")

    gate = status_source.split("bool FRuntimeUIStatusSnapshot::IsFullStatusCaptureReady", 1)[1]
    assert "EControlAckStatus ControlAckStatus" in status_header
    assert "bControlAckAccepted" in status_header
    assert "FullStatus.Status" in gate
    assert "EControlAckStatus::Accepted" in gate
    assert "EControlAckStatus::Confirmed" in gate
    assert "HudCaptureRejectsNonAcceptedAckStates" in automation
    for nonqualifying_status in (
        "EControlAckStatus::Pending",
        "EControlAckStatus::Rejected",
        "EControlAckStatus::Released",
        "EControlAckStatus::TimeoutZero",
    ):
        assert nonqualifying_status in automation
    for qualifying_status in (
        "EControlAckStatus::Accepted",
        "EControlAckStatus::Confirmed",
    ):
        assert qualifying_status in automation


def test_runtime_hud_screenshot_paths_reject_reparse_traversal() -> None:
    contract = (
        UI / "Private" / "LingTuSimHudScreenshotContract.cpp"
    ).read_text(encoding="utf-8")
    subsystem = (
        UI / "Private" / "LingTuSimRuntimeUIWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")

    for security_primitive in (
        ".IsSymlink",
        "FILE_FLAG_OPEN_REPARSE_POINT",
        "FILE_ATTRIBUTE_REPARSE_POINT",
        "GetFinalPathNameByHandleW",
        "ValidateHudScreenshotTarget",
        "reparse_path_component",
        "final_path_mismatch",
    ):
        assert security_primitive in contract
    assert subsystem.count("ValidateHudScreenshotTarget") >= 3
    assert "ValidateEvidenceTempTarget" in contract
    assert "GetFileAttributesW" in contract
    assert "CreateFileW" in contract
    assert "GetFileInformationByHandle" in contract
    assert "CREATE_NEW" in contract
    assert "PathToInspect += TEXT(\"/\")" in contract
    assert contract.index("const bool bIsDrive = FPaths::IsDrive(Current)") < contract.index(
        "ValidateExistingPlainPath(PathToInspect"
    )
    assert "WriteEvidenceTempFileNoFollow" in subsystem
    assert "HudScreenshotPathValidationRejectsForeignAndNoncanonicalTargets" in automation
    assert "FGuid::NewGuid()" in automation
    assert "absent run-owned target validation failed with error" in automation
    assert "dangling screenshot target is rejected without following it" in automation
    assert "evidence temp is created exclusively through the no-follow writer" in automation


def test_runtime_actions_camera_recording_and_exit_are_authority_gated() -> None:
    model = (UI / "Private" / "LingTuSimRuntimeUIModel.cpp").read_text(
        encoding="utf-8"
    )
    processor = (
        UI / "Private" / "LingTuSimRuntimeUIInputProcessor.cpp"
    ).read_text(encoding="utf-8")
    world = (
        UI / "Private" / "LingTuSimRuntimeUIWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    status = (UI / "Private" / "LingTuSimRuntimeUIStatus.cpp").read_text(
        encoding="utf-8"
    )
    automation = (
        UI / "Private" / "Tests" / "LingTuSimRuntimeUITest.cpp"
    ).read_text(encoding="utf-8")

    for key in ("EKeys::C", "EKeys::R", "EKeys::X"):
        assert key in model
    assert processor.index("if (!DriveInput.GetSnapshot().bViewportFocused)") < processor.index(
        "FRuntimeUIActionPolicy::ResolveKey"
    )
    assert "CycleRuntimeCameraMode" in world
    assert "CanRequestRecordStart" in world
    assert "RecordStopCommit" in world
    assert "EOperatorRuntimeRequestType::Exit" in world
    assert "FPlatformMisc::RequestExit(false)" in world
    for exit_gate in (
        "Full.EventId != PendingExitEventId",
        "Full.EventId != CorrelatedEventId",
        "Full.SourceEpoch == 0",
        "Full.SourceSequence == 0",
        "IsLowerHexSha256(Full.IntentDatagramSha256)",
        "EControlAckStatus::Confirmed",
        "EControlStatusRuntimeState::Stopped",
        "EControlSafeStopState::Zeroed",
        "Admitted.LinearX != 0.0",
        "bEngineExitAlreadyRequested",
    ):
        assert exit_gate in status
    assert "ExitRequiresExactStoppedZeroConfirmation" in automation
    assert "invalid successful-send digest cannot close UE" in automation
    assert "missing source sequence cannot close UE" in automation


def test_runtime_camera_modes_bind_real_view_targets_and_read_back() -> None:
    visual = PLUGIN / "Source" / "LingTuSimVisual"
    header = (visual / "Public" / "LingTuSimVisualWorldSubsystem.h").read_text(
        encoding="utf-8"
    )
    source = (
        visual / "Private" / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    automation = (
        visual / "Private" / "Tests" / "LingTuSimVisualRuntimeTest.cpp"
    ).read_text(encoding="utf-8")

    for mode in ("Follow", "Inspection", "Free"):
        assert mode in header
    assert "UpdateMotionCameraTransform" in source
    assert "FindUniqueInspectionCamera" in source
    assert "inspection_camera_ambiguous" in source
    assert "EnsureFreeCamera" in source
    assert "SetViewTarget(Candidate)" in source
    assert "GetViewTarget() != Candidate" in source
    assert "free_camera_view_target_not_confirmed" in source
    assert "CameraModesUseConfirmedViewTargets" in automation
    assert "ambiguous inspection tag fails closed" in automation
    update_motion_camera = _extract_cpp_method_body(
        source,
        "ULingTuSimVisualWorldSubsystem::UpdateMotionCameraTransform",
    )
    assert "CameraYaw" not in update_motion_camera


def test_visual_world_rebinds_session_camera_on_next_tick_after_begin_play() -> None:
    source = (
        PLUGIN
        / "Source"
        / "LingTuSimVisual"
        / "Private"
        / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    begin_play = _extract_cpp_method_body(
        source,
        "ULingTuSimVisualWorldSubsystem::OnWorldBeginPlay",
    )
    deferred_rebind = _extract_cpp_method_body(
        source,
        "ULingTuSimVisualWorldSubsystem::"
        "ReassertSessionCameraViewTargetAfterActorBeginPlay",
    )

    first_bind = begin_play.index("SetSessionCameraViewTargetForPlayer0();")
    next_tick = begin_play.index("InWorld.GetTimerManager().SetTimerForNextTick")

    assert first_bind < next_tick
    assert (
        "&ULingTuSimVisualWorldSubsystem::"
        "ReassertSessionCameraViewTargetAfterActorBeginPlay"
    ) in begin_play[next_tick:]
    assert "SetSessionCameraViewTargetForPlayer0();" in deferred_rebind


def test_visual_latest_applied_snapshot_accessor_is_read_only_and_tested() -> None:
    visual = PLUGIN / "Source" / "LingTuSimVisual"
    header = (visual / "Public" / "LingTuSimVisualWorldSubsystem.h").read_text(
        encoding="utf-8"
    )
    source = (
        visual / "Private" / "LingTuSimVisualWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    automation = (
        visual / "Private" / "Tests" / "LingTuSimVisualRuntimeTest.cpp"
    ).read_text(encoding="utf-8")

    assert "bool GetLatestAppliedSnapshot" in header
    assert "OutSnapshot = LatestAppliedSnapshot" in source
    assert source.index("SnapshotGate.Commit(Snapshot);") < source.index(
        "LatestAppliedSnapshot = Snapshot;"
    )
    assert "Rebind clears the prior generation's latest-applied snapshot" in automation


def test_visual_runtime_stateful_automation_uses_real_world_and_guarded_evidence_fixture() -> None:
    automation = (
        PLUGIN
        / "Source"
        / "LingTuSimVisual"
        / "Private"
        / "Tests"
        / "LingTuSimVisualRuntimeTest.cpp"
    ).read_text(encoding="utf-8")

    udp_future = automation.split(
        "bool FLingTuSimVisualUdpFutureGenerationTest::RunTest", 1
    )[1].split("IMPLEMENT_SIMPLE_AUTOMATION_TEST", 1)[0]
    atomic = automation.split(
        "bool FLingTuSimVisualAtomicRebindTest::RunTest", 1
    )[1].split("IMPLEMENT_SIMPLE_AUTOMATION_TEST", 1)[0]
    readiness = automation.split(
        "bool FLingTuSimVisualReadinessEvidenceTransitionTest::RunTest", 1
    )[1].split("IMPLEMENT_SIMPLE_AUTOMATION_TEST", 1)[0]
    stale_future = automation.split(
        "bool FLingTuSimVisualReadinessRejectsStaleAndFutureTest::RunTest", 1
    )[1].split("IMPLEMENT_SIMPLE_AUTOMATION_TEST", 1)[0]

    for body in (udp_future, atomic, readiness, stale_future):
        assert "FScopedVisualTestWorld TestWorld" in body
        assert "TestWorld.GetVisualSubsystem()" in body
        assert "NewObject<ULingTuSimVisualWorldSubsystem>()" not in body

    assert automation.count("NewObject<ULingTuSimVisualWorldSubsystem>()") == 1
    assert ": World(CreateVisualTestWorld())" in automation
    assert "World->GetSubsystem<ULingTuSimVisualWorldSubsystem>()" in automation
    assert "FPaths::ConvertRelativePathToFull" in automation
    assert 'FPaths::Combine(RunDirectory, TEXT("logs"))' in automation
    assert "latest-applied entity count is exact" in readiness
    entity_count_guard = readiness.index("LatestApplied.Entities.Num() != 1")
    entity_index = readiness.index("LatestApplied.Entities[0]")
    assert entity_count_guard < entity_index
