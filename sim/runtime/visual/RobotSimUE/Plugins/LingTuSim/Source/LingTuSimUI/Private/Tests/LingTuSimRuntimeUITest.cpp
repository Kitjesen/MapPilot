#if WITH_DEV_AUTOMATION_TESTS

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "LingTuSimBlueprintStatusSubsystem.h"
#include "LingTuSimFrontEndLogin.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "LingTuSimFrontEndScreenshotDriver.h"
#endif
#include "LingTuSimGameSelection.h"
#include "LingTuSimHudScreenshotContract.h"
#include "LingTuSimRobotDriveInput.h"
#include "LingTuSimRuntimeUIModel.h"
#include "LingTuSimRuntimeUIPolicy.h"
#include "LingTuSimRuntimeUIStatus.h"
#include "LingTuSimSessionService.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#if PLATFORM_WINDOWS
#include <Windows.h>

#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace {
TSharedPtr<FJsonObject> ParseJsonObjectForAssetReviewTest(const FString &Json) {
  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  FJsonSerializer::Deserialize(Reader, Root);
  return Root;
}

FString AssetReviewCardJson(const FString &Id, const int32 Order) {
  return FString::Printf(TEXT("{\"id\":\"%s\",\"title\":\"%s\",\"description\":\"Review %s\","),
                         *Id, *Id, *Id) +
         FString::Printf(TEXT("\"order\":%d,\"asset_class\":\"movable_object\","), Order) +
         TEXT(
             "\"review\":{\"stage\":\"unavailable\",\"disposition\":\"unavailable\","
             "\"reason\":\"not qualified\"},\"evidence\":[],")
             TEXT(
                 "\"render_policy\":{\"physics_authority\":\"mujoco\","
                 "\"purpose\":\"presentation_review_only\",\"qualified_visual\":false,"
                 "\"runnable\":false,\"unreal_collision_profile\":\"NoCollision\","
                 "\"unreal_simulate_physics\":false},")
                 TEXT(
                     "\"capabilities\":{\"selectable_for_review\":false,\"runnable\":false,"
                     "\"qualified_visual\":false},\"tags\":[]}");
}

FString AvailableAssetReviewSelectionJson(const FString &CardsJson) {
  return TEXT(
             "{\"asset_review\":{\"availability\":{\"state\":\"available\","
             "\"reason\":\"validated asset review catalog\"},\"catalog\":{")
             TEXT(
                 "\"schema\":\"lingtu.sim.game-asset-review-catalog.v1\","
                 "\"title\":\"Asset review\",") +
         FString::Printf(TEXT("\"policy\":%s,"),
                         *FString(TEXT("{\"physics_authority\":\"mujoco\","
                                       "\"purpose\":\"presentation_review_only\","
                                       "\"qualified_visual\":false,\"runnable\":false,"
                                       "\"unreal_collision_profile\":\"NoCollision\","
                                       "\"unreal_simulate_physics\":false}"))) +
         TEXT(
             "\"coverage\":{\"qualified_movable_object_visual\":{\"available\":false,"
             "\"reason\":\"no qualified movable-object visual is registered\"}},") +
         FString::Printf(TEXT("\"cards\":[%s],\"digest\":\"%s\"}}}"), *CardsJson,
                         *FString::ChrN(64, TEXT('a')));
}

LingTuSim::UI::FRuntimeUIStatusSnapshot MakeQualificationReadyStatus(
    const LingTuSim::EControlStatusUIMode UIMode = LingTuSim::EControlStatusUIMode::Drive,
    const LingTuSim::EControlStatusCameraMode CameraMode =
        LingTuSim::EControlStatusCameraMode::Follow,
    const LingTuSim::EControlRecordingState RecordingState =
        LingTuSim::EControlRecordingState::Idle) {
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status;
  Status.bSessionAvailable = true;
  Status.SessionId = TEXT("session-a");
  Status.ModelGeneration = 7;
  Status.bControlBindingAvailable = true;
  Status.RunId = TEXT("run-qualification");
  Status.ResetGeneration = 11;
  Status.bIdentityCoherent = true;
  Status.VisualState = TEXT("Active");
  Status.bLatestAppliedTruthAvailable = true;
  Status.TruthSequence = 29;
  Status.SimTimeNs = 3'000'000'000;
  Status.bObservedBaseVelocityAvailable = true;
  Status.bInputObserved = true;
  Status.RequestedInput.bViewportFocused = true;
  Status.RequestedInput.bDeadman = true;
  Status.bFullStatusAvailable = true;
  Status.bFullStatusFresh = true;
  Status.bFullStatusIdentityCoherent = true;
  Status.FullStatusAgeNs = 1'000'000;

  LingTuSim::FControlStatusEnvelope &Full = Status.FullStatus;
  Full.RunId = Status.RunId;
  Full.SessionId = Status.SessionId;
  Full.BootId = TEXT("boot-qualification");
  Full.SourceId = TEXT("ue-operator");
  Full.EventId = TEXT("event-29");
  Full.IntentDatagramSha256 = FString::ChrN(64, TEXT('a'));
  Full.ModelGeneration = Status.ModelGeneration;
  Full.ResetGeneration = Status.ResetGeneration;
  Full.ServerStatusSequence = 31;
  Full.ServerMonotonicNs = 4'000'000'000;
  Full.ReceivedMonotonicNs = 4'001'000'000;
  Full.SimTimeNs = static_cast<uint64>(Status.SimTimeNs);
  Full.TruthSequence = Status.TruthSequence;
  Full.SourceEpoch = 2;
  Full.SourceSequence = 29;
  Full.Status = LingTuSim::EControlAckStatus::Accepted;
  Full.Motion.RequestedAxes.bAvailable = true;
  Full.Motion.RequestedAxes.Forward = 0.5;
  Full.Motion.AdmittedTwist.bAvailable = true;
  Full.Motion.AdmittedTwist.LinearX = 0.5;
  Full.Motion.ObservedBaseVelocity.bAvailable = true;
  Full.Motion.ObservedBaseVelocity.LinearX = 0.49;

  LingTuSim::FControlStatusReadinessFacet *Facets[] = {
      &Full.Readiness.Physics,
      &Full.Readiness.Control,
      &Full.Readiness.Visual,
      &Full.Readiness.Sensors,
  };
  for (LingTuSim::FControlStatusReadinessFacet *Facet : Facets) {
    Facet->State = LingTuSim::EControlBindingState::Active;
    Facet->bRequired = true;
    Facet->SourceId = TEXT("qualification-source");
  }
  const TCHAR *StreamIds[] = {
      TEXT("front_rgb"), TEXT("front_depth"), TEXT("imu"), TEXT("mid360"), TEXT("truth_odom"),
  };
  for (const TCHAR *StreamId : StreamIds) {
    LingTuSim::FControlStatusSensor Sensor;
    Sensor.StreamId = StreamId;
    Sensor.State = LingTuSim::EControlSensorState::Active;
    Sensor.SampleCount = 1;
    Full.Sensors.Add(MoveTemp(Sensor));
  }
  Full.Recording.State = RecordingState;
  Full.UI.UIMode = UIMode;
  Full.UI.CameraMode = CameraMode;
  switch (UIMode) {
    case LingTuSim::EControlStatusUIMode::Tactical:
      Status.ActualUIMode = TEXT("tactical");
      break;
    case LingTuSim::EControlStatusUIMode::Menu:
      Status.ActualUIMode = TEXT("menu");
      break;
    case LingTuSim::EControlStatusUIMode::Build:
      Status.ActualUIMode = TEXT("build");
      break;
    case LingTuSim::EControlStatusUIMode::Drive:
    default:
      Status.ActualUIMode = TEXT("drive");
      break;
  }
  switch (CameraMode) {
    case LingTuSim::EControlStatusCameraMode::Inspection:
      Status.ActualCameraMode = TEXT("inspection");
      break;
    case LingTuSim::EControlStatusCameraMode::Free:
      Status.ActualCameraMode = TEXT("free");
      break;
    case LingTuSim::EControlStatusCameraMode::Follow:
      Status.ActualCameraMode = TEXT("follow");
      break;
    case LingTuSim::EControlStatusCameraMode::Unavailable:
    default:
      Status.ActualCameraMode = TEXT("unavailable");
      break;
  }
  return Status;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIInteractiveGameEnablesByDefaultTest,
                                 "LingTuSim.UI.Runtime.InteractiveGameEnablesByDefault",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIInteractiveGameEnablesByDefaultTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  TestTrue(TEXT("interactive -game enables the runtime UI by default"),
           LingTuSim::UI::FRuntimeUIPolicy::ShouldEnable(TEXT("-game"), false, true));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIUnattendedDisablesByDefaultTest,
                                 "LingTuSim.UI.Runtime.UnattendedDisablesByDefault",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIUnattendedDisablesByDefaultTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  TestFalse(TEXT("unattended game runs hide the runtime UI by default"),
            LingTuSim::UI::FRuntimeUIPolicy::ShouldEnable(TEXT("-game -unattended"), true, true));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIForceFlagOverridesUnattendedTest,
                                 "LingTuSim.UI.Runtime.ForceFlagOverridesUnattended",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIForceFlagOverridesUnattendedTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  TestTrue(TEXT("explicit force flag can enable capture evidence in unattended runs"),
           LingTuSim::UI::FRuntimeUIPolicy::ShouldEnable(TEXT("-game -unattended -LingTuRuntimeUI"),
                                                         true, true));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIDisableFlagWinsTest,
                                 "LingTuSim.UI.Runtime.DisableFlagWins",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIDisableFlagWinsTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  TestFalse(TEXT("disable flag wins when both explicit flags are present"),
            LingTuSim::UI::FRuntimeUIPolicy::ShouldEnable(
                TEXT("-game -LingTuRuntimeUI -LingTuDisableRuntimeUI"), false, true));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIBuildKeyTogglesBuildModeTest,
                                 "LingTuSim.UI.Runtime.BuildKeyTogglesBuildMode",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIBuildKeyTogglesBuildModeTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIModeController Controller;
  TestTrue(TEXT("B is a runtime UI key"), Controller.HandleKey(EKeys::B));
  TestEqual(TEXT("B enters build mode"), Controller.GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Build);
  TestTrue(TEXT("B remains a runtime UI key"), Controller.HandleKey(EKeys::B));
  TestEqual(TEXT("B returns to drive mode"), Controller.GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Drive);
  TestFalse(TEXT("robot movement keys remain outside the runtime UI"),
            Controller.HandleKey(EKeys::W));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUITabKeyTogglesTacticalModeTest,
                                 "LingTuSim.UI.Runtime.TabKeyTogglesTacticalMode",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUITabKeyTogglesTacticalModeTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIModeController Controller;
  TestTrue(TEXT("Tab is a runtime UI key"), Controller.HandleKey(EKeys::Tab));
  TestEqual(TEXT("Tab enters tactical mode"), Controller.GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Tactical);
  TestTrue(TEXT("Tab remains a runtime UI key"), Controller.HandleKey(EKeys::Tab));
  TestEqual(TEXT("Tab returns to drive mode"), Controller.GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Drive);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIEscapeRestoresPreviousModeTest,
                                 "LingTuSim.UI.Runtime.EscapeRestoresPreviousMode",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIEscapeRestoresPreviousModeTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIModeController Controller;
  Controller.ToggleBuild();
  TestTrue(TEXT("Escape is a runtime UI key"), Controller.HandleKey(EKeys::Escape));
  TestEqual(TEXT("Escape enters pause mode"), Controller.GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Pause);
  TestTrue(TEXT("Escape remains a runtime UI key"), Controller.HandleKey(EKeys::Escape));
  TestEqual(TEXT("second Escape restores the mode that was paused"), Controller.GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Build);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIMenuExitRecordAndCameraKeysAreScopedTest,
                                 "LingTuSim.UI.Runtime.MenuExitRecordAndCameraKeysAreScoped",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIMenuExitRecordAndCameraKeysAreScopedTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::ERuntimeUIAction Action = LingTuSim::UI::ERuntimeUIAction::ToggleRecording;
  TestFalse(TEXT("X does not request exit in Drive"),
            LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Drive,
                                                              EKeys::X, Action));
  TestTrue(TEXT("X requests exit only from Menu"),
           LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Pause,
                                                             EKeys::X, Action));
  TestEqual(TEXT("Menu X resolves to runtime exit"), Action, LingTuSim::UI::ERuntimeUIAction::Exit);
  TestTrue(TEXT("R requests an authority-resolved recording toggle"),
           LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Drive,
                                                             EKeys::R, Action));
  TestEqual(TEXT("R never mutates a local recording flag"), Action,
            LingTuSim::UI::ERuntimeUIAction::ToggleRecording);
  TestTrue(TEXT("C cycles a real camera outside Menu"),
           LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(
               LingTuSim::UI::ERuntimeUIMode::Tactical, EKeys::C, Action));
  TestEqual(TEXT("C resolves to the camera seam"), Action,
            LingTuSim::UI::ERuntimeUIAction::CycleCamera);
  TestFalse(TEXT("C is disabled while Menu owns the viewport"),
            LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Pause,
                                                              EKeys::C, Action));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIGameSelectionKeysAreMenuScopedTest,
                                 "LingTuSim.UI.Runtime.GameSelectionKeysAreMenuScoped",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIGameSelectionKeysAreMenuScopedTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::ERuntimeUIAction Action = LingTuSim::UI::ERuntimeUIAction::ToggleRecording;
  TestFalse(TEXT("Down remains available to gameplay outside Menu"),
            LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Drive,
                                                              EKeys::Down, Action));
  TestTrue(TEXT("Menu Down selects the next compiled option"),
           LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Pause,
                                                             EKeys::Down, Action));
  TestEqual(TEXT("Menu Down action"), Action, LingTuSim::UI::ERuntimeUIAction::SelectNextGame);
  TestTrue(TEXT("Menu Up selects the previous compiled option"),
           LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Pause,
                                                             EKeys::Up, Action));
  TestEqual(TEXT("Menu Up action"), Action, LingTuSim::UI::ERuntimeUIAction::SelectPreviousGame);
  TestTrue(TEXT("Menu Enter confirms through the launcher handoff"),
           LingTuSim::UI::FRuntimeUIActionPolicy::ResolveKey(LingTuSim::UI::ERuntimeUIMode::Pause,
                                                             EKeys::Enter, Action));
  TestEqual(TEXT("Menu Enter action"), Action,
            LingTuSim::UI::ERuntimeUIAction::ConfirmGameSelection);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUINoSessionDisplaysUnavailableTest,
                                 "LingTuSim.UI.Runtime.NoSessionDisplaysUnavailable",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUINoSessionDisplaysUnavailableTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FString PreviousSessionId;
  uint64 PreviousGeneration = 0;
  const bool bHadPreviousSession =
      LingTuSim::FSessionService::GetBoundSession(PreviousSessionId, PreviousGeneration);
  LingTuSim::FSessionService::UnbindSession();

  const LingTuSim::UI::FRuntimeUIStatus Status =
      LingTuSim::UI::FRuntimeUIStatusReader::Read(nullptr);
  TestFalse(TEXT("no session is reported as unavailable"), Status.bSessionAvailable);
  TestEqual(TEXT("session label is explicit"), Status.SessionState, FString(TEXT("Unavailable")));
  TestEqual(TEXT("visual label is explicit"), Status.VisualState, FString(TEXT("Unavailable")));
  TestEqual(TEXT("control label is explicit"), Status.ControlState, FString(TEXT("Unavailable")));
  TestEqual(TEXT("ACK label is explicit"), Status.ControlAckState, FString(TEXT("Unavailable")));
  TestEqual(TEXT("observed velocity label is explicit"), Status.ObservedBaseVelocityState,
            FString(TEXT("Unavailable")));
  TestFalse(TEXT("an unavailable status cannot qualify a Drive capture"),
            Status.IsDriveCaptureReady());

  if (bHadPreviousSession) {
    TestTrue(
        TEXT("previous session binding is restored"),
        LingTuSim::FSessionService::RebindSession(MoveTemp(PreviousSessionId), PreviousGeneration));
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIRequestedInputNeverBecomesAcceptedMotionTest,
                                 "LingTuSim.UI.Runtime.RequestedInputNeverBecomesAcceptedMotion",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIRequestedInputNeverBecomesAcceptedMotionTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  FString PreviousSessionId;
  uint64 PreviousGeneration = 0;
  const bool bHadPreviousSession =
      LingTuSim::FSessionService::GetBoundSession(PreviousSessionId, PreviousGeneration);
  LingTuSim::FSessionService::UnbindSession();

  LingTuSim::UI::FRuntimeUILocalState LocalState;
  LocalState.bInputObserved = true;
  LocalState.RequestedInput.bDriveMode = true;
  LocalState.RequestedInput.bViewportFocused = true;
  LocalState.RequestedInput.bDeadman = true;
  LocalState.RequestedInput.Forward = 0.75f;
  const LingTuSim::UI::FRuntimeUIStatusSnapshot Status =
      LingTuSim::UI::FRuntimeUIStatusReader::Read(nullptr, &LocalState);

  TestTrue(TEXT("local requested input remains displayable"), Status.bInputObserved);
  TestEqual(TEXT("requested normalized forward is preserved"), Status.RequestedInput.Forward,
            0.75f);
  TestFalse(TEXT("requested input never fabricates an ACK"), Status.bControlAckAvailable);
  TestFalse(TEXT("requested input never fabricates observed motion"),
            Status.bObservedBaseVelocityAvailable);
  TestEqual(TEXT("observed motion stays explicitly unavailable"), Status.ObservedBaseVelocityState,
            FString(TEXT("Unavailable")));

  if (bHadPreviousSession) {
    TestTrue(
        TEXT("previous session binding is restored"),
        LingTuSim::FSessionService::RebindSession(MoveTemp(PreviousSessionId), PreviousGeneration));
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIHudCaptureRequiresInputAckAndObservedTruthTest,
                                 "LingTuSim.UI.Runtime.HudCaptureRequiresInputAckAndObservedTruth",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIHudCaptureRequiresInputAckAndObservedTruthTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status = MakeQualificationReadyStatus();
  TestTrue(TEXT("the complete authoritative status qualifies Drive"), Status.IsDriveCaptureReady());
  Status.bInputObserved = false;
  TestFalse(TEXT("Drive capture requires real local input observation"),
            Status.IsDriveCaptureReady());
  Status.bInputObserved = true;
  Status.FullStatus.Motion.AdmittedTwist.bAvailable = false;
  TestFalse(TEXT("requested input without admitted motion is not capture evidence"),
            Status.IsDriveCaptureReady());
  Status.FullStatus.Motion.AdmittedTwist.bAvailable = true;
  Status.FullStatus.Motion.ObservedBaseVelocity.bAvailable = false;
  TestFalse(TEXT("admission without authoritative observed motion is incomplete"),
            Status.IsDriveCaptureReady());
  Status.FullStatus.Motion.ObservedBaseVelocity.bAvailable = true;
  Status.FullStatus.UI.CameraMode = LingTuSim::EControlStatusCameraMode::Unavailable;
  TestFalse(TEXT("a local-only camera mode cannot qualify a HUD capture"),
            Status.IsDriveCaptureReady());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIHudCaptureRejectsNonAcceptedAckStatesTest,
                                 "LingTuSim.UI.Runtime.HudCaptureRejectsNonAcceptedAckStates",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIHudCaptureRejectsNonAcceptedAckStatesTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status = MakeQualificationReadyStatus();

  const LingTuSim::EControlAckStatus NonqualifyingStatuses[] = {
      LingTuSim::EControlAckStatus::Pending,
      LingTuSim::EControlAckStatus::Rejected,
      LingTuSim::EControlAckStatus::Released,
      LingTuSim::EControlAckStatus::TimeoutZero,
  };
  for (const LingTuSim::EControlAckStatus AckStatus : NonqualifyingStatuses) {
    Status.FullStatus.Status = AckStatus;
    TestFalse(TEXT("non-accepted authoritative status cannot qualify capture"),
              Status.IsDriveCaptureReady());
  }

  Status.FullStatus.Status = LingTuSim::EControlAckStatus::Accepted;
  TestTrue(TEXT("Accepted ACK qualifies the ACK checkpoint"), Status.IsDriveCaptureReady());
  Status.FullStatus.Status = LingTuSim::EControlAckStatus::Confirmed;
  TestTrue(TEXT("Confirmed ACK qualifies the ACK checkpoint"), Status.IsDriveCaptureReady());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIBoundSessionReadLeavesIngressOwnedByVisualTest,
                                 "LingTuSim.UI.Runtime.BoundSessionReadLeavesIngressOwnedByVisual",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIBoundSessionReadLeavesIngressOwnedByVisualTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  FString PreviousSessionId;
  uint64 PreviousGeneration = 0;
  const bool bHadPreviousSession =
      LingTuSim::FSessionService::GetBoundSession(PreviousSessionId, PreviousGeneration);
  LingTuSim::FSessionService::UnbindSession();

  const FString SessionId = TEXT("session-a");
  TestTrue(TEXT("fixture session binds"), LingTuSim::FSessionService::RebindSession(SessionId, 7));
  LingTuSim::FSnapshotEnvelope Snapshot;
  Snapshot.SessionId = SessionId;
  Snapshot.ModelGeneration = 7;
  Snapshot.Sequence = 1;
  TestEqual(TEXT("fixture snapshot is accepted"),
            LingTuSim::FSessionService::PublishSnapshot(Snapshot),
            LingTuSim::ESnapshotPublishResult::Accepted);

  const LingTuSim::UI::FRuntimeUIStatus Status =
      LingTuSim::UI::FRuntimeUIStatusReader::Read(nullptr);
  TestTrue(TEXT("bound session is visible"), Status.bSessionAvailable);
  TestEqual(TEXT("bound generation is copied"), Status.ModelGeneration, static_cast<uint64>(7));
  TestEqual(TEXT("worldless visual remains explicit"), Status.VisualState,
            FString(TEXT("Unavailable")));

  LingTuSim::FSnapshotEnvelope RemainingSnapshot;
  TestTrue(TEXT("status read does not take the visual runtime's pending snapshot"),
           LingTuSim::FSessionService::TryTakeLatestSnapshot(RemainingSnapshot));
  TestEqual(TEXT("visual ingress retains the original sequence"), RemainingSnapshot.Sequence,
            static_cast<uint64>(1));

  LingTuSim::FSessionService::UnbindSession();
  if (bHadPreviousSession) {
    TestTrue(
        TEXT("previous session binding is restored"),
        LingTuSim::FSessionService::RebindSession(MoveTemp(PreviousSessionId), PreviousGeneration));
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIKeyboardMapsAndCancelsOppositesTest,
                                 "LingTuSim.UI.Runtime.KeyboardMapsAndCancelsOpposites",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIKeyboardMapsAndCancelsOppositesTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRobotDriveInputState State;
  State.SetDriveMode(true);
  State.SetViewportFocused(true);
  State.HandleKeyDown(EKeys::LeftShift, false);
  State.HandleKeyDown(EKeys::W, false);

  LingTuSim::UI::FRobotDriveInputSnapshot Snapshot = State.GetSnapshot();
  TestTrue(TEXT("keyboard deadman is active"), Snapshot.bDeadman);
  TestEqual(TEXT("W requests forward"), Snapshot.Forward, 1.0f);
  TestEqual(TEXT("W does not request body-left"), Snapshot.Left, 0.0f);
  TestEqual(TEXT("W does not request yaw-left"), Snapshot.YawLeft, 0.0f);

  State.HandleKeyDown(EKeys::S, false);
  Snapshot = State.GetSnapshot();
  TestEqual(TEXT("W and S cancel"), Snapshot.Forward, 0.0f);
  State.HandleKeyUp(EKeys::W);
  TestEqual(TEXT("S requests backward"), State.GetSnapshot().Forward, -1.0f);
  State.HandleKeyDown(EKeys::W, false);

  State.HandleKeyDown(EKeys::A, false);
  State.HandleKeyDown(EKeys::D, false);
  State.HandleKeyDown(EKeys::Q, false);
  State.HandleKeyDown(EKeys::E, false);
  Snapshot = State.GetSnapshot();
  TestEqual(TEXT("A and D cancel"), Snapshot.Left, 0.0f);
  TestEqual(TEXT("Q and E cancel"), Snapshot.YawLeft, 0.0f);
  State.HandleKeyUp(EKeys::A);
  State.HandleKeyUp(EKeys::Q);
  Snapshot = State.GetSnapshot();
  TestEqual(TEXT("D requests body-right"), Snapshot.Left, -1.0f);
  TestEqual(TEXT("E requests yaw-right"), Snapshot.YawLeft, -1.0f);
  State.HandleKeyDown(EKeys::A, false);
  State.HandleKeyDown(EKeys::Q, false);

  State.HandleKeyUp(EKeys::S);
  State.HandleKeyUp(EKeys::D);
  State.HandleKeyUp(EKeys::E);
  Snapshot = State.GetSnapshot();
  TestEqual(TEXT("releasing S restores W"), Snapshot.Forward, 1.0f);
  TestEqual(TEXT("releasing D restores A"), Snapshot.Left, 1.0f);
  TestEqual(TEXT("releasing E restores Q"), Snapshot.YawLeft, 1.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIKeyboardRepeatIsIgnoredTest,
                                 "LingTuSim.UI.Runtime.KeyboardRepeatIsIgnored",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIKeyboardRepeatIsIgnoredTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRobotDriveInputState State;
  State.SetDriveMode(true);
  State.SetViewportFocused(true);

  TestEqual(TEXT("initial key down changes state"), State.HandleKeyDown(EKeys::W, false),
            LingTuSim::UI::ERobotDriveInputUpdate::Changed);
  TestEqual(TEXT("repeat is handled without changing state"), State.HandleKeyDown(EKeys::W, true),
            LingTuSim::UI::ERobotDriveInputUpdate::Handled);
  TestEqual(TEXT("repeat does not double the axis"), State.GetSnapshot().Forward, 0.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuRuntimeUIGamepadDeadZoneClampAndCameraAxesStaySeparateTest,
    "LingTuSim.UI.Runtime.GamepadDeadZoneClampAndCameraAxesStaySeparate",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIGamepadDeadZoneClampAndCameraAxesStaySeparateTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRobotDriveInputState State;
  State.SetDriveMode(true);
  State.SetViewportFocused(true);
  State.HandleKeyDown(EKeys::Gamepad_LeftShoulder, false);

  State.HandleAnalog(EKeys::Gamepad_LeftY, 0.10f);
  TestEqual(TEXT("left stick rests inside the radial dead zone"), State.GetSnapshot().Forward,
            0.0f);

  State.HandleAnalog(EKeys::Gamepad_LeftX, -2.0f);
  State.HandleAnalog(EKeys::Gamepad_LeftY, 0.0f);
  TestEqual(TEXT("left stick maps physical left to body-left"), State.GetSnapshot().Left, 1.0f);
  State.HandleAnalog(EKeys::Gamepad_LeftX, 0.0f);
  State.HandleAnalog(EKeys::Gamepad_LeftY, 2.0f);
  State.HandleAnalog(EKeys::Gamepad_LeftTriggerAxis, 2.0f);
  State.HandleAnalog(EKeys::Gamepad_RightX, 0.75f);
  State.HandleAnalog(EKeys::Gamepad_RightY, -0.50f);
  const LingTuSim::UI::FRobotDriveInputSnapshot Snapshot = State.GetSnapshot();
  TestEqual(TEXT("left stick is clamped"), Snapshot.Forward, 1.0f);
  TestEqual(TEXT("left trigger requests clamped yaw-left"), Snapshot.YawLeft, 1.0f);
  TestEqual(TEXT("right stick never leaks into robot body-left"), Snapshot.Left, 0.0f);
  TestEqual(TEXT("right stick never leaks into robot yaw-left"), Snapshot.YawLeft, 1.0f);
  TestTrue(TEXT("right stick exposes camera yaw"), Snapshot.CameraYaw > 0.0f);
  TestTrue(TEXT("right stick exposes camera pitch"), Snapshot.CameraPitch < 0.0f);

  State.HandleAnalog(EKeys::Gamepad_LeftTriggerAxis, 0.0f);
  State.HandleAnalog(EKeys::Gamepad_RightTriggerAxis, 2.0f);
  TestEqual(TEXT("right trigger requests yaw-right"), State.GetSnapshot().YawLeft, -1.0f);

  State.SetDriveMode(false);
  State.SetViewportFocused(true);
  State.HandleAnalog(EKeys::Gamepad_RightX, 0.8f);
  const LingTuSim::UI::FRobotDriveInputSnapshot NonDriveLook = State.GetSnapshot();
  TestTrue(TEXT("camera look remains controllable outside robot Drive mode"),
           NonDriveLook.CameraYaw > 0.0f);
  TestEqual(TEXT("non-Drive camera look never emits robot forward"), NonDriveLook.Forward, 0.0f);
  TestEqual(TEXT("non-Drive camera look never emits robot body-left"), NonDriveLook.Left, 0.0f);
  TestEqual(TEXT("non-Drive camera look never emits robot yaw"), NonDriveLook.YawLeft, 0.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIMotionRequiresDriveDeadmanAndForegroundTest,
                                 "LingTuSim.UI.Runtime.MotionRequiresDriveDeadmanAndForeground",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIMotionRequiresDriveDeadmanAndForegroundTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRobotDriveInputState State;
  State.SetDriveMode(true);
  State.SetViewportFocused(true);
  State.HandleKeyDown(EKeys::W, false);
  TestEqual(TEXT("movement without deadman is zero"), State.GetSnapshot().Forward, 0.0f);

  State.HandleKeyDown(EKeys::LeftShift, false);
  TestEqual(TEXT("deadman admits movement"), State.GetSnapshot().Forward, 1.0f);

  TestEqual(TEXT("leaving Drive emits release"), State.SetDriveMode(false),
            LingTuSim::UI::ERobotDriveInputUpdate::Released);
  TestEqual(TEXT("non-Drive movement is zero"), State.GetSnapshot().Forward, 0.0f);

  State.SetDriveMode(true);
  State.HandleKeyDown(EKeys::LeftShift, false);
  State.HandleKeyDown(EKeys::W, false);
  TestEqual(TEXT("focus loss emits release"), State.SetViewportFocused(false),
            LingTuSim::UI::ERobotDriveInputUpdate::Released);
  TestEqual(TEXT("background movement is zero"), State.GetSnapshot().Forward, 0.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIReleasePathsPublishZeroTest,
                                 "LingTuSim.UI.Runtime.ReleasePathsPublishZero",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIReleasePathsPublishZeroTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRobotDriveInputState State;
  State.SetDriveMode(true);
  State.SetViewportFocused(true);
  State.HandleKeyDown(EKeys::LeftShift, false);
  State.HandleKeyDown(EKeys::W, false);

  TestEqual(TEXT("deadman key-up emits release"), State.HandleKeyUp(EKeys::LeftShift),
            LingTuSim::UI::ERobotDriveInputUpdate::Released);
  TestTrue(TEXT("release snapshot is explicit"), State.GetSnapshot().bRelease);
  TestEqual(TEXT("release snapshot zeros forward"), State.GetSnapshot().Forward, 0.0f);

  State.HandleKeyDown(EKeys::LeftShift, false);
  TestEqual(TEXT("teardown release is explicit"), State.ReleaseAll(),
            LingTuSim::UI::ERobotDriveInputUpdate::Released);
  TestFalse(TEXT("teardown clears viewport focus"), State.GetSnapshot().bViewportFocused);
  TestFalse(TEXT("teardown clears deadman"), State.GetSnapshot().bDeadman);
  TestEqual(TEXT("teardown zeros body-left"), State.GetSnapshot().Left, 0.0f);
  TestEqual(TEXT("teardown zeros yaw-left"), State.GetSnapshot().YawLeft, 0.0f);
  TestEqual(TEXT("teardown zeros camera yaw"), State.GetSnapshot().CameraYaw, 0.0f);
  TestEqual(TEXT("teardown zeros camera pitch"), State.GetSnapshot().CameraPitch, 0.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIMultipleEligibleViewportContextsFailClosedTest,
                                 "LingTuSim.UI.Runtime.MultipleEligibleViewportContextsFailClosed",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIMultipleEligibleViewportContextsFailClosedTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  TestTrue(TEXT("one eligible world with one local player may own input"),
           LingTuSim::UI::FRuntimeUIPolicy::CanOwnInputContext(1, 1));
  TestFalse(TEXT("multiple eligible worlds fail closed"),
            LingTuSim::UI::FRuntimeUIPolicy::CanOwnInputContext(2, 1));
  TestFalse(TEXT("split-screen local players fail closed"),
            LingTuSim::UI::FRuntimeUIPolicy::CanOwnInputContext(1, 2));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIHudScreenshotArgumentsAreExactTripleTest,
                                 "LingTuSim.UI.Runtime.HudScreenshotArgumentsAreExactTriple",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIHudScreenshotArgumentsAreExactTripleTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const FString Root = FPaths::ConvertRelativePathToFull(
      FPaths::AutomationTransientDir() / TEXT("LingTuHudContract") / TEXT("screenshots"));
  const FString Complete = FString::Printf(
      TEXT("-LingTuHudDriveScreenshot=\"%s/hud-drive.png\" ")
          TEXT("-LingTuHudTacticalScreenshot=\"%s/hud-tactical.png\" ")
              TEXT("-LingTuHudMenuRecordingScreenshot=\"%s/hud-menu-recording.png\""),
      *Root, *Root, *Root);
  TArray<LingTuSim::UI::FHudScreenshotTarget> Targets;
  bool bConfigured = false;
  FString Error;
  const bool bParsed = LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(Complete, Targets,
                                                                               bConfigured, Error);
  TestTrue(TEXT("the valid quoted screenshot triple parses"), bParsed);
  TestTrue(TEXT("the screenshot triple is configured"), bConfigured);
  TestEqual(TEXT("all three screenshot targets are retained"), Targets.Num(), 3);
  if (!bParsed || !bConfigured || Targets.Num() != 3) {
    AddError(
        FString::Printf(TEXT("cannot inspect screenshot triple after parse failure: %s"), *Error));
    return false;
  }
  TestFalse(TEXT("command-line quotes are not part of the drive path"),
            Targets[0].ScreenshotPath.Contains(TEXT("\"")));
  TestEqual(TEXT("drive target is exact"), Targets[0].ExpectedFilename,
            FString(TEXT("hud-drive.png")));
  TestEqual(TEXT("tactical target is exact"), Targets[1].ExpectedFilename,
            FString(TEXT("hud-tactical.png")));
  TestEqual(TEXT("menu recording target is exact"), Targets[2].ExpectedFilename,
            FString(TEXT("hud-menu-recording.png")));
  TestFalse(TEXT("menu cannot capture before drive even when menu is the actual mode"),
            LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
                Targets[2], 0, LingTuSim::UI::ERuntimeUIMode::Pause));
  TestTrue(TEXT("drive is the only first capture"),
           LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
               Targets[0], 0, LingTuSim::UI::ERuntimeUIMode::Drive));
  TestFalse(TEXT("menu cannot skip tactical after drive"),
            LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
                Targets[2], 1, LingTuSim::UI::ERuntimeUIMode::Pause));
  TestTrue(TEXT("tactical is the only second capture"),
           LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
               Targets[1], 1, LingTuSim::UI::ERuntimeUIMode::Tactical));
  TestTrue(TEXT("menu recording is the only third capture"),
           LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
               Targets[2], 2, LingTuSim::UI::ERuntimeUIMode::Pause));

  TestFalse(TEXT("an unmatched leading quote is rejected"),
            LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(
                TEXT("-LingTuHudDriveScreenshot=\"C:/run/screenshots/hud-drive.png"), Targets,
                bConfigured, Error));
  TestEqual(TEXT("leading quote failure is machine-readable"), Error,
            FString(TEXT("hud_screenshot_argument_quotes_invalid")));
  TestFalse(TEXT("an unmatched trailing quote is rejected"),
            LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(
                TEXT("-LingTuHudDriveScreenshot=C:/run/screenshots/hud-drive.png\""), Targets,
                bConfigured, Error));
  TestEqual(TEXT("trailing quote failure is machine-readable"), Error,
            FString(TEXT("hud_screenshot_argument_quotes_invalid")));
  TestFalse(TEXT("a residual quote is rejected"),
            LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(
                TEXT("-LingTuHudDriveScreenshot=C:/run/screen\"shots/hud-drive.png"), Targets,
                bConfigured, Error));
  TestEqual(TEXT("residual quote failure is machine-readable"), Error,
            FString(TEXT("hud_screenshot_argument_quotes_invalid")));

  const FString DuplicateAfterNormalization =
      TEXT("-LingTuHudDriveScreenshot=\"C:/run/screenshots/hud-drive.png\" ")
          TEXT("-LingTuHudDriveScreenshot=C:/run/screenshots/hud-drive.png ")
              TEXT("-LingTuHudTacticalScreenshot=C:/run/screenshots/hud-tactical.png ") TEXT(
                  "-LingTuHudMenuRecordingScreenshot=C:/run/screenshots/hud-menu-recording.png");
  TestFalse(TEXT("quoted and unquoted duplicate assignments are rejected"),
            LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(DuplicateAfterNormalization,
                                                                    Targets, bConfigured, Error));
  TestEqual(TEXT("duplicate failure is machine-readable"), Error,
            FString(TEXT("hud_screenshot_arguments_partial_or_duplicated")));

  TestFalse(
      TEXT("a partial screenshot argument set fails closed"),
      LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(
          Complete.Left(Complete.Find(TEXT(" -LingTuHudTactical"))), Targets, bConfigured, Error));
  TestEqual(TEXT("partial failure is machine-readable"), Error,
            FString(TEXT("hud_screenshot_arguments_partial_or_duplicated")));
  TestFalse(TEXT("the legacy singular screenshot argument is rejected"),
            LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(
                TEXT("-LingTuHudScreenshot=C:/run/logs/hud.png"), Targets, bConfigured, Error));
  TestEqual(TEXT("legacy failure is explicit"), Error,
            FString(TEXT("legacy_singular_hud_screenshot_argument_rejected")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuRuntimeUIHudScreenshotPathValidationRejectsForeignAndNoncanonicalTargetsTest,
    "LingTuSim.UI.Runtime.HudScreenshotPathValidationRejectsForeignAndNoncanonicalTargets",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIHudScreenshotPathValidationRejectsForeignAndNoncanonicalTargetsTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  IFileManager &FileManager = IFileManager::Get();
  const FString FixtureId = FGuid::NewGuid().ToString(EGuidFormats::Digits);
  const FString RunDirectory = FPaths::ConvertRelativePathToFull(
      FPaths::AutomationTransientDir() / TEXT("LingTuHudPathValidationRun") / FixtureId);
  const FString LogDirectory = RunDirectory / TEXT("logs");
  const FString ScreenshotDirectory = RunDirectory / TEXT("screenshots");
  FileManager.DeleteDirectory(*RunDirectory, false, true);
  if (!TestTrue(TEXT("fixture log directory is created"),
                FileManager.MakeDirectory(*LogDirectory, true)) ||
      !TestTrue(TEXT("fixture screenshot directory is created"),
                FileManager.MakeDirectory(*ScreenshotDirectory, true))) {
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }

  LingTuSim::UI::FHudScreenshotTarget Target;
  Target.CommandLineName = TEXT("LingTuHudDriveScreenshot");
  Target.ExpectedFilename = TEXT("hud-drive.png");
  Target.ModeName = TEXT("drive");
  Target.Mode = LingTuSim::UI::ERuntimeUIMode::Drive;
  Target.ScreenshotPath = ScreenshotDirectory / Target.ExpectedFilename;
  Target.EvidencePath = FPaths::ChangeExtension(Target.ScreenshotPath, TEXT("evidence.json"));
  FString Error;
  if (!TestTrue(TEXT("an absent exact run-owned target is valid before request"),
                LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
                    Target, LogDirectory,
                    LingTuSim::UI::EHudScreenshotValidationPhase::BeforeRequest, Error))) {
    AddError(
        FString::Printf(TEXT("absent run-owned target validation failed with error='%s'"), *Error));
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }

#if PLATFORM_WINDOWS
  const FString ReparseBackingDirectory = RunDirectory / TEXT("reparse-backing");
  FileManager.DeleteDirectory(*ReparseBackingDirectory, false, true);
  if (!TestTrue(TEXT("reparse backing directory is created"),
                FileManager.MakeDirectory(*ReparseBackingDirectory, true)) ||
      !TestTrue(TEXT("plain screenshot directory is removed before link fixture"),
                FileManager.DeleteDirectory(*ScreenshotDirectory, false, true))) {
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }
  constexpr DWORD AllowUnprivilegedCreate = 0x2;
  if (::CreateSymbolicLinkW(*ScreenshotDirectory, *ReparseBackingDirectory,
                            SYMBOLIC_LINK_FLAG_DIRECTORY | AllowUnprivilegedCreate)) {
    TestFalse(TEXT("a run/screenshots reparse ancestor fails closed"),
              LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
                  Target, LogDirectory, LingTuSim::UI::EHudScreenshotValidationPhase::BeforeRequest,
                  Error));
    TestEqual(TEXT("reparse ancestor failure is explicit"), Error,
              FString(TEXT("reparse_path_component")));
    TestTrue(TEXT("reparse fixture is removed without following it"),
             ::RemoveDirectoryW(*ScreenshotDirectory) != 0);
  } else {
    AddWarning(
        TEXT("Windows symlink creation unavailable; reparse behavior remains source-covered"));
  }
  if (!TestTrue(TEXT("plain screenshot directory is restored"),
                FileManager.MakeDirectory(*ScreenshotDirectory, true))) {
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }
  FileManager.DeleteDirectory(*ReparseBackingDirectory, false, true);

  const FString MissingScreenshotBacking = RunDirectory / TEXT("missing-screenshot-backing.png");
  if (::CreateSymbolicLinkW(*Target.ScreenshotPath, *MissingScreenshotBacking,
                            AllowUnprivilegedCreate)) {
    const bool bDanglingTargetAccepted =
        LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
            Target, LogDirectory, LingTuSim::UI::EHudScreenshotValidationPhase::BeforeRequest,
            Error);
    TestFalse(TEXT("a dangling screenshot target is rejected without following it"),
              bDanglingTargetAccepted);
    TestEqual(TEXT("dangling target failure is explicit"), Error,
              FString(TEXT("reparse_target_file")));
    if (!TestTrue(TEXT("dangling target link is removed without following it"),
                  ::DeleteFileW(*Target.ScreenshotPath) != 0)) {
      FileManager.DeleteDirectory(*RunDirectory, false, true);
      return false;
    }
  } else {
    AddWarning(TEXT(
        "Windows dangling symlink creation unavailable; no-follow target remains source-covered"));
  }
#endif

  if (!TestTrue(TEXT("evidence temp is created exclusively through the no-follow writer"),
                LingTuSim::UI::FHudScreenshotContract::WriteEvidenceTempFileNoFollow(
                    Target, TEXT("{}\n"), Error))) {
    AddError(
        FString::Printf(TEXT("evidence temp no-follow writer failed with error='%s'"), *Error));
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }
  if (!TestTrue(
          TEXT("the no-follow writer leaves one plain temp file"),
          LingTuSim::UI::FHudScreenshotContract::ValidateEvidenceTempTarget(Target, true, Error))) {
    AddError(FString::Printf(TEXT("evidence temp validation failed with error='%s'"), *Error));
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }
  if (!TestTrue(TEXT("evidence temp fixture is removed"),
                FileManager.Delete(*(Target.EvidencePath + TEXT(".tmp")), false, true, true))) {
    FileManager.DeleteDirectory(*RunDirectory, false, true);
    return false;
  }

  Target.ScreenshotPath = RunDirectory / TEXT("foreign/hud-drive.png");
  Target.EvidencePath = FPaths::ChangeExtension(Target.ScreenshotPath, TEXT("evidence.json"));
  TestFalse(TEXT("a target outside run/screenshots fails closed"),
            LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
                Target, LogDirectory, LingTuSim::UI::EHudScreenshotValidationPhase::BeforeRequest,
                Error));
  TestEqual(TEXT("foreign target failure is explicit"), Error,
            FString(TEXT("path_outside_run_screenshots_directory")));
  FileManager.DeleteDirectory(*RunDirectory, false, true);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIHudScreenshotPngDimensionsMustBeExactTest,
                                 "LingTuSim.UI.Runtime.HudScreenshotPngDimensionsMustBeExact",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIHudScreenshotPngDimensionsMustBeExactTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const FString Path = FPaths::ConvertRelativePathToFull(FPaths::AutomationTransientDir() /
                                                         TEXT("LingTuHudDimensions.png"));
  TArray<uint8> Header = {
      137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 'I', 'H', 'D', 'R', 0, 0, 7, 128, 0, 0, 4, 56,
  };
  TestTrue(TEXT("fixture PNG header is written"), FFileHelper::SaveArrayToFile(Header, *Path));
  int32 Width = 0;
  int32 Height = 0;
  FString Error;
  TestTrue(TEXT("PNG dimensions are read from IHDR"),
           LingTuSim::UI::FHudScreenshotContract::ReadPngDimensions(Path, Width, Height, Error));
  TestEqual(TEXT("width is exact"), Width, 1920);
  TestEqual(TEXT("height is exact"), Height, 1080);
  IFileManager::Get().Delete(*Path, false, true, true);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIHudEvidenceJsonEscapesUntrustedStringsTest,
                                 "LingTuSim.UI.Runtime.HudEvidenceJsonEscapesUntrustedStrings",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIHudEvidenceJsonEscapesUntrustedStringsTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status = MakeQualificationReadyStatus();
  const FString Untrusted = TEXT("artifact\"\\line\n\u96ea");
  Status.FullStatus.Recording.ArtifactId = Untrusted;
  Status.FullStatus.EventId = TEXT("event\"with\\delimiters");

  LingTuSim::UI::FHudScreenshotTarget Target;
  Target.Mode = LingTuSim::UI::ERuntimeUIMode::Drive;
  Target.ModeName = TEXT("drive");
  Target.ExpectedFilename = TEXT("hud-drive.png");
  Target.ScreenshotPath = TEXT("C:/run/screenshots/hud-drive.png");
  Target.EvidencePath = TEXT("C:/run/screenshots/hud-drive.evidence.json");
  FString Json;
  FString Error;
  constexpr uint64 CapturedMonotonicNs = 4'003'000'000;
  TestTrue(TEXT("qualification-ready evidence serializes through the JSON API"),
           LingTuSim::UI::FHudScreenshotContract::SerializeEvidenceJson(
               Target, Status, CapturedMonotonicNs, 4096, 1920, 1080, Json, Error));
  TestTrue(TEXT("quotes are JSON escaped"), Json.Contains(TEXT("\\\"")));
  TestTrue(TEXT("backslashes are JSON escaped"), Json.Contains(TEXT("\\\\")));

  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  TestTrue(TEXT("serialized evidence remains parseable JSON"),
           FJsonSerializer::Deserialize(Reader, Root) && Root.IsValid());
  if (Root.IsValid()) {
    const TSharedPtr<FJsonObject> Recording = Root->GetObjectField(TEXT("recording"));
    TestEqual(TEXT("untrusted artifact text round-trips without injection"),
              Recording->GetStringField(TEXT("artifact_id")), Untrusted);
    TestTrue(TEXT("serialized evidence is qualification-ready"),
             Root->GetBoolField(TEXT("qualification_ready")));
    uint64 StatusAgeNs = 0;
    TestTrue(TEXT("status age remains an exact integer"),
             Root->TryGetNumberField(TEXT("status_age_ns"), StatusAgeNs));
    TestEqual(TEXT("status age is measured at the capture instant, not copied from a prior tick"),
              StatusAgeNs, CapturedMonotonicNs - Status.FullStatus.ReceivedMonotonicNs);
    TestNotEqual(TEXT("stale tick age is never serialized as capture evidence"), StatusAgeNs,
                 Status.FullStatusAgeNs);
  }

  TestFalse(TEXT("a capture timestamp before status receipt fails closed"),
            LingTuSim::UI::FHudScreenshotContract::SerializeEvidenceJson(
                Target, Status, Status.FullStatus.ReceivedMonotonicNs - 1, 4096, 1920, 1080, Json,
                Error));
  TestEqual(TEXT("monotonic ordering failure is explicit"), Error,
            FString(TEXT("captured_time_precedes_status_receive")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuRuntimeUIRecordingActionsUseOnlyAuthoritativeLifecycleTest,
    "LingTuSim.UI.Runtime.RecordingActionsUseOnlyAuthoritativeLifecycle",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIRecordingActionsUseOnlyAuthoritativeLifecycleTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status = MakeQualificationReadyStatus();
  TestTrue(TEXT("idle enables record_start"), Status.CanRequestRecordStart());
  TestFalse(TEXT("idle cannot stop a recording"), Status.CanRequestRecordStopCommit());
  Status.FullStatus.Recording.State = LingTuSim::EControlRecordingState::Recording;
  TestFalse(TEXT("recording cannot start another writer"), Status.CanRequestRecordStart());
  TestTrue(TEXT("recording enables record_stop_commit"), Status.CanRequestRecordStopCommit());
  Status.bFullStatusFresh = false;
  TestFalse(TEXT("stale status disables start"), Status.CanRequestRecordStart());
  TestFalse(TEXT("stale status disables stop"), Status.CanRequestRecordStopCommit());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIExitRequiresExactStoppedZeroConfirmationTest,
                                 "LingTuSim.UI.Runtime.ExitRequiresExactStoppedZeroConfirmation",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIExitRequiresExactStoppedZeroConfirmationTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status = MakeQualificationReadyStatus(
      LingTuSim::EControlStatusUIMode::Menu, LingTuSim::EControlStatusCameraMode::Follow);
  const FString ExitEventId = TEXT("boot-qualification:2:29");
  Status.FullStatus.EventId = ExitEventId;
  Status.FullStatus.Motion.AdmittedTwist = LingTuSim::FControlStatusVelocity{};
  Status.FullStatus.Motion.AdmittedTwist.bAvailable = true;
  Status.FullStatus.Motion.RequestedAxes = LingTuSim::FControlStatusRequestedAxes{};
  Status.FullStatus.Runtime.RuntimeState = LingTuSim::EControlStatusRuntimeState::Stopped;
  Status.FullStatus.Runtime.ControlOwner = TEXT("unavailable");
  Status.FullStatus.Runtime.bDeadman = false;
  Status.FullStatus.Runtime.SafeStopState = LingTuSim::EControlSafeStopState::Zeroed;
  FString Blocker;

  Status.FullStatus.Status = LingTuSim::EControlAckStatus::Accepted;
  TestFalse(TEXT("accepted but unconfirmed exit never closes UE"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                      Blocker));
  Status.FullStatus.Status = LingTuSim::EControlAckStatus::Rejected;
  TestFalse(TEXT("rejected exit never closes UE"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                      Blocker));
  Status.FullStatus.Status = LingTuSim::EControlAckStatus::Confirmed;
  TestFalse(TEXT("a different successful event cannot close UE"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(
                TEXT("boot-qualification:2:30"), false, Status, Blocker));
  Status.FullStatus.IntentDatagramSha256 = FString::ChrN(63, TEXT('a'));
  TestFalse(TEXT("an invalid successful-send digest cannot close UE"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                      Blocker));
  Status.FullStatus.IntentDatagramSha256 = FString::ChrN(64, TEXT('a'));
  Status.FullStatus.SourceSequence = 0;
  TestFalse(TEXT("a missing source sequence cannot close UE"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                      Blocker));
  Status.FullStatus.SourceSequence = 29;
  Status.FullStatus.Runtime.RuntimeState = LingTuSim::EControlStatusRuntimeState::Running;
  TestFalse(TEXT("confirmed event still waits for STOPPED"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                      Blocker));
  Status.FullStatus.Runtime.RuntimeState = LingTuSim::EControlStatusRuntimeState::Stopped;
  Status.FullStatus.Motion.AdmittedTwist.LinearX = 0.000001;
  TestFalse(TEXT("nonzero admitted twist blocks normal exit"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                      Blocker));
  Status.FullStatus.Motion.AdmittedTwist.LinearX = 0.0;
  TestTrue(TEXT("exact correlated stopped zero confirmation allows one normal exit"),
           LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, false, Status,
                                                                     Blocker));
  TestFalse(TEXT("the same confirmation cannot request exit twice"),
            LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(ExitEventId, true, Status,
                                                                      Blocker));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIGameSelectionModelTest,
                                 "LingTuSim.UI.Runtime.GameSelectionModel",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIGameSelectionModelTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FGameSelectionOption Runnable;
  Runnable.Id = TEXT("factory_inspection");
  Runnable.Title = TEXT("Factory inspection");
  Runnable.Order = 10;
  Runnable.Availability = LingTuSim::UI::EGameSelectionAvailability::Runnable;
  Runnable.Robot = {TEXT("thunderv4"), TEXT("1.0.3"), TEXT("Thunder V4")};
  Runnable.World = {TEXT("factory_park_hf"), TEXT("1.1.0"), TEXT("Factory Park")};
  Runnable.Mode = TEXT("inspection");
  Runnable.BundleDirectory = TEXT("C:/compiled/factory");
  Runnable.SessionId = TEXT("session-a");
  Runnable.BundleArtifacts = {{TEXT("physics.plan.json")}};

  LingTuSim::UI::FGameSelectionOption Preview = Runnable;
  Preview.Id = TEXT("forest_patrol_preview");
  Preview.Title = TEXT("Forest patrol preview");
  Preview.Order = 20;
  Preview.Availability = LingTuSim::UI::EGameSelectionAvailability::PreviewOnly;
  Preview.AvailabilityReason = TEXT("forest production route is not qualified");
  Preview.BundleDirectory.Reset();
  Preview.SessionId.Reset();
  Preview.BundleArtifacts.Reset();

  LingTuSim::UI::FGameSelectionOption Quarantined = Preview;
  Quarantined.Id = TEXT("tripo_asset_review");
  Quarantined.Title = TEXT("Tripo asset review");
  Quarantined.Order = 30;
  Quarantined.Availability = LingTuSim::UI::EGameSelectionAvailability::Quarantined;
  Quarantined.AvailabilityReason = TEXT("license and UE import are unverified");

  LingTuSim::UI::FGameSelectionCatalog Catalog;
  Catalog.Title = TEXT("LingTu Simulation");
  Catalog.Options = {Runnable, Preview, Quarantined};
  LingTuSim::UI::FGameSelectionModel Model;
  Model.SetCatalog(MoveTemp(Catalog));

  TestEqual(TEXT("first option is initially selected"), Model.GetSelectedOption()->Id, Runnable.Id);
  FString Blocker;
  TestTrue(TEXT("validated runnable bundle can be confirmed"), Model.CanConfirm(Blocker));

  TestTrue(TEXT("next selection is available"), Model.SelectNext());
  TestEqual(TEXT("next option is preview"), Model.GetSelectedOption()->Id, Preview.Id);
  TestFalse(TEXT("preview-only option cannot be confirmed"), Model.CanConfirm(Blocker));
  TestEqual(TEXT("preview blocker is preserved"), Blocker, Preview.AvailabilityReason);
  TestTrue(TEXT("next selection reaches quarantine"), Model.SelectNext());
  TestFalse(TEXT("quarantined option cannot be confirmed"), Model.MarkConfirmed(Blocker));
  TestTrue(TEXT("selection wraps"), Model.SelectNext());
  TestEqual(TEXT("wrapped option is runnable"), Model.GetSelectedOption()->Id, Runnable.Id);
  TestTrue(TEXT("previous selection also wraps"), Model.SelectPrevious());
  TestEqual(TEXT("previous wrapped option is quarantine"), Model.GetSelectedOption()->Id,
            Quarantined.Id);
  TestTrue(TEXT("direct selection finds a known id"), Model.SelectById(Runnable.Id));
  TestFalse(TEXT("direct selection rejects an unknown id"), Model.SelectById(TEXT("unknown")));
  TestTrue(TEXT("confirmation records only the selected option id"), Model.MarkConfirmed(Blocker));
  TestEqual(TEXT("confirmed option is explicit"), Model.GetConfirmedOptionId(), Runnable.Id);
  TestFalse(TEXT("next is locked after commit"), Model.SelectNext());
  TestFalse(TEXT("previous is locked after commit"), Model.SelectPrevious());
  TestFalse(TEXT("direct selection is locked after commit"), Model.SelectById(Preview.Id));
  TestFalse(TEXT("confirm is locked after commit"), Model.CanConfirm(Blocker));
  TestEqual(TEXT("post-commit blocker is explicit"), Blocker,
            FString(TEXT("game selection has already been committed")));
  TestFalse(TEXT("second commit is rejected"), Model.MarkConfirmed(Blocker));
  TestEqual(TEXT("committed selection remains unchanged"), Model.GetSelectedOption()->Id,
            Runnable.Id);
  return true;
}

namespace {
void SetRunnableLoginTestCatalog(LingTuSim::UI::FGameSelectionModel &Model) {
  LingTuSim::UI::FGameSelectionOption Runnable;
  Runnable.Id = TEXT("forest_patrol");
  Runnable.Title = TEXT("Forest patrol");
  Runnable.Availability = LingTuSim::UI::EGameSelectionAvailability::Runnable;
  Runnable.Robot = {TEXT("thunderv4"), TEXT("1.0.3"), TEXT("Thunder V4")};
  Runnable.World = {TEXT("forest_hf_2km"), TEXT("1.0.0"), TEXT("Forest")};
  Runnable.BundleDirectory = TEXT("C:/compiled/forest");
  Runnable.SessionId = TEXT("session-a");
  Runnable.BundleArtifacts = {{TEXT("physics.plan.json")}};
  LingTuSim::UI::FGameSelectionCatalog Catalog;
  Catalog.Options = {MoveTemp(Runnable)};
  Model.SetCatalog(MoveTemp(Catalog));
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginDefaultsLoggedOutTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.DefaultsLoggedOut",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginDefaultsLoggedOutTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FFrontEndLoginModel Login;
  TestFalse(TEXT("the front end starts logged out"), Login.IsLoggedIn());
  TestEqual(TEXT("the default method is explicit"), Login.GetMethod(),
            LingTuSim::UI::EFrontEndLoginMethod::None);
  TestTrue(TEXT("no operator identity is fabricated"), Login.GetDisplayName().IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginRejectsBlankOperatorTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.RejectsBlankOperator",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginRejectsBlankOperatorTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FFrontEndLoginModel Login;
  FString Error;
  TestFalse(TEXT("a whitespace-only local operator is rejected"),
            Login.SubmitLocalOperator(TEXT(" \t\r\n "), Error));
  TestFalse(TEXT("a rejected operator does not log in"), Login.IsLoggedIn());
  TestFalse(TEXT("the rejection explains the blocker"), Error.IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginAcceptsLocalOperatorTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.AcceptsLocalOperator",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginAcceptsLocalOperatorTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FFrontEndLoginModel Login;
  FString Error;
  TestTrue(TEXT("a named local operator can enter the front end"),
           Login.SubmitLocalOperator(TEXT("  Field Operator  "), Error));
  TestTrue(TEXT("local entry establishes an in-memory identity"), Login.IsLoggedIn());
  TestEqual(TEXT("local entry trims the display name"), Login.GetDisplayName(),
            FString(TEXT("Field Operator")));
  TestEqual(TEXT("local entry records the honest method"), Login.GetMethod(),
            LingTuSim::UI::EFrontEndLoginMethod::LocalOperator);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginValidatesDisplayNameBoundaryTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.ValidatesDisplayNameBoundary",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginValidatesDisplayNameBoundaryTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  FString Error;
  LingTuSim::UI::FFrontEndLoginModel BoundaryLogin;
  TestTrue(TEXT("a 64-character display name is accepted"),
           BoundaryLogin.SubmitLocalOperator(FString::ChrN(64, TEXT('A')), Error));

  LingTuSim::UI::FFrontEndLoginModel OversizedLogin;
  TestFalse(TEXT("a 65-character display name is rejected"),
            OversizedLogin.SubmitLocalOperator(FString::ChrN(65, TEXT('B')), Error));
  TestFalse(TEXT("an oversized display name does not establish identity"),
            OversizedLogin.IsLoggedIn());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginRejectsControlCharactersTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.RejectsControlCharacters",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginRejectsControlCharactersTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FString Error;
  LingTuSim::UI::FFrontEndLoginModel NewlineLogin;
  TestFalse(TEXT("an embedded newline is rejected"),
            NewlineLogin.SubmitLocalOperator(TEXT("Field\nOperator"), Error));

  LingTuSim::UI::FFrontEndLoginModel DeleteLogin;
  FString DeleteName(TEXT("FieldOperator"));
  DeleteName.AppendChar(static_cast<TCHAR>(0x7f));
  TestFalse(TEXT("the DEL control character is rejected"),
            DeleteLogin.SubmitLocalOperator(DeleteName, Error));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuRuntimeUIFrontEndLoginFailsClosedWithoutSessionBindingTest,
    "LingTuSim.UI.Runtime.FrontEndLogin.FailsClosedWithoutSessionBinding",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginFailsClosedWithoutSessionBindingTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  FString Error;
  LingTuSim::UI::FFrontEndLoginModel Login;
  TestTrue(TEXT("local identity may be established before binding"),
           Login.SubmitLocalOperator(TEXT("Operator 01"), Error));
  TestFalse(TEXT("session confirmation fails closed before binding"),
            Login.CanConfirmSession(Error));
  TestFalse(TEXT("the missing binding reason is explicit"), Error.IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuRuntimeUIFrontEndLoginKeepsIdentityAfterRejectedEditTest,
    "LingTuSim.UI.Runtime.FrontEndLogin.KeepsIdentityAfterRejectedEdit",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginKeepsIdentityAfterRejectedEditTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  FString Error;
  LingTuSim::UI::FFrontEndLoginModel Login;
  TestTrue(TEXT("the original local identity is accepted"),
           Login.SubmitLocalOperator(TEXT("Operator 01"), Error));
  TestFalse(TEXT("an invalid replacement is rejected"),
            Login.SubmitLocalOperator(FString::ChrN(65, TEXT('X')), Error));
  TestTrue(TEXT("the original identity remains logged in"), Login.IsLoggedIn());
  TestEqual(TEXT("the original identity is preserved"), Login.GetDisplayName(),
            FString(TEXT("Operator 01")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginOnlineFailsClosedTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.OnlineFailsClosed",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginOnlineFailsClosedTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FFrontEndLoginModel Login;
  FString Error;
  TestFalse(TEXT("unconfigured Online Account cannot authenticate"),
            Login.SubmitOnlineAccount(Error));
  TestFalse(TEXT("failed online entry stays logged out"), Login.IsLoggedIn());
  TestEqual(TEXT("the surfaced online blocker is the returned blocker"), Error,
            Login.GetOnlineUnavailableReason());
  TestFalse(TEXT("Online Account never becomes a claimed method"),
            Login.GetMethod() == LingTuSim::UI::EFrontEndLoginMethod::OnlineAccount);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginGatesSessionConfirmationTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.GatesSessionConfirmation",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginGatesSessionConfirmationTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FGameSelectionModel Session;
  SetRunnableLoginTestCatalog(Session);
  LingTuSim::UI::FFrontEndLoginModel Login;
  Login.BindSessionModel(Session);
  FString Error;
  TestFalse(TEXT("a runnable session cannot be confirmed while logged out"),
            Login.CanConfirmSession(Error));
  TestFalse(TEXT("the logged-out confirmation blocker is explicit"), Error.IsEmpty());
  TestTrue(TEXT("local operator entry succeeds"),
           Login.SubmitLocalOperator(TEXT("Operator 01"), Error));
  TestTrue(TEXT("the same runnable session is admissible after local entry"),
           Login.CanConfirmSession(Error));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginLogoutBeforeCommitTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.LogoutBeforeCommit",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginLogoutBeforeCommitTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FGameSelectionModel Session;
  SetRunnableLoginTestCatalog(Session);
  LingTuSim::UI::FFrontEndLoginModel Login;
  Login.BindSessionModel(Session);
  FString Error;
  TestTrue(TEXT("local operator entry succeeds"),
           Login.SubmitLocalOperator(TEXT("Operator 01"), Error));
  TestTrue(TEXT("logout remains available before session commit"), Login.Logout(Error));
  TestFalse(TEXT("logout clears the in-memory identity"), Login.IsLoggedIn());
  TestTrue(TEXT("logout clears the display name"), Login.GetDisplayName().IsEmpty());
  TestTrue(TEXT("the uncommitted login gate remains editable"), Login.CanEdit());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndLoginFreezesAfterCommitTest,
                                 "LingTuSim.UI.Runtime.FrontEndLogin.FreezesAfterCommit",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndLoginFreezesAfterCommitTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FGameSelectionModel Session;
  SetRunnableLoginTestCatalog(Session);
  LingTuSim::UI::FFrontEndLoginModel Login;
  Login.BindSessionModel(Session);
  FString Error;
  TestTrue(TEXT("local operator entry succeeds"),
           Login.SubmitLocalOperator(TEXT("Operator 01"), Error));
  TestTrue(TEXT("the admitted session commits through its authority model"),
           Login.CanConfirmSession(Error) && Session.MarkConfirmed(Error));
  TestTrue(TEXT("session commit freezes login state"), Login.IsFrozen());
  TestFalse(TEXT("logout is rejected after commit"), Login.Logout(Error));
  TestTrue(TEXT("the committed operator identity is retained"), Login.IsLoggedIn());
  TestEqual(TEXT("the committed operator name is retained"), Login.GetDisplayName(),
            FString(TEXT("Operator 01")));
  TestFalse(TEXT("operator replacement is rejected after commit"),
            Login.SubmitLocalOperator(TEXT("Operator 02"), Error));
  TestFalse(TEXT("online entry is also rejected after commit"), Login.SubmitOnlineAccount(Error));
  TestFalse(TEXT("the committed gate is no longer editable"), Login.CanEdit());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIAssetReviewModelTest,
                                 "LingTuSim.UI.Runtime.AssetReviewModel",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIAssetReviewModelTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const FString Cards = AssetReviewCardJson(TEXT("alpha"), 1) + TEXT(",") +
                        AssetReviewCardJson(TEXT("beta"), 2) + TEXT(",") +
                        AssetReviewCardJson(TEXT("gamma"), 3);
  const TSharedPtr<FJsonObject> Root =
      ParseJsonObjectForAssetReviewTest(AvailableAssetReviewSelectionJson(Cards));
  LingTuSim::UI::FAssetReviewCatalog Catalog;
  FString Error;
  TestTrue(TEXT("strict embedded review catalog parses"),
           LingTuSim::UI::FAssetReviewCatalogLoader::ParseEmbedded(Root, Catalog, Error));
  TestTrue(TEXT("available state remains explicit"), Catalog.bAvailable);
  TestEqual(TEXT("producer ordering is preserved"), Catalog.Cards.Num(), 3);
  TestEqual(TEXT("first ordered card is alpha"), Catalog.Cards[0].Id, FString(TEXT("alpha")));

  LingTuSim::UI::FAssetReviewModel Model;
  Model.SetCatalog(MoveTemp(Catalog));
  TestEqual(TEXT("first card is initially current"), Model.GetCurrentCard()->Id,
            FString(TEXT("alpha")));
  TestTrue(TEXT("next browses deterministically"), Model.SelectNext());
  TestEqual(TEXT("next card is beta"), Model.GetCurrentCard()->Id, FString(TEXT("beta")));
  TestTrue(TEXT("direct card browse succeeds"), Model.SelectById(TEXT("gamma")));
  TestTrue(TEXT("next wraps to the first card"), Model.SelectNext());
  TestEqual(TEXT("wrapped card is alpha"), Model.GetCurrentCard()->Id, FString(TEXT("alpha")));
  TestTrue(TEXT("previous wraps to the final card"), Model.SelectPrevious());
  TestEqual(TEXT("previous wrapped card is gamma"), Model.GetCurrentCard()->Id,
            FString(TEXT("gamma")));
  TestFalse(TEXT("unknown card id is rejected"), Model.SelectById(TEXT("missing")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIAssetReviewUnavailableAndInvalidTest,
                                 "LingTuSim.UI.Runtime.AssetReviewUnavailableAndInvalid",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIAssetReviewUnavailableAndInvalidTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FAssetReviewCatalog Catalog;
  FString Error;
  const TSharedPtr<FJsonObject> UnavailableRoot = ParseJsonObjectForAssetReviewTest(
      TEXT("{\"asset_review\":{\"availability\":{\"state\":\"unavailable\","
           "\"reason\":\"asset review catalog not provided\"},\"catalog\":null}}"));
  TestTrue(
      TEXT("explicit unavailable wrapper parses"),
      LingTuSim::UI::FAssetReviewCatalogLoader::ParseEmbedded(UnavailableRoot, Catalog, Error));
  TestFalse(TEXT("unavailable state is retained"), Catalog.bAvailable);
  TestTrue(TEXT("unavailable catalog has no cards"), Catalog.Cards.IsEmpty());
  LingTuSim::UI::FAssetReviewModel EmptyModel;
  EmptyModel.SetCatalog(MoveTemp(Catalog));
  TestNull(TEXT("unavailable model has no current card"), EmptyModel.GetCurrentCard());
  TestFalse(TEXT("unavailable model cannot browse next"), EmptyModel.SelectNext());
  TestFalse(TEXT("unavailable model cannot browse previous"), EmptyModel.SelectPrevious());

  const TSharedPtr<FJsonObject> EmptyAvailableRoot =
      ParseJsonObjectForAssetReviewTest(AvailableAssetReviewSelectionJson(TEXT("")));
  TestFalse(
      TEXT("available catalog cannot silently contain zero cards"),
      LingTuSim::UI::FAssetReviewCatalogLoader::ParseEmbedded(EmptyAvailableRoot, Catalog, Error));

  FString InvalidDisposition =
      AvailableAssetReviewSelectionJson(AssetReviewCardJson(TEXT("bad"), 0));
  InvalidDisposition.ReplaceInline(TEXT("\"disposition\":\"unavailable\""),
                                   TEXT("\"disposition\":\"approved\""));
  TestFalse(TEXT("unknown disposition fails closed"),
            LingTuSim::UI::FAssetReviewCatalogLoader::ParseEmbedded(
                ParseJsonObjectForAssetReviewTest(InvalidDisposition), Catalog, Error));

  const TSharedPtr<FJsonObject> InvalidStateRoot = ParseJsonObjectForAssetReviewTest(
      TEXT("{\"asset_review\":{\"availability\":{\"state\":\"partial\","
           "\"reason\":\"unsupported\"},\"catalog\":null}}"));
  TestFalse(
      TEXT("unknown wrapper availability fails closed"),
      LingTuSim::UI::FAssetReviewCatalogLoader::ParseEmbedded(InvalidStateRoot, Catalog, Error));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIAssetReviewSessionIntentIsolationTest,
                                 "LingTuSim.UI.Runtime.AssetReviewSessionIntentIsolation",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIAssetReviewSessionIntentIsolationTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FGameSelectionOption Runnable;
  Runnable.Id = TEXT("factory_inspection");
  Runnable.Title = TEXT("Factory inspection");
  Runnable.Availability = LingTuSim::UI::EGameSelectionAvailability::Runnable;
  Runnable.Robot = {TEXT("thunderv4"), TEXT("1.0.3"), TEXT("Thunder V4")};
  Runnable.World = {TEXT("factory_park_hf"), TEXT("1.1.0"), TEXT("Factory Park")};
  Runnable.BundleDirectory = TEXT("C:/compiled/factory");
  Runnable.SessionId = TEXT("session-a");
  Runnable.BundleArtifacts = {{TEXT("physics.plan.json")}};
  LingTuSim::UI::FGameSelectionCatalog GameCatalog;
  GameCatalog.Options = {Runnable};
  LingTuSim::UI::FGameSelectionModel SessionModel;
  SessionModel.SetCatalog(MoveTemp(GameCatalog));

  const FString Cards =
      AssetReviewCardJson(TEXT("alpha"), 1) + TEXT(",") + AssetReviewCardJson(TEXT("beta"), 2);
  LingTuSim::UI::FAssetReviewCatalog ReviewCatalog;
  FString Error;
  TestTrue(TEXT("review catalog parses for isolation test"),
           LingTuSim::UI::FAssetReviewCatalogLoader::ParseEmbedded(
               ParseJsonObjectForAssetReviewTest(AvailableAssetReviewSelectionJson(Cards)),
               ReviewCatalog, Error));
  LingTuSim::UI::FAssetReviewModel ReviewModel;
  ReviewModel.SetCatalog(MoveTemp(ReviewCatalog));
  ReviewModel.BindSessionModel(SessionModel);

  const FString OriginalOptionId = SessionModel.GetSelectedOption()->Id;
  const FString OriginalSessionId = SessionModel.GetSelectedOption()->SessionId;
  TestTrue(TEXT("review browsing works before session commit"), ReviewModel.SelectNext());
  TestEqual(TEXT("review browsing never changes the session selection"),
            SessionModel.GetSelectedOption()->Id, OriginalOptionId);
  TestEqual(TEXT("review browsing never changes the handoff session id"),
            SessionModel.GetSelectedOption()->SessionId, OriginalSessionId);
  TestTrue(TEXT("session selection can commit independently"), SessionModel.MarkConfirmed(Error));
  TestTrue(TEXT("review browsing observes the committed freeze"), ReviewModel.IsBrowsingFrozen());
  const FString FrozenCardId = ReviewModel.GetCurrentCard()->Id;
  TestFalse(TEXT("next is frozen after session commit"), ReviewModel.SelectNext());
  TestFalse(TEXT("previous is frozen after session commit"), ReviewModel.SelectPrevious());
  TestFalse(TEXT("direct browse is frozen after session commit"),
            ReviewModel.SelectById(TEXT("alpha")));
  TestEqual(TEXT("frozen review card remains unchanged"), ReviewModel.GetCurrentCard()->Id,
            FrozenCardId);
  TestEqual(TEXT("confirmed session intent remains unchanged"), SessionModel.GetConfirmedOptionId(),
            OriginalOptionId);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIGameSelectionCatalogContractTest,
                                 "LingTuSim.UI.Runtime.GameSelectionCatalogContract",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIGameSelectionCatalogContractTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const FString SessionId = TEXT("session-a");
  const FString Root =
      FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"), TEXT("LingTuSimUI"),
                      FGuid::NewGuid().ToString(EGuidFormats::Digits));
  const FString BundleDirectory = FPaths::Combine(Root, TEXT("bundles"), TEXT("factory"));
  TestTrue(TEXT("creates catalog test bundle directory"),
           IFileManager::Get().MakeDirectory(*BundleDirectory, true));

  const TMap<FString, FString> ArtifactJson = {
      {TEXT("physics.plan.json"),
       FString::Printf(
           TEXT("{\"schema\":\"lingtu.sim.physics-plan.v1\",\"session_id\":\"%s\"}"),
           *SessionId)},
      {TEXT("visual.plan.json"),
       FString::Printf(TEXT("{\"schema\":\"lingtu.sim.visual-plan.v1\",\"session_id\":\"%s\","
                            "\"world\":{\"package\":{\"id\":\"world\",\"version\":\"1.0.0\"}},"
                            "\"robots\":[{\"package\":{\"id\":\"robot\",\"version\":\"1.0.0\"}}]}"),
                       *SessionId)},
      {TEXT("sensor.plan.json"),
       FString::Printf(TEXT("{\"schema\":\"lingtu.sim.sensor-plan.v1\",\"session_id\":\"%s\"}"),
                       *SessionId)},
      {TEXT("control.plan.json"),
       FString::Printf(
           TEXT("{\"schema\":\"lingtu.sim.control-plan.v1\",\"session_id\":\"%s\"}"),
           *SessionId)},
      {TEXT("transport.intent.json"),
       FString::Printf(
           TEXT("{\"schema\":\"lingtu.sim.transport-intent.v1\",\"session_id\":\"%s\"}"),
           *SessionId)},
  };
  TArray<FString> ArtifactNames;
  ArtifactJson.GetKeys(ArtifactNames);
  ArtifactNames.Sort();
  FString ArtifactDescriptors;
  for (const FString &Name : ArtifactNames) {
    const FString Path = FPaths::Combine(BundleDirectory, Name);
    TestTrue(*FString::Printf(TEXT("writes %s"), *Name),
             FFileHelper::SaveStringToFile(ArtifactJson[Name], *Path,
                                           FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    ArtifactDescriptors += ArtifactDescriptors.IsEmpty() ? TEXT("") : TEXT(",");
    ArtifactDescriptors += FString::Printf(TEXT("{\"path\":\"%s\"}"), *Name);
  }

  const FString CatalogPath = FPaths::Combine(Root, TEXT("catalog.json"));
  const FString CatalogJson = FString::Printf(
      TEXT(
          "{\"schema\":\"lingtu.sim.game-selection-catalog.v1\","
          "\"title\":\"Test\","
          "\"asset_summary\":{\"catalog_package_count\":0,\"source_candidate_count\":0,"
          "\"quarantined_count\":0,\"unverified_count\":0,"
          "\"availability\":{\"state\":\"unavailable\",\"reason\":\"asset library not provided\"}},"
          "\"asset_review\":{\"availability\":{\"state\":\"unavailable\","
          "\"reason\":\"asset review catalog not provided\"},\"catalog\":null},"
          "\"entries\":[{\"id\":\"factory\",\"title\":\"Factory\",\"description\":\"Test\","
          "\"order\":1,\"mode\":\"unreal\",\"tags\":[],"
          "\"availability\":{\"state\":\"runnable\",\"reason\":\"validated\"},"
          "\"robot\":{\"id\":\"robot\",\"version\":\"1.0.0\",\"label\":\"Robot\"},"
          "\"world\":{\"id\":\"world\",\"version\":\"1.0.0\",\"label\":\"World\"},"
          "\"scenario\":null,\"bundle\":{\"directory\":\"bundles/factory\","
          "\"session_id\":\"%s\",\"artifacts\":[%s]}}]}"),
      *SessionId, *ArtifactDescriptors);
  TestTrue(TEXT("writes catalog"),
           FFileHelper::SaveStringToFile(CatalogJson, *CatalogPath,
                                         FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));

  LingTuSim::UI::FGameSelectionCatalog Catalog;
  FString Error;
  TestTrue(TEXT("path-only catalog and bundle load"),
           LingTuSim::UI::FGameSelectionCatalogLoader::LoadFromFile(CatalogPath, Catalog, Error));
  TestFalse(TEXT("unavailable asset library is explicit"), Catalog.AssetSummary.bLibraryAvailable);

  const FString ControlPath = FPaths::Combine(BundleDirectory, TEXT("control.plan.json"));
  TestTrue(TEXT("writes mismatched plan session id"),
           FFileHelper::SaveStringToFile(
               TEXT("{\"schema\":\"lingtu.sim.control-plan.v1\",\"session_id\":\"other\"}"),
                                         *ControlPath,
                                         FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
  TestFalse(TEXT("bundle session id mismatch is rejected"),
            LingTuSim::UI::FGameSelectionCatalogLoader::LoadFromFile(CatalogPath, Catalog, Error));

  TestTrue(TEXT("removes a described bundle artifact"), IFileManager::Get().Delete(*ControlPath));
  TestFalse(TEXT("missing described bundle artifact is rejected"),
            LingTuSim::UI::FGameSelectionCatalogLoader::LoadFromFile(CatalogPath, Catalog, Error));

  IFileManager::Get().DeleteDirectory(*Root, false, true);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIGameSelectionAvailabilityNamesTest,
                                 "LingTuSim.UI.Runtime.GameSelectionAvailabilityNames",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIGameSelectionAvailabilityNamesTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  TestEqual(TEXT("runnable label"),
            FString(LingTuSim::UI::GameSelectionAvailabilityName(
                LingTuSim::UI::EGameSelectionAvailability::Runnable)),
            FString(TEXT("RUNNABLE")));
  TestEqual(TEXT("preview label"),
            FString(LingTuSim::UI::GameSelectionAvailabilityName(
                LingTuSim::UI::EGameSelectionAvailability::PreviewOnly)),
            FString(TEXT("PREVIEW ONLY")));
  TestEqual(TEXT("quarantine label"),
            FString(LingTuSim::UI::GameSelectionAvailabilityName(
                LingTuSim::UI::EGameSelectionAvailability::Quarantined)),
            FString(TEXT("QUARANTINED")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIBlueprintProjectionPreservesAuthorityLayersTest,
                                 "LingTuSim.UI.Blueprint.ProjectionPreservesAuthorityLayers",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIBlueprintProjectionPreservesAuthorityLayersTest::RunTest(
    const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status = MakeQualificationReadyStatus();
  Status.SessionState = TEXT("Bound");
  Status.ControlState = TEXT("Bound");
  Status.Blocker.Reset();
  Status.BodyBindingCount = 21;
  Status.ScenarioActorCount = 3;
  Status.bControlAckAvailable = true;
  Status.bControlAckAccepted = true;
  Status.ControlAckState = TEXT("Accepted");
  Status.FullStatus.Runtime.RuntimeState = LingTuSim::EControlStatusRuntimeState::Running;
  Status.FullStatus.Runtime.ControlOwner = TEXT("ue-operator");
  Status.FullStatus.Runtime.bDeadman = true;
  Status.FullStatus.Runtime.SafeStopState = LingTuSim::EControlSafeStopState::Clear;
  Status.FullStatus.Motion.RequestedAxes.Forward = 0.8;
  Status.FullStatus.Motion.AdmittedTwist.LinearX = 0.5;
  Status.FullStatus.Motion.AdmittedTwist.LinearY = -0.1;
  Status.FullStatus.Motion.AdmittedTwist.AngularZ = 0.2;
  Status.ObservedBaseStableId = TEXT("thunderv4/base_link");
  Status.ObservedBaseLinearVelocityMps = FVector(0.49, -0.08, 0.01);
  Status.ObservedBaseAngularVelocityRadps = FVector(0.0, 0.0, 0.19);
  Status.FullStatus.Recording.State = LingTuSim::EControlRecordingState::Recording;

  const FLingTuSimBlueprintRuntimeStatus View =
      LingTuSim::UI::FBlueprintRuntimeStatusProjection::Project(Status);
  TestTrue(TEXT("coherent current truth is presentation-ready"), View.bPresentationReady);
  TestEqual(TEXT("all visual body bindings are projected"), View.BodyBindingCount, 21);
  TestEqual(TEXT("runtime ownership remains explicit"), View.ControlOwner,
            FString(TEXT("ue-operator")));
  TestEqual(TEXT("requested operator axes remain distinct"), View.RequestedForward, 0.8);
  TestEqual(TEXT("admitted control velocity remains distinct"), View.AdmittedLinearVelocityMps.X,
            0.5);
  TestEqual(TEXT("observed MuJoCo truth remains distinct"), View.ObservedBaseLinearVelocityMps.X,
            0.49);
  TestEqual(TEXT("runtime state has a Blueprint-safe label"), View.RuntimeState,
            FString(TEXT("Running")));
  TestEqual(TEXT("recording lifecycle has a Blueprint-safe label"), View.RecordingState,
            FString(TEXT("Recording")));

  const FLingTuSimBlueprintRuntimeStatus SameView =
      LingTuSim::UI::FBlueprintRuntimeStatusProjection::Project(Status);
  TestTrue(TEXT("identical projections do not create duplicate change events"), View == SameView);
  Status.TruthSequence += 1;
  const FLingTuSimBlueprintRuntimeStatus NextTruthView =
      LingTuSim::UI::FBlueprintRuntimeStatusProjection::Project(Status);
  TestTrue(TEXT("a new truth sample creates a Blueprint status change"), View != NextTruthView);
  return true;
}

#if !UE_BUILD_SHIPPING
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndScreenshotRunContractTest,
                                 "LingTuSim.UI.Runtime.FrontEndScreenshot.RunContract",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndScreenshotRunContractTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using LingTuSim::UI::FFrontEndScreenshotDriver;
  FString RunId;
  FString Error;
  bool bConfigured = true;
  TestTrue(TEXT("an absent development screenshot argument is accepted"),
           FFrontEndScreenshotDriver::ParseCommandLine(TEXT("-game"), RunId, bConfigured, Error));
  TestFalse(TEXT("an absent argument does not configure the driver"), bConfigured);

  TestTrue(TEXT("a safe run id configures the driver"),
           FFrontEndScreenshotDriver::ParseCommandLine(
               TEXT("-LingTuDevFrontEndScreenshots=qa.Run-01"), RunId, bConfigured, Error));
  TestTrue(TEXT("the safe argument is configured"), bConfigured);
  TestEqual(TEXT("the exact run id is retained"), RunId, FString(TEXT("qa.Run-01")));
  TestTrue(TEXT("a one-character run id is valid"),
           FFrontEndScreenshotDriver::IsSafeRunId(TEXT("A")));
  TestTrue(TEXT("the 128-character boundary is valid"),
           FFrontEndScreenshotDriver::IsSafeRunId(FString::ChrN(128, TEXT('z'))));
  TestFalse(TEXT("a run id cannot begin with punctuation"),
            FFrontEndScreenshotDriver::IsSafeRunId(TEXT(".qa")));
  TestFalse(TEXT("path separators are rejected"),
            FFrontEndScreenshotDriver::IsSafeRunId(TEXT("qa/run")));
  TestFalse(TEXT("non-ASCII alphanumeric characters are rejected"),
            FFrontEndScreenshotDriver::IsSafeRunId(TEXT("视觉QA")));
  TestFalse(TEXT("the 129-character boundary is rejected"),
            FFrontEndScreenshotDriver::IsSafeRunId(FString::ChrN(129, TEXT('z'))));
  TestFalse(TEXT("duplicate assignments fail closed"),
            FFrontEndScreenshotDriver::ParseCommandLine(
                TEXT("-LingTuDevFrontEndScreenshots=a -LingTuDevFrontEndScreenshots=b"), RunId,
                bConfigured, Error));

  const LingTuSim::UI::FFrontEndScreenshotPaths Paths =
      FFrontEndScreenshotDriver::BuildOutputPaths(TEXT("qa.Run-01"));
  const FString ExpectedDirectory = FPaths::ConvertRelativePathToFull(
      FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"), TEXT("LingTuSimUI"),
                      TEXT("FrontEnd"), TEXT("qa.Run-01")));
  TestTrue(TEXT("the run directory is fixed below Project Saved"),
           FPaths::IsSamePath(Paths.RunDirectory, ExpectedDirectory));
  TestEqual(TEXT("login screenshot has its fixed basename"),
            FPaths::GetCleanFilename(Paths.LoginScreenshot), FString(TEXT("login.png")));
  TestEqual(TEXT("asset library screenshot has its fixed basename"),
            FPaths::GetCleanFilename(Paths.AssetLibraryScreenshot),
            FString(TEXT("asset-library.png")));
  TestEqual(TEXT("success sentinel has its fixed basename"),
            FPaths::GetCleanFilename(Paths.SuccessSentinel), FString(TEXT("success.json")));
  TestEqual(TEXT("error sentinel has its fixed basename"),
            FPaths::GetCleanFilename(Paths.ErrorSentinel), FString(TEXT("error.txt")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuRuntimeUIFrontEndScreenshotPolicyTest,
                                 "LingTuSim.UI.Runtime.FrontEndScreenshot.Policy",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuRuntimeUIFrontEndScreenshotPolicyTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using LingTuSim::UI::FFrontEndScreenshotDriver;
  FString Error;
  TestTrue(TEXT("selector plus unattended plus real RHI is admitted"),
           FFrontEndScreenshotDriver::ValidateStartPolicy(true, true, true, false, Error));
  TestFalse(TEXT("ordinary runtime HUD is rejected"),
            FFrontEndScreenshotDriver::ValidateStartPolicy(false, true, true, false, Error));
  TestFalse(TEXT("interactive invocation is rejected"),
            FFrontEndScreenshotDriver::ValidateStartPolicy(true, false, true, false, Error));
  TestFalse(TEXT("a renderer-disabled process is rejected"),
            FFrontEndScreenshotDriver::ValidateStartPolicy(true, true, false, false, Error));
  TestFalse(TEXT("NullRHI is rejected explicitly"),
            FFrontEndScreenshotDriver::ValidateStartPolicy(true, true, true, true, Error));
  return true;
}
#endif

#endif
