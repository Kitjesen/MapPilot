#if WITH_DEV_AUTOMATION_TESTS
#include "LingTuSimInspectionGuidance.h"
#include "LingTuSimInspectionProjection.h"
#include "Misc/AutomationTest.h"
#include "SLingTuSimRuntimeHUD.h"
#include "LingTuSimRuntimeUIModel.h"
#include "LingTuSimSessionService.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#include "Input/Events.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

namespace {
LingTuSim::UI::FInspectionProjection Task() {
  LingTuSim::UI::FInspectionProjection T;
  T.TaskId = TEXT("task-a"); T.RouteId = TEXT("route-a"); T.MapId = TEXT("map-a");
  T.ProductSessionId = TEXT("product-a");
  T.MapContentEpoch = 3; T.RouteRevision = 7;
  T.bBound = T.bTaskAvailable = T.bLauncherBindingVerified = true;
  T.bExecutionConfirmed = T.bHistoryComplete = T.bProgressVerified = true;
  T.CurrentPointId = TEXT("dock"); T.CurrentPointNumber = 2;
  T.Phase = TEXT("NAVIGATING");
  return T;
}
FString GuidanceJson() {
  return TEXT(R"({"schema_version":"lingtu.inspection.guidance.v1","task_id":"task-a","product_session_id":"product-a",
    "runtime":{"env":"sim","product":"inspection","state":"active","product_session_id":"product-a"},
    "identity":{"task_id":"task-a","route_id":"route-a","route_revision":7,
      "map_id":"map-a","map_content_epoch":3},"available":true,"reason":"",
    "point_id":"dock","phase":"NAVIGATING",
    "goal":{"frame_id":"map","x":13,"y":24,"z":0},
    "robot_pose":{"frame_id":"map","x":10,"y":20,"z":0,"yaw":0},
    "path":{"available":false,"reason":"execution_path_unbound","points":[]}})");
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInspectionGuidanceBindingTest,
  "LingTuSim.UI.Runtime.InspectionGuidance.Binding", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FInspectionGuidanceBindingTest::RunTest(const FString&) {
  auto T = Task(); FString Error;
  TestTrue(TEXT("guidance binds to exact task, route and current point"),
    LingTuSim::UI::FInspectionGuidance::Parse(GuidanceJson(), T, T.Guidance, Error));
  T.Guidance.ReceivedSeconds = 10;
  TestTrue(TEXT("fresh native target is displayable"), T.Guidance.IsCurrent(T, 11));
  TestEqual(TEXT("distance is map-frame straight distance, not invented path length"), T.Guidance.DistanceMeters(), 5.0);
  TestTrue(TEXT("relative bearing uses localization yaw"), FMath::IsNearlyEqual(T.Guidance.BearingDegrees(), 53.130102, .001));
  TestFalse(TEXT("old guidance is hidden"), T.Guidance.IsCurrent(T, 14));
  T.CurrentPointId = TEXT("next");
  TestFalse(TEXT("advancing native point invalidates previous target"), T.Guidance.IsCurrent(T, 11));
  T = Task();
  TestFalse(TEXT("another route revision is rejected"), LingTuSim::UI::FInspectionGuidance::Parse(
    GuidanceJson().Replace(TEXT("\"route_revision\":7"), TEXT("\"route_revision\":8")), T, T.Guidance, Error));
  TestFalse(TEXT("another frame is rejected"), LingTuSim::UI::FInspectionGuidance::Parse(
    GuidanceJson().Replace(TEXT("\"frame_id\":\"map\""), TEXT("\"frame_id\":\"odom\"")), T, T.Guidance, Error));
  TestTrue(TEXT("real Gateway remains diagnosable"), LingTuSim::UI::FInspectionGuidance::Parse(
    GuidanceJson().Replace(TEXT("\"env\":\"sim\""), TEXT("\"env\":\"real\"")), T, T.Guidance, Error));
  TestFalse(TEXT("simulation UI never guides or controls a real task"), T.Guidance.bSimulationRuntime);
  auto OldSession = Task(); OldSession.ProductSessionId = TEXT("previous-product");
  TestTrue(TEXT("Product rollover remains diagnosable"), LingTuSim::UI::FInspectionGuidance::Parse(GuidanceJson(), OldSession, OldSession.Guidance, Error));
  TestFalse(TEXT("same task/map cannot bind a new Product session"), OldSession.Guidance.bSimulationRuntime);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInspectionGuidanceMenuTest,
  "LingTuSim.UI.Runtime.InspectionGuidance.MenuIntent", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FInspectionGuidanceMenuTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Mode = MakeShared<FRuntimeUIModeController>(); Mode->TogglePause();
  auto Projection = MakeShared<FInspectionProjection>(Task());
  Projection->bCanPause = true;
  Projection->Guidance.bSimulationRuntime = true;
  Projection->Guidance.ProductSessionId = Projection->ProductSessionId;
  int32 Requests = 0;
  const auto HUD = SNew(SLingTuSimRuntimeHUD).ModeController(Mode)
    .LocalState(MakeShared<FRuntimeUILocalState>()).PlayerSettings(MakeShared<FPlayerSettings>())
    .LoginModel(MakeShared<FFrontEndLoginModel>()).SelectionModel(MakeShared<FGameSelectionModel>())
    .AssetReviewModel(MakeShared<FAssetReviewModel>()).InspectionProjection(Projection)
    .SelectionFeedback(MakeShared<FString>())
    .OnInspectionPause(FSimpleDelegate::CreateLambda([&Requests]() { ++Requests; }));
  HUD->SlatePrepass(1);
  TFunction<bool(TSharedRef<SWidget>)> ContainsCaption = [&](TSharedRef<SWidget> W) {
    if (W->GetType() == TEXT("STextBlock") && StaticCastSharedRef<STextBlock>(W)->GetText().ToString() == TEXT("暂停巡检")) return true;
    auto* Children = W->GetChildren();
    for (int32 I = 0; I < Children->Num(); ++I) if (ContainsCaption(Children->GetChildAt(I))) return true;
    return false;
  };
  TFunction<TSharedPtr<SButton>(TSharedRef<SWidget>)> Find = [&](TSharedRef<SWidget> W) -> TSharedPtr<SButton> {
    if (!W->GetVisibility().IsVisible()) return nullptr;
    if (W->GetType() == TEXT("SButton") && ContainsCaption(W)) return StaticCastSharedRef<SButton>(W);
    auto* Children = W->GetChildren();
    for (int32 I = 0; I < Children->Num(); ++I) if (auto B = Find(Children->GetChildAt(I))) return B;
    return nullptr;
  };
  HUD->PreviewPlayerMenuPage(TEXT("journal"));
  auto Button = Find(HUD);
  TestTrue(TEXT("bound task has an actual visible pause button"), Button.IsValid());
  if (Button.IsValid()) {
    const FKeyEvent Enter(EKeys::Enter, FModifierKeysState(), 0, false, 13, 13);
    Button->OnKeyDown(FGeometry(), Enter); Button->OnKeyUp(FGeometry(), Enter);
  }
  TestEqual(TEXT("activation routes an intent to the owner"), Requests, 1);
  TestEqual(TEXT("UI cannot turn request into native pause truth"), Projection->Phase, FString(TEXT("NAVIGATING")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInspectionGuidanceControlTest,
  "LingTuSim.UI.Runtime.InspectionGuidance.ControlAndPhases", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FInspectionGuidanceControlTest::RunTest(const FString&) {
  auto T = Task(); T.Guidance.bSimulationRuntime = true; T.Guidance.ProductSessionId = T.ProductSessionId;
  T.bCanPause = true;
  TestTrue(TEXT("pause uses native advertised action"), T.CanRequestControl(TEXT("pause")));
  TestFalse(TEXT("resume never follows from local menu state"), T.CanRequestControl(TEXT("resume")));
  T.bCommandPending = true;
  TestFalse(TEXT("pending request cannot be duplicated"), T.CanRequestControl(TEXT("pause")));
  T.bCommandPending = false; T.bCanPause = false; T.bCanResume = true; T.Phase = TEXT("PAUSED");
  TestTrue(TEXT("native paused task can be explicitly resumed"), T.CanRequestControl(TEXT("resume")));
  TestEqual(TEXT("paused presentation explains explicit resume"), LingTuSim::UI::FInspectionGuidance::PhaseText(T).ToString(), FString(TEXT("任务已暂停 · 请显式继续")));
  T.Phase = TEXT("SETTLING");
  TestEqual(TEXT("arrival is not capture success"), LingTuSim::UI::FInspectionGuidance::PhaseText(T).ToString(), FString(TEXT("等待机器人停稳")));
  T.Phase = TEXT("ACTION_PENDING");
  TestEqual(TEXT("capture stays pending until native evidence result"), LingTuSim::UI::FInspectionGuidance::PhaseText(T).ToString(), FString(TEXT("采集中 · 等待证据结果")));
  T.bTerminal = true;
  TestFalse(TEXT("terminal task cannot be resumed"), T.CanRequestControl(TEXT("resume")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInspectionGuidanceGenerationTickTest,
  "LingTuSim.UI.Runtime.InspectionGuidance.GenerationTick", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FInspectionGuidanceGenerationTickTest::RunTest(const FString&) {
  using namespace LingTuSim;
  using namespace LingTuSim::UI;
  FString PreviousSession; uint64 PreviousGeneration = 0;
  const bool bHadSession = FSessionService::GetBoundSession(PreviousSession, PreviousGeneration);
  FSessionService::UnbindSession();
  ON_SCOPE_EXIT {
    FSessionService::UnbindSession();
    if (bHadSession) FSessionService::RebindSession(PreviousSession, PreviousGeneration);
  };
  if (!TestTrue(TEXT("initial session binds"), FSessionService::RebindSession(TEXT("session-a"), 7))) return false;
  auto Projection = MakeShared<FInspectionProjection>(Task());
  const auto HUD = SNew(SLingTuSimRuntimeHUD).ModeController(MakeShared<FRuntimeUIModeController>())
    .LocalState(MakeShared<FRuntimeUILocalState>()).PlayerSettings(MakeShared<FPlayerSettings>())
    .LoginModel(MakeShared<FFrontEndLoginModel>()).SelectionModel(MakeShared<FGameSelectionModel>())
    .AssetReviewModel(MakeShared<FAssetReviewModel>()).InspectionProjection(Projection)
    .SelectionFeedback(MakeShared<FString>());
  const auto Tick = [&]() { HUD->Tick(FGeometry(), 1.0, 1.F / 30); };
  Projection->Guidance.bAvailable = true;
  Tick();
  TestTrue(TEXT("unchanged generation keeps guidance"), Projection->Guidance.bAvailable);
  FSessionService::UnbindSession();
  if (!TestTrue(TEXT("replacement session binds at the same model generation"),
                FSessionService::RebindSession(TEXT("session-b"), 7))) return false;
  Tick();
  TestFalse(TEXT("runtime session change clears guidance immediately"), Projection->Guidance.bAvailable);
  Projection->Guidance.bAvailable = true;
  if (!TestTrue(TEXT("next model generation binds"),
                FSessionService::RebindSession(TEXT("session-b"), 8))) return false;
  Tick();
  TestFalse(TEXT("runtime model generation change clears guidance immediately"), Projection->Guidance.bAvailable);
  const auto BindReset = [&](uint64 Reset) {
    FControlTransportBinding Binding;
    Binding.RunId = TEXT("run-guidance"); Binding.SessionId = TEXT("session-b");
    Binding.BootId = TEXT("boot-guidance"); Binding.SourceId = TEXT("robotsimue.local_player.0");
    Binding.ModelGeneration = 8; Binding.ResetGeneration = Reset;
    Binding.LogDirectory = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir());
    Binding.OriginEvidencePath = FPaths::Combine(Binding.LogDirectory, TEXT("ue-control-origin.jsonl"));
    Binding.IntentPort = 25124; Binding.StatusPort = 25125;
    const auto Sender = MakeShared<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe>(
      Binding, [](const TArray<uint8>&, FString&) { return true; },
      [](const FString&, FString&) { return true; });
    return FSessionService::BindControlTransport(Sender);
  };
  if (!TestTrue(TEXT("first reset binding exists"), BindReset(11))) return false;
  Tick();
  Projection->Guidance.bAvailable = true;
  Tick();
  TestTrue(TEXT("unchanged reset keeps guidance"), Projection->Guidance.bAvailable);
  if (!TestTrue(TEXT("new reset binding exists"), BindReset(12))) return false;
  Tick();
  TestFalse(TEXT("runtime reset clears guidance immediately"), Projection->Guidance.bAvailable);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInspectionGuidancePlacementTest,
  "LingTuSim.UI.Runtime.InspectionGuidance.Placement", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FInspectionGuidancePlacementTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto T = Task(); FString Error;
  FInspectionGuidance::Parse(GuidanceJson(), T, T.Guidance, Error);
  FNavigationPose Pose; Pose.bAvailable = true;
  Pose.PositionMeters = FVector(100, 50, 1);
  Pose.Rotation = FQuat(FVector::UpVector, PI / 2);
  const FVector Projected = T.Guidance.EstimatedTargetUnrealCm(Pose);
  TestTrue(TEXT("map relative target is anchored at observed base pose with handedness conversion"),
    Projected.Equals(FVector(9600, -5300, 100), .01));
  const auto Edge = FInspectionGuidance::PlaceMarker(FVector2D(2500, 400), FVector2D(1920, 1080));
  TestTrue(TEXT("offscreen marker remains in safe area"), Edge.X <= 1856 && Edge.X >= 64 && Edge.Y >= 64 && Edge.Y <= 1016);
  auto NoPose = Task(); NoPose.bStale = true;
  NoPose.Guidance = T.Guidance; NoPose.Guidance.ReceivedSeconds = 10;
  TestFalse(TEXT("stale task hides guidance regardless of cached pose"), NoPose.Guidance.IsCurrent(NoPose, 11));
  return true;
}
#endif
