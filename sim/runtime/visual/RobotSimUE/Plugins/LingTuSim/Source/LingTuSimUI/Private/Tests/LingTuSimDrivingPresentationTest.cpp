#if WITH_DEV_AUTOMATION_TESTS
#include "SLingTuSimRuntimeHUD.h"

#include "Input/Events.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Misc/AutomationTest.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"

namespace {
bool HasVisibleText(const TSharedRef<SWidget>& Widget, const FString& Text) {
  if (!Widget->GetVisibility().IsVisible()) return false;
  if (Widget->GetType() == TEXT("STextBlock") &&
      StaticCastSharedRef<STextBlock>(Widget)->GetText().ToString() == Text) {
    return true;
  }
  FChildren* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index) {
    if (HasVisibleText(Children->GetChildAt(Index), Text)) return true;
  }
  return false;
}

TSharedPtr<SButton> FindButton(const TSharedRef<SWidget>& Widget, const FString& Caption) {
  if (!Widget->GetVisibility().IsVisible()) return nullptr;
  if (Widget->GetType() == TEXT("SButton") && HasVisibleText(Widget, Caption)) {
    return StaticCastSharedRef<SButton>(Widget);
  }
  FChildren* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index) {
    if (TSharedPtr<SButton> Button = FindButton(Children->GetChildAt(Index), Caption)) return Button;
  }
  return nullptr;
}

void AcceptButton(const TSharedRef<SButton>& Button) {
  const FKeyEvent Accept(EKeys::Enter, FModifierKeysState(), 0, false, 13, 13);
  Button->OnKeyDown(FGeometry(), Accept);
  Button->OnKeyUp(FGeometry(), Accept);
}

TSharedRef<LingTuSim::UI::SLingTuSimRuntimeHUD> MakeMenuHUD(
    const TSharedRef<LingTuSim::UI::FRuntimeUIModeController>& Mode,
    const TSharedRef<LingTuSim::UI::FRuntimeUILocalState>& LocalState,
    FSimpleDelegate OnCycleCamera) {
  Mode->TogglePause();
  TSharedRef<LingTuSim::UI::SLingTuSimRuntimeHUD> HUD =
      SNew(LingTuSim::UI::SLingTuSimRuntimeHUD)
          .ModeController(Mode)
          .LocalState(LocalState)
          .LoginModel(MakeShared<LingTuSim::UI::FFrontEndLoginModel>())
          .SelectionModel(MakeShared<LingTuSim::UI::FGameSelectionModel>())
          .AssetReviewModel(MakeShared<LingTuSim::UI::FAssetReviewModel>())
          .InspectionProjection(MakeShared<LingTuSim::UI::FInspectionProjection>())
          .SelectionFeedback(MakeShared<FString>())
          .OnCycleCamera(OnCycleCamera);
  HUD->SlatePrepass(1.0F);
  return HUD;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuDrivingMenuCameraUnavailableTest,
    "LingTuSim.UI.Runtime.DrivingPresentation.CameraUnavailableNotInMenu",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuDrivingMenuCameraUnavailableTest::RunTest(const FString&) {
  const TSharedRef<LingTuSim::UI::FRuntimeUIModeController> Mode =
      MakeShared<LingTuSim::UI::FRuntimeUIModeController>();
  const TSharedRef<LingTuSim::UI::FRuntimeUILocalState> LocalState =
      MakeShared<LingTuSim::UI::FRuntimeUILocalState>();
  const TSharedRef<LingTuSim::UI::SLingTuSimRuntimeHUD> HUD =
      MakeMenuHUD(Mode, LocalState, FSimpleDelegate::CreateLambda([]() {}));
  const TSharedPtr<SButton> CameraButton = FindButton(HUD, TEXT("切换视角"));
  TestFalse(TEXT("camera belongs to driving controls, not player menu"), CameraButton.IsValid());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuDrivingMenuCameraRestoresModeTest,
    "LingTuSim.UI.Runtime.DrivingPresentation.ContinuePreservesCamera",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuDrivingMenuCameraRestoresModeTest::RunTest(const FString&) {
  const TSharedRef<LingTuSim::UI::FRuntimeUIModeController> Mode =
      MakeShared<LingTuSim::UI::FRuntimeUIModeController>();
  const TSharedRef<LingTuSim::UI::FRuntimeUILocalState> LocalState =
      MakeShared<LingTuSim::UI::FRuntimeUILocalState>();
  LocalState->ActualCameraMode = TEXT("follow");
  int32 CycleCount = 0;
  const TSharedRef<LingTuSim::UI::SLingTuSimRuntimeHUD> HUD = MakeMenuHUD(
      Mode, LocalState,
      FSimpleDelegate::CreateLambda([&CycleCount]() { ++CycleCount; }));
  TestFalse(TEXT("even an available camera has no technical menu entry"), FindButton(HUD, TEXT("切换视角")).IsValid());
  const auto Continue = FindButton(HUD, TEXT("继续游戏"));
  if (!TestTrue(TEXT("continue exists"), Continue.IsValid())) return false;
  AcceptButton(Continue.ToSharedRef());
  TestEqual(TEXT("continue restores Drive"), Mode->GetMode(),
            LingTuSim::UI::ERuntimeUIMode::Drive);
  TestEqual(TEXT("continue does not silently switch the camera"), CycleCount, 0);
  HUD->Invalidate(EInvalidateWidgetReason::Layout);
  HUD->SlatePrepass(1.0F);
  TestFalse(TEXT("camera echo diagnostics do not clutter the compact HUD"),
            HasVisibleText(HUD, TEXT("跟随视角 · 待回显")));
  TestEqual(TEXT("camera state remains unchanged despite simplified presentation"), LocalState->ActualCameraMode, FString(TEXT("follow")));
  return true;
}
#endif
