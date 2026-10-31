#if WITH_DEV_AUTOMATION_TESTS
#include "LingTuSimRuntimeUIInputProcessor.h"
#include "Framework/Application/SlateApplication.h"
#include "Framework/Application/SlateUser.h"
#include "Input/Events.h"
#include "Misc/AutomationTest.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"

namespace {
FPointerEvent Mouse(const FKey& Button = EKeys::Invalid, bool bRightHeld = false,
                    FVector2D Delta = FVector2D::ZeroVector, float Wheel = 0) {
  TSet<FKey> Pressed;
  if (bRightHeld) Pressed.Add(EKeys::RightMouseButton);
  return FPointerEvent(FSlateApplication::CursorPointerIndex, FVector2D(200, 200), FVector2D(200, 200) - Delta,
                       Pressed, Button, Wheel, FModifierKeysState());
}

struct FMouseCameraFixture {
  TSharedRef<LingTuSim::UI::FRuntimeUIModeController> Mode = MakeShared<LingTuSim::UI::FRuntimeUIModeController>();
  TSharedRef<SButton> Viewport = SNew(SButton);
  TSharedRef<SButton> OtherPanel = SNew(SButton);
  TSharedRef<SWindow> Window = SNew(SWindow).ClientSize(FVector2D(800, 600))
      [SNew(SHorizontalBox) + SHorizontalBox::Slot()[Viewport] + SHorizontalBox::Slot()[OtherPanel]];
  TSharedPtr<SWidget> PreviousFocus;
  TSharedPtr<SWidget> PreviousCaptor;
  bool bPreviouslyActive = false;
  bool bSelector = false;
  bool bTextEntry = false;
  TArray<LingTuSim::UI::FRuntimeCameraInput> Camera;
  TArray<LingTuSim::UI::ERuntimeUIAction> Actions;
  TArray<LingTuSim::UI::FRobotDriveInputSnapshot> Robot;
  TArray<LingTuSim::UI::ERobotDrivePublishReason> RobotReasons;
  TUniquePtr<LingTuSim::UI::FLingTuSimRuntimeUIInputProcessor> Processor;

  FMouseCameraFixture() {
    auto& App = FSlateApplication::Get();
    PreviousFocus = App.GetKeyboardFocusedWidget();
    PreviousCaptor = App.GetCursorUser()->GetCursorCaptor();
    bPreviouslyActive = App.IsActive();
    App.AddWindow(Window, false);
    App.OnApplicationActivationChanged(true);
    App.SetKeyboardFocus(Viewport, EFocusCause::SetDirectly);
    Capture(Viewport);
    Processor = MakeUnique<LingTuSim::UI::FLingTuSimRuntimeUIInputProcessor>(
        Mode, Viewport, Viewport,
        [this](const auto& Input, auto Reason) { Robot.Add(Input); RobotReasons.Add(Reason); },
        [this](auto Action) { Actions.Add(Action); },
        [this]() { return bSelector; }, [this]() { return bTextEntry; },
        [this](const auto& Input) { Camera.Add(Input); });
  }
  ~FMouseCameraFixture() {
    Processor.Reset();
    auto& App = FSlateApplication::Get();
    App.GetCursorUser()->ReleaseCursorCapture();
    if (PreviousCaptor.IsValid()) Capture(PreviousCaptor.ToSharedRef());
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(bPreviouslyActive);
    App.RequestDestroyWindow(Window);
  }
  void Capture(TSharedRef<SWidget> Widget) {
    auto& App = FSlateApplication::Get();
    FWidgetPath Path;
    App.GeneratePathToWidgetUnchecked(Widget, Path);
    App.GetCursorUser()->SetCursorCaptor(Widget, Path);
  }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuDrivePublishCadenceTest,
    "LingTuSim.UI.Runtime.DriveInput.PublishCadence", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FLingTuDrivePublishCadenceTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto& App = FSlateApplication::Get();
  for (const int32 FramesPerSecond : {60, 120}) {
    FMouseCameraFixture F;
    const auto Press = [&F, &App](const FKey& Key) {
      F.Processor->HandleKeyDownEvent(App, FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0));
    };
    const auto Tick = [&F, &App](float DeltaTime) {
      F.Processor->Tick(DeltaTime, App, App.GetPlatformCursor().ToSharedRef());
    };
    Press(EKeys::W);
    if (!TestTrue(TEXT("forward input publishes immediately"), !F.Robot.IsEmpty() && F.Robot.Last().bDeadman)) return false;
    F.Robot.Reset();
    F.RobotReasons.Reset();
    for (int32 Frame = 0; Frame < FramesPerSecond * 2; ++Frame) Tick(1.0f / FramesPerSecond);
    TestTrue(FString::Printf(TEXT("%d FPS preserves 50 Hz periodic publication over two seconds (%d samples)"),
                            FramesPerSecond, F.Robot.Num()), F.Robot.Num() >= 99 && F.Robot.Num() <= 100);
    for (const auto Reason : F.RobotReasons)
      TestEqual(TEXT("held input sends periodic samples"), Reason, ERobotDrivePublishReason::Periodic);

    const int32 HeldCount = F.Robot.Num();
    F.Processor->HandleKeyUpEvent(App, FKeyEvent(EKeys::W, FModifierKeysState(), 0, false, 0, 0));
    if (!TestEqual(TEXT("key release publishes before the next tick"), F.Robot.Num(), HeldCount + 1)) return false;
    TestEqual(TEXT("key release has release reason"), F.RobotReasons.Last(), ERobotDrivePublishReason::Release);
    TestTrue(TEXT("key release sends zero with deadman off"), F.Robot.Last().bRelease && !F.Robot.Last().bDeadman &&
             F.Robot.Last().Forward == 0 && F.Robot.Last().Left == 0 && F.Robot.Last().YawLeft == 0);
    Tick(0.1f);
    TestEqual(TEXT("released input sends no periodic sample"), F.Robot.Num(), HeldCount + 1);

    Press(EKeys::W);
    Tick(0.015f);
    const int32 BeforeChange = F.Robot.Num();
    Press(EKeys::A);
    if (!TestEqual(TEXT("state change publishes before the next tick"), F.Robot.Num(), BeforeChange + 1)) return false;
    TestEqual(TEXT("state change has state reason"), F.RobotReasons.Last(), ERobotDrivePublishReason::StateChanged);
    TestEqual(TEXT("state change includes lateral input"), F.Robot.Last().Left, 1.0f);
    Tick(0.01f);
    TestEqual(TEXT("state change starts a fresh periodic interval"), F.Robot.Num(), BeforeChange + 1);
    Tick(0.011f);
    TestEqual(TEXT("periodic publication resumes after state change"), F.Robot.Num(), BeforeChange + 2);
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuMouseCameraActionsTest,
    "LingTuSim.UI.Runtime.MouseCamera.Actions", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FLingTuMouseCameraActionsTest::RunTest(const FString&) {
  FMouseCameraFixture F;
  auto& App = FSlateApplication::Get();
  TestFalse(TEXT("unarmed movement is not camera look"), F.Processor->HandleMouseMoveEvent(App, Mouse(EKeys::Invalid, false, {10, -5})));
  TestTrue(TEXT("fractional wheel controls zoom"), F.Processor->HandleMouseWheelOrGestureEvent(App, Mouse(EKeys::Invalid, false, FVector2D::ZeroVector, .5f), nullptr));
  if (!TestEqual(TEXT("one zoom adjustment"), F.Camera.Num(), 1)) return false;
  TestEqual(TEXT("trackpad fraction is preserved"), F.Camera[0].ZoomSteps, .5f);
  TestTrue(TEXT("right press arms mouse look"), F.Processor->HandleMouseButtonDownEvent(App, Mouse(EKeys::RightMouseButton, true)));
  TestTrue(TEXT("right drag controls camera"), F.Processor->HandleMouseMoveEvent(App, Mouse(EKeys::Invalid, true, {10, -5})));
  if (!TestEqual(TEXT("one drag adjustment"), F.Camera.Num(), 2)) return false;
  TestTrue(FString::Printf(TEXT("horizontal delta %.8f becomes 1.6 degrees"), F.Camera[1].YawDeltaDegrees),
           FMath::IsNearlyEqual(F.Camera[1].YawDeltaDegrees, 1.6f, 1.e-4f));
  TestTrue(FString::Printf(TEXT("upward drag %.8f becomes 0.8 degrees"), F.Camera[1].PitchDeltaDegrees),
           FMath::IsNearlyEqual(F.Camera[1].PitchDeltaDegrees, .8f, 1.e-4f));
  TestTrue(TEXT("right release is consumed"), F.Processor->HandleMouseButtonUpEvent(App, Mouse(EKeys::RightMouseButton)));
  TestFalse(TEXT("release ends mouse look"), F.Processor->HandleMouseMoveEvent(App, Mouse(EKeys::Invalid, true, {10, 0})));
  TestTrue(TEXT("middle click switches the existing camera action"), F.Processor->HandleMouseButtonDownEvent(App, Mouse(EKeys::MiddleMouseButton)));
  if (TestEqual(TEXT("middle click is one action"), F.Actions.Num(), 1))
    TestEqual(TEXT("middle click uses cycle intent"), F.Actions[0], LingTuSim::UI::ERuntimeUIAction::CycleCamera);
  TestFalse(TEXT("left button remains available"), F.Processor->HandleMouseButtonDownEvent(App, Mouse(EKeys::LeftMouseButton)));
  for (const auto& Input : F.Robot)
    TestTrue(TEXT("mouse camera cannot arm or move robot"), !Input.bDeadman && Input.Forward == 0 && Input.Left == 0 && Input.YawLeft == 0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuMouseCameraContextTest,
    "LingTuSim.UI.Runtime.MouseCamera.ContextGuards", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FLingTuMouseCameraContextTest::RunTest(const FString&) {
  FMouseCameraFixture F;
  auto& App = FSlateApplication::Get();
  const auto Right = Mouse(EKeys::RightMouseButton, true);
  const auto Drag = Mouse(EKeys::Invalid, true, {10, 5});
  const auto Wheel = Mouse(EKeys::Invalid, false, FVector2D::ZeroVector, 1);
  F.Processor->HandleMouseButtonDownEvent(App, Right);
  F.Processor->HandleKeyDownEvent(App, FKeyEvent(EKeys::Escape, FModifierKeysState(), 0, false, 0, 0));
  TestFalse(TEXT("menu drag is not gameplay"), F.Processor->HandleMouseMoveEvent(App, Drag));
  TestFalse(TEXT("menu scroll stays in UI"), F.Processor->HandleMouseWheelOrGestureEvent(App, Wheel, nullptr));
  TestFalse(TEXT("menu middle click cannot change camera"), F.Processor->HandleMouseButtonDownEvent(App, Mouse(EKeys::MiddleMouseButton)));
  F.Processor->HandleKeyDownEvent(App, FKeyEvent(EKeys::Escape, FModifierKeysState(), 0, false, 0, 0));
  TestFalse(TEXT("returning to Drive requires fresh right press"), F.Processor->HandleMouseMoveEvent(App, Drag));
  F.Processor->HandleMouseButtonDownEvent(App, Right);
  App.OnApplicationActivationChanged(false);
  TestFalse(TEXT("focus loss ends look"), F.Processor->HandleMouseMoveEvent(App, Drag));
  App.OnApplicationActivationChanged(true);
  TestFalse(TEXT("focus restore does not rearm held mouse"), F.Processor->HandleMouseMoveEvent(App, Drag));
  F.bTextEntry = true;
  TestFalse(TEXT("text entry owns scroll"), F.Processor->HandleMouseWheelOrGestureEvent(App, Wheel, nullptr));
  F.bTextEntry = false;
  F.bSelector = true;
  TestFalse(TEXT("front end owns mouse buttons"), F.Processor->HandleMouseButtonDownEvent(App, Right));
  TestFalse(TEXT("front end owns scroll"), F.Processor->HandleMouseWheelOrGestureEvent(App, Wheel, nullptr));
  F.bSelector = false;
  F.Processor->HandleMouseButtonDownEvent(App, Right);
  F.Capture(F.OtherPanel);
  TestTrue(TEXT("viewport keyboard focus remains while another panel captures pointer"), F.Viewport->HasKeyboardFocus());
  TestFalse(TEXT("another panel owns right click"), F.Processor->HandleMouseButtonDownEvent(App, Right));
  TestFalse(TEXT("another panel owns middle click"), F.Processor->HandleMouseButtonDownEvent(App, Mouse(EKeys::MiddleMouseButton)));
  TestFalse(TEXT("another panel owns wheel"), F.Processor->HandleMouseWheelOrGestureEvent(App, Wheel, nullptr));
  F.Capture(F.Viewport);
  TestFalse(TEXT("returning capture does not resurrect drag"), F.Processor->HandleMouseMoveEvent(App, Drag));
  F.Processor->HandleMouseButtonDownEvent(App, Right);
  F.Capture(F.OtherPanel);
  TestFalse(TEXT("another panel owns right release after capture transfer"),
            F.Processor->HandleMouseButtonUpEvent(App, Mouse(EKeys::RightMouseButton)));
  F.Capture(F.Viewport);
  TestFalse(TEXT("capture-transfer release cannot resurrect look"), F.Processor->HandleMouseMoveEvent(App, Drag));
  F.Processor->HandleMouseButtonDownEvent(App, Right);
  TestFalse(TEXT("missing held-button state also ends look"), F.Processor->HandleMouseMoveEvent(App, Mouse(EKeys::Invalid, false, {10, 5})));
  TestFalse(TEXT("held state cannot resurrect drag"), F.Processor->HandleMouseMoveEvent(App, Drag));
  F.Processor->ReleaseInput();
  TestFalse(TEXT("teardown cannot rearm camera"), F.Processor->HandleMouseButtonDownEvent(App, Right));
  TestEqual(TEXT("blocked contexts dispatched no look"), F.Camera.Num(), 0);
  TestEqual(TEXT("blocked contexts dispatched no camera cycle"), F.Actions.Num(), 0);
  return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuDriveContextHelpTest,
    "LingTuSim.UI.Runtime.MouseCamera.HoldHelp", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FLingTuDriveContextHelpTest::RunTest(const FString&) {
  FMouseCameraFixture F;
  auto& App = FSlateApplication::Get();
  const auto Press = [&F, &App](const FKey& Key, bool bRepeat = false) {
    return F.Processor->HandleKeyDownEvent(App, FKeyEvent(Key, FModifierKeysState(), 0, bRepeat, 0, 0));
  };
  const auto Release = [&F, &App](const FKey& Key) {
    return F.Processor->HandleKeyUpEvent(App, FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0));
  };
  TestTrue(TEXT("hold F1 is handled locally"), Press(EKeys::F1));
  TestTrue(TEXT("holding shows the guide"), F.Mode->IsDriveHelpVisible());
  TestEqual(TEXT("help does not invoke a runtime action"), F.Actions.Num(), 0);
  for (const auto& Input : F.Robot)
    TestTrue(TEXT("help alone cannot arm or move robot"), !Input.bDeadman && Input.Forward == 0 && Input.Left == 0 && Input.YawLeft == 0);
  TestTrue(TEXT("help release is handled"), Release(EKeys::F1));
  TestFalse(TEXT("release hides the guide"), F.Mode->IsDriveHelpVisible());
  Press(EKeys::Gamepad_Special_Left);
  TestTrue(TEXT("gamepad View offers the same guide"), F.Mode->IsDriveHelpVisible());
  Press(EKeys::Escape);
  TestFalse(TEXT("opening menu clears the guide"), F.Mode->IsDriveHelpVisible());
  Press(EKeys::Escape);
  Press(EKeys::F1, true);
  TestFalse(TEXT("returning with a held repeat cannot reopen the guide"), F.Mode->IsDriveHelpVisible());
  Press(EKeys::F1);
  App.OnApplicationActivationChanged(false);
  Press(EKeys::F1, true);
  TestFalse(TEXT("focus loss clears help"), F.Mode->IsDriveHelpVisible());
  App.OnApplicationActivationChanged(true);
  Press(EKeys::F1, true);
  TestFalse(TEXT("focus regain requires a fresh press"), F.Mode->IsDriveHelpVisible());
  F.bSelector = true;
  Press(EKeys::F1);
  TestFalse(TEXT("selector does not acquire driving help"), F.Mode->IsDriveHelpVisible());
  F.bSelector = false;
  F.bTextEntry = true;
  Press(EKeys::F1);
  TestFalse(TEXT("text entry keeps ownership"), F.Mode->IsDriveHelpVisible());
  F.bTextEntry = false;
  Press(EKeys::F1);
  F.Processor->ReleaseInput();
  TestFalse(TEXT("teardown clears help"), F.Mode->IsDriveHelpVisible());
  Press(EKeys::F1);
  TestFalse(TEXT("teardown cannot reopen help"), F.Mode->IsDriveHelpVisible());
  return true;
}
#endif
