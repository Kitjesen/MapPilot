#if WITH_DEV_AUTOMATION_TESTS

#include "SLingTuSimShowroomInput.h"

#include "Framework/Application/SlateApplication.h"
#include "Input/Events.h"
#include "Misc/AutomationTest.h"

namespace {
using LingTuSim::UI::EShowroomGesture;
using LingTuSim::UI::SLingTuSimShowroomInput;

struct FGestureRecord {
  EShowroomGesture Gesture;
  FVector2D Delta;
};

struct FShowroomInputFixture {
  bool bCanInteract = true;
  TArray<FGestureRecord> Gestures;
  TSharedRef<SLingTuSimShowroomInput> Widget;

  FShowroomInputFixture()
      : Widget(SNew(SLingTuSimShowroomInput)
                   .CanInteract_Lambda([this] { return bCanInteract; })
                   .OnGesture([this](EShowroomGesture Gesture, FVector2D Delta) {
                     Gestures.Add({Gesture, Delta});
                   })) {}
};

FGeometry Geometry(FVector2D Size = FVector2D(400, 200)) {
  return FGeometry::MakeRoot(Size, FSlateLayoutTransform());
}

FPointerEvent Mouse(const FKey& EffectingButton, const TSet<FKey>& Held = {},
                    FVector2D Position = FVector2D(200, 100),
                    FVector2D LastPosition = FVector2D(200, 100),
                    float Wheel = 0, bool bShift = false) {
  const FModifierKeysState Modifiers(bShift, false, false, false, false, false,
                                     false, false, false);
  return FPointerEvent(FSlateApplication::CursorPointerIndex, Position, LastPosition,
                       Held, EffectingButton, Wheel, Modifiers);
}

FKeyEvent Key(const FKey& InKey) {
  return FKeyEvent(InKey, FModifierKeysState(), 0, false, 0, 0);
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomMouseGestureTest,
    "LingTuSim.UI.Runtime.Showroom.Input.MouseGestures",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomMouseGestureTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  const FGeometry Geo = Geometry();
  const auto Drag = [&](const FKey& Button, bool bShift = false) {
    F.Widget->OnMouseButtonDown(Geo, Mouse(Button, {Button}, {}, {}, 0, bShift));
    F.Widget->OnMouseMove(Geo, Mouse(EKeys::Invalid, {Button}, {240, 80}, {200, 100}, 0, bShift));
    F.Widget->OnMouseButtonUp(Geo, Mouse(Button));
  };
  Drag(EKeys::LeftMouseButton);
  Drag(EKeys::RightMouseButton);
  Drag(EKeys::MiddleMouseButton);
  Drag(EKeys::LeftMouseButton, true);
  if (!TestEqual(TEXT("four drags dispatch four gestures"), F.Gestures.Num(), 4)) return false;
  TestEqual(TEXT("left drag orbits"), F.Gestures[0].Gesture, EShowroomGesture::Orbit);
  TestEqual(TEXT("right drag orbits"), F.Gestures[1].Gesture, EShowroomGesture::Orbit);
  TestEqual(TEXT("middle drag pans"), F.Gestures[2].Gesture, EShowroomGesture::Pan);
  TestEqual(TEXT("shift-left drag pans"), F.Gestures[3].Gesture, EShowroomGesture::Pan);
  TestEqual(TEXT("pixel delta is normalized by geometry size"), F.Gestures[0].Delta,
            FVector2D(0.1, -0.1));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomMouseReleaseTest,
    "LingTuSim.UI.Runtime.Showroom.Input.MouseRelease",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomMouseReleaseTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  const FGeometry Geo = Geometry();
  F.Widget->OnMouseButtonDown(Geo, Mouse(EKeys::LeftMouseButton, {EKeys::LeftMouseButton}));
  TestFalse(TEXT("wrong button release is unhandled"),
            F.Widget->OnMouseButtonUp(Geo, Mouse(EKeys::RightMouseButton)).IsEventHandled());
  TestTrue(TEXT("wrong button release preserves drag"), F.Widget->IsDragging());
  TestTrue(TEXT("matching release outside geometry is handled"),
           F.Widget->OnMouseButtonUp(Geo, Mouse(EKeys::LeftMouseButton, {}, {900, 900})).IsEventHandled());
  TestFalse(TEXT("matching release outside geometry ends drag"), F.Widget->IsDragging());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomMouseCancellationTest,
    "LingTuSim.UI.Runtime.Showroom.Input.Cancellation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomMouseCancellationTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  const FGeometry Geo = Geometry();
  F.Widget->OnMouseButtonDown(Geo, Mouse(EKeys::LeftMouseButton, {EKeys::LeftMouseButton}));
  F.Widget->OnMouseCaptureLost(FCaptureLostEvent());
  TestFalse(TEXT("capture loss cancels drag"), F.Widget->IsDragging());
  F.Widget->OnMouseButtonDown(Geo, Mouse(EKeys::RightMouseButton, {EKeys::RightMouseButton}));
  F.Widget->OnFocusLost(FFocusEvent());
  TestFalse(TEXT("focus loss cancels drag"), F.Widget->IsDragging());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomWheelGuardTest,
    "LingTuSim.UI.Runtime.Showroom.Input.WheelGuard",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomWheelGuardTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  F.bCanInteract = false;
  TestFalse(TEXT("wheel is unhandled while interaction is disabled"),
            F.Widget->OnMouseWheel(Geometry(), Mouse(EKeys::Invalid, {}, {}, {}, 0.5f)).IsEventHandled());
  TestEqual(TEXT("disabled wheel dispatches nothing"), F.Gestures.Num(), 0);
  F.bCanInteract = true;
  TestTrue(TEXT("wheel is handled while interaction is enabled"),
           F.Widget->OnMouseWheel(Geometry(), Mouse(EKeys::Invalid, {}, {}, {}, 0.5f)).IsEventHandled());
  if (!TestEqual(TEXT("enabled wheel dispatches once"), F.Gestures.Num(), 1)) return false;
  TestEqual(TEXT("wheel dispatches zoom"), F.Gestures[0].Gesture, EShowroomGesture::Zoom);
  TestEqual(TEXT("fractional wheel delta is preserved"), F.Gestures[0].Delta, FVector2D(0.5, 0));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomDoubleClickTest,
    "LingTuSim.UI.Runtime.Showroom.Input.DoubleClickReset",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomDoubleClickTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  F.Widget->OnMouseButtonDown(Geometry(), Mouse(EKeys::LeftMouseButton, {EKeys::LeftMouseButton}));
  TestTrue(TEXT("left double click is handled"),
           F.Widget->OnMouseButtonDoubleClick(Geometry(), Mouse(EKeys::LeftMouseButton)).IsEventHandled());
  TestFalse(TEXT("double click ends an active drag"), F.Widget->IsDragging());
  if (!TestEqual(TEXT("double click dispatches once"), F.Gestures.Num(), 1)) return false;
  TestEqual(TEXT("double click dispatches reset"), F.Gestures[0].Gesture, EShowroomGesture::Reset);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomKeyboardViewTest,
    "LingTuSim.UI.Runtime.Showroom.Input.KeyboardView",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomKeyboardViewTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  const FKey Keys[] = {EKeys::Home, EKeys::Add, EKeys::Left, EKeys::Up};
  for (const FKey& ViewKey : Keys)
    TestTrue(TEXT("view key is handled"), F.Widget->OnKeyDown(Geometry(), Key(ViewKey)).IsEventHandled());
  if (!TestEqual(TEXT("four view keys dispatch four gestures"), F.Gestures.Num(), 4)) return false;
  TestEqual(TEXT("Home resets"), F.Gestures[0].Gesture, EShowroomGesture::Reset);
  TestEqual(TEXT("plus zooms in"), F.Gestures[1].Delta, FVector2D(1, 0));
  TestEqual(TEXT("left arrow orbits left"), F.Gestures[2].Delta, FVector2D(-0.025, 0));
  TestEqual(TEXT("up arrow orbits up"), F.Gestures[3].Delta, FVector2D(0, -0.025));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FShowroomGameplayKeyTest,
    "LingTuSim.UI.Runtime.Showroom.Input.GameplayKeyUnhandled",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomGameplayKeyTest::RunTest(const FString&) {
  FShowroomInputFixture F;
  TestFalse(TEXT("gameplay W remains unhandled"),
            F.Widget->OnKeyDown(Geometry(), Key(EKeys::W)).IsEventHandled());
  TestEqual(TEXT("gameplay W dispatches no showroom gesture"), F.Gestures.Num(), 0);
  return true;
}

#endif
