#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimShowroomView.h"

#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimShowroomViewResetTest, "LingTuSim.UI.Showroom.View.Reset",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimShowroomViewResetTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FShowroomViewState View;
  TestEqual(TEXT("default yaw is the perspective yaw"), View.YawDegrees, -25.0);
  TestEqual(TEXT("default pitch is the perspective pitch"), View.PitchDegrees, 12.0);
  TestEqual(TEXT("default zoom is neutral"), View.Zoom, 1.0);
  TestEqual(TEXT("default pan is centered"), View.Pan, FVector2D::ZeroVector);

  View.SetPreset(LingTuSim::UI::EShowroomView::Top);
  View.Orbit(FVector2D(0.25, -0.25));
  View.Dolly(2.0);
  View.Translate(FVector2D(0.5, -0.5));
  View.Reset();

  TestEqual(TEXT("reset restores yaw for a model change"), View.YawDegrees, -25.0);
  TestEqual(TEXT("reset restores pitch for a model change"), View.PitchDegrees, 12.0);
  TestEqual(TEXT("reset restores zoom for a model change"), View.Zoom, 1.0);
  TestEqual(TEXT("reset restores centered pan for a model change"), View.Pan,
            FVector2D::ZeroVector);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimShowroomViewPresetsTest, "LingTuSim.UI.Showroom.View.Presets",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimShowroomViewPresetsTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  struct FPresetExpectation {
    LingTuSim::UI::EShowroomView Preset;
    double Yaw;
    double Pitch;
  };
  const FPresetExpectation Expectations[] = {
      {LingTuSim::UI::EShowroomView::Perspective, -25.0, 12.0},
      {LingTuSim::UI::EShowroomView::Front, 0.0, 12.0},
      {LingTuSim::UI::EShowroomView::Side, 90.0, 12.0},
      {LingTuSim::UI::EShowroomView::Back, 180.0, 12.0},
      {LingTuSim::UI::EShowroomView::Top, 0.0, 70.0},
  };

  for (const FPresetExpectation &Expectation : Expectations) {
    LingTuSim::UI::FShowroomViewState View;
    View.Dolly(2.0);
    View.Translate(FVector2D(0.5, -0.5));
    View.SetPreset(Expectation.Preset);
    TestEqual(TEXT("preset applies yaw"), View.YawDegrees, Expectation.Yaw);
    TestEqual(TEXT("preset applies pitch"), View.PitchDegrees, Expectation.Pitch);
    TestEqual(TEXT("preset resets zoom"), View.Zoom, 1.0);
    TestEqual(TEXT("preset resets pan"), View.Pan, FVector2D::ZeroVector);
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimShowroomViewOrbitTest, "LingTuSim.UI.Showroom.View.Orbit",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimShowroomViewOrbitTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FShowroomViewState View;
  View.Orbit(FVector2D(0.25, 0.25));
  TestEqual(TEXT("normalized horizontal orbit spans 240 degrees"), View.YawDegrees, 35.0);
  TestEqual(TEXT("normalized vertical orbit spans 120 degrees"), View.PitchDegrees, 42.0);

  View.Orbit(FVector2D(1.0, 10.0));
  TestEqual(TEXT("yaw wraps into the signed 180 degree range"), View.YawDegrees, -85.0);
  TestEqual(TEXT("pitch clamps at the top view"), View.PitchDegrees, 70.0);
  View.Orbit(FVector2D(-10.0, -10.0));
  TestTrue(TEXT("extreme yaw remains in the signed 180 degree range"),
           View.YawDegrees >= -180.0 && View.YawDegrees <= 180.0);
  TestEqual(TEXT("pitch clamps above the horizon"), View.PitchDegrees, 5.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimShowroomViewZoomAndPanTest, "LingTuSim.UI.Showroom.View.ZoomAndPan",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimShowroomViewZoomAndPanTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FShowroomViewState View;
  View.Dolly(2.0);
  TestEqual(TEXT("wheel delta applies exponential zoom"), View.Zoom,
            FMath::Pow(1.12, 2.0));
  View.Dolly(100.0);
  TestEqual(TEXT("zoom clamps at its upper bound"), View.Zoom, 1.8);
  View.Dolly(-100.0);
  TestEqual(TEXT("zoom clamps at its lower bound"), View.Zoom, 0.65);

  View.Translate(FVector2D(0.5, -0.25));
  TestEqual(TEXT("normalized horizontal translation scales pan"), View.Pan.X, 0.075);
  TestEqual(TEXT("normalized vertical translation scales pan"), View.Pan.Y, -0.0375);
  View.Translate(FVector2D(100.0, -100.0));
  TestEqual(TEXT("horizontal pan clamps at its upper bound"), View.Pan.X, 0.10);
  TestEqual(TEXT("vertical pan clamps at its lower bound"), View.Pan.Y, -0.10);
  return true;
}

#endif
