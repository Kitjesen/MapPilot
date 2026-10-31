#if WITH_DEV_AUTOMATION_TESTS
#include "SLingTuSimMeasurements.h"
#include "Misc/AutomationTest.h"

// Numeric projection fixtures only: these tests make no sensor/physics claim.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMeasurementImageDisplayTest,
  "LingTuSim.UI.Runtime.Measurements.ImageDisplay",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMeasurementImageDisplayTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FMeasurementPreview P;
  P.Kind = TEXT("thermal"); P.Width = 2; P.Height = 1;
  P.Primary = {TOptional<double>(273.15), TOptional<double>(310.0)};
  P.ValidMask = {true, false};
  TestTrue(TEXT("Kelvin is retained"), MeasurementDisplay::Temperature(P, 0, false).GetValue() == 273.15);
  TestTrue(TEXT("Celsius is a view conversion"), FMath::IsNearlyZero(MeasurementDisplay::Temperature(P, 0, true).GetValue()));
  TestFalse(TEXT("invalid mask wins over a numeric payload"), MeasurementDisplay::Temperature(P, 1, false).IsSet());
  const FColor Invalid = MeasurementDisplay::PixelColor(P, 1, 273.15, 310.0);
  TestTrue(TEXT("invalid pixels are explicit checkerboard, not black"), Invalid == FColor(175, 175, 175));
  TestTrue(TEXT("constant thermal range is visible"), MeasurementDisplay::PixelColor(P, 0, 273.15, 273.15) != FColor::Black);
  P.Primary[0].Reset();
  TestFalse(TEXT("nullable primary is invalid even with mask set"), MeasurementDisplay::IsValidPixel(P, 0));
  TestFalse(TEXT("out-of-range inspection is unknown"), MeasurementDisplay::IsValidPixel(P, 2));

  P.Kind = TEXT("semantic"); P.Primary[0] = 0.0; P.Secondary = {TOptional<double>(42.0), TOptional<double>()};
  TestTrue(TEXT("class zero with an instance is real ground-truth data"), MeasurementDisplay::IsValidPixel(P, 0));
  TestFalse(TEXT("semantic IDs are not temperatures"), MeasurementDisplay::Temperature(P, 0, false).IsSet());
  P.Secondary[0].Reset();
  TestFalse(TEXT("missing instance cannot be rendered as a valid class"), MeasurementDisplay::IsValidPixel(P, 0));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMeasurementPressureDisplayTest,
  "LingTuSim.UI.Runtime.Measurements.PressureDisplay",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMeasurementPressureDisplayTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FMeasurementPreview P;
  P.Kind = TEXT("acoustic"); P.Channels = 2; P.SampleRateHz = 1000; P.SourceSampleCount = 20;
  P.BinStarts = {0, 10}; P.BinEnds = {10, 20};
  P.MinPa = {TOptional<double>(0.0), TOptional<double>(), TOptional<double>(-2.0), TOptional<double>(3.0)};
  P.MaxPa = {TOptional<double>(0.0), TOptional<double>(), TOptional<double>(4.0), TOptional<double>(3.0)};
  P.ValidFractions = {1, 0, .5, 1};
  const auto Constant = MeasurementDisplay::PressureBin(P, 1, 1);
  TestTrue(TEXT("second channel uses channel-major indexing"), Constant.IsSet());
  if (Constant.IsSet()) {
    TestEqual(TEXT("constant pressure is retained for point rendering"), Constant->MinPa, 3.0);
    TestEqual(TEXT("constant upper pressure is retained"), Constant->MaxPa, 3.0);
    TestEqual(TEXT("bin start is block-relative sample time"), Constant->StartSeconds, .01);
    TestEqual(TEXT("bin end is exclusive block-relative sample time"), Constant->EndSeconds, .02);
  }
  const auto Partial = MeasurementDisplay::PressureBin(P, 1, 0);
  TestTrue(TEXT("partial validity remains distinguishable"), Partial.IsSet() && Partial->ValidFraction == .5);
  const auto Zero = MeasurementDisplay::PressureBin(P, 0, 0);
  TestTrue(TEXT("valid 0 Pa is not an invalid gap"), Zero.IsSet() && Zero->MinPa == 0 && Zero->MaxPa == 0);
  TestFalse(TEXT("empty bin is not a zero-pressure line"), MeasurementDisplay::PressureBin(P, 0, 1).IsSet());
  TestFalse(TEXT("channel outside source is unknown"), MeasurementDisplay::PressureBin(P, 2, 0).IsSet());
  P.ValidFractions[3] = 0;
  TestFalse(TEXT("zero validity invalidates finite bounds"), MeasurementDisplay::PressureBin(P, 1, 1).IsSet());
  P.SampleRateHz = 0;
  TestFalse(TEXT("unknown sample clock is not a fabricated time"), MeasurementDisplay::PressureBin(P, 0, 0).IsSet());
  return true;
}
#endif
