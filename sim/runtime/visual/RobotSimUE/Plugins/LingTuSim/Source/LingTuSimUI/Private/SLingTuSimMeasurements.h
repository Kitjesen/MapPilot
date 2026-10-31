#pragma once

#include "CoreMinimal.h"
#include "LingTuSimMeasurementProjection.h"
#include "Widgets/SCompoundWidget.h"

class FJsonObject;
class FDeferredCleanupSlateBrush;
class SButton;
class SComboButton;
class SVerticalBox;

namespace LingTuSim::UI {
class FMeasurementClient;

// Display math only: these helpers neither synthesize measurements nor control physics.
namespace MeasurementDisplay {
struct FPressureBin {
  double MinPa = 0;
  double MaxPa = 0;
  double StartSeconds = 0;
  double EndSeconds = 0;
  double ValidFraction = 0;
};
TOptional<double> Temperature(const FMeasurementPreview& Preview, int32 Index, bool bCelsius);
bool IsValidPixel(const FMeasurementPreview& Preview, int32 Index);
FColor PixelColor(const FMeasurementPreview& Preview, int32 Index, double LowK, double HighK);
TOptional<FPressureBin> PressureBin(const FMeasurementPreview& Preview, int32 Channel, int32 Bin);
}

/** Menu-only projection. Parent owns visibility, source binding, HTTP and input mode. */
class SLingTuSimMeasurements final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimMeasurements) {}
    SLATE_ARGUMENT(TSharedPtr<FMeasurementClient>, Client)
    SLATE_EVENT(FSimpleDelegate, OnBack)
  SLATE_END_ARGS()

  void Construct(const FArguments& Args);
  void FocusInitialControl();
  virtual bool SupportsKeyboardFocus() const override { return true; }
  virtual FReply OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Event) override;
  virtual void Tick(const FGeometry& Geometry, double Time, float DeltaTime) override;

 private:
  TSharedRef<SWidget> BuildSensorMenu();
  void RefreshProjection();
  void RebuildDisplay(const FMeasurementPreview& Preview);
  void RebuildControls(const TSharedPtr<FJsonObject>& Settings);
  void Send(const FString& Context, const FString& Operation, TSharedPtr<FJsonObject> Params);
  FString BindingKey() const;
  FText StatusText() const;
  FText InspectText() const;
  bool CanSend() const;

  TSharedPtr<FMeasurementClient> Client;
  FSimpleDelegate OnBack;
  TSharedPtr<SButton> BackButton;
  TSharedPtr<SComboButton> SensorButton;
  TSharedPtr<SVerticalBox> Display;
  TSharedPtr<SVerticalBox> Controls;
  TArray<TSharedPtr<FDeferredCleanupSlateBrush>> ImageBrushes;
  TOptional<FMeasurementPreview> CachedPreview;
  FString CachedControlKey;
  FString CachedSettings;
  FString LocalFeedback;
  bool bCelsius = true;
  int32 Row = 0;
  int32 Column = 0;
  int32 Channel = 0;
  int32 Bin = 0;
};
}
