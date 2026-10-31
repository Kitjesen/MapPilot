#pragma once

#include "LingTuSimSensorSelection.h"
#include "Types/SlateEnums.h"
#include "Widgets/SCompoundWidget.h"

class SButton;
class SCheckBox;
class SEditableTextBox;
template <typename OptionType> class SComboBox;
struct FSlateDynamicImageBrush;

namespace LingTuSim::UI {

/** Edits next-start configuration. The parent owns persistence, launch and focus restoration. */
class SLingTuSimSensorSelection final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimSensorSelection) {}
  SLATE_ARGUMENT(TSharedPtr<FSensorSelectionModel>, Model)
  SLATE_EVENT(FSimpleDelegate, OnChanged)
  SLATE_EVENT(FSimpleDelegate, OnClose)
  SLATE_END_ARGS()

  void Construct(const FArguments& Args);
  void FocusInitialControl();
  bool IsTextEntryEditing() const;
  virtual bool SupportsKeyboardFocus() const override { return true; }

 private:
  TSharedRef<SWidget> SensorRow(const FSensorChoice& Sensor);
  TSharedRef<SWidget> TypeIllustration(const FSensorChoice& Sensor);
  TSharedRef<SWidget> CameraControls(const FSensorChoice& Sensor);
  TSharedRef<SWidget> AngleEntry(const FSensorChoice& Sensor, bool bPan);
  void CommitRate(const FString& Id, const FText& Text, ETextCommit::Type CommitType);
  void CommitAngle(const FString& Id, bool bPan, const FText& Text, ETextCommit::Type CommitType);
  TSharedPtr<FSensorSelectionModel> Model;
  FSimpleDelegate OnChanged;
  FSimpleDelegate OnClose;
  TMap<FString, TSharedPtr<SCheckBox>> EnableBoxes;
  TMap<FString, TSharedPtr<SEditableTextBox>> RateBoxes;
  TMap<FString, TSharedPtr<SEditableTextBox>> PanBoxes;
  TMap<FString, TSharedPtr<SEditableTextBox>> TiltBoxes;
  TArray<TSharedPtr<FString>> CameraPresets;
  TMap<FString, TSharedPtr<SComboBox<TSharedPtr<FString>>>> ProjectionBoxes;
  // Brushes outlive the SImage raw brush pointers; no asynchronous image loading.
  TMap<FString, TSharedPtr<FSlateDynamicImageBrush>> Illustrations;
  TSharedPtr<SButton> CloseButton;
};
}  // namespace LingTuSim::UI
