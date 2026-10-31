#pragma once

#include "LingTuSimGameSelection.h"
#include "Widgets/SCompoundWidget.h"

class SButton;
class SEditableTextBox;
class SScrollBox;
class SBox;
struct FSlateDynamicImageBrush;

namespace LingTuSim::UI {
class SLingTuSimShowroom;
class SLingTuSimShowroomInput;
class SLingTuSimSensorSelection;

/** Selects only existing compiled choices; it never composes or switches a live scene. */
class SLingTuSimRobotMapPicker final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimRobotMapPicker) : _CanEdit(false), _CanConfirm(false) {}
  SLATE_ARGUMENT(TSharedPtr<FGameSelectionModel>, Model)
  SLATE_ATTRIBUTE(bool, CanEdit)
  SLATE_ATTRIBUTE(bool, CanConfirm)
  SLATE_ATTRIBUTE(FText, Status)
  SLATE_EVENT(FSimpleDelegate, OnSelectionChanged)
  SLATE_EVENT(FOnClicked, OnConfirm)
  SLATE_EVENT(FOnClicked, OnBack)
  SLATE_EVENT(FOnClicked, OnBrowseAssets)
  SLATE_END_ARGS()
  void Construct(const FArguments& Args);
  void FocusInitialControl();
  virtual bool SupportsKeyboardFocus() const override { return true; }
  enum class EStep : uint8 { Robot, World };
  EStep GetStep() const { return Step; }
  bool GoBack();
  FReply PrimaryAction();
  bool IsPreviewExpanded() const { return bPreviewExpanded; }
  bool IsSearchEditing() const;
  bool IsShowroomReady() const;
  const FString& GetRobotCategory() const { return RobotCategory; }
  int32 GetVisibleRobotCount() const;
  void SetRobotCategory(const FString& Category);
  bool OpenSensorSelection();
  bool IsSensorSelectionOpen() const { return bSensorSelectionOpen; }
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  void PreviewShowroomInspection();
#endif

 private:
  TSharedRef<SWidget> StepRail();
  TSharedRef<SWidget> CategoryRail();
  TSharedRef<SWidget> PreviewPanel();
  TSharedRef<SWidget> Choice(const FGameSelectionPackageSummary& Package, bool bRobot);
  TSharedRef<SWidget> ConfigurationChoice(const FGameSelectionOption& Option);
  void RefreshSelection();
  void ChangeStep(EStep NewStep);
  bool CanAdvance() const;
  void LoadPreviews();
  bool MatchesRobot(const FGameSelectionPackageSummary& Package) const;
  void ApplyRobotFilter();
  const FSlateBrush* RobotBrush(const FString& Id) const;
  const FSlateBrush* PreviewBrush() const;
  FText SelectedPackageLabel(bool bRobot) const;
  FText StepTitle() const;
  FText PairSummary() const;
  FText PairAvailability() const;
  FText WorldCompatibility(const FString& WorldId) const;
  TSharedPtr<FGameSelectionModel> Model;
  TAttribute<bool> CanEdit;
  TAttribute<bool> CanConfirm;
  TAttribute<FText> Status;
  FSimpleDelegate OnSelectionChanged;
  FOnClicked OnConfirm;
  FOnClicked OnBack;
  EStep Step = EStep::Robot;
  bool bPreviewExpanded = false;
  bool bSensorSelectionOpen = false;
  void CloseSensorSelection();
  TSharedPtr<SButton> SensorsButton;
  TSharedPtr<SBox> SensorSelectionHost;
  TSharedPtr<SLingTuSimSensorSelection> SensorSelectionPanel;
  FString RobotCategory = TEXT("all");
  FString RobotSearch;
  TArray<FGameSelectionPackageSummary> RobotChoices;
  TMap<FString, TSharedPtr<FSlateDynamicImageBrush>> Previews;
  TMap<FString, TSharedPtr<SButton>> RobotButtons;
  TMap<FString, TSharedPtr<SButton>> WorldButtons;
  TSharedPtr<SButton> PrimaryButton;
  TSharedPtr<SButton> PreviewButton;
  TSharedPtr<SButton> WorldPreviewButton;
  TSharedPtr<SButton> ClosePreviewButton;
  TSharedPtr<SButton> BackButton;
  TSharedPtr<SEditableTextBox> SearchBox;
  TSharedPtr<SScrollBox> RobotScroll;
  TSharedPtr<SLingTuSimShowroom> Showroom;
  TSharedPtr<SLingTuSimShowroomInput> ShowroomInput;
};
}
